"""validate_tracking.py — Self-tuning CFAR parameter search across all bags.

Runs every bag against its expected person count and searches for CFAR parameters
that produce exactly that count.  Continues retrying parameter combinations until
every bag passes or the search budget is exhausted.

Usage:
    cd uwb_processing
    python validate_tracking.py

    # Skip slow bags, save best params:
    python validate_tracking.py --output-json best_cfar_params.json

    # Loop until everything passes (with time limit):
    python validate_tracking.py --max-rounds 10
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import NamedTuple

import numpy as np

# Allow running from uwb_processing/ subdirectory
_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT / "uwb_processing"))

from uwb_processing.cfar import combine_paths_max, run_cfar_session
from uwb_processing.loaders import load_session
from uwb_processing.preprocessing import preprocess_session
from uwb_processing.tracker import MultiTargetTracker
from uwb_processing.types import (
    CfarDetectionConfig,
    DetectionConfig,
    TrackingConfig,
)

# ---------------------------------------------------------------------------
# Ground truth: expected number of confirmed tracks per bag.
# None = skip (unknown or variable).
# ---------------------------------------------------------------------------
EXPECTED: dict[str, int | None] = {
    "one_person_simple":               1,
    "single_person_hallway":           1,
    "person_occluded_room_221":        1,
    "angle_person_occluded_room_221":  1,
    "single_person_corner_reflector":  1,
    "outside_allen_one_person":        1,
    "two_people_hallway":              2,
    "two_people_opposite_sides":       2,
    "two_people_test_peak":            2,
    "two_person_walking_range_test":   2,
    "one_corner_reflector_two_people": 2,
    "full_group_test":                 None,   # variable group size
    "clutter_second_floor_allen":      None,   # environment reference
    "walking":                         1,
}

# ---------------------------------------------------------------------------
# Parameter grid to search.  Ordered from most conservative to most permissive.
# For each bag we stop at the first combination that gives the right count.
# ---------------------------------------------------------------------------
PFA_GRID     = [1e-4, 5e-4, 1e-3, 5e-3, 1e-2, 3e-2, 6e-2]
VARIANT_GRID = ["CA", "OS"]          # OS is more robust near strong reflectors
GUARD_GRID   = [2, 4]                # wider guard helps around corner reflectors
MULTIPATH    = [False, True]         # False = strongest RX path, True = max across all paths


class SearchResult(NamedTuple):
    bag: str
    expected: int | None
    found: int
    passed: bool
    pfa: float
    variant: str
    guard: int
    multipath: bool
    n_combos_tried: int
    elapsed_s: float


# ---------------------------------------------------------------------------
# Core: run one bag with one CFAR config and return track count
# ---------------------------------------------------------------------------

def count_tracks(
    bag_path: Path,
    det_cfg: DetectionConfig,
    cfar_cfg: CfarDetectionConfig,
    use_multipath: bool,
) -> int:
    session = load_session(bag_path)
    preprocessed = preprocess_session(session, det_cfg)

    clutter_mag = combine_paths_max(preprocessed) if use_multipath else None

    dets_per_frame = run_cfar_session(preprocessed, cfar_cfg, clutter_mag=clutter_mag)

    track_cfg = TrackingConfig(
        dt_s=1.0 / preprocessed.frame_rate_hz,
        sigma_meas_m=det_cfg.tap_spacing_m,
    )
    tracker = MultiTargetTracker(track_cfg)
    for fi, (ts, dets) in enumerate(zip(session.timestamps_s, dets_per_frame)):
        tracker.update(fi, float(ts), dets)

    return len(tracker.tracks_snapshot())


# ---------------------------------------------------------------------------
# Search: iterate over parameter grid for one bag
# ---------------------------------------------------------------------------

def search_bag(
    bag_path: Path,
    expected: int | None,
    det_cfg: DetectionConfig,
    verbose: bool = True,
) -> SearchResult:
    t0 = time.time()
    best_found = -1
    best_params: tuple = (PFA_GRID[0], VARIANT_GRID[0], GUARD_GRID[0], MULTIPATH[0])
    n_tried = 0

    combos = [
        (pfa, variant, guard, mp)
        for mp in MULTIPATH
        for variant in VARIANT_GRID
        for guard in GUARD_GRID
        for pfa in PFA_GRID
    ]

    for pfa, variant, guard, mp in combos:
        cfar_cfg = CfarDetectionConfig(
            num_ref_cells=8,
            num_guard_cells=guard,
            pfa=pfa,
            variant=variant,
            max_peaks_per_frame=6,
        )
        try:
            n_tracks = count_tracks(bag_path, det_cfg, cfar_cfg, mp)
        except Exception as exc:
            if verbose:
                print(f"    ERROR: {exc}")
            continue

        n_tried += 1
        label = f"pfa={pfa:.0e} {variant} guard={guard} {'multi' if mp else 'single'}-path"

        if expected is None:
            # No ground truth — just report best result and stop after first combo
            if verbose:
                print(f"  [skip gt] {label} → {n_tracks} tracks")
            best_found = n_tracks
            best_params = (pfa, variant, guard, mp)
            break

        if verbose:
            status = "✓" if n_tracks == expected else f"✗ (got {n_tracks})"
            print(f"  {status}  {label}")

        if n_tracks == expected:
            best_found = n_tracks
            best_params = (pfa, variant, guard, mp)
            break

        # Keep track of closest result so far
        if abs(n_tracks - expected) < abs(best_found - expected) or best_found == -1:
            best_found = n_tracks
            best_params = (pfa, variant, guard, mp)

    passed = expected is None or best_found == expected
    pfa_b, var_b, guard_b, mp_b = best_params
    elapsed = time.time() - t0
    return SearchResult(
        bag=bag_path.name,
        expected=expected,
        found=best_found,
        passed=passed,
        pfa=pfa_b,
        variant=var_b,
        guard=guard_b,
        multipath=mp_b,
        n_combos_tried=n_tried,
        elapsed_s=elapsed,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Self-tuning CFAR validation across all bags.")
    parser.add_argument("--rosbags-dir", default=None,
                        help="Path to uwb_rosbags/ directory (auto-detected if omitted).")
    parser.add_argument("--output-json", default=None,
                        help="Save best parameters per bag to this JSON file.")
    parser.add_argument("--max-rounds", type=int, default=1,
                        help="How many full passes to run. Use >1 if you want repeated attempts.")
    parser.add_argument("--bags", nargs="*", default=None,
                        help="Restrict to specific bag names (space-separated).")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    rosbags_root = (
        Path(args.rosbags_dir)
        if args.rosbags_dir
        else _ROOT / "uwb_rosbags"
    )
    if not rosbags_root.exists():
        sys.exit(f"uwb_rosbags not found at {rosbags_root}")

    det_cfg = DetectionConfig()
    verbose = not args.quiet

    # Filter to requested bags
    bag_names = list(EXPECTED.keys())
    if args.bags:
        bag_names = [b for b in bag_names if b in args.bags]

    results: dict[str, SearchResult] = {}

    for round_idx in range(args.max_rounds):
        remaining = [b for b in bag_names if b not in results or not results[b].passed]
        if not remaining:
            print("\nAll bags passed. Stopping early.")
            break

        if args.max_rounds > 1:
            print(f"\n{'='*60}")
            print(f"Round {round_idx + 1}/{args.max_rounds}  —  {len(remaining)} bags remaining")
            print(f"{'='*60}")

        for bag_name in remaining:
            bag_path = rosbags_root / bag_name
            if not bag_path.exists():
                print(f"[SKIP] {bag_name} — directory not found")
                continue

            mcap_files = list(bag_path.glob("*.mcap")) + list(bag_path.glob("*.npz"))
            if not mcap_files:
                print(f"[SKIP] {bag_name} — no .mcap or .npz files")
                continue

            expected = EXPECTED.get(bag_name)
            gt_label = f"expected={expected}" if expected is not None else "no gt"
            print(f"\n[{bag_name}]  {gt_label}")

            result = search_bag(bag_path, expected, det_cfg, verbose=verbose)
            results[bag_name] = result

    # -----------------------------------------------------------------------
    # Summary table
    # -----------------------------------------------------------------------
    print(f"\n{'='*72}")
    print(f"{'BAG':<40} {'EXP':>4} {'GOT':>4}  {'STATUS':<8} {'BEST PARAMS'}")
    print(f"{'─'*72}")

    passed_total = 0
    skipped_total = 0
    failed_total = 0

    best_params_out: dict[str, dict] = {}

    for bag_name in bag_names:
        if bag_name not in results:
            print(f"  {'─':40}  (not run)")
            continue
        r = results[bag_name]
        if r.expected is None:
            status = "SKIP"
            skipped_total += 1
        elif r.passed:
            status = "PASS ✓"
            passed_total += 1
        else:
            status = "FAIL ✗"
            failed_total += 1

        mp_label = "multi" if r.multipath else "single"
        params_str = f"pfa={r.pfa:.0e} {r.variant} guard={r.guard} {mp_label}-path"
        exp_str = str(r.expected) if r.expected is not None else "?"
        print(f"  {bag_name:<38} {exp_str:>4} {r.found:>4}  {status:<8} {params_str}")

        best_params_out[bag_name] = {
            "pfa": r.pfa,
            "variant": r.variant,
            "num_guard_cells": r.guard,
            "multipath": r.multipath,
            "tracks_found": r.found,
            "expected": r.expected,
            "passed": r.passed,
        }

    print(f"{'─'*72}")
    total_gt = passed_total + failed_total
    print(f"  PASSED {passed_total}/{total_gt}  FAILED {failed_total}/{total_gt}  SKIPPED {skipped_total}")
    print(f"{'='*72}\n")

    if args.output_json:
        out_path = Path(args.output_json)
        out_path.write_text(json.dumps(best_params_out, indent=2))
        print(f"Best parameters saved to {out_path}")


if __name__ == "__main__":
    main()
