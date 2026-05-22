"""run_best_params.py — Batch runner that replays every bag with its best CFAR parameters.

Reads the JSON produced by validate_tracking.py (--output-json) and calls
process_session() for each bag, producing all standard analysis outputs.

Usage:
    cd uwb_processing
    python run_best_params.py --params-json best_cfar_params.json

    # Also render animations:
    python run_best_params.py --params-json best_cfar_params.json --animate --animate-3d

    # Run even bags that failed validation:
    python run_best_params.py --params-json best_cfar_params.json --run-all

    # Restrict to specific bags:
    python run_best_params.py --params-json best_cfar_params.json --bags two_people_hallway walking
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT / "uwb_processing"))

from uwb_processing.run_session import process_session
from uwb_processing.types import CfarDetectionConfig, DetectionConfig


def _build_cfar_config(entry: dict) -> CfarDetectionConfig:
    return CfarDetectionConfig(
        num_ref_cells=8,
        num_guard_cells=int(entry.get("num_guard_cells", 2)),
        pfa=float(entry["pfa"]),
        variant=str(entry.get("variant", "CA")),
        max_peaks_per_frame=6,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run all bags with the best CFAR params from validate_tracking.py."
    )
    parser.add_argument(
        "--params-json",
        default="best_cfar_params.json",
        help="Path to the JSON file produced by validate_tracking.py --output-json.",
    )
    parser.add_argument(
        "--rosbags-dir",
        default=None,
        help="Path to uwb_rosbags/ directory (auto-detected if omitted).",
    )
    parser.add_argument(
        "--output-root",
        default=None,
        help="Root directory for per-bag output sub-folders (default: uwb_outputs/).",
    )
    parser.add_argument(
        "--run-all",
        action="store_true",
        help="Process bags even if they failed validation (passed=False).",
    )
    parser.add_argument(
        "--bags",
        nargs="*",
        default=None,
        help="Restrict to specific bag names (space-separated).",
    )
    parser.add_argument("--animate",     action="store_true", help="Render scrolling animation.")
    parser.add_argument("--animate-3d",  action="store_true", help="Render 3D CIR surface plot.")
    parser.add_argument("--animate-aoa", action="store_true", help="Render AoA animation.")
    parser.add_argument("--use-moviepy", action="store_true", help="Use moviepy instead of FFMpeg.")
    parser.add_argument("--topic",       default="/uwb/frame_raw", help="ROS topic name.")
    args = parser.parse_args()

    params_path = Path(args.params_json)
    if not params_path.exists():
        sys.exit(
            f"Params file not found: {params_path}\n"
            "Run validate_tracking.py --output-json first."
        )

    params: dict[str, dict] = json.loads(params_path.read_text())

    rosbags_root = (
        Path(args.rosbags_dir) if args.rosbags_dir else _ROOT / "uwb_rosbags"
    )
    output_root = (
        Path(args.output_root) if args.output_root else _ROOT / "uwb_outputs"
    )

    bag_names = list(params.keys())
    if args.bags:
        bag_names = [b for b in bag_names if b in args.bags]

    det_cfg = DetectionConfig()

    passed_total = 0
    skipped_total = 0
    failed_total = 0
    run_total = 0

    print(f"\nRunning {len(bag_names)} bag(s) with best CFAR parameters.\n")

    for bag_name in bag_names:
        entry = params[bag_name]
        bag_path = rosbags_root / bag_name

        if not bag_path.exists():
            print(f"[SKIP] {bag_name} — directory not found")
            skipped_total += 1
            continue

        mcap_files = list(bag_path.glob("*.mcap")) + list(bag_path.glob("*.npz"))
        if not mcap_files:
            print(f"[SKIP] {bag_name} — no .mcap or .npz files")
            skipped_total += 1
            continue

        passed = entry.get("passed", True)
        expected = entry.get("expected")
        tracks_found = entry.get("tracks_found", "?")

        if not passed and not args.run_all:
            status = f"FAIL (expected={expected}, got={tracks_found})"
            print(f"[SKIP] {bag_name} — {status}  (use --run-all to force)")
            skipped_total += 1
            continue

        cfar_cfg = _build_cfar_config(entry)
        use_multipath = bool(entry.get("multipath", False))
        out_dir = output_root / bag_name

        pfa_label   = f"pfa={entry['pfa']:.0e}"
        var_label   = entry.get("variant", "CA")
        guard_label = f"guard={entry.get('num_guard_cells', 2)}"
        mp_label    = "multi-path" if use_multipath else "single-path"
        print(
            f"\n[{bag_name}]  {pfa_label} {var_label} {guard_label} {mp_label}"
            f"  expected={expected}  prev_found={tracks_found}"
        )

        t0 = time.time()
        try:
            summary, out_path = process_session(
                input_path=bag_path,
                output_dir=out_dir,
                config=det_cfg,
                cfar_config=cfar_cfg,
                topic=args.topic,
                animate=args.animate,
                animate_aoa=args.animate_aoa,
                animate_3d=args.animate_3d,
                use_moviepy=args.use_moviepy,
                use_multipath=use_multipath,
            )
            elapsed = time.time() - t0
            n_tracks = summary.get("confirmed_tracks", "?")
            print(f"  done in {elapsed:.1f}s — {n_tracks} confirmed track(s) — outputs → {out_path}")
            run_total += 1
            if passed:
                passed_total += 1
            else:
                failed_total += 1
        except Exception as exc:
            elapsed = time.time() - t0
            print(f"  ERROR after {elapsed:.1f}s: {exc}")
            failed_total += 1

    print(f"\n{'='*60}")
    print(f"Done.  ran={run_total}  skipped={skipped_total}  errors={failed_total}")
    print(f"Outputs written under: {output_root}")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
