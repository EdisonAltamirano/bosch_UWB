from __future__ import annotations

import csv
import itertools
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT / "uwb_processing"))

from uwb_processing.breathing import run_breathing_extraction
from uwb_processing.cfar import CfarDetectionConfig, merge_dual_channel_peaks, run_cfar_session
from uwb_processing.loaders import load_session
from uwb_processing.preprocessing import preprocess_session
from uwb_processing.sweep import SweepExpectation, SweepMetrics, score_metrics
from uwb_processing.tracker import MultiTargetTracker
from uwb_processing.types import DetectionConfig, TrackingConfig

ROSBAGS_DIR = _ROOT / "uwb_rosbags"
OUTPUT_JSON = _HERE / "sweep_results.json"
OUTPUT_CSV = _HERE / "sweep_results.csv"
TOP_K = 3
VERBOSE = True

SWEEP_BAGS = [
    "single_person_walking_range_05_22",
    "two_person_range_resolution",
    "two_person_fov",
    "breathing_kaytlin",
    "breathing_kaytlin_edison",
]

PFA_GRID = [1e-3, 5e-3, 1e-2, 3e-2, 6e-2]
GUARD_GRID = [1, 2]
MAX_PEAKS_GRID = [4, 6]
MERGE_TOLERANCE_GRID = [0, 1]
GATE_GRID_M = [0.30, 0.45, 0.60, 0.75]
CONFIRM_HITS_GRID = [2, 3]
MAX_MISSES_GRID = [5, 8, 12]


@dataclass(slots=True)
class SweepCombo:
    pfa: float
    num_guard_cells: int
    max_peaks_per_frame: int
    merge_tolerance_taps: int
    gate_distance_m: float
    confirm_hits: int
    max_misses: int


@dataclass(slots=True)
class ComboResult:
    bag: str
    combo: SweepCombo
    metrics: SweepMetrics
    score: float
    notes: list[str]


BASELINE_COMBO = SweepCombo(
    pfa=1e-3,
    num_guard_cells=2,
    max_peaks_per_frame=4,
    merge_tolerance_taps=1,
    gate_distance_m=0.45,
    confirm_hits=3,
    max_misses=5,
)


BAG_EXPECTATIONS: dict[str, SweepExpectation] = {
    "single_person_walking_range_05_22": SweepExpectation(
        expected_track_count=1,
        expected_max_active_tracks=1,
        min_longest_track_fraction=0.35,
        max_multi_peak_fraction=0.02,
    ),
    "two_person_range_resolution": SweepExpectation(
        expected_track_count=2,
        expected_max_active_tracks=2,
        min_longest_track_fraction=0.10,
        min_multi_peak_fraction=0.08,
        min_separation_pass_rate=0.95,
    ),
    "two_person_fov": SweepExpectation(
        expected_track_count=2,
        expected_max_active_tracks=2,
        min_longest_track_fraction=0.08,
        min_multi_peak_fraction=0.04,
        min_separation_pass_rate=0.90,
    ),
    "breathing_kaytlin": SweepExpectation(
        expected_track_count=1,
        expected_max_active_tracks=1,
        min_longest_track_fraction=0.10,
        expected_breathing_count=1,
        breathing_bpm_range=(12.0, 20.0),
        breathing_arc_range_deg=(30.0, 180.0),
    ),
    "breathing_kaytlin_edison": SweepExpectation(
        expected_track_count=2,
        expected_max_active_tracks=2,
        min_longest_track_fraction=0.08,
        expected_breathing_count=2,
        breathing_bpm_range=(10.0, 24.0),
        breathing_arc_range_deg=(20.0, 180.0),
    ),
}


def _per_path_clutter_mag(preprocessed, path_idx: int) -> np.ndarray:
    frames = preprocessed.session.frames
    global_mean = frames.mean(axis=0, keepdims=True)
    clutter = np.abs(frames - global_mean).astype(np.float32)
    wall_mask = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m
    path_mag = clutter[:, path_idx, :].copy()
    path_mag[:, ~wall_mask] = 0.0
    return path_mag


def _fraction(num: int, den: int) -> float:
    return float(num) / float(den) if den else 0.0


def _best_breathing_track(breathing_tracks: list[dict], expectation: SweepExpectation) -> tuple[float | None, float | None]:
    if not breathing_tracks:
        return None, None
    if expectation.breathing_bpm_range is None and expectation.breathing_arc_range_deg is None:
        first = breathing_tracks[0]
        return first["freq_bpm"], first["phase_arc_deg"]

    def penalty(item: dict) -> float:
        bpm = item["freq_bpm"]
        arc = item["phase_arc_deg"]
        score = 0.0
        if expectation.breathing_bpm_range is not None and bpm is not None:
            low, high = expectation.breathing_bpm_range
            if bpm < low:
                score += low - bpm
            elif bpm > high:
                score += bpm - high
        if expectation.breathing_arc_range_deg is not None and arc is not None:
            low, high = expectation.breathing_arc_range_deg
            if arc < low:
                score += 0.1 * (low - arc)
            elif arc > high:
                score += 0.1 * (arc - high)
        return score

    best = min(breathing_tracks, key=penalty)
    return best["freq_bpm"], best["phase_arc_deg"]


def evaluate_combo(preprocessed, combo: SweepCombo, expectation: SweepExpectation) -> SweepMetrics:
    session = preprocessed.session
    cfar_cfg = CfarDetectionConfig(
        num_ref_cells=8,
        num_guard_cells=combo.num_guard_cells,
        pfa=combo.pfa,
        max_peaks_per_frame=combo.max_peaks_per_frame,
    )

    n_paths = session.frames.shape[1]
    if n_paths >= 2:
        dets_c = run_cfar_session(preprocessed, cfar_cfg, clutter_mag=_per_path_clutter_mag(preprocessed, 0))
        dets_b = run_cfar_session(preprocessed, cfar_cfg, clutter_mag=_per_path_clutter_mag(preprocessed, 1))
        detections = [
            merge_dual_channel_peaks(c, b, fi, preprocessed.range_axis_m, combo.merge_tolerance_taps)
            for fi, (c, b) in enumerate(zip(dets_c, dets_b))
        ]
    else:
        detections = run_cfar_session(preprocessed, cfar_cfg)

    tracker = MultiTargetTracker(
        TrackingConfig(
            dt_s=1.0 / max(preprocessed.frame_rate_hz, 1.0),
            sigma_meas_m=0.15,
            gate_distance_m=combo.gate_distance_m,
            confirm_hits=combo.confirm_hits,
            max_misses=combo.max_misses,
            init_hits=combo.confirm_hits,
            init_window_frames=combo.confirm_hits + 2,
        )
    )

    max_active_track_count = 0
    for fi, dets in enumerate(detections):
        timestamp_s = float(session.timestamps_s[fi]) if fi < len(session.timestamps_s) else float(fi) / max(preprocessed.frame_rate_hz, 1.0)
        active_tracks = tracker.update(fi, timestamp_s, dets)
        max_active_track_count = max(max_active_track_count, len(active_tracks))

    confirmed_tracks = tracker.tracks_snapshot()
    longest_track_fraction = 0.0
    if confirmed_tracks and session.frames.shape[0] > 0:
        longest_track_fraction = max(len(track.history_m) for track in confirmed_tracks) / float(session.frames.shape[0])

    multi_peak_frames = [frame_dets for frame_dets in detections if len(frame_dets) >= 2]
    multi_peak_fraction = _fraction(len(multi_peak_frames), len(detections))

    pair_count = 0
    pair_pass_count = 0
    for frame_dets in multi_peak_frames:
        taps = sorted(det.tap_idx for det in frame_dets)
        for tap_a, tap_b in zip(taps, taps[1:]):
            pair_count += 1
            if (tap_b - tap_a) >= 2:
                pair_pass_count += 1
    separation_pass_rate = _fraction(pair_pass_count, pair_count) if pair_count else None

    breathing_tracks: list[dict] = []
    for track in confirmed_tracks:
        if len(track.history_t) < 32:
            continue
        ts = session.timestamps_s
        t_start = float(min(track.history_t))
        t_end = float(max(track.history_t))
        frame_mask = (ts >= t_start) & (ts <= t_end)
        tap_idx = int(np.clip(
            round(float(np.mean(track.history_m)) / preprocessed.config.tap_spacing_m),
            0,
            preprocessed.highpass_complex.shape[1] - 1,
        ))
        br = run_breathing_extraction(
            preprocessed.highpass_complex,
            preprocessed.peak_tap_per_frame,
            frame_mask,
            tap_idx,
            preprocessed.frame_rate_hz,
        )
        breathing_tracks.append(
            {
                "track_id": track.track_id,
                "is_static": br.is_static,
                "freq_bpm": br.freq_bpm,
                "phase_arc_deg": br.phase_arc_deg,
            }
        )

    valid_breathing_count = 0
    for item in breathing_tracks:
        if not item["is_static"]:
            continue
        bpm = item["freq_bpm"]
        arc = item["phase_arc_deg"]
        bpm_ok = expectation.breathing_bpm_range is None or (
            bpm is not None and expectation.breathing_bpm_range[0] <= bpm <= expectation.breathing_bpm_range[1]
        )
        arc_ok = expectation.breathing_arc_range_deg is None or (
            arc is not None and expectation.breathing_arc_range_deg[0] <= arc <= expectation.breathing_arc_range_deg[1]
        )
        if bpm_ok and arc_ok:
            valid_breathing_count += 1

    best_breathing_bpm, best_breathing_arc_deg = _best_breathing_track(breathing_tracks, expectation)

    return SweepMetrics(
        confirmed_track_count=len(confirmed_tracks),
        max_active_track_count=max_active_track_count,
        longest_track_fraction=longest_track_fraction,
        multi_peak_fraction=multi_peak_fraction,
        separation_pass_rate=separation_pass_rate,
        breathing_candidate_count=len(breathing_tracks),
        valid_breathing_count=valid_breathing_count,
        best_breathing_bpm=best_breathing_bpm,
        best_breathing_arc_deg=best_breathing_arc_deg,
    )


def _result_to_dict(result: ComboResult) -> dict:
    return {
        "bag": result.bag,
        "combo": asdict(result.combo),
        "metrics": asdict(result.metrics),
        "score": result.score,
        "notes": result.notes,
    }


def main() -> None:
    verbose = VERBOSE
    rosbags_dir = ROSBAGS_DIR
    if not rosbags_dir.exists():
        sys.exit(f"rosbags directory not found: {rosbags_dir}")

    selected_bags = [bag for bag in SWEEP_BAGS if bag in BAG_EXPECTATIONS]
    if not selected_bags:
        sys.exit("No known bags selected.")

    combos = [
        SweepCombo(
            pfa=pfa,
            num_guard_cells=guard,
            max_peaks_per_frame=max_peaks,
            merge_tolerance_taps=merge_tol,
            gate_distance_m=gate,
            confirm_hits=confirm_hits,
            max_misses=max_misses,
        )
        for pfa, guard, max_peaks, merge_tol, gate, confirm_hits, max_misses in itertools.product(
            PFA_GRID,
            GUARD_GRID,
            MAX_PEAKS_GRID,
            MERGE_TOLERANCE_GRID,
            GATE_GRID_M,
            CONFIRM_HITS_GRID,
            MAX_MISSES_GRID,
        )
    ]

    if verbose:
        print(f"Sweeping {len(combos)} parameter combinations across {len(selected_bags)} bag(s).")

    det_cfg = DetectionConfig(
        tap_spacing_m=0.15,
        default_range_gate_m=(0.30, 6.0),
        wall_clip_m=0.30,
    )

    report: dict[str, dict] = {}
    csv_rows: list[dict] = []

    for bag_name in selected_bags:
        bag_path = rosbags_dir / bag_name
        if not bag_path.exists():
            if verbose:
                print(f"[skip] {bag_name}: directory not found")
            continue

        expectation = BAG_EXPECTATIONS[bag_name]
        if verbose:
            print(f"\n[{bag_name}]")
        load_t0 = time.time()
        try:
            session = load_session(bag_path)
            preprocessed = preprocess_session(session, det_cfg)
        except Exception as exc:
            if verbose:
                print(f"  [skip] failed to load session: {exc}")
            report[bag_name] = {"error": str(exc)}
            continue
        load_elapsed = time.time() - load_t0
        if verbose:
            print(
                f"  loaded in {load_elapsed:.1f}s  |  {session.frames.shape[0]} frames  |  {preprocessed.frame_rate_hz:.1f} Hz"
            )

        baseline_metrics = evaluate_combo(preprocessed, BASELINE_COMBO, expectation)
        baseline_score, baseline_notes = score_metrics(baseline_metrics, expectation)

        results: list[ComboResult] = []
        for combo in combos:
            metrics = evaluate_combo(preprocessed, combo, expectation)
            score, notes = score_metrics(metrics, expectation)
            results.append(ComboResult(bag=bag_name, combo=combo, metrics=metrics, score=score, notes=notes))

        results.sort(key=lambda item: item.score, reverse=True)
        best = results[0]
        if verbose:
            print(
                f"  baseline score={baseline_score:.1f} tracks={baseline_metrics.confirmed_track_count} "
                f"active={baseline_metrics.max_active_track_count} multi={baseline_metrics.multi_peak_fraction:.3f}"
            )
            print(
                f"  best     score={best.score:.1f} tracks={best.metrics.confirmed_track_count} "
                f"active={best.metrics.max_active_track_count} multi={best.metrics.multi_peak_fraction:.3f} "
                f"combo={asdict(best.combo)}"
            )

        report[bag_name] = {
            "baseline": {
                "combo": asdict(BASELINE_COMBO),
                "metrics": asdict(baseline_metrics),
                "score": baseline_score,
                "notes": baseline_notes,
            },
            "best": _result_to_dict(best),
            "top_results": [_result_to_dict(item) for item in results[: max(1, TOP_K)]],
        }

        csv_rows.append(
            {
                "bag": bag_name,
                "kind": "baseline",
                "score": baseline_score,
                **asdict(BASELINE_COMBO),
                **asdict(baseline_metrics),
            }
        )
        for rank, item in enumerate(results[: max(1, TOP_K)], start=1):
            csv_rows.append(
                {
                    "bag": bag_name,
                    "kind": f"best_{rank}",
                    "score": item.score,
                    **asdict(item.combo),
                    **asdict(item.metrics),
                }
            )

    output_json = OUTPUT_JSON
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(report, indent=2))

    output_csv = OUTPUT_CSV
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    if csv_rows:
        with output_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
            writer.writeheader()
            writer.writerows(csv_rows)

    if verbose:
        print(f"\nSaved JSON report to {output_json}")
        print(f"Saved CSV report to {output_csv}")


if __name__ == "__main__":
    main()
