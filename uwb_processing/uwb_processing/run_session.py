from __future__ import annotations

# Allow direct execution (python3 run_session.py) in addition to module form.
if __name__ == "__main__" and __package__ is None:
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
    __package__ = "uwb_processing"

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Quick-run settings — only change BAG_NAME, then run:
#   python -m uwb_processing.run_session
# Paths are resolved relative to the current working directory (/home/ws/src).
# ---------------------------------------------------------------------------
BAG_NAME = "outside_allen_one_person"

# Resolve roots from the repo src/ directory so the script works regardless
# of where it is invoked from (e.g. /home/ws vs /home/ws/src).
# __file__ lives at <src>/uwb_processing/uwb_processing/run_session.py
# parents[2] → <src>/
_SRC_ROOT     = Path(__file__).resolve().parents[2]
_ROSBAGS_ROOT = _SRC_ROOT / "uwb_rosbags"
_ANNOTATIONS  = _SRC_ROOT / "ground_truth/annotations"

_DEFAULT_INPUT      = _ROSBAGS_ROOT / BAG_NAME
_DEFAULT_ANNOTATION = _ANNOTATIONS / f"{BAG_NAME}.yaml"   # used only if the file exists
_DEFAULT_OUTPUT     = _ROSBAGS_ROOT / BAG_NAME / "analysis"

import numpy as np

from .annotations import load_annotation_file
from .detection import detect_window
from .loaders import load_session
from .plotting import (
    save_peak_tracking_plot,
    save_range_doppler_video,
    save_range_time_plot,
    save_spectrogram_plot,
)
from .preprocessing import preprocess_session
from .types import AnnotationWindow, DetectionConfig, SessionAnnotation


def _slugify(value: str) -> str:
    keep = [ch.lower() if ch.isalnum() else "_" for ch in value]
    return "".join(keep).strip("_") or "window"


def _default_annotation(session_id: str, duration_s: float) -> SessionAnnotation:
    return SessionAnnotation(
        session_id=session_id,
        source=session_id,
        notes="Auto-generated full-session window because no annotation file was provided.",
        windows=[
            AnnotationWindow(
                label="unknown",
                start_s=0.0,
                end_s=duration_s,
                expected_range_m=None,
                notes="Full session auto-window.",
            )
        ],
    )


def _summary_from_session(session, preprocessed, detections, annotation_path: Path | None) -> dict[str, Any]:
    return {
        "session_id": session.session_id,
        "source_path": str(session.source_path),
        "source_kind": session.source_kind,
        "annotation_path": str(annotation_path) if annotation_path else None,
        "frame_count": int(session.frames.shape[0]),
        "path_count": int(session.frames.shape[1]),
        "tap_count": int(session.frames.shape[2]),
        "duration_s": float(session.timestamps_s[-1]) if session.timestamps_s.size else 0.0,
        "frame_rate_hz": preprocessed.frame_rate_hz,
        "selected_path": preprocessed.selected_path,
        "dominant_tap": preprocessed.dominant_tap,
        "dominant_range_m": float(preprocessed.range_axis_m[preprocessed.dominant_tap]),
        "significant_taps": [int(t) for t in preprocessed.significant_taps],
        "significant_ranges_m": [float(r) for r in preprocessed.significant_ranges_m],
        "peak_magnitude_threshold_db": preprocessed.config.peak_magnitude_threshold_db,
        "range_doppler_video": None,
        "detections": [result.to_dict() for result in detections],
        "config": asdict(preprocessed.config),
    }


def _write_detection_csv(path: Path, detections) -> None:
    fieldnames = list(detections[0].to_dict().keys()) if detections else []
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for detection in detections:
            writer.writerow(detection.to_dict())


def process_session(
    input_path: str | Path,
    annotation_path: str | Path | None = None,
    output_dir: str | Path | None = None,
    config: DetectionConfig | None = None,
    topic: str = "/uwb/frame_raw",
    show_video_progress: bool = True,
) -> tuple[dict[str, Any], Path]:
    config = config or DetectionConfig()
    session = load_session(input_path, topic=topic)
    annotation = load_annotation_file(annotation_path) if annotation_path else _default_annotation(
        session_id=session.session_id,
        duration_s=float(session.timestamps_s[-1]) if session.timestamps_s.size else 0.0,
    )

    initial_gate = annotation.windows[0].expected_range_m if annotation.windows else None
    preprocessed = preprocess_session(session, config=config, range_gate_m=initial_gate)

    base_output = Path(output_dir) if output_dir else Path(input_path) / "analysis"
    base_output.mkdir(parents=True, exist_ok=True)

    save_range_time_plot(preprocessed, base_output / "range_time.png")
    save_peak_tracking_plot(preprocessed, base_output / "peak_tracking.png")
    range_doppler_video_path = save_range_doppler_video(
        preprocessed,
        base_output / "range_doppler.mp4",
        show_progress=show_video_progress,
    )

    detections = []
    for window in annotation.windows:
        window_dir = base_output / _slugify(window.label)
        window_dir.mkdir(parents=True, exist_ok=True)
        detection, artifacts = detect_window(preprocessed, window, artifact_dir=window_dir)
        save_spectrogram_plot(
            artifacts["doppler"],
            window_dir / "doppler.png",
            frequency_limit_hz=config.doppler_limit_hz,
        )
        save_spectrogram_plot(
            artifacts["microdoppler"],
            window_dir / "microdoppler.png",
            frequency_limit_hz=config.microdoppler_limit_hz,
        )
        detections.append(detection)

    summary = _summary_from_session(
        session=session,
        preprocessed=preprocessed,
        detections=detections,
        annotation_path=Path(annotation_path) if annotation_path else None,
    )
    summary["range_doppler_video"] = str(range_doppler_video_path)
    (base_output / "summary.json").write_text(json.dumps(summary, indent=2))
    _write_detection_csv(base_output / "detections.csv", detections)
    return summary, base_output


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run offline UWB processing for one rosbag or NPZ session.")
    parser.add_argument("--input", default=None, help="Rosbag directory or NPZ directory (defaults to BAG_NAME).")
    parser.add_argument("--annotation", help="YAML annotation file for the session.")
    parser.add_argument("--output-dir", help="Directory for plots and summaries.")
    parser.add_argument("--topic", default="/uwb/frame_raw", help="Rosbag topic containing UWB frames.")
    parser.add_argument("--default-range-min-m", type=float, default=0.3)
    parser.add_argument("--default-range-max-m", type=float, default=6.0)
    parser.add_argument("--wall-clip-m", type=float, default=0.3)
    parser.add_argument("--motion-threshold", type=float, default=3.0)
    parser.add_argument(
        "--enable-offline-rds",
        action="store_true",
        help="Apply an additional offline RDS high-pass. Leave disabled when the radar firmware already performs drift compensation.",
    )
    parser.add_argument("--rds-cutoff-hz", type=float, default=0.2)
    parser.add_argument("--carrier-frequency-hz", type=float, default=6.5e9,
                        help="UWB carrier frequency in Hz (default: 6.5e9 = channel 5).")
    parser.add_argument("--doppler-window-s", type=float, default=4.0)
    parser.add_argument("--microdoppler-window-s", type=float, default=1.5)
    parser.add_argument("--stft-overlap", type=float, default=0.75)
    parser.add_argument(
        "--peak-magnitude-threshold-db",
        type=float,
        default=55.0,
        help="Plot all local-max range peaks at or above this dB level in peak_tracking.png.",
    )
    parser.add_argument(
        "--peak-min-separation-taps",
        type=int,
        default=2,
        help="Minimum tap spacing between reported peaks.",
    )
    parser.add_argument(
        "--no-video-progress",
        action="store_true",
        help="Disable tqdm progress bar during range_doppler video export.",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> DetectionConfig:
    return DetectionConfig(
        default_range_gate_m=(args.default_range_min_m, args.default_range_max_m),
        wall_clip_m=args.wall_clip_m,
        offline_rds_enabled=args.enable_offline_rds,
        rds_cutoff_hz=args.rds_cutoff_hz,
        carrier_frequency_hz=args.carrier_frequency_hz,
        motion_threshold=args.motion_threshold,
        doppler_window_s=args.doppler_window_s,
        microdoppler_window_s=args.microdoppler_window_s,
        stft_overlap=args.stft_overlap,
        peak_magnitude_threshold_db=args.peak_magnitude_threshold_db,
        peak_min_separation_taps=args.peak_min_separation_taps,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    input_path      = args.input      or _DEFAULT_INPUT
    output_dir      = args.output_dir or _DEFAULT_OUTPUT
    annotation_path = args.annotation
    if annotation_path is None and _DEFAULT_ANNOTATION.exists():
        annotation_path = _DEFAULT_ANNOTATION

    summary, out = process_session(
        input_path=input_path,
        annotation_path=annotation_path,
        output_dir=output_dir,
        config=config_from_args(args),
        topic=args.topic,
        show_video_progress=not args.no_video_progress,
    )
    print(json.dumps({"output_dir": str(out), "detections": summary["detections"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
