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
BAG_NAME = "breathing-blank-new-1"

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

# Quick-run toggles. Edit these instead of typing CLI flags.
QUICKRUN_TOPIC = "/uwb/frame_raw"
QUICKRUN_ENABLE_CIR_ANIMATION = False
QUICKRUN_ENABLE_AOA_ANIMATION = False
QUICKRUN_ENABLE_3D_SURFACE = False
QUICKRUN_ENABLE_RDM_ANIMATION = False       # ← set True to generate range_doppler_animation.mp4
QUICKRUN_ENABLE_BREATHING = True            # ← set True to generate breathing_map.png + rate estimate
QUICKRUN_USE_MOVIEPY = False
QUICKRUN_USE_MULTIPATH_CFAR = False
QUICKRUN_AOA_RX_SPACING_M = 0.10
QUICKRUN_AOA_FOV_DEG = 60.0
QUICKRUN_AOA_ANGLE_BINS = 121

import numpy as np

from .analysis import build_track_slow_time_signal, compute_breathing_map, compute_spectrogram
from .animate_aoa import save_aoa_animation
from .annotations import load_annotation_file
from .aoa import build_range_angle_map, estimate_track_aoa_session, pair_rx_channels
from .cfar import combine_paths_max, run_cfar_session
from .detection import detect_window
from .loaders import load_session
from .plotting import (
    save_all_cfar_peaks_tap_plot,
    save_breathing_map,
    save_multi_peak_tracking_plot,
    save_peak_tracking_plot,
    save_phase_iq_plot,
    save_presence_monitoring_plot,
    save_presence_score_plot,
    save_range_doppler_heatmap,
    save_range_doppler_heatmap_animation,
    save_range_doppler_plot,
    save_range_time_plot,
    save_spectrogram_plot,
)
from .preprocessing import preprocess_session
from .tracker import MultiTargetTracker
from .types import (
    AnnotationWindow,
    CfarDetectionConfig,
    DetectionConfig,
    SessionAnnotation,
    TrackingConfig,
)


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


def _is_supported_session_dir(path: Path) -> bool:
    return path.is_dir() and (
        (path / "metadata.yaml").exists()
        or any(path.glob("*.npz"))
    )


def _discover_session_inputs(input_path: str | Path, all_rosbags: bool) -> list[Path]:
    session_path = Path(input_path)
    if not all_rosbags:
        return [session_path]

    if _is_supported_session_dir(session_path):
        return [session_path]

    if not session_path.is_dir():
        raise FileNotFoundError(f"Session root does not exist: {session_path}")

    discovered = sorted(
        (child for child in session_path.iterdir() if _is_supported_session_dir(child)),
        key=lambda child: child.name.lower(),
    )
    if not discovered:
        raise FileNotFoundError(f"No rosbag or NPZ session directories found in {session_path}")
    return discovered


def _resolve_annotation_path(
    session_input: str | Path,
    annotation_path: str | Path | None,
    annotations_root: Path = _ANNOTATIONS,
) -> Path | None:
    session_dir = Path(session_input)
    if annotation_path is None:
        default_annotation = annotations_root / f"{session_dir.name}.yaml"
        return default_annotation if default_annotation.exists() else None

    resolved = Path(annotation_path)
    if resolved.is_dir():
        candidate = resolved / f"{session_dir.name}.yaml"
        return candidate if candidate.exists() else None
    return resolved


def _resolve_output_dir(
    session_input: str | Path,
    output_dir: str | Path | None,
    *,
    batch_mode: bool,
) -> Path:
    session_dir = Path(session_input)
    if output_dir is None:
        return session_dir / "analysis"

    resolved = Path(output_dir)
    return resolved / session_dir.name if batch_mode else resolved


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
        "presence_frames": int(np.count_nonzero(preprocessed.presence_mask)),
        "presence_fraction": float(np.mean(preprocessed.presence_mask.astype(np.float32))),
        "presence_score_peak": float(np.max(preprocessed.roi_to_background_power_ratio, initial=0.0)),
        "background_power_reference": float(preprocessed.background_power_reference),
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


def _write_tracks_csv(path: Path, tracks_per_frame: list[list], timestamps: np.ndarray) -> None:
    import csv as _csv
    with path.open("w", newline="") as fh:
        writer = _csv.writer(fh)
        writer.writerow(["frame_idx", "timestamp_s", "track_id", "range_m", "range_rate_m_s", "confirmed"])
        for fi, tracks in enumerate(tracks_per_frame):
            ts = float(timestamps[fi]) if fi < len(timestamps) else float("nan")
            for track in tracks:
                writer.writerow([
                    fi, f"{ts:.4f}", track.track_id,
                    f"{track.state[0]:.4f}", f"{track.state[1]:.4f}",
                    int(track.confirmed),
                ])


def process_session(
    input_path: str | Path,
    annotation_path: str | Path | None = None,
    output_dir: str | Path | None = None,
    config: DetectionConfig | None = None,
    cfar_config: CfarDetectionConfig | None = None,
    topic: str = "/uwb/frame_raw",
    animate: bool = False,
    animate_aoa: bool = False,
    animate_3d: bool = False,
    rdm_animate: bool = False,
    breathing: bool = False,
    use_moviepy: bool = False,
    use_multipath: bool = False,
    aoa_rx_spacing_m: float = 0.10,
    aoa_fov_deg: float = 60.0,
    aoa_angle_bins: int = 121,
) -> tuple[dict[str, Any], Path]:
    config = config or DetectionConfig()
    cfar_config = cfar_config or CfarDetectionConfig()

    session = load_session(input_path, topic=topic)
    annotation = load_annotation_file(annotation_path) if annotation_path else _default_annotation(
        session_id=session.session_id,
        duration_s=float(session.timestamps_s[-1]) if session.timestamps_s.size else 0.0,
    )

    initial_gate = annotation.windows[0].expected_range_m if annotation.windows else None
    preprocessed = preprocess_session(session, config=config, range_gate_m=initial_gate)

    base_output = Path(output_dir) if output_dir else Path(input_path) / "analysis"
    base_output.mkdir(parents=True, exist_ok=True)

    # --- existing single-path plots ---
    save_range_time_plot(preprocessed, base_output / "range_time.png")
    save_peak_tracking_plot(preprocessed, base_output / "peak_tracking.png")
    save_range_doppler_plot(preprocessed, base_output / "range_doppler.png")
    save_phase_iq_plot(preprocessed, base_output / "phase_iq.png")
    save_presence_score_plot(preprocessed, base_output / "presence_score.png")
    save_presence_monitoring_plot(preprocessed, base_output / "presence_monitoring.png")

    # --- scientific Range-Doppler heatmap (jet, range on X, velocity on Y) ---
    save_range_doppler_heatmap(preprocessed, base_output / "range_doppler_heatmap.png")

    # --- breathing (vital-sign) map: range x breathing-rate + rate estimate ---
    breathing_result = None
    if breathing:
        breathing_result = compute_breathing_map(preprocessed)
        save_breathing_map(breathing_result, base_output / "breathing_map.png")
        print(
            f"[breathing]  #1 (best): {breathing_result.best_rate_bpm:.1f} brpm "
            f"@ {breathing_result.best_range_m:.2f} m "
            f"(SNR {breathing_result.snr_db:.1f} dB)"
        )
        # ---- top-5 runner-up peaks ----------------------------------------
        band_mask = (
            (breathing_result.rate_bpm >= breathing_result.band_hz[0] * 60.0)
            & (breathing_result.rate_bpm <= breathing_result.band_hz[1] * 60.0)
        )
        mag_lin = 10.0 ** (breathing_result.power_db / 20.0)   # (F, R) linear
        band_mag = mag_lin[band_mask, :]                        # (Fb, R)
        band_rates = breathing_result.rate_bpm[band_mask]       # (Fb,)
        # Flatten and pick the top 6 cells by magnitude (best + 5 runners-up).
        flat_indices = np.argsort(band_mag.ravel())[::-1]
        print("[breathing]  next 5 peaks in physiological band:")
        printed = 0
        for flat_idx in flat_indices:
            if printed >= 6:
                break
            fi, ri = np.unravel_index(flat_idx, band_mag.shape)
            rate_bpm = float(band_rates[fi])
            range_m  = float(breathing_result.range_axis_m[ri])
            peak_val = float(band_mag[fi, ri])
            noise_ref = float(np.median(mag_lin[~band_mask, ri])) if np.any(~band_mask) else 1e-9
            snr = 20.0 * np.log10((peak_val + 1e-9) / (noise_ref + 1e-9))
            if printed == 0:
                printed += 1   # skip rank-1 (already printed as best)
                continue
            print(f"  #{printed + 1}: {rate_bpm:.1f} brpm @ {range_m:.2f} m  SNR {snr:.1f} dB")
            printed += 1
        # -------------------------------------------------------------------
    # if rdm_animate:
    #     print("[rdm] generating Range-Doppler animation ...")
    #     save_range_doppler_heatmap_animation(
    #         preprocessed, base_output / "range_doppler_animation.mp4"
    #     )
    #     print("[rdm] Range-Doppler animation saved.")

    # --- CFAR multi-peak detection ---
    cfar_clutter = combine_paths_max(preprocessed) if use_multipath else None
    detections_per_frame = run_cfar_session(preprocessed, cfar_config, clutter_mag=cfar_clutter)

    track_cfg = TrackingConfig(
        dt_s=1.0 / preprocessed.frame_rate_hz,
        sigma_meas_m=config.tap_spacing_m,
    )
    tracker = MultiTargetTracker(track_cfg)
    tracks_per_frame: list[list] = []
    for fi, (ts, dets) in enumerate(zip(session.timestamps_s, detections_per_frame)):
        tracks_per_frame.append(tracker.update(fi, float(ts), dets))

    all_confirmed = tracker.tracks_snapshot()

    # --- multi-peak outputs ---
    from .visualize_3d import save_cfar_range_time_plot
    save_cfar_range_time_plot(
        preprocessed, detections_per_frame, all_confirmed,
        base_output / "cfar_range_time.png",
    )
    save_all_cfar_peaks_tap_plot(preprocessed, detections_per_frame, base_output / "all_cfar_peaks_taps.png")
    save_multi_peak_tracking_plot(preprocessed, all_confirmed, base_output / "multi_peak_tracks.png")
    _write_tracks_csv(base_output / "multi_peak_tracks.csv", tracks_per_frame, session.timestamps_s)

    # --- per-track Doppler and micro-Doppler at the track's moving range ---
    min_frames_for_stft = max(8, int(preprocessed.frame_rate_hz * 1.0))
    for track in all_confirmed:
        if len(track.history_t) < min_frames_for_stft:
            continue
        track_dir = base_output / f"track_{track.track_id:02d}"
        track_dir.mkdir(exist_ok=True)

        signal, mean_range_m = build_track_slow_time_signal(preprocessed, track)
        if signal.size < 4:
            continue

        mean_tap = int(round(mean_range_m / config.tap_spacing_m))

        doppler = compute_spectrogram(
            signal,
            frame_rate_hz=preprocessed.frame_rate_hz,
            window_s=config.doppler_window_s,
            overlap=config.stft_overlap,
            kind="doppler",
            selected_range_m=mean_range_m,
            selected_tap=mean_tap,
            selected_path=preprocessed.selected_path,
        )
        save_spectrogram_plot(doppler, track_dir / "doppler.png",
                              frequency_limit_hz=config.doppler_limit_hz)

        phase_signal = np.unwrap(np.angle(signal.astype(np.complex128))).astype(np.float64)
        microdoppler = compute_spectrogram(
            phase_signal,
            frame_rate_hz=preprocessed.frame_rate_hz,
            window_s=config.microdoppler_window_s,
            overlap=config.stft_overlap,
            kind="microdoppler",
            selected_range_m=mean_range_m,
            selected_tap=mean_tap,
            selected_path=preprocessed.selected_path,
        )
        save_spectrogram_plot(microdoppler, track_dir / "microdoppler.png",
                              frequency_limit_hz=config.microdoppler_limit_hz)

    if animate_3d:
        from .visualize_3d import save_cir_3d_surface
        ds = max(1, preprocessed.clutter_removed.shape[0] // 500)
        save_cir_3d_surface(
            preprocessed,
            base_output / "range_time_3d.png",
            downsample_frames=ds,
            confirmed_tracks=all_confirmed,
        )

    if animate:
        from .animate_cir import animate_range_time
        animate_range_time(
            preprocessed, detections_per_frame, tracks_per_frame,
            output_path=base_output / "cir_animation.mp4",
            fps=10,
            window_frames=100,
            use_moviepy=use_moviepy,
        )

    if animate_aoa:
        print("[aoa] preparing paired RX channels")
        paired = pair_rx_channels(preprocessed.session)
        print("[aoa] estimating per-frame track angles")
        track_aoas_per_frame = estimate_track_aoa_session(
            paired,
            detections_per_frame,
            tracks_per_frame,
            carrier_frequency_hz=config.carrier_frequency_hz,
            rx_spacing_m=aoa_rx_spacing_m,
            fov_limit_deg=aoa_fov_deg,
        )
        range_angle_frames: list[np.ndarray] = []
        angle_axis_deg = np.linspace(-aoa_fov_deg, aoa_fov_deg, aoa_angle_bins, dtype=np.float32)
        range_axis_m = preprocessed.range_axis_m.astype(np.float32)
        print("[aoa] building range-angle grids")
        for frame_idx in range(len(session.timestamps_s)):
            if frame_idx == 0 or frame_idx == len(session.timestamps_s) - 1 or frame_idx % 25 == 0:
                print(f"[aoa] grid frame {frame_idx + 1}/{len(session.timestamps_s)}")
            grid, angle_axis_deg, range_axis_m = build_range_angle_map(
                paired,
                range_axis_m=range_axis_m,
                frame_idx=frame_idx,
                carrier_frequency_hz=config.carrier_frequency_hz,
                rx_spacing_m=aoa_rx_spacing_m,
                fov_limit_deg=aoa_fov_deg,
                angle_bins=aoa_angle_bins,
            )
            range_angle_frames.append(grid)
        print("[aoa] encoding aoa_tracks.mp4")
        save_aoa_animation(
            timestamps_s=session.timestamps_s,
            track_aoas_per_frame=track_aoas_per_frame,
            range_angle_grids=np.asarray(range_angle_frames, dtype=np.float32),
            angle_axis_deg=angle_axis_deg,
            range_axis_m=range_axis_m,
            output_path=base_output / "aoa_tracks.mp4",
            fov_limit_deg=aoa_fov_deg,
        )

    # --- per-window detection (existing) ---
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
    summary["cfar_config"] = {
        "num_ref_cells": cfar_config.num_ref_cells,
        "num_guard_cells": cfar_config.num_guard_cells,
        "pfa": cfar_config.pfa,
        "variant": cfar_config.variant,
        "max_peaks_per_frame": cfar_config.max_peaks_per_frame,
    }
    summary["confirmed_tracks"] = len(all_confirmed)
    if breathing_result is not None:
        summary["breathing"] = breathing_result.to_dict()
    (base_output / "summary.json").write_text(json.dumps(summary, indent=2))
    _write_detection_csv(base_output / "detections.csv", detections)
    return summary, base_output


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run offline UWB processing for one rosbag or NPZ session.")
    parser.add_argument("--input", default=None, help="Rosbag directory or NPZ directory (defaults to BAG_NAME).")
    parser.add_argument(
        "--all-rosbags",
        action="store_true",
        help="Process every rosbag/NPZ session under --input, or under uwb_rosbags/ when --input is omitted.",
    )
    parser.add_argument("--annotation", help="YAML annotation file for the session.")
    parser.add_argument("--output-dir", help="Directory for plots and summaries.")
    parser.add_argument("--topic", default=QUICKRUN_TOPIC, help="Rosbag topic containing UWB frames.")
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
    # CFAR multi-peak detection
    parser.add_argument("--cfar-pfa", type=float, default=1e-3,
                        help="CFAR target false-alarm probability (default 1e-3).")
    parser.add_argument("--cfar-ref", type=int, default=8,
                        help="CFAR reference cells each side (default 8 ≈ 1.2 m).")
    parser.add_argument("--cfar-guard", type=int, default=2,
                        help="CFAR guard cells each side (default 2 ≈ 0.3 m).")
    parser.add_argument("--cfar-variant", default="CA", choices=["CA", "OS", "GO"],
                        help="CFAR variant: CA (default), OS, or GO.")
    parser.add_argument("--max-peaks", type=int, default=4,
                        help="Maximum CFAR peaks returned per frame (default 4).")
    parser.add_argument("--gate-m", type=float, default=0.75,
                        help="Kalman tracker gate distance in metres (default 0.75).")
    # Visualisation
    parser.add_argument("--rdm-animate", action="store_true", default=QUICKRUN_ENABLE_RDM_ANIMATION,
                        help="Produce range_doppler_animation.mp4 via sliding-window FFT.")
    parser.add_argument("--breathing", action="store_true", default=QUICKRUN_ENABLE_BREATHING,
                        help="Produce breathing_map.png (range x breathing-rate) and estimate the breathing rate.")
    parser.add_argument("--animate", action="store_true", default=QUICKRUN_ENABLE_CIR_ANIMATION,
                        help="Produce cir_animation.mp4 (requires ffmpeg).")
    parser.add_argument("--animate-aoa", action="store_true", default=QUICKRUN_ENABLE_AOA_ANIMATION,
                        help="Produce aoa_tracks.mp4 using two-RX AoA estimation.")
    parser.add_argument("--animate-3d", action="store_true", default=QUICKRUN_ENABLE_3D_SURFACE,
                        help="Produce range_time_3d.png (3-D CIR surface).")
    parser.add_argument("--use-moviepy", action="store_true", default=QUICKRUN_USE_MOVIEPY,
                        help="Use moviepy for animation instead of FuncAnimation.")
    parser.add_argument("--multipath", action="store_true", default=QUICKRUN_USE_MULTIPATH_CFAR,
                        help="Combine all RX paths (element-wise max) before CFAR instead of using only the strongest path.")
    parser.add_argument("--aoa-rx-spacing-m", type=float, default=QUICKRUN_AOA_RX_SPACING_M,
                        help="RX antenna spacing in metres used for AoA estimation.")
    parser.add_argument("--aoa-fov-deg", type=float, default=QUICKRUN_AOA_FOV_DEG,
                        help="Symmetric AoA field of view limit in degrees.")
    parser.add_argument("--aoa-angle-bins", type=int, default=QUICKRUN_AOA_ANGLE_BINS,
                        help="Number of angle bins in the range-angle heatmap.")
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
    )


def cfar_config_from_args(args: argparse.Namespace) -> CfarDetectionConfig:
    return CfarDetectionConfig(
        num_ref_cells=args.cfar_ref,
        num_guard_cells=args.cfar_guard,
        pfa=args.cfar_pfa,
        variant=args.cfar_variant,
        max_peaks_per_frame=args.max_peaks,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    input_root = args.input or (_ROSBAGS_ROOT if args.all_rosbags else _DEFAULT_INPUT)
    session_inputs = _discover_session_inputs(input_root, all_rosbags=args.all_rosbags)
    batch_mode = args.all_rosbags and len(session_inputs) > 1
    config = config_from_args(args)
    cfar_config = cfar_config_from_args(args)

    results: list[dict[str, Any]] = []
    for index, session_input in enumerate(session_inputs, start=1):
        if batch_mode:
            print(f"[batch] processing {index}/{len(session_inputs)}: {session_input.name}")

        session_annotation = _resolve_annotation_path(session_input, args.annotation)
        session_output_dir = _resolve_output_dir(session_input, args.output_dir, batch_mode=batch_mode)

        summary, out = process_session(
            input_path=session_input,
            annotation_path=session_annotation,
            output_dir=session_output_dir,
            config=config,
            cfar_config=cfar_config,
            topic=args.topic,
            animate=args.animate,
            animate_aoa=args.animate_aoa,
            animate_3d=args.animate_3d,
            rdm_animate=args.rdm_animate,
            breathing=args.breathing,
            use_moviepy=args.use_moviepy,
            use_multipath=args.multipath,
            aoa_rx_spacing_m=args.aoa_rx_spacing_m,
            aoa_fov_deg=args.aoa_fov_deg,
            aoa_angle_bins=args.aoa_angle_bins,
        )
        results.append(
            {
                "session_id": summary["session_id"],
                "input_path": str(session_input),
                "output_dir": str(out),
                "detections": summary["detections"],
            }
        )

    if batch_mode:
        print(json.dumps({"processed_sessions": results}, indent=2))
    else:
        print(json.dumps(results[0], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
