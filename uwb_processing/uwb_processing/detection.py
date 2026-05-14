from __future__ import annotations

import numpy as np

from .analysis import build_phase_signal, build_slow_time_signal, compute_spectrogram, dominant_frequency_hz
from .types import AnnotationWindow, DetectionResult, PreprocessedSession


ABSENT_LABELS = {"empty", "absence", "no_human", "no_presence", "background_only"}


def window_mask(preprocessed: PreprocessedSession, start_s: float, end_s: float) -> np.ndarray:
    timestamps = preprocessed.session.timestamps_s
    return (timestamps >= start_s) & (timestamps <= end_s)


def infer_expected_presence(label: str) -> bool | None:
    normalized = label.strip().lower()
    if not normalized or normalized == "unknown":
        return None
    return normalized not in ABSENT_LABELS


def detect_window(
    preprocessed: PreprocessedSession,
    window: AnnotationWindow,
    artifact_dir,
) -> tuple[DetectionResult, dict[str, object]]:
    frame_mask = window_mask(preprocessed, window.start_s, window.end_s)
    if not np.any(frame_mask):
        raise ValueError(f"Window {window.label!r} does not overlap the session timestamps.")

    range_gate_m = window.expected_range_m or preprocessed.config.default_range_gate_m
    low_m, high_m = range_gate_m
    roi_mask = (preprocessed.range_axis_m >= low_m) & (preprocessed.range_axis_m <= high_m)
    if not np.any(roi_mask):
        roi_mask = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m

    window_signal = preprocessed.highpass_complex[frame_mask]  # (W, T) complex
    if window_signal.size == 0:
        raise ValueError(f"Window {window.label!r} has an empty signal after frame masking.")

    # Power (E[|x|²]) per frame in ROI — AN-SCA-14453 §9.4
    roi_power_per_frame = np.mean(np.abs(window_signal[:, roi_mask]) ** 2, axis=1)

    # Reference variance: median power across all taps to isolate target tap contribution
    all_tap_power = np.mean(np.abs(window_signal) ** 2, axis=0)
    reference_variance = float(np.median(all_tap_power)) + 1e-6

    power_p95 = float(np.percentile(roi_power_per_frame, 95))
    power_mean = float(np.mean(roi_power_per_frame))
    score_ratio = power_p95 / reference_variance
    predicted_present = bool(score_ratio >= preprocessed.config.motion_threshold)

    # Dominant tap by power within ROI
    roi_tap_indices = np.flatnonzero(roi_mask)
    roi_tap_power = np.mean(np.abs(window_signal[:, roi_mask]) ** 2, axis=0)
    dominant_tap_local = int(np.argmax(roi_tap_power))
    dominant_tap = int(roi_tap_indices[dominant_tap_local])
    dominant_range_m = float(preprocessed.range_axis_m[dominant_tap])

    # Doppler: complex IQ at dominant tap
    slow_time_signal, selected_tap = build_slow_time_signal(preprocessed, frame_mask, range_gate_m)
    doppler = compute_spectrogram(
        slow_time_signal,
        frame_rate_hz=preprocessed.frame_rate_hz,
        window_s=preprocessed.config.doppler_window_s,
        overlap=preprocessed.config.stft_overlap,
        kind="doppler",
        selected_range_m=float(preprocessed.range_axis_m[selected_tap]),
        selected_tap=selected_tap,
        selected_path=preprocessed.selected_path,
    )

    # Micro-Doppler: unwrapped phase at dominant tap (AN-SCA-14453 §9.5)
    phase_signal = build_phase_signal(preprocessed, frame_mask, selected_tap)
    microdoppler = compute_spectrogram(
        phase_signal,
        frame_rate_hz=preprocessed.frame_rate_hz,
        window_s=preprocessed.config.microdoppler_window_s,
        overlap=preprocessed.config.stft_overlap,
        kind="microdoppler",
        selected_range_m=float(preprocessed.range_axis_m[selected_tap]),
        selected_tap=selected_tap,
        selected_path=preprocessed.selected_path,
    )

    result = DetectionResult(
        label=window.label,
        expected_present=infer_expected_presence(window.label),
        predicted_present=predicted_present,
        presence_score=score_ratio,
        threshold=preprocessed.config.motion_threshold,
        power_mean=power_mean,
        power_p95=power_p95,
        dominant_range_m=dominant_range_m,
        dominant_frequency_hz=dominant_frequency_hz(doppler, preprocessed.config.zero_doppler_hz),
        selected_path=preprocessed.selected_path,
        selected_tap=selected_tap,
        start_s=window.start_s,
        end_s=window.end_s,
        artifact_dir=artifact_dir,
        notes=window.notes,
    )
    return result, {"doppler": doppler, "microdoppler": microdoppler, "frame_mask": frame_mask}
