from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SPEED_OF_LIGHT_M_S = 299_792_458.0


@dataclass(slots=True)
class PairedRxFrameSet:
    rx1: np.ndarray
    rx2: np.ndarray
    rx_ids: np.ndarray


@dataclass(slots=True)
class TrackAoA:
    track_id: int
    angle_deg: float
    range_m: float
    frame_idx: int


def _nearest_detection_tap_idx(detections, target_range_m: float, default_tap_idx: int = 0) -> int:
    if not detections:
        return int(default_tap_idx)
    nearest = min(detections, key=lambda detection: abs(float(detection.range_m) - target_range_m))
    return int(nearest.tap_idx)


def pair_rx_channels(session, expected_rx_ids: tuple[int, int] = (1, 2)) -> PairedRxFrameSet:
    if session.frames.shape[1] < 2:
        raise ValueError("AoA requires at least two RX paths per logical frame.")
    rx_ids = np.asarray(session.rx_antenna_ids[:2], dtype=np.uint8)
    if tuple(int(x) for x in rx_ids.tolist()) != tuple(expected_rx_ids):
        raise ValueError(f"Expected RX ids {expected_rx_ids}, got {tuple(int(x) for x in rx_ids.tolist())}.")
    return PairedRxFrameSet(
        rx1=session.frames[:, 0, :].astype(np.complex64),
        rx2=session.frames[:, 1, :].astype(np.complex64),
        rx_ids=rx_ids,
    )


def estimate_angle_deg_from_pair(
    rx1_value: complex,
    rx2_value: complex,
    carrier_frequency_hz: float,
    rx_spacing_m: float,
    fov_limit_deg: float,
    previous_angle_deg: float | None,
) -> float:
    wavelength_m = SPEED_OF_LIGHT_M_S / float(carrier_frequency_hz)
    phase_difference_wrapped = float(np.angle(rx2_value * np.conj(rx1_value)))
    scale = wavelength_m / (2.0 * np.pi * float(rx_spacing_m))

    candidates_deg: list[float] = []
    for branch_index in range(-3, 4):
        phase_difference = phase_difference_wrapped + 2.0 * np.pi * branch_index
        sine_theta = phase_difference * scale
        if abs(sine_theta) > 1.0:
            continue
        angle_deg = float(np.degrees(np.arcsin(sine_theta)))
        if -float(fov_limit_deg) <= angle_deg <= float(fov_limit_deg):
            candidates_deg.append(angle_deg)

    if not candidates_deg:
        sine_theta = float(np.clip(phase_difference_wrapped * scale, -1.0, 1.0))
        return float(np.clip(np.degrees(np.arcsin(sine_theta)), -float(fov_limit_deg), float(fov_limit_deg)))

    if previous_angle_deg is not None:
        return float(min(candidates_deg, key=lambda candidate: abs(candidate - previous_angle_deg)))
    return float(min(candidates_deg, key=lambda candidate: abs(candidate)))


def estimate_track_aoa_session(
    paired: PairedRxFrameSet,
    detections_per_frame,
    tracks_per_frame,
    carrier_frequency_hz: float,
    rx_spacing_m: float,
    fov_limit_deg: float,
) -> list[list[TrackAoA]]:
    all_track_aoas: list[list[TrackAoA]] = []
    previous_angles_deg: dict[int, float] = {}

    for frame_idx, tracks in enumerate(tracks_per_frame):
        frame_track_aoas: list[TrackAoA] = []
        detections = detections_per_frame[frame_idx] if frame_idx < len(detections_per_frame) else []

        for track in tracks:
            tap_idx = _nearest_detection_tap_idx(detections, target_range_m=float(track.state[0]), default_tap_idx=0)
            angle_deg = estimate_angle_deg_from_pair(
                paired.rx1[frame_idx, tap_idx],
                paired.rx2[frame_idx, tap_idx],
                carrier_frequency_hz=carrier_frequency_hz,
                rx_spacing_m=rx_spacing_m,
                fov_limit_deg=fov_limit_deg,
                previous_angle_deg=previous_angles_deg.get(track.track_id),
            )
            previous_angles_deg[track.track_id] = angle_deg
            frame_track_aoas.append(
                TrackAoA(
                    track_id=track.track_id,
                    angle_deg=angle_deg,
                    range_m=float(track.state[0]),
                    frame_idx=frame_idx,
                )
            )

        if not frame_track_aoas:
            for detection_index, detection in enumerate(detections):
                angle_deg = estimate_angle_deg_from_pair(
                    paired.rx1[frame_idx, detection.tap_idx],
                    paired.rx2[frame_idx, detection.tap_idx],
                    carrier_frequency_hz=carrier_frequency_hz,
                    rx_spacing_m=rx_spacing_m,
                    fov_limit_deg=fov_limit_deg,
                    previous_angle_deg=previous_angles_deg.get(detection_index),
                )
                previous_angles_deg[detection_index] = angle_deg
                frame_track_aoas.append(
                    TrackAoA(
                        track_id=detection_index,
                        angle_deg=angle_deg,
                        range_m=float(detection.range_m),
                        frame_idx=frame_idx,
                    )
                )
        all_track_aoas.append(frame_track_aoas)

    return all_track_aoas


def build_range_angle_map(
    paired: PairedRxFrameSet,
    range_axis_m: np.ndarray,
    frame_idx: int,
    carrier_frequency_hz: float,
    rx_spacing_m: float,
    fov_limit_deg: float,
    angle_bins: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    angle_axis_deg = np.linspace(-fov_limit_deg, fov_limit_deg, angle_bins, dtype=np.float32)
    range_axis_arr = np.asarray(range_axis_m, dtype=np.float32)
    grid = np.zeros((range_axis_arr.size, angle_bins), dtype=np.float32)

    for tap_idx in range(range_axis_arr.size):
        angle_deg = estimate_angle_deg_from_pair(
            paired.rx1[frame_idx, tap_idx],
            paired.rx2[frame_idx, tap_idx],
            carrier_frequency_hz=carrier_frequency_hz,
            rx_spacing_m=rx_spacing_m,
            fov_limit_deg=fov_limit_deg,
            previous_angle_deg=None,
        )
        angle_bin = int(np.argmin(np.abs(angle_axis_deg - angle_deg)))
        grid[tap_idx, angle_bin] = float(
            np.abs(paired.rx1[frame_idx, tap_idx]) + np.abs(paired.rx2[frame_idx, tap_idx])
        )

    return grid, angle_axis_deg, range_axis_arr
