from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(slots=True)
class CfarDetection:
    frame_idx: int
    tap_idx: int
    range_m: float
    magnitude: float
    threshold: float


@dataclass(slots=True)
class CfarDetectionConfig:
    num_ref_cells: int = 8
    num_guard_cells: int = 2
    pfa: float = 1e-3
    variant: str = "CA"      # "CA" | "OS" | "GO"
    os_rank: int = 6
    cluster_min_gap_taps: int = 2
    max_peaks_per_frame: int = 4


@dataclass
class Track:
    track_id: int
    state: np.ndarray            # [range_m, range_rate_m_s]
    covariance: np.ndarray       # (2, 2)
    hit_count: int = 0
    miss_count: int = 0
    confirmed: bool = False
    history_m: list[float] = field(default_factory=list)
    history_t: list[float] = field(default_factory=list)


@dataclass(slots=True)
class TrackingConfig:
    dt_s: float = 1.0 / 20.0
    sigma_process_m: float = 0.05
    sigma_process_v: float = 0.30
    sigma_meas_m: float = 0.15
    gate_distance_m: float = 0.75
    confirm_hits: int = 5
    max_misses: int = 5
    init_hits: int = 2
    init_window_frames: int = 3


@dataclass(slots=True)
class AnnotationWindow:
    label: str
    start_s: float
    end_s: float
    wall_present: bool = True
    expected_range_m: tuple[float, float] | None = None
    notes: str = ""


@dataclass(slots=True)
class SessionAnnotation:
    session_id: str
    source: str
    wall_description: str = ""
    notes: str = ""
    windows: list[AnnotationWindow] = field(default_factory=list)


@dataclass(slots=True)
class DetectionConfig:
    range_resolution_m: float = 0.30
    tap_spacing_m: float = 0.15
    # Tap index of the true 0-m point in the CIR window.  For the NCJ29D6 the
    # chip's hardware delay is stripped by the UCI layer before delivery, so the
    # window starts at range ≈ 0 m and this should remain 0.  Only change it if
    # a calibration measurement confirms a residual system delay.
    spillover_tap_0idx: int = 0
    default_range_gate_m: tuple[float, float] = (0.3, 6.0)
    wall_clip_m: float = 0.3
    offline_rds_enabled: bool = False
    rds_cutoff_hz: float = 0.2
    motion_threshold: float = 3.0
    doppler_window_s: float = 4.0
    microdoppler_window_s: float = 1.5
    stft_overlap: float = 0.75
    carrier_frequency_hz: float = 6.5e9
    zero_doppler_hz: float = 0.15
    doppler_limit_hz: float = 6.0
    microdoppler_limit_hz: float = 10.0
    # --- breathing (vital-sign) analysis ---
    # Physiological breathing band: ~6-36 breaths/min = 0.1-0.6 Hz.
    breathing_band_hz: tuple[float, float] = (0.1, 0.6)
    # Upper limit of the breathing-rate display axis (heart rate would extend
    # this, but breathing alone only needs ~1 Hz).
    breathing_display_max_hz: float = 1.0
    # Range gate to search for a breathing subject (close range, indoors).
    breathing_range_gate_m: tuple[float, float] = (1.0, 3.0)


@dataclass(slots=True)
class RadarSession:
    source_path: Path
    source_kind: str
    frames: np.ndarray
    timestamps_s: np.ndarray
    rx_antenna_ids: np.ndarray
    tx_antenna_ids: np.ndarray
    cir_start_offsets: np.ndarray
    bytes_per_tap: int
    block_size: int
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def session_id(self) -> str:
        return self.source_path.name


@dataclass(slots=True)
class PreprocessedSession:
    session: RadarSession
    config: DetectionConfig
    frame_rate_hz: float
    range_axis_m: np.ndarray
    roi_mask: np.ndarray
    selected_path: int
    raw_magnitude: np.ndarray
    highpass_complex: np.ndarray
    clutter_removed: np.ndarray
    power_per_tap: np.ndarray
    dominant_tap: int
    background_power_reference: float
    presence_threshold: float
    roi_to_background_power_ratio: np.ndarray
    presence_mask: np.ndarray
    peak_tap_per_frame: np.ndarray
    peak_range_m_per_frame: np.ndarray
    smoothed_peak_range_m: np.ndarray

    @property
    def variance_per_tap(self) -> np.ndarray:
        return self.power_per_tap

    @property
    def presence_score_per_frame(self) -> np.ndarray:
        return self.roi_to_background_power_ratio


@dataclass(slots=True)
class SpectrogramResult:
    kind: str
    frequencies_hz: np.ndarray
    times_s: np.ndarray
    magnitude_db: np.ndarray
    selected_range_m: float
    selected_tap: int
    selected_path: int


@dataclass(slots=True)
class BreathingResult:
    """Range x breathing-rate spectral map and the extracted vital estimate.

    The map is the slow-time FFT of the clutter-suppressed complex CIR at every
    range bin, with the frequency axis expressed in breaths-per-minute.  A
    breathing subject appears as a bright spot at their range and rate.
    """

    range_axis_m: np.ndarray          # (R,) valid range bins (X axis)
    rate_bpm: np.ndarray              # (F,) breathing-rate axis in breaths/min (Y axis)
    power_db: np.ndarray              # (F, R) magnitude map in dB
    best_range_m: float               # range of the strongest in-band peak
    best_rate_bpm: float              # breathing rate at that peak (breaths/min)
    best_rate_hz: float               # same rate expressed in Hz
    snr_db: float                     # in-band peak vs out-of-band median (dB)
    frame_rate_hz: float
    selected_path: int
    band_hz: tuple[float, float]

    def to_dict(self) -> dict[str, Any]:
        return {
            "best_range_m": self.best_range_m,
            "best_rate_bpm": self.best_rate_bpm,
            "best_rate_hz": self.best_rate_hz,
            "snr_db": self.snr_db,
            "frame_rate_hz": self.frame_rate_hz,
            "selected_path": self.selected_path,
            "band_hz": list(self.band_hz),
        }


@dataclass(slots=True)
class DetectionResult:
    label: str
    expected_present: bool | None
    predicted_present: bool
    presence_score: float
    threshold: float
    power_mean: float
    power_p95: float
    dominant_range_m: float
    dominant_frequency_hz: float | None
    selected_path: int
    selected_tap: int
    start_s: float
    end_s: float
    artifact_dir: Path
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "expected_present": self.expected_present,
            "predicted_present": self.predicted_present,
            "presence_score": self.presence_score,
            "threshold": self.threshold,
            "power_mean": self.power_mean,
            "power_p95": self.power_p95,
            "dominant_range_m": self.dominant_range_m,
            "dominant_frequency_hz": self.dominant_frequency_hz,
            "selected_path": self.selected_path,
            "selected_tap": self.selected_tap,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "artifact_dir": str(self.artifact_dir),
            "notes": self.notes,
        }
