from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


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
    # Peak picking for peak_tracking plot (local maxima on clutter-suppressed |x| in dB).
    peak_magnitude_threshold_db: float = 55.0
    peak_min_separation_taps: int = 2


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
    selected_path: int
    raw_magnitude: np.ndarray
    highpass_complex: np.ndarray
    clutter_removed: np.ndarray
    variance_per_tap: np.ndarray
    dominant_tap: int
    significant_taps: np.ndarray
    significant_ranges_m: np.ndarray


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
