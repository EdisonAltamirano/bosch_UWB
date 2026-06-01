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
    range_rate_m_s: float | None = None   # phase-Doppler / range-Doppler bin estimate
    azimuth_deg: float | None = None      # MVDR or PDoA AoA estimate; None = not computed


@dataclass(slots=True)
class CfarDetectionConfig:
    num_ref_cells: int = 8
    num_guard_cells: int = 2
    pfa: float = 1e-3
    variant: str = "CA"      # "CA" | "OS" | "GO" | "SO"
    os_rank: int = 6
    cluster_min_gap_taps: int = 2
    max_peaks_per_frame: int = 4


@dataclass
class Track:
    track_id: int
    state: np.ndarray            # 1D: [range_m, range_rate_m_s] | 2D EKF: [x_m, y_m, vx_mps, vy_mps]
    covariance: np.ndarray       # (2,2) or (4,4)
    hit_count: int = 0
    miss_count: int = 0
    confirmed: bool = False
    history_m: list[float] = field(default_factory=list)
    history_t: list[float] = field(default_factory=list)
    trajectory_m: list[float] = field(default_factory=list)
    trajectory_t: list[float] = field(default_factory=list)
    trajectory_observed: list[bool] = field(default_factory=list)
    trajectory_x: list[float] = field(default_factory=list)   # Cartesian lateral (2D EKF only)
    trajectory_y: list[float] = field(default_factory=list)   # Cartesian depth  (2D EKF only)


@dataclass(slots=True)
class TrackingConfig:
    dt_s: float = 1.0 / 20.0
    sigma_process_m: float = 0.05
    sigma_process_v: float = 0.30
    sigma_meas_m: float = 0.15
    sigma_meas_v: float = 0.15
    filter_name: str = "kalman"         # "kalman" | "ekf"
    gate_distance_m: float = 0.75
    gate_chi2: float = 9.21
    confirm_hits: int = 5
    max_misses: int = 5
    init_hits: int = 2
    init_window_frames: int = 3
    use_hungarian: bool = True          # False = legacy greedy nearest-neighbour
    vel_weight: float = 0.0             # >0 adds velocity consistency to gate (tune on two-person bags)
    min_track_duration_s: float = 1.0   # tracks shorter than this skip breathing extraction
    velocity_decay: float = 1.0         # per-frame velocity multiplier during miss periods (<1 prevents prediction drift)
    use_dbscan_init: bool = False        # replace pool-based initiation with DBSCAN clustering on unassigned detections
    use_2d_ekf: bool = False             # Cartesian EKF state [x,y,vx,vy]; requires azimuth_deg in detections
    use_group_association: bool = False  # each track absorbs all gated unassigned dets and updates on avg (paper eq. 15)
    sigma_meas_az_deg: float = 5.0       # azimuth measurement noise (degrees) for 2D EKF


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
    max_rd_fft_frames: int = 256
    rd_hop_frames: int = 0  # 0 = auto (W//4); set explicitly to 1 for frame-exact (slow)
    microdoppler_window_s: float = 1.5
    stft_overlap: float = 0.75
    carrier_frequency_hz: float = 6.5e9
    zero_doppler_hz: float = 0.15
    doppler_limit_hz: float = 6.0
    microdoppler_limit_hz: float = 10.0
    peak_centroid_half_width_m: float = 0.30
    occ_min_duration_s: float = 3.0


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
    peak_range_centroid_m_per_frame: np.ndarray
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


# ---------------------------------------------------------------------------
# Range-Doppler heatmap tracking types
# ---------------------------------------------------------------------------

@dataclass
class RangeDopplerFrame:
    grid: np.ndarray              # (D, T) float32 — STFT magnitude
    doppler_axis_mps: np.ndarray  # (D,) float64
    range_axis_m: np.ndarray      # (T,) float64
    timestamp_s: float
    frame_idx: int


@dataclass
class DopplerCluster:
    range_m: float
    velocity_mps: float
    magnitude: float
    n_points: int


@dataclass
class DopplerTrack:
    track_id: int
    state: np.ndarray              # (2,) [range_m, velocity_mps]
    covariance: np.ndarray         # (2, 2)
    history_range_m: list[float]
    history_velocity_mps: list[float]
    history_t: list[float]
    confirmed: bool
    miss_count: int
