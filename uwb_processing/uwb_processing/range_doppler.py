from __future__ import annotations

import numpy as np

from .cfar import cfar_1d, extract_peaks
from .types import CfarDetectionConfig, PreprocessedSession, RangeDopplerFrame

LAMBDA_M = 0.0499  # 6 GHz carrier wavelength in metres


def compute_range_doppler_frames(
    session: PreprocessedSession,
    n_fft: int = 32,
    hop: int = 16,
    window: str = "hann",
) -> list[RangeDopplerFrame]:
    """Slide an STFT window over highpass_complex and return one RangeDopplerFrame per hop.

    n_fft must be a power of 2.  Frame rate is derived from timestamps — never assumed fixed.
    """
    assert n_fft > 0 and (n_fft & (n_fft - 1)) == 0, "n_fft must be a power of 2"

    data = session.highpass_complex          # (F, T) complex64
    timestamps = session.session.timestamps_s
    range_axis = session.range_axis_m.copy()

    F, T = data.shape
    if F < n_fft:
        return []

    fps = 1.0 / float(np.mean(np.diff(timestamps))) if len(timestamps) > 1 else 20.0

    freqs_hz = np.fft.fftshift(np.fft.fftfreq(n_fft, d=1.0 / fps))
    doppler_axis_mps = (freqs_hz * LAMBDA_M / 2.0).astype(np.float64)

    if window == "hann":
        win = np.hanning(n_fft).astype(np.float32)
    else:
        win = np.ones(n_fft, dtype=np.float32)

    frames: list[RangeDopplerFrame] = []
    start = 0
    frame_idx = 0
    while start + n_fft <= F:
        segment = data[start : start + n_fft].astype(np.complex64)  # (n_fft, T)
        windowed = segment * win[:, np.newaxis]
        spectrum = np.fft.fft(windowed, axis=0)
        magnitude = np.abs(np.fft.fftshift(spectrum, axes=0)).astype(np.float32)

        ts = float(np.median(timestamps[start : start + n_fft]))
        frames.append(RangeDopplerFrame(
            grid=magnitude,
            doppler_axis_mps=doppler_axis_mps.copy(),
            range_axis_m=range_axis,
            timestamp_s=ts,
            frame_idx=frame_idx,
        ))
        start += hop
        frame_idx += 1

    return frames


def detect_on_rd_frame(
    rd_frame: RangeDopplerFrame,
    cfar_cfg: CfarDetectionConfig | None = None,
    zero_doppler_thresh_mps: float = 0.08,
) -> list[tuple[float, float, float]]:
    """Return (range_m, velocity_mps, magnitude) detections from a single RD frame.

    1D CA-CFAR is applied along each Doppler row.  Bins with |velocity| below
    zero_doppler_thresh_mps are skipped to suppress static clutter.
    """
    if cfar_cfg is None:
        cfar_cfg = CfarDetectionConfig(
            num_ref_cells=4,
            num_guard_cells=1,
            pfa=1e-2,
            variant="CA",
            max_peaks_per_frame=6,
        )

    grid = rd_frame.grid                  # (D, T) float32
    doppler_axis = rd_frame.doppler_axis_mps
    range_axis = rd_frame.range_axis_m
    D = grid.shape[0]

    detections: list[tuple[float, float, float]] = []
    for d in range(D):
        vel = float(doppler_axis[d])
        if abs(vel) < zero_doppler_thresh_mps:
            continue

        profile = grid[d].astype(np.float64)
        det_mask, thr_profile = cfar_1d(
            profile,
            num_ref=cfar_cfg.num_ref_cells,
            num_guard=cfar_cfg.num_guard_cells,
            pfa=cfar_cfg.pfa,
            variant=cfar_cfg.variant,
        )
        peaks = extract_peaks(
            detection_mask=det_mask,
            profile=profile,
            range_axis_m=range_axis,
            frame_idx=d,
            threshold_profile=thr_profile,
            cluster_min_gap_taps=cfar_cfg.cluster_min_gap_taps,
            max_peaks=cfar_cfg.max_peaks_per_frame,
        )
        for peak in peaks:
            detections.append((peak.range_m, vel, peak.magnitude))

    return detections
