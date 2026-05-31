from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import lfilter

# Ch9, fc = 7987.2 MHz
LAMBDA_M: float = 0.0375
BREATHING_BAND_HZ: tuple[float, float] = (0.1, 0.8)
DEFAULT_NFFT: int = 1024
DEFAULT_HP_CUTOFF_HZ: float = 0.05


@dataclass(slots=True)
class BreathingResult:
    tap_idx: int
    is_static: bool
    freq_hz: float | None
    freq_bpm: float | None
    snr_db: float | None
    phase_arc_deg: float | None
    phase_hp: np.ndarray
    fft_freqs_hz: np.ndarray
    fft_magnitude: np.ndarray


def is_target_static(peak_tap_series: np.ndarray, max_drift_taps: int = 1) -> bool:
    """True if tap range ≤ max_drift_taps over the extraction window (guide §4.1)."""
    if peak_tap_series.size < 2:
        return True
    return bool(int(peak_tap_series.max()) - int(peak_tap_series.min()) <= max_drift_taps)


def _iir_highpass(signal: np.ndarray, frame_rate_hz: float, cutoff_hz: float) -> np.ndarray:
    """First-order IIR high-pass on a 1-D real array (guide §4.4; AN-SCA-14453 §5.3)."""
    alpha = float(frame_rate_hz) / (float(frame_rate_hz) + 2.0 * np.pi * cutoff_hz)
    b = np.array([alpha, -alpha])
    a = np.array([1.0, -alpha])
    return lfilter(b, a, signal.astype(np.float64)).astype(np.float64)


def extract_breathing_phase(
    highpass_complex: np.ndarray,
    frame_mask: np.ndarray,
    tap_idx: int,
    frame_rate_hz: float,
    hp_cutoff_hz: float = DEFAULT_HP_CUTOFF_HZ,
) -> np.ndarray:
    """Steps 2–4 of guide §4: extract complex at tap → unwrap phase → IIR HP filter."""
    signal = highpass_complex[frame_mask, tap_idx].astype(np.complex128)
    unwrapped = np.unwrap(np.angle(signal))
    return _iir_highpass(unwrapped, frame_rate_hz, hp_cutoff_hz)


def estimate_breathing_rate(
    phase_hp: np.ndarray,
    frame_rate_hz: float,
    nfft: int = DEFAULT_NFFT,
    search_band_hz: tuple[float, float] = BREATHING_BAND_HZ,
) -> tuple[float | None, float | None, float | None]:
    """Steps 5–6 of guide §4: FFT → dominant peak in search band → SNR.

    Returns (freq_hz, freq_bpm, snr_db). All None if data is too short or no peak
    found in the breathing band.
    """
    N = len(phase_hp)
    if N < 32:
        return None, None, None

    n_fft = max(N, nfft)
    window = np.hanning(N)
    padded = np.zeros(n_fft)
    padded[:N] = phase_hp * window
    spectrum = np.abs(np.fft.rfft(padded, n=n_fft)) ** 2
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / frame_rate_hz)

    lo, hi = search_band_hz
    band = (freqs >= lo) & (freqs <= hi)
    if not np.any(band):
        return None, None, None

    peak_local = int(np.argmax(spectrum[band]))
    freq_hz = float(freqs[band][peak_local])
    freq_bpm = freq_hz * 60.0

    # SNR: peak power vs. median noise outside the breathing band (guide Eq. snr_breath)
    noise_mask = (freqs < lo) | (freqs > hi)
    noise_floor = (float(np.median(spectrum[noise_mask])) if np.any(noise_mask) else 1.0) + 1e-12
    snr_db = float(10.0 * np.log10(spectrum[band][peak_local] / noise_floor + 1e-12))

    return freq_hz, freq_bpm, snr_db


def compute_phase_arc_deg(phase_hp: np.ndarray) -> float:
    """Half-peak-to-peak phase excursion in degrees (guide Eq. arc: Φ = 4π m_b / λ)."""
    return float(np.degrees(np.ptp(phase_hp) / 2.0))


def run_breathing_extraction(
    highpass_complex: np.ndarray,
    peak_tap_per_frame: np.ndarray,
    frame_mask: np.ndarray,
    tap_idx: int,
    frame_rate_hz: float,
    nfft: int = DEFAULT_NFFT,
    hp_cutoff_hz: float = DEFAULT_HP_CUTOFF_HZ,
    search_band_hz: tuple[float, float] = BREATHING_BAND_HZ,
) -> BreathingResult:
    """Full 6-step breathing pipeline from guide §4.

    Parameters
    ----------
    highpass_complex : (N_frames, N_taps) complex array after clutter removal
    peak_tap_per_frame : (N_frames,) int array of dominant tap per frame
    frame_mask : bool mask selecting the extraction window
    tap_idx : target tap to extract phase from (from confirmed track or dominant tap)
    frame_rate_hz : measured frame rate (from timestamps, not hardcoded)
    """
    windowed_taps = peak_tap_per_frame[frame_mask]
    static = is_target_static(windowed_taps)

    phase_hp = extract_breathing_phase(highpass_complex, frame_mask, tap_idx, frame_rate_hz, hp_cutoff_hz)
    freq_hz, freq_bpm, snr_db = estimate_breathing_rate(phase_hp, frame_rate_hz, nfft, search_band_hz)

    N = len(phase_hp)
    if N >= 32:
        n_fft_actual = max(N, nfft)
        window = np.hanning(N)
        padded = np.zeros(n_fft_actual)
        padded[:N] = phase_hp * window
        fft_mag = np.abs(np.fft.rfft(padded, n=n_fft_actual)).astype(np.float32)
        fft_freqs = np.fft.rfftfreq(n_fft_actual, d=1.0 / frame_rate_hz).astype(np.float32)
    else:
        fft_freqs = np.zeros(1, dtype=np.float32)
        fft_mag = np.zeros(1, dtype=np.float32)

    arc_deg = compute_phase_arc_deg(phase_hp) if N >= 2 else None

    return BreathingResult(
        tap_idx=tap_idx,
        is_static=static,
        freq_hz=freq_hz,
        freq_bpm=freq_bpm,
        snr_db=snr_db,
        phase_arc_deg=arc_deg,
        phase_hp=phase_hp.astype(np.float32),
        fft_freqs_hz=fft_freqs,
        fft_magnitude=fft_mag,
    )
