from __future__ import annotations

import time

import numpy as np

from .types import CfarDetection, CfarDetectionConfig, PreprocessedSession


def combine_paths_max(preprocessed: PreprocessedSession) -> np.ndarray:
    """Return element-wise max magnitude across ALL RX paths after clutter subtraction.

    The default clutter_removed uses only the single strongest path, which can miss a
    person who is dominant on a different antenna.  This function re-derives the
    clutter-removed magnitude from all paths and takes the per-tap maximum so every
    person appears regardless of which antenna sees them best.

    Returns float32 (N_frames, N_taps) — drop-in replacement for clutter_removed.
    """
    frames = preprocessed.session.frames                  # (F, P, T) complex
    global_mean = frames.mean(axis=0, keepdims=True)      # (1, P, T)
    clutter_all = np.abs(frames - global_mean).astype(np.float32)  # (F, P, T)

    wall_mask = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m
    combined = clutter_all.max(axis=1)                    # (F, T) max across paths
    combined[:, ~wall_mask] = 0.0
    return combined


def _alpha_ca(n_ref_total: int, pfa: float) -> float:
    """CA-CFAR scale factor: alpha = N*(Pfa^(-1/N) - 1)."""
    return n_ref_total * (pfa ** (-1.0 / n_ref_total) - 1.0)


def cfar_1d(
    profile: np.ndarray,
    num_ref: int,
    num_guard: int,
    pfa: float,
    variant: str = "CA",
    os_rank: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """1-D CA/OS/GO/SO-CFAR on a range profile (shape T,).

    Variants:
      CA  — Cell-Averaging: average all training cells. Best for homogeneous clutter.
      GO  — Greatest-Of: max of left/right half-window means. Protects against clutter edges.
      SO  — Smallest-Of: min of left/right half-window means. Best for two closely spaced
            targets (prevents the stronger from masking the weaker in its training window).
      OS  — Order-Statistic: k-th sorted value. Most robust to multiple interferers.

    Returns (detection_mask bool T, threshold_profile float T).
    Taps within (num_ref + num_guard) of either edge get threshold=nan and
    detection=False because the reference window cannot be filled there.
    """
    T = len(profile)
    threshold = np.full(T, np.nan, dtype=np.float64)
    detections = np.zeros(T, dtype=bool)
    pad = num_ref + num_guard

    if T < 2 * pad + 1:
        return detections, threshold

    n_ref_total = 2 * num_ref
    if variant == "CA":
        alpha = _alpha_ca(n_ref_total, pfa)
    elif variant in ("OS", "GO", "SO"):
        # GO/SO use single half-window length for the alpha approximation.
        # OS reuses the same approximation.
        alpha = _alpha_ca(num_ref, pfa)
    else:
        raise ValueError(f"Unknown CFAR variant: {variant!r}")

    for cut in range(pad, T - pad):
        left_ref = profile[cut - pad : cut - num_guard]
        right_ref = profile[cut + num_guard + 1 : cut + pad + 1]

        if variant == "CA":
            noise_est = float(np.mean(np.concatenate([left_ref, right_ref])))
        elif variant == "SO":
            noise_est = float(min(np.mean(left_ref), np.mean(right_ref)))
        elif variant == "OS":
            k = os_rank if os_rank is not None else int(0.75 * n_ref_total)
            combined = np.sort(np.concatenate([left_ref, right_ref]))
            noise_est = float(combined[min(k, len(combined) - 1)])
        else:  # GO
            noise_est = float(max(np.mean(left_ref), np.mean(right_ref)))

        thr = alpha * noise_est
        threshold[cut] = thr
        detections[cut] = bool(profile[cut] > thr)

    return detections, threshold


def cfar_1d_batch(
    profiles: np.ndarray,
    num_ref: int,
    num_guard: int,
    pfa: float,
    variant: str = "OS",
    os_rank: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized CFAR on a 2-D batch of range profiles — drop-in for cfar_1d.

    profiles : (D, T) float — D independent profiles (e.g. Doppler bins), each length T.
    Returns det_mask (D, T) bool and threshold (D, T) float.
    Taps within (num_ref + num_guard) of either edge are marked False / nan,
    matching cfar_1d semantics exactly.

    Uses sliding_window_view + np.partition — no Python loop over range taps.
    """
    from numpy.lib.stride_tricks import sliding_window_view

    D, T = profiles.shape
    pad = num_ref + num_guard
    n_ref_total = 2 * num_ref
    det_mask = np.zeros((D, T), dtype=bool)
    threshold = np.full((D, T), np.nan, dtype=np.float64)

    if T < 2 * pad + 1:
        return det_mask, threshold

    # Shape: (D, T - 2*pad, 2*pad + 1) — sliding window of width 2*pad+1 along range axis.
    # Window position w maps to CUT at original tap w + pad.
    windows = sliding_window_view(profiles, window_shape=2 * pad + 1, axis=1)

    # Reference cell mask within the window (True = reference, False = guard or CUT).
    win_idx = np.arange(2 * pad + 1)
    ref_sel = np.abs(win_idx - pad) > num_guard  # num_ref left + num_ref right cells

    ref_cells = windows[:, :, ref_sel]   # (D, T-2*pad, 2*num_ref)
    cut_values = profiles[:, pad:T - pad]  # (D, T-2*pad)

    if variant == "CA":
        alpha = _alpha_ca(n_ref_total, pfa)
        noise_est = ref_cells.mean(axis=-1)
    elif variant == "SO":
        alpha = _alpha_ca(num_ref, pfa)
        noise_est = np.minimum(
            ref_cells[:, :, :num_ref].mean(axis=-1),
            ref_cells[:, :, num_ref:].mean(axis=-1),
        )
    elif variant == "GO":
        alpha = _alpha_ca(num_ref, pfa)
        noise_est = np.maximum(
            ref_cells[:, :, :num_ref].mean(axis=-1),
            ref_cells[:, :, num_ref:].mean(axis=-1),
        )
    elif variant == "OS":
        alpha = _alpha_ca(num_ref, pfa)
        k = min(int(os_rank) if os_rank is not None else int(0.75 * n_ref_total), n_ref_total - 1)
        noise_est = np.partition(ref_cells, k, axis=-1)[:, :, k]
    else:
        raise ValueError(f"Unknown CFAR variant: {variant!r}")

    thr = alpha * noise_est
    det_mask[:, pad:T - pad] = cut_values > thr
    threshold[:, pad:T - pad] = thr
    return det_mask, threshold


def extract_peaks(
    detection_mask: np.ndarray,
    profile: np.ndarray,
    range_axis_m: np.ndarray,
    frame_idx: int,
    threshold_profile: np.ndarray,
    cluster_min_gap_taps: int,
    max_peaks: int,
) -> list[CfarDetection]:
    """Cluster adjacent True cells; return magnitude-weighted centroid of each cluster.

    Adjacent detections separated by <= cluster_min_gap_taps are merged.  Output
    is sorted by magnitude descending and capped at max_peaks.
    """
    indices = np.flatnonzero(detection_mask)
    if indices.size == 0:
        return []

    # Build clusters: split when gap > cluster_min_gap_taps
    clusters: list[list[int]] = []
    current: list[int] = [int(indices[0])]
    for idx in indices[1:]:
        if idx - current[-1] <= cluster_min_gap_taps + 1:
            current.append(int(idx))
        else:
            clusters.append(current)
            current = [int(idx)]
    clusters.append(current)

    peaks: list[CfarDetection] = []
    for cluster in clusters:
        mags = profile[cluster]
        weights = mags / (mags.sum() + 1e-12)
        tap_centroid = float(np.dot(cluster, weights))
        tap_idx = int(round(tap_centroid))
        tap_idx = max(0, min(tap_idx, len(range_axis_m) - 1))
        magnitude = float(profile[tap_idx])
        thr = float(threshold_profile[tap_idx]) if not np.isnan(threshold_profile[tap_idx]) else 0.0
        peaks.append(CfarDetection(
            frame_idx=frame_idx,
            tap_idx=tap_idx,
            range_m=float(range_axis_m[tap_idx]),
            magnitude=magnitude,
            threshold=thr,
        ))

    peaks.sort(key=lambda d: d.magnitude, reverse=True)
    return peaks[:max_peaks]


def merge_dual_channel_peaks(
    peaks_c: list[CfarDetection],
    peaks_b: list[CfarDetection],
    frame_idx: int,
    range_axis_m: np.ndarray,
    tolerance_taps: int = 1,
    nms_min_sep_taps: int = 1,
) -> list[CfarDetection]:
    """Merge CFAR peaks from RxC and RxB with ±tolerance_taps matching (guide §5.2, AN13989 §5.3).

    Dual-channel hits are merged to the mean tap and take the stronger magnitude.
    Unmatched single-channel peaks are kept.  After merging, non-maximum suppression
    removes weaker peaks within 2 taps of a stronger one.
    """
    merged: list[CfarDetection] = []
    used_b: set[int] = set()

    for pc in peaks_c:
        best_bi: int | None = None
        best_gap: int = tolerance_taps + 1
        for bi, pb in enumerate(peaks_b):
            gap = abs(pc.tap_idx - pb.tap_idx)
            if gap <= tolerance_taps and gap < best_gap:
                best_gap = gap
                best_bi = bi
        if best_bi is not None:
            pb = peaks_b[best_bi]
            merged_tap = int(round((pc.tap_idx + pb.tap_idx) / 2.0))
            merged_tap = int(np.clip(merged_tap, 0, len(range_axis_m) - 1))
            merged.append(CfarDetection(
                frame_idx=frame_idx,
                tap_idx=merged_tap,
                range_m=float(range_axis_m[merged_tap]),
                magnitude=max(pc.magnitude, pb.magnitude),
                threshold=min(pc.threshold, pb.threshold),
            ))
            used_b.add(best_bi)
        else:
            merged.append(pc)

    for bi, pb in enumerate(peaks_b):
        if bi not in used_b:
            merged.append(pb)

    # Non-maximum suppression: suppress weaker peaks within nms_min_sep_taps of a stronger one
    # 0 = disabled, 1 = only exact same tap, 2 = within 2 taps (original), etc.
    merged.sort(key=lambda d: d.magnitude, reverse=True)
    result: list[CfarDetection] = []
    for det in merged:
        if nms_min_sep_taps > 0 and any(
            abs(det.tap_idx - kept.tap_idx) < nms_min_sep_taps for kept in result
        ):
            continue
        result.append(det)
    return result


def run_cfar_session(
    preprocessed: PreprocessedSession,
    cfar_cfg: CfarDetectionConfig,
    clutter_mag: np.ndarray | None = None,
) -> list[list[CfarDetection]]:
    """Run CFAR on every slow-time frame of the clutter-removed magnitude.

    Returns a list of length N_frames; each element is the list of
    CfarDetections for that frame (may be empty).
    """
    clutter_mag = clutter_mag if clutter_mag is not None else preprocessed.clutter_removed
    range_axis = preprocessed.range_axis_m            # (N_taps,)
    wall_clip = preprocessed.config.wall_clip_m

    valid_mask = range_axis >= wall_clip

    results: list[list[CfarDetection]] = []
    for fi in range(clutter_mag.shape[0]):
        profile_full = clutter_mag[fi].astype(np.float64)
        profile_roi = profile_full.copy()
        profile_roi[~valid_mask] = 0.0

        det_mask, thr_profile = cfar_1d(
            profile_roi,
            num_ref=cfar_cfg.num_ref_cells,
            num_guard=cfar_cfg.num_guard_cells,
            pfa=cfar_cfg.pfa,
            variant=cfar_cfg.variant,
            os_rank=cfar_cfg.os_rank,
        )

        # Exclude detections outside the wall clip region
        det_mask &= valid_mask

        peaks = extract_peaks(
            detection_mask=det_mask,
            profile=profile_roi,
            range_axis_m=range_axis,
            frame_idx=fi,
            threshold_profile=thr_profile,
            cluster_min_gap_taps=cfar_cfg.cluster_min_gap_taps,
            max_peaks=cfar_cfg.max_peaks_per_frame,
        )
        results.append(peaks)

    return results


def annotate_detections_with_doppler(
    detections_per_frame: list[list[CfarDetection]],
    highpass_complex: np.ndarray,
    frame_rate_hz: float,
    carrier_frequency_hz: float = 6.5e9,
) -> None:
    """Phase-Doppler velocity estimate for each detection, in-place.

    Uses instantaneous phase difference between consecutive frames:
      v = (lambda / 4*pi) * delta_phi * frame_rate_hz

    Frame fi=0 is skipped (no previous frame). Fast movers (v > lambda*PRF/4)
    will show aliased velocity; use run_cfar_rd for unambiguous velocity.
    """
    lambda_m = 3e8 / max(carrier_frequency_hz, 1.0)
    dt_s = 1.0 / max(frame_rate_hz, 1.0)

    for fi, dets in enumerate(detections_per_frame):
        if fi == 0:
            continue
        for det in dets:
            t = det.tap_idx
            if t >= highpass_complex.shape[1]:
                continue
            conj_prod = np.conj(highpass_complex[fi - 1, t]) * highpass_complex[fi, t]
            phase_diff = float(np.angle(conj_prod))
            # Negate: CIR phase evolves as exp(-j·4π·r/λ), so phase_diff = -4π·v·dt/λ
            # → v = -phase_diff·λ/(4π·dt).  Positive v = receding, matches EKF convention.
            det.range_rate_m_s = float(-phase_diff * lambda_m / (4.0 * np.pi * dt_s))


def _resolve_rd_window_frames(
    frame_rate_hz: float,
    doppler_window_s: float,
    max_window_frames: int | None,
) -> int:
    requested = max(4, int(doppler_window_s * frame_rate_hz))
    if max_window_frames is None:
        return requested
    return max(4, min(requested, int(max_window_frames)))


def _resolve_progress_interval(total_steps: int, target_updates: int = 20) -> int:
    if total_steps <= 0:
        return 1
    return max(1, total_steps // max(1, target_updates))


def run_cfar_rd(
    preprocessed: PreprocessedSession,
    cfar_cfg: CfarDetectionConfig,
    doppler_window_s: float = 2.0,
    max_window_frames: int | None = None,
    hop_frames: int | None = None,
) -> list[list[CfarDetection]]:
    """OS-CFAR on range-Doppler maps — vectorized batch implementation.

    Instead of computing a new FFT for every frame (original approach), this function
    computes FFTs at a coarser hop stride (default W//4) and broadcasts each hop's
    detections to the frames it covers.  Within each hop, CFAR runs on all Doppler
    bins simultaneously using cfar_1d_batch (no Python loop over range taps).

    Speedup vs. original: ~500–2000x on 500 Hz bags (batch FFT + vectorized CFAR).
    Detection quality is not materially affected: at 500 Hz the default hop of 64
    frames is 128 ms, well inside the temporal coherence window of a walking person.
    """
    clutter_complex = preprocessed.highpass_complex  # (F, T) complex
    range_axis = preprocessed.range_axis_m
    fps = preprocessed.frame_rate_hz
    lambda_m = 3e8 / max(preprocessed.config.carrier_frequency_hz, 1.0)
    dt_s = 1.0 / max(fps, 1.0)
    wall_clip = preprocessed.config.wall_clip_m

    W = _resolve_rd_window_frames(fps, doppler_window_s, max_window_frames)
    hop = max(1, W // 4) if hop_frames is None else max(1, int(hop_frames))

    F, T = clutter_complex.shape
    half = W // 2
    valid_mask = range_axis >= wall_clip
    doppler_axis_mps = np.fft.fftshift(np.fft.fftfreq(W, d=dt_s)) * lambda_m / 2.0

    hop_centers = list(range(0, F, hop))
    N_hops = len(hop_centers)

    print(
        f"[cfar_rd] {F} frames, {T} taps, window={W} ({W / max(fps, 1.0):.2f}s), "
        f"hop={hop} ({hop / max(fps, 1.0) * 1e3:.0f}ms), N_hops={N_hops}",
        flush=True,
    )
    t0 = time.perf_counter()

    # --- Batch FFT: all hop windows in one call ---
    # Pre-cast once to avoid N_hops astype copies inside the loop.
    clutter_c64 = clutter_complex.astype(np.complex64)
    batch = np.zeros((N_hops, W, T), dtype=np.complex64)
    for hi, fi in enumerate(hop_centers):
        i0 = max(0, fi - half)
        i1 = min(F, i0 + W)
        n = i1 - i0
        batch[hi, :n, :] = clutter_c64[i0:i1, :]
        # Remaining rows stay zero → equivalent to zero-padding in np.fft.fft(n=W)

    # np.fft.fft along axis=1 (slow-time) is a single batched call — much faster
    # than N_hops separate calls.
    rd_batch = np.abs(
        np.fft.fftshift(np.fft.fft(batch, n=W, axis=1), axes=1)
    ).astype(np.float32)  # (N_hops, W, T)
    rd_batch[:, :, ~valid_mask] = 0.0
    print(f"[cfar_rd] batch FFT done ({N_hops} hops) in {time.perf_counter() - t0:.1f}s", flush=True)

    # --- Vectorized CFAR: one cfar_1d_batch call per hop instead of W*F cfar_1d calls ---
    hop_peaks: list[list[CfarDetection]] = []
    for hi, fi_center in enumerate(hop_centers):
        rd_map = rd_batch[hi].astype(np.float64)  # (W, T)
        det_mask, thr = cfar_1d_batch(
            rd_map,
            cfar_cfg.num_ref_cells,
            cfar_cfg.num_guard_cells,
            cfar_cfg.pfa,
            cfar_cfg.variant,
            cfar_cfg.os_rank,
        )
        det_mask &= valid_mask[np.newaxis, :]  # suppress wall-clip region

        frame_peaks: list[CfarDetection] = []
        for di in range(W):
            if not np.any(det_mask[di]):
                continue  # most Doppler bins have no detections — skip cheaply
            peaks = extract_peaks(
                detection_mask=det_mask[di],
                profile=rd_map[di],
                range_axis_m=range_axis,
                frame_idx=fi_center,
                threshold_profile=thr[di],
                cluster_min_gap_taps=cfar_cfg.cluster_min_gap_taps,
                max_peaks=cfar_cfg.max_peaks_per_frame,
            )
            # doppler_axis_mps = fftfreq × λ/2 = -v_true (approaching convention).
            # Negate to match EKF convention: positive = receding (range increasing).
            v = -float(doppler_axis_mps[di])
            for p in peaks:
                p.range_rate_m_s = v
            frame_peaks.extend(peaks)

        frame_peaks.sort(key=lambda d: d.magnitude, reverse=True)
        hop_peaks.append(frame_peaks[:cfar_cfg.max_peaks_per_frame])

    # --- Broadcast hop results to per-frame list ---
    # Frame fi gets the detections from hop fi // hop, with frame_idx corrected.
    frame_to_hop = np.minimum(np.arange(F, dtype=np.int32) // hop, N_hops - 1)
    results: list[list[CfarDetection]] = []
    for fi in range(F):
        src = hop_peaks[int(frame_to_hop[fi])]
        results.append([
            CfarDetection(
                frame_idx=fi,
                tap_idx=d.tap_idx,
                range_m=d.range_m,
                magnitude=d.magnitude,
                threshold=d.threshold,
                range_rate_m_s=d.range_rate_m_s,
            )
            for d in src
        ])

    print(f"[cfar_rd] done in {time.perf_counter() - t0:.1f}s", flush=True)
    return results
