from __future__ import annotations

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
    """1-D CA/OS/GO-CFAR on a range profile (shape T,).

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
    elif variant in ("OS", "GO"):
        # GO uses the greater-of half-window; OS uses rank ordering.
        # Both reuse the CA alpha as an approximation for the scale factor.
        alpha = _alpha_ca(num_ref, pfa)
    else:
        raise ValueError(f"Unknown CFAR variant: {variant!r}")

    for cut in range(pad, T - pad):
        left_ref = profile[cut - pad : cut - num_guard]
        right_ref = profile[cut + num_guard + 1 : cut + pad + 1]

        if variant == "CA":
            noise_est = float(np.mean(np.concatenate([left_ref, right_ref])))
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

    # Mask to restrict CFAR to taps at or beyond the wall clip
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
