from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .types import CfarDetection, PreprocessedSession, Track
from .visualization_utils import db_limits, jet_color, magnitude_to_db


def save_cir_3d_surface(
    preprocessed: PreprocessedSession,
    output_path: Path,
    max_range_m: float = 6.0,
    downsample_frames: int = 1,
    elev: float = 35.0,
    azim: float = -60.0,
    confirmed_tracks: list[Track] | None = None,
) -> None:
    """3-D surface plot of the clutter-removed CIR waterfall.

    X = range (m), Y = slow-time (s), Z = CIR magnitude in dB.
    Bright peaks correspond to moving reflectors / human targets.
    Optional confirmed track trajectories are overlaid as coloured 3-D lines.
    """
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    valid = (preprocessed.range_axis_m >= preprocessed.config.wall_clip_m) & \
            (preprocessed.range_axis_m <= max_range_m)
    R = preprocessed.range_axis_m[valid]                             # (Nr,)
    T = preprocessed.session.timestamps_s[::downsample_frames]      # (Nf,)
    Z = magnitude_to_db(preprocessed.clutter_removed[::downsample_frames][:, valid]) # (Nf, Nr)
    vmin, vmax = db_limits(Z)

    X_mesh, Y_mesh = np.meshgrid(R, T)

    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(
        X_mesh, Y_mesh, Z,
        cmap="jet",
        linewidth=0,
        antialiased=False,
        alpha=0.85,
        vmin=vmin,
        vmax=vmax,
    )
    fig.colorbar(surf, ax=ax, shrink=0.45, pad=0.05, label="CIR Magnitude (dB)")

    # Overlay confirmed tracks as coloured 3-D lines
    if confirmed_tracks:
        track_count = max(len(confirmed_tracks), 1)
        for i, track in enumerate(confirmed_tracks):
            if len(track.history_t) < 2:
                continue
            ts = np.array(track.history_t)
            rs = np.array(track.history_m)
            # Interpolate magnitude along the track for the Z coordinate
            rs_clip = np.clip(rs, R.min(), R.max())
            zvals = np.array([
                float(np.interp(r, R, Z[np.searchsorted(T, t, side="right") - 1]))
                for t, r in zip(ts, rs_clip)
            ])
            color = jet_color(i, track_count)
            ax.plot(rs, ts, zvals, color=color, linewidth=2.0,
                    label=f"track {track.track_id}", zorder=5)
        ax.legend(loc="upper left", fontsize=8)

    ax.set_xlabel("Range (m)")
    ax.set_ylabel("Time (s)")
    ax.set_zlabel("CIR Magnitude (dB)")
    ax.set_title(
        f"3-D CIR Waterfall — UWB Pulse Radar  "
        f"(path {preprocessed.selected_path})"
    )
    ax.view_init(elev=elev, azim=azim)
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_cfar_range_time_plot(
    preprocessed: PreprocessedSession,
    detections_per_frame: list[list[CfarDetection]],
    confirmed_tracks: list[Track],
    output_path: Path,
    max_range_m: float = 6.0,
) -> None:
    """2-D range-time heatmap with CFAR detections (scatter) and track trajectories (lines)."""
    valid = (preprocessed.range_axis_m >= preprocessed.config.wall_clip_m) & \
            (preprocessed.range_axis_m <= max_range_m)
    range_axis = preprocessed.range_axis_m[valid]
    timestamps = preprocessed.session.timestamps_s
    Z = magnitude_to_db(preprocessed.clutter_removed[:, valid]).T  # (Nr, Nf)
    vmin, vmax = db_limits(Z)

    fig, ax = plt.subplots(figsize=(13, 6))
    im = ax.imshow(
        Z,
        aspect="auto",
        origin="lower",
        extent=[timestamps[0], timestamps[-1], range_axis[0], range_axis[-1]],
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
    )
    fig.colorbar(im, ax=ax, label="Magnitude (dB)")

    # CFAR detections
    det_t, det_r = [], []
    for fi, dets in enumerate(detections_per_frame):
        if fi < len(timestamps):
            for d in dets:
                det_t.append(timestamps[fi])
                det_r.append(d.range_m)
    if det_t:
        ax.scatter(det_t, det_r, c=[jet_color(1, 4)], s=8, alpha=0.6, linewidths=0,
                   label="CFAR detection", zorder=3)

    # Confirmed track trajectories
    track_count = max(len(confirmed_tracks), 1)
    for i, track in enumerate(confirmed_tracks):
        if len(track.history_t) < 2:
            continue
        ax.plot(track.history_t, track.history_m,
                color=jet_color(i, track_count), linewidth=1.5,
                label=f"track {track.track_id}", zorder=4)

    ax.set_title(f"Range-Time + CFAR Detections + Tracks  (path {preprocessed.selected_path})")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Range (m)")
    if det_t or confirmed_tracks:
        ax.legend(loc="upper right", fontsize=8, markerscale=2)
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
