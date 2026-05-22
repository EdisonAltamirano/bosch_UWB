from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .types import CfarDetection, PreprocessedSession, Track
from .visualization_utils import db_limits, jet_color, magnitude_to_db


def animate_range_time(
    preprocessed: PreprocessedSession,
    detections_per_frame: list[list[CfarDetection]],
    tracks_per_frame: list[list[Track]],
    output_path: Path,
    fps: int = 10,
    window_frames: int = 100,
    max_range_m: float = 6.0,
    use_moviepy: bool = False,
) -> None:
    """Scrolling 2-D range-time animation with CFAR detections and track positions.

    Defaults to matplotlib FuncAnimation + FFMpegWriter (always available).
    Set use_moviepy=True to render via moviepy VideoClip instead (better for
    compositing or when ffmpeg is not on PATH).
    """
    valid = (preprocessed.range_axis_m >= preprocessed.config.wall_clip_m) & \
            (preprocessed.range_axis_m <= max_range_m)
    range_axis = preprocessed.range_axis_m[valid]
    timestamps = preprocessed.session.timestamps_s
    Z_full = magnitude_to_db(preprocessed.clutter_removed[:, valid])  # (Nf, Nr)

    N_frames = len(timestamps)
    window_frames = min(window_frames, N_frames)
    max_track_id = max(
        (track.track_id for tracks in tracks_per_frame for track in tracks),
        default=0,
    ) + 1

    if use_moviepy:
        _animate_moviepy(
            Z_full, timestamps, range_axis, detections_per_frame, tracks_per_frame,
            N_frames, window_frames, fps, max_track_id, output_path,
        )
    else:
        _animate_funcanimation(
            Z_full, timestamps, range_axis, detections_per_frame, tracks_per_frame,
            N_frames, window_frames, fps, max_track_id, output_path,
        )


# ---------------------------------------------------------------------------
# matplotlib FuncAnimation backend
# ---------------------------------------------------------------------------

def _animate_funcanimation(
    Z_full, timestamps, range_axis, detections_per_frame, tracks_per_frame,
    N_frames, window_frames, fps, max_track_id, output_path: Path,
) -> None:
    from matplotlib.animation import FuncAnimation, FFMpegWriter

    vmin, vmax = db_limits(Z_full)

    fig, axes = plt.subplots(
        1, 2, figsize=(14, 5),
        gridspec_kw={"width_ratios": [3, 1]},
    )
    ax_waterfall, ax_profile = axes

    # Left panel: scrolling waterfall
    t0, t1 = timestamps[0], timestamps[min(window_frames - 1, N_frames - 1)]
    im = ax_waterfall.imshow(
        Z_full[:window_frames, :].T,
        aspect="auto",
        origin="lower",
        extent=[t0, t1, range_axis[0], range_axis[-1]],
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
    )
    cursor_line = ax_waterfall.axvline(x=t0, color="red", linewidth=1.0, alpha=0.8)
    det_scatter = ax_waterfall.scatter([], [], c=[jet_color(1, 4)], s=12, alpha=0.7, linewidths=0,
                                       zorder=3, label="CFAR")
    track_lines = []
    ax_waterfall.set_xlabel("Time (s)")
    ax_waterfall.set_ylabel("Range (m)")
    ax_waterfall.set_title("CIR Range-Time Waterfall (dB)")
    ax_waterfall.legend(loc="upper right", fontsize=7)
    fig.colorbar(im, ax=ax_waterfall, label="Magnitude (dB)", fraction=0.03)

    # Right panel: current CIR range profile with CFAR threshold
    (profile_line,) = ax_profile.plot([], range_axis, color=jet_color(2, 4), linewidth=1.0)
    (thresh_line,)  = ax_profile.plot([], range_axis, color=jet_color(3, 4), linewidth=0.8,
                                       linestyle="--", alpha=0.7, label="CFAR thr")
    det_dots = ax_profile.scatter([], [], c=[jet_color(1, 4)], s=20, zorder=3)
    ax_profile.set_xlim(vmin, vmax * 1.1)
    ax_profile.set_ylim(range_axis[0], range_axis[-1])
    ax_profile.set_xlabel("Magnitude (dB)")
    ax_profile.set_title("Current frame")
    ax_profile.legend(fontsize=7)

    fig.tight_layout()

    def _update(fi: int):
        # Scrolling window bounds
        win_start = max(0, fi - window_frames + 1)
        win_end = fi + 1
        t_win = timestamps[win_start:win_end]
        z_win = Z_full[win_start:win_end, :]

        im.set_data(z_win.T)
        im.set_extent([t_win[0], t_win[-1], range_axis[0], range_axis[-1]])
        cursor_line.set_xdata([timestamps[fi], timestamps[fi]])

        # Detection scatter on waterfall (current window only)
        d_t, d_r = [], []
        for f in range(win_start, win_end):
            if f < len(detections_per_frame):
                for det in detections_per_frame[f]:
                    d_t.append(timestamps[f])
                    d_r.append(det.range_m)
        det_scatter.set_offsets(np.column_stack([d_t, d_r]) if d_t else np.empty((0, 2)))

        # Track lines on waterfall
        for ln in track_lines:
            ln.remove()
        track_lines.clear()
        if fi < len(tracks_per_frame):
            for i, track in enumerate(tracks_per_frame[fi]):
                ts_clip = [t for t in track.history_t if timestamps[win_start] <= t <= timestamps[fi]]
                rs_clip = track.history_m[-len(ts_clip):]
                if len(ts_clip) >= 2:
                    (ln,) = ax_waterfall.plot(ts_clip, rs_clip,
                                              color=jet_color(track.track_id, max_track_id),
                                              linewidth=1.5, zorder=4)
                    track_lines.append(ln)

        # Right panel: current range profile
        profile = Z_full[fi, :]
        profile_line.set_xdata(profile)
        profile_line.set_ydata(range_axis)

        # Approximate threshold from detection magnitudes (for display)
        d_mags, d_rs = [], []
        if fi < len(detections_per_frame):
            for det in detections_per_frame[fi]:
                d_mags.append(float(magnitude_to_db(np.asarray([det.threshold], dtype=np.float32))[0]))
                d_rs.append(det.range_m)
        if d_mags:
            thresh_line.set_xdata(d_mags)
            thresh_line.set_ydata(d_rs)
            det_dots.set_offsets(np.column_stack([d_mags, d_rs]))
        else:
            thresh_line.set_xdata([])
            thresh_line.set_ydata([])
            det_dots.set_offsets(np.empty((0, 2)))

        ax_profile.set_title(f"Frame {fi}  t={timestamps[fi]:.2f}s")
        return im, cursor_line, det_scatter, profile_line, thresh_line, det_dots

    ani = FuncAnimation(fig, _update, frames=N_frames, interval=int(1000 / fps), blit=False)

    writer = FFMpegWriter(fps=fps, bitrate=2000, codec="libx264")
    ani.save(str(output_path), writer=writer)
    plt.close(fig)


# ---------------------------------------------------------------------------
# moviepy backend
# ---------------------------------------------------------------------------

def _animate_moviepy(
    Z_full, timestamps, range_axis, detections_per_frame, tracks_per_frame,
    N_frames, window_frames, fps, max_track_id, output_path: Path,
) -> None:
    try:
        from moviepy.editor import VideoClip
        from moviepy.video.io.bindings import mplfig_to_npimage
    except ImportError as exc:
        raise ImportError(
            "moviepy is not installed. Run `pip install moviepy` or use the default "
            "FuncAnimation backend (use_moviepy=False)."
        ) from exc

    vmin, vmax = db_limits(Z_full)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5),
                             gridspec_kw={"width_ratios": [3, 1]})
    ax_waterfall, ax_profile = axes

    im = ax_waterfall.imshow(
        Z_full[:window_frames].T,
        aspect="auto", origin="lower",
        extent=[timestamps[0], timestamps[min(window_frames - 1, N_frames - 1)],
                range_axis[0], range_axis[-1]],
        cmap="jet", vmin=vmin, vmax=vmax,
    )
    cursor_line = ax_waterfall.axvline(x=timestamps[0], color="red", linewidth=1.0)
    det_scatter = ax_waterfall.scatter([], [], c=[jet_color(1, 4)], s=12, zorder=3)
    track_lines: list = []
    ax_waterfall.set_xlabel("Time (s)")
    ax_waterfall.set_ylabel("Range (m)")
    fig.colorbar(im, ax=ax_waterfall, label="Magnitude (dB)", fraction=0.03)

    (profile_line,) = ax_profile.plot([], range_axis, color=jet_color(2, 4))
    ax_profile.set_xlim(vmin, vmax * 1.1)
    ax_profile.set_ylim(range_axis[0], range_axis[-1])
    ax_profile.set_xlabel("Magnitude (dB)")
    fig.tight_layout()

    def _make_frame(t: float) -> np.ndarray:
        fi = min(int(t * fps), N_frames - 1)
        win_start = max(0, fi - window_frames + 1)
        win_end = fi + 1

        t_win = timestamps[win_start:win_end]
        z_win = Z_full[win_start:win_end]
        im.set_data(z_win.T)
        im.set_extent([t_win[0], t_win[-1], range_axis[0], range_axis[-1]])
        cursor_line.set_xdata([timestamps[fi], timestamps[fi]])

        d_t, d_r = [], []
        for f in range(win_start, win_end):
            if f < len(detections_per_frame):
                for det in detections_per_frame[f]:
                    d_t.append(timestamps[f])
                    d_r.append(det.range_m)
        det_scatter.set_offsets(
            np.column_stack([d_t, d_r]) if d_t else np.empty((0, 2))
        )

        for ln in track_lines:
            ln.remove()
        track_lines.clear()
        if fi < len(tracks_per_frame):
            for i, track in enumerate(tracks_per_frame[fi]):
                ts_clip = [t_ for t_ in track.history_t
                           if timestamps[win_start] <= t_ <= timestamps[fi]]
                rs_clip = track.history_m[-len(ts_clip):]
                if len(ts_clip) >= 2:
                    (ln,) = ax_waterfall.plot(
                        ts_clip, rs_clip,
                        color=jet_color(track.track_id, max_track_id),
                        linewidth=1.5, zorder=4,
                    )
                    track_lines.append(ln)

        profile_line.set_xdata(Z_full[fi])
        profile_line.set_ydata(range_axis)
        ax_waterfall.set_title(f"CIR Waterfall (dB)  t={timestamps[fi]:.2f}s")
        ax_profile.set_title(f"Frame {fi}")
        fig.canvas.draw()
        return mplfig_to_npimage(fig)

    duration = N_frames / fps
    clip = VideoClip(_make_frame, duration=duration)
    clip.write_videofile(str(output_path), fps=fps, codec="libx264", audio=False,
                         logger=None)
    plt.close(fig)
