from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .aoa import TrackAoA
from .visualization_utils import db_limits, jet_color, magnitude_to_db

def render_aoa_frame(
    frame_idx: int,
    timestamp_s: float,
    track_aoas: list[TrackAoA],
    range_angle_grid: np.ndarray,
    angle_axis_deg: np.ndarray,
    range_axis_m: np.ndarray,
    fov_limit_deg: float,
) -> np.ndarray:
    fig = plt.figure(figsize=(12, 5.12))
    ax_polar = fig.add_subplot(1, 2, 1, projection="polar")
    ax_heatmap = fig.add_subplot(1, 2, 2)
    max_track_id = max((track_aoa.track_id for track_aoa in track_aoas), default=0) + 1
    range_angle_db = magnitude_to_db(range_angle_grid)
    vmin, vmax = db_limits(range_angle_db)

    theta_min = np.deg2rad(-float(fov_limit_deg))
    theta_max = np.deg2rad(float(fov_limit_deg))
    ax_polar.set_theta_zero_location("N")
    ax_polar.set_theta_direction(-1)
    ax_polar.set_thetamin(np.degrees(theta_min))
    ax_polar.set_thetamax(np.degrees(theta_max))
    ax_polar.set_title(f"Azimuth sector  frame={frame_idx}  t={timestamp_s:.2f}s")
    ax_polar.set_rlim(0.0, float(range_axis_m[-1]) if range_axis_m.size else 1.0)

    for track_aoa in track_aoas:
        color = jet_color(track_aoa.track_id, max_track_id)
        ax_polar.scatter(
            np.deg2rad(track_aoa.angle_deg),
            track_aoa.range_m,
            s=70,
            color=[color],
            edgecolors="white",
            linewidths=0.8,
        )
        ax_polar.text(
            np.deg2rad(track_aoa.angle_deg),
            track_aoa.range_m,
            f"#{track_aoa.track_id} {track_aoa.range_m:.1f}m",
            fontsize=8,
            fontweight="bold",
            color=color,
            bbox={"boxstyle": "round,pad=0.15", "facecolor": "white", "alpha": 0.75, "edgecolor": "none"},
        )

    im = ax_heatmap.imshow(
        range_angle_db,
        aspect="auto",
        origin="lower",
        extent=[angle_axis_deg[0], angle_axis_deg[-1], range_axis_m[0], range_axis_m[-1]],
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
    )
    ax_heatmap.set_title("Range-azimuth heatmap (dB)")
    ax_heatmap.set_xlabel("Azimuth (deg)")
    ax_heatmap.set_ylabel("Range (m)")
    for track_aoa in track_aoas:
        ax_heatmap.scatter(
            [track_aoa.angle_deg],
            [track_aoa.range_m],
            c=[jet_color(track_aoa.track_id, max_track_id)],
            s=45,
            edgecolors="white",
            linewidths=0.8,
        )
    fig.colorbar(im, ax=ax_heatmap, fraction=0.046, pad=0.04, label="Magnitude (dB)")

    fig.tight_layout()
    fig.canvas.draw()
    image = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8)[..., :3].copy()
    plt.close(fig)
    return image


def save_aoa_animation(
    timestamps_s: np.ndarray,
    track_aoas_per_frame: list[list[TrackAoA]],
    range_angle_grids: np.ndarray,
    angle_axis_deg: np.ndarray,
    range_axis_m: np.ndarray,
    output_path,
    fov_limit_deg: float,
    fps: int = 10,
) -> None:
    import imageio.v2 as imageio

    output_path = Path(output_path)
    partial_output_path = output_path.with_name(f"{output_path.stem}.partial{output_path.suffix}")
    if partial_output_path.exists():
        partial_output_path.unlink()

    total_frames = len(timestamps_s)
    if total_frames == 0:
        return

    dt = float(timestamps_s[1] - timestamps_s[0]) if total_frames > 1 else 0.1
    stride = max(1, round(1.0 / max(dt * fps, 1e-12)))
    render_indices = list(range(0, total_frames, stride))
    if render_indices[-1] != total_frames - 1:
        render_indices.append(total_frames - 1)

    print(
        f"[aoa] {total_frames} source frames -> {len(render_indices)} rendered "
        f"@ {fps} fps (stride {stride}) -> {partial_output_path.name}"
    )
    try:
        with imageio.get_writer(str(partial_output_path), fps=fps, codec="libx264") as writer:
            for render_i, frame_idx in enumerate(render_indices):
                timestamp_s = float(timestamps_s[frame_idx])
                if render_i == 0 or render_i == len(render_indices) - 1 or render_i % 25 == 0:
                    print(f"[aoa] rendering frame {render_i + 1}/{len(render_indices)}")
                rgb = render_aoa_frame(
                    frame_idx=frame_idx,
                    timestamp_s=timestamp_s,
                    track_aoas=track_aoas_per_frame[frame_idx],
                    range_angle_grid=range_angle_grids[frame_idx],
                    angle_axis_deg=angle_axis_deg,
                    range_axis_m=range_axis_m,
                    fov_limit_deg=fov_limit_deg,
                )
                writer.append_data(rgb)
        partial_output_path.replace(output_path)
        print(f"[aoa] wrote finalized mp4: {output_path.name}")
    except Exception:
        if partial_output_path.exists():
            partial_output_path.unlink()
        raise
