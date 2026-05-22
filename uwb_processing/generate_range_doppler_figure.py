"""
Generate a scientific Range-Doppler heatmap figure and animated video
for use in the UWB pipeline Beamer presentation.

Outputs
-------
uwb_outputs/range_doppler_static.png   — publication-quality static figure
uwb_outputs/range_doppler_animation.mp4 — animated video (targets moving)

Run
---
    python -m uwb_processing.generate_range_doppler_figure
"""
from __future__ import annotations

import os
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np

# ---------------------------------------------------------------------------
# Output directory
# ---------------------------------------------------------------------------
OUT_DIR = pathlib.Path(__file__).parent.parent / "uwb_outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ---------------------------------------------------------------------------
# Grid
# ---------------------------------------------------------------------------
RANGE_MIN, RANGE_MAX = 0.0, 7.5          # metres
VEL_MIN,   VEL_MAX   = -3.0, 3.0        # m/s
NR, NV = 512, 512                        # grid resolution

range_axis = np.linspace(RANGE_MIN, RANGE_MAX, NR)
vel_axis   = np.linspace(VEL_MIN,   VEL_MAX,   NV)
RR, VV = np.meshgrid(range_axis, vel_axis)   # (NV, NR)

# ---------------------------------------------------------------------------
# Target definitions at t = 0
# Each entry: (range_m, vel_m_s, peak_dB, range_sigma_m, vel_sigma_m_s)
# ---------------------------------------------------------------------------
TARGETS_T0 = [
    # main targets
    dict(r=3.4,  v= 0.5,  peak=18.0, sr=0.18, sv=0.14),
    dict(r=4.2,  v=-0.8,  peak=16.0, sr=0.18, sv=0.14),
    # Doppler ambiguity replicas (same range, velocity-aliased copies)
    dict(r=3.4,  v=-2.6,  peak=10.0, sr=0.22, sv=0.18),
    dict(r=4.2,  v= 2.4,  peak= 9.0, sr=0.22, sv=0.18),
]

# noise floor in dB
NOISE_FLOOR_DB = -20.0

# ---------------------------------------------------------------------------
# Helper: build one Range-Doppler map frame
# ---------------------------------------------------------------------------

def _gaussian_peak(r0: float, v0: float, peak_db: float,
                   sr: float, sv: float) -> np.ndarray:
    return peak_db * np.exp(
        -0.5 * ((RR - r0) / sr) ** 2
        - 0.5 * ((VV - v0) / sv) ** 2
    )


def build_map(targets: list[dict]) -> np.ndarray:
    """Return power map in dB."""
    # start from noise floor with spatially-correlated noise
    rng = np.random.default_rng(seed=42)
    noise = rng.standard_normal((NV, NR)) * 1.5
    # smooth noise slightly so it looks like clutter, not pure white noise
    from scipy.ndimage import gaussian_filter
    noise = gaussian_filter(noise, sigma=1.5)
    power = NOISE_FLOOR_DB + noise
    for t in targets:
        power = np.maximum(power, _gaussian_peak(
            t["r"], t["v"], t["peak"], t["sr"], t["sv"]
        ))
    return power


# ---------------------------------------------------------------------------
# Static figure
# ---------------------------------------------------------------------------

def make_static_figure(out_path: pathlib.Path) -> None:
    map_db = build_map(TARGETS_T0)

    fig, ax = plt.subplots(figsize=(8, 5.5), facecolor="white")

    # --- heatmap ---
    img = ax.pcolormesh(
        range_axis, vel_axis, map_db,
        cmap="jet",
        vmin=NOISE_FLOOR_DB,
        vmax=18.0,
        shading="auto",
        rasterized=True,
    )

    # --- colorbar ---
    cbar = fig.colorbar(img, ax=ax, pad=0.02)
    cbar.set_label("Power (dB)", fontsize=11)
    cbar.set_ticks(np.arange(-20, 20, 4))
    cbar.ax.tick_params(labelsize=9)

    # --- axes ---
    ax.set_xlabel("Range (m)", fontsize=12, color="black")
    ax.set_ylabel("Velocity (m/s)", fontsize=12, color="black")
    ax.set_xlim(RANGE_MIN, RANGE_MAX)
    ax.set_ylim(VEL_MIN, VEL_MAX)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.tick_params(axis="both", which="both", color="black", labelcolor="black",
                   labelsize=9)
    for spine in ax.spines.values():
        spine.set_edgecolor("black")

    # --- zero-Doppler line ---
    ax.axhline(0, color="white", linewidth=0.6, linestyle="--", alpha=0.4)

    # --- target annotations ---
    labels = [
        (3.4,  0.4,  "T1"),
        (4.2, -0.6,  "T2"),
        (3.4, -2.0,  "T1\nalias"),
        (4.2,  1.8,  "T2\nalias"),
    ]
    for r, v, lbl in labels:
        ax.annotate(
            lbl,
            xy=(r, v), xytext=(r + 0.35, v + 0.25),
            color="white", fontsize=8, fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="white", lw=0.8),
        )

    # --- title & caption ---
    ax.set_title("Range–Doppler Map", fontsize=13, color="black", pad=6)
    fig.text(
        0.5, 0.01,
        "Range-Doppler map showing two targets and Doppler ambiguity replicas.",
        ha="center", va="bottom", fontsize=9, color="#333333",
        style="italic",
    )

    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(out_path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"[static]  saved -> {out_path}")


# ---------------------------------------------------------------------------
# Animation helpers
# ---------------------------------------------------------------------------

def _interpolate_targets(t_norm: float) -> list[dict]:
    """
    Smoothly move targets over normalised time t_norm in [0, 1].

    T1 (main) approaches the radar (range decreases) and accelerates slightly.
    T2 (main) moves away slowly.
    Aliases mirror the velocity evolution of their parents.
    """
    # T1: range 3.4 → 2.2 m,  vel 0.4 → 0.7 m/s  (approaching)
    r1 = 3.4 - 1.2 * t_norm
    v1 = 0.4 + 0.3 * t_norm
    # T2: range 4.2 → 5.6 m,  vel -0.6 → -1.0 m/s (moving away)
    r2 = 4.2 + 1.4 * t_norm
    v2 = -0.6 - 0.4 * t_norm
    # Aliases mirror their parent's range; velocity offset spans the new ±3 m/s window
    fade1 = 0.80 + 0.20 * np.sin(2 * np.pi * t_norm)
    fade2 = 0.75 + 0.25 * np.cos(2 * np.pi * t_norm)

    return [
        dict(r=r1,  v=v1,        peak=18.0 * fade1, sr=0.18, sv=0.14),
        dict(r=r2,  v=v2,        peak=16.0 * fade2, sr=0.18, sv=0.14),
        dict(r=r1,  v=v1 - 3.1, peak=10.0 * fade1, sr=0.22, sv=0.18),
        dict(r=r2,  v=v2 + 3.2, peak= 9.0 * fade2, sr=0.22, sv=0.18),
    ]


def make_animation(out_path: pathlib.Path, n_frames: int = 120, fps: int = 20) -> None:
    """
    Render each frame with matplotlib, capture as RGB numpy array,
    then encode to MP4 via OpenCV VideoWriter (no ffmpeg binary needed).
    """
    import cv2
    from scipy.ndimage import gaussian_filter

    rng = np.random.default_rng(seed=7)

    DPI = 120
    fig, ax = plt.subplots(figsize=(8, 5.5), facecolor="white")
    fig.canvas.draw()  # ensure canvas is initialised

    # stable colorbar — draw once on the first map
    init_map = build_map(TARGETS_T0)
    img = ax.pcolormesh(
        range_axis, vel_axis, init_map,
        cmap="jet",
        vmin=NOISE_FLOOR_DB,
        vmax=18.0,
        shading="auto",
        rasterized=True,
    )

    cbar = fig.colorbar(img, ax=ax, pad=0.02)
    cbar.set_label("Power (dB)", fontsize=11)
    cbar.set_ticks(np.arange(-20, 20, 4))
    cbar.ax.tick_params(labelsize=9)

    ax.set_xlabel("Range (m)", fontsize=12, color="black")
    ax.set_ylabel("Velocity (m/s)", fontsize=12, color="black")
    ax.set_xlim(RANGE_MIN, RANGE_MAX)
    ax.set_ylim(VEL_MIN, VEL_MAX)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.tick_params(axis="both", which="both", color="black", labelcolor="black",
                   labelsize=9)
    for spine in ax.spines.values():
        spine.set_edgecolor("black")
    ax.axhline(0, color="white", linewidth=0.6, linestyle="--", alpha=0.4)

    title = ax.set_title(
        "Range-Doppler Map Over Time - Frame 001",
        fontsize=12, color="black",
    )
    fig.text(
        0.5, 0.01,
        "Range-Doppler map showing two targets and Doppler ambiguity replicas.",
        ha="center", va="bottom", fontsize=9, color="#333333", style="italic",
    )
    fig.tight_layout(rect=[0, 0.04, 1, 1])

    # --- determine frame pixel dimensions after layout ---
    fig.canvas.draw()
    buf = fig.canvas.buffer_rgba()
    rgba = np.asarray(buf)
    h, w = rgba.shape[:2]
    # ensure dimensions are even (required by most codecs)
    h = h if h % 2 == 0 else h - 1
    w = w if w % 2 == 0 else w - 1

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))

    for frame_idx in range(n_frames):
        t_norm = frame_idx / max(n_frames - 1, 1)
        targets = _interpolate_targets(t_norm)

        noise = rng.standard_normal((NV, NR)) * 1.5
        noise = gaussian_filter(noise, sigma=1.5)
        power = NOISE_FLOOR_DB + noise
        for t in targets:
            power = np.maximum(
                power,
                _gaussian_peak(t["r"], t["v"], t["peak"], t["sr"], t["sv"]),
            )

        img.set_array(power.ravel())
        title.set_text(
            f"Range-Doppler Map Over Time - Frame {frame_idx + 1:03d}"
        )
        fig.canvas.draw()

        buf = fig.canvas.buffer_rgba()
        rgba = np.asarray(buf)[:h, :w, :3]   # crop to even dims, drop alpha
        bgr = rgba[:, :, ::-1].copy()         # RGB -> BGR for OpenCV
        writer.write(bgr)

        if (frame_idx + 1) % 20 == 0:
            print(f"  frame {frame_idx + 1:03d}/{n_frames}")

    writer.release()
    plt.close(fig)
    print(f"[animation] saved -> {out_path}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    from scipy.ndimage import gaussian_filter  # validate import early

    static_path = OUT_DIR / "range_doppler_static.png"
    anim_path   = OUT_DIR / "range_doppler_animation.mp4"

    print("Generating static Range-Doppler figure …")
    make_static_figure(static_path)

    print("Generating animated Range-Doppler video (120 frames @ 20 fps) …")
    make_animation(anim_path, n_frames=120, fps=20)

    print("\nDone.")
    print(f"  Static : {static_path}")
    print(f"  Video  : {anim_path}")
