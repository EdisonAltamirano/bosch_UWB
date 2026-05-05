"""
Analyze UWB CIR frames saved as .npz files.

Usage:
    python3 analyze_uwb.py                  # loads all .npz in same directory
    python3 analyze_uwb.py /path/to/dir     # loads from specified directory
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

OUT_DIR = Path("/home/ws/src/uwb_npz")


def load_frames(npz_dir: Path) -> list[dict]:
    files = sorted(npz_dir.glob("*.npz"))
    if not files:
        print(f"No .npz files found in {npz_dir}")
        sys.exit(1)
    print(f"Loading {len(files)} frames from {npz_dir} ...")
    frames = []
    for f in files:
        d = np.load(f)
        frames.append({k: d[k] for k in d.files})
    return frames


def cir_magnitude(frame: dict) -> np.ndarray:
    return np.sqrt(frame["taps_real"].astype(np.float32) ** 2
                   + frame["taps_imag"].astype(np.float32) ** 2)


def plot_single_frame(frame: dict, frame_idx: int = 0, out_dir: Path = OUT_DIR,
                      tap_resolution_cm: float = 15.0):
    mag = cir_magnitude(frame)
    n_paths, n_taps = mag.shape
    dist_axis_m = np.arange(n_taps) * tap_resolution_cm / 100.0

    fig, axes = plt.subplots(n_paths, 1, figsize=(12, 2.5 * n_paths), sharex=True)
    if n_paths == 1:
        axes = [axes]
    fig.suptitle(f"CIR magnitude — frame {frame_idx}  "
                 f"({tap_resolution_cm:.0f}cm/tap, max {dist_axis_m[-1]:.1f}m)", fontsize=13)

    for i, ax in enumerate(axes):
        rx = frame["rx_antenna_ids"][i]
        tx = frame["tx_antenna_ids"][i]
        offset = frame["cir_start_offsets"][i]
        ax.plot(dist_axis_m, mag[i], linewidth=0.9)
        ax.set_ylabel("Magnitude")
        ax.set_title(f"RX path {i}  (ant RX={rx} TX={tx}, start_offset={offset}, "
                     f"tap_res={tap_resolution_cm:.0f}cm)", fontsize=9)
        ax.grid(True, alpha=0.3)

        # Secondary x axis with tap index
        ax2 = ax.twiny()
        ax2.set_xlim(0, n_taps)
        ax2.set_xlabel("Tap index", fontsize=8, color="gray")
        ax2.tick_params(axis="x", labelcolor="gray", labelsize=7)

    axes[-1].set_xlabel("Distance (m)", fontsize=11)
    plt.tight_layout()
    out = out_dir / "cir_single_frame.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")
    plt.show()


def plot_waterfall(frames: list[dict], path_idx: int = 0, out_dir: Path = OUT_DIR,
                   tap_resolution_cm: float = 15.0, frame_interval_ms: float = 100.0):
    valid = [f for f in frames if cir_magnitude(f).shape[0] > path_idx]
    if not valid:
        print(f"[waterfall] path_idx={path_idx} exceeds all frames — skipping.")
        return
    if len(valid) < len(frames):
        print(f"[waterfall] path_idx={path_idx}: skipped {len(frames)-len(valid)} frames with fewer paths.")
    mags = np.stack([cir_magnitude(f)[path_idx] for f in valid], axis=0)  # (frames, taps)
    n_frames, n_taps = mags.shape

    max_dist_m   = n_taps * tap_resolution_cm / 100.0
    total_time_s = n_frames * frame_interval_ms / 1000.0

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.imshow(
        mags,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        cmap="viridis",
        extent=[0, max_dist_m, 0, total_time_s],
    )
    plt.colorbar(im, ax=ax, label="CIR magnitude")

    # Primary axes — real units
    ax.set_xlabel("Distance (m)", fontsize=11)
    ax.set_ylabel("Time (s)", fontsize=11)

    # Secondary X axis — tap index
    ax_top = ax.twiny()
    ax_top.set_xlim(0, n_taps)
    ax_top.set_xlabel("Tap index", fontsize=9, color="gray")
    ax_top.tick_params(axis="x", labelcolor="gray", labelsize=8)

    # Secondary Y axis — frame index
    ax_right = ax.twinx()
    ax_right.set_ylim(0, n_frames)
    ax_right.set_ylabel("Frame index", fontsize=9, color="gray")
    ax_right.tick_params(axis="y", labelcolor="gray", labelsize=8)

    rx = frames[0]["rx_antenna_ids"][path_idx]
    tx = frames[0]["tx_antenna_ids"][path_idx]
    ax.set_title(
        f"CIR waterfall — RX path {path_idx}  (ant RX={rx} TX={tx})\n"
        f"{n_frames} frames × {frame_interval_ms:.0f}ms = {total_time_s:.1f}s  |  "
        f"{n_taps} taps × {tap_resolution_cm:.0f}cm/tap = {max_dist_m:.1f}m max",
        fontsize=11,
    )
    plt.tight_layout()
    out = out_dir / f"cir_waterfall_path{path_idx}.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")
    plt.show()


def plot_peak_tap_over_time(frames: list[dict], out_dir: Path = OUT_DIR):
    n_paths = frames[0]["taps_real"].shape[0]
    fig, ax = plt.subplots(figsize=(12, 5))

    for i in range(n_paths):
        peaks = [int(np.argmax(cir_magnitude(f)[i])) for f in frames]
        rx = frames[0]["rx_antenna_ids"][i]
        tx = frames[0]["tx_antenna_ids"][i]
        ax.plot(peaks, label=f"path {i} (RX={rx} TX={tx})", linewidth=0.9)

    ax.set_title("Peak tap index over time (all RX paths)", fontsize=13)
    ax.set_xlabel("Frame index")
    ax.set_ylabel("Peak tap index")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out = out_dir / "cir_peak_tap.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")
    plt.show()


def print_summary(frames: list[dict]):
    f0 = frames[0]
    print("\n--- Dataset summary ---")
    print(f"  Frames loaded   : {len(frames)}")
    print(f"  RX paths/frame  : {f0['num_samples']}")
    print(f"  Taps/path       : {f0['taps_real'].shape[1]}")
    print(f"  bytes_per_tap   : {f0['bytes_per_tap']}")
    print(f"  block_size      : {f0['block_size']}")
    print(f"  RX antenna IDs  : {f0['rx_antenna_ids'].tolist()}")
    print(f"  TX antenna IDs  : {f0['tx_antenna_ids'].tolist()}")
    print()


def main():
    npz_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
    frames = load_frames(npz_dir)
    print_summary(frames)

    TAP_RES_CM      = 15.0   # cm por tap  (00=15cm, 01=7.5cm — ver tap_resolution_cm en metadata)
    FRAME_INTERVAL  = 100.0  # ms por frame (ranging_interval_ms configurado en STM32)

    # 1. CIR magnitude for the first frame
    plot_single_frame(frames[0], frame_idx=0, tap_resolution_cm=TAP_RES_CM)

    # 2. Waterfall for each RX path
    n_paths = frames[0]["taps_real"].shape[0]
    for p in range(n_paths):
        plot_waterfall(frames, path_idx=p,
                       tap_resolution_cm=TAP_RES_CM,
                       frame_interval_ms=FRAME_INTERVAL)

    # 3. Peak tap index over time
    plot_peak_tap_over_time(frames)


if __name__ == "__main__":
    main()
