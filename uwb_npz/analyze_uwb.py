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


def plot_single_frame(frame: dict, frame_idx: int = 0, out_dir: Path = OUT_DIR):
    mag = cir_magnitude(frame)
    n_paths, n_taps = mag.shape
    tap_axis = np.arange(n_taps)

    fig, axes = plt.subplots(n_paths, 1, figsize=(12, 2.5 * n_paths), sharex=True)
    if n_paths == 1:
        axes = [axes]
    fig.suptitle(f"CIR magnitude — frame {frame_idx}", fontsize=13)

    for i, ax in enumerate(axes):
        rx = frame["rx_antenna_ids"][i]
        tx = frame["tx_antenna_ids"][i]
        offset = frame["cir_start_offsets"][i]
        ax.plot(tap_axis, mag[i], linewidth=0.9)
        ax.set_ylabel("Magnitude")
        ax.set_title(f"RX path {i}  (ant RX={rx} TX={tx}, start_offset={offset})",
                     fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Tap index")
    plt.tight_layout()
    out = out_dir / "cir_single_frame.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")
    plt.show()


def plot_waterfall(frames: list[dict], path_idx: int = 0, out_dir: Path = OUT_DIR):
    mags = np.stack([cir_magnitude(f)[path_idx] for f in frames], axis=0)  # (frames, taps)

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.imshow(
        mags,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        cmap="viridis",
    )
    plt.colorbar(im, ax=ax, label="CIR magnitude")
    rx = frames[0]["rx_antenna_ids"][path_idx]
    tx = frames[0]["tx_antenna_ids"][path_idx]
    ax.set_title(f"CIR waterfall — RX path {path_idx} (ant RX={rx} TX={tx})", fontsize=13)
    ax.set_xlabel("Tap index")
    ax.set_ylabel("Frame index")
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

    # 1. CIR magnitude for the first frame
    plot_single_frame(frames[0], frame_idx=0)

    # 2. Waterfall for each RX path
    n_paths = frames[0]["taps_real"].shape[0]
    for p in range(n_paths):
        plot_waterfall(frames, path_idx=p)

    # 3. Peak tap index over time
    plot_peak_tap_over_time(frames)


if __name__ == "__main__":
    main()
