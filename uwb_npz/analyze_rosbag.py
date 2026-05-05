"""
Decode a UWB MCAP rosbag and generate the same plots as analyze_uwb.py.

Usage:
    python3 analyze_rosbag.py /path/to/bag_dir
    python3 analyze_rosbag.py /path/to/bag_dir --tap-res 7.5 --frame-interval 100

The bag directory is the folder created by uwb_rosbag_recorder_node (contains
metadata.yaml + the .mcap file).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message

try:
    from sensors_interfaces.msg import UwbFrame as UWBFrameMsg
except ImportError:
    try:
        from sensors_interfaces.msg import UWBFrame as UWBFrameMsg
    except ImportError:
        from sensors_interfaces.msg import UWB_Frame as UWBFrameMsg  # type: ignore

try:
    from sensors.sr250_protocol import parse_cir_udp_payload
except ImportError:
    from sr250_protocol import parse_cir_udp_payload  # type: ignore


# ---------------------------------------------------------------------------
# Rosbag reading
# ---------------------------------------------------------------------------

def load_frames_from_bag(bag_path: Path, topic: str = "/uwb/frame_raw") -> list[dict]:
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    filter_ = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_)

    frames: list[dict] = []
    skipped = 0

    while reader.has_next():
        _topic, data, _ts = reader.read_next()
        msg: UWBFrameMsg = deserialize_message(data, UWBFrameMsg)

        if int(msg.radar_data_type) != 0x00:
            skipped += 1
            continue

        raw_payload = bytes(msg.raw_payload)
        try:
            parsed = parse_cir_udp_payload(raw_payload)
        except Exception as exc:
            skipped += 1
            print(f"[warn] parse error (seq={msg.seq}): {exc}")
            continue

        num_samples = parsed["num_samples"]
        taps_per_block = parsed["actual_cir_taps_per_block"]
        if taps_per_block is None:
            taps_per_block = len(parsed["samples"][0]["taps"]) if parsed["samples"] else 0

        taps_real = np.zeros((num_samples, taps_per_block), dtype=np.int16)
        taps_imag = np.zeros((num_samples, taps_per_block), dtype=np.int16)
        rx_antenna_ids = np.zeros(num_samples, dtype=np.uint8)
        tx_antenna_ids = np.zeros(num_samples, dtype=np.uint8)
        cir_start_offsets = np.zeros(num_samples, dtype=np.uint16)

        for i, sample in enumerate(parsed["samples"]):
            meta = sample["metadata"]
            rx_antenna_ids[i] = meta["rx_antenna_id"]
            tx_antenna_ids[i] = meta["tx_antenna_id"]
            cir_start_offsets[i] = meta["cir_start_offset"]
            for j, (r, im) in enumerate(sample["taps"]):
                taps_real[i, j] = r
                taps_imag[i, j] = im

        frames.append({
            "num_samples": num_samples,
            "taps_real": taps_real,
            "taps_imag": taps_imag,
            "rx_antenna_ids": rx_antenna_ids,
            "tx_antenna_ids": tx_antenna_ids,
            "cir_start_offsets": cir_start_offsets,
            "bytes_per_tap": parsed["bytes_per_tap"],
            "block_size": parsed["block_size"],
            "stamp_sec": msg.stamp.sec,
            "stamp_nanosec": msg.stamp.nanosec,
        })

    del reader
    print(f"Loaded {len(frames)} CIR frames from bag ({skipped} skipped).")
    return frames


# ---------------------------------------------------------------------------
# Plotting (same as analyze_uwb.py)
# ---------------------------------------------------------------------------

def cir_magnitude(frame: dict) -> np.ndarray:
    return np.sqrt(
        frame["taps_real"].astype(np.float32) ** 2
        + frame["taps_imag"].astype(np.float32) ** 2
    )


def print_summary(frames: list[dict]) -> None:
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


def plot_single_frame(frame: dict, frame_idx: int, out_dir: Path,
                      tap_resolution_cm: float = 15.0) -> None:
    mag = cir_magnitude(frame)
    n_paths, n_taps = mag.shape
    dist_axis_m = np.arange(n_taps) * tap_resolution_cm / 100.0

    fig, axes = plt.subplots(n_paths, 1, figsize=(12, 2.5 * n_paths), sharex=True)
    if n_paths == 1:
        axes = [axes]
    fig.suptitle(
        f"CIR magnitude — frame {frame_idx}  "
        f"({tap_resolution_cm:.0f} cm/tap, max {dist_axis_m[-1]:.1f} m)",
        fontsize=13,
    )

    for i, ax in enumerate(axes):
        rx = frame["rx_antenna_ids"][i]
        tx = frame["tx_antenna_ids"][i]
        offset = frame["cir_start_offsets"][i]
        ax.plot(dist_axis_m, mag[i], linewidth=0.9)
        ax.set_ylabel("Magnitude")
        ax.set_title(
            f"RX path {i}  (ant RX={rx} TX={tx}, start_offset={offset}, "
            f"tap_res={tap_resolution_cm:.0f} cm)",
            fontsize=9,
        )
        ax.grid(True, alpha=0.3)

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


def _timestamps_s(frames: list[dict]) -> np.ndarray:
    t = np.array([f["stamp_sec"] + f["stamp_nanosec"] * 1e-9 for f in frames])
    return t - t[0]


def plot_clutter_removed_waterfall(frames: list[dict], path_idx: int, out_dir: Path,
                                   tap_resolution_cm: float = 15.0) -> None:
    valid = [f for f in frames if cir_magnitude(f).shape[0] > path_idx]
    if not valid:
        return

    mags = np.stack([cir_magnitude(f)[path_idx] for f in valid], axis=0).astype(np.float32)
    background = mags.mean(axis=0)
    clutter_removed = np.abs(mags - background)

    # dB scale — compresses dynamic range so weak reflections become visible
    clutter_db = 20.0 * np.log10(clutter_removed + 1.0)

    time_s = _timestamps_s(valid)
    total_time_s = float(time_s[-1])
    n_frames, n_taps = clutter_db.shape
    max_dist_m = n_taps * tap_resolution_cm / 100.0

    dt_half = np.diff(time_s) / 2.0
    time_edges = np.concatenate([
        [time_s[0] - (dt_half[0] if len(dt_half) else 0)],
        time_s[:-1] + dt_half,
        [time_s[-1] + (dt_half[-1] if len(dt_half) else 0)],
    ])
    dist_edges = np.arange(n_taps + 1) * tap_resolution_cm / 100.0

    # clip colorscale to [5th, 99th] percentile so a single bright outlier
    # doesn't flatten everything else to the bottom of the colormap
    vmin = float(np.percentile(clutter_db, 5))
    vmax = float(np.percentile(clutter_db, 99))

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.pcolormesh(dist_edges, time_edges, clutter_db,
                       cmap="plasma", shading="flat", vmin=vmin, vmax=vmax)
    plt.colorbar(im, ax=ax, label="ΔCIR magnitude (dB)")
    ax.set_xlabel("Distance (m)", fontsize=11)
    ax.set_ylabel("Time (s)", fontsize=11)
    rx = valid[0]["rx_antenna_ids"][path_idx]
    tx = valid[0]["tx_antenna_ids"][path_idx]
    ax.set_title(
        f"Clutter-removed CIR (dB) — RX path {path_idx}  (ant RX={rx} TX={tx})\n"
        f"{n_frames} frames, {total_time_s:.1f} s  |  vmin={vmin:.1f} dB  vmax={vmax:.1f} dB",
        fontsize=11,
    )
    plt.tight_layout()
    out = out_dir / f"cir_clutter_removed_path{path_idx}.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")
    plt.show()


def plot_waterfall(frames: list[dict], path_idx: int, out_dir: Path,
                   tap_resolution_cm: float = 15.0,
                   frame_interval_ms: float = 100.0) -> None:
    valid = [f for f in frames if cir_magnitude(f).shape[0] > path_idx]
    if not valid:
        print(f"[waterfall] path_idx={path_idx} exceeds all frames — skipping.")
        return
    if len(valid) < len(frames):
        print(f"[waterfall] path_idx={path_idx}: skipped {len(frames)-len(valid)} frames with fewer paths.")

    mags = np.stack([cir_magnitude(f)[path_idx] for f in valid], axis=0)
    n_frames, n_taps = mags.shape

    time_s = _timestamps_s(valid)
    total_time_s = float(time_s[-1])
    avg_interval_ms = (total_time_s / (n_frames - 1) * 1000.0) if n_frames > 1 else 0.0

    max_dist_m = n_taps * tap_resolution_cm / 100.0

    # Build cell edges so each row is anchored to its real timestamp.
    dist_edges = np.arange(n_taps + 1) * tap_resolution_cm / 100.0
    dt_half = np.diff(time_s) / 2.0
    time_edges = np.concatenate([
        [time_s[0] - (dt_half[0] if len(dt_half) else 0)],
        time_s[:-1] + dt_half,
        [time_s[-1] + (dt_half[-1] if len(dt_half) else 0)],
    ])

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.pcolormesh(dist_edges, time_edges, mags, cmap="viridis", shading="flat")
    plt.colorbar(im, ax=ax, label="CIR magnitude")

    ax.set_xlabel("Distance (m)", fontsize=11)
    ax.set_ylabel("Time (s)", fontsize=11)

    ax_top = ax.twiny()
    ax_top.set_xlim(0, n_taps)
    ax_top.set_xlabel("Tap index", fontsize=9, color="gray")
    ax_top.tick_params(axis="x", labelcolor="gray", labelsize=8)

    ax_right = ax.twinx()
    ax_right.set_ylim(time_edges[0], time_edges[-1])
    ax_right.set_yticks(time_s[::max(1, n_frames // 10)])
    ax_right.set_yticklabels([str(i) for i in range(0, n_frames, max(1, n_frames // 10))])
    ax_right.set_ylabel("Frame index", fontsize=9, color="gray")
    ax_right.tick_params(axis="y", labelcolor="gray", labelsize=8)

    rx = valid[0]["rx_antenna_ids"][path_idx]
    tx = valid[0]["tx_antenna_ids"][path_idx]
    ax.set_title(
        f"CIR waterfall — RX path {path_idx}  (ant RX={rx} TX={tx})\n"
        f"{n_frames} frames, {total_time_s:.1f} s  (avg {avg_interval_ms:.1f} ms/frame)  |  "
        f"{n_taps} taps × {tap_resolution_cm:.0f} cm/tap = {max_dist_m:.1f} m max",
        fontsize=11,
    )
    plt.tight_layout()
    out = out_dir / f"cir_waterfall_path{path_idx}.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")
    plt.show()


def plot_peak_tap_over_time(frames: list[dict], out_dir: Path,
                            tap_resolution_cm: float = 15.0,
                            frame_interval_ms: float = 100.0) -> None:
    n_paths = frames[0]["taps_real"].shape[0]
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    time_axis_s = _timestamps_s(frames)

    for i in range(n_paths):
        valid_idx = [j for j, f in enumerate(frames) if cir_magnitude(f).shape[0] > i]
        peaks_tap = [int(np.argmax(cir_magnitude(frames[j])[i])) for j in valid_idx]
        peaks_m = [p * tap_resolution_cm / 100.0 for p in peaks_tap]
        t = time_axis_s[valid_idx]
        rx = frames[0]["rx_antenna_ids"][i] if i < len(frames[0]["rx_antenna_ids"]) else "?"
        tx = frames[0]["tx_antenna_ids"][i] if i < len(frames[0]["tx_antenna_ids"]) else "?"

        axes[0].plot(t, peaks_m, label=f"path {i} (RX={rx} TX={tx})", linewidth=0.9)
        axes[1].plot(t, peaks_tap, label=f"path {i} (RX={rx} TX={tx})", linewidth=0.9)

    axes[0].set_title("Peak distance over time (all RX paths)", fontsize=13)
    axes[0].set_ylabel("Peak distance (m)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_title("Peak tap index over time (all RX paths)", fontsize=11)
    axes[1].set_ylabel("Peak tap index")
    axes[1].set_xlabel("Time (s)", fontsize=11)
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out = out_dir / "cir_peak_tap.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

BAG_PATH        = Path("/home/ws/src/uwb_rosbags/walking")
TOPIC           = "/uwb/frame_raw"
TAP_RES_CM      = 15.0
FRAME_INTERVAL  = 100.0
OUT_DIR         = None   # None → usa el mismo directorio que el bag


def main() -> None:
    bag_path = BAG_PATH
    if not bag_path.exists():
        print(f"Error: bag path not found: {bag_path}")
        sys.exit(1)

    out_dir = Path(OUT_DIR) if OUT_DIR else bag_path
    out_dir.mkdir(parents=True, exist_ok=True)

    frames = load_frames_from_bag(bag_path, topic=TOPIC)
    if not frames:
        print("No CIR frames found in bag.")
        sys.exit(1)

    print_summary(frames)

    plot_single_frame(frames[0], frame_idx=0, out_dir=out_dir,
                      tap_resolution_cm=TAP_RES_CM)

    n_paths = frames[0]["taps_real"].shape[0]
    for p in range(n_paths):
        plot_waterfall(frames, path_idx=p, out_dir=out_dir,
                       tap_resolution_cm=TAP_RES_CM,
                       frame_interval_ms=FRAME_INTERVAL)
        plot_clutter_removed_waterfall(frames, path_idx=p, out_dir=out_dir,
                                       tap_resolution_cm=TAP_RES_CM)

    plot_peak_tap_over_time(frames, out_dir=out_dir,
                            tap_resolution_cm=TAP_RES_CM,
                            frame_interval_ms=FRAME_INTERVAL)


if __name__ == "__main__":
    main()
