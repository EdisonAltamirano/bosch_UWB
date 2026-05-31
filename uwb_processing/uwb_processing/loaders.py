from __future__ import annotations

from pathlib import Path
from typing import Iterable
import warnings

import numpy as np

from .types import RadarSession


def load_session(path: str | Path, topic: str = "/uwb/frame_raw") -> RadarSession:
    source_path = _resolve_session_source(Path(path))
    if source_path.is_dir() and (source_path / "metadata.yaml").exists():
        return _load_rosbag_directory(source_path, topic=topic)
    if source_path.is_dir() and _is_new_format_directory(source_path):
        return _load_new_format_directory(source_path)
    if source_path.is_dir():
        return _load_npz_directory(source_path)
    raise FileNotFoundError(f"Unsupported input path: {source_path}")


def _resolve_session_source(source_path: Path) -> Path:
    if source_path.exists():
        return source_path

    search_roots = _candidate_workspace_roots()
    rosbag_matches: list[Path] = []
    npz_matches: list[Path] = []

    for root in search_roots:
        direct_candidate = root / source_path
        if direct_candidate.exists():
            if direct_candidate.is_dir() and (direct_candidate / "metadata.yaml").exists():
                rosbag_matches.append(direct_candidate)
            elif direct_candidate.is_dir():
                npz_matches.append(direct_candidate)

        rosbag_candidate = root / "uwb_rosbags" / source_path
        if rosbag_candidate.is_dir() and (rosbag_candidate / "metadata.yaml").exists():
            rosbag_matches.append(rosbag_candidate)

        npz_candidate = root / "uwb_npz" / source_path
        if npz_candidate.is_dir():
            npz_matches.append(npz_candidate)

    if rosbag_matches:
        return rosbag_matches[0]
    if npz_matches:
        return npz_matches[0]
    return source_path


def _candidate_workspace_roots() -> list[Path]:
    cwd = Path.cwd().resolve()
    module_root = Path(__file__).resolve().parents[2]

    candidates = [cwd, cwd / "src", module_root]
    unique_candidates: list[Path] = []
    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique_candidates.append(resolved)
    return unique_candidates


def _is_new_format_directory(source_dir: Path) -> bool:
    raw_dir = source_dir / "raw"
    if raw_dir.is_dir() and any(raw_dir.glob("frame_*.npy")):
        return True
    return any(source_dir.glob("frame_*.npy"))


def _new_format_frame_files(source_dir: Path) -> list[Path]:
    raw_dir = source_dir / "raw"
    search_dir = raw_dir if raw_dir.is_dir() else source_dir
    return sorted(search_dir.glob("frame_*.npy"))


def _load_new_format_directory(source_dir: Path, logical_frame_interval_s: float = 0.096) -> RadarSession:
    """Load flattened SR250 CIR sample-block dumps stored as frame_*.npy files.

    Expected layout:
    - either <session>/raw/frame_000001.npy, ... or <session>/frame_000001.npy, ...
    - each file contains a flat int16 array representing many sample blocks
    - one sample block = 128 complex int16 values = 8 metadata words + 120 CIR taps
    """
    try:
        from sensors.sr250_protocol import parse_metadata_block
    except ImportError:
        from sensors.sensors.sr250_protocol import parse_metadata_block  # type: ignore

    files = _new_format_frame_files(source_dir)
    if not files:
        raise FileNotFoundError(f"No frame_*.npy files found in {source_dir}")

    sample_blocks_by_counter: dict[int, list[dict]] = {}
    sample_order = 0
    taps_per_sample = 128
    metadata_taps = 8
    cir_taps = taps_per_sample - metadata_taps

    for file_path in files:
        flat = np.load(file_path)
        flat = np.asarray(flat, dtype=np.int16).reshape(-1)
        ints_per_sample_block = taps_per_sample * 2
        if flat.size % ints_per_sample_block != 0:
            raise ValueError(
                f"{file_path} has {flat.size} int16 values; expected a multiple of {ints_per_sample_block}"
            )
        sample_blocks = flat.reshape(-1, taps_per_sample, 2)
        for block in sample_blocks:
            metadata_bytes = np.asarray(block[:metadata_taps, :], dtype="<i2").tobytes(order="C")
            metadata = parse_metadata_block(
                metadata_bytes,
                session_handle=0,
                status=0,
                radar_data_type=0,
                num_samples=0,
                taps_per_sample=taps_per_sample,
                rfu=0,
                sample_index=sample_order,
            )
            cir_pairs = np.asarray(block[metadata_taps:, :], dtype=np.float32)
            taps = cir_pairs[:, 0] + 1j * cir_pairs[:, 1]
            counter = int(metadata.cir_counter)
            sample_blocks_by_counter.setdefault(counter, []).append(
                {
                    "order": sample_order,
                    "rx_path": int(metadata.rx_path),
                    "rx_antenna_id": int(metadata.rx_antenna_id),
                    "tx_antenna_id": int(metadata.tx_antenna_id),
                    "rx_timestamp": int(metadata.rx_timestamp),
                    "cir_start_offset": int(metadata.cir_start_offset),
                    "taps": np.asarray(taps[:cir_taps], dtype=np.complex64),
                }
            )
            sample_order += 1

    frame_records: list[dict] = []
    for counter in sorted(sample_blocks_by_counter):
        samples = sample_blocks_by_counter[counter]
        samples.sort(key=lambda entry: (entry["rx_path"], entry["rx_antenna_id"], entry["order"]))
        frame_records.append(
            {
                "taps": np.stack([entry["taps"] for entry in samples], axis=0),
                "timestamp_s": float(counter) * logical_frame_interval_s,
                "cir_counter": int(counter),
                "cir_counters": np.asarray([counter] * len(samples), dtype=np.uint32),
                "rx_paths": np.asarray([entry["rx_path"] for entry in samples], dtype=np.uint8),
                "rx_timestamps": np.asarray([entry["rx_timestamp"] for entry in samples], dtype=np.uint32),
                "rx_antenna_ids": np.asarray([entry["rx_antenna_id"] for entry in samples], dtype=np.uint8),
                "tx_antenna_ids": np.asarray([entry["tx_antenna_id"] for entry in samples], dtype=np.uint8),
                "cir_start_offsets": np.asarray([entry["cir_start_offset"] for entry in samples], dtype=np.uint16),
                "bytes_per_tap": 4,
                "block_size": 512,
            }
        )

    if not frame_records:
        raise RuntimeError(f"No decodable CIR sample blocks found in {source_dir}")
    return _stack_frame_records(source_dir, "new_format", frame_records)


def _load_npz_directory(npz_dir: Path) -> RadarSession:
    files = sorted(npz_dir.glob("*.npz"))
    if not files:
        raise FileNotFoundError(f"No .npz files found in {npz_dir}")

    frame_records = []
    for index, file_path in enumerate(files):
        frame = np.load(file_path)
        taps_real = frame["taps_real"].astype(np.float32)
        taps_imag = frame["taps_imag"].astype(np.float32)
        stamp_sec = float(frame["stamp_sec"]) if "stamp_sec" in frame.files else float(index)
        stamp_nanosec = float(frame["stamp_nanosec"]) if "stamp_nanosec" in frame.files else 0.0
        timestamp_s = stamp_sec + stamp_nanosec * 1e-9
        frame_records.append(
            {
                "taps": taps_real + 1j * taps_imag,
                "timestamp_s": timestamp_s,
                "rx_antenna_ids": frame["rx_antenna_ids"],
                "tx_antenna_ids": frame["tx_antenna_ids"],
                "cir_start_offsets": frame["cir_start_offsets"],
                "cir_counters": frame["cir_counters"] if "cir_counters" in frame.files else None,
                "rx_paths": frame["rx_paths"] if "rx_paths" in frame.files else None,
                "rx_timestamps": frame["timestamps_rx"] if "timestamps_rx" in frame.files else None,
                "bytes_per_tap": int(frame["bytes_per_tap"]),
                "block_size": int(frame["block_size"]),
            }
        )
    normalized_records = _normalize_batched_frame_records(frame_records)
    return _stack_frame_records(npz_dir, "npz", normalized_records)


def _load_rosbag_directory(bag_dir: Path, topic: str) -> RadarSession:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        try:
            from sensors_interfaces.msg import UwbFrame as UwbFrameMsg
        except ImportError:
            from sensors_interfaces.msg import UWB_Frame as UwbFrameMsg  # type: ignore
        from sensors.sr250_protocol import parse_cir_udp_payload
    except ImportError as exc:
        raise RuntimeError(
            "rosbag loading requires a sourced ROS environment with rosbag2_py, "
            "sensors_interfaces, and sensors.sr250_protocol available. "
            "Run this inside the project ROS container or a sourced workspace."
        ) from exc

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    frame_records = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, UwbFrameMsg)
        if int(msg.radar_data_type) != 0x00:
            continue
        parsed = parse_cir_udp_payload(bytes(msg.raw_payload))
        num_samples = int(parsed["num_samples"])
        taps_per_block = parsed["actual_cir_taps_per_block"]
        if taps_per_block is None:
            taps_per_block = len(parsed["samples"][0]["taps"]) if parsed["samples"] else 0
        taps = np.zeros((num_samples, taps_per_block), dtype=np.complex64)
        rx_antenna_ids = np.zeros((num_samples,), dtype=np.uint8)
        tx_antenna_ids = np.zeros((num_samples,), dtype=np.uint8)
        cir_start_offsets = np.zeros((num_samples,), dtype=np.uint16)
        cir_counters = np.zeros((num_samples,), dtype=np.uint32)
        rx_paths = np.zeros((num_samples,), dtype=np.uint8)
        rx_timestamps = np.zeros((num_samples,), dtype=np.uint32)
        for sample_index, sample in enumerate(parsed["samples"]):
            meta = sample["metadata"]
            cir_counters[sample_index] = meta["cir_counter"]
            rx_paths[sample_index] = meta["rx_path"]
            rx_timestamps[sample_index] = meta["rx_timestamp"]
            rx_antenna_ids[sample_index] = meta["rx_antenna_id"]
            tx_antenna_ids[sample_index] = meta["tx_antenna_id"]
            cir_start_offsets[sample_index] = meta["cir_start_offset"]
            taps[sample_index, :] = np.asarray(
                [complex(real_part, imag_part) for real_part, imag_part in sample["taps"]],
                dtype=np.complex64,
            )
        frame_records.append(
            {
                "taps": taps,
                "timestamp_s": float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9,
                "cir_counters": cir_counters,
                "rx_paths": rx_paths,
                "rx_timestamps": rx_timestamps,
                "rx_antenna_ids": rx_antenna_ids,
                "tx_antenna_ids": tx_antenna_ids,
                "cir_start_offsets": cir_start_offsets,
                "bytes_per_tap": int(parsed["bytes_per_tap"]),
                "block_size": int(parsed["block_size"]),
            }
        )
    if not frame_records:
        raise RuntimeError(f"No CIR frames found in {bag_dir}")
    normalized_records = _normalize_batched_frame_records(frame_records)
    return _stack_frame_records(bag_dir, "rosbag", normalized_records)


def _normalize_batched_frame_records(frame_records: Iterable[dict]) -> list[dict]:
    records = list(frame_records)
    if not records:
        return []
    if not any(record.get("cir_counters") is not None for record in records):
        return records

    logical_records: list[dict] = []
    anchor_counters: list[int] = []
    anchor_times_s: list[float] = []

    for record in records:
        taps = np.asarray(record["taps"])
        cir_counters = record.get("cir_counters")
        if cir_counters is None:
            logical_records.append(record)
            continue

        cir_counters_arr = np.asarray(cir_counters)
        if cir_counters_arr.ndim != 1 or cir_counters_arr.size != taps.shape[0]:
            warnings.warn(
                "Ignoring CIR counter metadata because it does not match the sample axis.",
                RuntimeWarning,
            )
            logical_records.append(record)
            continue

        unique_counters = list(dict.fromkeys(int(counter) for counter in cir_counters_arr.tolist()))
        anchor_counters.append(unique_counters[-1])
        anchor_times_s.append(float(record["timestamp_s"]))

        rx_paths_arr = _optional_array(record.get("rx_paths"))
        rx_ant_arr = np.asarray(record["rx_antenna_ids"])
        tx_ant_arr = np.asarray(record["tx_antenna_ids"])
        cir_start_arr = np.asarray(record["cir_start_offsets"])
        rx_timestamps_arr = _optional_array(record.get("rx_timestamps"))

        for counter in unique_counters:
            sample_indices = np.flatnonzero(cir_counters_arr == counter)
            order = _path_sort_order(rx_paths_arr, rx_ant_arr, sample_indices)
            sample_indices = sample_indices[order]
            logical_records.append(
                {
                    "taps": taps[sample_indices, :],
                    "timestamp_s": float(record["timestamp_s"]),
                    "source_timestamp_s": float(record["timestamp_s"]),
                    "cir_counter": int(counter),
                    "cir_counters": np.asarray(cir_counters_arr[sample_indices]),
                    "rx_paths": None if rx_paths_arr is None else np.asarray(rx_paths_arr[sample_indices]),
                    "rx_timestamps": None if rx_timestamps_arr is None else np.asarray(rx_timestamps_arr[sample_indices]),
                    "rx_antenna_ids": np.asarray(rx_ant_arr[sample_indices]),
                    "tx_antenna_ids": np.asarray(tx_ant_arr[sample_indices]),
                    "cir_start_offsets": np.asarray(cir_start_arr[sample_indices]),
                    "bytes_per_tap": int(record["bytes_per_tap"]),
                    "block_size": int(record["block_size"]),
                }
            )

    if anchor_counters:
        _assign_logical_timestamps_from_counters(logical_records, anchor_counters, anchor_times_s)
    return logical_records


def _optional_array(value):
    if value is None:
        return None
    return np.asarray(value)


def _path_sort_order(rx_paths: np.ndarray | None, rx_antennas: np.ndarray, sample_indices: np.ndarray) -> np.ndarray:
    if rx_paths is not None:
        return np.lexsort((rx_antennas[sample_indices], rx_paths[sample_indices]))
    return np.argsort(rx_antennas[sample_indices], kind="stable")


def _assign_logical_timestamps_from_counters(
    logical_records: list[dict],
    anchor_counters: list[int],
    anchor_times_s: list[float],
) -> None:
    if len(anchor_counters) < 2:
        return

    anchor_counters_arr = np.asarray(anchor_counters, dtype=np.float64)
    anchor_times_arr = np.asarray(anchor_times_s, dtype=np.float64)
    order = np.argsort(anchor_counters_arr, kind="stable")
    anchor_counters_arr = anchor_counters_arr[order]
    anchor_times_arr = anchor_times_arr[order]

    unique_mask = np.concatenate(([True], np.diff(anchor_counters_arr) > 0))
    anchor_counters_arr = anchor_counters_arr[unique_mask]
    anchor_times_arr = anchor_times_arr[unique_mask]
    if anchor_counters_arr.size < 2:
        return

    counter_deltas = np.diff(anchor_counters_arr)
    time_deltas = np.diff(anchor_times_arr)
    positive = counter_deltas > 0
    if not np.any(positive):
        return
    seconds_per_counter = float(np.median(time_deltas[positive] / counter_deltas[positive]))
    if seconds_per_counter <= 0:
        return

    for record in logical_records:
        counter = record.get("cir_counter")
        if counter is None:
            continue
        counter_value = float(counter)
        if counter_value <= anchor_counters_arr[0]:
            timestamp_s = anchor_times_arr[0] - (anchor_counters_arr[0] - counter_value) * seconds_per_counter
        elif counter_value >= anchor_counters_arr[-1]:
            timestamp_s = anchor_times_arr[-1] + (counter_value - anchor_counters_arr[-1]) * seconds_per_counter
        else:
            timestamp_s = float(np.interp(counter_value, anchor_counters_arr, anchor_times_arr))
        record["timestamp_s"] = timestamp_s


def _stack_frame_records(source_path: Path, source_kind: str, frame_records: Iterable[dict]) -> RadarSession:
    frames = list(frame_records)
    min_paths = min(record["taps"].shape[0] for record in frames)
    min_taps = min(record["taps"].shape[1] for record in frames)
    if any(record["taps"].shape != (min_paths, min_taps) for record in frames):
        warnings.warn(
            "Inconsistent frame shapes detected; cropping all frames to the smallest common shape.",
            RuntimeWarning,
        )

    stacked_frames = np.stack(
        [record["taps"][:min_paths, :min_taps] for record in frames],
        axis=0,
    ).astype(np.complex64)
    timestamps_s = np.asarray([record["timestamp_s"] for record in frames], dtype=np.float64)
    timestamps_s -= timestamps_s[0]
    first = frames[0]
    return RadarSession(
        source_path=source_path,
        source_kind=source_kind,
        frames=stacked_frames,
        timestamps_s=timestamps_s,
        rx_antenna_ids=np.asarray(first["rx_antenna_ids"][:min_paths]),
        tx_antenna_ids=np.asarray(first["tx_antenna_ids"][:min_paths]),
        cir_start_offsets=np.asarray(first["cir_start_offsets"][:min_paths]),
        bytes_per_tap=int(first["bytes_per_tap"]),
        block_size=int(first["block_size"]),
        metadata={
            "frame_count": int(stacked_frames.shape[0]),
            "path_count": int(stacked_frames.shape[1]),
            "tap_count": int(stacked_frames.shape[2]),
        },
    )
