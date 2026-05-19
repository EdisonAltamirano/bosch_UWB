#!/usr/bin/env python3
"""
Live range-Doppler viewer for /uwb/frame_raw.

Subscribes to CIR frames, keeps a rolling slow-time buffer, and refreshes a
heatmap on a timer (default 0.5 s). Intended for Docker on macOS with XQuartz:
  DISPLAY=host.docker.internal:0
  xhost +localhost   (on the Mac host)

Run (after colcon build):
  ros2 run sensors uwb_live_rdm_viewer
"""

from __future__ import annotations

import os
import sys
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Protocol

# Writable matplotlib cache (hostuser home may not be writable in Docker).
_MPL_DIR = os.environ.get("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ["MPLCONFIGDIR"] = _MPL_DIR
os.makedirs(_MPL_DIR, exist_ok=True)

import numpy as np
import rclpy
from rclpy.node import Node

from sensors.live_frame_parse import message_to_path_taps
from sensors.rdm_core import (
    compute_range_axis_m,
    compute_range_doppler_map,
    estimate_frame_rate_hz,
    preprocess_buffer,
)

try:
    from sensors_interfaces.msg import UwbFrame as UwbFrameMsg
except ImportError:
    try:
        from sensors_interfaces.msg import UWBFrame as UwbFrameMsg
    except ImportError:
        from sensors_interfaces.msg import UWB_Frame as UwbFrameMsg  # type: ignore


class _Display(Protocol):
    def show(self, plot_data: np.ndarray, vmin: float, vmax: float, title: str) -> None: ...

    def show_buffering(self, message: str) -> None: ...

    def close(self) -> None: ...


class _OpenCvDisplay:
    _WINDOW = "UWB Live Range-Doppler"

    def __init__(self) -> None:
        import cv2

        self._cv2 = cv2
        cv2.namedWindow(self._WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._WINDOW, 960, 540)

    def show(self, plot_data: np.ndarray, vmin: float, vmax: float, title: str) -> None:
        span = max(vmax - vmin, 1e-6)
        norm = np.clip((plot_data - vmin) / span, 0.0, 1.0)
        gray = (norm * 255.0).astype(np.uint8)
        gray = np.flipud(gray)  # match matplotlib origin="lower"
        colored = self._cv2.applyColorMap(gray, self._cv2.COLORMAP_PLASMA)
        self._cv2.imshow(self._WINDOW, colored)
        self._cv2.setWindowTitle(self._WINDOW, title)
        self._cv2.waitKey(1)

    def show_buffering(self, message: str) -> None:
        canvas = np.zeros((240, 640, 3), dtype=np.uint8)
        self._cv2.putText(
            canvas,
            message,
            (20, 120),
            self._cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            self._cv2.LINE_AA,
        )
        self._cv2.imshow(self._WINDOW, canvas)
        self._cv2.setWindowTitle(self._WINDOW, "UWB Live Range-Doppler")
        self._cv2.waitKey(1)

    def close(self) -> None:
        self._cv2.destroyAllWindows()


class _MatplotlibDisplay:
    def __init__(self, backend: str) -> None:
        import matplotlib.pyplot as plt

        self._plt = plt
        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(10.0, 6.0))
        self._im = None
        self._backend = backend
        self._ax.set_title("Waiting for CIR frames…")
        self._ax.set_xlabel("Radial velocity (m/s)")
        self._ax.set_ylabel("Range (m)")
        self._fig.tight_layout()
        plt.show(block=False)

    def show(
        self,
        plot_data: np.ndarray,
        vmin: float,
        vmax: float,
        title: str,
        extent: list[float],
    ) -> None:
        plt = self._plt
        if self._im is None:
            self._ax.clear()
            self._im = self._ax.imshow(
                plot_data,
                aspect="auto",
                origin="lower",
                extent=extent,
                cmap="plasma",
                vmin=vmin,
                vmax=vmax,
            )
            self._ax.axvline(0, color="white", linewidth=0.6, alpha=0.5, linestyle="--")
            self._fig.colorbar(self._im, ax=self._ax, label="Magnitude (dB)")
        else:
            self._im.set_data(plot_data)
            self._im.set_extent(extent)
            self._im.set_clim(vmin, vmax)

        self._ax.set_title(title)
        self._ax.set_xlabel("Radial velocity (m/s)  [negative = approaching]")
        self._ax.set_ylabel("Range (m)")
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()
        plt.pause(0.001)

    def show_buffering(self, message: str) -> None:
        self._ax.set_title(message)
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

    def close(self) -> None:
        self._plt.close(self._fig)


def _tkinter_available() -> bool:
    try:
        import importlib.util

        return importlib.util.find_spec("_tkinter") is not None
    except Exception:
        return False


def _try_matplotlib_backend(requested: str) -> tuple[str, _MatplotlibDisplay]:
    import matplotlib

    candidates: list[str] = []
    if requested and requested.lower() not in {"auto", ""}:
        candidates.append(requested)
    candidates.extend(["TkAgg", "Qt5Agg", "GTK3Agg"])

    errors: list[str] = []
    for backend in candidates:
        try:
            matplotlib.use(backend, force=True)
            display = _MatplotlibDisplay(backend)
            return backend, display
        except Exception as exc:
            errors.append(f"{backend}: {exc}")
    hint = "Rebuild image: make uwb.build (installs python3-tk)." if not _tkinter_available() else ""
    raise RuntimeError("matplotlib backends failed: " + "; ".join(errors) + (" " + hint if hint else ""))


def _try_opencv_display() -> _OpenCvDisplay:
    if not os.environ.get("DISPLAY", ""):
        raise RuntimeError("DISPLAY is not set")
    return _OpenCvDisplay()


def _init_display(mode: str, mpl_backend: str, logger) -> tuple[str, _Display]:
    display_env = os.environ.get("DISPLAY", "")
    if not display_env:
        raise RuntimeError(
            "DISPLAY is not set inside the container. On macOS: start XQuartz, run "
            "'make uwb.xhost' on the host, and use compose.mac.yaml (DISPLAY=host.docker.internal:0)."
        )

    errors: list[str] = []
    order: list[str]
    if mode == "matplotlib":
        order = ["matplotlib"]
    elif mode == "opencv":
        order = ["opencv"]
    else:
        # Prefer OpenCV on Docker/Mac — fewer deps than TkAgg.
        order = ["opencv", "matplotlib"]

    for choice in order:
        try:
            if choice == "opencv":
                return "opencv", _try_opencv_display()
            backend, mpl_display = _try_matplotlib_backend(mpl_backend)
            return f"matplotlib/{backend}", mpl_display
        except Exception as exc:
            errors.append(f"{choice}: {exc}")
            logger.warning(str(exc))

    raise RuntimeError(
        "No live display backend available.\n"
        + "\n".join(errors)
        + "\nHost checklist: XQuartz running → make uwb.xhost → make uwb.build → make uwb.up"
    )


@dataclass(slots=True)
class _BufferedFrame:
    timestamp_s: float
    taps: np.ndarray


class UWBLiveRdmViewer(Node):
    def __init__(self) -> None:
        super().__init__("uwb_live_rdm_viewer")

        self.declare_parameter("topic_name", "/uwb/frame_raw")
        self.declare_parameter("refresh_period_s", 0.5)
        self.declare_parameter("doppler_window_s", 4.0)
        self.declare_parameter("min_frames", 8)
        self.declare_parameter("default_range_min_m", 0.3)
        self.declare_parameter("default_range_max_m", 6.0)
        self.declare_parameter("wall_clip_m", 0.3)
        self.declare_parameter("tap_spacing_m", 0.15)
        self.declare_parameter("carrier_frequency_hz", 6.5e9)
        self.declare_parameter("matplotlib_backend", "auto")
        self.declare_parameter("display_mode", "auto")

        self._topic = self.get_parameter("topic_name").value
        self._refresh_s = float(self.get_parameter("refresh_period_s").value)
        self._doppler_window_s = float(self.get_parameter("doppler_window_s").value)
        self._min_frames = int(self.get_parameter("min_frames").value)
        self._range_gate = (
            float(self.get_parameter("default_range_min_m").value),
            float(self.get_parameter("default_range_max_m").value),
        )
        self._wall_clip_m = float(self.get_parameter("wall_clip_m").value)
        self._tap_spacing_m = float(self.get_parameter("tap_spacing_m").value)
        self._carrier_hz = float(self.get_parameter("carrier_frequency_hz").value)
        mpl_backend = str(self.get_parameter("matplotlib_backend").value)
        display_mode = str(self.get_parameter("display_mode").value)

        max_frames = max(self._min_frames, int(round(self._doppler_window_s * 15.0)) + 4)
        self._buffer: Deque[_BufferedFrame] = deque(maxlen=max_frames)
        self._frame_count = 0
        self._vmin: Optional[float] = None
        self._vmax: Optional[float] = None
        self._last_extent: list[float] = [-1.0, 1.0, 0.0, 6.0]

        display_kind, self._display = _init_display(display_mode, mpl_backend, self.get_logger())
        self.get_logger().info(
            f"Live RDM viewer: topic={self._topic} refresh={self._refresh_s}s "
            f"window≈{self._doppler_window_s}s display={display_kind} "
            f"DISPLAY={os.environ.get('DISPLAY', '')!r} MPLCONFIGDIR={_MPL_DIR!r} "
            f"tkinter={_tkinter_available()}"
        )

        self._sub = self.create_subscription(
            UwbFrameMsg,
            self._topic,
            self._on_frame,
            50,
        )
        self._timer = self.create_timer(self._refresh_s, self._refresh_plot)

    def _on_frame(self, msg: UwbFrameMsg) -> None:
        taps = message_to_path_taps(msg)
        if taps is None:
            return
        timestamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        self._buffer.append(_BufferedFrame(timestamp_s=timestamp_s, taps=taps))
        self._frame_count += 1

    def _compute_rdm(self) -> tuple[np.ndarray, float, float, str] | None:
        if len(self._buffer) < self._min_frames:
            return None

        timestamps = np.asarray([frame.timestamp_s for frame in self._buffer], dtype=np.float64)
        timestamps = timestamps - timestamps[0]
        min_paths = min(frame.taps.shape[0] for frame in self._buffer)
        min_taps = min(frame.taps.shape[1] for frame in self._buffer)
        frames = np.stack(
            [frame.taps[:min_paths, :min_taps] for frame in self._buffer],
            axis=0,
        ).astype(np.complex64)

        range_axis = compute_range_axis_m(frames.shape[2], self._tap_spacing_m)
        frame_rate_hz = estimate_frame_rate_hz(timestamps)
        signal, selected_path = preprocess_buffer(
            frames,
            range_axis,
            self._range_gate,
            self._wall_clip_m,
        )

        win_frames = max(self._min_frames, int(round(self._doppler_window_s * frame_rate_hz)))
        if signal.shape[0] > win_frames:
            signal = signal[-win_frames:, :]

        rdm_db, velocity_ms = compute_range_doppler_map(signal, frame_rate_hz, self._carrier_hz)
        range_valid = range_axis >= self._wall_clip_m
        plot_data = rdm_db[:, range_valid].T

        p2, p98 = np.percentile(plot_data, [2, 98])
        if self._vmin is None:
            self._vmin, self._vmax = float(p2), float(p98)
        else:
            alpha = 0.25
            self._vmin = alpha * float(p2) + (1.0 - alpha) * self._vmin
            self._vmax = alpha * float(p98) + (1.0 - alpha) * self._vmax

        self._last_extent = [
            float(velocity_ms[0]),
            float(velocity_ms[-1]),
            float(range_axis[range_valid][0]),
            float(range_axis[range_valid][-1]),
        ]
        title = (
            f"Live RDM path {selected_path} | {signal.shape[0]} fr @ {frame_rate_hz:.1f} Hz"
        )
        return plot_data, self._vmin, self._vmax, title

    def _refresh_plot(self) -> None:
        result = self._compute_rdm()
        if result is None:
            msg = (
                f"Buffering {len(self._buffer)}/{self._min_frames} frames "
                f"(rx={self._frame_count})"
            )
            self._display.show_buffering(msg)
            return

        plot_data, vmin, vmax, title = result
        if isinstance(self._display, _MatplotlibDisplay):
            self._display.show(plot_data, vmin, vmax, title, self._last_extent)
        else:
            self._display.show(plot_data, vmin, vmax, title)

    def destroy_node(self) -> None:
        try:
            self._display.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[UWBLiveRdmViewer] = None
    try:
        node = UWBLiveRdmViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
