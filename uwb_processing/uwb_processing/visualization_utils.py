from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors as mcolors


JET_CMAP = plt.get_cmap("jet")
TRACK_PALETTE_HEX = (
    "#00B3B8",
    "#FF6B6B",
    "#FFD166",
    "#4D96FF",
    "#7BC043",
    "#9B5DE5",
    "#F9844A",
    "#43AA8B",
)
FOV_BACKGROUND = "#FBFDFF"
FOV_SECTOR_FILL = "#EAF2FF"
FOV_SECTOR_EDGE = "#62759C"
FOV_GRID = "#B7C2D0"
FOV_TEXT = "#1F2A44"


def magnitude_to_db(values: np.ndarray, floor: float = 1e-6) -> np.ndarray:
    magnitude = np.maximum(np.asarray(values, dtype=np.float32), np.float32(floor))
    return (20.0 * np.log10(magnitude)).astype(np.float32)


def power_ratio_to_db(values: np.ndarray, floor: float = 1e-6) -> np.ndarray:
    power_ratio = np.maximum(np.asarray(values, dtype=np.float32), np.float32(floor))
    return (10.0 * np.log10(power_ratio)).astype(np.float32)


def jet_color(index: int, total: int) -> tuple[float, float, float, float]:
    if total <= 1:
        position = 0.5
    else:
        position = float(index) / float(total - 1)
    return JET_CMAP(position)


def track_color(index: int) -> tuple[float, float, float, float]:
    return mcolors.to_rgba(TRACK_PALETTE_HEX[index % len(TRACK_PALETTE_HEX)])


def with_alpha(color: str | tuple[float, float, float, float], alpha: float) -> tuple[float, float, float, float]:
    rgba = list(mcolors.to_rgba(color))
    rgba[3] = float(alpha)
    return tuple(rgba)


def db_limits(values: np.ndarray, percentile: float = 99.5, dynamic_range_db: float = 38.0) -> tuple[float, float]:
    finite = np.asarray(values, dtype=np.float32)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        vmax = 0.0
    else:
        vmax = float(np.percentile(finite, percentile))
    return vmax - dynamic_range_db, vmax
