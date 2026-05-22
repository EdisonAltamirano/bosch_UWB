from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np


JET_CMAP = plt.get_cmap("jet")


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


def db_limits(values: np.ndarray, percentile: float = 99.5, dynamic_range_db: float = 38.0) -> tuple[float, float]:
    finite = np.asarray(values, dtype=np.float32)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        vmax = 0.0
    else:
        vmax = float(np.percentile(finite, percentile))
    return vmax - dynamic_range_db, vmax
