from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from .types import RangeDopplerFrame

_RETR_SRC = Path(__file__).resolve().parents[2] / "radar-detection-transformer" / "src"


class RETRDetector:
    """Zero-shot RETR inference adapter for UWB range-Doppler frames.

    RETR was trained on 77 GHz FMCW MMVR data.  UWB input is adapted by:
    - log-compressing each RD grid
    - resizing to (256, 128) to match RETR's radar-plane dimensions
    - normalising with session-level mean/std (not per-frame)
    - duplicating the same tensor for both hm_hori and hm_vert
    """

    def __init__(
        self,
        weights_path: str | Path,
        device: str = "cpu",
        n_frames: int = 4,
        score_thresh: float = 0.5,
        zero_doppler_thresh_mps: float = 0.08,
    ) -> None:
        weights_path = Path(weights_path)
        if not weights_path.exists():
            raise FileNotFoundError(f"RETR weights not found: {weights_path}")

        src_str = str(_RETR_SRC)
        if src_str not in sys.path:
            sys.path.insert(0, src_str)

        from models.retr import RETR  # type: ignore[import]

        self.device = torch.device(device)
        self.n_frames = n_frames
        self.score_thresh = score_thresh
        self.zero_doppler_thresh_mps = zero_doppler_thresh_mps

        print(f"[RETR] loading weights from {weights_path.name}  thresh={score_thresh}")
        self._model = RETR(task="SEG", thresh_mask=score_thresh).to(self.device)

        state = torch.load(str(weights_path), map_location=device, weights_only=False)
        if isinstance(state, dict) and "model" in state:
            state = state["model"]

        missing, unexpected = self._model.load_state_dict(state, strict=False)
        if missing:
            print(f"[RETR]   missing keys : {len(missing)}")
        if unexpected:
            print(f"[RETR]   unexpected keys: {len(unexpected)}")

        self._model.eval()
        print(f"[RETR] model ready on {device}  (hm_hori = hm_vert approximation)")

    # ------------------------------------------------------------------
    def detect(
        self,
        rd_frames: list[RangeDopplerFrame],
    ) -> list[list[tuple[float, float, float]]]:
        """Run RETR on every RD frame and return (range_m, vel_mps, score) lists."""
        if not rd_frames:
            return []

        prepared = [self._prepare_single(rdf) for rdf in rd_frames]

        session_mean, session_std = self._fit_norm(prepared)
        print(f"[RETR] session norm — mean={session_mean:.3f}  std={session_std:.3f}")

        out: list[list[tuple[float, float, float]]] = []
        total_dets = 0

        for i, rdf in enumerate(rd_frames):
            window = self._build_window(prepared, i, session_mean, session_std)
            with torch.no_grad():
                preds = self._model(window, window)  # hm_hori = hm_vert
            dets = self._decode(preds[0], rdf)
            out.append(dets)
            total_dets += len(dets)

            if i == 0 or i == len(rd_frames) - 1 or (i + 1) % 100 == 0:
                mean_dets = total_dets / (i + 1)
                print(
                    f"[RETR] frame {i + 1:4d}/{len(rd_frames)}"
                    f"  dets={len(dets)}"
                    f"  running mean={mean_dets:.2f}/frame"
                )

        return out

    # ------------------------------------------------------------------
    def _prepare_single(self, rdf: RangeDopplerFrame) -> np.ndarray:
        g = rdf.grid.astype(np.float32)          # (D, T)
        g = np.maximum(g, 1e-6)
        g = np.log(g)
        g = np.where(np.isfinite(g), g, 0.0)
        t = torch.from_numpy(g).unsqueeze(0).unsqueeze(0)   # (1, 1, D, T)
        t = F.interpolate(t, size=(256, 128), mode="bilinear", align_corners=False)
        return t.squeeze().numpy()                # (256, 128)

    def _fit_norm(self, prepared: list[np.ndarray]) -> tuple[float, float]:
        all_vals = np.concatenate([p.ravel() for p in prepared])
        return float(all_vals.mean()), float(all_vals.std())

    def _build_window(
        self,
        prepared: list[np.ndarray],
        i: int,
        mean: float,
        std: float,
    ) -> torch.Tensor:
        pad = np.zeros((256, 128), dtype=np.float32)
        frames = []
        for offset in range(self.n_frames - 1, -1, -1):
            idx = i - offset
            raw = prepared[idx] if idx >= 0 else pad
            normed = (raw - mean) / max(std, 1e-6)
            frames.append(torch.from_numpy(normed.astype(np.float32)))
        stacked = torch.stack(frames).unsqueeze(0)   # (1, n_frames, 256, 128)
        return stacked.to(self.device)

    def _decode(
        self,
        pred: dict,
        rdf: RangeDopplerFrame,
    ) -> list[tuple[float, float, float]]:
        scores = pred.get("scores")
        hboxes = pred.get("hboxes")

        if scores is None or hboxes is None or len(scores) == 0:
            return []

        scores_np = scores.cpu().numpy()
        hboxes_np = hboxes.cpu().numpy()   # (N, 4) xyxy  x∈[0,128]  y∈[0,256]

        range_axis = rdf.range_axis_m
        doppler_axis = rdf.doppler_axis_mps
        r_min, r_max = float(range_axis[0]),  float(range_axis[-1])
        v_min, v_max = float(doppler_axis[0]), float(doppler_axis[-1])

        detections: list[tuple[float, float, float]] = []
        for k in range(len(scores_np)):
            x1, y1, x2, y2 = hboxes_np[k]
            cx = 0.5 * (x1 + x2)   # column index → range axis
            cy = 0.5 * (y1 + y2)   # row index    → Doppler axis

            range_m = float(np.interp(cx, [0.0, 127.0], [r_min, r_max]))
            vel_mps = float(np.interp(cy, [0.0, 255.0], [v_min, v_max]))

            if not (np.isfinite(range_m) and np.isfinite(vel_mps)):
                continue
            range_m = float(np.clip(range_m, r_min, r_max))
            vel_mps = float(np.clip(vel_mps, v_min, v_max))

            if abs(vel_mps) < self.zero_doppler_thresh_mps:
                continue

            detections.append((range_m, vel_mps, float(scores_np[k])))

        return detections
