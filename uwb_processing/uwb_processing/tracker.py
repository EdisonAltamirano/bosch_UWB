from __future__ import annotations

import numpy as np

from .types import CfarDetection, Track, TrackingConfig


class MultiTargetTracker:
    """Constant-velocity Kalman filter tracker with nearest-neighbour gating.

    Lifecycle: Tentative → Confirmed (hit_count >= confirm_hits)
               Confirmed → Deleted  (miss_count >= max_misses)
               Tentative → Deleted  (not confirmed within init_window_frames)
    """

    def __init__(self, cfg: TrackingConfig) -> None:
        self.cfg = cfg
        self._tracks: list[Track] = []
        self._next_id: int = 0
        # Each entry: {hits, start_frame, last_frame, last_range, last_ts}
        self._pool: list[dict] = []

    # ------------------------------------------------------------------
    # State transition and noise matrices
    # ------------------------------------------------------------------

    def _F(self) -> np.ndarray:
        dt = self.cfg.dt_s
        return np.array([[1.0, dt], [0.0, 1.0]])

    def _Q(self) -> np.ndarray:
        dt = self.cfg.dt_s
        sr, sv = self.cfg.sigma_process_m, self.cfg.sigma_process_v
        return np.diag([sr ** 2 * dt, sv ** 2 * dt])

    _H = np.array([[1.0, 0.0]])   # measurement matrix (range only)

    def _R(self) -> np.ndarray:
        return np.array([[self.cfg.sigma_meas_m ** 2]])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        frame_idx: int,
        timestamp_s: float,
        detections: list[CfarDetection],
    ) -> list[Track]:
        """Process one frame. Returns all currently confirmed tracks."""
        F, Q, H, R = self._F(), self._Q(), self._H, self._R()

        # 1. Predict
        for track in self._tracks:
            track.state = F @ track.state
            track.covariance = F @ track.covariance @ F.T + Q

        # 2. Associate — greedy nearest-neighbour with gate
        meas_ranges = [d.range_m for d in detections]
        assigned_tracks: set[int] = set()
        assigned_dets: set[int] = set()

        if self._tracks and meas_ranges:
            cost = np.full((len(self._tracks), len(meas_ranges)), np.inf)
            for ti, track in enumerate(self._tracks):
                pred_r = float(track.state[0])
                for di, r in enumerate(meas_ranges):
                    gap = abs(pred_r - r)
                    if gap <= self.cfg.gate_distance_m:
                        cost[ti, di] = gap

            while True:
                flat = np.argmin(cost)
                ti, di = int(flat // cost.shape[1]), int(flat % cost.shape[1])
                if cost[ti, di] == np.inf:
                    break
                assigned_tracks.add(ti)
                assigned_dets.add(di)
                cost[ti, :] = np.inf
                cost[:, di] = np.inf

                track = self._tracks[ti]
                y = meas_ranges[di] - float((H @ track.state)[0])
                S = float((H @ track.covariance @ H.T + R)[0, 0])
                K = (track.covariance @ H.T) / S          # (2, 1)
                track.state = track.state + K[:, 0] * y
                track.covariance = (np.eye(2) - K @ H) @ track.covariance
                track.hit_count += 1
                track.miss_count = 0
                if track.hit_count >= self.cfg.confirm_hits:
                    track.confirmed = True
                track.history_m.append(float(track.state[0]))
                track.history_t.append(timestamp_s)

        # 3. Increment miss counter for unassigned tracks
        for ti, track in enumerate(self._tracks):
            if ti not in assigned_tracks:
                track.miss_count += 1

        # 4. Tentative track initiation from unassigned detections
        for di, det in enumerate(detections):
            if di not in assigned_dets:
                self._try_init(det, frame_idx, timestamp_s)

        # 5. Delete tracks that exceeded the miss budget
        self._tracks = [t for t in self._tracks if t.miss_count < self.cfg.max_misses]

        return [t for t in self._tracks if t.confirmed]

    def tracks_snapshot(self) -> list[Track]:
        return [t for t in self._tracks if t.confirmed]

    # ------------------------------------------------------------------
    # Tentative track pool
    # ------------------------------------------------------------------

    def _try_init(self, det: CfarDetection, frame_idx: int, timestamp_s: float) -> None:
        cfg = self.cfg
        # Look for a nearby pool entry within the initiation window
        for entry in self._pool:
            age = frame_idx - entry["last_frame"]
            if age > cfg.init_window_frames:
                continue
            if abs(det.range_m - entry["last_range"]) <= cfg.gate_distance_m:
                entry["hits"] += 1
                entry["last_range"] = det.range_m
                entry["last_frame"] = frame_idx
                entry["last_ts"] = timestamp_s
                if entry["hits"] >= cfg.init_hits:
                    self._promote(det, entry, timestamp_s)
                return

        # No matching entry — start a new tentative entry
        self._pool.append({
            "hits": 1,
            "start_frame": frame_idx,
            "last_frame": frame_idx,
            "last_range": det.range_m,
            "last_ts": timestamp_s,
        })
        # Prune stale pool entries
        self._pool = [
            e for e in self._pool
            if frame_idx - e["last_frame"] <= cfg.init_window_frames
        ]

    def _promote(self, det: CfarDetection, entry: dict, timestamp_s: float) -> None:
        track = Track(
            track_id=self._next_id,
            state=np.array([det.range_m, 0.0]),
            covariance=np.diag([self.cfg.sigma_meas_m ** 2, self.cfg.sigma_process_v ** 2]),
        )
        self._next_id += 1
        track.hit_count = entry["hits"]
        track.history_m.append(det.range_m)
        track.history_t.append(timestamp_s)
        self._tracks.append(track)
        self._pool.remove(entry)
