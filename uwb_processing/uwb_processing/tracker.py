from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment

from .types import CfarDetection, Track, TrackingConfig


class MultiTargetTracker:
    """EKF-based multi-target tracker.

    Implements the group tracking algorithm from Jiang & Guo IGARSS 2025
    (Sec. III-B) with three optional upgrades controlled by TrackingConfig:

    use_2d_ekf=True
        Cartesian state [x_m, y_m, vx_mps, vy_mps] with nonlinear polar
        measurement model h(x) = [r, az_rad, ṙ] and full EKF Jacobian.
        Requires CfarDetection.azimuth_deg populated before tracking
        (call annotate_detections_aoa in run_session before the tracker loop).

    use_group_association=True
        After the primary Hungarian assignment each confirmed track also
        absorbs any additional unassigned detections within its gate.
        The EKF update uses the averaged measurement (paper eq. 15).

    use_dbscan_init=True
        Unassigned detections are clustered with DBSCAN before track
        initiation.  Dense clusters spawn a new tentative track directly;
        isolated noise points still go through the pool-based counter.

    Lifecycle: Tentative → Confirmed (hit_count >= confirm_hits)
               Confirmed → Deleted  (miss_count >= max_misses)
               Tentative → Deleted  (not confirmed within init_window_frames)
    """

    def __init__(self, cfg: TrackingConfig) -> None:
        self.cfg = cfg
        self._tracks: list[Track] = []
        self._finished_confirmed_tracks: list[Track] = []
        self._next_id: int = 0
        self._pool: list[dict] = []

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    def _state_dim(self) -> int:
        return 4 if self.cfg.use_2d_ekf else 2

    def _range_from_state(self, state: np.ndarray) -> float:
        if self.cfg.use_2d_ekf:
            return float(np.sqrt(state[0] ** 2 + state[1] ** 2 + 1e-12))
        return float(state[0])

    # ------------------------------------------------------------------
    # 1-D KF / EKF matrices
    # ------------------------------------------------------------------

    def _F(self) -> np.ndarray:
        dt = self.cfg.dt_s
        return np.array([[1.0, dt], [0.0, 1.0]])

    def _Q(self) -> np.ndarray:
        dt = self.cfg.dt_s
        sr, sv = self.cfg.sigma_process_m, self.cfg.sigma_process_v
        return np.diag([sr ** 2 * dt, sv ** 2 * dt])

    _H1 = np.array([[1.0, 0.0]])

    def _measurement_prediction_1d(self, state: np.ndarray) -> np.ndarray:
        if self.cfg.filter_name == "ekf":
            return np.array([np.sqrt(state[0] ** 2 + 1e-12)], dtype=np.float64)
        return self._H1 @ state

    def _measurement_jacobian_1d(self, state: np.ndarray) -> np.ndarray:
        if self.cfg.filter_name == "ekf":
            denom = float(np.sqrt(state[0] ** 2 + 1e-12))
            return np.array([[float(state[0]) / denom, 0.0]], dtype=np.float64)
        return self._H1

    # ------------------------------------------------------------------
    # 2-D Cartesian EKF matrices (paper §II-B, §II-C)
    #
    # State:   [x_m, y_m, vx_mps, vy_mps]
    #          x = lateral (positive = right), y = depth (positive = forward)
    # Azimuth: θ = atan2(x, y)  — angle from boresight (+y axis)
    # ------------------------------------------------------------------

    def _F2d(self) -> np.ndarray:
        dt = self.cfg.dt_s
        return np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=np.float64)

    def _Q2d(self) -> np.ndarray:
        dt = self.cfg.dt_s
        sp, sv = self.cfg.sigma_process_m, self.cfg.sigma_process_v
        return np.diag([sp ** 2 * dt, sp ** 2 * dt, sv ** 2 * dt, sv ** 2 * dt])

    def _h2d(self, state: np.ndarray) -> np.ndarray:
        """Nonlinear measurement model h(x) = [r, az_rad, ṙ]  (paper eq. 9)."""
        x, y, vx, vy = state
        r = float(np.sqrt(x ** 2 + y ** 2 + 1e-12))
        az = float(np.arctan2(x, y))
        rdot = float((x * vx + y * vy) / r)
        return np.array([r, az, rdot], dtype=np.float64)

    def _H2d_jacobian(self, state: np.ndarray) -> np.ndarray:
        """Jacobian of h2d evaluated at state  (3 × 4)."""
        x, y, vx, vy = state
        r2 = x ** 2 + y ** 2 + 1e-12
        r = float(np.sqrt(r2))
        vr = (x * vx + y * vy) / r
        dr    = np.array([x / r,               y / r,               0.0,   0.0  ])
        daz   = np.array([y / r2,             -x / r2,               0.0,   0.0  ])
        drdot = np.array([
            (vx * r2 - x * r * vr) / (r2 * r),
            (vy * r2 - y * r * vr) / (r2 * r),
            x / r,
            y / r,
        ])
        return np.array([dr, daz, drdot], dtype=np.float64)

    # ------------------------------------------------------------------
    # Measurement components — dispatch 1-D vs 2-D
    # ------------------------------------------------------------------

    def _measurement_components(
        self,
        track: Track,
        det: CfarDetection,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if self.cfg.use_2d_ekf:
            return self._measurement_components_2d(track, det)

        # 1-D path: range-only or range+velocity
        if det.range_rate_m_s is not None:
            z_meas = np.array([det.range_m, det.range_rate_m_s], dtype=np.float64)
            z_pred = np.array(track.state, dtype=np.float64)
            H = np.eye(2, dtype=np.float64)
            R = np.diag([self.cfg.sigma_meas_m ** 2, self.cfg.sigma_meas_v ** 2])
            return z_meas, z_pred, H, R

        H = self._measurement_jacobian_1d(track.state)
        z_pred = self._measurement_prediction_1d(track.state)
        z_meas = np.array([det.range_m], dtype=np.float64)
        R = np.array([[self.cfg.sigma_meas_m ** 2]], dtype=np.float64)
        return z_meas, z_pred, H, R

    def _measurement_components_2d(
        self,
        track: Track,
        det: CfarDetection,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Build measurement vector for 2-D EKF; active rows depend on det fields."""
        h_full = self._h2d(track.state)           # [r_pred, az_pred, rdot_pred]
        H_full = self._H2d_jacobian(track.state)  # (3, 4)

        rows: list[int] = [0]
        z_list: list[float] = [det.range_m]
        r_diag: list[float] = [self.cfg.sigma_meas_m ** 2]

        if det.azimuth_deg is not None:
            rows.append(1)
            z_list.append(float(np.radians(det.azimuth_deg)))
            r_diag.append(float(np.radians(self.cfg.sigma_meas_az_deg)) ** 2)

        if det.range_rate_m_s is not None:
            rows.append(2)
            z_list.append(det.range_rate_m_s)
            r_diag.append(self.cfg.sigma_meas_v ** 2)

        H = H_full[rows, :]
        z_pred = h_full[rows]
        z_meas = np.array(z_list, dtype=np.float64)
        R = np.diag(r_diag)
        return z_meas, z_pred, H, R

    # ------------------------------------------------------------------
    # Gating
    # ------------------------------------------------------------------

    def _assignment_cost(self, track: Track, det: CfarDetection) -> float:
        r_pred = self._range_from_state(track.state)
        if abs(r_pred - det.range_m) > self.cfg.gate_distance_m:
            return np.inf
        z_meas, z_pred, H, R = self._measurement_components(track, det)
        innov = z_meas - z_pred
        S = H @ track.covariance @ H.T + R
        mah2 = float(innov.T @ np.linalg.solve(S, innov))
        if mah2 > self.cfg.gate_chi2:
            return np.inf
        return float(np.sqrt(mah2))

    # ------------------------------------------------------------------
    # Track history helpers
    # ------------------------------------------------------------------

    def _initial_velocity(self, det: CfarDetection, entry: dict, timestamp_s: float) -> float:
        if det.range_rate_m_s is not None:
            return float(det.range_rate_m_s)
        prev_range = entry.get("prev_range") or entry.get("start_range")
        prev_ts = entry.get("prev_ts") or entry.get("start_ts")
        if prev_range is None or prev_ts is None:
            return 0.0
        delta_t = float(timestamp_s) - float(prev_ts)
        if delta_t <= 0.0:
            return 0.0
        return float((det.range_m - float(prev_range)) / delta_t)

    def _append_observed_sample(self, track: Track, timestamp_s: float, range_m: float) -> None:
        track.history_t.append(timestamp_s)
        track.history_m.append(range_m)
        track.trajectory_t.append(timestamp_s)
        track.trajectory_m.append(range_m)
        track.trajectory_observed.append(True)
        if self.cfg.use_2d_ekf and len(track.state) == 4:
            track.trajectory_x.append(float(track.state[0]))
            track.trajectory_y.append(float(track.state[1]))

    def _append_predicted_sample(self, track: Track, timestamp_s: float) -> None:
        if track.trajectory_t and timestamp_s <= track.trajectory_t[-1]:
            return
        r = self._range_from_state(track.state)
        track.trajectory_t.append(timestamp_s)
        track.trajectory_m.append(r)
        track.trajectory_observed.append(False)
        if self.cfg.use_2d_ekf and len(track.state) == 4:
            track.trajectory_x.append(float(track.state[0]))
            track.trajectory_y.append(float(track.state[1]))

    # ------------------------------------------------------------------
    # Group association helper — average a set of detections (paper eq. 15)
    # ------------------------------------------------------------------

    def _average_detections(self, group: list[CfarDetection]) -> CfarDetection:
        if len(group) == 1:
            return group[0]
        avg_range = float(np.mean([d.range_m for d in group]))
        vels = [d.range_rate_m_s for d in group if d.range_rate_m_s is not None]
        avg_vel: float | None = float(np.mean(vels)) if vels else None
        azs = [d.azimuth_deg for d in group if d.azimuth_deg is not None]
        avg_az: float | None = float(np.mean(azs)) if azs else None
        best = min(group, key=lambda d: abs(d.range_m - avg_range))
        return CfarDetection(
            frame_idx=group[0].frame_idx,
            tap_idx=best.tap_idx,
            range_m=avg_range,
            magnitude=float(np.mean([d.magnitude for d in group])),
            threshold=float(max(d.threshold for d in group)),
            range_rate_m_s=avg_vel,
            azimuth_deg=avg_az,
        )

    # ------------------------------------------------------------------
    # EKF update (shared between primary and grouped updates)
    # ------------------------------------------------------------------

    def _ekf_update(self, track: Track, det: CfarDetection) -> None:
        dim = self._state_dim()
        z_meas, z_pred, H, R = self._measurement_components(track, det)
        innov = z_meas - z_pred
        S = H @ track.covariance @ H.T + R
        K = track.covariance @ H.T @ np.linalg.inv(S)
        track.state = track.state + K @ innov
        track.covariance = (np.eye(dim) - K @ H) @ track.covariance

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
        if self.cfg.use_2d_ekf:
            F, Q = self._F2d(), self._Q2d()
        else:
            F, Q = self._F(), self._Q()

        # 1. Predict
        for track in self._tracks:
            if track.miss_count > 0 and self.cfg.velocity_decay < 1.0:
                if self.cfg.use_2d_ekf:
                    track.state[2] *= self.cfg.velocity_decay
                    track.state[3] *= self.cfg.velocity_decay
                else:
                    track.state[1] *= self.cfg.velocity_decay
            track.state = F @ track.state
            track.covariance = F @ track.covariance @ F.T + Q

        # 2. Hungarian assignment with Mahalanobis gate
        assigned_tracks: set[int] = set()
        assigned_dets: set[int] = set()

        if self._tracks and detections:
            cost = np.full((len(self._tracks), len(detections)), np.inf)
            for ti, track in enumerate(self._tracks):
                for di, det in enumerate(detections):
                    cost[ti, di] = self._assignment_cost(track, det)

            if self.cfg.use_hungarian:
                cost_finite = np.where(np.isinf(cost), 1e9, cost)
                row_ind, col_ind = linear_sum_assignment(cost_finite)
                pairs = [(int(ti), int(di)) for ti, di in zip(row_ind, col_ind)
                         if cost[ti, di] < np.inf]
            else:
                cost_g = cost.copy()
                pairs = []
                while True:
                    flat = int(np.argmin(cost_g))
                    ti, di = flat // cost_g.shape[1], flat % cost_g.shape[1]
                    if cost_g[ti, di] == np.inf:
                        break
                    pairs.append((ti, di))
                    cost_g[ti, :] = np.inf
                    cost_g[:, di] = np.inf

            for ti, di in pairs:
                assigned_tracks.add(ti)
                assigned_dets.add(di)

            # Group association: absorb additional gated unassigned detections
            # into each assigned track; update with averaged measurement (paper eq. 15).
            track_groups: dict[int, list[int]] = {ti: [di] for ti, di in pairs}
            if self.cfg.use_group_association:
                for ti in list(track_groups.keys()):
                    track = self._tracks[ti]
                    for di2, det2 in enumerate(detections):
                        if di2 not in assigned_dets:
                            if self._assignment_cost(track, det2) < np.inf:
                                track_groups[ti].append(di2)
                                assigned_dets.add(di2)

            for ti, det_indices in track_groups.items():
                track = self._tracks[ti]
                det_for_update = self._average_detections(
                    [detections[di] for di in det_indices]
                )
                self._ekf_update(track, det_for_update)
                track.hit_count += 1
                track.miss_count = 0
                if track.hit_count >= self.cfg.confirm_hits:
                    track.confirmed = True
                self._append_observed_sample(
                    track, timestamp_s, self._range_from_state(track.state),
                )

        # 3. Increment miss counter for unassigned tracks
        for ti, track in enumerate(self._tracks):
            if ti not in assigned_tracks:
                track.miss_count += 1
                if track.confirmed:
                    self._append_predicted_sample(track, timestamp_s)

        # 4. Tentative track initiation from unassigned detections
        unassigned = [detections[di] for di in range(len(detections)) if di not in assigned_dets]
        if self.cfg.use_dbscan_init and len(unassigned) >= 2:
            self._dbscan_init(unassigned, frame_idx, timestamp_s)
        else:
            for det in unassigned:
                self._try_init(det, frame_idx, timestamp_s)

        # 5. Delete tracks that exceeded the miss budget; archive confirmed ones
        surviving: list[Track] = []
        for track in self._tracks:
            if track.miss_count < self.cfg.max_misses:
                surviving.append(track)
            elif track.confirmed:
                self._finished_confirmed_tracks.append(track)
        self._tracks = surviving

        return [t for t in self._tracks if t.confirmed]

    def active_tracks_snapshot(self) -> list[Track]:
        return [t for t in self._tracks if t.confirmed]

    def tracks_snapshot(self, include_finished: bool = True) -> list[Track]:
        active = [t for t in self._tracks if t.confirmed]
        if not include_finished:
            return active
        return [*self._finished_confirmed_tracks, *active]

    # ------------------------------------------------------------------
    # Tentative track pool
    # ------------------------------------------------------------------

    def _try_init(self, det: CfarDetection, frame_idx: int, timestamp_s: float) -> None:
        cfg = self.cfg
        for entry in self._pool:
            if frame_idx - entry["last_frame"] > cfg.init_window_frames:
                continue
            if abs(det.range_m - entry["last_range"]) <= cfg.gate_distance_m:
                entry["prev_range"] = entry["last_range"]
                entry["prev_ts"] = entry["last_ts"]
                entry["hits"] += 1
                entry["last_range"] = det.range_m
                entry["last_frame"] = frame_idx
                entry["last_ts"] = timestamp_s
                entry["history_ranges"].append(det.range_m)
                entry["history_ts"].append(timestamp_s)
                if entry["hits"] >= cfg.init_hits:
                    self._promote(det, entry, timestamp_s)
                return

        self._pool.append({
            "hits": 1,
            "start_frame": frame_idx,
            "start_range": det.range_m,
            "start_ts": timestamp_s,
            "last_frame": frame_idx,
            "last_range": det.range_m,
            "last_ts": timestamp_s,
            "prev_range": None,
            "prev_ts": None,
            "history_ranges": [det.range_m],
            "history_ts": [timestamp_s],
        })
        self._pool = [
            e for e in self._pool
            if frame_idx - e["last_frame"] <= cfg.init_window_frames
        ]

    def _promote(self, det: CfarDetection, entry: dict, timestamp_s: float) -> None:
        initial_velocity = self._initial_velocity(det, entry, timestamp_s)

        if self.cfg.use_2d_ekf:
            az_deg = det.azimuth_deg if det.azimuth_deg is not None else 0.0
            az_rad = np.radians(az_deg)
            r = det.range_m
            state = np.array([
                r * np.sin(az_rad),
                r * np.cos(az_rad),
                initial_velocity * np.sin(az_rad),
                initial_velocity * np.cos(az_rad),
            ], dtype=np.float64)
            # Propagate polar → Cartesian covariance via Jacobian
            sig_az_rad = float(np.radians(
                self.cfg.sigma_meas_az_deg if det.azimuth_deg is not None else 45.0
            ))
            J = np.array([
                [np.sin(az_rad), r * np.cos(az_rad)],
                [np.cos(az_rad), -r * np.sin(az_rad)],
            ])
            P_polar = np.diag([self.cfg.sigma_meas_m ** 2, sig_az_rad ** 2])
            P_pos = J @ P_polar @ J.T
            covariance = np.block([
                [P_pos,           np.zeros((2, 2))],
                [np.zeros((2, 2)), np.diag([self.cfg.sigma_process_v ** 2] * 2)],
            ])
        else:
            state = np.array([det.range_m, initial_velocity], dtype=np.float64)
            covariance = np.diag([self.cfg.sigma_meas_m ** 2, self.cfg.sigma_process_v ** 2])

        track = Track(track_id=self._next_id, state=state, covariance=covariance)
        self._next_id += 1
        track.hit_count = entry["hits"]
        track.confirmed = track.hit_count >= self.cfg.confirm_hits
        for sample_t, sample_r in zip(entry["history_ts"], entry["history_ranges"]):
            self._append_observed_sample(track, float(sample_t), float(sample_r))
        self._tracks.append(track)
        if entry in self._pool:
            self._pool.remove(entry)

    def _promote_cluster(
        self,
        cluster_dets: list[CfarDetection],
        frame_idx: int,
        timestamp_s: float,
    ) -> None:
        """Promote a DBSCAN cluster as a new tentative/confirmed track."""
        rep = self._average_detections(cluster_dets)
        entry = {
            "hits": len(cluster_dets),
            "start_frame": frame_idx,
            "start_range": rep.range_m,
            "start_ts": timestamp_s,
            "last_frame": frame_idx,
            "last_range": rep.range_m,
            "last_ts": timestamp_s,
            "prev_range": None,
            "prev_ts": None,
            "history_ranges": [d.range_m for d in cluster_dets],
            "history_ts": [timestamp_s] * len(cluster_dets),
        }
        if entry["hits"] >= self.cfg.init_hits:
            self._promote(rep, entry, timestamp_s)
        else:
            self._pool.append(entry)

    def _dbscan_init(
        self,
        unassigned: list[CfarDetection],
        frame_idx: int,
        timestamp_s: float,
    ) -> None:
        """Cluster unassigned detections with DBSCAN; promote dense clusters directly."""
        try:
            from sklearn.cluster import DBSCAN
        except ImportError:
            for det in unassigned:
                self._try_init(det, frame_idx, timestamp_s)
            return

        coords = np.array([[d.range_m, d.range_rate_m_s or 0.0] for d in unassigned])
        labels = DBSCAN(eps=self.cfg.gate_distance_m, min_samples=2).fit_predict(coords)

        used: set[int] = set()
        for label in set(labels):
            if label == -1:
                continue
            indices = [i for i, lbl in enumerate(labels) if lbl == label]
            self._promote_cluster([unassigned[i] for i in indices], frame_idx, timestamp_s)
            used.update(indices)

        # Noise points (-1) still go through the pool counter
        for i, det in enumerate(unassigned):
            if i not in used:
                self._try_init(det, frame_idx, timestamp_s)
