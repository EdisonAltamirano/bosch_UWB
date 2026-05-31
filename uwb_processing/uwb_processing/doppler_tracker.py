from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import DBSCAN

from .types import DopplerCluster, DopplerTrack


def dbscan_cluster(
    detections: list[tuple[float, float, float]],
    range_scale: float = 3.0,
    vel_scale: float = 1.0,
    eps: float = 0.35,
    min_samples: int = 2,
    grid_max: float | None = None,
) -> list[DopplerCluster]:
    """Cluster (range_m, velocity_mps, magnitude) detections with DBSCAN.

    Axes are normalised before clustering so range and velocity contribute
    proportionally.  Magnitude-weighted centroids are returned per cluster.
    """
    if len(detections) < min_samples:
        return []

    ranges = np.array([d[0] for d in detections])
    vels = np.array([d[1] for d in detections])
    mags = np.array([d[2] for d in detections])

    pts = np.column_stack([ranges / range_scale, vels / vel_scale])
    labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(pts)

    mag_min = 0.05 * float(grid_max) if grid_max is not None else 0.0

    clusters: list[DopplerCluster] = []
    for label in np.unique(labels):
        if label == -1:
            continue
        mask = labels == label
        cluster_mags = mags[mask]
        total_mag = float(cluster_mags.sum())
        if total_mag < mag_min:
            continue
        weights = cluster_mags / (total_mag + 1e-12)
        centroid_r = float(np.dot(ranges[mask], weights))
        centroid_v = float(np.dot(vels[mask], weights))
        clusters.append(DopplerCluster(
            range_m=centroid_r,
            velocity_mps=centroid_v,
            magnitude=total_mag,
            n_points=int(mask.sum()),
        ))

    return clusters


class _TrackInternal:
    def __init__(self, track_id: int, state: np.ndarray, cov: np.ndarray) -> None:
        self.track_id = track_id
        self.state = state          # (2,) mutable
        self.cov = cov              # (2,2) mutable
        self.hit_count = 0
        self.miss_count = 0
        self.confirmed = False
        self.history_range_m: list[float] = []
        self.history_velocity_mps: list[float] = []
        self.history_t: list[float] = []


class DopplerTracker:
    """DBSCAN-then-Kalman people tracker in range × velocity space.

    Call update() once per RangeDopplerFrame with the raw CFAR detections.
    DBSCAN clustering is done internally before Kalman assignment.
    """

    def __init__(
        self,
        sigma_r: float = 0.10,
        sigma_v: float = 0.05,
        meas_r: float = 0.15,
        meas_v: float = 0.10,
        gate: float = 1.5,
        confirm_hits: int = 3,
        max_misses: int = 5,
        range_scale: float = 3.0,
        vel_scale: float = 1.0,
        min_samples: int = 1,
    ) -> None:
        self._Q = np.diag([sigma_r**2, sigma_v**2])
        self._R = np.diag([meas_r**2, meas_v**2])
        self._gate_sq = gate**2
        self._confirm_hits = confirm_hits
        self._max_misses = max_misses
        self._range_scale = range_scale
        self._vel_scale = vel_scale
        self._min_samples = min_samples

        self._tracks: list[_TrackInternal] = []
        self._next_id = 0
        self._last_t: float | None = None

    # ------------------------------------------------------------------
    def update(
        self,
        frame_idx: int,
        timestamp_s: float,
        detections: list[tuple[float, float, float]],
    ) -> list[DopplerTrack]:
        """Cluster detections, then predict-assign-update all Kalman tracks."""
        dt = (timestamp_s - self._last_t) if self._last_t is not None else 0.05
        dt = max(dt, 1e-3)
        self._last_t = timestamp_s

        # Cluster raw detections
        grid_max = float(max(d[2] for d in detections)) if detections else 1.0
        clusters = dbscan_cluster(
            detections,
            range_scale=self._range_scale,
            vel_scale=self._vel_scale,
            grid_max=grid_max,
            min_samples=self._min_samples,
        )

        # Predict all tracks
        F = np.array([[1.0, dt], [0.0, 1.0]])
        for track in self._tracks:
            track.state = F @ track.state
            track.cov = F @ track.cov @ F.T + self._Q

        # Hungarian assignment
        matched_t: set[int] = set()
        matched_c: set[int] = set()

        if self._tracks and clusters:
            cost = np.zeros((len(self._tracks), len(clusters)))
            for i, track in enumerate(self._tracks):
                for j, cl in enumerate(clusters):
                    diff = np.array([
                        (cl.range_m - track.state[0]) / self._range_scale,
                        (cl.velocity_mps - track.state[1]) / self._vel_scale,
                    ])
                    cost[i, j] = float(diff @ diff)

            row_ind, col_ind = linear_sum_assignment(cost)
            for i, j in zip(row_ind, col_ind):
                if cost[i, j] > self._gate_sq:
                    continue
                self._kalman_update(self._tracks[i], clusters[j], timestamp_s)
                matched_t.add(i)
                matched_c.add(j)

        # Increment miss counters for unmatched tracks
        for i, track in enumerate(self._tracks):
            if i not in matched_t:
                track.miss_count += 1

        # Initiate new tracks from unmatched clusters
        for j, cl in enumerate(clusters):
            if j not in matched_c:
                state = np.array([cl.range_m, cl.velocity_mps])
                cov = self._R.copy()
                t = _TrackInternal(track_id=self._next_id, state=state, cov=cov)
                t.hit_count = 1
                t.history_range_m.append(cl.range_m)
                t.history_velocity_mps.append(cl.velocity_mps)
                t.history_t.append(timestamp_s)
                self._tracks.append(t)
                self._next_id += 1

        # Prune dead tracks
        self._tracks = [t for t in self._tracks if t.miss_count <= self._max_misses]

        return self._snapshot()

    # ------------------------------------------------------------------
    def _kalman_update(
        self, track: _TrackInternal, cl: DopplerCluster, timestamp_s: float
    ) -> None:
        z = np.array([cl.range_m, cl.velocity_mps])
        H = np.eye(2)
        S = H @ track.cov @ H.T + self._R
        K = track.cov @ H.T @ np.linalg.inv(S)
        track.state = track.state + K @ (z - H @ track.state)
        track.cov = (np.eye(2) - K @ H) @ track.cov
        track.hit_count += 1
        track.miss_count = 0
        track.history_range_m.append(float(track.state[0]))
        track.history_velocity_mps.append(float(track.state[1]))
        track.history_t.append(timestamp_s)
        if not track.confirmed and track.hit_count >= self._confirm_hits:
            track.confirmed = True

    def _snapshot(self) -> list[DopplerTrack]:
        return [
            DopplerTrack(
                track_id=t.track_id,
                state=t.state.copy(),
                covariance=t.cov.copy(),
                history_range_m=list(t.history_range_m),
                history_velocity_mps=list(t.history_velocity_mps),
                history_t=list(t.history_t),
                confirmed=t.confirmed,
                miss_count=t.miss_count,
            )
            for t in self._tracks
        ]

    def confirmed_tracks_snapshot(self) -> list[DopplerTrack]:
        return [t for t in self._snapshot() if t.confirmed]
