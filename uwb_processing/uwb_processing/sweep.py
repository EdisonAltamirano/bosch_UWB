from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class SweepExpectation:
    expected_track_count: int | None = None
    expected_max_active_tracks: int | None = None
    min_longest_track_fraction: float | None = None
    min_multi_peak_fraction: float | None = None
    max_multi_peak_fraction: float | None = None
    min_separation_pass_rate: float | None = None
    expected_breathing_count: int | None = None
    breathing_bpm_range: tuple[float, float] | None = None
    breathing_arc_range_deg: tuple[float, float] | None = None


@dataclass(slots=True)
class SweepMetrics:
    confirmed_track_count: int
    max_active_track_count: int
    longest_track_fraction: float
    multi_peak_fraction: float
    separation_pass_rate: float | None
    breathing_candidate_count: int
    valid_breathing_count: int
    best_breathing_bpm: float | None
    best_breathing_arc_deg: float | None


def _range_penalty(value: float | None, allowed_range: tuple[float, float] | None, scale: float) -> float:
    if value is None or allowed_range is None:
        return 0.0
    low, high = allowed_range
    if value < low:
        return (low - value) * scale
    if value > high:
        return (value - high) * scale
    return 0.0


def score_metrics(metrics: SweepMetrics, expectation: SweepExpectation) -> tuple[float, list[str]]:
    score = 0.0
    notes: list[str] = []

    if expectation.expected_track_count is not None:
        diff = abs(metrics.confirmed_track_count - expectation.expected_track_count)
        score -= 80.0 * diff
        if diff == 0:
            score += 20.0
            notes.append("track-count match")
        else:
            notes.append(
                f"track-count diff={diff} (got {metrics.confirmed_track_count}, expected {expectation.expected_track_count})"
            )

    if expectation.expected_max_active_tracks is not None:
        shortfall = max(0, expectation.expected_max_active_tracks - metrics.max_active_track_count)
        overflow = max(0, metrics.max_active_track_count - expectation.expected_max_active_tracks)
        score -= 120.0 * shortfall
        score -= 40.0 * overflow
        if shortfall == 0 and overflow == 0:
            score += 20.0
            notes.append("active-track match")
        elif shortfall > 0:
            notes.append(
                f"active-track shortfall={shortfall} (got {metrics.max_active_track_count}, expected {expectation.expected_max_active_tracks})"
            )
        else:
            notes.append(
                f"active-track overflow={overflow} (got {metrics.max_active_track_count}, expected {expectation.expected_max_active_tracks})"
            )

    if expectation.min_longest_track_fraction is not None:
        shortfall = max(0.0, expectation.min_longest_track_fraction - metrics.longest_track_fraction)
        score -= 200.0 * shortfall
        if shortfall == 0.0:
            score += 20.0
            notes.append("track-coverage pass")
        else:
            notes.append(
                f"track-coverage shortfall={shortfall:.3f} (got {metrics.longest_track_fraction:.3f})"
            )

    if expectation.min_multi_peak_fraction is not None:
        shortfall = max(0.0, expectation.min_multi_peak_fraction - metrics.multi_peak_fraction)
        score -= 150.0 * shortfall
        if shortfall == 0.0:
            notes.append("multi-peak density pass")
        else:
            notes.append(
                f"multi-peak density shortfall={shortfall:.3f} (got {metrics.multi_peak_fraction:.3f})"
            )

    if expectation.max_multi_peak_fraction is not None:
        excess = max(0.0, metrics.multi_peak_fraction - expectation.max_multi_peak_fraction)
        score -= 150.0 * excess
        if excess > 0.0:
            notes.append(
                f"multi-peak density excess={excess:.3f} (got {metrics.multi_peak_fraction:.3f})"
            )

    if expectation.min_separation_pass_rate is not None:
        actual = 0.0 if metrics.separation_pass_rate is None else metrics.separation_pass_rate
        shortfall = max(0.0, expectation.min_separation_pass_rate - actual)
        score -= 200.0 * shortfall
        if shortfall == 0.0:
            score += 10.0
            notes.append("separation pass")
        else:
            notes.append(f"separation shortfall={shortfall:.3f} (got {actual:.3f})")

    if expectation.expected_breathing_count is not None:
        diff = abs(metrics.valid_breathing_count - expectation.expected_breathing_count)
        score -= 100.0 * diff
        if diff == 0:
            score += 15.0
            notes.append("breathing-count match")
        else:
            notes.append(
                f"breathing-count diff={diff} (got {metrics.valid_breathing_count}, expected {expectation.expected_breathing_count})"
            )

    bpm_penalty = _range_penalty(
        metrics.best_breathing_bpm,
        expectation.breathing_bpm_range,
        scale=5.0,
    )
    score -= bpm_penalty
    if bpm_penalty == 0.0 and expectation.breathing_bpm_range is not None and metrics.best_breathing_bpm is not None:
        notes.append("breathing BPM in range")
    elif bpm_penalty > 0.0:
        notes.append(f"breathing BPM penalty={bpm_penalty:.1f}")

    arc_penalty = _range_penalty(
        metrics.best_breathing_arc_deg,
        expectation.breathing_arc_range_deg,
        scale=0.5,
    )
    score -= arc_penalty
    if arc_penalty == 0.0 and expectation.breathing_arc_range_deg is not None and metrics.best_breathing_arc_deg is not None:
        notes.append("breathing arc in range")
    elif arc_penalty > 0.0:
        notes.append(f"breathing arc penalty={arc_penalty:.1f}")

    return score, notes
