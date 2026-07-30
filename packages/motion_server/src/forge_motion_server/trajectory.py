from __future__ import annotations

import math
from dataclasses import dataclass

from forge_msgs import JointTrajectory, JointTrajectoryPoint

_MAX_QUINTIC_VELOCITY = 1.875
_MAX_QUINTIC_ACCELERATION = 10.0 / math.sqrt(3.0)


class TrajectoryGenerationError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class PlannedTrajectory:
    trajectory: JointTrajectory
    duration_ns: int
    minimum_duration_ns: int


def plan_quintic_trajectory(
    *,
    joint_names: tuple[str, ...],
    start_positions: tuple[float, ...],
    target_positions: tuple[float, ...],
    max_velocity: tuple[float, ...],
    max_acceleration: tuple[float, ...],
    velocity_scale: float,
    acceleration_scale: float,
    sample_period_ns: int,
    requested_duration_ns: int | None,
    max_points: int = 10_000,
) -> PlannedTrajectory:
    size = len(joint_names)
    vectors = (start_positions, target_positions, max_velocity, max_acceleration)
    if size == 0 or any(len(vector) != size for vector in vectors):
        raise TrajectoryGenerationError("trajectory vectors must match the non-empty joint group")
    if not 0.0 < velocity_scale <= 1.0 or not 0.0 < acceleration_scale <= 1.0:
        raise TrajectoryGenerationError("trajectory scales must be in the interval (0, 1]")
    if sample_period_ns <= 0 or max_points < 2:
        raise TrajectoryGenerationError("sample period and max_points must be positive")

    minimum_s = 0.0
    deltas: list[float] = []
    for start, target, velocity, acceleration in zip(
        start_positions,
        target_positions,
        max_velocity,
        max_acceleration,
        strict=True,
    ):
        values = (start, target, velocity, acceleration)
        if any(not math.isfinite(value) for value in values):
            raise TrajectoryGenerationError("trajectory inputs must be finite")
        if velocity <= 0.0 or acceleration <= 0.0:
            raise TrajectoryGenerationError(
                "joint velocity and acceleration limits must be positive"
            )
        delta = target - start
        distance = abs(delta)
        deltas.append(delta)
        velocity_duration = _MAX_QUINTIC_VELOCITY * distance / (velocity * velocity_scale)
        acceleration_duration = math.sqrt(
            _MAX_QUINTIC_ACCELERATION * distance / (acceleration * acceleration_scale)
        )
        minimum_s = max(minimum_s, velocity_duration, acceleration_duration)

    minimum_duration_ns = max(sample_period_ns, math.ceil(minimum_s * 1_000_000_000.0))
    if requested_duration_ns is None:
        duration_ns = minimum_duration_ns
    else:
        if requested_duration_ns < minimum_duration_ns:
            raise TrajectoryGenerationError(
                f"requested duration {requested_duration_ns} ns is below feasible minimum "
                f"{minimum_duration_ns} ns"
            )
        duration_ns = requested_duration_ns

    segment_count = max(1, math.ceil(duration_ns / sample_period_ns))
    if segment_count + 1 > max_points:
        raise TrajectoryGenerationError(
            f"trajectory requires {segment_count + 1} points, exceeding limit {max_points}"
        )
    duration_s = duration_ns / 1_000_000_000.0
    points: list[JointTrajectoryPoint] = []
    previous_time = -1
    for index in range(segment_count + 1):
        time_ns = duration_ns if index == segment_count else index * duration_ns // segment_count
        if time_ns <= previous_time:
            raise TrajectoryGenerationError("trajectory sample times are not strictly increasing")
        previous_time = time_ns
        tau = index / segment_count
        scale = _quintic_position_scale(tau)
        scale_rate = _quintic_velocity_scale(tau) / duration_s
        points.append(
            JointTrajectoryPoint(
                positions=[
                    start + scale * delta
                    for start, delta in zip(start_positions, deltas, strict=True)
                ],
                velocities=[scale_rate * delta for delta in deltas],
                accelerations=[],
                effort=[],
                time_from_start_ns=time_ns,
            )
        )
    return PlannedTrajectory(
        trajectory=JointTrajectory(joint_names=list(joint_names), points=points),
        duration_ns=duration_ns,
        minimum_duration_ns=minimum_duration_ns,
    )


def _quintic_position_scale(tau: float) -> float:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def _quintic_velocity_scale(tau: float) -> float:
    return 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
