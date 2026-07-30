from __future__ import annotations

import math

import pytest
from forge_motion_server.trajectory import TrajectoryGenerationError, plan_quintic_trajectory


def _plan(**overrides):
    arguments = {
        "joint_names": ("j1", "j2"),
        "start_positions": (0.0, 0.0),
        "target_positions": (1.0, -0.5),
        "max_velocity": (1.0, 1.0),
        "max_acceleration": (2.0, 2.0),
        "velocity_scale": 1.0,
        "acceleration_scale": 1.0,
        "sample_period_ns": 50_000_000,
        "requested_duration_ns": None,
        "max_points": 1000,
    }
    arguments.update(overrides)
    return plan_quintic_trajectory(**arguments)


def test_quintic_trajectory_has_smooth_complete_endpoints() -> None:
    planned = _plan()
    first = planned.trajectory.points[0]
    last = planned.trajectory.points[-1]

    assert first.time_from_start_ns == 0
    assert first.positions == [0.0, 0.0]
    assert first.velocities == [0.0, 0.0]
    assert last.time_from_start_ns == planned.duration_ns
    assert last.positions == pytest.approx([1.0, -0.5])
    assert last.velocities == pytest.approx([0.0, 0.0], abs=1e-12)
    assert all(not point.accelerations and not point.effort for point in planned.trajectory.points)
    assert all(
        earlier.time_from_start_ns < later.time_from_start_ns
        for earlier, later in zip(
            planned.trajectory.points, planned.trajectory.points[1:], strict=False
        )
    )


def test_trajectory_respects_analytic_velocity_and_acceleration_duration() -> None:
    planned = _plan()
    duration_s = planned.duration_ns / 1_000_000_000.0

    assert 1.875 / duration_s <= 1.0 + 1e-9
    assert (10.0 / math.sqrt(3.0)) / duration_s**2 <= 2.0 + 1e-9


def test_cubic_controller_interpolation_respects_joint_limits() -> None:
    planned = _plan()
    maximum_velocity = 0.0
    maximum_acceleration = 0.0
    points = planned.trajectory.points
    for lower, upper in zip(points, points[1:], strict=False):
        duration_s = (upper.time_from_start_ns - lower.time_from_start_ns) / 1e9
        for sample in range(101):
            u = sample / 100.0
            u2 = u * u
            velocity_coefficients = (
                (6.0 * u2 - 6.0 * u) / duration_s,
                3.0 * u2 - 4.0 * u + 1.0,
                (-6.0 * u2 + 6.0 * u) / duration_s,
                3.0 * u2 - 2.0 * u,
            )
            acceleration_coefficients = (
                (12.0 * u - 6.0) / duration_s**2,
                (6.0 * u - 4.0) / duration_s,
                (-12.0 * u + 6.0) / duration_s**2,
                (6.0 * u - 2.0) / duration_s,
            )
            for p0, v0, p1, v1 in zip(
                lower.positions,
                lower.velocities,
                upper.positions,
                upper.velocities,
                strict=True,
            ):
                values = (p0, v0, p1, v1)
                velocity = sum(
                    coefficient * value
                    for coefficient, value in zip(velocity_coefficients, values, strict=True)
                )
                acceleration = sum(
                    coefficient * value
                    for coefficient, value in zip(acceleration_coefficients, values, strict=True)
                )
                maximum_velocity = max(maximum_velocity, abs(velocity))
                maximum_acceleration = max(maximum_acceleration, abs(acceleration))

    assert maximum_velocity <= 1.0 + 1e-9
    assert maximum_acceleration <= 2.0 + 1e-9


def test_requested_duration_must_be_feasible_and_point_count_is_bounded() -> None:
    baseline = _plan()
    with pytest.raises(TrajectoryGenerationError, match="below feasible minimum"):
        _plan(requested_duration_ns=baseline.minimum_duration_ns - 1)

    extended = _plan(requested_duration_ns=baseline.minimum_duration_ns + 1_000_000_000)
    assert extended.duration_ns == baseline.minimum_duration_ns + 1_000_000_000

    with pytest.raises(TrajectoryGenerationError, match="exceeding limit"):
        _plan(requested_duration_ns=10_000_000_000, max_points=3)


def test_zero_motion_still_produces_valid_hold_trajectory() -> None:
    planned = _plan(target_positions=(0.0, 0.0))

    assert planned.duration_ns == 50_000_000
    assert len(planned.trajectory.points) == 2
    assert planned.trajectory.points[-1].positions == [0.0, 0.0]
