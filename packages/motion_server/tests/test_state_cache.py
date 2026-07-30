from __future__ import annotations

import pytest
from forge_motion_server.state_cache import JointStateCache
from forge_msgs import JointState


def test_cache_reorders_extra_joints_and_estimates_velocity() -> None:
    cache = JointStateCache()
    cache.update(
        JointState(
            name=["extra", "j2", "j1"],
            position=[4.0, 0.0, 0.0],
            velocity=[],
            effort=[],
        ),
        0,
    )
    cache.update(
        JointState(
            name=["j2", "j1", "extra"],
            position=[-0.1, 0.2, 4.0],
            velocity=[],
            effort=[],
        ),
        100_000_000,
    )

    snapshot = cache.snapshot(("j1", "j2"), 100_000_000, 200_000_000)

    assert snapshot is not None
    assert snapshot.positions == (0.2, -0.1)
    assert snapshot.velocities == pytest.approx((2.0, -1.0))


def test_cache_rejects_missing_and_stale_state() -> None:
    cache = JointStateCache()
    cache.update(
        JointState(name=["j1"], position=[0.0], velocity=[0.0], effort=[]),
        10,
    )

    with pytest.raises(ValueError, match="missing"):
        cache.snapshot(("j1", "j2"), 10, 100)
    assert cache.snapshot(("j1",), 111, 100) is None
