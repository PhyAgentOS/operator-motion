from __future__ import annotations

from pathlib import Path

import pytest
from forge_motion_server.config import load_config, motion_server_config_from_mapping


def _mapping() -> dict:
    return {
        "state_timeout_ms": 150,
        "planning_timeout_ms": 75,
        "execution_timeout_margin_ms": 1500,
        "final_state_timeout_ms": 250,
        "child_cancel_retry_ms": 80,
        "child_cancel_timeout_ms": 500,
        "result_cache_size": 16,
        "groups": {
            "arm": {
                "joint_names": ["j1", "j2"],
                "urdf_path": "robot.urdf",
                "base_frame": "base",
                "tip_frame": "tcp",
                "max_velocity": {"j1": 1.0, "j2": 2.0},
                "max_acceleration": {"j1": 3.0, "j2": 4.0},
                "trajectory_sample_period_ms": 25,
            }
        },
    }


def test_config_preserves_joint_order_and_resolves_urdf(tmp_path: Path) -> None:
    config = motion_server_config_from_mapping(_mapping(), base_path=tmp_path)
    group = config.groups["arm"]

    assert group.joint_names == ("j1", "j2")
    assert group.max_velocity == (1.0, 2.0)
    assert group.max_acceleration == (3.0, 4.0)
    assert group.urdf_path == (tmp_path / "robot.urdf").resolve()
    assert group.trajectory_sample_period_ns == 25_000_000
    assert config.state_timeout_ns == 150_000_000
    assert config.execution_timeout_margin_ns == 1_500_000_000
    assert config.final_state_timeout_ns == 250_000_000
    assert config.child_cancel_retry_ns == 80_000_000


def test_config_rejects_unknown_and_incomplete_limit_maps() -> None:
    mapping = _mapping()
    mapping["unexpected"] = True
    with pytest.raises(ValueError, match="unknown motion server"):
        motion_server_config_from_mapping(mapping)

    mapping = _mapping()
    del mapping["groups"]["arm"]["max_velocity"]["j2"]
    with pytest.raises(ValueError, match="missing=.*j2"):
        motion_server_config_from_mapping(mapping)

    mapping = _mapping()
    mapping["child_cancel_retry_ms"] = mapping["child_cancel_timeout_ms"]
    with pytest.raises(ValueError, match="retry_ns must be less"):
        motion_server_config_from_mapping(mapping)


def test_load_config_uses_file_directory_for_relative_urdf(tmp_path: Path) -> None:
    path = tmp_path / "motion.yaml"
    path.write_text(
        """
groups:
  arm:
    joint_names: [j1]
    urdf_path: assets/robot.urdf
    base_frame: base
    tip_frame: tcp
    max_velocity: {j1: 1.0}
    max_acceleration: {j1: 2.0}
""".strip(),
        encoding="utf-8",
    )

    config = load_config(path)

    assert config.groups["arm"].urdf_path == (tmp_path / "assets/robot.urdf").resolve()
