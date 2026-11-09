from __future__ import annotations

from pathlib import Path

import pytest
from forge_motion_server.config import (
    MotionServerConfig,
    load_config,
    motion_server_config_from_mapping,
)


def test_omitted_ik_settings_load_the_validated_defaults() -> None:
    mapping = _mapping()
    mapping.pop("planning_timeout_ms")
    config = motion_server_config_from_mapping(mapping)
    group = config.groups["arm"]
    assert config.planning_timeout_ns == 1_000_000_000
    assert MotionServerConfig(groups=config.groups).planning_timeout_ns == 1_000_000_000
    assert group.ik_position_tolerance_scale == 0.5
    assert group.ik_position_tolerance_max_m == 0.002
    assert group.ik_orientation_weight == 0.01
    assert group.ik_damping == 0.1
    assert group.ik_max_attempts == 8
    assert group.ik_attempt_timeout_ms == 80
    assert group.ik_seed_perturbation_rad == 0.35


def test_explicit_ik_settings_override_defaults() -> None:
    mapping = _mapping()
    settings = {
        "ik_position_tolerance_scale": 0.25,
        "ik_position_tolerance_max_m": 0.001,
        "ik_orientation_weight": 0.04,
        "ik_damping": 0.05,
        "ik_max_attempts": 3,
        "ik_attempt_timeout_ms": 25,
        "ik_seed_perturbation_rad": 0.1,
    }
    mapping["groups"]["arm"].update(settings)
    config = motion_server_config_from_mapping(mapping)
    assert config.planning_timeout_ns == 75_000_000
    for name, value in settings.items():
        assert getattr(config.groups["arm"], name) == value


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
