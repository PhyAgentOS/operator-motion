from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

import yaml


@dataclass(frozen=True, slots=True)
class MotionGroupConfig:
    name: str
    joint_names: tuple[str, ...]
    urdf_path: Path
    base_frame: str
    tip_frame: str
    max_velocity: tuple[float, ...]
    max_acceleration: tuple[float, ...]
    trajectory_sample_period_ns: int = 50_000_000
    max_trajectory_points: int = 10_000
    start_velocity_tolerance: float = 0.1
    final_joint_tolerance: float = 0.03
    final_position_tolerance_m: float = 0.01
    final_orientation_tolerance_rad: float = 0.05
    ik_max_solution_joint_displacement: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("group name must be a non-empty string")
        if not isinstance(self.joint_names, tuple):
            raise ValueError("joint_names must be a tuple")
        if not isinstance(self.urdf_path, Path):
            raise ValueError("urdf_path must be a Path")
        if not self.joint_names or any(
            not isinstance(name, str) or not name for name in self.joint_names
        ):
            raise ValueError("joint_names must contain non-empty strings")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be unique")
        if (
            not isinstance(self.base_frame, str)
            or not self.base_frame
            or not isinstance(self.tip_frame, str)
            or not self.tip_frame
        ):
            raise ValueError("base_frame and tip_frame must be non-empty strings")
        if not isinstance(self.max_velocity, tuple) or not isinstance(self.max_acceleration, tuple):
            raise ValueError("max_velocity and max_acceleration must be tuples")
        if len(self.max_velocity) != len(self.joint_names):
            raise ValueError("max_velocity must contain one value per joint")
        if len(self.max_acceleration) != len(self.joint_names):
            raise ValueError("max_acceleration must contain one value per joint")
        for field_name in ("max_velocity", "max_acceleration"):
            values = getattr(self, field_name)
            if any(not math.isfinite(value) or value <= 0.0 for value in values):
                raise ValueError(f"{field_name} values must be finite and positive")
        if (
            isinstance(self.trajectory_sample_period_ns, bool)
            or not isinstance(self.trajectory_sample_period_ns, int)
            or self.trajectory_sample_period_ns <= 0
        ):
            raise ValueError("trajectory_sample_period_ns must be a positive integer")
        if (
            isinstance(self.max_trajectory_points, bool)
            or not isinstance(self.max_trajectory_points, int)
            or self.max_trajectory_points < 2
        ):
            raise ValueError("max_trajectory_points must be an integer of at least 2")
        for field_name in ("start_velocity_tolerance", "final_joint_tolerance"):
            value = getattr(self, field_name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be finite and non-negative")
        for field_name in (
            "final_position_tolerance_m",
            "final_orientation_tolerance_rad",
        ):
            value = getattr(self, field_name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field_name} must be finite and positive")
        displacement = self.ik_max_solution_joint_displacement
        if displacement is not None and (not math.isfinite(displacement) or displacement <= 0.0):
            raise ValueError("ik_max_solution_joint_displacement must be finite and positive")


@dataclass(frozen=True, slots=True)
class MotionServerConfig:
    groups: Mapping[str, MotionGroupConfig]
    state_timeout_ns: int = 200_000_000
    planning_timeout_ns: int = 100_000_000
    execution_timeout_margin_ns: int = 2_000_000_000
    final_state_timeout_ns: int = 200_000_000
    child_cancel_retry_ns: int = 100_000_000
    child_cancel_timeout_ns: int = 1_000_000_000
    result_cache_size: int = 128

    def __post_init__(self) -> None:
        groups = dict(self.groups)
        if not groups:
            raise ValueError("groups must contain at least one group")
        if any(name != group.name for name, group in groups.items()):
            raise ValueError("group mapping keys must match group names")
        object.__setattr__(self, "groups", MappingProxyType(groups))
        for field_name in (
            "state_timeout_ns",
            "planning_timeout_ns",
            "execution_timeout_margin_ns",
            "final_state_timeout_ns",
            "child_cancel_retry_ns",
            "child_cancel_timeout_ns",
        ):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{field_name} must be a positive integer number of nanoseconds")
        if self.child_cancel_retry_ns >= self.child_cancel_timeout_ns:
            raise ValueError("child_cancel_retry_ns must be less than child_cancel_timeout_ns")
        if (
            isinstance(self.result_cache_size, bool)
            or not isinstance(self.result_cache_size, int)
            or self.result_cache_size <= 0
        ):
            raise ValueError("result_cache_size must be a positive integer")


def load_config(path: str | Path) -> MotionServerConfig:
    config_path = Path(path).expanduser().resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("motion server config must be a YAML mapping")
    return motion_server_config_from_mapping(data, base_path=config_path.parent)


def motion_server_config_from_mapping(
    data: dict[str, Any], *, base_path: Path | None = None
) -> MotionServerConfig:
    allowed = {
        "groups",
        "state_timeout_ms",
        "planning_timeout_ms",
        "execution_timeout_margin_ms",
        "final_state_timeout_ms",
        "child_cancel_retry_ms",
        "child_cancel_timeout_ms",
        "result_cache_size",
    }
    _reject_unknown(data, allowed, "motion server")
    raw_groups = data.get("groups")
    if not isinstance(raw_groups, dict) or not raw_groups:
        raise ValueError("groups must be a non-empty mapping")
    root = (base_path or Path.cwd()).resolve()
    groups: dict[str, MotionGroupConfig] = {}
    for raw_name, raw_group in raw_groups.items():
        if not isinstance(raw_name, str) or not raw_name:
            raise ValueError("group names must be non-empty strings")
        if not isinstance(raw_group, dict):
            raise ValueError(f"group {raw_name!r} must be a mapping")
        groups[raw_name] = _group_from_mapping(raw_name, raw_group, root)
    return MotionServerConfig(
        groups=groups,
        state_timeout_ns=_milliseconds_to_ns(data.get("state_timeout_ms", 200), "state_timeout_ms"),
        planning_timeout_ns=_milliseconds_to_ns(
            data.get("planning_timeout_ms", 100), "planning_timeout_ms"
        ),
        execution_timeout_margin_ns=_milliseconds_to_ns(
            data.get("execution_timeout_margin_ms", 2000),
            "execution_timeout_margin_ms",
        ),
        final_state_timeout_ns=_milliseconds_to_ns(
            data.get("final_state_timeout_ms", 200), "final_state_timeout_ms"
        ),
        child_cancel_retry_ns=_milliseconds_to_ns(
            data.get("child_cancel_retry_ms", 100), "child_cancel_retry_ms"
        ),
        child_cancel_timeout_ns=_milliseconds_to_ns(
            data.get("child_cancel_timeout_ms", 1000), "child_cancel_timeout_ms"
        ),
        result_cache_size=_positive_int(data.get("result_cache_size", 128), "result_cache_size"),
    )


def _group_from_mapping(name: str, data: dict[str, Any], root: Path) -> MotionGroupConfig:
    allowed = {
        "joint_names",
        "urdf_path",
        "base_frame",
        "tip_frame",
        "max_velocity",
        "max_acceleration",
        "trajectory_sample_period_ms",
        "max_trajectory_points",
        "start_velocity_tolerance",
        "final_joint_tolerance",
        "final_position_tolerance_m",
        "final_orientation_tolerance_rad",
        "ik_max_solution_joint_displacement",
    }
    _reject_unknown(data, allowed, f"group {name!r}")
    joint_names = _string_list(data.get("joint_names"), f"groups.{name}.joint_names")
    urdf_value = data.get("urdf_path")
    if not isinstance(urdf_value, str) or not urdf_value:
        raise ValueError(f"groups.{name}.urdf_path must be a non-empty string")
    urdf_path = Path(urdf_value).expanduser()
    if not urdf_path.is_absolute():
        urdf_path = root / urdf_path
    base_frame = data.get("base_frame")
    tip_frame = data.get("tip_frame")
    if not isinstance(base_frame, str) or not isinstance(tip_frame, str):
        raise ValueError(f"groups.{name} base_frame and tip_frame must be strings")
    return MotionGroupConfig(
        name=name,
        joint_names=joint_names,
        urdf_path=urdf_path.resolve(),
        base_frame=base_frame,
        tip_frame=tip_frame,
        max_velocity=_joint_values(data.get("max_velocity"), joint_names, "max_velocity"),
        max_acceleration=_joint_values(
            data.get("max_acceleration"), joint_names, "max_acceleration"
        ),
        trajectory_sample_period_ns=_milliseconds_to_ns(
            data.get("trajectory_sample_period_ms", 50), "trajectory_sample_period_ms"
        ),
        max_trajectory_points=_positive_int(
            data.get("max_trajectory_points", 10_000), "max_trajectory_points"
        ),
        start_velocity_tolerance=float(data.get("start_velocity_tolerance", 0.1)),
        final_joint_tolerance=float(data.get("final_joint_tolerance", 0.03)),
        final_position_tolerance_m=float(data.get("final_position_tolerance_m", 0.01)),
        final_orientation_tolerance_rad=float(data.get("final_orientation_tolerance_rad", 0.05)),
        ik_max_solution_joint_displacement=_optional_positive_float(
            data.get("ik_max_solution_joint_displacement"),
            "ik_max_solution_joint_displacement",
        ),
    )


def _joint_values(value: Any, names: tuple[str, ...], field_name: str) -> tuple[float, ...]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a mapping keyed by joint name")
    unknown = sorted(set(value) - set(names))
    missing = [name for name in names if name not in value]
    if unknown or missing:
        raise ValueError(f"{field_name} keys differ: missing={missing}, unknown={unknown}")
    return tuple(float(value[name]) for name in names)


def _string_list(value: Any, field_name: str) -> tuple[str, ...]:
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise ValueError(f"{field_name} must be a list of strings")
    return tuple(value)


def _reject_unknown(data: dict[str, Any], allowed: set[str], label: str) -> None:
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ValueError(f"unknown {label} config fields: {unknown}")


def _milliseconds_to_ns(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(float(value)) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return int(round(float(value) * 1_000_000))


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _optional_positive_float(value: Any, name: str) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result
