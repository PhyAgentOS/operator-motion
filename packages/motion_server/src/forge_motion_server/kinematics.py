from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from forge_kinematics import (
    IKOptions,
    IKRequest,
    IKStatus,
    KinematicGroup,
    PinocchioDlsSolver,
    PoseTarget,
    RobotModel,
)
from forge_msgs import Pose

from .config import MotionGroupConfig, MotionServerConfig


@dataclass(frozen=True, slots=True)
class InverseKinematicsOutcome:
    status: str
    message: str
    positions: tuple[float, ...] | None


class MotionKinematics(Protocol):
    def validate_positions(self, group_name: str, positions: tuple[float, ...]) -> None: ...

    def inverse(
        self,
        group_name: str,
        target_pose: Pose,
        seed_positions: tuple[float, ...],
        timeout_ns: int,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
    ) -> InverseKinematicsOutcome: ...

    def forward(self, group_name: str, positions: tuple[float, ...]) -> Pose: ...


@dataclass(slots=True)
class _GroupRuntime:
    config: MotionGroupConfig
    model: RobotModel
    group: KinematicGroup
    solver: PinocchioDlsSolver


class ForgeKinematicsAdapter:
    """Adapter from motion semantics to the public Forge kinematics library."""

    def __init__(self, config: MotionServerConfig) -> None:
        models: dict[Path, RobotModel] = {}
        runtimes: dict[str, _GroupRuntime] = {}
        for group_config in config.groups.values():
            model = models.get(group_config.urdf_path)
            if model is None:
                model = RobotModel.from_urdf(group_config.urdf_path)
                models[group_config.urdf_path] = model
            group = model.create_group(
                name=group_config.name,
                joint_names=group_config.joint_names,
                base_frame=group_config.base_frame,
                tip_frames=(group_config.tip_frame,),
            )
            runtimes[group_config.name] = _GroupRuntime(
                config=group_config,
                model=model,
                group=group,
                solver=PinocchioDlsSolver(group),
            )
        self._runtimes = runtimes

    def validate_positions(self, group_name: str, positions: tuple[float, ...]) -> None:
        runtime = self._runtime(group_name)
        runtime.group.to_robot_state(positions)

    def inverse(
        self,
        group_name: str,
        target_pose: Pose,
        seed_positions: tuple[float, ...],
        timeout_ns: int,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
    ) -> InverseKinematicsOutcome:
        runtime = self._runtime(group_name)
        target_matrix = pose_to_matrix(target_pose)
        context_state = runtime.model.create_state(
            dict(zip(runtime.config.joint_names, seed_positions, strict=True))
        )
        result = runtime.solver.solve(
            IKRequest(
                seed=seed_positions,
                targets=(
                    PoseTarget(
                        tip_frame=runtime.config.tip_frame,
                        reference_frame=runtime.config.base_frame,
                        pose=target_matrix,
                    ),
                ),
                options=IKOptions(
                    timeout_s=timeout_ns / 1_000_000_000.0,
                    position_tolerance_m=position_tolerance_m,
                    orientation_tolerance_rad=orientation_tolerance_rad,
                    max_solution_joint_displacement=(
                        runtime.config.ik_max_solution_joint_displacement
                    ),
                    allow_approximate_solution=False,
                ),
                context_state=context_state,
            )
        )
        positions = None if result.solution is None else tuple(float(v) for v in result.solution)
        return InverseKinematicsOutcome(
            status=result.status.value,
            message=result.message,
            positions=positions,
        )

    def forward(self, group_name: str, positions: tuple[float, ...]) -> Pose:
        runtime = self._runtime(group_name)
        return matrix_to_pose(runtime.group.forward(positions))

    def _runtime(self, group_name: str) -> _GroupRuntime:
        try:
            return self._runtimes[group_name]
        except KeyError as error:
            raise ValueError(f"unknown kinematic group {group_name!r}") from error


def pose_to_matrix(pose: Pose) -> np.ndarray:
    values = (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("pose values must be finite")
    norm = math.sqrt(pose.qx**2 + pose.qy**2 + pose.qz**2 + pose.qw**2)
    if norm <= 1e-12:
        raise ValueError("pose quaternion norm is too small")
    x, y, z, w = (pose.qx / norm, pose.qy / norm, pose.qz / norm, pose.qw / norm)
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.array([pose.x, pose.y, pose.z], dtype=np.float64)
    return matrix


def matrix_to_pose(matrix: object) -> Pose:
    transform = np.array(matrix, dtype=np.float64, copy=True)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("FK transform must be a finite 4x4 matrix")
    rotation = transform[:3, :3]
    qx, qy, qz, qw = _quaternion_from_rotation(rotation)
    return Pose(
        x=float(transform[0, 3]),
        y=float(transform[1, 3]),
        z=float(transform[2, 3]),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
    )


def pose_error(actual: Pose, target: Pose) -> tuple[float, float]:
    actual_matrix = pose_to_matrix(actual)
    target_matrix = pose_to_matrix(target)
    position_error = float(np.linalg.norm(target_matrix[:3, 3] - actual_matrix[:3, 3]))
    relative = actual_matrix[:3, :3].T @ target_matrix[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return position_error, math.acos(cosine)


def _quaternion_from_rotation(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


__all__ = [
    "ForgeKinematicsAdapter",
    "IKStatus",
    "InverseKinematicsOutcome",
    "MotionKinematics",
    "matrix_to_pose",
    "pose_error",
    "pose_to_matrix",
]
