from __future__ import annotations

from pathlib import Path

import pytest
from forge_motion_server.config import MotionGroupConfig, MotionServerConfig
from forge_motion_server.kinematics import InverseKinematicsOutcome
from forge_motion_server.server import MotionServer
from forge_msgs import MoveJointsGoal, MovePoseGoal, Pose


class FakeKinematics:
    def __init__(self) -> None:
        self.inverse_outcome = InverseKinematicsOutcome(
            status="SUCCESS",
            message="solved",
            positions=(0.2, -0.1, 0.3),
        )
        self.inverse_calls = 0

    def validate_positions(self, group_name: str, positions: tuple[float, ...]) -> None:
        if group_name != "arm":
            raise ValueError("unknown group")
        if len(positions) != 3 or any(abs(value) > 2.0 for value in positions):
            raise ValueError("position outside fake joint limits")

    def inverse(
        self,
        group_name: str,
        target_pose: Pose,
        seed_positions: tuple[float, ...],
        timeout_ns: int,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
    ) -> InverseKinematicsOutcome:
        self.inverse_calls += 1
        return self.inverse_outcome

    def forward(self, group_name: str, positions: tuple[float, ...]) -> Pose:
        return Pose(x=positions[0], y=positions[1], z=positions[2])


@pytest.fixture
def group_config() -> MotionGroupConfig:
    return MotionGroupConfig(
        name="arm",
        joint_names=("j1", "j2", "j3"),
        urdf_path=Path("unused.urdf"),
        base_frame="base",
        tip_frame="tcp",
        max_velocity=(1.0, 1.0, 1.0),
        max_acceleration=(2.0, 2.0, 2.0),
        trajectory_sample_period_ns=20_000_000,
        start_velocity_tolerance=0.1,
        final_joint_tolerance=0.03,
        final_position_tolerance_m=0.02,
        final_orientation_tolerance_rad=0.05,
    )


@pytest.fixture
def server_config(group_config: MotionGroupConfig) -> MotionServerConfig:
    return MotionServerConfig(
        groups={"arm": group_config},
        state_timeout_ns=200_000_000,
        planning_timeout_ns=50_000_000,
        child_cancel_retry_ns=20_000_000,
        child_cancel_timeout_ns=100_000_000,
        result_cache_size=4,
    )


@pytest.fixture
def fake_kinematics() -> FakeKinematics:
    return FakeKinematics()


@pytest.fixture
def server(server_config: MotionServerConfig, fake_kinematics: FakeKinematics) -> MotionServer:
    return MotionServer(server_config, fake_kinematics)


@pytest.fixture
def joints_goal() -> MoveJointsGoal:
    return MoveJointsGoal(
        group_name="arm",
        joint_names=["j1", "j2", "j3"],
        positions=[0.3, -0.2, 0.1],
        velocity_scale=0.8,
        acceleration_scale=0.7,
        requested_duration_ns=None,
    )


@pytest.fixture
def pose_goal() -> MovePoseGoal:
    return MovePoseGoal(
        group_name="arm",
        reference_frame="base",
        target_frame="tcp",
        target_pose=Pose(x=0.2, y=-0.1, z=0.3),
        velocity_scale=0.8,
        acceleration_scale=0.7,
        requested_duration_ns=None,
        position_tolerance_m=0.02,
        orientation_tolerance_rad=0.05,
    )
