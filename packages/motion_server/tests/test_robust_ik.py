import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from forge_motion_server.config import MotionGroupConfig, MotionServerConfig
from forge_motion_server.kinematics import ForgeKinematicsAdapter, pose_to_matrix
from forge_msgs import Pose

SEED = (
    0.44497169279595433,
    1.9481539643685906,
    -1.3274799757743672,
    0.0,
    1.0374411606779494,
    2.0162741650739293,
)
TARGET = Pose(x=0.3224, y=0.1559, z=0.0964, qx=0, qy=1, qz=0, qw=0)


@pytest.fixture
def adapter():
    urdf = Path(__file__).parent / "fixtures/bounded_arm.urdf"
    c = MotionGroupConfig(
        name="piper_arm",
        joint_names=tuple(f"joint{i}" for i in range(1, 7)),
        urdf_path=urdf,
        base_frame="arm_base",
        tip_frame="tcp",
        max_velocity=(1.5,) * 6,
        max_acceleration=(3.0,) * 6,
        ik_max_solution_joint_displacement=8.0,
    )
    return ForgeKinematicsAdapter(MotionServerConfig(groups={"piper_arm": c}))


def test_recorded_lift_reaches_original_goal_inside_hard_limits(adapter):
    result = adapter.inverse("piper_arm", TARGET, SEED, 1_000_000_000, 0.005, 0.15)
    assert result.status == "SUCCESS"
    rt = adapter._runtime("piper_arm")
    assert np.all(np.array(result.positions) >= rt.group.lower_limits)
    assert np.all(np.array(result.positions) <= rt.group.upper_limits)
    assert result.diagnostics["position_tolerance_m"] == 0.002
    assert result.diagnostics["attempts"][-1]["position_error_m"] <= 0.002
    assert result.diagnostics["attempts"][-1]["orientation_error_rad"] <= 0.15


def test_candidate_rejection_automatically_tries_other_numerical_solver(adapter):
    solver = adapter._runtime("piper_arm").solver
    observed = []

    def valid(q):
        observed.append(q.copy())
        return len(observed) > 1

    result = solver.solve(pose_to_matrix(TARGET), SEED, 0.002, 0.15, 1.0, validator=valid)
    assert result.positions is not None
    assert len(observed) >= 2
    assert result.diagnostics["attempts"][0]["status"] == "REJECTED_BY_VALIDATOR"
    assert result.diagnostics["attempts"][-1]["solver"] == "bounded"


def test_alternate_seeds_cannot_bypass_original_measured_displacement_bound(adapter):
    rt = adapter._runtime("piper_arm")
    rt.solver.config = replace(rt.config, ik_max_solution_joint_displacement=0.01)
    result = rt.solver.solve(pose_to_matrix(TARGET), SEED, 0.002, 0.15, 0.2)
    assert result.positions is None


def test_unreachable_target_has_finite_search_and_no_approximate_motion(adapter):
    solver = adapter._runtime("piper_arm").solver
    target = pose_to_matrix(TARGET)
    target[0, 3] = 2.0
    began = time.monotonic()
    result = solver.solve(target, SEED, 0.002, 0.15, 0.15)
    assert result.positions is None
    assert result.diagnostics["attempt_count"] <= 8
    assert time.monotonic() - began < 0.4
    assert "global unreachability is not established" in result.message


def test_invalid_measured_state_is_never_projected(adapter):
    solver = adapter._runtime("piper_arm").solver
    seed = np.array(SEED)
    seed[4] = 3.0
    with pytest.raises(ValueError):
        solver.solve(pose_to_matrix(TARGET), seed, 0.002, 0.15, 1.0)
