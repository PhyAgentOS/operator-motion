"""Forge motion planning and execution server."""

import sys
from pathlib import Path


def _vendored_forge_tool_src() -> str | None:
    """Locate the repo-checkout ``src`` dir containing the vendored forge_tool."""
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "forge_tool" / "__init__.py"
        if candidate.is_file():
            return str(candidate.parent.parent)
    return None


# In a repo checkout the forge_tool protocol is vendored at src/forge_tool; in a
# frozen executable it is already bundled and importable.
_VENDORED_SRC = _vendored_forge_tool_src()
if _VENDORED_SRC is not None and _VENDORED_SRC not in sys.path:
    sys.path.insert(0, _VENDORED_SRC)


from .action_endpoint import (
    ACTION_DESCRIPTOR,
    ActionShutdownOutcome,
    MotionActionEndpoint,
    goal_id_for_key,
)
from .config import (
    MotionGroupConfig,
    MotionServerConfig,
    load_config,
    motion_server_config_from_mapping,
)
from .kinematics import ForgeKinematicsAdapter, MotionKinematics
from .models import MotionServerOutput
from .node import MotionServerNode, run_dora_motion_server
from .planner import KinematicPlanner, MotionPlanner, PlannedMotion, PlanningFailure
from .server import MotionServer
from .state_cache import JointStateCache, JointStateSnapshot
from .trajectory import PlannedTrajectory, TrajectoryGenerationError, plan_quintic_trajectory

__all__ = [
    "ACTION_DESCRIPTOR",
    "ActionShutdownOutcome",
    "ForgeKinematicsAdapter",
    "JointStateCache",
    "JointStateSnapshot",
    "KinematicPlanner",
    "MotionGroupConfig",
    "MotionActionEndpoint",
    "MotionKinematics",
    "MotionPlanner",
    "MotionServer",
    "MotionServerConfig",
    "MotionServerNode",
    "MotionServerOutput",
    "PlannedMotion",
    "PlannedTrajectory",
    "PlanningFailure",
    "TrajectoryGenerationError",
    "goal_id_for_key",
    "load_config",
    "motion_server_config_from_mapping",
    "plan_quintic_trajectory",
    "run_dora_motion_server",
]
