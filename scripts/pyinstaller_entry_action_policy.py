"""PyInstaller entry point for the Motion Tool action policy."""

import cmeel
import dora
import forge_kinematics
import forge_msgs
import numpy
import pinocchio
import yaml
from forge_motion_server.policy_main import action_main

import forge_tool

_BUNDLED_DEPENDENCIES = (
    cmeel,
    dora,
    forge_kinematics,
    forge_msgs,
    forge_tool,
    numpy,
    pinocchio,
    yaml,
)


if __name__ == "__main__":
    raise SystemExit(action_main())
