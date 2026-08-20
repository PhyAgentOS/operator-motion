"""PyInstaller entry point for the Motion action server."""

import cmeel
import dora
import forge_kinematics
import forge_msgs
import numpy
import pinocchio
import yaml
from forge_motion_server.main import main

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
    raise SystemExit(main())
