from __future__ import annotations

import argparse
import logging

from .config import load_config
from .node import run_dora_motion_server


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="Forge MoveJoints and MovePose Dora action server")
    parser.add_argument("--config", required=True, help="Path to the motion server YAML config")
    args = parser.parse_args()
    return run_dora_motion_server(load_config(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
