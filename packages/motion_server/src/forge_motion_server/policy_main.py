from __future__ import annotations

import argparse

from .policy_nodes import run_action_policy


def action_main() -> int:
    argparse.ArgumentParser(description="Motion Action Tool policy node").parse_args()
    return run_action_policy()


if __name__ == "__main__":
    raise SystemExit(action_main())
