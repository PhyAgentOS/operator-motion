#!/usr/bin/env python3
"""Run and verify the real multi-process Dora MotionServer smoke dataflow."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


def _dora_executable(explicit: str | None) -> str:
    candidate = explicit or os.environ.get("DORA_CLI") or shutil.which("dora")
    if not candidate:
        raise RuntimeError(
            "dora CLI was not found; install a Dora CLI compatible with dora-rs 0.4.1 "
            "or pass --dora /path/to/dora (DORA_CLI is also supported)"
        )
    return candidate


def _stop_process_group(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait(timeout=5.0)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the Forge MotionServer Dora smoke")
    parser.add_argument("--dora", help="Path to the dora CLI executable")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()

    example_dir = Path(__file__).resolve().parent
    dataflow = example_dir / "dataflow.smoke.yaml"
    result_file = example_dir / "smoke-result.json"
    result_file.unlink(missing_ok=True)

    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")

    process: subprocess.Popen[bytes] | None = None
    report: dict[str, object] | None = None
    try:
        dora = _dora_executable(args.dora)
        process = subprocess.Popen(
            [dora, "run", dataflow.name],
            cwd=example_dir,
            start_new_session=True,
        )
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            if result_file.exists():
                try:
                    report = json.loads(result_file.read_text(encoding="utf-8"))
                    break
                except json.JSONDecodeError:
                    pass
            if process.poll() is not None:
                break
            time.sleep(0.05)
    except (OSError, RuntimeError) as error:
        print(f"smoke runner failed: {error}", file=sys.stderr)
        return 1
    finally:
        if process is not None:
            _stop_process_group(process)

    if report is None:
        returncode = process.returncode if process is not None else "not started"
        print(
            f"smoke produced no valid result report (dora exit code {returncode})",
            file=sys.stderr,
        )
        return 1
    if report.get("status") != "passed":
        print(json.dumps(report, indent=2), file=sys.stderr)
        return 1

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
