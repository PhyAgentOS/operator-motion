#!/usr/bin/env python3
"""Real Dora end-to-end client for MotionServer, controller, and fake Piper plant."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import pyarrow as pa
from dora import Node
from forge_motion_server.config import load_config
from forge_motion_server.kinematics import ForgeKinematicsAdapter
from forge_msgs import (
    JointState,
    MoveJointsFeedback,
    MoveJointsGoal,
    MoveJointsResult,
    MovePoseFeedback,
    MovePoseGoal,
    MovePoseResult,
)

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
SEED_POSITIONS = (0.0, 1.0, -1.0, 0.0, 0.0, 0.0)


class SmokeFailure(RuntimeError):
    pass


def _parameters(event: dict[str, Any]) -> dict[str, Any]:
    metadata = event.get("metadata")
    if not isinstance(metadata, dict):
        raise SmokeFailure("Action event is missing metadata")
    parameters = metadata.get("parameters")
    return parameters if isinstance(parameters, dict) else metadata


def _metadata(goal_id: str) -> dict[str, Any]:
    return {"goal_id": goal_id}


class MotionSmokeScenario:
    def __init__(self, node: Node, config_path: Path, result_file: Path, timeout_s: float) -> None:
        self.node = node
        self.config = load_config(config_path)
        self.kinematics = ForgeKinematicsAdapter(self.config)
        self.result_file = result_file
        self.deadline = time.monotonic() + timeout_s
        self.phase = "waiting_for_state"
        self.latest_positions: tuple[float, ...] | None = None
        self.goals: dict[str, MoveJointsGoal | MovePoseGoal] = {}
        self.feedback_counts: dict[str, int] = defaultdict(int)
        self.results: dict[str, list[MoveJointsResult | MovePoseResult]] = defaultdict(list)
        self.statuses: dict[str, list[str]] = defaultdict(list)
        self.cancel_sent = False
        self.active_duplicate_sent = False
        self.busy_result_received = False
        self.busy_execution_finished = False
        self.terminal_retry_sent = False
        self.completed = False

    def handle(self, event: dict[str, Any]) -> None:
        if time.monotonic() > self.deadline:
            raise SmokeFailure(f"smoke timed out in phase {self.phase}")
        event_type = event.get("type")
        if event_type == "ERROR":
            raise SmokeFailure(f"Dora node error: {event.get('error', 'unknown')}")
        if event_type != "INPUT":
            return
        input_id = event.get("id")
        if input_id == "joint_state":
            self._handle_state(JointState.from_arrow(event["value"]))
        elif input_id == "move_joints_feedback":
            self._handle_joints_feedback(event)
        elif input_id == "move_pose_feedback":
            self._handle_pose_feedback(event)
        elif input_id == "move_joints_result":
            self._handle_joints_result(event)
        elif input_id == "move_pose_result":
            self._handle_pose_result(event)

    def _handle_state(self, state: JointState) -> None:
        by_name = dict(zip(state.name, state.position, strict=True))
        missing = [name for name in JOINT_NAMES if name not in by_name]
        if missing:
            raise SmokeFailure(f"JointState is missing Piper joints: {missing}")
        positions = tuple(float(by_name[name]) for name in JOINT_NAMES)
        if any(not math.isfinite(value) for value in positions):
            raise SmokeFailure("JointState contains non-finite positions")
        self.latest_positions = positions
        if self.phase == "waiting_for_state":
            self._send_joints_goal(
                "seed",
                SEED_POSITIONS,
                velocity_scale=0.8,
                acceleration_scale=0.8,
            )
            self.phase = "seed_active"

    def _handle_joints_feedback(self, event: dict[str, Any]) -> None:
        goal_id = self._goal_id(event)
        feedback = MoveJointsFeedback.from_arrow(event["value"])
        self.feedback_counts[goal_id] += 1
        if goal_id == "cancel" and feedback.phase == "EXECUTING":
            if not self.cancel_sent and feedback.progress is not None and feedback.progress >= 0.1:
                self.cancel_sent = True
                self._send_cancel("move_joints_cancel", goal_id)
                self.phase = "cancel_requested"
        elif (
            goal_id == "busy-a" and feedback.phase == "EXECUTING" and not self.active_duplicate_sent
        ):
            self.active_duplicate_sent = True
            self._send_existing_goal("move_joints_goal", "busy-a")
            self._send_busy_pose()
            self.phase = "busy_and_duplicate_active"

    def _handle_pose_feedback(self, event: dict[str, Any]) -> None:
        goal_id = self._goal_id(event)
        MovePoseFeedback.from_arrow(event["value"])
        self.feedback_counts[goal_id] += 1

    def _handle_joints_result(self, event: dict[str, Any]) -> None:
        goal_id, status = self._terminal_metadata(event)
        result = MoveJointsResult.from_arrow(event["value"])
        self.results[goal_id].append(result)
        self.statuses[goal_id].append(status)
        if goal_id == "seed":
            self._expect(goal_id, "SUCCESS", "succeeded", 1)
            self._send_pose_goal()
            self.phase = "pose_active"
        elif goal_id == "cancel":
            self._expect(goal_id, "CANCELED", "canceled", 1)
            if not self.cancel_sent:
                raise SmokeFailure("cancel goal terminated before cancel was sent")
            self._send_busy_execution()
            self.phase = "busy_active"
        elif goal_id == "busy-a":
            count = len(self.results[goal_id])
            if count == 1:
                self._expect(goal_id, "SUCCESS", "succeeded", 1)
                if not self.active_duplicate_sent:
                    raise SmokeFailure("busy-a completed before active duplicate was sent")
                self.busy_execution_finished = True
                self._maybe_retry_terminal()
            elif count == 2:
                self._expect(goal_id, "SUCCESS", "succeeded", 2)
                if self.results[goal_id][0] != self.results[goal_id][1]:
                    raise SmokeFailure("terminal retry did not return cached MoveJoints result")
                self._finish()
            else:
                raise SmokeFailure("busy-a produced an unexpected extra terminal result")
        else:
            raise SmokeFailure(f"unexpected MoveJoints result for {goal_id!r}")

    def _handle_pose_result(self, event: dict[str, Any]) -> None:
        goal_id, status = self._terminal_metadata(event)
        result = MovePoseResult.from_arrow(event["value"])
        self.results[goal_id].append(result)
        self.statuses[goal_id].append(status)
        if goal_id == "pose":
            self._expect(goal_id, "SUCCESS", "succeeded", 1)
            if result.final_position_error_m is None or result.final_position_error_m > 0.01:
                raise SmokeFailure("MovePose final FK position residual is invalid")
            self._send_cancel_execution()
            self.phase = "cancel_active"
        elif goal_id == "busy-b":
            self._expect(goal_id, "BUSY", "aborted", 1)
            self.busy_result_received = True
            self._maybe_retry_terminal()
        else:
            raise SmokeFailure(f"unexpected MovePose result for {goal_id!r}")

    def _send_pose_goal(self) -> None:
        positions = self._positions()
        actual_pose = self.kinematics.forward("piper_arm", positions)
        target = actual_pose.model_copy(update={"x": actual_pose.x + 0.01})
        goal = MovePoseGoal(
            group_name="piper_arm",
            reference_frame="arm_base",
            target_frame="tcp",
            target_pose=target,
            velocity_scale=0.7,
            acceleration_scale=0.7,
            requested_duration_ns=None,
            position_tolerance_m=0.01,
            orientation_tolerance_rad=0.05,
        )
        self.goals["pose"] = goal
        self._send_existing_goal("move_pose_goal", "pose")

    def _send_cancel_execution(self) -> None:
        positions = self._positions()
        target = tuple(
            value + delta
            for value, delta in zip(positions, (0.3, 0.2, -0.2, 0.1, -0.1, 0.1), strict=True)
        )
        self._send_joints_goal(
            "cancel",
            target,
            velocity_scale=0.6,
            acceleration_scale=0.6,
            requested_duration_ns=2_000_000_000,
        )

    def _send_busy_execution(self) -> None:
        positions = self._positions()
        target = tuple(
            value + delta
            for value, delta in zip(positions, (0.2, 0.1, -0.1, 0.05, -0.05, 0.05), strict=True)
        )
        self._send_joints_goal(
            "busy-a",
            target,
            velocity_scale=0.7,
            acceleration_scale=0.7,
            requested_duration_ns=1_500_000_000,
        )

    def _send_busy_pose(self) -> None:
        actual_pose = self.kinematics.forward("piper_arm", self._positions())
        goal = MovePoseGoal(
            group_name="piper_arm",
            reference_frame="arm_base",
            target_frame="tcp",
            target_pose=actual_pose,
            velocity_scale=0.5,
            acceleration_scale=0.5,
            requested_duration_ns=None,
            position_tolerance_m=0.01,
            orientation_tolerance_rad=0.05,
        )
        self.goals["busy-b"] = goal
        self._send_existing_goal("move_pose_goal", "busy-b")

    def _send_joints_goal(
        self,
        goal_id: str,
        positions: tuple[float, ...],
        *,
        velocity_scale: float,
        acceleration_scale: float,
        requested_duration_ns: int | None = None,
    ) -> None:
        goal = MoveJointsGoal(
            group_name="piper_arm",
            joint_names=list(JOINT_NAMES),
            positions=list(positions),
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
            requested_duration_ns=requested_duration_ns,
        )
        self.goals[goal_id] = goal
        self._send_existing_goal("move_joints_goal", goal_id)

    def _send_existing_goal(self, output_id: str, goal_id: str) -> None:
        self.node.send_output(
            output_id,
            self.goals[goal_id].to_arrow(),
            metadata=_metadata(goal_id),
        )

    def _send_cancel(self, output_id: str, goal_id: str) -> None:
        self.node.send_output(
            output_id,
            pa.array([True], type=pa.bool_()),
            metadata=_metadata(goal_id),
        )

    def _maybe_retry_terminal(self) -> None:
        if self.busy_result_received and self.busy_execution_finished:
            if self.terminal_retry_sent:
                return
            self.terminal_retry_sent = True
            self._send_existing_goal("move_joints_goal", "busy-a")
            self.phase = "terminal_retry"

    def _expect(self, goal_id: str, code: str, status: str, count: int) -> None:
        if len(self.results[goal_id]) != count:
            raise SmokeFailure(f"unexpected result count for {goal_id}")
        if self.results[goal_id][-1].error_code != code:
            actual = self.results[goal_id][-1].error_code
            raise SmokeFailure(f"{goal_id} returned {actual}, expected {code}")
        if self.statuses[goal_id][-1] != status:
            actual = self.statuses[goal_id][-1]
            raise SmokeFailure(f"{goal_id} status was {actual}, expected {status}")

    def _goal_id(self, event: dict[str, Any]) -> str:
        goal_id = str(_parameters(event).get("goal_id", ""))
        if not goal_id:
            raise SmokeFailure("Action event has an empty goal_id")
        return goal_id

    def _terminal_metadata(self, event: dict[str, Any]) -> tuple[str, str]:
        parameters = _parameters(event)
        goal_id = str(parameters.get("goal_id", ""))
        status = str(parameters.get("goal_status", ""))
        if not goal_id or status not in {"succeeded", "aborted", "canceled"}:
            raise SmokeFailure("terminal Action metadata is invalid")
        return goal_id, status

    def _positions(self) -> tuple[float, ...]:
        if self.latest_positions is None:
            raise SmokeFailure("no JointState available")
        return self.latest_positions

    def _finish(self) -> None:
        expected_counts = {"seed": 1, "pose": 1, "cancel": 1, "busy-b": 1, "busy-a": 2}
        actual_counts = {goal_id: len(items) for goal_id, items in self.results.items()}
        if actual_counts != expected_counts:
            raise SmokeFailure(
                f"terminal counts differ: actual={actual_counts}, expected={expected_counts}"
            )
        report = {
            "status": "passed",
            "checks": [
                "move_joints_success",
                "move_pose_real_forge_ik",
                "final_fk_residual",
                "parent_cancel_propagation",
                "cross_action_busy",
                "active_duplicate_does_not_restart",
                "terminal_retry_returns_cached_result",
                "dora_action_metadata_and_arrow",
                "one_terminal_result_per_execution",
            ],
            "terminal_result_counts": actual_counts,
        }
        self.result_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        self.phase = "complete"
        self.completed = True


def main() -> int:
    parser = argparse.ArgumentParser(description="Forge MotionServer Dora smoke client")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--result-file", type=Path, default=Path("smoke-result.json"))
    parser.add_argument("--timeout-s", type=float, default=25.0)
    args = parser.parse_args()
    args.result_file.unlink(missing_ok=True)

    node = Node()
    scenario = MotionSmokeScenario(node, args.config, args.result_file, args.timeout_s)
    try:
        for event in node:
            if event.get("type") == "STOP":
                break
            scenario.handle(event)
            if scenario.completed:
                print("Forge MotionServer Dora smoke: PASSED", flush=True)
                return 0
    except Exception as error:
        report = {"status": "failed", "phase": scenario.phase, "error": str(error)}
        args.result_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"Forge MotionServer Dora smoke: FAILED: {error}", flush=True)
        return 1
    if not scenario.completed:
        report = {"status": "failed", "phase": scenario.phase, "error": "dataflow stopped early"}
        args.result_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
