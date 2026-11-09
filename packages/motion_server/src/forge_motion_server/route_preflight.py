"""Persistent, hardware-free JSON-lines route planner using execution IK.

This checks bounded sequential IK and sampled TCP clearance, not whole-arm
collision geometry. It never imports a robot driver or sends Dora commands.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time

import numpy as np
from forge_msgs import Pose

from .config import load_config
from .kinematics import ForgeKinematicsAdapter, matrix_to_pose, pose_to_matrix


def stage(name):
    name = name.rsplit(":", 1)[-1]
    if "descend" in name:
        return "descend"
    if name in ("hover", "lift"):
        return name
    if name == "lower":
        return "place"
    if name.startswith(("carry", "swing", "rise")):
        return "transit"
    if name.startswith(("withdraw", "retract", "prehome", "home", "tail_escape")):
        return "retreat"
    return "prepare"


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


class RoutePlanner:
    def __init__(self, config):
        self.config = config
        self.adapter = ForgeKinematicsAdapter(config)

    def plan(self, request):
        began = time.monotonic()
        deadline = began + min(float(request.get("timeout_s", 12)), 20)
        name = request["group_name"]
        runtime = self.adapter._runtime(name)
        group, solver = runtime.group, runtime.solver
        seed = np.asarray(request["seed_rad"], float)
        group.to_robot_state(seed)
        original = request["waypoints"]
        start = request.get("start_waypoints", [])
        if not original:
            raise ValueError("empty route")
        policies = request["stage_limits"]
        drop = request.get("drop", False)
        waypoint_names = {p["name"] for p in original}
        required = (
            {"hover", "descend", "lift", "swing"}
            if drop
            else {"hover", "descend", "lift", "carry", "lower", "withdraw"}
        )
        if not required <= waypoint_names:
            raise ValueError(
                "incomplete manipulation route: " + str(sorted(required - waypoint_names))
            )
        if drop and not any(x.startswith(("home", "park")) for x in waypoint_names):
            raise ValueError("drop return route is missing")
        bearing = np.arctan2(original[0]["target_pose"]["y"], original[0]["target_pose"]["x"])
        radial_axis = [-np.sin(bearing), np.cos(bearing), 0.0]
        # First preserve the supplied route. Search task-permitted shared pick
        # attitudes only if needed; return the first well-conditioned route.
        choices = [(0, 0)] + [
            (tilt, yaw)
            for tilt in (5, 10, 15, 20, 25, 0)
            for yaw in (0, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90)
            if (tilt, yaw) != (0, 0)
        ]
        best = None
        failures = []
        total_attempts = 0
        for tilt, yaw in choices:
            if time.monotonic() >= deadline:
                break
            total_attempts += 1
            route = copy.deepcopy(original)
            transform = rotation(radial_axis, np.deg2rad(-tilt)) @ rotation(
                [0, 0, 1], np.deg2rad(yaw)
            )
            for wp in route:
                if wp["name"] in ("park", "prehome", "home") or wp["name"].startswith("unlock_"):
                    continue
                t = pose_to_matrix(Pose(**wp["target_pose"]))
                t[:3, :3] = transform @ t[:3, :3]
                p = matrix_to_pose(t)
                wp["target_pose"].update(qx=p.qx, qy=p.qy, qz=p.qz, qw=p.qw)
            if drop:
                grasp_attitude = next(w["target_pose"] for w in route if w["name"] == "descend")
                lift_waypoint = next(w for w in route if w["name"] == "lift")
                lift_waypoint["target_pose"].update(
                    {key: grasp_attitude[key] for key in ("qx", "qy", "qz", "qw")}
                )
            q = seed.copy()
            diagnostics = []
            retries = []
            failed = False
            descend_q = None
            sequence = copy.deepcopy(start) + route
            for wp in sequence:
                kind = stage(wp["name"])
                limits = policies[kind]
                physical_position = min(limits["position_m"], limits["z_m"])
                ptol, otol = solver.tolerances(physical_position, limits["orientation_rad"])
                target = pose_to_matrix(Pose(**wp["target_pose"]))
                before = q.copy()
                from_z = group.forward(before)[2, 3]
                floor = min(from_z, target[2, 3]) - 0.003
                if kind == "transit" and not drop:
                    contact = next(p["target_pose"]["z"] for p in route if p["name"] == "descend")
                    lower = next(p["target_pose"]["z"] for p in route if p["name"] == "lower")
                    floor = max(contact, lower) + 0.020

                def valid(candidate, before=before, floor=floor):
                    points = [
                        group.forward(before + t * (candidate - before))[:3, 3]
                        for t in np.linspace(0, 1, 31)
                    ]
                    return min(float(p[2]) for p in points) >= floor

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    failed = True
                    break
                result = solver.solve(target, q, ptol, otol, min(remaining, 1.0), validator=valid)
                if result.positions is None:
                    failures.append(
                        {
                            "stage": wp["name"],
                            "tilt_deg": tilt,
                            "yaw_deg": yaw,
                            "diagnostics": result.diagnostics,
                        }
                    )
                    failed = True
                    break
                q = result.positions
                margin = float(np.min(np.minimum(q - group.lower_limits, group.upper_limits - q)))
                sampled_z = min(
                    float(group.forward(before + t * (q - before))[2, 3])
                    for t in np.linspace(0, 1, 31)
                )
                diagnostics.append(
                    {
                        "name": wp["name"],
                        "joints_rad": q.tolist(),
                        "joint_limit_margin_rad": margin,
                        "sampled_min_tcp_z_m": sampled_z,
                        "ik": result.diagnostics,
                    }
                )
                if wp["name"] == "descend":
                    descend_q = q.copy()
                if wp["name"] == "lift" and descend_q is not None:
                    retry = next(p for p in route if p["name"] == "descend")
                    retry_target = pose_to_matrix(Pose(**retry["target_pose"]))
                    retry_target[2, 3] -= 0.003
                    retry_limits = policies["descend"]
                    retry_tol = solver.tolerances(
                        min(retry_limits["position_m"], retry_limits["z_m"]),
                        retry_limits["orientation_rad"],
                    )
                    retry_result = solver.solve(
                        retry_target,
                        descend_q,
                        *retry_tol,
                        min(max(0.001, deadline - time.monotonic()), 1.0),
                    )
                    if retry_result.positions is None:
                        failed = True
                        break
                    retry_lift = solver.solve(
                        target,
                        retry_result.positions,
                        ptol,
                        otol,
                        min(max(0.001, deadline - time.monotonic()), 1.0),
                    )
                    if retry_lift.positions is None:
                        failed = True
                        break
                    # Check the altered retry path as well as both endpoints.
                    retry_floor = (
                        min(group.forward(retry_result.positions)[2, 3], target[2, 3]) - 0.003
                    )
                    retry_min = min(
                        float(
                            group.forward(
                                retry_result.positions
                                + t * (retry_lift.positions - retry_result.positions)
                            )[2, 3]
                        )
                        for t in np.linspace(0, 1, 31)
                    )
                    if retry_min < retry_floor:
                        failed = True
                        break
                    retries.extend(
                        [
                            {
                                "name": "retry_descend",
                                "joints_rad": retry_result.positions.tolist(),
                                "ik": retry_result.diagnostics,
                            },
                            {
                                "name": "retry_lift",
                                "joints_rad": retry_lift.positions.tolist(),
                                "sampled_min_tcp_z_m": retry_min,
                                "ik": retry_lift.diagnostics,
                            },
                        ]
                    )
            if failed:
                continue
            work = [x for x in diagnostics if stage(x["name"]) not in ("prepare", "retreat")]
            margin = min(x["joint_limit_margin_rad"] for x in work)
            if margin < .01:
                # Avoid returning a route that already rests on a hard stop;
                # live feedback and tracking will differ from the prediction.
                continue
            proposal = {
                "waypoints": route,
                "pose_planning": {
                    "checked": True,
                    "method": "motion_server_shared_robust_ik",
                    "drop": drop,
                    "tilt_deg": tilt,
                    "yaw_offset_deg": yaw,
                    "attempts": total_attempts,
                    "retry_checked": True,
                    "collision_checked": False,
                    "seed_rad": seed.tolist(),
                    "seed_received_ns": request.get("seed_received_ns"),
                    "minimum_work_joint_margin_rad": margin,
                    "route": diagnostics,
                    "retry_route": retries,
                },
            }
            if best is None or margin > best["pose_planning"]["minimum_work_joint_margin_rad"]:
                best = proposal
            if margin >= 0.02:
                break
            # Do not spend the entire latency budget optimizing a valid route.
            if best is not None and total_attempts >= 5:
                break
        if best is None:
            return {
                "ok": False,
                "error": "no checked sequential route within the planning budget",
                "attempts": total_attempts,
                "elapsed_s": time.monotonic() - began,
                "failures": failures[-3:],
            }
        best["pose_planning"]["elapsed_s"] = time.monotonic() - began
        best["pose_planning"]["attempts"] = total_attempts
        best["ok"] = True
        return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    planner = RoutePlanner(load_config(args.config))
    for line in sys.stdin:
        try:
            request = json.loads(line)
            response = planner.plan(request)
        except Exception as error:
            response = {"ok": False, "error": f"{type(error).__name__}: {error}"}
        print(json.dumps(response, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
