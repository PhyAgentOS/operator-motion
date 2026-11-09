"""Recorded route and nearby-scene offline latency/feasibility comparison."""

import argparse
import json
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
from forge_motion_server.config import MotionServerConfig, load_config
from forge_motion_server.route_preflight import RoutePlanner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1] / "packages/motion_server/tests/fixtures"
    config = load_config(args.config)
    base = next(iter(config.groups.values()))
    cases = []
    for kind in ("tabletop", "drop"):
        template = json.loads((root / f"{kind}-request.json").read_text())
        for index, (dx, dy) in enumerate(
            [(0, 0), (0.006, 0), (-0.006, 0), (0, 0.006), (0, -0.006)]
        ):
            request = json.loads(json.dumps(template))
            for wp in request["waypoints"]:
                if wp["name"] not in ("park", "prehome", "home"):
                    wp["target_pose"]["x"] += dx
                    wp["target_pose"]["y"] += dy
            cases.append((f"{kind}_{index}", request))
    results = []
    for weight in (0.01, 0.0225, 0.04):
        for damping in (0.03, 0.05, 0.1):
            group = replace(base, ik_orientation_weight=weight, ik_damping=damping)
            planner = RoutePlanner(MotionServerConfig(groups={group.name: group}))
            rows = []
            for name, request in cases:
                began = time.monotonic()
                response = planner.plan(request)
                row = {"case": name, "ok": response["ok"], "elapsed_s": time.monotonic() - began}
                if response["ok"]:
                    proof = response["pose_planning"]
                    rows_ik = [s["ik"] for s in proof["route"] + proof["retry_route"]]
                    row.update(
                        attitude_attempts=proof["attempts"],
                        margin_rad=proof["minimum_work_joint_margin_rad"],
                        ik_calls=len(rows_ik),
                        ik_attempts=sum(x["attempt_count"] for x in rows_ik),
                        maximum_ik_position_error_m=max(
                            x["attempts"][-1]["position_error_m"] for x in rows_ik
                        ),
                        maximum_ik_orientation_error_rad=max(
                            x["attempts"][-1]["orientation_error_rad"] for x in rows_ik
                        ),
                    )
                rows.append(row)
            result = {
                "orientation_weight": weight,
                "damping": damping,
                "successes": sum(x["ok"] for x in rows),
                "cases": len(rows),
                "p50_s": float(np.percentile([x["elapsed_s"] for x in rows], 50)),
                "p95_s": float(np.percentile([x["elapsed_s"] for x in rows], 95)),
                "rows": rows,
            }
            results.append(result)
            print({k: v for k, v in result.items() if k != "rows"}, flush=True)
            Path(args.output).write_text(
                json.dumps(
                    {
                        "scope": (
                            "offline, recorded + 6mm XY perturbations; "
                            "not hardware reliability"
                        ),
                        "results": results,
                    },
                    indent=2,
                )
            )


if __name__ == "__main__":
    main()
