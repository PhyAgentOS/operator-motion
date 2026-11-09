import json
from dataclasses import replace
from pathlib import Path

from forge_motion_server.config import MotionServerConfig
from forge_motion_server.route_preflight import RoutePlanner
from test_robust_ik import adapter as adapter

FIXTURES = Path(__file__).parent / "fixtures"


def test_drop_checks_pick_release_and_entire_return_before_any_command(adapter):
    config = adapter._runtime("piper_arm").config
    planner = RoutePlanner(MotionServerConfig(groups={"piper_arm": config}))
    request = json.loads((FIXTURES / "drop-request.json").read_text())
    result = planner.plan(request)
    assert result["ok"], result
    proof = result["pose_planning"]
    assert proof["drop"]
    assert [x["name"] for x in proof["route"]] == [
        x["name"] for x in request["start_waypoints"] + request["waypoints"]
    ]
    assert [x["name"] for x in proof["retry_route"]] == ["retry_descend", "retry_lift"]
    assert proof["minimum_work_joint_margin_rad"] >= 0.02


def test_tabletop_uses_the_same_runtime_ik_and_preserves_contact_coordinates(adapter):
    config = adapter._runtime("piper_arm").config
    planner = RoutePlanner(MotionServerConfig(groups={"piper_arm": config}))
    request = json.loads((FIXTURES / "tabletop-request.json").read_text())
    result = planner.plan(request)
    assert result["ok"], result
    for before, after in zip(request["waypoints"], result["waypoints"], strict=True):
        for axis in ("x", "y", "z"):
            assert before["target_pose"][axis] == after["target_pose"][axis]
    assert result["pose_planning"]["method"] == "motion_server_shared_robust_ik"


def test_unreachable_post_release_return_rejects_whole_drop_route(adapter):
    config = replace(
        adapter._runtime("piper_arm").config, ik_max_attempts=2, ik_attempt_timeout_ms=15
    )
    planner = RoutePlanner(MotionServerConfig(groups={"piper_arm": config}))
    request = json.loads((FIXTURES / "drop-request.json").read_text())
    request["waypoints"][-1]["target_pose"]["x"] = 1.5
    request["timeout_s"] = 0.35
    result = planner.plan(request)
    assert not result["ok"]
    assert "waypoints" not in result
