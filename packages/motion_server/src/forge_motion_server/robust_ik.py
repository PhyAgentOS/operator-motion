"""Bounded IK retries shared by motion execution and offline route preflight.

Retries are numerical searches only: no candidate is executed here. Every
accepted candidate is independently checked against the original request and
the original measured seed, including after alternate-seed searches.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from forge_kinematics import IKOptions, IKRequest, PoseTarget
from forge_kinematics._solver_math import _so3_log
from forge_kinematics.bounded_least_squares import (
    BoundedLeastSquaresConfig,
    PinocchioBoundedLeastSquaresSolver,
)
from forge_kinematics.dls import DlsConfig, PinocchioDlsSolver


@dataclass
class RobustIKResult:
    positions: np.ndarray | None
    status: str
    message: str
    diagnostics: dict


class RobustIK:
    def __init__(self, group, config):
        self.group, self.config = group, config
        self.primary = PinocchioDlsSolver(group, DlsConfig(damping=config.ik_damping))
        self.damped = PinocchioDlsSolver(group, DlsConfig(damping=max(0.1, 2 * config.ik_damping)))
        self.bounded = PinocchioBoundedLeastSquaresSolver(
            group, BoundedLeastSquaresConfig(max_nfev=80)
        )
        self.cached = None

    def tolerances(self, position, orientation):
        return min(
            position * self.config.ik_position_tolerance_scale,
            self.config.ik_position_tolerance_max_m,
        ), orientation

    def solve(
        self,
        target,
        measured_seed,
        position_tolerance,
        orientation_tolerance,
        timeout_s,
        validator: Callable | None = None,
    ):
        began = time.monotonic()
        deadline = began + timeout_s
        group, cfg = self.group, self.config
        original = np.asarray(measured_seed, dtype=float)
        # Validate real state, rather than silently projecting a measured seed.
        group.to_robot_state(original)
        if np.any(original < group.lower_limits) or np.any(original > group.upper_limits):
            raise ValueError("measured seed is outside configured joint limits")
        target = np.asarray(target, dtype=float)
        base = original.copy()
        candidates = [
            ("dls", self.primary, base),
            ("bounded", self.bounded, base),
            ("dls_damped", self.damped, base),
        ]
        if self.cached is not None:
            candidates.append(("bounded_cached", self.bounded, self.cached.copy()))
        rng = np.random.default_rng(2401)
        for index in range(cfg.ik_max_attempts):
            if index == 0:
                seed = (group.lower_limits + group.upper_limits) / 2
            else:
                seed = original + rng.normal(0, cfg.ik_seed_perturbation_rad, len(original))
            seed = np.clip(seed, group.lower_limits, group.upper_limits)
            candidates.append(("bounded_seed", self.bounded, seed))
        attempts = []
        accepted = None
        for name, solver, seed in candidates[: cfg.ik_max_attempts]:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            request = IKRequest(
                seed=seed,
                targets=[
                    PoseTarget(
                        tip_frame=group.tip_frames[0],
                        reference_frame=group.base_frame,
                        pose=target,
                        orientation_weight=cfg.ik_orientation_weight,
                    )
                ],
                options=IKOptions(
                    timeout_s=min(remaining, cfg.ik_attempt_timeout_ms / 1000),
                    position_tolerance_m=position_tolerance,
                    orientation_tolerance_rad=orientation_tolerance,
                    max_solution_joint_displacement=cfg.ik_max_solution_joint_displacement,
                    allow_approximate_solution=False,
                ),
            )
            result = solver.solve(request)
            row = {
                "solver": name,
                "status": result.status.value,
                "iterations": result.iterations,
                "elapsed_s": result.elapsed_s,
                "position_error_m": result.raw_position_error_m,
                "orientation_error_rad": result.raw_orientation_error_rad,
                "minimum_singular_value": result.minimum_singular_value,
                "joint_limit_margin": result.minimum_joint_limit_margin,
                "message": result.message,
            }
            attempts.append(row)
            if result.solution is None:
                continue
            q = np.asarray(result.solution)
            fk = group.forward(q)
            pos = float(np.linalg.norm(fk[:3, 3] - target[:3, 3]))
            ori = float(np.linalg.norm(_so3_log(target[:3, :3] @ fk[:3, :3].T)))
            displacement = np.abs(group.difference(original, q))
            good = (
                np.all(np.isfinite(q))
                and np.all(q >= group.lower_limits)
                and np.all(q <= group.upper_limits)
                and pos <= position_tolerance
                and ori <= orientation_tolerance
            )
            if cfg.ik_max_solution_joint_displacement is not None:
                good &= np.all(displacement <= cfg.ik_max_solution_joint_displacement)
            row.update(
                position_error_m=pos,
                orientation_error_rad=ori,
                max_joint_displacement_rad=float(np.max(displacement)),
            )
            if good and validator is not None:
                good = bool(validator(q))
                if not good:
                    row["message"] = "candidate rejected by path/state validator"
            if not good:
                row["status"] = "REJECTED_BY_VALIDATOR"
                continue
            accepted = q.copy()
            self.cached = q.copy()
            break
        elapsed = time.monotonic() - began
        status = (
            "SUCCESS"
            if accepted is not None
            else ("TIMED_OUT" if time.monotonic() >= deadline else "NO_SOLUTION")
        )
        message = (
            "validated IK candidate"
            if accepted is not None
            else "bounded IK search exhausted; global unreachability is not established"
        )
        diag = {
            "status": status,
            "elapsed_s": elapsed,
            "attempt_count": len(attempts),
            "position_tolerance_m": position_tolerance,
            "orientation_tolerance_rad": orientation_tolerance,
            "orientation_weight": cfg.ik_orientation_weight,
            "seed_rad": original.tolist(),
            "target_matrix": target.tolist(),
            "attempts": attempts,
        }
        return RobustIKResult(accepted, status, message, diag)
