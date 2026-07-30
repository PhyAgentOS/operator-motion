from __future__ import annotations

import math
from dataclasses import dataclass

from forge_msgs import JointState


@dataclass(frozen=True, slots=True)
class JointStateSnapshot:
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    received_ns: int


class JointStateCache:
    """Fresh named robot state cache shared by configured motion groups."""

    def __init__(self) -> None:
        self._positions: dict[str, float] = {}
        self._velocities: dict[str, float] = {}
        self._received_ns: int | None = None

    @property
    def received_ns(self) -> int | None:
        return self._received_ns

    def update(self, state: JointState, now_ns: int) -> None:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if self._received_ns is not None and now_ns < self._received_ns:
            raise ValueError("now_ns must be monotonic")
        if not state.position:
            raise ValueError("JointState.position must be complete")
        positions = dict(zip(state.name, state.position, strict=True))
        if any(not math.isfinite(float(value)) for value in positions.values()):
            raise ValueError("JointState positions must be finite")

        velocities: dict[str, float]
        if state.velocity:
            velocities = {
                name: float(value) for name, value in zip(state.name, state.velocity, strict=True)
            }
            if any(not math.isfinite(value) for value in velocities.values()):
                raise ValueError("JointState velocities must be finite")
        elif self._received_ns is not None and now_ns > self._received_ns:
            dt_s = (now_ns - self._received_ns) / 1_000_000_000.0
            velocities = {
                name: (float(position) - self._positions[name]) / dt_s
                if name in self._positions
                else 0.0
                for name, position in positions.items()
            }
        else:
            velocities = {name: 0.0 for name in positions}

        self._positions = {name: float(value) for name, value in positions.items()}
        self._velocities = velocities
        self._received_ns = now_ns

    def snapshot(
        self,
        joint_names: tuple[str, ...],
        now_ns: int,
        timeout_ns: int,
    ) -> JointStateSnapshot | None:
        if self._received_ns is None or now_ns - self._received_ns > timeout_ns:
            return None
        missing = [name for name in joint_names if name not in self._positions]
        if missing:
            raise ValueError(f"JointState is missing configured joints: {missing}")
        return JointStateSnapshot(
            joint_names=joint_names,
            positions=tuple(self._positions[name] for name in joint_names),
            velocities=tuple(self._velocities[name] for name in joint_names),
            received_ns=self._received_ns,
        )
