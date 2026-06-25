"""Reference trajectories for the px4_multirotor_controller UAV NMPC OCP."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable

import numpy as np

from .dynamics import QuadrotorParams, default_quadrotor_params
from .math_utils import pack_state, rotation_from_body_z, vee


@dataclass(frozen=True)
class FlatOutput:
    """Differential-flatness style position derivatives."""

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray


class BaseTrajectory:
    """Base class for state/control reference generation."""

    def __init__(self, dt: float = 0.05, params: QuadrotorParams | None = None) -> None:
        self.dt = float(dt)
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        self.params = params if params is not None else default_quadrotor_params()

    def flat_output(self, time: float) -> FlatOutput:
        raise NotImplementedError

    def initial_state(self) -> tuple[np.ndarray, np.ndarray]:
        flat = self.flat_output(0.0)
        return flat.position.copy(), flat.velocity.copy()

    def get_reference(self, time: float) -> tuple[np.ndarray, np.ndarray]:
        flat = self.flat_output(float(time))
        rotation = self._rotation_at(time)
        omega = self._omega_at(time)
        angular_acceleration = self._angular_acceleration_at(time)
        thrust_specific = np.linalg.norm(flat.acceleration + self.params.g * self.params.e3)

        xref = pack_state(flat.position, flat.velocity, rotation, omega)
        uref = np.concatenate(([thrust_specific], angular_acceleration))
        return xref, uref

    def _rotation_at(self, time: float) -> np.ndarray:
        flat = self.flat_output(float(time))
        thrust_vector = flat.acceleration + self.params.g * self.params.e3
        return rotation_from_body_z(thrust_vector, yaw=0.0)

    def _omega_at(self, time: float) -> np.ndarray:
        dt = self.dt
        time = float(time)
        if time < 0.5 * dt:
            r0 = self._rotation_at(time)
            rp = self._rotation_at(time + dt)
            rpp = self._rotation_at(time + 2.0 * dt)
            rdot = (-3.0 * r0 + 4.0 * rp - rpp) / (2.0 * dt)
            omega_hat = r0.T @ rdot
        else:
            rm = self._rotation_at(time - dt)
            r0 = self._rotation_at(time)
            rp = self._rotation_at(time + dt)
            rdot = (rp - rm) / (2.0 * dt)
            omega_hat = r0.T @ rdot
        return vee(0.5 * (omega_hat - omega_hat.T))

    def _angular_acceleration_at(self, time: float) -> np.ndarray:
        dt = self.dt
        time = float(time)
        w0 = self._omega_at(time)
        wp = self._omega_at(time + dt)
        if time < 0.5 * dt:
            wpp = self._omega_at(time + 2.0 * dt)
            return (-3.0 * w0 + 4.0 * wp - wpp) / (2.0 * dt)
        wm = self._omega_at(time - dt)
        return (wp - wm) / (2.0 * dt)


class HoverTrajectory(BaseTrajectory):
    """Hover at a fixed target position."""

    def __init__(
        self,
        target_pos: Iterable[float] = (1.0, 1.0, 1.0),
        dt: float = 0.05,
        params: QuadrotorParams | None = None,
    ) -> None:
        super().__init__(dt=dt, params=params)
        self.target_pos = np.asarray(target_pos, dtype=float).reshape(3)

    def flat_output(self, time: float) -> FlatOutput:
        return FlatOutput(
            position=self.target_pos.copy(),
            velocity=np.zeros(3),
            acceleration=np.zeros(3),
            jerk=np.zeros(3),
        )

    def initial_state(self) -> tuple[np.ndarray, np.ndarray]:
        return self.target_pos.copy(), np.zeros(3)

    def get_reference(self, time: float) -> tuple[np.ndarray, np.ndarray]:
        xref = pack_state(self.target_pos, np.zeros(3), np.eye(3), np.zeros(3))
        return xref, np.array([self.params.g, 0.0, 0.0, 0.0], dtype=float)


class LineTrajectory(BaseTrajectory):
    """Quintic polynomial line from ``s0`` to ``sd``."""

    def __init__(
        self,
        s0: Iterable[float] = (0.0, 0.0, 0.0),
        v0: Iterable[float] = (0.0, 0.0, 0.0),
        sd: Iterable[float] = (1.0, 1.0, 1.0),
        vd: Iterable[float] = (0.0, 0.0, 0.0),
        duration: float = 8.0,
        dt: float = 0.05,
        params: QuadrotorParams | None = None,
    ) -> None:
        super().__init__(dt=dt, params=params)
        self.s0 = np.asarray(s0, dtype=float).reshape(3)
        self.v0 = np.asarray(v0, dtype=float).reshape(3)
        self.sd = np.asarray(sd, dtype=float).reshape(3)
        self.vd = np.asarray(vd, dtype=float).reshape(3)
        self.duration = float(duration)
        if self.duration <= 0.0:
            raise ValueError("duration must be positive")
        self._coeff = self._compute_poly5_coeff()

    def _compute_poly5_coeff(self) -> tuple[np.ndarray, ...]:
        t_final = self.duration
        a0 = self.s0
        a1 = self.v0
        a2 = np.zeros(3)
        a_delta = self.sd - self.s0 - self.v0 * t_final
        d_delta = self.v0 - self.vd
        a3 = (10.0 * a_delta + 4.0 * d_delta * t_final) / (t_final**3)
        a4 = -(15.0 * a_delta + 7.0 * d_delta * t_final) / (t_final**4)
        a5 = (3.0 * (2.0 * a_delta + d_delta * t_final)) / (t_final**5)
        return a0, a1, a2, a3, a4, a5

    def flat_output(self, time: float) -> FlatOutput:
        tau = min(max(float(time), 0.0), self.duration)
        a0, a1, a2, a3, a4, a5 = self._coeff
        position = a0 + a1 * tau + a3 * tau**3 + a4 * tau**4 + a5 * tau**5
        velocity = a1 + 3.0 * a3 * tau**2 + 4.0 * a4 * tau**3 + 5.0 * a5 * tau**4
        acceleration = 6.0 * a3 * tau + 12.0 * a4 * tau**2 + 20.0 * a5 * tau**3
        jerk = 6.0 * a3 + 24.0 * a4 * tau + 60.0 * a5 * tau**2
        return FlatOutput(position, velocity, acceleration, jerk)


class LemniscateTrajectory(BaseTrajectory):
    """3D lemniscate of Bernoulli in the XY plane."""

    def __init__(
        self,
        radius: float = 1.0,
        omega: float = 0.9,
        height: float = 1.0,
        dt: float = 0.05,
        params: QuadrotorParams | None = None,
    ) -> None:
        super().__init__(dt=dt, params=params)
        self.radius = float(radius)
        self.omega = float(omega)
        self.height = float(height)

    def flat_output(self, time: float) -> FlatOutput:
        t = float(time)
        a = self.radius
        w = self.omega
        sin_wt = np.sin(w * t)
        cos_wt = np.cos(w * t)
        position = np.array([a * sin_wt, a * sin_wt * cos_wt, self.height])
        velocity = np.array([a * w * cos_wt, a * w * (cos_wt**2 - sin_wt**2), 0.0])
        acceleration = np.array([-a * w**2 * sin_wt, -4.0 * a * w**2 * sin_wt * cos_wt, 0.0])
        jerk = np.array(
            [-a * w**3 * cos_wt, -4.0 * a * w**3 * (cos_wt**2 - sin_wt**2), 0.0]
        )
        return FlatOutput(position, velocity, acceleration, jerk)


class HelixYZTrajectory(BaseTrajectory):
    """Helix in the YZ plane with slow linear motion in X."""

    def __init__(
        self,
        radius: float = 1.0,
        omega: float = 1.5,
        helix_scl: float = 10.0,
        dt: float = 0.05,
        params: QuadrotorParams | None = None,
    ) -> None:
        super().__init__(dt=dt, params=params)
        self.radius = float(radius)
        self.omega = float(omega)
        self.helix_scl = float(helix_scl)

    def flat_output(self, time: float) -> FlatOutput:
        t = float(time)
        w = self.omega
        r = self.radius
        s = self.helix_scl
        position = np.array([t / s, r * np.cos(w * t), r * np.sin(w * t)])
        velocity = np.array([1.0 / s, -r * w * np.sin(w * t), r * w * np.cos(w * t)])
        acceleration = np.array([0.0, -r * w**2 * np.cos(w * t), -r * w**2 * np.sin(w * t)])
        jerk = np.array([0.0, r * w**3 * np.sin(w * t), -r * w**3 * np.cos(w * t)])
        return FlatOutput(position, velocity, acceleration, jerk)


class HelixXYTrajectory(BaseTrajectory):
    """Helix in the XY plane with slow linear motion in Z."""

    def __init__(
        self,
        radius: float = 1.0,
        omega: float = 0.9,
        helix_scl: float = 10.0,
        dt: float = 0.05,
        params: QuadrotorParams | None = None,
    ) -> None:
        super().__init__(dt=dt, params=params)
        self.radius = float(radius)
        self.omega = float(omega)
        self.helix_scl = float(helix_scl)

    def flat_output(self, time: float) -> FlatOutput:
        t = float(time)
        w = self.omega
        r = self.radius
        s = self.helix_scl
        position = np.array([r * np.cos(w * t), r * np.sin(w * t), t / s])
        velocity = np.array([-r * w * np.sin(w * t), r * w * np.cos(w * t), 1.0 / s])
        acceleration = np.array([-r * w**2 * np.cos(w * t), -r * w**2 * np.sin(w * t), 0.0])
        jerk = np.array([r * w**3 * np.sin(w * t), -r * w**3 * np.cos(w * t), 0.0])
        return FlatOutput(position, velocity, acceleration, jerk)


class TorusKnotTrajectory(BaseTrajectory):
    """Torus-knot reference used by the Python NMPC validation runner."""

    def __init__(
        self,
        omega: float = 0.9,
        scale: float = 0.3,
        dt: float = 0.05,
        params: QuadrotorParams | None = None,
    ) -> None:
        super().__init__(dt=dt, params=params)
        self.omega = float(omega)
        self.scale = float(scale)

    def flat_output(self, time: float) -> FlatOutput:
        t = float(time)
        w = self.omega
        sc = self.scale
        position = sc * np.array(
            [
                np.sin(w * t) + 2.0 * np.sin(2.0 * w * t),
                np.cos(w * t) - 2.0 * np.cos(2.0 * w * t),
                4.0 + np.sin(3.0 * w * t),
            ]
        )
        velocity = sc * np.array(
            [
                w * np.cos(w * t) + 4.0 * w * np.cos(2.0 * w * t),
                -w * np.sin(w * t) + 4.0 * w * np.sin(2.0 * w * t),
                3.0 * w * np.cos(3.0 * w * t),
            ]
        )
        acceleration = sc * np.array(
            [
                -(w**2) * np.sin(w * t) - 8.0 * w**2 * np.sin(2.0 * w * t),
                -(w**2) * np.cos(w * t) + 8.0 * w**2 * np.cos(2.0 * w * t),
                -9.0 * w**2 * np.sin(3.0 * w * t),
            ]
        )
        jerk = sc * np.array(
            [
                -(w**3) * np.cos(w * t) - 16.0 * w**3 * np.cos(2.0 * w * t),
                w**3 * np.sin(w * t) - 16.0 * w**3 * np.sin(2.0 * w * t),
                -27.0 * w**3 * np.cos(3.0 * w * t),
            ]
        )
        return FlatOutput(position, velocity, acceleration, jerk)


TRAJECTORY_NAMES = {
    1: "hover",
    2: "line",
    3: "lemniscate",
    4: "helix_yz",
    5: "helix_xy",
    6: "torus_knot",
}

_NAME_TO_ID = {name: task_id for task_id, name in TRAJECTORY_NAMES.items()}


def normalize_task_id(task: int | str) -> int:
    """Normalize a numeric or named trajectory identifier."""

    if isinstance(task, str):
        key = task.strip().lower().replace("-", "_")
        if key.isdigit():
            task_id = int(key)
        else:
            if key not in _NAME_TO_ID:
                raise ValueError(f"unknown trajectory '{task}'")
            task_id = _NAME_TO_ID[key]
    else:
        task_id = int(task)
    if task_id not in TRAJECTORY_NAMES:
        raise ValueError(f"task_id must be in {sorted(TRAJECTORY_NAMES)}")
    return task_id


def trajectory_factory(
    task: int | str,
    *,
    dt: float = 0.05,
    params: QuadrotorParams | None = None,
    xy_offset: Iterable[float] = (1.0, 1.0),
    overrides: Dict[str, float | Iterable[float]] | None = None,
) -> BaseTrajectory:
    """Create one of the six MATLAB-aligned reference trajectories."""

    task_id = normalize_task_id(task)
    params = params if params is not None else default_quadrotor_params()
    offset = np.asarray(tuple(xy_offset), dtype=float).reshape(-1)
    if offset.size == 0:
        offset = np.array([1.0, 1.0])
    if offset.size == 1:
        offset = np.array([offset[0], 0.0])
    xy = offset[:2]

    defaults: Dict[int, Dict[str, object]] = {
        1: {"target_pos": np.array([xy[0], xy[1], 1.0])},
        2: {
            "s0": np.zeros(3),
            "v0": np.zeros(3),
            "sd": np.array([xy[0], xy[1], 1.0]),
            "vd": np.zeros(3),
            "duration": 8.0,
        },
        3: {"radius": 1.0, "omega": 0.9, "height": 1.0},
        4: {"radius": 1.0, "omega": 1.5, "helix_scl": 10.0},
        5: {"radius": 1.0, "omega": 0.9, "helix_scl": 10.0},
        6: {"omega": 0.9, "scale": 0.3},
    }
    kwargs = defaults[task_id].copy()
    if overrides:
        kwargs.update(overrides)
    kwargs.update({"dt": dt, "params": params})

    constructors = {
        1: HoverTrajectory,
        2: LineTrajectory,
        3: LemniscateTrajectory,
        4: HelixYZTrajectory,
        5: HelixXYTrajectory,
        6: TorusKnotTrajectory,
    }
    return constructors[task_id](**kwargs)
