"""High-level quadrotor dynamics for the px4_multirotor_controller UAV NMPC OCP."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .math_utils import (
    OMEGA,
    QUAT,
    STATE_SIZE,
    VEL,
    as_vector,
    pack_state,
    quat_derivative,
    quat_to_rotation,
    unpack_state_quat,
)


@dataclass(frozen=True)
class QuadrotorParams:
    """Physical parameters needed by the high-level NMPC model."""

    mass: float = 1.5
    g: float = 9.8066
    e3: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    disturbance_accel_I: np.ndarray = field(default_factory=lambda: np.zeros(3))
    disturbance_alpha_B: np.ndarray = field(default_factory=lambda: np.zeros(3))
    disturbance_accel_sin_amp_I: np.ndarray = field(default_factory=lambda: np.zeros(3))
    disturbance_accel_sin_freq_hz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    disturbance_accel_sin_phase: np.ndarray = field(default_factory=lambda: np.zeros(3))
    disturbance_alpha_sin_amp_B: np.ndarray = field(default_factory=lambda: np.zeros(3))
    disturbance_alpha_sin_freq_hz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    disturbance_alpha_sin_phase: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        object.__setattr__(self, "e3", as_vector(self.e3, 3, "e3"))
        for name in (
            "disturbance_accel_I",
            "disturbance_alpha_B",
            "disturbance_accel_sin_amp_I",
            "disturbance_accel_sin_freq_hz",
            "disturbance_accel_sin_phase",
            "disturbance_alpha_sin_amp_B",
            "disturbance_alpha_sin_freq_hz",
            "disturbance_alpha_sin_phase",
        ):
            object.__setattr__(self, name, as_vector(getattr(self, name), 3, name))


def default_quadrotor_params() -> QuadrotorParams:
    """Return the MATLAB draft's default mass, gravity, and body-axis convention."""

    return QuadrotorParams()


def evaluate_disturbance(params: QuadrotorParams, time: float) -> tuple[np.ndarray, np.ndarray]:
    """Evaluate optional additive inertial acceleration and body angular acceleration."""

    accel_I = params.disturbance_accel_I.copy()
    alpha_B = params.disturbance_alpha_B.copy()

    if np.any(params.disturbance_accel_sin_amp_I) and np.any(
        params.disturbance_accel_sin_freq_hz
    ):
        accel_I += params.disturbance_accel_sin_amp_I * np.sin(
            2.0 * np.pi * params.disturbance_accel_sin_freq_hz * time
            + params.disturbance_accel_sin_phase
        )

    if np.any(params.disturbance_alpha_sin_amp_B) and np.any(
        params.disturbance_alpha_sin_freq_hz
    ):
        alpha_B += params.disturbance_alpha_sin_amp_B * np.sin(
            2.0 * np.pi * params.disturbance_alpha_sin_freq_hz * time
            + params.disturbance_alpha_sin_phase
        )

    return accel_I, alpha_B


def quadrotor_dynamics(
    time: float,
    state: np.ndarray,
    control: np.ndarray,
    params: QuadrotorParams | None = None,
) -> np.ndarray:
    """Continuous dynamics for ``u = [T/m; angular_acceleration(3)]``.

    The model intentionally bypasses motor allocation and torque dynamics. It
    matches the high-level MATLAB dynamics:

    ``p_dot = v``
    ``v_dot = -g e3 + (T/m) R e3``
    ``q_dot = 0.5 * q * [0, omega]``
    ``omega_dot = angular_acceleration``
    """

    if params is None:
        params = default_quadrotor_params()

    state = as_vector(state, STATE_SIZE, "state")
    control = as_vector(control, 4, "control")
    _, velocity, quat, omega = unpack_state_quat(state)

    thrust_specific = control[0]
    angular_acceleration = control[1:4]
    accel_dist, alpha_dist = evaluate_disturbance(params, time)

    position_dot = velocity
    rotation = quat_to_rotation(quat)
    velocity_dot = -params.g * params.e3 + thrust_specific * (rotation @ params.e3)
    velocity_dot += accel_dist
    quat_dot = quat_derivative(quat, omega)
    omega_dot = angular_acceleration + alpha_dist

    state_dot = np.zeros(STATE_SIZE, dtype=float)
    state_dot[0:3] = position_dot
    state_dot[VEL] = velocity_dot
    state_dot[QUAT] = quat_dot
    state_dot[OMEGA] = omega_dot
    return state_dot


def hover_state(position: np.ndarray | None = None) -> np.ndarray:
    """Return a static hover state with identity attitude."""

    if position is None:
        position = np.zeros(3)
    return pack_state(position, np.zeros(3), np.eye(3), np.zeros(3))
