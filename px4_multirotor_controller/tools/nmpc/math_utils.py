"""Numerical helpers for the px4_multirotor_controller Python quadrotor NMPC OCP.

The Python controller now uses a quaternion state rather than the MATLAB
draft's 9-element rotation-matrix state:

    x = [pos(3); vel(3); quat_wxyz(4); omega(3)]

The public packing helpers still accept a 3x3 rotation matrix, so the reference
generators and runner can stay close to the MATLAB notation while the stored
state is the 13D quaternion form used by the Python implementation.
"""

from __future__ import annotations

from typing import Callable, Tuple

import numpy as np


STATE_SIZE = 13
CONTROL_SIZE = 4

POS = slice(0, 3)
VEL = slice(3, 6)
QUAT = slice(6, 10)
OMEGA = slice(10, 13)


def as_vector(value: np.ndarray, size: int, name: str) -> np.ndarray:
    """Return *value* as a finite 1-D float vector with the requested size."""

    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != size:
        raise ValueError(f"{name} must have {size} elements, got {arr.size}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains non-finite values")
    return arr


def skew(omega: np.ndarray) -> np.ndarray:
    """Return the skew-symmetric matrix for a 3-vector."""

    wx, wy, wz = as_vector(omega, 3, "omega")
    return np.array(
        [
            [0.0, -wz, wy],
            [wz, 0.0, -wx],
            [-wy, wx, 0.0],
        ],
        dtype=float,
    )


def vee(mat: np.ndarray) -> np.ndarray:
    """Inverse of :func:`skew` for a 3x3 skew-symmetric matrix."""

    mat = np.asarray(mat, dtype=float).reshape(3, 3)
    return np.array([mat[2, 1], mat[0, 2], mat[1, 0]], dtype=float)


def normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    """Normalize a vector, returning *fallback* if the norm is degenerate."""

    vec = np.asarray(vec, dtype=float).reshape(-1)
    norm = np.linalg.norm(vec)
    if norm < 1e-9:
        return np.asarray(fallback, dtype=float).reshape(vec.shape)
    return vec / norm


def quat_normalize(quat: np.ndarray) -> np.ndarray:
    """Return a unit Hamilton quaternion in ``[w, x, y, z]`` order."""

    quat = as_vector(quat, 4, "quat")
    normalized = normalize(quat, np.array([1.0, 0.0, 0.0, 0.0]))
    if normalized[0] < 0.0:
        normalized = -normalized
    return normalized


def quat_conjugate(quat: np.ndarray) -> np.ndarray:
    """Return the conjugate of a Hamilton quaternion."""

    quat = quat_normalize(quat)
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=float)


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Hamilton product of two ``[w, x, y, z]`` quaternions."""

    w1, x1, y1, z1 = as_vector(left, 4, "left_quat")
    w2, x2, y2, z2 = as_vector(right, 4, "right_quat")
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_derivative(quat: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    """Quaternion kinematics for ``R_dot = R skew(omega_body)``."""

    quat = quat_normalize(quat)
    omega_quat = np.concatenate(([0.0], as_vector(omega_body, 3, "omega_body")))
    return 0.5 * quat_multiply(quat, omega_quat)


def quat_to_rotation(quat: np.ndarray) -> np.ndarray:
    """Convert a unit Hamilton quaternion to a body-to-world rotation matrix."""

    w, x, y, z = quat_normalize(quat)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_to_quat(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit Hamilton quaternion."""

    rotation = project_rotation_matrix(rotation)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s_val = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                0.25 * s_val,
                (rotation[2, 1] - rotation[1, 2]) / s_val,
                (rotation[0, 2] - rotation[2, 0]) / s_val,
                (rotation[1, 0] - rotation[0, 1]) / s_val,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            s_val = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quat = np.array(
                [
                    (rotation[2, 1] - rotation[1, 2]) / s_val,
                    0.25 * s_val,
                    (rotation[0, 1] + rotation[1, 0]) / s_val,
                    (rotation[0, 2] + rotation[2, 0]) / s_val,
                ]
            )
        elif axis == 1:
            s_val = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quat = np.array(
                [
                    (rotation[0, 2] - rotation[2, 0]) / s_val,
                    (rotation[0, 1] + rotation[1, 0]) / s_val,
                    0.25 * s_val,
                    (rotation[1, 2] + rotation[2, 1]) / s_val,
                ]
            )
        else:
            s_val = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quat = np.array(
                [
                    (rotation[1, 0] - rotation[0, 1]) / s_val,
                    (rotation[0, 2] + rotation[2, 0]) / s_val,
                    (rotation[1, 2] + rotation[2, 1]) / s_val,
                    0.25 * s_val,
                ]
            )
    return quat_normalize(quat)


def project_rotation_matrix(rotation: np.ndarray) -> np.ndarray:
    """Project a numeric 3x3 matrix onto SO(3) with an SVD polar projection."""

    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    u_mat, _, vt_mat = np.linalg.svd(rotation)
    projected = u_mat @ vt_mat
    if np.linalg.det(projected) < 0.0:
        u_mat[:, -1] *= -1.0
        projected = u_mat @ vt_mat
    return projected


def pack_state(
    position: np.ndarray,
    velocity: np.ndarray,
    attitude: np.ndarray,
    omega: np.ndarray,
) -> np.ndarray:
    """Pack position, velocity, attitude, and body rate into the 13D state.

    ``attitude`` may be either a 3x3 rotation matrix or a 4-element
    ``[w, x, y, z]`` quaternion.
    """

    position = as_vector(position, 3, "position")
    velocity = as_vector(velocity, 3, "velocity")
    attitude_arr = np.asarray(attitude, dtype=float)
    if attitude_arr.size == 4:
        quat = quat_normalize(attitude_arr)
    elif attitude_arr.size == 9:
        quat = rotation_to_quat(attitude_arr.reshape(3, 3))
    else:
        raise ValueError("attitude must be a 4D quaternion or 3x3 rotation matrix")
    omega = as_vector(omega, 3, "omega")
    return np.concatenate([position, velocity, quat, omega])


def unpack_state_quat(state: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Unpack the 13D state into position, velocity, quaternion, and body rate."""

    state = as_vector(state, STATE_SIZE, "state")
    position = state[POS].copy()
    velocity = state[VEL].copy()
    quat = quat_normalize(state[QUAT])
    omega = state[OMEGA].copy()
    return position, velocity, quat, omega


def unpack_state(state: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Unpack the 13D state into position, velocity, rotation, and body rate."""

    position, velocity, quat, omega = unpack_state_quat(state)
    return position, velocity, quat_to_rotation(quat), omega


def project_state_rotation(state: np.ndarray) -> np.ndarray:
    """Normalize the quaternion block of the state.

    The name is kept for compatibility with the earlier rotation-matrix draft.
    """

    position, velocity, quat, omega = unpack_state_quat(state)
    return pack_state(position, velocity, quat, omega)


def rotation_from_body_z(body_z: np.ndarray, yaw: float = 0.0) -> np.ndarray:
    """Build a rotation whose third body axis equals ``body_z``."""

    z_b = normalize(body_z, np.array([0.0, 0.0, 1.0]))
    x_c = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=float)

    y_b = np.cross(z_b, x_c)
    if np.linalg.norm(y_b) < 1e-8:
        x_c = np.array([0.0, 1.0, 0.0], dtype=float)
        y_b = np.cross(z_b, x_c)
    y_b = normalize(y_b, np.array([0.0, 1.0, 0.0]))
    x_b = normalize(np.cross(y_b, z_b), np.array([1.0, 0.0, 0.0]))

    return project_rotation_matrix(np.column_stack((x_b, y_b, z_b)))


def so3_log(rotation: np.ndarray) -> np.ndarray:
    """Return the Lie algebra vector ``log(rotation)^vee``."""

    rotation = project_rotation_matrix(rotation)
    cos_theta = 0.5 * (np.trace(rotation) - 1.0)
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-8:
        return 0.5 * vee(rotation - rotation.T)
    return theta / (2.0 * np.sin(theta)) * vee(rotation - rotation.T)


def so3_log_error(rotation: np.ndarray, rotation_ref: np.ndarray) -> np.ndarray:
    """Return ``log(rotation_ref.T @ rotation)^vee``."""

    return so3_log(project_rotation_matrix(rotation_ref).T @ project_rotation_matrix(rotation))


def rk4_step(deriv: Callable[[np.ndarray], np.ndarray], state: np.ndarray, dt: float) -> np.ndarray:
    """One fixed-step fourth-order Runge-Kutta update for autonomous systems."""

    state = np.asarray(state, dtype=float).reshape(-1)
    k1 = deriv(state)
    k2 = deriv(state + 0.5 * dt * k1)
    k3 = deriv(state + 0.5 * dt * k2)
    k4 = deriv(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def rk4_step_time(
    deriv: Callable[[float, np.ndarray], np.ndarray],
    time: float,
    state: np.ndarray,
    dt: float,
) -> np.ndarray:
    """One fixed-step RK4 update for time-varying dynamics."""

    state = np.asarray(state, dtype=float).reshape(-1)
    k1 = deriv(time, state)
    k2 = deriv(time + 0.5 * dt, state + 0.5 * dt * k1)
    k3 = deriv(time + 0.5 * dt, state + 0.5 * dt * k2)
    k4 = deriv(time + dt, state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def to_px4_bodyrate_thrust(
    predicted_state: np.ndarray,
    u_mpc: np.ndarray,
    *,
    hover_specific_thrust: float = 9.8066,
    hover_thrust_norm: float = 0.5,
) -> tuple[float, np.ndarray]:
    """Map predicted NMPC state and ``u0`` to PX4 body-rate and thrust.

    The OCP input contains specific thrust and angular acceleration, while PX4's
    attitude setpoint interface expects normalized thrust and body-rate
    setpoints. Use the optimizer-predicted next-state angular velocity for the
    body-rate command so the bridge matches the solver discretization and the
    C++ runtime backend.
    """

    predicted_state = as_vector(predicted_state, STATE_SIZE, "predicted_state")
    u_mpc = as_vector(u_mpc, CONTROL_SIZE, "u_mpc")
    if hover_specific_thrust <= 1e-9:
        raise ValueError("hover_specific_thrust must be positive")
    if not 0.0 < hover_thrust_norm <= 1.0:
        raise ValueError("hover_thrust_norm must be in (0, 1]")

    body_rate_cmd = predicted_state[OMEGA].copy()
    thrust_norm = hover_thrust_norm * u_mpc[0] / hover_specific_thrust
    return float(np.clip(thrust_norm, 0.0, 1.0)), body_rate_cmd
