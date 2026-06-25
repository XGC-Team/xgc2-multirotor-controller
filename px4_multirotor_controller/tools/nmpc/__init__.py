"""Python acados/HPIPM quadrotor NMPC package."""

from .controller import (
    AcadosNMPCController,
    AcadosBackendUnavailable,
    Bounds,
    ControllerInfo,
    CostWeights,
    MPCConfig,
    optional_backend_status,
)
from .dynamics import QuadrotorParams, default_quadrotor_params, hover_state, quadrotor_dynamics
from .math_utils import (
    pack_state,
    project_rotation_matrix,
    project_state_rotation,
    quat_to_rotation,
    rotation_to_quat,
    so3_log_error,
    to_px4_bodyrate_thrust,
    unpack_state,
)
from .references import (
    TRAJECTORY_NAMES,
    BaseTrajectory,
    HelixXYTrajectory,
    HelixYZTrajectory,
    HoverTrajectory,
    LemniscateTrajectory,
    LineTrajectory,
    TorusKnotTrajectory,
    trajectory_factory,
)


def __getattr__(name: str):
    if name in {"SimulationConfig", "run_multi_trajectory_sim"}:
        from .runner import SimulationConfig, run_multi_trajectory_sim

        return {
            "SimulationConfig": SimulationConfig,
            "run_multi_trajectory_sim": run_multi_trajectory_sim,
        }[name]
    raise AttributeError(name)

__all__ = [
    "AcadosBackendUnavailable",
    "AcadosNMPCController",
    "BaseTrajectory",
    "Bounds",
    "ControllerInfo",
    "CostWeights",
    "HelixXYTrajectory",
    "HelixYZTrajectory",
    "HoverTrajectory",
    "LemniscateTrajectory",
    "LineTrajectory",
    "MPCConfig",
    "QuadrotorParams",
    "SimulationConfig",
    "TRAJECTORY_NAMES",
    "TorusKnotTrajectory",
    "default_quadrotor_params",
    "hover_state",
    "optional_backend_status",
    "pack_state",
    "project_rotation_matrix",
    "project_state_rotation",
    "quat_to_rotation",
    "quadrotor_dynamics",
    "rotation_to_quat",
    "run_multi_trajectory_sim",
    "so3_log_error",
    "to_px4_bodyrate_thrust",
    "trajectory_factory",
    "unpack_state",
]
