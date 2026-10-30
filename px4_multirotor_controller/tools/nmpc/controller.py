"""Python acados OCP source for the px4_multirotor_controller UAV NMPC backend."""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
from time import perf_counter

import numpy as np

from .dynamics import QuadrotorParams, default_quadrotor_params
from .math_utils import (
    OMEGA,
    POS,
    QUAT,
    STATE_SIZE,
    THRUST_ACT,
    VEL,
    as_vector,
    project_state_rotation,
)
from .references import BaseTrajectory


@dataclass(frozen=True)
class Bounds:
    """Default state and input bounds for the UAV NMPC OCP."""

    u_min: np.ndarray = field(default_factory=lambda: np.array([5.0, -3.4906585, -3.4906585, -0.8726646]))
    u_max: np.ndarray = field(default_factory=lambda: np.array([20.373, 3.4906585, 3.4906585, 0.8726646]))
    p_min: np.ndarray = field(default_factory=lambda: np.array([-100.0, -100.0, 0.2]))
    p_max: np.ndarray = field(default_factory=lambda: np.array([100.0, 100.0, 6.0]))
    v_min: np.ndarray = field(default_factory=lambda: np.array([-10.0, -10.0, -10.0]))
    v_max: np.ndarray = field(default_factory=lambda: np.array([10.0, 10.0, 10.0]))
    omega_min: np.ndarray = field(default_factory=lambda: np.array([-6.0, -6.0, -6.0]))
    omega_max: np.ndarray = field(default_factory=lambda: np.array([6.0, 6.0, 6.0]))
    alpha_min: np.ndarray = field(default_factory=lambda: np.array([-15.0, -15.0, -2.0]))
    alpha_max: np.ndarray = field(default_factory=lambda: np.array([15.0, 15.0, 2.0]))
    tilt_max_deg: float = 45.0

    def input_pairs(self, horizon_steps: int) -> list[tuple[float, float]]:
        lower = as_vector(self.u_min, 4, "u_min")
        upper = as_vector(self.u_max, 4, "u_max")
        return list(zip(np.tile(lower, horizon_steps), np.tile(upper, horizon_steps)))

    def state_lbx_ubx(self) -> tuple[np.ndarray, np.ndarray]:
        lbx = -1e3 * np.ones(STATE_SIZE)
        ubx = 1e3 * np.ones(STATE_SIZE)
        lbx[POS] = self.p_min
        ubx[POS] = self.p_max
        lbx[VEL] = self.v_min
        ubx[VEL] = self.v_max
        lbx[QUAT] = -1.0
        ubx[QUAT] = 1.0
        lbx[OMEGA] = self.omega_min
        ubx[OMEGA] = self.omega_max
        return lbx, ubx


@dataclass(frozen=True)
class CostWeights:
    """Diagonal cost weights used by the acados NMPC backend."""

    position: np.ndarray = field(default_factory=lambda: 120.0 * np.ones(3))
    velocity: np.ndarray = field(default_factory=lambda: 30.0 * np.ones(3))
    thrust_direction: np.ndarray = field(default_factory=lambda: 40.0 * np.ones(3))
    yaw: float = 1.0
    omega: np.ndarray = field(default_factory=lambda: 0.5 * np.ones(3))
    thrust_actual: float = 2.0
    thrust_command_delta: float = 0.35
    body_rate_command_delta: np.ndarray = field(default_factory=lambda: np.array([8.0, 8.0, 16.0]))
    # Simulation starting point only. These weights make the cost at each
    # configured alpha limit comparable to the UGV baseline W_alpha * 3^2 = 9.
    angular_acceleration: np.ndarray = field(
        default_factory=lambda: np.array([0.04, 0.04, 2.25])
    )
    terminal_position: np.ndarray = field(default_factory=lambda: 300.0 * np.ones(3))
    terminal_velocity: np.ndarray = field(default_factory=lambda: 80.0 * np.ones(3))
    terminal_thrust_direction: np.ndarray = field(default_factory=lambda: 80.0 * np.ones(3))
    terminal_yaw: float = 2.0
    terminal_omega: np.ndarray = field(default_factory=lambda: 1.0 * np.ones(3))
    terminal_thrust_actual: float = 4.0
    control: np.ndarray = field(default_factory=lambda: np.array([0.08, 0.80, 0.80, 2.00]))
    unit_quat_penalty: float = 10.0
    input_slack: np.ndarray = field(default_factory=lambda: np.array([1.0e8, 1.0e6, 1.0e6, 1.0e6]))
    state_slack_position: np.ndarray = field(default_factory=lambda: 1.0e4 * np.ones(3))
    state_slack_velocity: np.ndarray = field(default_factory=lambda: 2.0e3 * np.ones(3))
    state_slack_omega: np.ndarray = field(default_factory=lambda: 1.0e3 * np.ones(3))
    tilt_slack: float = 5.0e4
    terminal_state_slack_position: np.ndarray = field(default_factory=lambda: 2.0e4 * np.ones(3))
    terminal_state_slack_velocity: np.ndarray = field(default_factory=lambda: 4.0e3 * np.ones(3))
    terminal_state_slack_omega: np.ndarray = field(default_factory=lambda: 2.0e3 * np.ones(3))
    terminal_tilt_slack: float = 1.0e5
    slack_linear_ratio: float = 1.0e-2


@dataclass(frozen=True)
class MPCConfig:
    """Configuration for the acados NMPC controller."""

    horizon: float = 1.0
    steps: int = 10
    backend: str = "acados"
    max_iter: int = 20
    ftol: float = 1e-4
    warm_start: bool = True
    control_interval: float | None = None
    model_name: str = "uav_nmpc"
    code_export_directory: str | None = None
    json_file: str | None = None

    @property
    def dt(self) -> float:
        if self.steps <= 0:
            raise ValueError("steps must be positive")
        if self.horizon <= 0.0:
            raise ValueError("horizon must be positive")
        return self.horizon / self.steps


@dataclass
class ControllerInfo:
    """Metadata for one controller update."""

    status: int
    accepted: bool
    optimizer_success: bool
    message: str
    objective: float
    update_ms: float
    solve_ms: float
    iterations: int
    qp_iter: int
    res_stat: float
    res_eq: float
    res_ineq: float
    res_comp: float
    time_qp_ms: float
    time_lin_ms: float
    finite_solution: bool


def thrust_direction_error(rotation: np.ndarray, reference_rotation: np.ndarray) -> np.ndarray:
    """Return the yaw-invariant thrust-direction residual ``z_ref x z``."""

    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    reference_rotation = np.asarray(reference_rotation, dtype=float).reshape(3, 3)
    return np.cross(reference_rotation[:, 2], rotation[:, 2])


def projected_heading_error(rotation: np.ndarray, reference_rotation: np.ndarray) -> float:
    """Return signed heading error projected onto the reference thrust plane."""

    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    reference_rotation = np.asarray(reference_rotation, dtype=float).reshape(3, 3)
    z_ref = reference_rotation[:, 2]
    x_axis = _normalize_projected_heading(rotation[:, 0], z_ref)
    x_ref = _normalize_projected_heading(reference_rotation[:, 0], z_ref)
    return float(np.dot(z_ref, np.cross(x_ref, x_axis)))


def _normalize_projected_heading(x_axis: np.ndarray, z_ref: np.ndarray) -> np.ndarray:
    projected = x_axis - z_ref * float(np.dot(z_ref, x_axis))
    norm = float(np.linalg.norm(projected))
    if norm < 1e-9:
        return np.zeros(3)
    return projected / norm


# The single installed SDK. Discovery, alternate roots, sys.path/library-path
# rewriting and ctypes preloading are intentionally absent: the released
# package setup selects the code-generation interpreter and this module only
# validates the canonical installation read-only.
ACADOS_ROOT = Path("/opt/xgc2/acados")
ACADOS_REF = "v0.6.0"
ACADOS_SHA = "503364817c872d474ab5bed219c26760ac267769"
ACADOS_PACKAGE_BASE = "0.6.0-1"


def _installed_acados_status() -> dict[str, bool | str]:
    """Validate the installed xgc2-acados SDK without mutating the process."""

    root = ACADOS_ROOT
    status: dict[str, bool | str] = {
        "acados_root": str(root),
        "acados_root_exists": root.is_dir(),
        "acados_template_source_exists": (
            root / "interfaces" / "acados_template" / "acados_template"
        ).is_dir(),
        "casadi": False,
        "acados_template": False,
    }
    conflict = os.environ.get("ACADOS_SOURCE_DIR")
    if conflict:
        try:
            resolved = Path(conflict).expanduser().resolve()
        except OSError:
            resolved = None
        if resolved != root.resolve():
            status["error"] = (
                f"ACADOS_SOURCE_DIR={conflict} conflicts with the installed SDK at {root}"
            )
            return status
    if not root.is_dir():
        status["error"] = f"xgc2-acados is not installed at {root}; install the released package"
        return status
    try:
        lock = dict(
            line.split("=", 1)
            for line in (root / "acados.lock").read_text().splitlines()
            if "=" in line
        )
    except OSError as error:
        status["error"] = f"cannot read the installed SDK lock {root / 'acados.lock'}: {error}"
        return status
    lock = {key.strip(): value.strip() for key, value in lock.items()}
    if (
        lock.get("ACADOS_REF") != ACADOS_REF
        or lock.get("ACADOS_SHA") != ACADOS_SHA
        or lock.get("XGC2_PACKAGE_VERSION") != ACADOS_PACKAGE_BASE
    ):
        status["error"] = (
            f"installed SDK lock is not the selected acados {ACADOS_REF} "
            f"({ACADOS_SHA}, package base {ACADOS_PACKAGE_BASE})"
        )
        return status
    try:
        import acados_template
        import casadi
    except Exception as error:
        status["error"] = (
            f"casadi/acados_template is not importable from the SDK environment: {error}"
        )
        return status
    template_path = Path(acados_template.__file__).resolve()
    casadi_path = Path(casadi.__file__).resolve()
    if root.resolve() not in template_path.parents:
        status["error"] = (
            f"acados_template is loaded from {template_path}, outside the installed SDK {root}"
        )
        return status
    if (root / "python").resolve() not in casadi_path.parents:
        status["error"] = (
            f"casadi is loaded from {casadi_path}, outside the installed SDK {root}/python"
        )
        return status
    status["casadi"] = True
    status["acados_template"] = True
    return status


def _format_float_token(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def optional_backend_status() -> dict[str, bool | str]:
    """Report the installed SDK/backend availability without mutating state."""

    return _installed_acados_status()


class AcadosBackendUnavailable(RuntimeError):
    """Raised when the optional acados backend is requested but unavailable."""


class AcadosNMPCController:
    """Generated acados/HPIPM NMPC backend.

    The backend is only usable when `casadi` and `acados_template` are
    importable and the acados shared libraries are available to the process.
    It never switches to an alternate optimizer backend.
    """

    def __init__(
        self,
        reference: BaseTrajectory,
        params: QuadrotorParams | None = None,
        config: MPCConfig | None = None,
        bounds: Bounds | None = None,
        weights: CostWeights | None = None,
        build_solver: bool = True,
    ) -> None:
        status = optional_backend_status()
        if not status["casadi"] or not status["acados_template"]:
            reason = status.get("error", "the installed xgc2-acados SDK is not available")
            raise AcadosBackendUnavailable(
                f"backend='acados' requires the installed xgc2-acados SDK: {reason}"
            )
        self.reference = reference
        self.params = params if params is not None else default_quadrotor_params()
        if config is None:
            default_config = MPCConfig(backend="acados")
            self.config = MPCConfig(backend="acados", control_interval=default_config.dt)
        else:
            self.config = config
        if self.config.backend != "acados":
            raise ValueError("AcadosNMPCController only supports backend='acados'")
        if self.config.warm_start and self.config.control_interval is None:
            raise ValueError("control_interval must be set when warm_start is enabled")
        self.bounds = bounds if bounds is not None else Bounds()
        self.weights = weights if weights is not None else CostWeights()
        self.last_u: np.ndarray | None = None
        self.u_guess: np.ndarray | None = None
        self.x_guess: np.ndarray | None = None
        self.solve_history: list[ControllerInfo] = []
        self.ocp_solver = self._build_solver() if build_solver else None

    def update(self, time: float, state: np.ndarray) -> tuple[np.ndarray, ControllerInfo]:
        """Solve one acados OCP update."""

        if self.ocp_solver is None:
            raise RuntimeError("acados solver was not built for this controller instance")

        update_start = perf_counter()
        state = project_state_rotation(state)
        self._set_x0_constraint(state)

        _, current_uref = self.reference.get_reference(time)
        alpha_sqrt_weight = np.sqrt(
            as_vector(self.weights.angular_acceleration, 3, "angular_acceleration")
        )
        if self.last_u is not None:
            last_commanded_thrust = float(self.last_u[0])
            last_commanded_body_rate = self.last_u[1:4].copy()
        else:
            last_commanded_thrust = float(current_uref[0])
            last_commanded_body_rate = current_uref[1:4].copy()

        for stage in range(self.config.steps + 1):
            xref, uref = self.reference.get_reference(time + stage * self.config.dt)
            parameter = np.concatenate(
                (
                    xref,
                    uref,
                    np.array([last_commanded_thrust]),
                    last_commanded_body_rate,
                    alpha_sqrt_weight,
                )
            )
            self.ocp_solver.set(stage, "p", parameter)
            if self.x_guess is not None:
                self.ocp_solver.set(stage, "x", self.x_guess[min(stage, self.x_guess.shape[0] - 1)])
            else:
                self.ocp_solver.set(stage, "x", xref)
        if self.u_guess is not None:
            for stage in range(self.config.steps):
                self.ocp_solver.set(stage, "u", self.u_guess[min(stage, self.u_guess.shape[0] - 1)])
        else:
            for stage in range(self.config.steps):
                _, uref = self.reference.get_reference(time + stage * self.config.dt)
                self.ocp_solver.set(stage, "u", uref)

        solve_start = perf_counter()
        status = int(self.ocp_solver.solve())
        solve_ms = (perf_counter() - solve_start) * 1e3

        controls = np.vstack([self.ocp_solver.get(stage, "u") for stage in range(self.config.steps)])
        states = np.vstack([self.ocp_solver.get(stage, "x") for stage in range(self.config.steps + 1)])
        objective = self._safe_solver_cost()
        iterations = self._safe_solver_stat_int("sqp_iter")
        qp_iter = self._safe_solver_stat_int("qp_iter")
        res_stat, res_eq, res_ineq, res_comp = self._safe_solver_residuals()
        time_qp_ms = 1e3 * self._safe_solver_stat_float("time_qp")
        time_lin_ms = 1e3 * self._safe_solver_stat_float("time_lin")
        finite_solution = bool(np.all(np.isfinite(controls)) and np.all(np.isfinite(states)))
        optimizer_success = status == 0
        accepted = optimizer_success and finite_solution
        if accepted:
            self.last_u = controls[0].copy()
            if (
                self.config.control_interval is not None
                and self.config.control_interval >= 0.5 * self.config.dt
            ):
                self.u_guess = np.vstack((controls[1:, :], controls[-1:, :]))
                self.x_guess = np.vstack((states[1:, :], states[-1:, :]))
            else:
                self.u_guess = controls.copy()
                self.x_guess = states.copy()
        else:
            _, ref_u = self.reference.get_reference(time)
            self.last_u = self.last_u if self.last_u is not None else ref_u
        update_ms = (perf_counter() - update_start) * 1e3

        info = ControllerInfo(
            status=status,
            accepted=accepted,
            optimizer_success=optimizer_success,
            message=f"acados status {status}; accepted={accepted}; finite_solution={finite_solution}",
            objective=objective,
            update_ms=update_ms,
            solve_ms=solve_ms,
            iterations=iterations,
            qp_iter=qp_iter,
            res_stat=res_stat,
            res_eq=res_eq,
            res_ineq=res_ineq,
            res_comp=res_comp,
            time_qp_ms=time_qp_ms,
            time_lin_ms=time_lin_ms,
            finite_solution=finite_solution,
        )
        self.solve_history.append(info)
        return self.last_u.copy(), info

    def _safe_solver_cost(self) -> float:
        try:
            return float(self.ocp_solver.get_cost())
        except Exception:
            return float("nan")

    def _safe_solver_stat_float(self, field: str) -> float:
        try:
            value = np.asarray(self.ocp_solver.get_stats(field), dtype=float).reshape(-1)
        except Exception:
            return float("nan")
        finite = value[np.isfinite(value)]
        return float(finite[-1]) if finite.size else float("nan")

    def _safe_solver_stat_int(self, field: str) -> int:
        value = self._safe_solver_stat_float(field)
        if not np.isfinite(value):
            return 0
        return int(round(value))

    def _safe_solver_residuals(self) -> tuple[float, float, float, float]:
        try:
            residuals = np.asarray(self.ocp_solver.get_stats("residuals"), dtype=float).reshape(-1)
        except Exception:
            return (float("nan"), float("nan"), float("nan"), float("nan"))
        out = np.full(4, np.nan)
        count = min(4, residuals.size)
        out[:count] = residuals[:count]
        return tuple(float(value) for value in out)

    def _set_x0_constraint(self, state: np.ndarray) -> None:
        """Update the fixed initial-state constraint using the acados constraint API."""

        try:
            self.ocp_solver.constraints_set(0, "lbx", state)
            self.ocp_solver.constraints_set(0, "ubx", state)
        except AttributeError:
            self.ocp_solver.set(0, "lbx", state)
            self.ocp_solver.set(0, "ubx", state)

    def _build_solver(self):
        ocp, AcadosOcpSolver = self._build_ocp()
        try:
            return AcadosOcpSolver(ocp, verbose=False)
        except TypeError:
            return AcadosOcpSolver(ocp)

    def _build_ocp(self):
        try:
            import casadi as ca
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except ImportError as exc:
            raise AcadosBackendUnavailable(str(exc)) from exc

        nx = STATE_SIZE
        nu = 4
        np_param = nx + nu + 7
        x = ca.SX.sym("x", nx)
        xdot = ca.SX.sym("xdot", nx)
        u = ca.SX.sym("u", nu)
        p = ca.SX.sym("p", np_param)

        pos = x[0:3]
        vel = x[3:6]
        quat = x[6:10]
        omega = x[10:13]
        thrust_command = u[0]
        thrust_actual = x[THRUST_ACT]
        body_rate_command = u[1:4]
        quat_dot = self._casadi_quat_derivative(ca, quat, omega)
        rotation = self._casadi_quat_to_rotation(ca, quat)
        e3 = ca.vertcat(0.0, 0.0, 1.0)
        thrust_actual_dot = (
            thrust_command - thrust_actual
        ) / self.params.thrust_time_constant
        f_expl = ca.vertcat(
            vel,
            -self.params.g * e3 + thrust_actual * (rotation @ e3),
            quat_dot,
            (body_rate_command - omega) / self.params.body_rate_time_constant,
            thrust_actual_dot,
        )

        model = AcadosModel()
        model.name = self.config.model_name
        model.x = x
        model.xdot = xdot
        model.u = u
        model.p = p
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl
        cost_y_expr, cost_weights = self._casadi_residual(
            ca, x, u, p, terminal=False, initial=False
        )
        cost_y_expr_0, cost_weights_0 = self._casadi_residual(
            ca, x, u, p, terminal=False, initial=True
        )
        cost_y_expr_e, cost_weights_e = self._casadi_residual(ca, x, u, p, terminal=True)
        model.cost_y_expr = cost_y_expr
        model.cost_y_expr_0 = cost_y_expr_0
        model.cost_y_expr_e = cost_y_expr_e
        tilt_expr = ca.vertcat(rotation[2, 2])
        equivalent_alpha = (body_rate_command - omega) / self.params.body_rate_time_constant
        path_constraint_expr = ca.vertcat(tilt_expr, equivalent_alpha)
        model.con_h_expr_0 = path_constraint_expr
        model.con_h_expr = path_constraint_expr
        model.con_h_expr_e = tilt_expr

        ocp = AcadosOcp()
        code_export_directory = (
            Path(self.config.code_export_directory)
            if self.config.code_export_directory
            else Path(__file__).resolve().parent
            / "results"
            / "acados_codegen"
            / f"{self.config.model_name}_N{self.config.steps}_Tf{_format_float_token(self.config.horizon)}"
        )
        code_export_directory.mkdir(parents=True, exist_ok=True)
        ocp.code_gen_options.code_export_directory = str(code_export_directory)
        ocp.code_gen_options.json_file = (
            self.config.json_file
            if self.config.json_file
            else str(code_export_directory / f"{self.config.model_name}_acados_ocp.json")
        )
        ocp.model = model
        ocp.name = self.config.model_name
        ocp.solver_options.N_horizon = self.config.steps
        ocp.solver_options.tf = self.config.horizon
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.diag(cost_weights)
        ocp.cost.W_0 = np.diag(cost_weights_0)
        ocp.cost.W_e = np.diag(cost_weights_e)
        ocp.cost.yref = np.zeros(cost_y_expr.shape[0])
        ocp.cost.yref_0 = np.zeros(cost_y_expr_0.shape[0])
        ocp.cost.yref_e = np.zeros(cost_y_expr_e.shape[0])

        ocp.constraints.x0 = np.zeros(nx)
        ocp.constraints.lbu = self.bounds.u_min
        ocp.constraints.ubu = self.bounds.u_max
        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.idxsbu = np.arange(nu)

        idxbx = np.array([0, 1, 2, 3, 4, 5, 10, 11, 12])
        lbx, ubx = self.bounds.state_lbx_ubx()
        ocp.constraints.idxbx = idxbx
        ocp.constraints.lbx = lbx[idxbx]
        ocp.constraints.ubx = ubx[idxbx]
        ocp.constraints.idxsbx = np.arange(idxbx.size)
        ocp.constraints.idxbx_e = idxbx
        ocp.constraints.lbx_e = lbx[idxbx]
        ocp.constraints.ubx_e = ubx[idxbx]
        ocp.constraints.idxsbx_e = np.arange(idxbx.size)
        tilt_min = float(np.cos(np.deg2rad(self.bounds.tilt_max_deg)))
        alpha_min = as_vector(self.bounds.alpha_min, 3, "alpha_min")
        alpha_max = as_vector(self.bounds.alpha_max, 3, "alpha_max")
        ocp.constraints.lh_0 = np.concatenate((np.array([tilt_min]), alpha_min))
        ocp.constraints.uh_0 = np.concatenate((np.array([1.0e3]), alpha_max))
        ocp.constraints.lh = np.concatenate((np.array([tilt_min]), alpha_min))
        ocp.constraints.uh = np.concatenate((np.array([1.0e3]), alpha_max))
        ocp.constraints.idxsh = np.array([0])
        ocp.constraints.lh_e = np.array([tilt_min])
        ocp.constraints.uh_e = np.array([1.0e3])
        ocp.constraints.idxsh_e = np.array([0])

        stage_slack = np.concatenate(
            (
                self.weights.input_slack,
                self.weights.state_slack_position,
                self.weights.state_slack_velocity,
                self.weights.state_slack_omega,
                np.array([self.weights.tilt_slack]),
            )
        )
        terminal_slack = np.concatenate(
            (
                self.weights.terminal_state_slack_position,
                self.weights.terminal_state_slack_velocity,
                self.weights.terminal_state_slack_omega,
                np.array([self.weights.terminal_tilt_slack]),
            )
        )
        initial_slack = self.weights.input_slack
        ocp.cost.Zl = stage_slack
        ocp.cost.Zu = stage_slack
        ocp.cost.zl = self.weights.slack_linear_ratio * stage_slack
        ocp.cost.zu = self.weights.slack_linear_ratio * stage_slack
        ocp.cost.Zl_0 = initial_slack
        ocp.cost.Zu_0 = initial_slack
        ocp.cost.zl_0 = self.weights.slack_linear_ratio * initial_slack
        ocp.cost.zu_0 = self.weights.slack_linear_ratio * initial_slack
        ocp.cost.Zl_e = terminal_slack
        ocp.cost.Zu_e = terminal_slack
        ocp.cost.zl_e = self.weights.slack_linear_ratio * terminal_slack
        ocp.cost.zu_e = self.weights.slack_linear_ratio * terminal_slack

        ocp.parameter_values = np.zeros(np_param)
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.qp_solver_cond_N = min(5, self.config.steps)
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.nlp_solver_max_iter = self.config.max_iter
        ocp.solver_options.nlp_solver_tol_stat = self.config.ftol
        ocp.solver_options.nlp_solver_tol_eq = self.config.ftol
        ocp.solver_options.nlp_solver_tol_ineq = self.config.ftol
        ocp.solver_options.nlp_solver_tol_comp = self.config.ftol
        ocp.solver_options.globalization = "MERIT_BACKTRACKING"
        ocp.solver_options.regularize_method = "CONVEXIFY"
        ocp.solver_options.print_level = 0
        return ocp, AcadosOcpSolver

    def export_solver_code(self) -> None:
        """Generate acados C solver code without building a shared library."""

        ocp, AcadosOcpSolver = self._build_ocp()
        AcadosOcpSolver.generate(ocp, verbose=False)

    def _casadi_residual(self, ca, x, u, p, *, terminal: bool, initial: bool = False):
        xref = p[0:STATE_SIZE]
        uref = p[STATE_SIZE : STATE_SIZE + 4]
        last_commanded_thrust = p[STATE_SIZE + 4]
        last_commanded_body_rate = p[STATE_SIZE + 5 : STATE_SIZE + 8]
        alpha_sqrt_weight = p[STATE_SIZE + 8 : STATE_SIZE + 11]
        rotation = self._casadi_quat_to_rotation(ca, x[6:10])
        rotation_ref = self._casadi_quat_to_rotation(ca, xref[6:10])
        e_thrust_direction = self._casadi_thrust_direction_error(ca, rotation, rotation_ref)
        e_yaw = self._casadi_projected_heading_error(ca, rotation, rotation_ref)
        e_pos = x[0:3] - xref[0:3]
        e_vel = x[3:6] - xref[3:6]
        e_omega = x[10:13] - xref[10:13]
        e_thrust_actual = x[THRUST_ACT] - xref[THRUST_ACT]
        equivalent_alpha = (u[1:4] - x[10:13]) / self.params.body_rate_time_constant
        e_alpha = ca.diag(alpha_sqrt_weight) @ equivalent_alpha
        unit_error = ca.dot(x[6:10], x[6:10]) - 1.0
        if terminal:
            residual = ca.vertcat(
                e_pos,
                e_vel,
                e_thrust_direction,
                e_yaw,
                e_omega,
                e_thrust_actual,
                unit_error,
            )
            weights = np.concatenate(
                (
                    self.weights.terminal_position,
                    self.weights.terminal_velocity,
                    self.weights.terminal_thrust_direction,
                    np.array([self.weights.terminal_yaw]),
                    self.weights.terminal_omega,
                    np.array([self.weights.terminal_thrust_actual]),
                    np.array([self.weights.unit_quat_penalty]),
                )
            )
        else:
            e_u = u - uref
            residual_parts = [
                e_pos,
                e_vel,
                e_thrust_direction,
                e_yaw,
                e_omega,
                e_thrust_actual,
                e_u,
                e_alpha,
            ]
            weight_parts = [
                self.weights.position,
                self.weights.velocity,
                self.weights.thrust_direction,
                np.array([self.weights.yaw]),
                self.weights.omega,
                np.array([self.weights.thrust_actual]),
                self.weights.control,
                np.ones(3),
            ]
            if initial:
                e_thrust_command_delta = u[0] - last_commanded_thrust
                e_body_rate_command_delta = u[1:4] - last_commanded_body_rate
                residual_parts.append(e_thrust_command_delta)
                residual_parts.append(e_body_rate_command_delta)
                weight_parts.append(np.array([self.weights.thrust_command_delta]))
                weight_parts.append(self.weights.body_rate_command_delta)
            residual_parts.append(unit_error)
            weight_parts.append(np.array([self.weights.unit_quat_penalty]))
            residual = ca.vertcat(
                *residual_parts,
            )
            weights = np.concatenate(
                tuple(weight_parts)
            )
        return residual, weights

    @staticmethod
    def _casadi_quat_derivative(ca, quat, omega):
        norm_q = ca.sqrt(ca.dot(quat, quat) + 1e-12)
        q = quat / norm_q
        w, x_val, y_val, z_val = q[0], q[1], q[2], q[3]
        wx, wy, wz = omega[0], omega[1], omega[2]
        return 0.5 * ca.vertcat(
            -x_val * wx - y_val * wy - z_val * wz,
            w * wx + y_val * wz - z_val * wy,
            w * wy - x_val * wz + z_val * wx,
            w * wz + x_val * wy - y_val * wx,
        )

    @staticmethod
    def _casadi_quat_to_rotation(ca, quat):
        norm_q = ca.sqrt(ca.dot(quat, quat) + 1e-12)
        q = quat / norm_q
        w, x_val, y_val, z_val = q[0], q[1], q[2], q[3]
        return ca.vertcat(
            ca.horzcat(1 - 2 * (y_val**2 + z_val**2), 2 * (x_val * y_val - z_val * w), 2 * (x_val * z_val + y_val * w)),
            ca.horzcat(2 * (x_val * y_val + z_val * w), 1 - 2 * (x_val**2 + z_val**2), 2 * (y_val * z_val - x_val * w)),
            ca.horzcat(2 * (x_val * z_val - y_val * w), 2 * (y_val * z_val + x_val * w), 1 - 2 * (x_val**2 + y_val**2)),
        )

    @staticmethod
    def _casadi_thrust_direction_error(ca, rotation, reference_rotation):
        return ca.cross(reference_rotation[:, 2], rotation[:, 2])

    @staticmethod
    def _casadi_projected_heading_error(ca, rotation, reference_rotation):
        z_ref = reference_rotation[:, 2]
        x_axis = AcadosNMPCController._casadi_project_heading(ca, rotation[:, 0], z_ref)
        x_ref = AcadosNMPCController._casadi_project_heading(ca, reference_rotation[:, 0], z_ref)
        return ca.dot(z_ref, ca.cross(x_ref, x_axis))

    @staticmethod
    def _casadi_project_heading(ca, x_axis, z_ref):
        projected = x_axis - z_ref * ca.dot(z_ref, x_axis)
        return projected / ca.sqrt(ca.dot(projected, projected) + 1e-12)
