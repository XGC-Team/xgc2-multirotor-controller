"""Closed-loop multi-trajectory runner for the px4_multirotor_controller UAV NMPC OCP."""

from __future__ import annotations

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from nmpc.controller import (
        AcadosBackendUnavailable,
        AcadosNMPCController,
        Bounds,
        MPCConfig,
        optional_backend_status,
    )
    from nmpc.dynamics import default_quadrotor_params, quadrotor_dynamics
    from nmpc.math_utils import (
        STATE_SIZE,
        pack_state,
        project_state_rotation,
        quat_to_rotation,
        rk4_step_time,
        rotation_to_quat,
        unpack_state,
        unpack_state_quat,
    )
    from nmpc.references import TRAJECTORY_NAMES, normalize_task_id, trajectory_factory
else:
    from .controller import (
        AcadosBackendUnavailable,
        AcadosNMPCController,
        Bounds,
        MPCConfig,
        optional_backend_status,
    )
    from .dynamics import default_quadrotor_params, quadrotor_dynamics
    from .math_utils import (
        STATE_SIZE,
        pack_state,
        project_state_rotation,
        quat_to_rotation,
        rk4_step_time,
        rotation_to_quat,
        unpack_state,
        unpack_state_quat,
    )
    from .references import TRAJECTORY_NAMES, normalize_task_id, trajectory_factory

import argparse
import json
import platform
import subprocess
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class SimulationConfig:
    """Closed-loop runner configuration."""

    task_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
    dt_ctrl: float = 0.01
    dt_sim: float = 0.005
    tsim: float = 1.0
    mpc_horizon: float = 1.0
    mpc_div: int = 10
    max_iter: int = 20
    backend: str = "acados"
    xy_offset: tuple[float, float] = (1.0, 1.0)
    initial_pos_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    initial_vel_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    initial_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    disturbance_accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    disturbance_accel_sin_amp: tuple[float, float, float] = (0.0, 0.0, 0.0)
    disturbance_accel_sin_freq: tuple[float, float, float] = (0.0, 0.0, 0.0)
    verbose: bool = True


def validate_config(cfg: SimulationConfig) -> None:
    """Validate runner timing and optimizer configuration."""

    if cfg.dt_ctrl <= 0.0:
        raise ValueError("dt_ctrl must be positive")
    if cfg.dt_sim <= 0.0:
        raise ValueError("dt_sim must be positive")
    if cfg.tsim < 0.0:
        raise ValueError("tsim must be non-negative")
    if cfg.mpc_horizon <= 0.0:
        raise ValueError("mpc_horizon must be positive")
    if cfg.mpc_div <= 0:
        raise ValueError("mpc_div must be positive")
    if cfg.max_iter <= 0:
        raise ValueError("max_iter must be positive")
    if cfg.backend != "acados":
        raise ValueError("only backend='acados' is supported")
    if len(cfg.initial_pos_offset) != 3:
        raise ValueError("initial_pos_offset must have three elements")
    if len(cfg.initial_vel_offset) != 3:
        raise ValueError("initial_vel_offset must have three elements")
    if len(cfg.initial_rpy_deg) != 3:
        raise ValueError("initial_rpy_deg must have three elements")
    if len(cfg.disturbance_accel) != 3:
        raise ValueError("disturbance_accel must have three elements")
    if len(cfg.disturbance_accel_sin_amp) != 3:
        raise ValueError("disturbance_accel_sin_amp must have three elements")
    if len(cfg.disturbance_accel_sin_freq) != 3:
        raise ValueError("disturbance_accel_sin_freq must have three elements")
    ratio = cfg.dt_ctrl / cfg.dt_sim
    if ratio < 1.0 or abs(ratio - round(ratio)) > 1e-10:
        raise ValueError("dt_ctrl must be an integer multiple of dt_sim")


def run_multi_trajectory_sim(cfg: SimulationConfig | None = None) -> dict[str, object]:
    """Run closed-loop NMPC simulations for the configured trajectory set."""

    cfg = cfg if cfg is not None else SimulationConfig()
    validate_config(cfg)
    params = build_params(cfg)
    cases = []
    summaries = []

    for task_id in cfg.task_ids:
        trajectory = trajectory_factory(
            task_id,
            dt=cfg.dt_ctrl,
            params=params,
            xy_offset=cfg.xy_offset,
        )
        case = run_one_case(task_id, trajectory, params, cfg)
        cases.append(case)
        summaries.append(case["summary"])
        if cfg.verbose:
            summary = case["summary"]
            print(
                "task {task_id} {name}: success {success_rate:.1f}%, "
                "opt-success {optimizer_success_rate:.1f}%, RMSE pos {rmse_pos:.3f} m, "
                "peak pos {peak_pos:.3f} m, update p95 {update_p95_ms:.1f} ms, "
                "solve p95 {solve_p95_ms:.1f} ms".format(
                    **summary
                )
            )

    return {
        "config": asdict(cfg),
        "backend_status": optional_backend_status(),
        "solver_metadata": solver_metadata(cfg),
        "cases": cases,
        "summary": summaries,
    }


def run_one_case(task_id: int, trajectory, params, cfg: SimulationConfig) -> dict[str, object]:
    """Run one trajectory case with receding-horizon feedback."""

    state, initial_command = trajectory.get_reference(0.0)
    state = project_state_rotation(state)
    state = apply_initial_perturbation(state, cfg)

    mpc_config = MPCConfig(
        horizon=cfg.mpc_horizon,
        steps=cfg.mpc_div,
        backend=cfg.backend,
        max_iter=cfg.max_iter,
        control_interval=cfg.dt_ctrl,
    )
    controller = AcadosNMPCController(trajectory, params=params, config=mpc_config)

    time_values = np.arange(0.0, cfg.tsim + 0.5 * cfg.dt_sim, cfg.dt_sim)
    states = np.zeros((time_values.size, STATE_SIZE))
    refs = np.zeros((time_values.size, STATE_SIZE))
    ref_accel = np.zeros((time_values.size, 3))
    vehicle_accel = np.zeros((time_values.size, 3))
    commands = np.zeros((time_values.size, 4))
    statuses: list[int] = []
    accepted: list[bool] = []
    finite_solution: list[bool] = []
    optimizer_success: list[bool] = []
    update_ms: list[float] = []
    solve_ms: list[float] = []
    objective: list[float] = []
    iterations: list[int] = []
    qp_iter: list[int] = []
    res_stat: list[float] = []
    res_eq: list[float] = []
    res_ineq: list[float] = []
    res_comp: list[float] = []
    time_qp_ms: list[float] = []
    time_lin_ms: list[float] = []

    command = initial_command
    next_ctrl_time = 0.0
    states[0] = state
    refs[0] = trajectory.get_reference(0.0)[0]
    ref_accel[0] = trajectory.flat_output(0.0).acceleration
    commands[0] = command
    vehicle_accel[0] = quadrotor_dynamics(0.0, state, command, params)[3:6]

    for idx in range(1, time_values.size):
        t_prev = time_values[idx - 1]
        if t_prev >= next_ctrl_time - 1e-12:
            command, info = controller.update(next_ctrl_time, state)
            statuses.append(info.status)
            accepted.append(info.accepted)
            finite_solution.append(info.finite_solution)
            optimizer_success.append(info.optimizer_success)
            update_ms.append(info.update_ms)
            solve_ms.append(info.solve_ms)
            objective.append(info.objective)
            iterations.append(info.iterations)
            qp_iter.append(info.qp_iter)
            res_stat.append(info.res_stat)
            res_eq.append(info.res_eq)
            res_ineq.append(info.res_ineq)
            res_comp.append(info.res_comp)
            time_qp_ms.append(info.time_qp_ms)
            time_lin_ms.append(info.time_lin_ms)
            next_ctrl_time += cfg.dt_ctrl

        state = rk4_step_time(
            lambda sub_time, x: quadrotor_dynamics(sub_time, x, command, params),
            t_prev,
            state,
            cfg.dt_sim,
        )
        state = project_state_rotation(state)
        states[idx] = state
        refs[idx] = trajectory.get_reference(time_values[idx])[0]
        ref_accel[idx] = trajectory.flat_output(time_values[idx]).acceleration
        commands[idx] = command
        vehicle_accel[idx] = quadrotor_dynamics(time_values[idx], state, command, params)[3:6]

    case = {
        "task_id": int(task_id),
        "name": TRAJECTORY_NAMES[int(task_id)],
        "time": time_values,
        "states": states,
        "reference": refs,
        "reference_acceleration": ref_accel,
        "vehicle_acceleration": vehicle_accel,
        "commands": commands,
        "controller_status": np.asarray(statuses, dtype=int),
        "accepted_control": np.asarray(accepted, dtype=bool),
        "finite_solution": np.asarray(finite_solution, dtype=bool),
        "optimizer_success": np.asarray(optimizer_success, dtype=bool),
        "controller_update_ms": np.asarray(update_ms, dtype=float),
        "controller_solve_ms": np.asarray(solve_ms, dtype=float),
        "objective": np.asarray(objective, dtype=float),
        "sqp_iter": np.asarray(iterations, dtype=int),
        "qp_iter": np.asarray(qp_iter, dtype=int),
        "res_stat": np.asarray(res_stat, dtype=float),
        "res_eq": np.asarray(res_eq, dtype=float),
        "res_ineq": np.asarray(res_ineq, dtype=float),
        "res_comp": np.asarray(res_comp, dtype=float),
        "time_qp_ms": np.asarray(time_qp_ms, dtype=float),
        "time_lin_ms": np.asarray(time_lin_ms, dtype=float),
    }
    case["summary"] = compute_summary(case)
    return case


def compute_summary(case: dict[str, object]) -> dict[str, float | int | str]:
    states = np.asarray(case["states"], dtype=float)
    refs = np.asarray(case["reference"], dtype=float)
    pos_err = states[:, 0:3] - refs[:, 0:3]
    vel_err = states[:, 3:6] - refs[:, 3:6]
    pos_norm = np.linalg.norm(pos_err, axis=1)
    vel_norm = np.linalg.norm(vel_err, axis=1)

    statuses = np.asarray(case["controller_status"])
    accepted = np.asarray(case["accepted_control"])
    finite_solution = np.asarray(case["finite_solution"])
    optimizer_success = np.asarray(case["optimizer_success"])
    update_ms = np.asarray(case["controller_update_ms"], dtype=float)
    solve_ms = np.asarray(case["controller_solve_ms"], dtype=float)
    objective = np.asarray(case["objective"], dtype=float)
    sqp_iter = np.asarray(case["sqp_iter"], dtype=float)
    qp_iter = np.asarray(case["qp_iter"], dtype=float)
    res_stat = np.asarray(case["res_stat"], dtype=float)
    res_eq = np.asarray(case["res_eq"], dtype=float)
    res_ineq = np.asarray(case["res_ineq"], dtype=float)
    res_comp = np.asarray(case["res_comp"], dtype=float)
    time_qp_ms = np.asarray(case["time_qp_ms"], dtype=float)
    time_lin_ms = np.asarray(case["time_lin_ms"], dtype=float)

    rotation_errors = []
    quat_norm_errors = []
    constraint_violations = []
    bounds = Bounds()
    for state in states:
        _, _, quat, _ = unpack_state_quat(state)
        rotation = quat_to_rotation(quat)
        rotation_errors.append(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
        quat_norm_errors.append(abs(float(np.dot(state[6:10], state[6:10]) - 1.0)))
        constraint_violations.append(state_constraint_violation(state, bounds))

    return {
        "task_id": int(case["task_id"]),
        "name": str(case["name"]),
        "rmse_pos": float(np.sqrt(np.mean(pos_norm**2))),
        "peak_pos": float(np.max(pos_norm)),
        "rmse_vel": float(np.sqrt(np.mean(vel_norm**2))),
        "peak_vel": float(np.max(vel_norm)),
        "success_rate": 100.0 * _safe_mean(statuses == 0),
        "accepted_rate": 100.0 * _safe_mean(accepted),
        "finite_solution_rate": 100.0 * _safe_mean(finite_solution),
        "optimizer_success_rate": 100.0 * _safe_mean(optimizer_success),
        "update_mean_ms": _finite_mean(update_ms),
        "update_max_ms": _finite_max(update_ms),
        "update_p95_ms": _finite_percentile(update_ms, 95.0),
        "solve_mean_ms": _finite_mean(solve_ms),
        "solve_max_ms": _finite_max(solve_ms),
        "solve_p95_ms": _finite_percentile(solve_ms, 95.0),
        "objective_mean": _finite_mean(objective),
        "objective_final": _finite_last(objective),
        "sqp_iter_max": _finite_max(sqp_iter),
        "qp_iter_max": _finite_max(qp_iter),
        "res_stat_max": _finite_max(res_stat),
        "res_eq_max": _finite_max(res_eq),
        "res_ineq_max": _finite_max(res_ineq),
        "res_comp_max": _finite_max(res_comp),
        "time_qp_p95_ms": _finite_percentile(time_qp_ms, 95.0),
        "time_lin_p95_ms": _finite_percentile(time_lin_ms, 95.0),
        "quat_norm_error_max": float(np.max(quat_norm_errors)),
        "rotation_orthogonality_max": float(np.max(rotation_errors)),
        "state_constraint_violation_max": float(np.max(constraint_violations)),
    }


def build_params(cfg: SimulationConfig):
    """Build plant parameters, including optional validation disturbances."""

    params = default_quadrotor_params()
    return replace(
        params,
        disturbance_accel_I=np.asarray(cfg.disturbance_accel, dtype=float),
        disturbance_accel_sin_amp_I=np.asarray(cfg.disturbance_accel_sin_amp, dtype=float),
        disturbance_accel_sin_freq_hz=np.asarray(cfg.disturbance_accel_sin_freq, dtype=float),
    )


def solver_metadata(cfg: SimulationConfig) -> dict[str, object]:
    """Return validation metadata that ties results to an OCP configuration."""

    status = optional_backend_status()
    acados_root = str(status.get("acados_root", ""))
    return {
        "model_name": "uav_nmpc",
        "nx": STATE_SIZE,
        "nu": 4,
        "np": STATE_SIZE + 4,
        "ny": 17,
        "nyn": 13,
        "nh": 1,
        "nhn": 1,
        "horizon": cfg.mpc_horizon,
        "steps": cfg.mpc_div,
        "dt": cfg.mpc_horizon / cfg.mpc_div,
        "control_interval": cfg.dt_ctrl,
        "nlp_solver_type": "SQP_RTI",
        "qp_solver": "PARTIAL_CONDENSING_HPIPM",
        "rti_status_note": (
            "status==0 means the SQP_RTI feedback step succeeded; it is not "
            "evidence that each OCP was solved as a fully converged SQP NLP."
        ),
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
        "casadi_version": _module_version("casadi"),
        "acados_template_version": _module_version("acados_template"),
        "acados_commit": _git_commit(acados_root) if acados_root else "",
    }


def apply_initial_perturbation(state: np.ndarray, cfg: SimulationConfig) -> np.ndarray:
    """Apply optional initial position, velocity, and attitude offsets."""

    pos_offset = np.asarray(cfg.initial_pos_offset, dtype=float)
    vel_offset = np.asarray(cfg.initial_vel_offset, dtype=float)
    rpy_deg = np.asarray(cfg.initial_rpy_deg, dtype=float)
    if not (
        np.any(np.abs(pos_offset) > 0.0)
        or np.any(np.abs(vel_offset) > 0.0)
        or np.any(np.abs(rpy_deg) > 0.0)
    ):
        return state

    position, velocity, quat, omega = unpack_state_quat(state)
    rotation = quat_to_rotation(quat)
    rotation = rotation @ rpy_to_rotation(np.deg2rad(rpy_deg))
    return pack_state(position + pos_offset, velocity + vel_offset, rotation_to_quat(rotation), omega)


def rpy_to_rotation(rpy_rad: np.ndarray) -> np.ndarray:
    """Return the ZYX yaw-pitch-roll rotation for roll, pitch, yaw radians."""

    roll, pitch, yaw = np.asarray(rpy_rad, dtype=float).reshape(3)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def state_constraint_violation(state: np.ndarray, bounds: Bounds) -> float:
    """Return the largest scalar violation of the soft state bounds."""

    state = np.asarray(state, dtype=float).reshape(-1)
    lbx, ubx = bounds.state_lbx_ubx()
    box_violation = np.maximum(np.maximum(lbx - state, 0.0), np.maximum(state - ubx, 0.0))
    _, _, rotation, _ = unpack_state(state)
    tilt_violation = max(np.cos(np.deg2rad(bounds.tilt_max_deg)) - rotation[2, 2], 0.0)
    return float(max(np.max(box_violation), tilt_violation))


def parse_task_ids(raw: str) -> tuple[int, ...]:
    if raw.strip().lower() == "all":
        return tuple(TRAJECTORY_NAMES)
    return tuple(normalize_task_id(part) for part in raw.split(",") if part.strip())


def parse_vec3(raw: str) -> tuple[float, float, float]:
    values = tuple(float(part.strip()) for part in raw.split(",") if part.strip())
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")
    return values


def summary_to_jsonable(summary: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    return [{key: _json_scalar(value) for key, value in item.items()} for item in summary]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", default="all", help="all, a task id, a name, or comma-separated tasks")
    parser.add_argument("--tsim", type=float, default=1.0)
    parser.add_argument("--dt-ctrl", type=float, default=0.01)
    parser.add_argument("--dt-sim", type=float, default=0.005)
    parser.add_argument("--mpc-horizon", type=float, default=1.0)
    parser.add_argument("--mpc-div", type=int, default=10)
    parser.add_argument("--max-iter", type=int, default=20)
    parser.add_argument("--backend", choices=("acados",), default="acados")
    parser.add_argument(
        "--initial-pos-offset",
        type=parse_vec3,
        default=(0.0, 0.0, 0.0),
        help="initial position offset as x,y,z meters",
    )
    parser.add_argument(
        "--initial-vel-offset",
        type=parse_vec3,
        default=(0.0, 0.0, 0.0),
        help="initial velocity offset as vx,vy,vz m/s",
    )
    parser.add_argument(
        "--initial-rpy-deg",
        type=parse_vec3,
        default=(0.0, 0.0, 0.0),
        help="initial attitude perturbation as roll,pitch,yaw degrees",
    )
    parser.add_argument(
        "--disturbance-accel",
        type=parse_vec3,
        default=(0.0, 0.0, 0.0),
        help="constant inertial acceleration disturbance ax,ay,az [m/s^2]",
    )
    parser.add_argument(
        "--disturbance-accel-sin-amp",
        type=parse_vec3,
        default=(0.0, 0.0, 0.0),
        help="sinusoidal inertial acceleration disturbance amplitude ax,ay,az [m/s^2]",
    )
    parser.add_argument(
        "--disturbance-accel-sin-freq",
        type=parse_vec3,
        default=(0.0, 0.0, 0.0),
        help="sinusoidal inertial acceleration disturbance frequency fx,fy,fz [Hz]",
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--json", action="store_true", help="print summary JSON")
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent / "results" / "latest"),
        help="directory for summary.json and per-task .npz results",
    )
    parser.add_argument("--plot", action="store_true", help="save trajectory PNGs under --output-dir")
    args = parser.parse_args(argv)

    cfg = SimulationConfig(
        task_ids=parse_task_ids(args.task),
        dt_ctrl=args.dt_ctrl,
        dt_sim=args.dt_sim,
        tsim=args.tsim,
        mpc_horizon=args.mpc_horizon,
        mpc_div=args.mpc_div,
        max_iter=args.max_iter,
        backend=args.backend,
        initial_pos_offset=args.initial_pos_offset,
        initial_vel_offset=args.initial_vel_offset,
        initial_rpy_deg=args.initial_rpy_deg,
        disturbance_accel=args.disturbance_accel,
        disturbance_accel_sin_amp=args.disturbance_accel_sin_amp,
        disturbance_accel_sin_freq=args.disturbance_accel_sin_freq,
        verbose=not args.quiet and not args.json,
    )
    try:
        out = run_multi_trajectory_sim(cfg)
    except (AcadosBackendUnavailable, NotImplementedError) as exc:
        parser.error(str(exc))
    save_outputs(out, Path(args.output_dir), plot=args.plot)

    if args.json:
        print(
            json.dumps(
                {
                    "config": out["config"],
                    "backend_status": out["backend_status"],
                    "solver_metadata": out["solver_metadata"],
                    "summary": summary_to_jsonable(out["summary"]),
                },
                indent=2,
                sort_keys=True,
            )
        )
    return 0


def save_outputs(out: dict[str, object], output_dir: Path, *, plot: bool = False) -> None:
    """Save simulation arrays and summaries for offline inspection."""

    output_dir.mkdir(parents=True, exist_ok=True)
    summary = summary_to_jsonable(out["summary"])
    (output_dir / "summary.json").write_text(
        json.dumps(
            {
                "config": out["config"],
                "backend_status": out["backend_status"],
                "solver_metadata": out["solver_metadata"],
                "summary": summary,
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    for case in out["cases"]:
        name = str(case["name"])
        task_id = int(case["task_id"])
        stem = f"task{task_id}_{name}"
        np.savez(
            output_dir / f"{stem}.npz",
            time=np.asarray(case["time"]),
            states=np.asarray(case["states"]),
            reference=np.asarray(case["reference"]),
            reference_acceleration=np.asarray(case["reference_acceleration"]),
            vehicle_acceleration=np.asarray(case["vehicle_acceleration"]),
            commands=np.asarray(case["commands"]),
            controller_status=np.asarray(case["controller_status"]),
            accepted_control=np.asarray(case["accepted_control"]),
            finite_solution=np.asarray(case["finite_solution"]),
            optimizer_success=np.asarray(case["optimizer_success"]),
            controller_update_ms=np.asarray(case["controller_update_ms"]),
            controller_solve_ms=np.asarray(case["controller_solve_ms"]),
            objective=np.asarray(case["objective"]),
            sqp_iter=np.asarray(case["sqp_iter"]),
            qp_iter=np.asarray(case["qp_iter"]),
            res_stat=np.asarray(case["res_stat"]),
            res_eq=np.asarray(case["res_eq"]),
            res_ineq=np.asarray(case["res_ineq"]),
            res_comp=np.asarray(case["res_comp"]),
            time_qp_ms=np.asarray(case["time_qp_ms"]),
            time_lin_ms=np.asarray(case["time_lin_ms"]),
        )
        if plot:
            save_case_plots(case, output_dir, stem)


def save_case_plots(case: dict[str, object], output_dir: Path, stem: str) -> None:
    """Save tracking, command, and solver diagnostic plots for one case."""

    save_position_plot(case, output_dir / f"{stem}_position.png")
    save_trajectory_plot(case, output_dir / f"{stem}_trajectory.png")
    save_error_plot(case, output_dir / f"{stem}_error.png")
    save_acceleration_plot(case, output_dir / f"{stem}_acceleration.png")
    save_reference_accel_plot(case, output_dir / f"{stem}_ref_accel.png")
    save_command_plot(case, output_dir / f"{stem}_command.png")
    save_solver_plot(case, output_dir / f"{stem}_solver.png")


def save_position_plot(case: dict[str, object], path: Path) -> None:
    """Save a compact position-tracking plot for one case."""

    plt = _load_pyplot()

    time_values = np.asarray(case["time"], dtype=float)
    states = np.asarray(case["states"], dtype=float)
    refs = np.asarray(case["reference"], dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(7.0, 5.5), sharex=True)
    labels = ("x", "y", "z")
    for idx, ax in enumerate(axes):
        ax.plot(time_values, refs[:, idx], "k--", linewidth=1.2, label="ref")
        ax.plot(time_values, states[:, idx], linewidth=1.4, label="state")
        ax.set_ylabel(labels[idx])
        ax.grid(True, alpha=0.3)
    axes[0].set_title(f"task {case['task_id']} {case['name']} position tracking")
    axes[-1].set_xlabel("time [s]")
    axes[0].legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_trajectory_plot(case: dict[str, object], path: Path) -> None:
    """Save XY and XZ path projections for one case."""

    plt = _load_pyplot()

    states = np.asarray(case["states"], dtype=float)
    refs = np.asarray(case["reference"], dtype=float)

    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.8))
    projections = ((0, 1, "x [m]", "y [m]", "XY path"), (0, 2, "x [m]", "z [m]", "XZ path"))
    for ax, (a, b, xlabel, ylabel, title) in zip(axes, projections):
        ax.plot(refs[:, a], refs[:, b], "k--", linewidth=1.2, label="ref")
        ax.plot(states[:, a], states[:, b], linewidth=1.4, label="state")
        ax.plot(states[0, a], states[0, b], "go", markersize=4, label="start")
        ax.plot(states[-1, a], states[-1, b], "ro", markersize=4, label="end")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    fig.suptitle(f"task {case['task_id']} {case['name']} path")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_error_plot(case: dict[str, object], path: Path) -> None:
    """Save position and velocity tracking error plots for one case."""

    plt = _load_pyplot()

    time_values = np.asarray(case["time"], dtype=float)
    states = np.asarray(case["states"], dtype=float)
    refs = np.asarray(case["reference"], dtype=float)
    pos_err = states[:, 0:3] - refs[:, 0:3]
    vel_err = states[:, 3:6] - refs[:, 3:6]

    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
    axes[0].plot(time_values, pos_err)
    axes[0].plot(time_values, np.linalg.norm(pos_err, axis=1), "k", linewidth=1.2, label="norm")
    axes[0].set_ylabel("pos error [m]")
    axes[0].legend(("x", "y", "z", "norm"), loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time_values, vel_err)
    axes[1].plot(time_values, np.linalg.norm(vel_err, axis=1), "k", linewidth=1.2, label="norm")
    axes[1].set_ylabel("vel error [m/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(("vx", "vy", "vz", "norm"), loc="best")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle(f"task {case['task_id']} {case['name']} tracking error")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_command_plot(case: dict[str, object], path: Path) -> None:
    """Save high-level NMPC command plots for one case."""

    plt = _load_pyplot()

    time_values = np.asarray(case["time"], dtype=float)
    commands = np.asarray(case["commands"], dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
    axes[0].step(time_values, commands[:, 0], where="post", linewidth=1.4)
    axes[0].set_ylabel("T/m [m/s^2]")
    axes[0].grid(True, alpha=0.3)

    for idx, label in enumerate(("ax", "ay", "az"), start=1):
        axes[1].step(time_values, commands[:, idx], where="post", linewidth=1.3, label=label)
    axes[1].set_ylabel("angular accel [rad/s^2]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle(f"task {case['task_id']} {case['name']} NMPC command")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_reference_accel_plot(case: dict[str, object], path: Path) -> None:
    """Save smooth flat-output translational acceleration reference."""

    plt = _load_pyplot()

    time_values = np.asarray(case["time"], dtype=float)
    ref_accel = np.asarray(case["reference_acceleration"], dtype=float)

    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    ax.plot(time_values, ref_accel, linewidth=1.3)
    ax.set_title(f"task {case['task_id']} {case['name']} reference acceleration")
    ax.set_ylabel("accel [m/s^2]")
    ax.set_xlabel("time [s]")
    ax.legend(("ax_ref", "ay_ref", "az_ref"), loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_acceleration_plot(case: dict[str, object], path: Path) -> None:
    """Save translational acceleration tracking for one case."""

    plt = _load_pyplot()

    time_values = np.asarray(case["time"], dtype=float)
    ref_accel = np.asarray(case["reference_acceleration"], dtype=float)
    vehicle_accel = np.asarray(case["vehicle_acceleration"], dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(7.0, 5.5), sharex=True)
    labels = ("ax", "ay", "az")
    for idx, ax in enumerate(axes):
        ax.plot(time_values, ref_accel[:, idx], "k--", linewidth=1.2, label="ref")
        ax.plot(time_values, vehicle_accel[:, idx], linewidth=1.3, label="vehicle")
        ax.set_ylabel(f"{labels[idx]} [m/s^2]")
        ax.grid(True, alpha=0.3)
    axes[0].set_title(f"task {case['task_id']} {case['name']} translational acceleration")
    axes[-1].set_xlabel("time [s]")
    axes[0].legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_solver_plot(case: dict[str, object], path: Path) -> None:
    """Save solver status, timing, objective, and residual diagnostics."""

    plt = _load_pyplot()

    update_ms = np.asarray(case["controller_update_ms"], dtype=float)
    solve_ms = np.asarray(case["controller_solve_ms"], dtype=float)
    statuses = np.asarray(case["controller_status"], dtype=int)
    accepted = np.asarray(case["accepted_control"], dtype=bool)
    finite_solution = np.asarray(case["finite_solution"], dtype=bool)
    optimizer_success = np.asarray(case["optimizer_success"], dtype=bool)
    objective = np.asarray(case["objective"], dtype=float)
    res_stat = np.asarray(case["res_stat"], dtype=float)
    res_eq = np.asarray(case["res_eq"], dtype=float)
    res_ineq = np.asarray(case["res_ineq"], dtype=float)
    res_comp = np.asarray(case["res_comp"], dtype=float)
    ctrl_idx = np.arange(update_ms.size)

    fig, axes = plt.subplots(4, 1, figsize=(7.0, 6.5), sharex=True)
    axes[0].plot(ctrl_idx, update_ms, marker="o", markersize=3, label="total")
    axes[0].plot(ctrl_idx, solve_ms, marker=".", markersize=3, label="solve")
    axes[0].set_ylabel("time [ms]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].step(ctrl_idx, statuses, where="post", label="status")
    axes[1].step(ctrl_idx, accepted.astype(float), where="post", label="accepted")
    axes[1].step(ctrl_idx, finite_solution.astype(float), where="post", label="finite")
    axes[1].step(ctrl_idx, optimizer_success.astype(float), where="post", label="optimizer ok")
    axes[1].set_ylabel("status")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(ctrl_idx, objective, marker="o", markersize=3)
    axes[2].set_ylabel("objective")
    axes[2].grid(True, alpha=0.3)

    axes[3].semilogy(ctrl_idx, res_stat, label="stat")
    axes[3].semilogy(ctrl_idx, res_eq, label="eq")
    axes[3].semilogy(ctrl_idx, res_ineq, label="ineq")
    axes[3].semilogy(ctrl_idx, res_comp, label="comp")
    axes[3].set_ylabel("residual")
    axes[3].set_xlabel("control update")
    axes[3].legend(loc="best")
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"task {case['task_id']} {case['name']} solver diagnostics")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _load_pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _safe_mean(values: np.ndarray) -> float:
    return float(np.mean(values)) if values.size else float("nan")


def _finite_mean(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    return float(np.mean(values)) if values.size else float("nan")


def _finite_max(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    return float(np.max(values)) if values.size else float("nan")


def _finite_percentile(values: np.ndarray, percentile: float) -> float:
    values = values[np.isfinite(values)]
    return float(np.percentile(values, percentile)) if values.size else float("nan")


def _finite_last(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    return float(values[-1]) if values.size else float("nan")


def _module_version(module_name: str) -> str:
    try:
        module = __import__(module_name)
    except Exception:
        return ""
    return str(getattr(module, "__version__", ""))


def _git_commit(path: str) -> str:
    if not path:
        return ""
    try:
        result = subprocess.run(
            ["git", "-C", path, "rev-parse", "HEAD"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except Exception:
        return ""
    return result.stdout.strip()


def _json_scalar(value):
    if isinstance(value, np.generic):
        return value.item()
    return value


if __name__ == "__main__":
    raise SystemExit(main())
