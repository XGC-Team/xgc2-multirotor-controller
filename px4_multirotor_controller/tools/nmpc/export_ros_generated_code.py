"""Export the Python-defined UAV acados solver into the ROS C++ package.

The generated OCP structure is only one half of runtime equivalence. The C++
wrapper must still mirror the Python solver interaction sequence every control
cycle:

1. set the fixed x0 lower/upper bounds;
2. set stage parameters p = [xref(13); uref(4)] for stages 0..N;
3. seed x/u warm starts;
4. call solve and reject nonzero status;
5. read u0 and predicted x1;
6. shift warm start with the next terminal reference;
7. map [T/m; angular_acceleration] to PX4 body-rate plus normalized thrust.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

if __package__ in {None, ""}:
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from nmpc.controller import AcadosNMPCController, MPCConfig
    from nmpc.dynamics import default_quadrotor_params
    from nmpc.references import HoverTrajectory
else:
    from .controller import AcadosNMPCController, MPCConfig
    from .dynamics import default_quadrotor_params
    from .references import HoverTrajectory


def repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in (here.parent, *here.parents):
        if (
            parent
            / "source"
            / "ros1_ws"
            / "src"
            / "controller"
            / "px4_multirotor_controller"
            / "tools"
            / "nmpc"
        ).exists():
            return parent
    return here.parents[3]


def default_generated_dir() -> Path:
    return (
        repo_root()
        / "source"
        / "ros1_ws"
        / "src"
        / "controller"
        / "px4_multirotor_controller"
        / "generated"
        / "nmpc"
        / "uav_nmpc"
    )


def clean_uav_generated(output_dir: Path) -> None:
    """Remove only UAV acados files, leaving UGV generated code untouched."""

    stale_paths = [
        output_dir / "acados_solver_uav_nmpc.c",
        output_dir / "acados_solver_uav_nmpc.h",
        output_dir / "acados_sim_solver_uav_nmpc.c",
        output_dir / "acados_sim_solver_uav_nmpc.h",
        output_dir / "main_uav_nmpc.c",
        output_dir / "main_sim_uav_nmpc.c",
        output_dir / "uav_nmpc_model",
        output_dir / "uav_nmpc_cost",
        output_dir / "uav_nmpc_constraints",
    ]
    for path in stale_paths:
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()


def remove_unused_export_files(output_dir: Path) -> None:
    """Drop demo/build files that are not compiled by the ROS package."""

    for name in (
        "CMakeLists.txt",
        "Makefile",
        "main_uav_nmpc.c",
        "main_sim_uav_nmpc.c",
        "acados_sim_solver_uav_nmpc.c",
        "acados_sim_solver_uav_nmpc.h",
        "libacados_ocp_solver_uav_nmpc.so",
        "uav_nmpc_acados_ocp.json",
        "acados_solver.pxd",
    ):
        path = output_dir / name
        if path.exists():
            path.unlink()


def export_solver(output_dir: Path, *, horizon: float, steps: int) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    clean_uav_generated(output_dir)

    json_file = output_dir / "uav_nmpc_acados_ocp.json"
    params = default_quadrotor_params()
    reference = HoverTrajectory(params=params)
    controller = AcadosNMPCController(
        reference,
        params=params,
        config=MPCConfig(
            horizon=horizon,
            steps=steps,
            backend="acados",
            max_iter=20,
            control_interval=horizon / steps,
            model_name="uav_nmpc",
            code_export_directory=str(output_dir),
            json_file=str(json_file),
        ),
        build_solver=False,
    )
    controller.export_solver_code()
    remove_unused_export_files(output_dir)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=default_generated_dir())
    parser.add_argument("--horizon", type=float, default=1.0)
    parser.add_argument("--steps", type=int, default=10)
    args = parser.parse_args(argv)

    export_solver(args.output_dir, horizon=args.horizon, steps=args.steps)
    print(f"exported UAV acados solver to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
