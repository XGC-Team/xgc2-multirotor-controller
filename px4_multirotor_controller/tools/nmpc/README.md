# Multirotor Controller Python Acados Quadrotor NMPC

This directory is the Python OCP source and validation toolchain for the
`px4_multirotor_controller` C++ UAV NMPC backend. It lives inside the C++ controller
package so the model, generated acados C code, C++ wrapper, PX4 bridge, tests,
and validation commands stay in one package boundary. It is acados-only:

```text
CasADi + acados AcadosOcpSolver + SQP_RTI + PARTIAL_CONDENSING_HPIPM
```

There is no alternate optimizer path. The code uses paths exported by the
system `xgc2-acados` package, including `acados_template`, `ACADOS_SOURCE_DIR`,
and the acados shared libraries.
The current repository validation uses Python 3.8.10; annotations are
postponed with `from __future__ import annotations`, so Python 3.8+ is the
supported interpreter target.

## Model

The Python state is quaternion based:

```text
x = [pos(3); vel(3); quat_wxyz(4); omega(3)]
u = [T/m; angular_acceleration(3)]
```

The high-level dynamics are intentionally one layer above PX4's direct
body-rate interface:

```text
p_dot     = v
v_dot     = -g e3 + (T/m) R(q) e3
q_dot     = 0.5 * q * [0, omega]
omega_dot = angular_acceleration
```

The NMPC command is therefore not sent to PX4 raw. Use
`to_px4_bodyrate_thrust()` or the ROS C++ tracking backend bridge:

```text
body_rate_cmd = predicted_x1.omega
thrust_norm   = hover_thrust_norm * (T/m) / g
```

This bridge still does not model the PX4 body-rate loop dynamics inside the
OCP, but it uses the optimizer-predicted next-state angular velocity instead of
manually integrating angular acceleration outside the solver. A future
deployment-oriented model can make the NMPC input `[normalized_thrust;
body_rate_cmd]` and approximate the inner loop with first-order body-rate
dynamics.

Attitude tracking cost uses the Lie algebra error
`log(R_ref.T @ R)^vee`; it does not use raw `R - R_ref` element error.
Both the Python plant model and the CasADi OCP model normalize the quaternion
before evaluating quaternion kinematics.

## Backend

`--backend acados` is the only supported backend. It builds an `AcadosOcp` and
`AcadosOcpSolver` with HPIPM partial condensing. If CasADi/acados cannot be
loaded, the runner exits with an error instead of silently switching backend.
An acados nonzero status is treated as rejected control even if the returned
vectors are finite; the controller falls back to the previous accepted command
or the current reference command.
Because the solver type is `SQP_RTI`, `status == 0` means a real-time iteration
step succeeded. It should not be read as proof that every OCP was solved as a
fully converged SQP NLP.

The acados OCP includes:

```text
input hard bounds:       T/m and angular acceleration
state hard bounds:       position, velocity, omega
nonlinear hard bounds:   R33(q) >= cos(tilt_max)
stage parameters:        [xref(13); uref(4)]
cost:                    NONLINEAR_LS tracking with SO(3) log attitude error
quaternion handling:     normalized simulation state plus unit-norm residual
```

## Run

From the repository root:

```bash
PYTHONPATH=source/ros1_ws/src/controller/px4_multirotor_controller/tools \
PYTHONPYCACHEPREFIX="${PWD}/devel/.cache/px4_multirotor_controller/pycache" \
python3 -B -m nmpc.runner --task hover --tsim 0.1 \
  --dt-ctrl 0.01 --dt-sim 0.005 --mpc-div 10 --mpc-horizon 1.0 \
  --max-iter 3 --json
```

For saved 20 s plots:

```bash
PYTHONPATH=source/ros1_ws/src/controller/px4_multirotor_controller/tools \
MPLCONFIGDIR="${PWD}/devel/.cache/px4_multirotor_controller/matplotlib" \
PYTHONPYCACHEPREFIX="${PWD}/devel/.cache/px4_multirotor_controller/pycache" \
python3 -B -m nmpc.runner --task all --tsim 20.0 \
  --dt-ctrl 0.01 --dt-sim 0.005 --mpc-div 10 --mpc-horizon 1.0 \
  --max-iter 20 --plot \
  --output-dir source/ros1_ws/src/controller/px4_multirotor_controller/tools/nmpc/results/current_commit_validation
```

For feedback robustness checks, add initial perturbations, for example:

```bash
PYTHONPATH=source/ros1_ws/src/controller/px4_multirotor_controller/tools \
python3 -B -m nmpc.runner --task hover --tsim 20.0 \
  --initial-pos-offset 0.3,-0.2,0.2 --initial-vel-offset 0.2,0.0,-0.1 \
  --initial-rpy-deg 8,5,0 --plot \
  --output-dir source/ros1_ws/src/controller/px4_multirotor_controller/tools/nmpc/results/perturbed_hover_20s
```

For model-mismatch checks, add inertial acceleration disturbances:

```bash
PYTHONPATH=source/ros1_ws/src/controller/px4_multirotor_controller/tools \
python3 -B -m nmpc.runner --task all --tsim 20.0 \
  --initial-pos-offset 0.3,-0.2,0.2 --initial-vel-offset 0.2,0.0,-0.1 \
  --initial-rpy-deg 8,5,0 --disturbance-accel 0.15,-0.05,0.0 \
  --disturbance-accel-sin-amp 0.05,0.05,0.0 \
  --disturbance-accel-sin-freq 0.2,0.35,0.0 --plot \
  --output-dir source/ros1_ws/src/controller/px4_multirotor_controller/tools/nmpc/results/perturbed_all_wind_20s
```

If this directory is unpacked as a standalone `nmpc/` package, run from the
parent directory with:

```bash
python3 -B -m nmpc.runner --task hover --tsim 0.1 --json
```

## Results

By default, results are saved under:

```text
source/ros1_ws/src/controller/px4_multirotor_controller/tools/nmpc/results/latest/
```

The whole `results/` directory is ignored by git. acados generated code and JSON
files are also ignored (`/c_generated_code/`, `/build/`, `/acados_ocp*.json`).
Temporary Python solver code generated by smoke tests is written below
`results/acados_codegen/<model>_N<steps>_Tf<horizon>/`; it is local validation
state and is not the committed C++ runtime solver. The committed runtime solver
is regenerated explicitly with `nmpc.export_ros_generated_code`.
With `--plot`, each task saves:

```text
*_position.png       x/y/z position tracking
*_trajectory.png     XY and XZ path projections
*_error.png          position and velocity errors
*_acceleration.png   vehicle acceleration vs reference acceleration
*_ref_accel.png      smooth reference acceleration
*_command.png        zero-order-held T/m and angular-acceleration commands
*_solver.png         acados status, accepted/finite flags, update/solve timing
*.npz                full arrays
summary.json         metrics
```

`success_rate` means `acados status == 0`. `accepted_rate`,
`finite_solution_rate`, and `optimizer_success_rate` are reported separately.
`update_*_ms` measures the full Python controller update, including setting
parameters, warm start, solve, and reading results. `solve_*_ms` measures only
the acados solve call. The `.npz` files also store objective values, SQP/QP
iteration counts, residuals, and QP/linearization timing from acados when those
statistics are available.

## Tests

```bash
PYTHONPATH=source/ros1_ws/src/controller/px4_multirotor_controller/tools \
PYTHONPYCACHEPREFIX="${PWD}/devel/.cache/px4_multirotor_controller/pycache" \
python3 -B -m unittest discover \
  -s source/ros1_ws/src/controller/px4_multirotor_controller/tools/nmpc/tests -v
```

The tests cover acados availability from `xgc2-acados`, exact hover
equilibrium, quaternion state layout, SO(3) log error, time-aware RK4,
warm-start storage, and runner timing validation.

## ROS C++ Export

The C++ controller uses the same Python-defined acados OCP. Regenerate the UAV
solver artifacts with:

```bash
PYTHONPATH=source/ros1_ws/src/controller/px4_multirotor_controller/tools \
PYTHONPYCACHEPREFIX="${PWD}/devel/.cache/px4_multirotor_controller/pycache" \
python3 -B -m nmpc.export_ros_generated_code
```

This writes only package-local UAV files under
`source/ros1_ws/src/controller/px4_multirotor_controller/generated/nmpc/uav_nmpc/`.
That directory is ignored by git and is compiled by `px4_multirotor_controller` as an
internal static library. UAV generated code does not use a shared generated-code
ROS package.

The generated C solver does not include Python-side runtime interactions. The
C++ wrapper must continue to mirror the Python update sequence:

1. set the fixed `x0` lower/upper bounds every cycle;
2. set `p=[xref(13); uref(4)]` for stages `0..N`;
3. seed `x/u` warm starts;
4. call solve and reject nonzero status;
5. read `u0` and predicted `x1`;
6. shift warm start with the next terminal reference;
7. map predicted `x1.omega` and `u0.T/m` to PX4 body-rate plus normalized thrust.
