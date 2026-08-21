from __future__ import annotations

import unittest

import numpy as np

from nmpc.controller import (
    AcadosNMPCController,
    Bounds,
    CostWeights,
    MPCConfig,
    optional_backend_status,
    projected_heading_error,
    thrust_direction_error,
)
from nmpc.dynamics import default_quadrotor_params, hover_state
from nmpc.math_utils import (
    STATE_SIZE,
    pack_state,
    quat_to_rotation,
    rk4_step_time,
    rotation_from_body_z,
    so3_log_error,
    to_px4_bodyrate_thrust,
    unpack_state_quat,
)
from nmpc.references import HoverTrajectory
from nmpc.runner import SimulationConfig, validate_config


class CoreMathTests(unittest.TestCase):
    def test_state_is_quaternion_based(self) -> None:
        state = pack_state(np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3))
        self.assertEqual(state.size, STATE_SIZE)
        self.assertEqual(STATE_SIZE, 14)
        _, _, quat, _ = unpack_state_quat(state)
        self.assertAlmostEqual(float(np.linalg.norm(quat)), 1.0)
        np.testing.assert_allclose(quat_to_rotation(quat), np.eye(3), atol=1e-12)

    def test_so3_log_error_is_zero_at_same_attitude(self) -> None:
        err = so3_log_error(np.eye(3), np.eye(3))
        np.testing.assert_allclose(err, np.zeros(3), atol=1e-12)

    def test_time_aware_rk4_advances_substep_time(self) -> None:
        out = rk4_step_time(lambda time, state: np.array([time]), 0.0, np.array([0.0]), 1.0)
        np.testing.assert_allclose(out, np.array([0.5]), atol=1e-12)

    def test_px4_bridge_maps_specific_thrust_and_body_rate(self) -> None:
        params = default_quadrotor_params()
        current_state = hover_state(np.array([0.0, 0.0, 1.0]))
        current_state[10:13] = np.array([0.4, -0.3, 0.2])
        command = np.array([params.g, 0.8, -0.4, 0.1])
        thrust_norm, body_rate = to_px4_bodyrate_thrust(
            current_state,
            command,
            predicted_body_rate=None,
            control_interval=0.01,
            hover_specific_thrust=params.g,
            hover_thrust_norm=0.5,
        )
        self.assertAlmostEqual(thrust_norm, 0.5)
        np.testing.assert_allclose(body_rate, command[1:4], atol=1e-12)

    def test_thrust_direction_error_ignores_pure_yaw(self) -> None:
        yaw = 0.7
        rotation = rotation_from_body_z(np.array([0.0, 0.0, 1.0]), yaw=yaw)

        np.testing.assert_allclose(
            thrust_direction_error(rotation, np.eye(3)),
            np.zeros(3),
            atol=1e-12,
        )
        self.assertAlmostEqual(projected_heading_error(rotation, np.eye(3)), np.sin(yaw))

    def test_thrust_direction_error_detects_tilt(self) -> None:
        tilt = 0.2
        body_z = np.array([np.sin(tilt), 0.0, np.cos(tilt)])
        rotation = rotation_from_body_z(body_z, yaw=0.0)

        error = thrust_direction_error(rotation, np.eye(3))

        self.assertGreater(np.linalg.norm(error), 0.1)
        np.testing.assert_allclose(error, np.array([0.0, np.sin(tilt), 0.0]), atol=1e-12)


class ControllerTests(unittest.TestCase):
    def require_acados(self) -> None:
        status = optional_backend_status()
        if not (status["casadi"] and status["acados_template"]):
            self.skipTest("casadi/acados_template not available")

    def test_bounds_match_runtime_product_defaults(self) -> None:
        bounds = Bounds()
        self.assertAlmostEqual(float(bounds.u_min[0]), 5.0)
        self.assertAlmostEqual(float(bounds.u_max[0]), 20.373)
        np.testing.assert_allclose(
            bounds.u_min[1:4],
            np.array([-3.4906585, -3.4906585, -0.8726646]),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            bounds.u_max[1:4],
            np.array([3.4906585, 3.4906585, 0.8726646]),
            atol=1e-12,
        )
        np.testing.assert_allclose(bounds.alpha_min, np.array([-15.0, -15.0, -2.0]))
        np.testing.assert_allclose(bounds.alpha_max, np.array([15.0, 15.0, 2.0]))

    def test_cost_weights_match_nullspace_yaw_baseline(self) -> None:
        weights = CostWeights()

        np.testing.assert_allclose(weights.position, 120.0 * np.ones(3), atol=1e-12)
        np.testing.assert_allclose(weights.velocity, 30.0 * np.ones(3), atol=1e-12)
        np.testing.assert_allclose(
            weights.thrust_direction,
            40.0 * np.ones(3),
            atol=1e-12,
        )
        self.assertAlmostEqual(weights.yaw, 1.0)
        np.testing.assert_allclose(weights.omega, 0.5 * np.ones(3), atol=1e-12)
        self.assertAlmostEqual(weights.thrust_actual, 2.0)
        self.assertAlmostEqual(weights.thrust_command_delta, 0.35)
        np.testing.assert_allclose(
            weights.body_rate_command_delta,
            np.array([8.0, 8.0, 16.0]),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            weights.angular_acceleration,
            np.array([0.04, 0.04, 2.25]),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            weights.control,
            np.array([0.08, 0.80, 0.80, 2.00]),
            atol=1e-12,
        )

    def test_ocp_cost_dimensions_use_initial_command_delta(self) -> None:
        self.require_acados()
        params = default_quadrotor_params()
        traj = HoverTrajectory(params=params)
        ctrl = AcadosNMPCController(
            traj,
            params=params,
            config=MPCConfig(horizon=0.2, steps=2, max_iter=3, control_interval=0.1),
            build_solver=False,
        )

        ocp, _ = ctrl._build_ocp()

        self.assertEqual(int(ocp.parameter_values.size), STATE_SIZE + 11)
        self.assertEqual(ocp.cost.W_0.shape, (26, 26))
        self.assertEqual(ocp.cost.W.shape, (22, 22))
        self.assertEqual(ocp.cost.W_e.shape, (15, 15))
        self.assertEqual(ocp.constraints.lh_0.shape, (4,))
        self.assertEqual(ocp.constraints.lh.shape, (4,))
        np.testing.assert_allclose(ocp.constraints.lh_0[1:4], np.array([-15.0, -15.0, -2.0]))
        np.testing.assert_allclose(ocp.constraints.uh_0[1:4], np.array([15.0, 15.0, 2.0]))
        np.testing.assert_allclose(ocp.constraints.lh[1:4], np.array([-15.0, -15.0, -2.0]))
        np.testing.assert_allclose(ocp.constraints.uh[1:4], np.array([15.0, 15.0, 2.0]))
        np.testing.assert_allclose(np.diag(ocp.cost.W_0)[18:21], np.ones(3), atol=1e-12)
        self.assertAlmostEqual(float(ocp.cost.W_0[21, 21]), 0.35)
        self.assertAlmostEqual(float(ocp.cost.W_0[22, 22]), 8.0)
        self.assertAlmostEqual(float(ocp.cost.W_0[23, 23]), 8.0)
        self.assertAlmostEqual(float(ocp.cost.W_0[24, 24]), 16.0)
        self.assertAlmostEqual(float(ocp.cost.W_0[25, 25]), 10.0)
        np.testing.assert_allclose(np.diag(ocp.cost.W)[18:21], np.ones(3), atol=1e-12)
        self.assertAlmostEqual(float(ocp.cost.W[21, 21]), 10.0)

    def test_stage_cost_uses_zero_reference_equivalent_angular_acceleration(self) -> None:
        self.require_acados()
        import casadi as ca

        params = default_quadrotor_params()
        traj = HoverTrajectory(params=params)
        ctrl = AcadosNMPCController(
            traj,
            params=params,
            config=MPCConfig(horizon=0.2, steps=2, max_iter=3, control_interval=0.1),
            build_solver=False,
        )
        x = ca.SX.sym("x", STATE_SIZE)
        u = ca.SX.sym("u", 4)
        p = ca.SX.sym("p", STATE_SIZE + 11)
        residual, _ = ctrl._casadi_residual(ca, x, u, p, terminal=False)
        evaluate = ca.Function("evaluate_alpha_residual", [x, u, p], [residual])

        state = hover_state(np.array([0.0, 0.0, 1.0]))
        command = np.array([params.g, 0.8, -0.4, 0.1])
        parameter = np.zeros(STATE_SIZE + 11)
        parameter[:STATE_SIZE] = state
        parameter[STATE_SIZE : STATE_SIZE + 4] = command
        parameter[STATE_SIZE + 8 : STATE_SIZE + 11] = np.array([0.2, 0.2, 1.5])

        value = np.asarray(evaluate(state, command, parameter)).reshape(-1)
        expected_alpha = command[1:4] / params.body_rate_time_constant
        np.testing.assert_allclose(
            value[18:21],
            parameter[STATE_SIZE + 8 : STATE_SIZE + 11] * expected_alpha,
            atol=1e-12,
        )

    def test_exact_hover_equilibrium(self) -> None:
        self.require_acados()
        params = default_quadrotor_params()
        traj = HoverTrajectory(target_pos=(1.0, 1.0, 1.0), params=params)
        ctrl = AcadosNMPCController(
            traj,
            params=params,
            config=MPCConfig(horizon=0.2, steps=2, max_iter=3, control_interval=0.1),
        )
        command, info = ctrl.update(0.0, hover_state(np.array([1.0, 1.0, 1.0])))
        self.assertEqual(info.status, 0)
        self.assertTrue(info.optimizer_success)
        np.testing.assert_allclose(command, np.array([params.g, 0.0, 0.0, 0.0]), atol=1e-6)

    def test_acados_backend_available_from_helper(self) -> None:
        status = optional_backend_status()
        if not (status["casadi"] and status["acados_template"]):
            self.skipTest("casadi/acados_template not available")
        self.assertTrue(status["acados_root_exists"])
        self.assertTrue(status["acados_template_source_exists"])
        self.assertTrue(status["casadi"])
        self.assertTrue(status["acados_template"])

    def test_acados_warm_start_stores_solution_guess(self) -> None:
        self.require_acados()
        params = default_quadrotor_params()
        traj = HoverTrajectory(params=params)
        ctrl = AcadosNMPCController(
            traj,
            params=params,
            config=MPCConfig(horizon=0.2, steps=2, max_iter=3, control_interval=0.1),
        )
        _, info = ctrl.update(0.0, hover_state(np.array([1.0, 1.0, 1.0])))
        self.assertTrue(info.accepted)
        self.assertEqual(ctrl.u_guess.shape, (2, 4))

    def test_warm_start_allows_faster_control_interval_than_prediction_step(self) -> None:
        self.require_acados()
        params = default_quadrotor_params()
        traj = HoverTrajectory(params=params)
        ctrl = AcadosNMPCController(
            traj,
            params=params,
            config=MPCConfig(horizon=1.0, steps=10, control_interval=0.01),
            build_solver=False,
        )
        self.assertAlmostEqual(ctrl.config.dt, 0.1)
        self.assertAlmostEqual(ctrl.config.control_interval, 0.01)

    def test_warm_start_requires_control_interval(self) -> None:
        self.require_acados()
        params = default_quadrotor_params()
        traj = HoverTrajectory(params=params)
        with self.assertRaises(ValueError):
            AcadosNMPCController(
                traj,
                params=params,
                config=MPCConfig(horizon=0.2, steps=2, warm_start=True),
                build_solver=False,
            )

    def test_nonzero_acados_status_rejects_finite_solution(self) -> None:
        self.require_acados()
        params = default_quadrotor_params()
        traj = HoverTrajectory(params=params)
        ctrl = AcadosNMPCController(
            traj,
            params=params,
            config=MPCConfig(horizon=0.2, steps=2, max_iter=3, control_interval=0.1),
            build_solver=False,
        )
        previous = np.array([params.g, 0.1, -0.1, 0.0])
        ctrl.last_u = previous.copy()
        ctrl.ocp_solver = _FiniteFailingSolver(params.g)

        command, info = ctrl.update(0.0, hover_state(np.array([1.0, 1.0, 1.0])))

        self.assertEqual(info.status, 2)
        self.assertFalse(info.accepted)
        self.assertFalse(info.optimizer_success)
        self.assertTrue(info.finite_solution)
        self.assertAlmostEqual(info.objective, 123.0)
        self.assertEqual(info.iterations, 1)
        self.assertEqual(info.qp_iter, 3)
        self.assertAlmostEqual(info.res_stat, 1e-4)
        self.assertAlmostEqual(info.res_eq, 2e-4)
        self.assertAlmostEqual(info.res_ineq, 3e-4)
        self.assertAlmostEqual(info.res_comp, 4e-4)
        np.testing.assert_allclose(command, previous)

    def test_update_passes_previous_command_as_parameter(self) -> None:
        self.require_acados()
        params = default_quadrotor_params()
        traj = HoverTrajectory(params=params)
        ctrl = AcadosNMPCController(
            traj,
            params=params,
            config=MPCConfig(horizon=0.2, steps=2, max_iter=3, control_interval=0.1),
            build_solver=False,
        )
        previous = np.array([params.g + 1.25, 0.1, -0.1, 0.0])
        ctrl.last_u = previous.copy()
        fake_solver = _FiniteFailingSolver(params.g)
        ctrl.ocp_solver = fake_solver

        ctrl.update(0.0, hover_state(np.array([1.0, 1.0, 1.0])))

        self.assertEqual(set(fake_solver.parameters.keys()), {0, 1, 2})
        for parameter in fake_solver.parameters.values():
            self.assertEqual(parameter.size, STATE_SIZE + 11)
            self.assertAlmostEqual(float(parameter[STATE_SIZE + 4]), previous[0])
            np.testing.assert_allclose(parameter[STATE_SIZE + 5 : STATE_SIZE + 8], previous[1:4])
            np.testing.assert_allclose(
                parameter[STATE_SIZE + 8 : STATE_SIZE + 11],
                np.sqrt(np.array([0.04, 0.04, 2.25])),
                atol=1e-12,
            )


class RunnerConfigTests(unittest.TestCase):
    def test_dt_ctrl_must_be_integer_multiple_of_dt_sim(self) -> None:
        with self.assertRaises(ValueError):
            validate_config(SimulationConfig(dt_ctrl=0.07, dt_sim=0.02))

    def test_valid_config_passes(self) -> None:
        validate_config(SimulationConfig(dt_ctrl=0.1, dt_sim=0.02))


class _FiniteFailingSolver:
    def __init__(self, gravity: float) -> None:
        self.gravity = gravity
        self.parameters: dict[int, np.ndarray] = {}

    def constraints_set(self, *_args) -> None:
        return None

    def set(self, *args) -> None:
        if len(args) == 3 and args[1] == "p":
            self.parameters[int(args[0])] = np.asarray(args[2], dtype=float).copy()
        return None

    def solve(self) -> int:
        return 2

    def get(self, stage: int, field: str) -> np.ndarray:
        if field == "u":
            return np.array([self.gravity, 0.0, 0.0, 0.0])
        if field == "x":
            return hover_state(np.array([1.0, 1.0, 1.0]))
        raise KeyError((stage, field))

    def get_cost(self) -> float:
        return 123.0

    def get_stats(self, field: str) -> np.ndarray:
        if field == "sqp_iter":
            return np.array([1.0])
        if field == "qp_iter":
            return np.array([3.0])
        if field == "residuals":
            return np.array([1e-4, 2e-4, 3e-4, 4e-4])
        if field in {"time_qp", "time_lin"}:
            return np.array([1e-5])
        raise KeyError(field)


if __name__ == "__main__":
    unittest.main()
