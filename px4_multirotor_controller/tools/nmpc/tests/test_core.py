from __future__ import annotations

import unittest

import numpy as np

from nmpc.controller import AcadosNMPCController, MPCConfig, optional_backend_status
from nmpc.dynamics import default_quadrotor_params, hover_state
from nmpc.math_utils import (
    STATE_SIZE,
    pack_state,
    quat_to_rotation,
    rk4_step_time,
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
        self.assertEqual(STATE_SIZE, 13)
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
        predicted_state = hover_state(np.array([0.0, 0.0, 1.0]))
        predicted_state[10:13] = np.array([0.4, -0.3, 0.2])
        command = np.array([params.g, 1.0, -2.0, 0.5])
        thrust_norm, body_rate = to_px4_bodyrate_thrust(
            predicted_state,
            command,
            hover_specific_thrust=params.g,
            hover_thrust_norm=0.5,
        )
        self.assertAlmostEqual(thrust_norm, 0.5)
        np.testing.assert_allclose(body_rate, np.array([0.4, -0.3, 0.2]), atol=1e-12)


class ControllerTests(unittest.TestCase):
    def require_acados(self) -> None:
        status = optional_backend_status()
        if not (status["casadi"] and status["acados_template"]):
            self.skipTest("casadi/acados_template not available")

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


class RunnerConfigTests(unittest.TestCase):
    def test_dt_ctrl_must_be_integer_multiple_of_dt_sim(self) -> None:
        with self.assertRaises(ValueError):
            validate_config(SimulationConfig(dt_ctrl=0.07, dt_sim=0.02))

    def test_valid_config_passes(self) -> None:
        validate_config(SimulationConfig(dt_ctrl=0.1, dt_sim=0.02))


class _FiniteFailingSolver:
    def __init__(self, gravity: float) -> None:
        self.gravity = gravity

    def constraints_set(self, *_args) -> None:
        return None

    def set(self, *_args) -> None:
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
