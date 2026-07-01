#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

#include <ros/console.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace px4_multirotor_controller {

UavNmpcSolver::UavNmpcSolver() = default;

UavNmpcSolver::~UavNmpcSolver() {
    cleanup();
}

bool UavNmpcSolver::initialize() {
    if (initialized_) {
        return true;
    }
    if (UAV_NMPC_NX != 14 || UAV_NMPC_NU != 4 || UAV_NMPC_NP != 19 || UAV_NMPC_N <= 0 ||
        UAV_NMPC_NY0 != 20 || UAV_NMPC_NY != 19 || UAV_NMPC_NYN != 15 || UAV_NMPC_NH != 1 ||
        UAV_NMPC_NHN != 1 || UAV_NMPC_NSBU != 4 || UAV_NMPC_NSBX != 9 || UAV_NMPC_NSH != 1 ||
        UAV_NMPC_NS != 14 || UAV_NMPC_NS0 != 4 || UAV_NMPC_NSBXN != 9 || UAV_NMPC_NSHN != 1 ||
        UAV_NMPC_NSN != 10) {
        ROS_ERROR("[UavNmpcSolver] Unexpected generated solver dimensions");
        return false;
    }

    capsule_ = uav_nmpc_acados_create_capsule();
    if (!capsule_) {
        ROS_ERROR("[UavNmpcSolver] Failed to create acados capsule");
        return false;
    }

    const int status = uav_nmpc_acados_create(capsule_);
    if (status != 0) {
        ROS_ERROR("[UavNmpcSolver] uav_nmpc_acados_create failed: %d", status);
        cleanup();
        return false;
    }
    initialized_ = true;
    if (!applyInputBounds()) {
        cleanup();
        return false;
    }

    resetWarmStart();
    ROS_INFO("[UavNmpcSolver] Initialized generated acados solver (N=%d, nx=%d, nu=%d)", UAV_NMPC_N,
             UAV_NMPC_NX, UAV_NMPC_NU);
    return true;
}

bool UavNmpcSolver::configureInputBounds(double specific_thrust_min, double specific_thrust_max,
                                         double max_roll_pitch_angular_acceleration,
                                         double max_yaw_angular_acceleration) {
    if (!std::isfinite(specific_thrust_min) || !std::isfinite(specific_thrust_max) ||
        specific_thrust_min < 0.0 || specific_thrust_max <= specific_thrust_min) {
        ROS_ERROR("[UavNmpcSolver] Invalid input thrust bounds [%.3f, %.3f]", specific_thrust_min,
                  specific_thrust_max);
        return false;
    }
    if (!std::isfinite(max_roll_pitch_angular_acceleration) ||
        max_roll_pitch_angular_acceleration <= 0.0 ||
        !std::isfinite(max_yaw_angular_acceleration) || max_yaw_angular_acceleration <= 0.0) {
        ROS_ERROR("[UavNmpcSolver] Invalid angular acceleration bounds roll_pitch=%.3f yaw=%.3f",
                  max_roll_pitch_angular_acceleration, max_yaw_angular_acceleration);
        return false;
    }

    input_lower_bounds_[0] = specific_thrust_min;
    input_upper_bounds_[0] = specific_thrust_max;
    input_lower_bounds_[1] = -max_roll_pitch_angular_acceleration;
    input_upper_bounds_[1] = max_roll_pitch_angular_acceleration;
    input_lower_bounds_[2] = -max_roll_pitch_angular_acceleration;
    input_upper_bounds_[2] = max_roll_pitch_angular_acceleration;
    input_lower_bounds_[3] = -max_yaw_angular_acceleration;
    input_upper_bounds_[3] = max_yaw_angular_acceleration;
    if (capsule_) {
        return applyInputBounds();
    }
    return true;
}

void UavNmpcSolver::resetWarmStart() {
    have_warm_start_ = false;
    for (auto& x : x_guess_) {
        x.setZero();
        x(6) = 1.0;
        x(13) = 9.8066;
    }
    Se3ControlVector nominal_u = Se3ControlVector::Zero();
    nominal_u(0) = 9.8066;
    nominal_u = clampInputGuess(nominal_u);
    for (auto& u : u_guess_) {
        u = nominal_u;
    }
}

bool UavNmpcSolver::solve(const Se3StateVector& x0, double thrust_actual,
                          double last_commanded_specific_thrust,
                          const std::vector<Se3Reference>& references) {
    if (!initialized_ && !initialize()) {
        return false;
    }
    if (references.size() != static_cast<size_t>(UAV_NMPC_N + 2)) {
        ROS_ERROR("[UavNmpcSolver] Expected %d references, got %zu", UAV_NMPC_N + 2,
                  references.size());
        return false;
    }
    if (!control::isFinite(x0) || !std::isfinite(thrust_actual) ||
        !std::isfinite(last_commanded_specific_thrust)) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcSolver] Non-finite initial state");
        return false;
    }

    const UavNmpcStateVector x0_internal = packInternalState(x0, thrust_actual);
    if (!setInitialState(x0_internal)) {
        return false;
    }

    for (int i = 0; i <= UAV_NMPC_N; ++i) {
        if (!setReference(i, references[static_cast<size_t>(i)], last_commanded_specific_thrust)) {
            return false;
        }
    }
    setGuesses(x0_internal, references);

    const auto t0 = std::chrono::steady_clock::now();
    solver_status_ = uav_nmpc_acados_solve(capsule_);
    const auto t1 = std::chrono::steady_clock::now();
    solve_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (solver_status_ != 0) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcSolver] Solve failed with status %d", solver_status_);
        return false;
    }

    readSolution();
    shiftWarmStart(references);
    return true;
}

bool UavNmpcSolver::applyInputBounds() {
    if (!capsule_) {
        return false;
    }

    ocp_nlp_config* config = uav_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = uav_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* in = uav_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out* out = uav_nmpc_acados_get_nlp_out(capsule_);

    int status = 0;
    for (int i = 0; i < UAV_NMPC_N; ++i) {
        status |= ocp_nlp_constraints_model_set(config, dims, in, out, i, "lbu",
                                                input_lower_bounds_.data());
        status |= ocp_nlp_constraints_model_set(config, dims, in, out, i, "ubu",
                                                input_upper_bounds_.data());
    }
    if (status != 0) {
        ROS_ERROR("[UavNmpcSolver] Failed to apply input bounds");
        return false;
    }
    return true;
}

bool UavNmpcSolver::setInitialState(const UavNmpcStateVector& x0) {
    ocp_nlp_config* config = uav_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = uav_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* in = uav_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out* out = uav_nmpc_acados_get_nlp_out(capsule_);
    int status = ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx",
                                               const_cast<double*>(x0.data()));
    status |= ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx",
                                            const_cast<double*>(x0.data()));
    if (status != 0) {
        ROS_ERROR("[UavNmpcSolver] Failed to set x0 constraint");
        return false;
    }
    return true;
}

bool UavNmpcSolver::setReference(int stage, const Se3Reference& reference,
                                 double last_commanded_specific_thrust) {
    Eigen::Matrix<double, UAV_NMPC_NP, 1> p;
    p.setZero();
    p.segment<UAV_NMPC_NX>(0) = packInternalReference(reference);
    p.segment<4>(UAV_NMPC_NX) = control::packControl(reference.control);
    p(UAV_NMPC_NX + 4) = last_commanded_specific_thrust;
    const int status = uav_nmpc_acados_update_params(capsule_, stage, p.data(), UAV_NMPC_NP);
    if (status != 0) {
        ROS_ERROR("[UavNmpcSolver] Failed to update params at stage %d", stage);
        return false;
    }
    return true;
}

void UavNmpcSolver::setGuesses(const UavNmpcStateVector& x0,
                               const std::vector<Se3Reference>& references) {
    ocp_nlp_config* config = uav_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = uav_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* in = uav_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out* out = uav_nmpc_acados_get_nlp_out(capsule_);

    if (!have_warm_start_) {
        for (int i = 0; i <= UAV_NMPC_N; ++i) {
            x_guess_[static_cast<size_t>(i)] =
                packInternalReference(references[static_cast<size_t>(i)]);
        }
        for (int i = 0; i < UAV_NMPC_N; ++i) {
            u_guess_[static_cast<size_t>(i)] =
                clampInputGuess(control::packControl(references[static_cast<size_t>(i)].control));
        }
    }
    x_guess_[0] = x0;

    for (int i = 0; i <= UAV_NMPC_N; ++i) {
        ocp_nlp_out_set(config, dims, out, in, i, "x", x_guess_[static_cast<size_t>(i)].data());
        if (i < UAV_NMPC_N) {
            ocp_nlp_out_set(config, dims, out, in, i, "u", u_guess_[static_cast<size_t>(i)].data());
        }
    }
}

void UavNmpcSolver::readSolution() {
    ocp_nlp_config* config = uav_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = uav_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out* out = uav_nmpc_acados_get_nlp_out(capsule_);

    max_quaternion_norm_error_ = 0.0;
    for (int i = 0; i <= UAV_NMPC_N; ++i) {
        ocp_nlp_out_get(config, dims, out, i, "x", x_solution_[static_cast<size_t>(i)].data());
        const auto quat = x_solution_[static_cast<size_t>(i)].segment<4>(6);
        max_quaternion_norm_error_ =
            std::max(max_quaternion_norm_error_, std::abs(quat.squaredNorm() - 1.0));
        predicted_states_[static_cast<size_t>(i)] =
            projectSe3State(x_solution_[static_cast<size_t>(i)]);
        if (i < UAV_NMPC_N) {
            ocp_nlp_out_get(config, dims, out, i, "u", u_solution_[static_cast<size_t>(i)].data());
        }
    }
    optimal_control_ = u_solution_[0];
    predicted_body_rate_ = x_solution_[1].segment<3>(10);
}

void UavNmpcSolver::shiftWarmStart(const std::vector<Se3Reference>& references) {
    (void)references;
    for (int i = 0; i <= UAV_NMPC_N; ++i) {
        x_guess_[static_cast<size_t>(i)] = x_solution_[static_cast<size_t>(i)];
    }

    for (int i = 0; i < UAV_NMPC_N; ++i) {
        u_guess_[static_cast<size_t>(i)] = u_solution_[static_cast<size_t>(i)];
    }
    have_warm_start_ = true;
}

void UavNmpcSolver::cleanup() {
    if (capsule_) {
        if (initialized_) {
            uav_nmpc_acados_free(capsule_);
        }
        uav_nmpc_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
    initialized_ = false;
}

UavNmpcStateVector UavNmpcSolver::packInternalState(const Se3StateVector& state,
                                                    double thrust_actual) const {
    UavNmpcStateVector value = UavNmpcStateVector::Zero();
    value.segment<13>(0) = state;
    value(13) = thrust_actual;
    return value;
}

UavNmpcStateVector UavNmpcSolver::packInternalReference(const Se3Reference& reference) const {
    return packInternalState(control::packState(reference.state),
                             reference.control.body_z_specific_force);
}

Se3StateVector UavNmpcSolver::projectSe3State(const UavNmpcStateVector& state) const {
    Se3StateVector value = Se3StateVector::Zero();
    value = state.segment<13>(0);
    return value;
}

Se3ControlVector UavNmpcSolver::clampInputGuess(const Se3ControlVector& input) const {
    Se3ControlVector value = input;
    for (int i = 0; i < UAV_NMPC_NU; ++i) {
        if (!std::isfinite(value(i))) {
            value(i) = 0.0;
        }
        value(i) = std::clamp(value(i), input_lower_bounds_[static_cast<size_t>(i)],
                              input_upper_bounds_[static_cast<size_t>(i)]);
    }
    return value;
}

}  // namespace px4_multirotor_controller
