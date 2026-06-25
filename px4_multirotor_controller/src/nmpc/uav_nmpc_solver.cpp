#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

#include <ros/console.h>

#include <algorithm>
#include <chrono>

namespace px4_multirotor_controller {

UavNmpcSolver::UavNmpcSolver() = default;

UavNmpcSolver::~UavNmpcSolver() {
    cleanup();
}

bool UavNmpcSolver::initialize() {
    if (initialized_) {
        return true;
    }
    if (UAV_NMPC_NX != 13 || UAV_NMPC_NU != 4 || UAV_NMPC_NP != 17 || UAV_NMPC_N != 10 ||
        UAV_NMPC_NH != 1 || UAV_NMPC_NHN != 1 || UAV_NMPC_NSBU != 4 || UAV_NMPC_NSBX != 9 ||
        UAV_NMPC_NSH != 1 || UAV_NMPC_NS != 14 || UAV_NMPC_NS0 != 4 || UAV_NMPC_NSBXN != 9 ||
        UAV_NMPC_NSHN != 1 || UAV_NMPC_NSN != 10) {
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
    resetWarmStart();
    ROS_INFO("[UavNmpcSolver] Initialized generated acados solver (N=%d, nx=%d, nu=%d)", UAV_NMPC_N,
             UAV_NMPC_NX, UAV_NMPC_NU);
    return true;
}

void UavNmpcSolver::resetWarmStart() {
    have_warm_start_ = false;
    for (auto& x : x_guess_) {
        x.setZero();
        x(6) = 1.0;
    }
    for (auto& u : u_guess_) {
        u.setZero();
    }
}

bool UavNmpcSolver::solve(const Se3StateVector& x0, const std::vector<Se3Reference>& references) {
    if (!initialized_ && !initialize()) {
        return false;
    }
    if (references.size() != static_cast<size_t>(UAV_NMPC_N + 2)) {
        ROS_ERROR("[UavNmpcSolver] Expected %d references, got %zu", UAV_NMPC_N + 2,
                  references.size());
        return false;
    }
    if (!control::isFinite(x0)) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcSolver] Non-finite initial state");
        return false;
    }

    if (!setInitialState(x0)) {
        return false;
    }

    for (int i = 0; i <= UAV_NMPC_N; ++i) {
        if (!setReference(i, references[static_cast<size_t>(i)])) {
            return false;
        }
    }
    setGuesses(x0, references);

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

bool UavNmpcSolver::setInitialState(const Se3StateVector& x0) {
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

bool UavNmpcSolver::setReference(int stage, const Se3Reference& reference) {
    Eigen::Matrix<double, UAV_NMPC_NP, 1> p;
    p.segment<13>(0) = control::packState(reference.state);
    p.segment<4>(13) = control::packControl(reference.control);
    const int status = uav_nmpc_acados_update_params(capsule_, stage, p.data(), UAV_NMPC_NP);
    if (status != 0) {
        ROS_ERROR("[UavNmpcSolver] Failed to update params at stage %d", stage);
        return false;
    }
    return true;
}

void UavNmpcSolver::setGuesses(const Se3StateVector& x0,
                               const std::vector<Se3Reference>& references) {
    ocp_nlp_config* config = uav_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = uav_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* in = uav_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out* out = uav_nmpc_acados_get_nlp_out(capsule_);

    if (!have_warm_start_) {
        for (int i = 0; i <= UAV_NMPC_N; ++i) {
            x_guess_[static_cast<size_t>(i)] =
                control::packState(references[static_cast<size_t>(i)].state);
        }
        for (int i = 0; i < UAV_NMPC_N; ++i) {
            u_guess_[static_cast<size_t>(i)] =
                control::packControl(references[static_cast<size_t>(i)].control);
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

}  // namespace px4_multirotor_controller
