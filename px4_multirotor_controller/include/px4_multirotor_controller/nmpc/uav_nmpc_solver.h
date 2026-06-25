#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <xgc2_math/control.hpp>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_uav_nmpc.h"
}

namespace px4_multirotor_controller {

namespace control = xgc2_math::control;
using Se3ControlVector = control::Se3ControlVector;
using Se3Reference = control::Se3Reference;
using Se3StateVector = control::Se3StateVector;

class UavNmpcSolver {
   public:
    UavNmpcSolver();
    ~UavNmpcSolver();

    UavNmpcSolver(const UavNmpcSolver&) = delete;
    UavNmpcSolver& operator=(const UavNmpcSolver&) = delete;

    bool initialize();
    void resetWarmStart();
    bool solve(const Se3StateVector& x0, const std::vector<Se3Reference>& references);

    Se3ControlVector optimalControl() const {
        return optimal_control_;
    }
    Eigen::Vector3d predictedBodyRate() const {
        return predicted_body_rate_;
    }
    int status() const {
        return solver_status_;
    }
    double solveTimeMs() const {
        return solve_time_ms_;
    }
    double maxQuaternionNormError() const {
        return max_quaternion_norm_error_;
    }
    bool initialized() const {
        return initialized_;
    }

    static constexpr int nx() {
        return UAV_NMPC_NX;
    }
    static constexpr int nu() {
        return UAV_NMPC_NU;
    }
    static constexpr int np() {
        return UAV_NMPC_NP;
    }
    static constexpr int horizonSteps() {
        return UAV_NMPC_N;
    }

   private:
    bool setInitialState(const Se3StateVector& x0);
    bool setReference(int stage, const Se3Reference& reference);
    void setGuesses(const Se3StateVector& x0, const std::vector<Se3Reference>& references);
    void readSolution();
    void shiftWarmStart(const std::vector<Se3Reference>& references);
    void cleanup();

    uav_nmpc_solver_capsule* capsule_{nullptr};
    bool initialized_{false};
    bool have_warm_start_{false};
    int solver_status_{-1};
    double solve_time_ms_{0.0};
    double max_quaternion_norm_error_{0.0};

    std::array<Se3StateVector, UAV_NMPC_N + 1> x_guess_{};
    std::array<Se3ControlVector, UAV_NMPC_N> u_guess_{};
    std::array<Se3StateVector, UAV_NMPC_N + 1> x_solution_{};
    std::array<Se3ControlVector, UAV_NMPC_N> u_solution_{};
    Se3ControlVector optimal_control_{Se3ControlVector::Zero()};
    Eigen::Vector3d predicted_body_rate_{Eigen::Vector3d::Zero()};
};

}  // namespace px4_multirotor_controller
