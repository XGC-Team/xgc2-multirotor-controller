#pragma once

#include <ros/ros.h>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

struct TrajectoryLifterConfig {
    double planning_period{0.1};  // MPC 规划周期，默认 10Hz = 0.1s
};

/// @brief 将 MPC 第一拍离散解提升为连续时间参考
/// @details 对应论文中的 trajectory lifter：
///          - 输入：k 时刻离散位置/速度偏移与控制输入 tau_i(k)
///          - 输出：连续时间位置、速度、加速度参考
///          - 模型：匀加速度解析解 + 控制输入零阶保持
class TrajectoryLifter {
   public:
    explicit inline TrajectoryLifter(
        const TrajectoryLifterConfig& config = TrajectoryLifterConfig{})
        : config_(config) {}

    inline Setpoint lift(const MpcTrajectoryState& mpc_traj, const ros::Time& current_time) const {
        Setpoint sp;
        if (!mpc_traj.is_valid) {
            return sp;
        }

        double dt = (current_time - mpc_traj.planning_time).toSec();
        if (dt < 0.0) {
            ROS_WARN_THROTTLE(1.0, "[TrajectoryLifter] Negative time offset: %.3f s, clock issue?",
                              dt);
            dt = 0.0;
        }
        if (dt > config_.planning_period) {
            dt = config_.planning_period;
        }

        // Constant-acceleration closed-form reconstruction.
        const Eigen::Vector3d pos_t = mpc_traj.position_k + mpc_traj.velocity_k * dt +
                                      0.5 * mpc_traj.acceleration_k * dt * dt;
        const Eigen::Vector3d vel_t = mpc_traj.velocity_k + mpc_traj.acceleration_k * dt;
        const Eigen::Vector3d acc_t = mpc_traj.acceleration_k;

        sp.x = pos_t.x();
        sp.y = pos_t.y();
        sp.z = pos_t.z();

        sp.vx = vel_t.x();
        sp.vy = vel_t.y();
        sp.vz = vel_t.z();

        sp.ax = acc_t.x();
        sp.ay = acc_t.y();
        sp.az = acc_t.z();

        sp.qx = mpc_traj.qx;
        sp.qy = mpc_traj.qy;
        sp.qz = mpc_traj.qz;
        sp.qw = mpc_traj.qw;
        sp.yaw_rate = mpc_traj.yaw_rate;
        sp.type_mask = mpc_traj.type_mask;
        sp.coordinate_frame = mpc_traj.coordinate_frame;
        return sp;
    }

    inline const TrajectoryLifterConfig& config() const {
        return config_;
    }

   private:
    TrajectoryLifterConfig config_;
};

}  // namespace px4_multirotor_controller
