#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

namespace px4_multirotor_controller {

/// @brief 二阶滑模有限时间控制器
/// @details 实现论文中的公式(26)(27)(28)
///          滑模变量：s_j = e_v_j + sqrt([e_v_j]^2 + k2*arctan(e_p_j))
///          控制律：u_f_j = -k1 * sat(s_j)
///          参考：Theorem 2, 有限时间收敛性证明见 Appendix C
class SlidingModeController {
   public:
    /// @brief 构造函数
    /// @param k1 控制增益（需满足稳定性条件）
    /// @param k2 滑模参数（影响收敛速度，k2 > 0）
    /// @param epsilon 边界层厚度（减小抖振，epsilon > 0）
    inline SlidingModeController(double k1, double k2, double epsilon)
        : k1_(k1), k2_(k2), epsilon_(epsilon) {}

    /// @brief 计算反馈控制量 u_f
    /// @param e_p 位置误差向量（期望位置 - 当前位置）
    /// @param e_v 速度误差向量（期望速度 - 当前速度）
    /// @return 反馈加速度向量 u_f（3维）
    inline Eigen::Vector3d computeFeedback(const Eigen::Vector3d& e_p,
                                           const Eigen::Vector3d& e_v) const {
        Eigen::Vector3d u_f;
        for (int j = 0; j < 3; ++j) {
            double s_j = computeSlidingVariable(e_p(j), e_v(j));
            u_f(j) = -k1_ * saturation(s_j);
        }
        return u_f;
    }

    /// @brief 更新控制器参数
    inline void setParams(double k1, double k2, double epsilon) {
        k1_ = k1;
        k2_ = k2;
        epsilon_ = epsilon;
    }

   private:
    /// @brief 饱和函数（公式28）
    /// @param s 滑模变量
    /// @return sat(s) = [s]^alpha if |s| > epsilon, else s/epsilon
    inline double saturation(double s) const {
        return (std::abs(s) > epsilon_) ? sign(s) : s / epsilon_;
    }

    /// @brief 计算单个分量的滑模变量（公式26）
    /// @param e_p_j 位置误差（单轴）
    /// @param e_v_j 速度误差（单轴）
    /// @return 滑模变量 s_j
    inline double computeSlidingVariable(double e_p_j, double e_v_j) const {
        return e_v_j + signBeta(signBeta(e_v_j, 2.0) + k2_ * std::atan(e_p_j), 0.5);
    }

    // 控制器参数
    double k1_;       // 控制增益
    double k2_;       // 滑模参数
    double epsilon_;  // 边界层厚度
};

}  // namespace px4_multirotor_controller
