#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace px4_multirotor_controller {

using Vector13d = Eigen::Matrix<double, 13, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;

inline double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

inline double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

inline double quaternionToYaw(double qx, double qy, double qz, double qw) {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

inline double sign(double value) {
    if (value > 0.0) {
        return 1.0;
    }
    if (value < 0.0) {
        return -1.0;
    }
    return 0.0;
}

inline double signBeta(double value, double beta) {
    return sign(value) * std::pow(std::abs(value), beta);
}

inline Eigen::Quaterniond yawToQuaternion(double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline Eigen::Vector3d normalizeVector(const Eigen::Vector3d& value,
                                       const Eigen::Vector3d& fallback) {
    const double norm = value.norm();
    if (!std::isfinite(norm) || norm < 1e-9) {
        return fallback;
    }
    return value / norm;
}

inline Eigen::Matrix3d rotationFromBodyZ(const Eigen::Vector3d& body_z, double yaw = 0.0) {
    const Eigen::Vector3d z_b = normalizeVector(body_z, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d x_c(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d y_b = z_b.cross(x_c);
    if (y_b.norm() < 1e-8) {
        x_c = Eigen::Vector3d::UnitY();
        y_b = z_b.cross(x_c);
    }
    y_b = normalizeVector(y_b, Eigen::Vector3d::UnitY());
    const Eigen::Vector3d x_b = normalizeVector(y_b.cross(z_b), Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotation;
    rotation.col(0) = x_b;
    rotation.col(1) = y_b;
    rotation.col(2) = z_b;
    return rotation;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d& skew) {
    return Eigen::Vector3d(skew(2, 1), skew(0, 2), skew(1, 0));
}

inline Eigen::Matrix<double, 9, 1> matToVecColumnMajor(const Eigen::Matrix3d& mat) {
    Eigen::Matrix<double, 9, 1> vec;
    vec = Eigen::Map<const Eigen::Matrix<double, 9, 1>>(mat.data());
    return vec;
}

inline Eigen::Matrix3d vecToMatColumnMajor(const Eigen::Matrix<double, 9, 1>& vec) {
    return Eigen::Map<const Eigen::Matrix3d>(vec.data());
}

inline Eigen::Matrix<double, 4, 1> quatToVecWxyz(const Eigen::Quaterniond& quat_in) {
    Eigen::Quaterniond quat = quat_in;
    if (!std::isfinite(quat.norm()) || quat.norm() < 1e-9) {
        quat = Eigen::Quaterniond::Identity();
    }
    quat.normalize();
    if (quat.w() < 0.0) {
        quat.coeffs() *= -1.0;
    }
    Eigen::Matrix<double, 4, 1> vec;
    vec << quat.w(), quat.x(), quat.y(), quat.z();
    return vec;
}

inline Eigen::Quaterniond vecWxyzToQuat(const Eigen::Matrix<double, 4, 1>& vec) {
    Eigen::Quaterniond quat(vec(0), vec(1), vec(2), vec(3));
    if (!std::isfinite(quat.norm()) || quat.norm() < 1e-9) {
        quat = Eigen::Quaterniond::Identity();
    }
    quat.normalize();
    if (quat.w() < 0.0) {
        quat.coeffs() *= -1.0;
    }
    return quat;
}

inline Eigen::Matrix3d projectRotation(const Eigen::Matrix3d& value) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(value, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d projected = svd.matrixU() * svd.matrixV().transpose();
    if (projected.determinant() < 0.0) {
        Eigen::Matrix3d u = svd.matrixU();
        u.col(2) *= -1.0;
        projected = u * svd.matrixV().transpose();
    }
    return projected;
}

inline bool isFinite(const Eigen::VectorXd& value) {
    return value.array().isFinite().all();
}

}  // namespace px4_multirotor_controller
