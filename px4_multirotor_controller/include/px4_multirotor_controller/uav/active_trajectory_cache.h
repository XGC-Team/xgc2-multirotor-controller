#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <vector>
#include <xgc2_math/control.hpp>
#include <xgc2_math/trajectory.hpp>

#include "multirotor_reference_trajectory/ActivePolynomialReference.h"
#include "multirotor_reference_trajectory/AnalyticReference.h"
#include "multirotor_reference_trajectory/SampledReference.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

namespace px4_multirotor_controller {

struct UavReferencePoint {
    double t_from_start{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
    double yaw_accel{0.0};
    uint32_t flags{0U};
};

class ActiveTrajectoryCache {
   public:
    bool updateAnalytic(const multirotor_reference_trajectory::AnalyticReference& msg,
                        const ros::Time& received_time);
    bool updatePolynomial(const multirotor_reference_trajectory::ActivePolynomialReference& msg,
                          const ros::Time& received_time);
    bool updateSampled(const multirotor_reference_trajectory::SampledReference& msg,
                       const ros::Time& received_time);
    void clear();

    bool sample(const ros::Time& now, double timeout, UavReferencePoint& sample) const;
    bool sampleHorizon(const ros::Time& now, double stage_dt, int horizon_steps, double timeout,
                       double gravity,
                       std::vector<xgc2_math::control::Se3Reference>& references) const;

    uint64_t sequence() const;
    uint32_t trajectoryId() const;
    uint32_t revision() const;
    bool valid(const ros::Time& now, double timeout) const;
    bool finiteTimeRemaining(const ros::Time& now, double timeout, double& remaining) const;

   private:
    static bool finiteVector(const Eigen::Vector3d& value);
    static std::unique_ptr<xgc2_math::trajectory::TrajectoryEvaluator3> buildAnalyticEvaluator(
        const multirotor_reference_trajectory::AnalyticReference& msg, uint32_t& flags);
    static bool buildPolynomialEvaluator(
        const multirotor_reference_trajectory::ActivePolynomialReference& msg,
        xgc2_math::trajectory::PiecewisePolynomialEvaluator3& evaluator, uint32_t& flags);
    static bool buildSampledEvaluator(const multirotor_reference_trajectory::SampledReference& msg,
                                      xgc2_math::trajectory::SampledEvaluator3& evaluator,
                                      uint32_t& flags);
    static UavReferencePoint toPoint(const xgc2_math::trajectory::FlatOutput3& flat, double t);
    static xgc2_math::control::Se3Reference toNmpcReference(
        const xgc2_math::trajectory::FullStateReference3& full);

    mutable std::mutex mutex_;
    std::shared_ptr<const xgc2_math::trajectory::TrajectoryEvaluator3> evaluator_;
    xgc2_math::trajectory::TrajectoryModelType type_{
        xgc2_math::trajectory::TrajectoryModelType::kNone};
    uint32_t trajectory_id_{0U};
    uint32_t revision_{0U};
    uint64_t sequence_{0U};
    ros::Time start_time_;
    ros::Time received_time_;
    uint32_t flags_{0U};
};

}  // namespace px4_multirotor_controller
