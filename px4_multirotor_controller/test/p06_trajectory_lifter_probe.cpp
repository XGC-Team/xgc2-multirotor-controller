// P06: exercise the owning headers, not a second lifter implementation.
// "characterization" cases expose current limitations; PASS does not endorse them.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>

#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/uav/mpc_trajectory_buffer.h"

namespace pc = px4_multirotor_controller;
using Values = std::array<double, 9>;
int checks = 0;
int failures = 0;
constexpr double tolerance = 1e-10;

Values values(const pc::Setpoint& p) {
    return {p.x, p.y, p.z, p.vx, p.vy, p.vz, p.ax, p.ay, p.az};
}
void print(const Values& a) {
    for (std::size_t i = 0; i < a.size(); ++i) std::cout << (i ? "," : "") << a[i];
}
bool equal(const Values& a, const Values& b) {
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i]) ||
            std::abs(a[i] - b[i]) > tolerance) return false;
    }
    return true;
}
void check(const std::string& name, bool ok) {
    ++checks;
    failures += !ok;
    std::cout << name << " result=" << (ok ? "PASS" : "FAIL") << '\n';
}
void record(const std::string& name, const Values& actual, const Values& expected) {
    std::cout << name << " actual=";
    print(actual);
    std::cout << " expected=";
    print(expected);
    std::cout << '\n';
    check(name, equal(actual, expected));
}
void record(const std::string& name, const pc::Setpoint& p, const Values& expected) {
    record(name, values(p), expected);
}
pc::MpcTrajectoryState sample(double p, double v, double a, double origin) {
    pc::MpcTrajectoryState s;
    s.position_k = Eigen::Vector3d(p, 0, 0);
    s.velocity_k = Eigen::Vector3d(v, 0, 0);
    s.acceleration_k = Eigen::Vector3d(a, 0, 0);
    s.planning_time = ros::Time(origin);
    s.type_mask = pc::kDefaultPvaLocalTypeMask;
    s.is_valid = true;
    return s;
}
pc::Setpoint lift(const pc::MpcTrajectoryState& s, double t,
                  uint16_t mask = pc::kDefaultPvaLocalTypeMask) {
    return pc::liftWorldLocal(s, ros::Time(t), mask, false);
}
Values difference(const pc::Setpoint& right, const pc::Setpoint& left) {
    auto r = values(right);
    const auto l = values(left);
    for (std::size_t i = 0; i < r.size(); ++i) r[i] -= l[i];
    return r;
}

int main() {
    ros::Time::init();
    std::cout << std::setprecision(17);
    std::cout << "P06 header probe; no ROS graph, controller state machine or flight test\n"
              << "values_order=px,py,pz,vx,vy,vz,ax,ay,az tolerance=" << tolerance << '\n';
    auto s = sample(1, 2, 3, 10);
    record("regression/pva", lift(s, 10.25), {1.59375,0,0,2.75,0,0,3,0,0});
    auto pv = s;
    pv.type_mask |= pc::kIgnoreAfxBit | pc::kIgnoreAfyBit | pc::kIgnoreAfzBit;
    record("regression/pv_ignored_acceleration", lift(pv, 10.25), {1.5,0,0,2,0,0,0,0,0});
    auto p = pv;
    p.position_k = Eigen::Vector3d(1,2,3);
    p.velocity_k = Eigen::Vector3d(4,5,6);
    p.type_mask |= pc::kIgnoreVxBit | pc::kIgnoreVyBit | pc::kIgnoreVzBit;
    record("regression/position_only_ignored_velocity", lift(p, 10.25), {1,2,3,0,0,0,0,0,0});
    auto mixed = s;
    mixed.type_mask |= pc::kIgnoreVxBit;
    record("regression/ignored_vx_active_ax", lift(mixed, 10.25), {1.09375,0,0,0,0,0,3,0,0});
    auto defaulted = p;
    defaulted.type_mask = 0;
    record("regression/default_mask", lift(defaulted, 10.25, p.type_mask), {1,2,3,0,0,0,0,0,0});
    record("regression/incoming_mask_overrides_default", lift(s,10.25,p.type_mask),
           {1.59375,0,0,2.75,0,0,3,0,0});
    auto sce = sample(9,.2,3,2);
    sce.position_k = Eigen::Vector3d(9,8,1.5);
    sce.velocity_k = Eigen::Vector3d(.2,-.1,0);
    sce.acceleration_k = Eigen::Vector3d(3,3,3);
    sce.type_mask = 3523;
    record("regression/sce_3523", lift(sce,2.2), {0,0,1.5,.2,-.1,0,0,0,0});
    check("regression/sce_mask_preserved", lift(sce,2.2).type_mask == 3523);
    auto ignored = s;
    ignored.type_mask |= 511;
    record("regression/all_pva_ignored", lift(ignored,10.25), {});
    check("regression/yaw_disabled", (lift(s,10.25).type_mask & 3072) == 3072);

    // Explicit counterexamples: these observations are not safety requirements.
    auto next = sample(.1,1,2,10.1);
    const auto old = sample(0,1,0,10);
    record("characterization/c1_not_c2_seam", difference(lift(next,10.1),lift(old,10.1)),
           {0,0,0,0,0,0,2,0,0});
    auto bad = next;
    bad.position_k.x() = .3;
    record("characterization/position_jump", difference(lift(bad,10.1),lift(old,10.1)),
           {.2,0,0,0,0,0,2,0,0});
    bad = next;
    bad.velocity_k.x() = 2;
    record("characterization/velocity_jump", difference(lift(bad,10.1),lift(old,10.1)),
           {0,0,0,1,0,0,2,0,0});
    record("characterization/late_effective_origin_seam", difference(lift(next,10.15),lift(old,10.15)),
           {.0025,0,0,.1,0,0,2,0,0});
    next.planning_time = ros::Time(10.15);
    record("characterization/late_receipt_origin_seam", difference(lift(next,10.15),lift(old,10.15)),
           {-.05,0,0,0,0,0,2,0,0});
    auto receipt = s;
    receipt.planning_time = ros::Time(10.2);
    record("characterization/receipt_not_effective_time", difference(lift(receipt,10.25),lift(s,10.25)),
           {-.49,0,0,-.6,0,0,0,0,0});
    record("characterization/long_age_no_expiry", lift(s,110), {15201,0,0,302,0,0,3,0,0});
    auto zero = s;
    zero.planning_time = ros::Time(0.0);
    record("characterization/zero_origin", lift(zero,10.25), {1,0,0,2,0,0,3,0,0});
    record("characterization/future_origin_clamped", lift(s,9.9), {1,0,0,2,0,0,3,0,0});
    record("characterization/clock_rollback_rewinds", difference(lift(s,9.9),lift(s,10.25)),
           {-.59375,0,0,-.75,0,0,0,0,0});

    // Inputs below model receipt-origin callbacks, not the ROS callback itself.
    pc::MpcTrajectoryBuffer buffer;
    check("regression/empty_buffer", !buffer.hasPending() && !buffer.promotePending(ros::Time(10.0)));
    buffer.cachePending(s);
    check("regression/promote", buffer.promotePending(buffer.pending().planning_time));
    const auto before_duplicate = lift(buffer.active(),10.25);
    receipt = s;
    receipt.planning_time = ros::Time(10.25);
    buffer.cachePending(receipt);
    buffer.promotePending(buffer.pending().planning_time);
    record("characterization/duplicate_receipt_rebases", difference(lift(buffer.active(),10.25),before_duplicate),
           {-.59375,0,0,-.75,0,0,0,0,0});
    buffer.cachePending(sample(2,0,0,11));
    buffer.cachePending(sample(1,0,0,11.1));
    buffer.promotePending(buffer.pending().planning_time);
    record("characterization/last_receipt_wins_not_sequence", lift(buffer.active(),11.1), {1,0,0,0,0,0,0,0,0});
    pc::TrajectoryLifter short_period(pc::TrajectoryLifterConfig{.001});
    pc::TrajectoryLifter long_period(pc::TrajectoryLifterConfig{100});
    record("characterization/period_is_not_horizon", difference(short_period.lift(s,ros::Time(110.0)),
           long_period.lift(s,ros::Time(110.0))), {});
    auto frame = s;
    frame.coordinate_frame = 8;
    check("characterization/body_frame_not_rejected", lift(frame,10.25).coordinate_frame == 8);
    frame.coordinate_frame = 0;
    check("regression/zero_frame_defaults_local", lift(frame,10.25).coordinate_frame == 1);
    auto force = s;
    force.type_mask |= 512;
    record("characterization/force_bit_not_acceleration_validation", lift(force,10.25),
           {1.59375,0,0,2.75,0,0,3,0,0});
    check("characterization/force_bit_preserved", (lift(force,10.25).type_mask & 512) != 0);
    check("regression/finite_ready", pc::passThroughReferenceReady(s));
    auto invalid = s;
    invalid.is_valid = false;
    check("regression/invalid_not_ready", !pc::passThroughReferenceReady(invalid));
    check("characterization/invalid_lift_has_no_status", lift(invalid,10.25).type_mask == 0);
    invalid = ignored;
    invalid.velocity_k.x() = std::numeric_limits<double>::quiet_NaN();
    check("characterization/ignored_nan_still_not_ready", !pc::passThroughReferenceReady(invalid));
    check("characterization/takeover_checks_position_not_velocity",
          pc::passThroughMayTakeSetpoint(s,1,0,0,.01,.01,false));
    check("regression/armed_does_not_gate_origin",
          pc::passThroughMayTakeSetpoint(s,50,0,0,.01,.01,true));

    // Bounded per-axis mask oracle; fixed integer-to-double mapping is reproducible.
    constexpr uint32_t seed = 157;
    std::mt19937 rng(seed);
    auto number = [&rng]() { return (static_cast<int>(rng() % 2001) - 1000) / 100.0; };
    int mask_failures = 0;
    for (uint16_t mask = 0; mask < 512; ++mask) {
        auto m = sample(0,0,0,10);
        // Explicit call order (C++ argument evaluation order is not a seed contract).
        Values input{};
        for (auto& value : input) value = number();
        m.position_k = Eigen::Vector3d(input[0],input[1],input[2]);
        m.velocity_k = Eigen::Vector3d(input[3],input[4],input[5]);
        m.acceleration_k = Eigen::Vector3d(input[6],input[7],input[8]);
        m.type_mask = mask | pc::kDefaultPvaLocalTypeMask;
        const double tau = (rng() % 500 + 1) / 1000.0;
        Values expected{};
        for (int axis = 0; axis < 3; ++axis) {
            const double v = (mask & (1 << (axis + 3))) ? 0 : input[axis + 3];
            const double a = (mask & (1 << (axis + 6))) ? 0 : input[axis + 6];
            if (!(mask & (1 << axis))) expected[axis] = input[axis] + v*tau + .5*a*tau*tau;
            if (!(mask & (1 << (axis+3)))) expected[axis+3] = v+a*tau;
            if (!(mask & (1 << (axis+6)))) expected[axis+6] = a;
        }
        const auto actual = lift(m,10+tau);
        const bool ok = equal(values(actual),expected) && actual.type_mask == m.type_mask;
        ++checks;
        failures += !ok;
        if (!ok && mask_failures++ < 3) {
            std::cout << "mask_failure mask=" << mask << " tau=" << tau << " input=";
            print(input);
            std::cout << " actual="; print(values(actual));
            std::cout << " expected="; print(expected); std::cout << '\n';
        }
    }
    std::cout << "mask_sweep seed=" << seed << " trials=512 failures=" << mask_failures << '\n';
    std::cout << "SUMMARY checks=" << checks << " failures=" << failures
              << " characterization_PASS_is_not_safety_acceptance\n";
    return failures ? 1 : 0;
}
