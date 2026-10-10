// px4_multirotor_controller: the PX4 multirotor controller as an xgc2-module module.
//
// It runs px4_multirotor_controller_core, the ROS-free core that the ROS node
// (px4_multirotor_controller_node) also runs: DroneController, its state machines and tracking
// strategies, ControllerDriver (receive statistics, the NMPC worker) and the same input handling
// (driver/trajectory_ingress.h, driver/command_input.h). What the node's input producers and output
// consumers do with ROS topics and MAVROS services, the module does with typed ports; the payloads
// are in px4_multirotor_controller/payloads.h and multirotor_reference_trajectory/payloads.h.
//
//   in  state_estimate        event  xgc2.px4.state_estimate.v1       alg/state_estimator/state
//   in  local_pose            event  xgc2.px4.pose.v1       required  mavros/local_position/pose
//   in  local_velocity        event  xgc2.px4.velocity.v1   required  mavros/local_position/velocity_local
//   in  imu                   event  xgc2.px4.imu.v1        required  mavros/imu/data
//   in  fcu_state             event  xgc2.px4.fcu_state.v1  required  mavros/state
//   in  battery               event  xgc2.px4.battery.v1              mavros/battery
//   in  vrpn_pose             event  xgc2.px4.pose.v1       required  pose (the canonical pose)
//   in  command               event  xgc2.px4.command.v1              /command
//   in  alg_setpoint          event  xgc2.px4.position_target.v1      alg/setpoint_raw/local
//   in  hover_thrust          event  xgc2.px4.hover_thrust.v1         hover_thrust/estimate_state
//   in  ref_active_analytic   state  xgc2.px4.reference_analytic.v1   the reference module's
//   in  ref_active_sampled    state  xgc2.px4.reference_sampled.v1    active/analytic, active/sampled
//   out setpoint              state  xgc2.px4.position_target.v1      mavros/setpoint_raw/local
//   out attitude_rate         state  xgc2.px4.attitude_rate_target.v1 mavros/setpoint_raw/attitude
//   out fcu_request           event  xgc2.px4.fcu_request.v1          mavros/cmd/command, mavros/set_mode
//   out status                state  xgc2.px4.controller_status.v1     custom/statustext
//   out ref_request           event  xgc2.px4.reference_analytic.v1   the reference module's analytic
//
// Every input sample is stamped by its producer with the time it was received. That stamp is the
// time of the receive statistics and of the input event, like ros::Time::now() in the node's
// callbacks. The sensor inputs are event ports, not state ports: the controller's receive
// statistics and its frame counters are part of its safety logic and count every message, as the
// node's callbacks do, so a burst of two messages within one step must not become one. Only the
// active reference is a state: the reference module publishes it as one, and the newest is the one
// that counts. A step applies the samples that arrived in stamp order, updates the controller at the
// host clock and handles the output events the way the node's consumers do; with the period of 1 ms
// it is one iteration of the node's 1 kHz control loop. The NMPC solver runs on a worker thread of
// ControllerDriver; its completion wakes the host, so the step that consumes the result does not wait
// for the next period.
//
// The telemetry the node publishes for observers (sensor statistics, state machine events, tracking
// error, NMPC debug samples) is not a port of the module.
//
// Configuration: a JSON object with the structure of config/uav_nmpc.yaml (a key path "smc/k1" is
// {"smc": {"k1": ...}}). Keys that are absent keep the values of that file, which is compiled into
// the core. world_boundary_json must be given (a string: "null" for no geofence or the boundary as
// JSON text), as in the ROS node. A key the controller does not read is an error. control_frequency
// is not a key: the module asks the host for a 1 kHz step like the node. A configure() while the
// controller is flying is refused (XGC2_ERR_STATE); on the ground it takes effect at once.
//
// start() builds the controller afresh; stop() tears it down (the solver thread is joined).

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <module_support/json_config.hpp>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "multirotor_reference_trajectory/payloads.h"
#include "module_config.h"
#include "payload_conversion.h"
#include "px4_multirotor_controller/common/core_log.h"
#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/driver/command_input.h"
#include "px4_multirotor_controller/driver/controller_config.h"
#include "px4_multirotor_controller/driver/controller_driver.h"
#include "px4_multirotor_controller/driver/trajectory_ingress.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"
#include "px4_multirotor_controller/payloads.h"
#include "px4_multirotor_controller/uav/reference_activation.h"
#include "xgc2/module.h"

#ifndef PX4_MODULE_VERSION
#define PX4_MODULE_VERSION "0"
#endif

namespace {

namespace pmc = px4_multirotor_controller;
namespace conv = px4_multirotor_controller::module;
namespace sm = state_machine;

enum Port : uint32_t {
    // Inputs first: the port index is also the input index of xgc2_step_ctx.changed_inputs, and
    // the first seven are the SensorStream values of ControllerDriver's statistics.
    kStateEstimate,
    kLocalPose,
    kLocalVelocity,
    kImu,
    kFcuState,
    kBattery,
    kVrpnPose,
    kCommand,
    kAlgSetpoint,
    kHoverThrust,
    kRefActiveAnalytic,
    kRefActiveSampled,
    kSetpoint,
    kAttitudeRate,
    kFcuRequest,
    kStatus,
    kRefRequest,
    kPortCount
};
constexpr uint32_t kInputCount = kRefActiveSampled + 1;
constexpr uint32_t kStatisticsStreams = static_cast<uint32_t>(pmc::SensorStream::Count);

static_assert(kStateEstimate == static_cast<uint32_t>(pmc::SensorStream::Estimate), "stream order");
static_assert(kLocalPose == static_cast<uint32_t>(pmc::SensorStream::LocalPose), "stream order");
static_assert(kLocalVelocity == static_cast<uint32_t>(pmc::SensorStream::LocalVelocity),
              "stream order");
static_assert(kImu == static_cast<uint32_t>(pmc::SensorStream::Imu), "stream order");
static_assert(kFcuState == static_cast<uint32_t>(pmc::SensorStream::State), "stream order");
static_assert(kBattery == static_cast<uint32_t>(pmc::SensorStream::Battery), "stream order");
static_assert(kVrpnPose == static_cast<uint32_t>(pmc::SensorStream::Pose), "stream order");
static_assert(kInputCount < 64, "changed_inputs is a 64-bit mask");

template <class T>
constexpr xgc2_port_desc describe(const char* name, xgc2_port_direction direction,
                                  xgc2_port_kind kind, const char* schema, uint32_t queue_depth = 0,
                                  uint32_t flags = 0) {
    return {name,
            static_cast<uint32_t>(direction),
            static_cast<uint32_t>(kind),
            schema,
            sizeof(T),
            alignof(T),
            queue_depth,
            flags};
}

constexpr xgc2_port_direction kIn = XGC2_PORT_IN;
constexpr xgc2_port_direction kOut = XGC2_PORT_OUT;
constexpr xgc2_port_kind kState = XGC2_PORT_STATE;
constexpr xgc2_port_kind kEvent = XGC2_PORT_EVENT;

const xgc2_port_desc kPorts[kPortCount] = {
    describe<xgc2_px4_state_estimate_v1>("state_estimate", kIn, kEvent,
                                         XGC2_PX4_STATE_ESTIMATE_SCHEMA, 8),
    describe<xgc2_px4_pose_v1>("local_pose", kIn, kEvent, XGC2_PX4_POSE_SCHEMA, 8,
                               XGC2_PORT_REQUIRED),
    describe<xgc2_px4_velocity_v1>("local_velocity", kIn, kEvent, XGC2_PX4_VELOCITY_SCHEMA, 8,
                                   XGC2_PORT_REQUIRED),
    describe<xgc2_px4_imu_v1>("imu", kIn, kEvent, XGC2_PX4_IMU_SCHEMA, 16, XGC2_PORT_REQUIRED),
    describe<xgc2_px4_fcu_state_v1>("fcu_state", kIn, kEvent, XGC2_PX4_FCU_STATE_SCHEMA, 8,
                                    XGC2_PORT_REQUIRED),
    describe<xgc2_px4_battery_v1>("battery", kIn, kEvent, XGC2_PX4_BATTERY_SCHEMA, 4),
    describe<xgc2_px4_pose_v1>("vrpn_pose", kIn, kEvent, XGC2_PX4_POSE_SCHEMA, 8,
                               XGC2_PORT_REQUIRED),
    describe<xgc2_px4_command_v1>("command", kIn, kEvent, XGC2_PX4_COMMAND_SCHEMA, 8),
    describe<xgc2_px4_position_target_v1>("alg_setpoint", kIn, kEvent,
                                          XGC2_PX4_POSITION_TARGET_SCHEMA, 8),
    describe<xgc2_px4_hover_thrust_v1>("hover_thrust", kIn, kEvent, XGC2_PX4_HOVER_THRUST_SCHEMA,
                                       8),
    describe<xgc2_px4_reference_analytic_v1>("ref_active_analytic", kIn, kState,
                                             XGC2_PX4_REFERENCE_ANALYTIC_SCHEMA),
    describe<xgc2_px4_reference_sampled_v1>("ref_active_sampled", kIn, kState,
                                            XGC2_PX4_REFERENCE_SAMPLED_SCHEMA),
    describe<xgc2_px4_position_target_v1>("setpoint", kOut, kState, XGC2_PX4_POSITION_TARGET_SCHEMA),
    describe<xgc2_px4_attitude_rate_target_v1>("attitude_rate", kOut, kState,
                                               XGC2_PX4_ATTITUDE_RATE_TARGET_SCHEMA),
    describe<xgc2_px4_fcu_request_v1>("fcu_request", kOut, kEvent, XGC2_PX4_FCU_REQUEST_SCHEMA, 8),
    describe<xgc2_px4_controller_status_v1>("status", kOut, kState,
                                            XGC2_PX4_CONTROLLER_STATUS_SCHEMA),
    describe<xgc2_px4_reference_analytic_v1>("ref_request", kOut, kEvent,
                                             XGC2_PX4_REFERENCE_ANALYTIC_SCHEMA, 4),
};

constexpr int kLogInfo = 1;
constexpr int kLogWarn = 2;
constexpr int kLogError = 3;
constexpr int kHealthOk = 0;
constexpr int kHealthDegraded = 1;
constexpr int64_t kControlPeriodNs = 1000000;  // the node's hard-coded 1 kHz loop

class ControllerModule;

// The core logs through one process-wide sink without a context. The module routes a line to the host
// of the instance whose call is running on this thread; the solver thread has no such call and falls
// back to stderr, like the core's default sink.
thread_local const ControllerModule* t_running = nullptr;
std::atomic<int> g_instances{0};

void coreLogSink(pmc::LogLevel level, const char* message);

std::string boundedString(const char* text, size_t capacity) {
    return std::string(text, strnlen(text, capacity));
}

// Copies `text` into a NUL-terminated field; false when it does not fit.
bool copyText(const std::string& text, char* field, size_t capacity) {
    if (text.size() >= capacity) return false;
    std::memset(field, 0, capacity);
    std::memcpy(field, text.data(), text.size());
    return true;
}

class ControllerModule {
   public:
    ControllerModule(const xgc2_host_api* host, void* host_ctx) : host_(host), ctx_(host_ctx) {
        if (g_instances.fetch_add(1) == 0) pmc::setLogSink(&coreLogSink);
    }

    ~ControllerModule() {
        stop();
        if (g_instances.fetch_sub(1) == 1) pmc::setLogSink(nullptr);
    }

    // Runs `f` with this instance as the target of the core's log; an exception is logged and
    // becomes the module's failure.
    template <class F>
    xgc2_status guarded(const char* where, F&& f) {
        const ControllerModule* const outer = t_running;
        t_running = this;
        xgc2_status status = XGC2_ERR_INTERNAL;
        try {
            status = f();
        } catch (const std::exception& e) {
            log(kLogError, std::string(where) + ": " + e.what());
        } catch (...) {
            log(kLogError, std::string(where) + ": unknown exception");
        }
        t_running = outer;
        return status;
    }

    void log(int level, const std::string& message) const {
        host_->log(ctx_, level, message.c_str());
    }

    // Parses the configuration and, when it is valid, makes it the controller's.
    xgc2_status configure(const xgc2_config* config) {
        pmc::ControllerConfig next;
        try {
            module_support::JsonConfig json(config);
            next = conv::readConfig(json, [this](bool warning, const std::string& m) {
                log(warning ? kLogWarn : kLogInfo, m);
            });
        } catch (const std::invalid_argument& e) {
            log(kLogError, std::string("invalid configuration: ") + e.what());
            return XGC2_ERR_INVALID;
        }
        if (runtime_ && !onTheGround()) {
            log(kLogWarn, "configuration refused: the controller is not on the ground");
            return XGC2_ERR_STATE;
        }
        config_ = next;
        if (runtime_) runtime_->driver.controller().setConfig(config_);
        return XGC2_OK;
    }

    xgc2_status start() {
        if (runtime_) return XGC2_ERR_STATE;
        runtime_ = std::make_unique<Runtime>(config_, [this] { host_->wake(ctx_); });
        runtime_->driver.start(time(host_->now_ns(ctx_)));
        host_->set_period_ns(ctx_, kControlPeriodNs);
        reported_ = false;
        reportIfChanged();
        return XGC2_OK;
    }

    xgc2_status stop() {
        if (runtime_) {
            // Joins the solver thread: no wake() is called after this returns.
            runtime_->driver.stop();
            runtime_.reset();
        }
        return XGC2_OK;
    }

    // One iteration of DroneRosNode::run(): the samples that arrived, one controller update, the
    // output events.
    xgc2_status step(const xgc2_step_ctx* ctx) {
        if (!runtime_) return XGC2_ERR_STATE;
        Runtime& rt = *runtime_;
        const pmc::Time now = time(ctx->now_ns);
        collectInputs(ctx);
        for (const Input& input : inputs_) {
            try {
                apply(rt, input);
            } catch (const std::exception& e) {
                log(kLogError, std::string("input ") + kPorts[input.port].name + " refused: " + e.what());
            }
        }
        rt.driver.update(now);
        wrote_ = write_failed_ = false;
        for (const sm::Event& event : rt.driver.controller().getStateMachine().currentOutputEvents()) {
            try {
                handle(rt, event, ctx->now_ns, now);
            } catch (const std::exception& e) {
                log(kLogError, "output event " + std::to_string(event.id) + " failed: " + e.what());
                write_failed_ = true;
            }
        }
        // The health follows the last writes: degraded after a refused one, ok again after a step
        // whose writes all succeeded.
        if (write_failed_) {
            output_failed_ = true;
        } else if (wrote_) {
            output_failed_ = false;
        }
        rt.driver.statistics().resetNewFlags();
        reportIfChanged();
        return XGC2_OK;
    }

   private:
    // Everything that exists while the module is started. start() builds it, stop() destroys it, so a
    // restart begins with a controller in SelfCheck and no history.
    struct Runtime {
        pmc::SensorData sensor;
        pmc::ControllerDriver driver;
        pmc::ReferenceActivation activation;

        Runtime(const pmc::ControllerConfig& config, std::function<void()> on_nmpc_result)
            : driver(sensor, std::move(on_nmpc_result)) {
            driver.controller().setConfig(config);
        }
    };

    struct Input {
        uint32_t port;
        xgc2_sample_view view;
    };

    static pmc::Time time(int64_t ns) {
        return pmc::Time().fromNSec(static_cast<uint64_t>(std::max<int64_t>(0, ns)));
    }

    static void stamp(const pmc::Time& t, uint32_t& sec, uint32_t& nsec) {
        sec = t.sec;
        nsec = t.nsec;
    }

    bool onTheGround() const {
        const sm::StateId state =
            runtime_->driver.controller().getStateMachine().currentState(pmc::region_type::CONTROL);
        return state == pmc::state_type::SelfCheck || state == pmc::state_type::Ready;
    }

    // The samples that arrived, in arrival order (the node's callbacks run in that order inside
    // ros::spinOnce()).
    void collectInputs(const xgc2_step_ctx* ctx) {
        inputs_.clear();
        for (uint32_t port = 0; port < kInputCount; ++port) {
            xgc2_sample_view view{};
            if (kPorts[port].kind == XGC2_PORT_STATE) {
                if ((ctx->changed_inputs & (uint64_t{1} << port)) != 0 &&
                    host_->read_latest(ctx_, port, &view) == XGC2_OK) {
                    inputs_.push_back({port, view});
                }
            } else {
                for (uint32_t n = 0; n < kPorts[port].queue_depth &&
                                     host_->read_next(ctx_, port, &view) == XGC2_OK;
                     ++n) {
                    inputs_.push_back({port, view});
                }
            }
        }
        std::stable_sort(inputs_.begin(), inputs_.end(), [](const Input& a, const Input& b) {
            return a.view.stamp_ns < b.view.stamp_ns;
        });
    }

    void post(Runtime& rt, sm::EventId id, double receipt_sec, const char* source) {
        sm::Event event(id, sm::EventTimestamp{receipt_sec});
        event.source = source;
        const sm::Status status = rt.driver.controller().getStateMachine().postEvent(std::move(event));
        if (!status.ok() && (post_failures_++ % 1000) == 0) {
            log(kLogError, std::string("cannot post input event from ") + source + ": " +
                               status.message + " (failures: " + std::to_string(post_failures_) + ")");
        }
    }

    // One input, as the node's input producer for that topic handles it.
    void apply(Runtime& rt, const Input& input) {
        if (input.view.size != kPorts[input.port].size) {
            log(kLogWarn, std::string("input ") + kPorts[input.port].name + " has a wrong size");
            return;
        }
        const pmc::Time receipt = time(input.view.stamp_ns);
        const double receipt_sec = receipt.toSec();
        if (input.port < kStatisticsStreams) {
            rt.driver.statistics().observe(static_cast<pmc::SensorStream>(input.port), receipt);
        }
        pmc::SensorData& sensor = rt.sensor;
        const void* data = input.view.data;
        switch (input.port) {
            case kStateEstimate: {
                const auto& m = *static_cast<const xgc2_px4_state_estimate_v1*>(data);
                sensor.x = m.position[0];
                sensor.y = m.position[1];
                sensor.z = m.position[2];
                sensor.vx = m.velocity[0];
                sensor.vy = m.velocity[1];
                sensor.vz = m.velocity[2];
                sensor.qx = m.orientation_xyzw[0];
                sensor.qy = m.orientation_xyzw[1];
                sensor.qz = m.orientation_xyzw[2];
                sensor.qw = m.orientation_xyzw[3];
                sensor.wx = m.angular_velocity[0];
                sensor.wy = m.angular_velocity[1];
                sensor.wz = m.angular_velocity[2];
                sensor.ax = m.linear_acceleration[0];
                sensor.ay = m.linear_acceleration[1];
                sensor.az = m.linear_acceleration[2];
                sensor.gx = m.gravity[0];
                sensor.gy = m.gravity[1];
                sensor.gz = m.gravity[2];
                sensor.accel_bias_x = m.accel_bias[0];
                sensor.accel_bias_y = m.accel_bias[1];
                sensor.accel_bias_z = m.accel_bias[2];
                sensor.uav_state_estimator_state = m.estimator_state;
                sensor.uav_state_estimator_flags = m.flags;
                sensor.uav_state_estimate_stamp = pmc::Time(m.stamp_sec, m.stamp_nsec).toSec();
                sensor.uav_state_filter_inertial_stamp = m.filter_inertial_stamp_sec;
                sensor.uav_state_filter_pose_stamp = m.filter_pose_stamp_sec;
                sensor.uav_state_last_vrpn_pose_stamp = m.last_vrpn_pose_stamp_sec;
                post(rt, pmc::event_type::INPUT_UAV_STATE_ESTIMATE_UPDATED, receipt_sec,
                     "alg/state_estimator/state");
                break;
            }
            case kLocalPose: {
                const auto& m = *static_cast<const xgc2_px4_pose_v1*>(data);
                sensor.local_x = m.position[0];
                sensor.local_y = m.position[1];
                sensor.local_z = m.position[2];
                sensor.local_qx = m.orientation_xyzw[0];
                sensor.local_qy = m.orientation_xyzw[1];
                sensor.local_qz = m.orientation_xyzw[2];
                sensor.local_qw = m.orientation_xyzw[3];
                post(rt, pmc::event_type::INPUT_LOCAL_POSITION_UPDATED, receipt_sec,
                     "mavros/local_position/pose");
                break;
            }
            case kLocalVelocity: {
                const auto& m = *static_cast<const xgc2_px4_velocity_v1*>(data);
                sensor.local_vx = m.linear[0];
                sensor.local_vy = m.linear[1];
                sensor.local_vz = m.linear[2];
                post(rt, pmc::event_type::INPUT_LOCAL_VELOCITY_UPDATED, receipt_sec,
                     "mavros/local_position/velocity_local");
                break;
            }
            case kImu:
                post(rt, pmc::event_type::INPUT_IMU_UPDATED, receipt_sec, "mavros/imu/data");
                break;
            case kFcuState: {
                const auto& m = *static_cast<const xgc2_px4_fcu_state_v1*>(data);
                sensor.fcu_connected = m.connected != 0;
                sensor.fcu_armed = m.armed != 0;
                sensor.fcu_guided = m.guided != 0;
                sensor.fcu_manual_input = m.manual_input != 0;
                sensor.fcu_mode = boundedString(m.mode, sizeof m.mode);
                sensor.fcu_system_status = m.system_status;
                post(rt, pmc::event_type::INPUT_FCU_STATE_UPDATED, receipt_sec, "mavros/state");
                break;
            }
            case kBattery: {
                const auto& m = *static_cast<const xgc2_px4_battery_v1*>(data);
                sensor.battery_percentage = m.percentage;
                post(rt, pmc::event_type::INPUT_BATTERY_UPDATED, receipt_sec, "mavros/battery");
                break;
            }
            case kVrpnPose: {
                const auto& m = *static_cast<const xgc2_px4_pose_v1*>(data);
                sensor.vrpn_x = m.position[0];
                sensor.vrpn_y = m.position[1];
                sensor.vrpn_z = m.position[2];
                sensor.vrpn_qx = m.orientation_xyzw[0];
                sensor.vrpn_qy = m.orientation_xyzw[1];
                sensor.vrpn_qz = m.orientation_xyzw[2];
                sensor.vrpn_qw = m.orientation_xyzw[3];
                post(rt, pmc::event_type::INPUT_VRPN_POSE_UPDATED, receipt_sec, "pose");
                break;
            }
            case kCommand: {
                const auto& m = *static_cast<const xgc2_px4_command_v1*>(data);
                const std::string text = boundedString(m.text, sizeof m.text);
                if (const auto event = pmc::commandInputEvent(text); event) {
                    post(rt, *event, receipt_sec, "command");
                } else {
                    log(kLogWarn, "unknown command: " + text);
                }
                break;
            }
            case kAlgSetpoint:
                applyPlannerSetpoint(rt, *static_cast<const xgc2_px4_position_target_v1*>(data),
                                     receipt, receipt_sec);
                break;
            case kHoverThrust: {
                const auto& m = *static_cast<const xgc2_px4_hover_thrust_v1*>(data);
                if (pmc::ingestHoverThrust(sensor, m.hover_thrust, m.flags,
                                           pmc::Time(m.stamp_sec, m.stamp_nsec).toSec(),
                                           receipt_sec)) {
                    post(rt, pmc::event_type::INPUT_HOVER_THRUST_UPDATED, receipt_sec,
                         "hover_thrust/estimate_state");
                }
                break;
            }
            case kRefActiveAnalytic: {
                pmc::reference::AnalyticReference message;
                if (conv::toCore(*static_cast<const xgc2_px4_reference_analytic_v1*>(data), message) &&
                    rt.driver.controller().activeTrajectoryCache().updateAnalytic(message, receipt)) {
                    post(rt, pmc::event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED, receipt_sec,
                         "alg/multirotor_reference_trajectory/active/analytic");
                } else {
                    log(kLogWarn, "rejected active analytic trajectory");
                }
                break;
            }
            case kRefActiveSampled: {
                pmc::reference::SampledReference message;
                if (conv::toCore(*static_cast<const xgc2_px4_reference_sampled_v1*>(data), message) &&
                    rt.driver.controller().activeTrajectoryCache().updateSampled(message, receipt)) {
                    post(rt, pmc::event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED, receipt_sec,
                         "alg/multirotor_reference_trajectory/active/sampled");
                } else {
                    log(kLogWarn, "rejected active sampled trajectory");
                }
                break;
            }
            default:
                break;
        }
    }

    void applyPlannerSetpoint(Runtime& rt, const xgc2_px4_position_target_v1& m,
                              const pmc::Time& receipt, double receipt_sec) {
        pmc::PositionTargetIngress ingress;
        ingress.position << m.position[0], m.position[1], m.position[2];
        ingress.velocity << m.velocity[0], m.velocity[1], m.velocity[2];
        ingress.acceleration << m.acceleration[0], m.acceleration[1], m.acceleration[2];
        ingress.yaw = m.yaw;
        ingress.yaw_rate = m.yaw_rate;
        ingress.type_mask = m.type_mask;
        ingress.coordinate_frame = m.coordinate_frame;
        ingress.header_stamp = pmc::Time(m.stamp_sec, m.stamp_nsec);
        ingress.receipt_time = receipt;
        switch (pmc::ingestPlannerSetpoint(rt.driver.controller(), ingress)) {
            case pmc::PlannerSetpointResult::kNotUsed:
                break;
            case pmc::PlannerSetpointResult::kRejectedSmcFrame:
                log(kLogWarn,
                    "SMC rejected PositionTarget: need world frame 1 and every P/V/A axis, with "
                    "FORCE clear");
                break;
            case pmc::PlannerSetpointResult::kRejectedEffectiveTime:
                log(kLogWarn, "effective-time setpoint needs a finite PVA and a non-zero header stamp");
                break;
            case pmc::PlannerSetpointResult::kCached:
                post(rt, pmc::event_type::INPUT_MPC_TRAJECTORY_UPDATED, receipt_sec,
                     "alg/setpoint_raw/local");
                break;
        }
    }

    // Fills one payload in the port's slot and commits it. A slot that is unavailable (an event queue
    // that is full) or a message that does not fit the payload is a failed write.
    template <class Payload, class Fill>
    void write(uint32_t port, int64_t stamp_ns, Fill&& fill) {
        void* slot = host_->write_begin(ctx_, port);
        if (slot == nullptr) {
            log(kLogError, std::string("output ") + kPorts[port].name + " is not writable");
            write_failed_ = true;
            return;
        }
        if (!fill(*static_cast<Payload*>(slot))) {
            host_->write_abort(ctx_, port);
            log(kLogError, std::string("message does not fit output ") + kPorts[port].name);
            write_failed_ = true;
            return;
        }
        if (host_->write_commit(ctx_, port, stamp_ns) == XGC2_OK) {
            wrote_ = true;
        } else {
            write_failed_ = true;
        }
    }

    // The output consumers of the ROS node, for one output event.
    void handle(Runtime& rt, const sm::Event& event, int64_t now_ns, const pmc::Time& now) {
        pmc::DroneController& controller = rt.driver.controller();
        switch (event.id) {
            case pmc::output_event_type::PUBLISH_SETPOINT:  // ControlOutputConsumer
                write<xgc2_px4_position_target_v1>(kSetpoint, now_ns, [&](auto& m) {
                    const pmc::Setpoint& s = controller.getSetpoint();
                    stamp(now, m.stamp_sec, m.stamp_nsec);
                    m.position[0] = s.x;
                    m.position[1] = s.y;
                    m.position[2] = s.z;
                    m.velocity[0] = s.vx;
                    m.velocity[1] = s.vy;
                    m.velocity[2] = s.vz;
                    m.acceleration[0] = s.ax;
                    m.acceleration[1] = s.ay;
                    m.acceleration[2] = s.az;
                    m.yaw = pmc::quaternionToYaw(s.qx, s.qy, s.qz, s.qw);
                    m.yaw_rate = s.yaw_rate;
                    m.type_mask = s.type_mask;
                    m.coordinate_frame = s.coordinate_frame;
                    std::memset(m.reserved, 0, sizeof m.reserved);
                    return true;
                });
                break;
            case pmc::output_event_type::PUBLISH_ATTITUDE_RATE_TARGET:  // ControlOutputConsumer
                write<xgc2_px4_attitude_rate_target_v1>(kAttitudeRate, now_ns, [&](auto& m) {
                    const pmc::AttitudeRateTarget& a = controller.getAttitudeRateTarget();
                    stamp(now, m.stamp_sec, m.stamp_nsec);
                    m.body_rate[0] = a.body_rate_x;
                    m.body_rate[1] = a.body_rate_y;
                    m.body_rate[2] = a.body_rate_z;
                    m.thrust = pmc::clamp(a.thrust, 0.0, 1.0);
                    return true;
                });
                break;
            case pmc::output_event_type::REQUEST_ARMING: {  // Px4ServiceOutputConsumer
                const auto it = event.payload.find("arm");
                if (it == event.payload.end() || !std::holds_alternative<bool>(it->second)) {
                    log(kLogWarn, "REQUEST_ARMING missing bool payload 'arm'");
                    break;
                }
                write<xgc2_px4_fcu_request_v1>(kFcuRequest, now_ns, [&](auto& m) {
                    std::memset(&m, 0, sizeof m);
                    stamp(now, m.stamp_sec, m.stamp_nsec);
                    m.kind = XGC2_PX4_FCU_REQUEST_ARM;
                    m.arm = std::get<bool>(it->second) ? 1U : 0U;
                    return true;
                });
                break;
            }
            case pmc::output_event_type::REQUEST_MODE: {  // Px4ServiceOutputConsumer
                const auto it = event.payload.find("mode");
                if (it == event.payload.end() || !std::holds_alternative<std::string>(it->second)) {
                    log(kLogWarn, "REQUEST_MODE missing string payload 'mode'");
                    break;
                }
                write<xgc2_px4_fcu_request_v1>(kFcuRequest, now_ns, [&](auto& m) {
                    std::memset(&m, 0, sizeof m);
                    stamp(now, m.stamp_sec, m.stamp_nsec);
                    m.kind = XGC2_PX4_FCU_REQUEST_MODE;
                    return copyText(std::get<std::string>(it->second), m.mode, sizeof m.mode);
                });
                break;
            }
            case pmc::output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION: {
                // ReferenceActivationOutputConsumer: the request goes to the reference generator.
                const double stamp_sec = event.timestamp > 0.0 ? event.timestamp : now.toSec();
                const pmc::reference::AnalyticReference request =
                    rt.activation.make(stamp_sec, controller.getSensorData(), controller.getConfig());
                write<xgc2_px4_reference_analytic_v1>(kRefRequest, now_ns, [&](auto& m) {
                    return conv::toPayload(request, "map", m);
                });
                break;
            }
            case pmc::output_event_type::REQUEST_NMPC_SOLVE:  // NmpcOutputConsumer
                rt.driver.nmpc().handle(event);
                break;
            case pmc::output_event_type::PUBLISH_CONTROLLER_STATUS:  // DebugOutputConsumer
                write<xgc2_px4_controller_status_v1>(kStatus, now_ns, [&](auto& m) {
                    std::string state = controller.getStateMachine().currentStateName(
                        pmc::region_type::CONTROL);
                    if (state.empty()) state = "Unknown";
                    stamp(now, m.stamp_sec, m.stamp_nsec);
                    return copyText(state, m.state, sizeof m.state);
                });
                break;
            default:
                // The node's telemetry for observers is not a port of the module.
                break;
        }
    }

    // The flight state as the instance's detail; degraded while the last output could not be written.
    void reportIfChanged() {
        const std::string state = runtime_->driver.controller().getStateMachine().currentStateName(
            pmc::region_type::CONTROL);
        const int health = output_failed_ ? kHealthDegraded : kHealthOk;
        if (reported_ && state == reported_state_ && health == reported_health_) return;
        reported_ = true;
        reported_state_ = state;
        reported_health_ = health;
        host_->report(ctx_, health, state.c_str());
    }

    const xgc2_host_api* host_;
    void* ctx_;
    pmc::ControllerConfig config_;
    std::unique_ptr<Runtime> runtime_;
    std::vector<Input> inputs_;
    uint64_t post_failures_{0};
    bool wrote_{false};
    bool write_failed_{false};
    bool output_failed_{false};
    bool reported_{false};
    std::string reported_state_;
    int reported_health_{kHealthOk};
};

void coreLogSink(pmc::LogLevel level, const char* message) {
    const int host_level = level == pmc::LogLevel::kError  ? kLogError
                           : level == pmc::LogLevel::kWarn ? kLogWarn
                                                           : kLogInfo;
    if (t_running != nullptr) {
        t_running->log(host_level, message);
        return;
    }
    std::fprintf(stderr, "[%s] %s\n",
                 level == pmc::LogLevel::kError  ? "ERROR"
                 : level == pmc::LogLevel::kWarn ? "WARN"
                                                 : "INFO",
                 message);
}

ControllerModule* module(xgc2_instance* handle) {
    return reinterpret_cast<ControllerModule*>(handle);
}

xgc2_status create(const xgc2_host_api* host, void* host_ctx, const xgc2_config* config,
                   xgc2_instance** out) {
    if (host == nullptr || out == nullptr || host->abi_major != XGC2_MODULE_ABI_MAJOR) {
        return XGC2_ERR_INVALID;
    }
    std::unique_ptr<ControllerModule> created;
    try {
        created = std::make_unique<ControllerModule>(host, host_ctx);
    } catch (...) {
        return XGC2_ERR_INTERNAL;
    }
    const xgc2_status status = created->guarded("create", [&] { return created->configure(config); });
    if (status != XGC2_OK) return status;
    *out = reinterpret_cast<xgc2_instance*>(created.release());
    return XGC2_OK;
}

xgc2_status configure(xgc2_instance* handle, const xgc2_config* config) {
    ControllerModule* m = module(handle);
    return m->guarded("configure", [&] { return m->configure(config); });
}

xgc2_status start(xgc2_instance* handle) {
    ControllerModule* m = module(handle);
    return m->guarded("start", [&] { return m->start(); });
}

xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* ctx) {
    ControllerModule* m = module(handle);
    return m->guarded("step", [&] { return m->step(ctx); });
}

xgc2_status stop(xgc2_instance* handle) {
    ControllerModule* m = module(handle);
    return m->guarded("stop", [&] { return m->stop(); });
}

void destroy(xgc2_instance* handle) {
    delete module(handle);
}

const xgc2_module_desc kDescriptor = {
    XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "px4_multirotor_controller",
    PX4_MODULE_VERSION,    kPorts,                kPortCount,
    &create,               &configure,            &start,
    &step,                 &stop,                 &destroy,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc2_module_desc* xgc2_module_entry(void) {
    return &kDescriptor;
}
