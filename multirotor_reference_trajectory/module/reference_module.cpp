// px4_multirotor_reference: the multirotor reference trajectory generator as an xgc2-module module.
//
// It runs multirotor_reference_trajectory_core, the ROS-free runtime that the ROS node
// (multirotor_reference_trajectory_node) also runs: the analytic curves, the sampled references
// and the state machine that activates them. What the node's input producer and output consumer do
// with ROS topics, the module does with typed ports; the payloads are in
// multirotor_reference_trajectory/payloads.h.
//
//   in  analytic         event  xgc2.px4.reference_analytic.v1  request/analytic
//   in  sampled          event  xgc2.px4.reference_sampled.v1   request/sampled
//   in  reset            event  xgc2.px4.reference_reset.v1     reset
//   out status           state  xgc2.px4.reference_status.v1    status
//   out active_analytic  state  xgc2.px4.reference_analytic.v1  active/analytic
//   out active_sampled   state  xgc2.px4.reference_sampled.v1   active/sampled
//
// The analytic port has several writers in an entity: the operator's or planner's requests and the
// controller's reference activation request.
//
// A request is stamped by its producer with its receipt time, which becomes the time of the event
// the runtime sees (the node uses ros::Time::now() in the callback). Every step reads the requests
// that arrived, then updates the runtime at the host clock and publishes what its output events ask
// for, exactly as one iteration of the node's main loop does.
//
// Configuration (JSON object; the keys and defaults are those of the ROS node's private
// parameters and config/multirotor_reference_trajectory.yaml): main_frequency (Hz, the step period
// the module asks the host for), status_rate, active_publish_rate, validation_sample_dt,
// trajectory_timeout, min_lead_time, max_velocity, max_acceleration, max_jerk, max_snap,
// min_specific_thrust. A key the module does not know is an error. A configure() while running
// restarts the runtime (the runtime's setConfig resets it), so it drops the active reference.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <module_support/json_config.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"
#include "multirotor_reference_trajectory/payloads.h"
#include "payload_conversion.h"
#include "xgc2/module.h"

#ifndef PX4_MODULE_VERSION
#define PX4_MODULE_VERSION "0"
#endif

namespace {

namespace mrt = multirotor_reference_trajectory;
namespace conv = multirotor_reference_trajectory::module;
namespace sm = state_machine;

enum Port : uint32_t {
    kAnalytic,
    kSampled,
    kReset,
    kStatus,
    kActiveAnalytic,
    kActiveSampled,
    kPortCount
};

constexpr uint32_t kRequestDepth[] = {8, 4, 4};  // analytic, sampled, reset

template <class T>
constexpr xgc2_port_desc describe(const char* name, xgc2_port_direction direction,
                                  xgc2_port_kind kind, const char* schema, uint32_t queue_depth) {
    return {name,
            static_cast<uint32_t>(direction),
            static_cast<uint32_t>(kind),
            schema,
            sizeof(T),
            alignof(T),
            queue_depth,
            0};
}

const xgc2_port_desc kPorts[kPortCount] = {
    describe<xgc2_px4_reference_analytic_v1>("analytic", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                             XGC2_PX4_REFERENCE_ANALYTIC_SCHEMA, kRequestDepth[0]),
    describe<xgc2_px4_reference_sampled_v1>("sampled", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                            XGC2_PX4_REFERENCE_SAMPLED_SCHEMA, kRequestDepth[1]),
    describe<xgc2_px4_reference_reset_v1>("reset", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                          XGC2_PX4_REFERENCE_RESET_SCHEMA, kRequestDepth[2]),
    describe<xgc2_px4_reference_status_v1>("status", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                           XGC2_PX4_REFERENCE_STATUS_SCHEMA, 0),
    describe<xgc2_px4_reference_analytic_v1>("active_analytic", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                             XGC2_PX4_REFERENCE_ANALYTIC_SCHEMA, 0),
    describe<xgc2_px4_reference_sampled_v1>("active_sampled", XGC2_PORT_OUT, XGC2_PORT_STATE,
                                            XGC2_PX4_REFERENCE_SAMPLED_SCHEMA, 0),
};

constexpr int kLogWarn = 2;
constexpr int kLogError = 3;
constexpr int kHealthOk = 0;
constexpr int kHealthDegraded = 1;

struct Settings {
    mrt::ReferenceTrajectoryConfig runtime;
    double main_frequency_hz{100.0};

    Settings() {
        // config/multirotor_reference_trajectory.yaml
        runtime.limits.min_specific_thrust = 0.1;
    }
};

Settings readSettings(module_support::JsonConfig& json) {
    Settings s;
    mrt::ReferenceTrajectoryConfig& c = s.runtime;
    json.read("main_frequency", s.main_frequency_hz);
    json.read("status_rate", c.status_rate_hz);
    json.read("active_publish_rate", c.active_publish_rate_hz);
    json.read("validation_sample_dt", c.validation_sample_dt);
    json.read("trajectory_timeout", c.trajectory_timeout);
    json.read("min_lead_time", c.min_lead_time);
    json.read("max_velocity", c.limits.max_velocity);
    json.read("max_acceleration", c.limits.max_acceleration);
    json.read("max_jerk", c.limits.max_jerk);
    json.read("max_snap", c.limits.max_snap);
    json.read("min_specific_thrust", c.limits.min_specific_thrust);
    const std::string unknown = json.firstUnusedKey();
    if (!unknown.empty()) {
        throw std::invalid_argument("unknown configuration key '" + unknown + "'");
    }
    if (!std::isfinite(s.main_frequency_hz) || s.main_frequency_hz <= 0.0) {
        throw std::invalid_argument("main_frequency must be a positive number");
    }
    return s;
}

double toSeconds(int64_t ns) {
    return mrt::Time().fromNSec(static_cast<uint64_t>(std::max<int64_t>(0, ns))).toSec();
}

const char* stateName(uint8_t state) {
    switch (state) {
        case mrt::reference::ReferenceStatus::STATE_SELF_CHECK:
            return "SelfCheck";
        case mrt::reference::ReferenceStatus::STATE_READY:
            return "Ready";
        case mrt::reference::ReferenceStatus::STATE_ACTIVE:
            return "Active";
        default:
            return "Fault";
    }
}

class ReferenceModule {
   public:
    ReferenceModule(const xgc2_host_api* host, void* host_ctx) : host_(host), ctx_(host_ctx) {}

    // Runs `f`; an exception is logged and becomes the module's failure.
    template <class F>
    xgc2_status guarded(const char* where, F&& f) {
        try {
            return f();
        } catch (const std::exception& e) {
            log(kLogError, std::string(where) + ": " + e.what());
        } catch (...) {
            log(kLogError, std::string(where) + ": unknown exception");
        }
        return XGC2_ERR_INTERNAL;
    }

    // Parses and stores the configuration; applies it when the module is running.
    xgc2_status configure(const xgc2_config* config) {
        try {
            module_support::JsonConfig json(config);
            settings_ = readSettings(json);
        } catch (const std::invalid_argument& e) {
            log(kLogError, std::string("invalid configuration: ") + e.what());
            return XGC2_ERR_INVALID;
        }
        if (running_)
            applySettings();
        return XGC2_OK;
    }

    xgc2_status start() {
        running_ = true;
        applySettings();
        return XGC2_OK;
    }

    xgc2_status stop() {
        running_ = false;
        return XGC2_OK;
    }

    // One iteration of ReferenceTrajectoryNode::run().
    xgc2_status step(const xgc2_step_ctx* ctx) {
        if (!running_)
            return XGC2_ERR_STATE;
        collectRequests();
        for (const Request& request : requests_)
            accept(request);
        const double now = toSeconds(ctx->now_ns);
        runtime_.update(now);
        // The health follows the last write: degraded after a refused one, ok again after a
        // successful one.
        for (const sm::Event& event : runtime_.stateMachine().currentOutputEvents()) {
            output_failed_ = !publish(event, ctx->now_ns, now);
        }
        reportIfChanged();
        return XGC2_OK;
    }

   private:
    struct Request {
        uint32_t port;
        xgc2_sample_view view;
    };

    void log(int level, const std::string& message) const {
        host_->log(ctx_, level, message.c_str());
    }

    // The runtime restarts, as the ROS node's runtime does when its config is set.
    void applySettings() {
        runtime_.setConfig(settings_.runtime);
        host_->set_period_ns(ctx_, std::llround(1e9 / settings_.main_frequency_hz));
        reported_ = false;
        reportIfChanged();
    }

    // Everything that arrived on the request ports, in arrival order (the node's callbacks run in
    // that order inside ros::spinOnce()).
    void collectRequests() {
        requests_.clear();
        for (uint32_t port : {uint32_t{kAnalytic}, uint32_t{kSampled}, uint32_t{kReset}}) {
            xgc2_sample_view view{};
            for (uint32_t n = 0;
                 n < kRequestDepth[port] && host_->read_next(ctx_, port, &view) == XGC2_OK; ++n) {
                requests_.push_back({port, view});
            }
        }
        std::stable_sort(
            requests_.begin(), requests_.end(),
            [](const Request& a, const Request& b) { return a.view.stamp_ns < b.view.stamp_ns; });
    }

    // ReferenceInputProducer::post, at the request's receipt time.
    void post(uint32_t event_id, const char* source, const Request& request) {
        sm::Event event(event_id, sm::EventTimestamp{toSeconds(request.view.stamp_ns)});
        event.source = source;
        const sm::Status status = runtime_.postEvent(std::move(event));
        if (!status.ok()) {
            log(kLogWarn, std::string("cannot post event from ") + source + ": " + status.message);
        }
    }

    void accept(const Request& request) {
        try {
            switch (request.port) {
                case kAnalytic:
                    acceptAnalytic(request);
                    break;
                case kSampled:
                    acceptSampled(request);
                    break;
                case kReset:
                    runtime_.reset();
                    post(mrt::event_type::RESET_REQUESTED, "reset", request);
                    break;
                default:
                    break;
            }
        } catch (const std::exception& e) {
            // A request the runtime cannot represent (a start time beyond the range of ROS time,
            // for example) is refused; the runtime keeps its state.
            log(kLogError, std::string("request refused: ") + e.what());
        }
    }

    void acceptAnalytic(const Request& request) {
        mrt::reference::AnalyticReference message;
        if (request.view.size != sizeof(xgc2_px4_reference_analytic_v1) ||
            !conv::toCore(*static_cast<const xgc2_px4_reference_analytic_v1*>(request.view.data),
                          message) ||
            !runtime_.acceptAnalytic(message)) {
            log(kLogWarn, "rejected analytic reference");
            return;
        }
        post(mrt::event_type::ANALYTIC_RECEIVED, "analytic_reference", request);
    }

    void acceptSampled(const Request& request) {
        mrt::reference::SampledReference message;
        if (request.view.size != sizeof(xgc2_px4_reference_sampled_v1) ||
            !conv::toCore(*static_cast<const xgc2_px4_reference_sampled_v1*>(request.view.data),
                          message) ||
            !runtime_.acceptSampled(message)) {
            log(kLogWarn, "rejected sampled reference");
            return;
        }
        post(mrt::event_type::SAMPLED_RECEIVED, "sampled_reference", request);
    }

    // Fills one payload in the port's slot; false when the slot is unavailable or the message does
    // not fit the payload.
    template <class Payload, class Fill>
    bool write(uint32_t port, int64_t stamp_ns, Fill&& fill) {
        void* slot = host_->write_begin(ctx_, port);
        if (slot == nullptr) {
            log(kLogError, std::string("output ") + kPorts[port].name + " is not writable");
            return false;
        }
        if (!fill(*static_cast<Payload*>(slot))) {
            host_->write_abort(ctx_, port);
            log(kLogError, std::string("message does not fit output ") + kPorts[port].name);
            return false;
        }
        return host_->write_commit(ctx_, port, stamp_ns) == XGC2_OK;
    }

    // ReferenceOutputConsumer::handle. Returns false when the output could not be written.
    bool publish(const sm::Event& event, int64_t now_ns, double now) {
        if (event.id == mrt::output_event_type::PUBLISH_STATUS) {
            const auto status = runtime_.makeStatus(event.timestamp > 0.0 ? event.timestamp : now);
            return write<xgc2_px4_reference_status_v1>(
                kStatus, now_ns, [&](auto& out) { return conv::toPayload(status, out); });
        }
        if (event.id == mrt::output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
            return write<xgc2_px4_reference_analytic_v1>(kActiveAnalytic, now_ns, [&](auto& out) {
                return conv::toPayload(runtime_.activeAnalyticMessage(), out);
            });
        }
        if (event.id == mrt::output_event_type::PUBLISH_ACTIVE_SAMPLED) {
            return write<xgc2_px4_reference_sampled_v1>(kActiveSampled, now_ns, [&](auto& out) {
                return conv::toPayload(runtime_.activeSampledMessage(), out);
            });
        }
        return true;
    }

    // The runtime state and whether the last step wrote all its outputs, as the instance's health.
    void reportIfChanged() {
        const uint8_t state = runtime_.currentState();
        const int health = output_failed_ ? kHealthDegraded : kHealthOk;
        if (reported_ && state == reported_state_ && health == reported_health_)
            return;
        reported_ = true;
        reported_state_ = state;
        reported_health_ = health;
        host_->report(ctx_, health, stateName(state));
    }

    const xgc2_host_api* host_;
    void* ctx_;
    Settings settings_;
    mrt::ReferenceTrajectoryRuntime runtime_;
    std::vector<Request> requests_;
    bool running_{false};
    bool output_failed_{false};
    bool reported_{false};
    uint8_t reported_state_{0};
    int reported_health_{kHealthOk};
};

ReferenceModule* module(xgc2_instance* handle) {
    return reinterpret_cast<ReferenceModule*>(handle);
}

xgc2_status create(const xgc2_host_api* host, void* host_ctx, const xgc2_config* config,
                   xgc2_instance** out) {
    if (host == nullptr || out == nullptr || host->abi_major != XGC2_MODULE_ABI_MAJOR) {
        return XGC2_ERR_INVALID;
    }
    std::unique_ptr<ReferenceModule> created;
    try {
        created = std::make_unique<ReferenceModule>(host, host_ctx);
    } catch (...) {
        return XGC2_ERR_INTERNAL;
    }
    const xgc2_status status =
        created->guarded("create", [&] { return created->configure(config); });
    if (status != XGC2_OK)
        return status;
    *out = reinterpret_cast<xgc2_instance*>(created.release());
    return XGC2_OK;
}

xgc2_status configure(xgc2_instance* handle, const xgc2_config* config) {
    ReferenceModule* m = module(handle);
    return m->guarded("configure", [&] { return m->configure(config); });
}

xgc2_status start(xgc2_instance* handle) {
    ReferenceModule* m = module(handle);
    return m->guarded("start", [&] { return m->start(); });
}

xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* ctx) {
    ReferenceModule* m = module(handle);
    return m->guarded("step", [&] { return m->step(ctx); });
}

xgc2_status stop(xgc2_instance* handle) {
    ReferenceModule* m = module(handle);
    return m->guarded("stop", [&] { return m->stop(); });
}

void destroy(xgc2_instance* handle) {
    delete module(handle);
}

const xgc2_module_desc kDescriptor = {
    XGC2_MODULE_ABI_MAJOR,
    XGC2_MODULE_ABI_MINOR,
    "px4_multirotor_reference",
    PX4_MODULE_VERSION,
    kPorts,
    kPortCount,
    &create,
    &configure,
    &start,
    &step,
    &stop,
    &destroy,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc2_module_desc* xgc2_module_entry(void) {
    return &kDescriptor;
}
