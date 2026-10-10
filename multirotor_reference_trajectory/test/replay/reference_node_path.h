#pragma once

// The ROS node's behavior without ROS: what ReferenceInputProducer does with a request and what one
// iteration of ReferenceTrajectoryNode::run() does with time. The replay harness traces it, and the
// module test compares the px4_multirotor_reference module with it. Requests are decoded from their
// ROS serialization, as the node receives them.

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <ros/serialization.h>
#include <std_msgs/Empty.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"
#include "multirotor_reference_trajectory/ros_reference_conversion.h"

namespace reference_replay {

namespace mrt = multirotor_reference_trajectory;
namespace msgs = multirotor_reference_trajectory_msgs;

// Request kinds of a replay stream (make_reference_stream.py).
constexpr uint8_t kAnalyticRequest = 1;
constexpr uint8_t kSampledRequest = 3;
constexpr uint8_t kResetRequest = 4;

struct Record {
    uint64_t t_ns;
    uint8_t kind;
    std::vector<uint8_t> data;
};

inline std::vector<Record> readStream(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    char magic[8];
    if (!in.read(magic, 8) || std::memcmp(magic, "MRTRPLY1", 8) != 0) {
        throw std::runtime_error("not a reference replay stream: " + path);
    }
    std::vector<Record> records;
    for (;;) {
        uint64_t t = 0;
        uint8_t kind = 0;
        uint32_t len = 0;
        if (!in.read(reinterpret_cast<char*>(&t), 8))
            break;
        in.read(reinterpret_cast<char*>(&kind), 1);
        in.read(reinterpret_cast<char*>(&len), 4);
        Record r{t, kind, std::vector<uint8_t>(len)};
        if (len > 0 && !in.read(reinterpret_cast<char*>(r.data.data()), len)) {
            throw std::runtime_error("truncated stream");
        }
        records.push_back(std::move(r));
    }
    return records;
}

template <typename M>
M decode(const std::vector<uint8_t>& d) {
    M m;
    ros::serialization::IStream s(const_cast<uint8_t*>(d.data()), static_cast<uint32_t>(d.size()));
    ros::serialization::deserialize(s, m);
    return m;
}

// What ReferenceOutputConsumer publishes for one output event of the runtime.
struct Published {
    ::state_machine::Event event;
    mrt::reference::ReferenceStatus status;      // PUBLISH_STATUS
    mrt::reference::AnalyticReference analytic;  // PUBLISH_ACTIVE_ANALYTIC
    mrt::reference::SampledReference sampled;    // PUBLISH_ACTIVE_SAMPLED
};

class NodePath {
   public:
    explicit NodePath(const mrt::ReferenceTrajectoryConfig& config) {
        runtime_.setConfig(config);
    }

    // ReferenceInputProducer's callback for the request, `receipt_sec` being ros::Time::now() in
    // it. Returns the lines the harness traces for it.
    std::string receive(const Record& r, double receipt_sec) {
        std::string trace;
        switch (r.kind) {
            case kAnalyticRequest:
                if (runtime_.acceptAnalytic(mrt::toCore(decode<msgs::AnalyticReference>(r.data)))) {
                    trace +=
                        post(mrt::event_type::ANALYTIC_RECEIVED, "analytic_reference", receipt_sec);
                } else {
                    trace += "  rejected\n";
                }
                break;
            case kSampledRequest:
                if (runtime_.acceptSampled(mrt::toCore(decode<msgs::SampledReference>(r.data)))) {
                    trace +=
                        post(mrt::event_type::SAMPLED_RECEIVED, "sampled_reference", receipt_sec);
                } else {
                    trace += "  rejected\n";
                }
                break;
            case kResetRequest:
                runtime_.reset();
                trace += post(mrt::event_type::RESET_REQUESTED, "reset", receipt_sec);
                break;
            default:
                throw std::runtime_error("unknown record kind");
        }
        return trace;
    }

    // The main loop after ros::spinOnce(): update the runtime, then dispatch its output events.
    std::vector<Published> update(double now_sec) {
        runtime_.update(now_sec);
        std::vector<Published> published;
        for (const auto& e : runtime_.stateMachine().currentOutputEvents()) {
            Published p;
            p.event = e;
            // ReferenceOutputConsumer::handle
            if (e.id == mrt::output_event_type::PUBLISH_STATUS) {
                p.status = runtime_.makeStatus(e.timestamp > 0.0 ? e.timestamp : now_sec);
            } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
                p.analytic = runtime_.activeAnalyticMessage();
            } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_SAMPLED) {
                p.sampled = runtime_.activeSampledMessage();
            }
            published.push_back(std::move(p));
        }
        return published;
    }

    mrt::ReferenceTrajectoryRuntime& runtime() {
        return runtime_;
    }

   private:
    // ReferenceInputProducer::post
    std::string post(uint32_t id, const char* source, double receipt_sec) {
        ::state_machine::Event event(id, ::state_machine::EventTimestamp{receipt_sec});
        event.source = source;
        const auto status = runtime_.postEvent(std::move(event));
        char line[160];
        std::snprintf(line, sizeof line, "  post %u %s %s\n", id, source,
                      status.ok() ? "ok" : status.message.c_str());
        return line;
    }

    mrt::ReferenceTrajectoryRuntime runtime_;
};

}  // namespace reference_replay
