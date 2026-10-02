#pragma once
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <state_machine/runtime/event_dispatcher.hpp>
#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/uav/nmpc_tracking_backend.h"
namespace px4_multirotor_controller {
// The original ROS NMPC dispatcher, shared by both transport adapters.
// Snapshot on the owner thread; one worker, one pending request, reject busy.
class NmpcExecution {
public:
 using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;
 using Clock = std::function<Time()>;
 using DiagnosticSink = std::function<void(uint64_t, const Time&, const UavNmpcTrackingBackend&, bool)>;
 struct Request { uint64_t sequence{0}; uint64_t generation{0}; Time now; SensorData sensor; ControllerConfig config; std::vector<Se3Reference> references; };
 using Compute = std::function<NmpcSolveResult(const Request&)>;
 NmpcExecution(DroneController&, Clock, EventSink, Compute compute = {});
 ~NmpcExecution();
 void start();
 void stop();
 bool handle(const ::state_machine::Event&);
 void setDiagnosticSink(DiagnosticSink sink) { diagnostic_sink_ = std::move(sink); }
private:
 void workerLoop(); void reject(uint64_t, uint64_t, int); void postResultEvent(uint64_t, uint64_t, bool);
 DroneController& controller_; Clock clock_; EventSink event_sink_; DiagnosticSink diagnostic_sink_;
 Compute compute_;
 UavNmpcTrackingBackend backend_;
 std::mutex mutex_; std::condition_variable condition_; std::thread worker_;
 bool stop_{false}; bool busy_{false}; bool has_pending_{false}; Request pending_;
};
}
