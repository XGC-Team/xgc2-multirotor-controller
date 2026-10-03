#include "px4_multirotor_controller/driver/nmpc_execution.h"
#include <utility>
#include <cmath>
namespace px4_multirotor_controller {
NmpcExecution::NmpcExecution(DroneController& controller, Clock clock, EventSink sink, Compute compute)
 : controller_(controller), clock_(std::move(clock)), event_sink_(std::move(sink)), compute_(std::move(compute)) {}
NmpcExecution::~NmpcExecution() { stop(); }
void NmpcExecution::start() {
 std::lock_guard<std::mutex> lock(mutex_);
 if (worker_.joinable()) return;
 stop_ = false; busy_ = false; has_pending_ = false; pending_ = Request{};
 worker_ = std::thread(&NmpcExecution::workerLoop, this);
}
void NmpcExecution::stop() {
 { std::lock_guard<std::mutex> lock(mutex_); stop_ = true; has_pending_ = false; pending_ = Request{}; }
 controller_.nmpcResultBuffer().invalidateGeneration();
 condition_.notify_all();
 if (worker_.joinable()) worker_.join();
 backend_.exit();
 { std::lock_guard<std::mutex> lock(mutex_); busy_ = false; }
}
bool NmpcExecution::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::REQUEST_NMPC_SOLVE) {
        return false;
    }

    // A raw nonfinite event time cannot be represented by the original Time.
    if (!std::isfinite(event.timestamp)) return true;

    const uint64_t sequence = event.correlation_id;
    const auto token = event.payload.find("control_generation");
    if (token == event.payload.end() || !std::holds_alternative<int64_t>(token->second)) return true;
    const uint64_t generation = static_cast<uint64_t>(std::get<int64_t>(token->second));
    if (!controller_.nmpcResultBuffer().isGenerationActive(generation)) return true;
    const ControllerConfig config = controller_.getConfig();
    const Time now(event.timestamp > 0.0 ? event.timestamp : clock_().toSec());
    Request request;
    request.sequence = sequence;
    request.generation = generation;
    request.config = config;
    request.now = now;
    request.sensor = controller_.getSensorData();

    const double stage_dt =
        config.nmpc.prediction_horizon / static_cast<double>(UavNmpcSolver::horizonSteps());
    if (!controller_.activeTrajectoryCache().sampleHorizon(
            now, stage_dt, UavNmpcSolver::horizonSteps(), config.nmpc.gravity,
            request.references)) {
        reject(sequence, generation, nmpc_solver_status::kReferenceSamplingFailed);
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_ || busy_ || has_pending_) {
            reject(sequence, generation, nmpc_solver_status::kDispatcherBusy);
            return true;
        }
        pending_ = std::move(request);
        has_pending_ = true;
    }
    condition_.notify_one();
    return true;
}

void NmpcExecution::workerLoop() {
    bool entered = false;
    uint64_t entered_generation = 0;
    while (true) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || has_pending_; });
            if (stop_) return;
            request = std::move(pending_);
            has_pending_ = false;
            busy_ = true;
        }
        NmpcSolveResult result;
        if (controller_.nmpcResultBuffer().isGenerationActive(request.generation)) {
            if (compute_) {
                result = compute_(request);
            } else {
                backend_.configure(request.config);
                if (entered_generation != request.generation) {
                    backend_.exit(); entered = false; entered_generation = request.generation;
                }
                if (!entered) entered = backend_.enter(request.sensor);
                if (entered) {
                    result.success = backend_.compute(request.sensor, request.references, request.now, result.target);
                    result.solver_status = backend_.status();
                    result.solve_time_ms = backend_.solveTimeMs();
                    if (diagnostic_sink_ && controller_.nmpcResultBuffer().isGenerationActive(request.generation)) diagnostic_sink_(request.sequence, request.now, backend_, result.success);
                } else {
                    result.solver_status = nmpc_solver_status::kBackendUnavailable;
                }
            }
        }
        result.sequence = request.sequence;
        result.control_generation = request.generation;
        result.stamp = request.now;
        if (controller_.nmpcResultBuffer().store(result)) {
            postResultEvent(request.sequence, request.generation, result.success);
        }
        { std::lock_guard<std::mutex> lock(mutex_); busy_ = false; }
    }
}

void NmpcExecution::reject(uint64_t sequence, uint64_t generation, int solver_status) {
    NmpcSolveResult result;
    result.sequence = sequence;
    result.control_generation = generation;
    result.success = false;
    result.solver_status = solver_status;
    result.stamp = clock_();
    if (controller_.nmpcResultBuffer().store(result)) postResultEvent(sequence, generation, false);
}

void NmpcExecution::postResultEvent(uint64_t sequence, uint64_t generation, bool success) {
    if (!event_sink_ || !controller_.nmpcResultBuffer().isGenerationActive(generation)) {
        return;
    }
    ::state_machine::Event event(
        success ? event_type::INPUT_NMPC_SOLVE_SUCCEEDED : event_type::INPUT_NMPC_SOLVE_FAILED,
        ::state_machine::EventTimestamp{clock_().toSec()});
    event.source = "nmpc_output_consumer";
    event.correlation_id = sequence;
    event.payload["control_generation"] = static_cast<int64_t>(generation);
    (void)event_sink_(std::move(event));
}


}
