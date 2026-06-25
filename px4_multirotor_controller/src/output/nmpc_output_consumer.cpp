#include "px4_multirotor_controller/output/nmpc_output_consumer.h"

#include <utility>

namespace px4_multirotor_controller {

NmpcOutputConsumer::NmpcOutputConsumer(DroneController& controller, EventSink event_sink)
    : controller_(controller), event_sink_(std::move(event_sink)) {
    backend_.configure(controller_.getConfig());
    worker_ = std::thread(&NmpcOutputConsumer::workerLoop, this);
}

NmpcOutputConsumer::~NmpcOutputConsumer() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    backend_.exit();
}

bool NmpcOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::REQUEST_NMPC_SOLVE) {
        return false;
    }

    const uint64_t sequence = event.correlation_id;
    const ControllerConfig config = controller_.getConfig();
    const ros::Time now(event.timestamp > 0.0 ? event.timestamp : ros::Time::now().toSec());
    Request request;
    request.sequence = sequence;
    request.now = now;
    request.sensor = controller_.getSensorData();

    const double stage_dt =
        config.nmpc.prediction_horizon / static_cast<double>(UavNmpcSolver::horizonSteps());
    if (!controller_.activeTrajectoryCache().sampleHorizon(
            now, stage_dt, UavNmpcSolver::horizonSteps(), config.nmpc.reference_timeout,
            config.nmpc.gravity, request.references)) {
        reject(sequence, nmpc_solver_status::kReferenceSamplingFailed);
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ || has_pending_) {
            reject(sequence, nmpc_solver_status::kDispatcherBusy);
            return true;
        }
        pending_ = std::move(request);
        has_pending_ = true;
    }
    condition_.notify_one();
    return true;
}

void NmpcOutputConsumer::workerLoop() {
    bool entered = false;
    while (true) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || has_pending_; });
            if (stop_) {
                return;
            }
            request = std::move(pending_);
            has_pending_ = false;
            busy_ = true;
        }

        NmpcSolveResult result;
        result.sequence = request.sequence;
        result.stamp = request.now;
        backend_.configure(controller_.getConfig());
        if (request.sequence == 1) {
            backend_.exit();
            entered = false;
        }
        if (!entered) {
            entered = backend_.enter(request.sensor);
        }
        if (entered) {
            result.success =
                backend_.compute(request.sensor, request.references, request.now, result.target);
            result.solver_status = backend_.status();
            result.solve_time_ms = backend_.solveTimeMs();
        } else {
            result.success = false;
            result.solver_status = nmpc_solver_status::kBackendUnavailable;
        }

        controller_.nmpcResultBuffer().store(result);
        postResultEvent(request.sequence, result.success);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
        }
    }
}

void NmpcOutputConsumer::reject(uint64_t sequence, int solver_status) {
    NmpcSolveResult result;
    result.sequence = sequence;
    result.success = false;
    result.solver_status = solver_status;
    result.stamp = ros::Time::now();
    controller_.nmpcResultBuffer().store(result);
    postResultEvent(sequence, false);
}

void NmpcOutputConsumer::postResultEvent(uint64_t sequence, bool success) {
    if (!event_sink_) {
        return;
    }
    ::state_machine::Event event(
        success ? event_type::INPUT_NMPC_SOLVE_SUCCEEDED : event_type::INPUT_NMPC_SOLVE_FAILED,
        ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = "nmpc_output_consumer";
    event.correlation_id = sequence;
    (void)event_sink_(std::move(event));
}

}  // namespace px4_multirotor_controller
