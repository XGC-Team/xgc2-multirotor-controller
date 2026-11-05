#include "px4_multirotor_controller/driver/nmpc_execution.h"
#include <utility>
#include <cmath>
namespace px4_multirotor_controller {
NmpcExecution::NmpcExecution(DroneController& controller, Clock clock, Completion on_result, Compute compute)
 : controller_(controller), clock_(std::move(clock)), on_result_(std::move(on_result)), compute_(std::move(compute)),
   worker_([this, entered = false, entered_generation = uint64_t{0}](const Request& request) mutable {
               return solve(request, entered, entered_generation);
           },
           [this](const NmpcSolveResult& result) { return controller_.nmpcResultBuffer().store(result); },
           [this](const NmpcSolveResult& result) { notifyResult(result.control_generation); }) {}
NmpcExecution::~NmpcExecution() { stop(); }
void NmpcExecution::start() { worker_.start(); }
void NmpcExecution::stop() {
 worker_.close();
 controller_.nmpcResultBuffer().invalidateGeneration();
 worker_.join();
 backend_.exit();
}
bool NmpcExecution::handle(const FlightNmpcRequest& effect) {
    // A raw nonfinite event time cannot be represented by the original Time.
    if (!std::isfinite(effect.timestamp)) return true;

    const uint64_t sequence = effect.sequence;
    const uint64_t generation = effect.generation;
    if (!controller_.nmpcResultBuffer().isGenerationActive(generation)) return true;
    const ControllerConfig config = controller_.getConfig();
    const Time now(effect.timestamp > 0.0 ? effect.timestamp : clock_().toSec());
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

    if (!worker_.trySubmit(std::move(request))) {
        // A rejection can wake the owner synchronously. Publish and notify
        // without holding the handoff mutex so that owner may re-enter it.
        reject(sequence, generation, nmpc_solver_status::kDispatcherBusy);
        return true;
    }
    return true;
}

NmpcSolveResult NmpcExecution::solve(const Request& request, bool& entered, uint64_t& entered_generation) {
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
    return result;
}

void NmpcExecution::reject(uint64_t sequence, uint64_t generation, int solver_status) {
    NmpcSolveResult result;
    result.sequence = sequence;
    result.control_generation = generation;
    result.success = false;
    result.solver_status = solver_status;
    result.stamp = clock_();
    if (controller_.nmpcResultBuffer().store(result)) notifyResult(generation);
}

void NmpcExecution::notifyResult(uint64_t generation) {
    if (on_result_ && controller_.nmpcResultBuffer().isGenerationActive(generation))
        on_result_();
}


}
