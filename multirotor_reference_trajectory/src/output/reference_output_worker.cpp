#include "multirotor_reference_trajectory/output/reference_output_worker.h"

#include <utility>

namespace multirotor_reference_trajectory {

ReferenceOutputWorker::~ReferenceOutputWorker() = default;
void ReferenceOutputWorker::start() { worker_.start(); }
void ReferenceOutputWorker::stop() { worker_.stop(); }
void ReferenceOutputWorker::post(ReferencePublication publication) {
    worker_.post(std::move(publication));
}

void ReferenceOutputWorker::execute(const ReferencePublication& publication) {
    std::visit([&](const auto& message) { publication.publisher.publish(message); },
               publication.message);
}

}  // namespace multirotor_reference_trajectory
