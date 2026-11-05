#include "multirotor_reference_trajectory/output/reference_output_worker.h"

#include <utility>

namespace multirotor_reference_trajectory {

ReferenceOutputWorker::~ReferenceOutputWorker() { stop(); }

void ReferenceOutputWorker::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;
    running_ = true;
    try {
        worker_ = std::thread(&ReferenceOutputWorker::run, this);
    } catch (...) {
        running_ = false;
        throw;
    }
}

void ReferenceOutputWorker::stop() {
    std::queue<ReferencePublication> discarded;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        pending_.swap(discarded);
    }
    ready_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void ReferenceOutputWorker::post(ReferencePublication publication) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        pending_.push(std::move(publication));
    }
    ready_.notify_one();
}

void ReferenceOutputWorker::run() {
    for (;;) {
        ReferencePublication publication;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [this] { return !running_ || !pending_.empty(); });
            if (!running_) return;
            publication = std::move(pending_.front());
            pending_.pop();
        }
        try {
            std::visit([&](const auto& message) { publication.publisher.publish(message); },
                       publication.message);
        } catch (...) {
            // The original asynchronous worker isolated failed publications.
        }
    }
}

}  // namespace multirotor_reference_trajectory
