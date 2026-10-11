// A thread that runs the closures handed to run(), one at a time (modules and tests).
//
// The host calls an instance's create, configure, start, step, stop and destroy one at a time, but
// from any of its worker threads, and a different one each time. A module that wraps a component
// bound to the thread that first used it (the state machine library's update() is) gives that
// component an owner thread and runs everything that touches it there: the step reads its inputs
// with the host API on the calling thread, hands the update to the owner thread with run(), and
// writes the outputs on the calling thread again. run() returns when the closure has finished, so
// the calling thread and the owner thread never work at the same time and every write of one is
// visible to the other.
#pragma once

#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>

namespace module_support {

class OwnerThread {
   public:
    OwnerThread() : thread_([this] { loop(); }) {}

    // Stops the thread; no run() may be in progress.
    ~OwnerThread() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        work_.notify_all();
        thread_.join();
    }

    OwnerThread(const OwnerThread&) = delete;
    OwnerThread& operator=(const OwnerThread&) = delete;

    bool onOwnerThread() const {
        return std::this_thread::get_id() == thread_.get_id();
    }

    // Runs f on the owner thread, waits for it and returns its result. An exception f throws is
    // thrown again in the caller. Called on the owner thread, run() calls f directly.
    template <class F>
    auto run(F&& f) -> decltype(f()) {
        using Result = decltype(f());
        if (onOwnerThread())
            return f();
        std::lock_guard<std::mutex> one_caller(caller_mutex_);
        std::optional<std::conditional_t<std::is_void_v<Result>, int, Result>> result;
        std::exception_ptr error;
        std::function<void()> job = [&] {
            try {
                if constexpr (std::is_void_v<Result>) {
                    f();
                } else {
                    result.emplace(f());
                }
            } catch (...) {
                error = std::current_exception();
            }
        };
        {
            std::unique_lock<std::mutex> lock(mutex_);
            job_ = &job;
            done_ = false;
            work_.notify_one();
            finished_.wait(lock, [this] { return done_; });
            job_ = nullptr;
        }
        if (error)
            std::rethrow_exception(error);
        if constexpr (!std::is_void_v<Result>)
            return std::move(*result);
    }

   private:
    void loop() {
        std::unique_lock<std::mutex> lock(mutex_);
        for (;;) {
            work_.wait(lock, [this] { return stop_ || (job_ != nullptr && !done_); });
            if (job_ == nullptr || done_)
                return;  // stop requested and nothing to run
            std::function<void()>* job = job_;
            lock.unlock();
            (*job)();
            lock.lock();
            done_ = true;  // the job is not touched again; the caller owns it from here on
            finished_.notify_one();
        }
    }

    std::mutex caller_mutex_;  // one run() at a time
    std::mutex mutex_;
    std::condition_variable work_;
    std::condition_variable finished_;
    std::function<void()>* job_{nullptr};
    bool done_{true};
    bool stop_{false};
    std::thread thread_;  // last: the loop starts when everything above exists
};

}  // namespace module_support
