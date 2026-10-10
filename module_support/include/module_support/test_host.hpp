// An in-test host for xgc2-module modules (tests only).
//
// Implements xgc2_host_api for one module instance without the real host: the test pushes input
// samples, advances the clock and calls step() itself, so a run is deterministic. It follows the
// rules of xgc2/module.h that matter to a module:
//  - read_latest/read_next/changed see the inputs as of the start of the step;
//  - write_begin hands out a slot filled with 0xA5 (a real slot holds stale data, a module must
//    not rely on zeroed memory), a second write_begin before the commit returns NULL;
//  - an output returns NULL from write_begin once its test-set capacity is used up (limitOutput),
//    as a full event queue does;
//  - wake() may be called from any thread; waitWake() lets the test wait for it.
#pragma once

#include <dlfcn.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "xgc2/module.h"

namespace module_support {

// A module shared library loaded with dlopen, as the host loads it.
class ModuleLibrary {
   public:
    explicit ModuleLibrary(const std::string& path) {
        handle_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (handle_ == nullptr)
            throw std::runtime_error(std::string("dlopen: ") + dlerror());
        auto entry =
            reinterpret_cast<xgc2_module_entry_fn>(dlsym(handle_, XGC2_MODULE_ENTRY_SYMBOL));
        if (entry == nullptr)
            throw std::runtime_error("module does not export xgc2_module_entry");
        desc_ = entry();
    }
    ~ModuleLibrary() {
        if (handle_ != nullptr)
            dlclose(handle_);
    }
    ModuleLibrary(const ModuleLibrary&) = delete;
    ModuleLibrary& operator=(const ModuleLibrary&) = delete;
    const xgc2_module_desc* desc() const {
        return desc_;
    }

   private:
    void* handle_{nullptr};
    const xgc2_module_desc* desc_{nullptr};
};

struct Sample {
    int64_t stamp_ns{0};
    std::vector<uint8_t> bytes;

    template <class T>
    T as() const {
        if (bytes.size() != sizeof(T))
            throw std::runtime_error("payload size mismatch");
        T value;
        std::memcpy(&value, bytes.data(), sizeof(T));
        return value;
    }
};

class TestHost {
   public:
    explicit TestHost(const xgc2_module_desc* desc) : desc_(desc), ports_(desc->port_count) {
        api_.abi_major = XGC2_MODULE_ABI_MAJOR;
        api_.abi_minor = XGC2_MODULE_ABI_MINOR;
        api_.write_begin = &TestHost::writeBegin;
        api_.write_commit = &TestHost::writeCommit;
        api_.write_abort = &TestHost::writeAbort;
        api_.read_latest = &TestHost::readLatest;
        api_.read_next = &TestHost::readNext;
        api_.changed = &TestHost::changedPort;
        api_.now_ns = &TestHost::nowNs;
        api_.wake = &TestHost::wakeHost;
        api_.set_period_ns = &TestHost::setPeriod;
        api_.log = &TestHost::logMessage;
        api_.report = &TestHost::reportHealth;
        uint32_t input_index = 0;
        for (uint32_t i = 0; i < desc->port_count; ++i) {
            ports_[i].desc = &desc->ports[i];
            if (desc->ports[i].direction == XGC2_PORT_IN)
                ports_[i].input_index = input_index++;
        }
    }
    ~TestHost() {
        destroy();
    }
    TestHost(const TestHost&) = delete;
    TestHost& operator=(const TestHost&) = delete;

    const xgc2_module_desc* desc() const {
        return desc_;
    }
    const xgc2_host_api* api() const {
        return &api_;
    }

    uint32_t port(const std::string& name) const {
        for (uint32_t i = 0; i < ports_.size(); ++i) {
            if (name == ports_[i].desc->name)
                return i;
        }
        throw std::runtime_error("no port " + name);
    }

    // ---- clock ----
    void setNow(int64_t ns) {
        now_ns_.store(ns);
    }
    int64_t now() const {
        return now_ns_.load();
    }

    // ---- lifecycle ----
    xgc2_status create(const std::string& json) {
        config_ = json;
        const xgc2_config config{config_.c_str(), config_.size()};
        return desc_->create(&api_, this, &config, &instance_);
    }
    xgc2_status configure(const std::string& json) {
        config_ = json;
        const xgc2_config config{config_.c_str(), config_.size()};
        return desc_->configure(instance_, &config);
    }
    xgc2_status start() {
        return desc_->start(instance_);
    }
    xgc2_status stop() {
        return desc_->stop(instance_);
    }
    void destroy() {
        if (instance_ != nullptr) {
            desc_->destroy(instance_);
            instance_ = nullptr;
        }
    }
    xgc2_instance* instance() const {
        return instance_;
    }

    // One step at the current clock. changed_inputs is derived from the samples pushed since the
    // previous step.
    xgc2_status step(uint32_t reasons = XGC2_STEP_TIMER) {
        xgc2_step_ctx ctx{};
        ctx.now_ns = now_ns_.load();
        ctx.step_index = step_index_++;
        ctx.reasons = reasons;
        for (Port& p : ports_) {
            if (p.desc->direction != XGC2_PORT_IN)
                continue;
            std::lock_guard<std::mutex> lock(mutex_);
            p.changed_in_step = p.dirty;
            p.dirty = false;
            if (p.changed_in_step)
                ctx.changed_inputs |= uint64_t{1} << p.input_index;
            p.step_latest = p.latest;
        }
        in_step_ = true;
        const xgc2_status status = desc_->step(instance_, &ctx);
        in_step_ = false;
        for (Port& p : ports_)
            p.held.clear();
        return status;
    }

    // ---- inputs ----
    // False when an event input is full (the sample is dropped, like the real host).
    bool push(uint32_t index, const void* data, uint32_t size, int64_t stamp_ns) {
        Port& p = ports_.at(index);
        if (p.desc->direction != XGC2_PORT_IN || size != p.desc->size) {
            throw std::runtime_error(std::string("bad push to ") + p.desc->name);
        }
        Sample sample;
        sample.stamp_ns = stamp_ns;
        sample.bytes.assign(static_cast<const uint8_t*>(data),
                            static_cast<const uint8_t*>(data) + size);
        std::lock_guard<std::mutex> lock(mutex_);
        if (p.desc->kind == XGC2_PORT_STATE) {
            p.latest = std::move(sample);
            p.has_latest = true;
            p.dirty = true;
            ++p.sequence;
            return true;
        }
        if (p.events.size() >= p.desc->queue_depth)
            return false;
        p.events.push_back(std::move(sample));
        ++p.sequence;
        return true;
    }
    template <class T>
    bool push(const std::string& name, const T& value, int64_t stamp_ns) {
        return push(port(name), &value, sizeof(T), stamp_ns);
    }

    // ---- outputs ----
    const std::vector<Sample>& outputs(uint32_t index) const {
        return ports_.at(index).committed;
    }
    const std::vector<Sample>& outputs(const std::string& name) const {
        return outputs(port(name));
    }
    template <class T>
    std::vector<T> outputsAs(const std::string& name) const {
        std::vector<T> result;
        for (const Sample& sample : outputs(name))
            result.push_back(sample.as<T>());
        return result;
    }
    void clearOutputs() {
        for (Port& p : ports_)
            p.committed.clear();
    }
    // The output accepts `count` more samples; after that write_begin returns NULL (counted in
    // refusedWrites()).
    void limitOutput(const std::string& name, size_t count) {
        Port& p = ports_.at(port(name));
        p.write_limit = p.committed.size() + count;
    }
    uint64_t refusedWrites(const std::string& name) const {
        return ports_.at(port(name)).refused;
    }

    // ---- observation ----
    uint64_t wakeCount() const {
        return wakes_.load();
    }
    // The thread that called wake() last.
    std::thread::id lastWakeThread() const {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        return wake_thread_;
    }
    // Waits until wakeCount() exceeds `seen`; false on timeout.
    bool waitWake(uint64_t seen, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(wake_mutex_);
        return wake_cv_.wait_for(lock, timeout, [&] { return wakes_.load() > seen; });
    }
    int64_t periodNs() const {
        return period_ns_.load();
    }
    std::vector<std::pair<int, std::string>> logs() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return logs_;
    }
    bool logged(int level, const std::string& fragment) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& entry : logs_) {
            if (entry.first == level && entry.second.find(fragment) != std::string::npos)
                return true;
        }
        return false;
    }
    std::vector<std::pair<int, std::string>> reports() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return reports_;
    }
    std::string lastReportDetail() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return reports_.empty() ? std::string() : reports_.back().second;
    }

   private:
    struct Port {
        const xgc2_port_desc* desc{nullptr};
        uint32_t input_index{0};
        // input state
        Sample latest;
        Sample step_latest;
        bool has_latest{false};
        bool dirty{false};
        bool changed_in_step{false};
        uint64_t sequence{0};
        std::deque<Sample> events;  // pending event samples (front is next)
        std::deque<Sample> held;    // event samples read in the current step (views stay valid)
        uint64_t read_sequence{0};
        // output state
        std::vector<uint8_t> slot;
        bool writing{false};
        std::vector<Sample> committed;
        size_t write_limit{SIZE_MAX};
        uint64_t refused{0};
    };

    static TestHost& self(void* ctx) {
        return *static_cast<TestHost*>(ctx);
    }

    static void* writeBegin(void* ctx, uint32_t index) {
        TestHost& h = self(ctx);
        if (index >= h.ports_.size())
            return nullptr;
        Port& p = h.ports_[index];
        if (p.desc->direction != XGC2_PORT_OUT || p.writing)
            return nullptr;
        if (p.committed.size() >= p.write_limit) {
            ++p.refused;
            return nullptr;
        }
        p.slot.assign(p.desc->size, 0xA5);
        p.writing = true;
        return p.slot.data();
    }
    static xgc2_status writeCommit(void* ctx, uint32_t index, int64_t stamp_ns) {
        TestHost& h = self(ctx);
        if (index >= h.ports_.size() || !h.ports_[index].writing)
            return XGC2_ERR_STATE;
        Port& p = h.ports_[index];
        p.writing = false;
        p.committed.push_back(Sample{stamp_ns, p.slot});
        return XGC2_OK;
    }
    static void writeAbort(void* ctx, uint32_t index) {
        TestHost& h = self(ctx);
        if (index < h.ports_.size())
            h.ports_[index].writing = false;
    }
    static xgc2_status readLatest(void* ctx, uint32_t index, xgc2_sample_view* out) {
        TestHost& h = self(ctx);
        if (!h.in_step_ || index >= h.ports_.size())
            return XGC2_ERR_STATE;
        Port& p = h.ports_[index];
        if (p.desc->direction != XGC2_PORT_IN || p.desc->kind != XGC2_PORT_STATE)
            return XGC2_ERR_INVALID;
        if (!p.has_latest)
            return XGC2_ERR_NODATA;
        out->data = p.step_latest.bytes.data();
        out->size = static_cast<uint32_t>(p.step_latest.bytes.size());
        out->seq = p.sequence;
        out->stamp_ns = p.step_latest.stamp_ns;
        return XGC2_OK;
    }
    static xgc2_status readNext(void* ctx, uint32_t index, xgc2_sample_view* out) {
        TestHost& h = self(ctx);
        if (!h.in_step_ || index >= h.ports_.size())
            return XGC2_ERR_STATE;
        Port& p = h.ports_[index];
        if (p.desc->direction != XGC2_PORT_IN || p.desc->kind != XGC2_PORT_EVENT)
            return XGC2_ERR_INVALID;
        std::lock_guard<std::mutex> lock(h.mutex_);
        if (p.events.empty())
            return XGC2_ERR_NODATA;
        // The view stays valid until the end of the step: keep the sample until then.
        p.held.push_back(std::move(p.events.front()));
        p.events.pop_front();
        const Sample& sample = p.held.back();
        out->data = sample.bytes.data();
        out->size = static_cast<uint32_t>(sample.bytes.size());
        out->seq = ++p.read_sequence;
        out->stamp_ns = sample.stamp_ns;
        return XGC2_OK;
    }
    static int changedPort(void* ctx, uint32_t index) {
        TestHost& h = self(ctx);
        if (!h.in_step_ || index >= h.ports_.size())
            return 0;
        return h.ports_[index].changed_in_step ? 1 : 0;
    }
    static int64_t nowNs(void* ctx) {
        return self(ctx).now_ns_.load();
    }
    static void wakeHost(void* ctx) {
        TestHost& h = self(ctx);
        {
            std::lock_guard<std::mutex> lock(h.wake_mutex_);
            h.wake_thread_ = std::this_thread::get_id();
            h.wakes_.fetch_add(1);
        }
        h.wake_cv_.notify_all();
    }
    static void setPeriod(void* ctx, int64_t period_ns) {
        self(ctx).period_ns_.store(period_ns);
    }
    static void logMessage(void* ctx, int level, const char* message) {
        TestHost& h = self(ctx);
        std::lock_guard<std::mutex> lock(h.mutex_);
        h.logs_.emplace_back(level, message == nullptr ? "" : message);
    }
    static void reportHealth(void* ctx, int health, const char* detail) {
        TestHost& h = self(ctx);
        std::lock_guard<std::mutex> lock(h.mutex_);
        h.reports_.emplace_back(health, detail == nullptr ? "" : detail);
    }

    const xgc2_module_desc* desc_;
    xgc2_host_api api_{};
    xgc2_instance* instance_{nullptr};
    std::vector<Port> ports_;
    std::string config_;
    std::atomic<int64_t> now_ns_{0};
    std::atomic<int64_t> period_ns_{0};
    std::atomic<uint64_t> wakes_{0};
    uint64_t step_index_{0};
    bool in_step_{false};
    mutable std::mutex mutex_;
    mutable std::mutex wake_mutex_;
    std::condition_variable wake_cv_;
    std::thread::id wake_thread_;
    std::vector<std::pair<int, std::string>> logs_;
    std::vector<std::pair<int, std::string>> reports_;
};

}  // namespace module_support
