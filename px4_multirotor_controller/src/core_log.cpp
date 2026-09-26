#include "px4_multirotor_controller/common/core_log.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace px4_multirotor_controller {
namespace {

void stderrSink(LogLevel level, const char* message) {
    const char* tag = level == LogLevel::kError ? "ERROR" : level == LogLevel::kWarn ? "WARN" : "INFO";
    std::fprintf(stderr, "[%s] %s\n", tag, message);
}

std::atomic<LogSink> g_sink{&stderrSink};

}  // namespace

void setLogSink(LogSink sink) {
    g_sink.store(sink != nullptr ? sink : &stderrSink);
}

void logMessage(LogLevel level, const char* message) {
    g_sink.load()(level, message);
}

void logFormat(LogLevel level, const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof buffer, format, args);
    va_end(args);
    logMessage(level, buffer);
}

}  // namespace px4_multirotor_controller
