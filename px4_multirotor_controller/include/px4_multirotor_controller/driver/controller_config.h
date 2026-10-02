#pragma once
#include <cstdio>
#include <functional>
#include <string>
#include <variant>
#include <vector>
#include "px4_multirotor_controller/common/types.h"
namespace px4_multirotor_controller {
using ControllerParameterValue = std::variant<bool, int, double, std::string, std::vector<double>>;
class ControllerParameters {
public:
 using Getter = std::function<bool(const std::string&, ControllerParameterValue&)>;
 using Logger = std::function<void(bool, const std::string&)>;
 explicit ControllerParameters(Getter override_getter = {}, Logger logger = {}, bool owning_profile = true);
 template<class T> bool getParam(const std::string& key, T& out) const {
   ControllerParameterValue value = out;
   if (!read(key, value)) return false;
   out = std::get<T>(value); return true;
 }
 template<class T> void param(const std::string& key, T& out, const T& fallback) const {
   out = fallback; (void)getParam(key, out);
 }
 template<class... Args> void logf(bool warning, const char* format, Args... args) const {
   if (!logger_) return;
   const int n = std::snprintf(nullptr, 0, format, args...);
   if (n < 0) return;
   std::vector<char> buffer(static_cast<size_t>(n) + 1);
   std::snprintf(buffer.data(), buffer.size(), format, args...);
   logger_(warning, buffer.data());
 }
private:
 bool read(const std::string&, ControllerParameterValue&) const;
 Getter override_, profile_; Logger logger_;
};
ControllerConfig readControllerConfig(const ControllerParameters&, const std::string& uav_name = {});
}
