// The payloads are plain old data in C++ too: the host copies them as bytes.
#include <cstdio>
#include <type_traits>

#include "multirotor_reference_trajectory/payloads.h"

template <class T>
constexpr bool plain() {
    return std::is_standard_layout<T>::value && std::is_trivially_copyable<T>::value;
}

static_assert(plain<xgc2_px4_reference_header_v1>(), "header");
static_assert(plain<xgc2_px4_reference_analytic_v1>(), "analytic");
static_assert(plain<xgc2_px4_reference_point_v1>(), "point");
static_assert(plain<xgc2_px4_reference_sampled_v1>(), "sampled");
static_assert(plain<xgc2_px4_reference_status_v1>(), "status");
static_assert(plain<xgc2_px4_reference_reset_v1>(), "reset");

int main() {
    std::puts("reference payload layout (C++) ok");
    return 0;
}
