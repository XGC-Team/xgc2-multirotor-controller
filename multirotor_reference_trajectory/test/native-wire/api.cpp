#include "multirotor_reference_trajectory/reference_wire.hpp"

#include <cassert>

namespace {
struct Time { uint32_t sec = 0; uint32_t nsec = 0; };
struct Header { uint32_t seq = 0; Time stamp; std::string frame_id; };
struct Vector3 { double x = 0; double y = 0; double z = 0; };
struct Quaternion { double x = 0; double y = 0; double z = 0; double w = 1; };
struct Pose { Vector3 position; Quaternion orientation; };
struct Analytic {
  Header header;
  uint32_t request_id = 0, trajectory_id = 0, revision = 0, flags = 0;
  Time start_time;
  uint16_t analytic_type = 0;
  double duration = 0;
  Pose origin;
  std::vector<double> params;
};
}  // namespace

int main() {
  Analytic in;
  in.header.seq = 7;
  in.header.stamp = {12, 345};
  in.header.frame_id = "world";
  in.request_id = 1;
  in.trajectory_id = 2;
  in.revision = 3;
  in.flags = 4;
  in.start_time = {15, 678};
  in.analytic_type = 9;
  in.duration = 2.5;
  in.origin.position = {1, 2, 3};
  in.origin.orientation = {0.1, 0.2, 0.3, 0.9};
  in.params = {4.5, 6.5};
  const auto bytes = xgc_ref_wire::encode_analytic(in);
  Analytic out;
  assert(xgc_ref_wire::decode_analytic(bytes.data(), bytes.size(), out));
  assert(out.header.seq == in.header.seq && out.header.stamp.sec == 12 &&
         out.header.stamp.nsec == 345 && out.header.frame_id == "world");
  assert(out.request_id == 1 && out.trajectory_id == 2 && out.revision == 3);
  assert(out.start_time.sec == 15 && out.start_time.nsec == 678);
  assert(out.origin.position.x == 1 && out.origin.orientation.w == 0.9);
  assert(out.params == in.params);
  auto truncated = bytes;
  truncated.pop_back();
  assert(!xgc_ref_wire::decode_analytic(truncated.data(), truncated.size(), out));
  auto trailing = bytes;
  trailing.push_back(0);
  assert(!xgc_ref_wire::decode_analytic(trailing.data(), trailing.size(), out));
  return 0;
}
