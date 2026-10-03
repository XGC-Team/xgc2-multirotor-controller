#include "multirotor_reference_trajectory/reference_wire_v1.h"

int main(void) {
  xgc_ref_analytic_v1 analytic = {0};
  xgc_ref_sampled_v1 sampled = {0};
  xgc_ref_status_v1 status = {0};
  xgc_ref_reset_v1 reset = {0};
  xgc_flat_ref_v1 flat = {0};
  return (int)(analytic.header.seq + sampled.points_len + status.state +
               reset.reserved + (uint64_t)flat.stamp);
}
