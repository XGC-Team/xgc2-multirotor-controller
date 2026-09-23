# P06 lifter regression and boundary probe

Task: [xgc2-harness#157](https://github.com/XGC-Team/xgc2-harness/issues/157).
Baseline: `noetic@a920325f55f5ea277b26ebf953232076e582ae09`.
This directory owns tests only; both execution modes include the product's
`trajectory_lifter.h` and `mpc_trajectory_buffer.h`. There is no second runtime lifter.

## What this patch changes

An ignored velocity was still added to an active position. For a position-only
reference `p=(1,2,3)`, ignored `v=(4,5,6)`, and elapsed time 0.25 s, the baseline
outputs `(2,3.25,4.5)` instead of holding `(1,2,3)`. The patch removes ignored
velocity contributions, as the existing implementation already does for ignored
acceleration. Masks, active PVA, SCE 3523, receipt-origin timing, and takeover
policy are otherwise retained. An ignored derivative has zero contribution; this
is not an estimate of the vehicle's actual velocity or acceleration.

## Native headers and dependencies

In an already provisioned ROS Noetic development environment, with roscpp,
ros1_utils, Eigen3 and JsonCpp available:

```sh
# Run from the repository root. Use a NEW output directory outside the source.
cmake -S px4_multirotor_controller/test/p06 -B /tmp/p06-native-build
cmake --build /tmp/p06-native-build --parallel 1
ctest --test-dir /tmp/p06-native-build --output-on-failure
```

This standalone target does not start a ROS node or require acados. It is not
automatically registered in the existing package-wide catkin test suite. Full
package tests, CI, Custom1 event timing and live publication need separate runs.

## Explicit offline dependency seam

When ROS/Eigen are unavailable, a narrower check can be requested explicitly:

```sh
python3 px4_multirotor_controller/test/p06/run_dependency_seam.py \
  --package-root "$PWD/px4_multirotor_controller" \
  --output-dir /tmp/p06-seamed-run
```

The output directory must not already exist or be inside the source package.
The script logs commands, compiler, environment, hashes, exit codes and raw
stdout/stderr. It compiles the actual two product headers, but substitutes the
small ROS time, Eigen vector and data-type surface. This is **not** a native ROS
build, ABI test, clock/reset test, simulator run, or aircraft validation. The
native CMake target never uses these doubles. GCC's undefined-behavior sanitizer
is enabled in the seam run; no address-sanitizer or thread-sanitizer claim is made.

For a red/green comparison, run the SAME probe and runner from this patch twice,
changing `--package-root` to a separate baseline checkout and the patched checkout.
Record both outputs; the baseline is expected to return nonzero. The mask sweep
has all 512 P/V/A ignore-bit combinations, seed 157, a fixed integer-to-double
mapping, and 1e-10 absolute numerical tolerance. The tolerance is a test oracle,
not a hardware tracking-error bound or a frozen T01 seam tolerance.

## Interpreting the output

There are 35 fixed checks and 512 mask cases. `regression/*` tests mask behavior
and compatibility. `characterization/*` deliberately records limitations:
position/velocity/acceleration seam jumps, late effective-vs-receipt origins,
duplicate receipt rebasing, last-arrival overwrite, future/zero origins, clock
rewind, unlimited extrapolation, frame/force flags and readiness boundaries.
A PASS on a characterization means the counterexample was reproduced; it does
not mean the behavior is safe, desirable, or a closed acceptance gate. The
buffer cases inject callback-shaped samples but do not execute the ROS callback.

T27/T32's existing receipt-origin policy remains unchanged. Do not infer
cross-plan C1/C2 continuity or one-beat delay compensation from a segment's
polynomial. T01 must freeze timing, continuity and validity contracts before
any new seam/acceptance algorithm. T28/T33 reserves T-RO SMC for a future Custom2;
this patch does not integrate SMC, attitude control or rate INDI into Custom1.

The run evidence and proposed (NOT frozen) `P06-20260924-01` interface belong in
`lxk36/paper-dmpc/research/tro-26-0979-v1/P06/attempt-01/`, linked from #157.
Independent review is pending; self-checks must not be reported as independent.
