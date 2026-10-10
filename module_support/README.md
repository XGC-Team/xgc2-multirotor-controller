# module_support

Shared by the two xgc2-module modules of this repository, `px4_multirotor_reference` and
`px4_multirotor_controller`. It is not a catkin package and nothing in it is installed.

| File | Used by | What |
|---|---|---|
| `include/xgc2/module.h` | modules, tests | The module ABI header (ABI major 2), a copy of `xgc2_module.h` of the design record and of `include/xgc2/module.h` of the xgc2-module repository. Modules are built against this copy until xgc2-module ships the header as an SDK package; keep it identical to the upstream file. |
| `include/module_support/json_config.hpp` | modules | The JSON object handed to `create`/`configure`, read by key path (`a/b` is `{"a": {"b": ...}}`, the ROS parameter spelling). Every key a module reads is remembered, so a key it never reads (a typo, or a key of another module) is reported instead of silently leaving a default in place. |
| `include/module_support/test_host.hpp` | tests only | An in-test host implementing `xgc2_host_api` for one module instance: the test pushes input samples, sets the clock and calls `step()`, so a run is deterministic. A slot from `write_begin` is filled with `0xA5` (a real slot holds stale data), `limitOutput()` makes a port refuse writes like a full event queue, `wake()` is counted. `ModuleLibrary` loads a module with `dlopen` the way the host does. |
| `exports.map` | modules | Linker version script: a module exports `xgc2_module_entry` and nothing else. |
