# module_support

Shared by the two xgc2-module modules of this repository, `px4_multirotor_reference` and
`px4_multirotor_controller`. It is not a catkin package and nothing in it is installed.

Both modules rely on the host's default `affinity = "sticky"`: each instance builds, updates and
destroys its thread-affine runtime on one home host worker throughout its lifecycle.

| File | Used by | What |
|---|---|---|
| `Xgc2Module::SDK` | modules, tests | The installed xgc2-module SDK supplies the ABI header, test host and module export script. |
| `include/module_support/json_config.hpp` | modules | The JSON object handed to `create`/`configure`, read by key path (`a/b` is `{"a": {"b": ...}}`, the ROS parameter spelling). Every key a module reads is remembered, so a key it never reads (a typo, or a key of another module) is reported instead of silently leaving a default in place. |
