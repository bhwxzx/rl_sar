# rl_sar — LW

`rl_sar` currently supports the LW robot for reinforcement-learning policy
Sim2Sim verification and physical deployment. The repository keeps generic
FSM, inference, control-loop, and simulation interfaces so another robot can
be added deliberately without restoring legacy robot implementations.

[中文文档](README_CN.md)

> [!CAUTION]
> Real-robot operation can cause falls, equipment damage, or injury. Complete
> the documented Sim2Sim and deployment checks first, use mechanical support
> and a physical emergency stop, and keep an exclusion area around the robot.

## Current support

| Robot | Policy runtime | Sim2Sim | Real robot |
|---|---|---|---|
| LW | ONNX Runtime | MuJoCo / `rl_sim_LW` | ROS 2 / `rl_real_LW` |

The four tracked LW policies are:

- `policy/LW/robot_lab/leg_loco`
- `policy/LW/robot_lab/wheel_loco`
- `policy/LW/robot_lab/leg_to_wheel`
- `policy/LW/robot_lab/wheel_to_leg`

## Checkout and dependencies

Clone with the remaining joystick submodule:

```bash
git clone --recursive https://github.com/bhwxzx/rl_sar.git
cd rl_sar
```

The LW robot description is tracked directly in
`src/rl_sar_zoo/LW_description`; builds do not download a separate multi-robot
zoo. `scripts/validate_lw_description.sh` verifies its content manifest.

The supported development path uses ROS 2 Foxy or Humble on Linux. Core build
dependencies include a C++17 compiler, CMake, yaml-cpp, Eigen, Boost, TBB,
OpenSSL, GLFW, ROS 2, and the inference/MuJoCo runtimes managed by the project
setup scripts.

Inference archives are restricted to the reviewed Linux platform/version
matrix in `scripts/inference_runtime_archives.json`. The complete archive
SHA-256, extracted candidate structure, and ELF architecture must pass before
installation. A valid existing runtime with missing or different provenance is
preserved and requires an explicit reviewed upgrade; Python `torch` and
`onnxruntime` packages are independent of this project runtime directory.

## Development build

Source ROS 2 and build from the repository root:

```bash
source /opt/ros/humble/setup.bash
./build.sh
```

The build validates the vendored LW description locally; it never clones or
updates robot descriptions from the network.

Before committing maintained C++ changes, run the warning-clean validation:

```bash
scripts/validate_lw_strict_build.sh
```

It performs a fresh Debug build and the full CTest suite with
`-Wall -Wextra -Wpedantic -Werror` on maintained LW targets. Vendored MuJoCo,
joystick, and hardware-SDK sources and headers are isolated from this warning
policy; their diagnostics cannot hide failures in maintained code.

## Sim2Sim

After building and sourcing the workspace:

```bash
source install/setup.bash
ros2 run rl_sar rl_sim_LW
```

Optional TorchScript actuator models can be enabled with:

```bash
ros2 run rl_sar rl_sim_LW \
    --policy-root /absolute/path/to/policy \
    --use_actuator_net
```

Both actuator models are resolved exclusively below the selected policy root
at `LW/robot_lab/motors/{leg,foot}_actuator_net.pt`. Startup reports both
resolved paths and fails if either model is missing, cannot load, or violates
the expected six-input/one-output contract.

The simulator loads
`src/rl_sar_zoo/LW_description/mjcf/scene.xml`. The terrain scene and its
height map are tracked in the same package and are covered by a headless model
loading test.

## Real-robot deployment

Use the Chinese [quick-start guide](docs/LW_QUICK_START_CN.md) for the standard
deployment path. Build rationale, bundle verification, target-host acceptance,
operator safety behavior, troubleshooting, and the Sim2Sim-before-real gate are
defined by the authoritative
[full deployment guide](docs/LW_BUILD_DEPLOYMENT_CN.md).

The ROS 2 launch entry point is:

```bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 ros2 launch rl_sar rl_real_LW.launch.py
```

Keep `PYTHONDONTWRITEBYTECODE=1`: the verified production launch directory is
an exact manifest-bound file set and must not acquire an unverified bytecode
cache.

Do not start the real node without completing the hardware-side checks in the
deployment guide.

## Repository boundaries

- `policy/LW/`: current policy configurations and models.
- `src/rl_sar/fsm_robot/fsm_LW.hpp`: current robot FSM and factory.
- `src/rl_sar/src/rl_real_LW.cpp`: real-robot entry point.
- `src/rl_sar/src/rl_sim_LW.cpp`: LW Sim2Sim entry point.
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/`: LW hardware SDK.
- `src/rl_sar_zoo/LW_description/`: vendored LW URDF, MJCF, meshes, and terrain.
- `src/rl_sar/library/core/`: shared control, inference, FSM, loop, and safety
  infrastructure retained for bounded future extension.

The detailed remove/retain inventory is recorded in
[docs/LW_REPOSITORY_SCOPE.md](docs/LW_REPOSITORY_SCOPE.md).

## Adding a future robot

Adding a robot is an explicit product-scope change. A complete addition should:

1. add one `policy/<robot>/` tree with validated configuration and versioned
   models;
2. add a reproducible `<robot>_description` package with provenance and no
   nested Git metadata;
3. add `fsm_<robot>.hpp`, register its factory with `REGISTER_FSM_FACTORY`, and
   include it from `fsm_all.hpp`;
4. add a robot-specific real or simulation adapter without putting hardware
   behavior in the generic FSM/SDK core;
5. add only the required SDK dependencies and CMake targets;
6. extend configuration, transition, safety, repository-scope, clean-build,
   and deployment tests before declaring support.

## License and provenance

The parent repository is licensed under Apache-2.0; see [LICENSE](LICENSE).
The vendored LW description records its source repositories, imported commit,
package license declaration, and preserved local changes in
[src/rl_sar_zoo/README.md](src/rl_sar_zoo/README.md).

The project retains the upstream generic framework attribution to
[`fan-ziqi/rl_sar`](https://github.com/fan-ziqi/rl_sar), the LW zoo fork at
[`bhwxzx/rl_sar_zoo`](https://github.com/bhwxzx/rl_sar_zoo), and the joystick
submodule from [`drewnoakes/joystick`](https://github.com/drewnoakes/joystick).
