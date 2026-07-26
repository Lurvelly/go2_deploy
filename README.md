# A2 Deploy — Unitree A2 Low-Level Policy Deployment

`a2_deploy` 用于在标准版 Unitree A2 上部署项目训练的 45 维观测、12 维动作速度策略，并提供 Python/MuJoCo 离线验证和基于 HG DDS 的 Sim2Sim/Sim2Real 控制链。它不是把旧 Go2 控制器简单改名：真机端使用 A2 官方 `unitree_hg` 消息、35 槽 `LowCmd_` 和独立的安全权限控制。

`a2_deploy` deploys a project-trained 45-observation/12-action velocity policy on the standard Unitree A2. It provides standalone Python/MuJoCo validation and an HG DDS control path shared by Sim2Sim and Sim2Real. This is not a renamed Go2 controller: the hardware runtime uses the A2 `unitree_hg` messages, the 35-slot `LowCmd_` layout, and an independent safety-authority layer.

> **安全警告 / Safety warning**
>
> 低层控制可能使机器人突然运动。首次真机运行必须吊装或可靠固定机器人，确保人员远离关节，并准备可立即触发的物理急停。先验证 STOP、DAMPING 和 STAND，再进入 CTRL。MuJoCo 验证不能替代真机安全验收。
>
> Low-level control can make the robot move without warning. For the first hardware run, suspend or firmly restrain the robot, keep people clear of all joints, and have a physical emergency stop immediately available. Validate STOP, DAMPING, and STAND before entering CTRL. MuJoCo validation does not replace hardware safety acceptance.

完整的线程、DDS 回调、状态机、安全门、策略推理和日志执行链见 [当前程序执行逻辑](docs/a2_deploy_execution_flow.md)。

For the complete thread, DDS callback, state-machine, safety-gate, policy-inference, and logging flow, see [Current Program Execution Flow](docs/a2_deploy_execution_flow.md).

## 项目概览 / Overview

控制器始终拥有 low policy、12 个关节目标、安全状态机和最终 `LowCmd`。外部导航层只提交 `[vx, vy, yaw_rate]` 三维动作；终端和手柄还可以提交 `arm/stand/ctrl/damp/rearm` 操作事件。未来的 high policy 不能直接写关节目标，也不能绕过安全状态机。

The controller always owns the low policy, the 12 joint targets, the safety state machine, and the final `LowCmd`. An external navigation layer submits only a 3D `[vx, vy, yaw_rate]` action. The terminal and gamepad may additionally submit `arm/stand/ctrl/damp/rearm` operator events. A future high policy cannot write joint targets or bypass the safety state machine.

当前实现面向标准 A2（MotionSwitcher form `"0"`），不支持 A2W，不兼容 Go2 的 `unitree_go` 消息、`mcf` 服务或旧 WTW/TS 模型。仓库自带的策略来自 Legged-Nexus A2 训练流程，不是 Unitree 官方预训练模型。

The current implementation targets the standard A2 (MotionSwitcher form `"0"`). It does not support A2W, Go2 `unitree_go` messages, `mcf` services, or the former WTW/teacher-student models. The bundled policy comes from the Legged-Nexus A2 training workflow and is not a Unitree-distributed pretrained model.

## 仓库包含的内容 / What's Included

| 能力 / Capability | 中文说明 | English |
|---|---|---|
| A2 low policy | 在 CPU LibTorch 上加载并运行 `float32 [1,45] -> [1,12]` TorchScript actor | Loads and runs a `float32 [1,45] -> [1,12]` TorchScript actor with CPU LibTorch |
| 合同校验 / Contract validation | 校验 YAML、模型 SHA-256、输入输出 shape、有限值和可选 golden output | Validates YAML, model SHA-256, input/output shapes, finite values, and an optional golden output |
| 双频控制 / Dual-rate control | 50 Hz 生成策略/站立请求，500 Hz 过滤目标并发布 LowCmd | Produces policy/stand requests at 50 Hz and filters/publishes LowCmd at 500 Hz |
| HG DDS | 订阅 LowState/MainBoardState，发布 CRC 正确的 35 槽 LowCmd | Subscribes to LowState/MainBoardState and publishes CRC-correct 35-slot LowCmd messages |
| 安全监督 / Safety supervision | CRC、tick、状态新鲜度、主板、MotionSwitcher、姿态、关节和策略 deadline 门禁 | Gates on CRC, ticks, freshness, mainboard state, MotionSwitcher, orientation, joints, and policy deadlines |
| 导航输入 / Navigation input | 支持手柄、仿真终端和本机 UDP high-policy 三种速度来源 | Supports gamepad, simulation-terminal, and loopback UDP high-policy velocity sources |
| Sim2Sim | 提供单进程 Python/MuJoCo 验证和 `a2_deploy + unitree_mujoco` DDS 联调 | Provides standalone Python/MuJoCo validation and `a2_deploy + unitree_mujoco` DDS integration |
| Sim2Real | 提供标准 A2 的网卡、状态机、受控停止、日志与故障处理流程 | Provides the network, state-machine, controlled-stop, logging, and fault workflow for a standard A2 |
| 测试 / Tests | 覆盖合同、安全、策略、LowCmd、导航过滤和 UDP 接收器 | Covers the contract, safety, policy, LowCmd, navigation filter, and UDP receiver |

## 仓库不包含的内容 / What's NOT Included

| 不包含 / Not included | 中文说明 | English |
|---|---|---|
| High policy 本体 / High-policy model | 不包含 SRU/SEA-Nav 网络、深度图处理、目标点逻辑、recurrent state 或训练 checkpoint | No SRU/SEA-Nav network, depth processing, goal logic, recurrent state, or trained high-policy checkpoint |
| 训练框架 / Training framework | 不负责 low/high policy 训练、数据集、奖励或环境定义 | Does not provide low/high-policy training, datasets, rewards, or environment definitions |
| 进程内 DDS 仿真 / In-process DDS simulation | `--sim` 仍连接独立 `unitree_mujoco` 进程，不会在 C++ 控制器内启动 MuJoCo | `--sim` still connects to a separate `unitree_mujoco` process; it does not launch MuJoCo inside the C++ controller |
| Go2/A2W ABI | 不支持 Go2 或 A2W 的消息、关节映射和模型合同 | Does not support Go2 or A2W messages, joint mappings, or model contracts |
| 安全认证 / Safety certification | 软件门禁和仿真结果不构成硬件安全认证 | Software gates and simulation results are not hardware safety certification |

## 架构 / Architecture

### 仓库结构 / Repository layout

```text
a2_deploy/
├── CMakeLists.txt             # C++ build, tests, staging, and installation
├── include/a2/                # Contract, config, policy, safety, command APIs
├── src/                       # CLI, DDS runtime, policy, safety, navigation
├── params/a2.yaml             # Frozen robot/policy/safety contract
├── models/                    # Bundled TorchScript actor and provenance
├── sim/                       # Standalone MuJoCo, UDP sender, config generator
├── assets/a2/                 # A2 MJCF, meshes, checksums, and licenses
├── tests/                     # CTest unit and integration-style checks
├── docs/                      # Detailed execution-flow documentation
├── LICENSE
└── THIRD_PARTY_NOTICES.md
```

核心实现按职责拆分，而不是让模型直接操作电机：

The implementation is split by responsibility; the model never writes motor commands directly:

| 组件 / Component | 职责 / Responsibility |
|---|---|
| [`src/main.cpp`](src/main.cpp) | CLI、配置路径解析、日志目录、仿真终端、UDP 轮询、信号处理 / CLI, config resolution, log directories, simulation terminal, UDP polling, and signals |
| [`src/controller.cpp`](src/controller.cpp) | DDS、50/500 Hz 线程、状态机、目标生成、LowCmd 发布和 CSV / DDS, 50/500 Hz loops, state machine, target generation, LowCmd publishing, and CSV |
| [`src/safety.cpp`](src/safety.cpp) | 安全输入、权限降级、故障锁存和显式 rearm / Safety inputs, authority downgrade, fault latching, and explicit rearm |
| [`src/policy.cpp`](src/policy.cpp) | 模型哈希、TorchScript 加载、shape/probe 检查与推理 / Model hashing, TorchScript loading, shape/probe checks, and inference |
| [`src/config.cpp`](src/config.cpp) | 加载并验证 `params/a2.yaml` 的固定合同 / Loads and validates the frozen `params/a2.yaml` contract |
| [`src/navigation.cpp`](src/navigation.cpp) | 3D action clamp、EMA、物理速度范围和 wire format / 3D action clamping, EMA, physical envelope, and wire format |
| [`src/high_command_receiver.cpp`](src/high_command_receiver.cpp) | loopback UDP、批量 drain 和 backlog fail-closed 处理 / Loopback UDP, batched draining, and fail-closed backlog handling |
| [`include/a2/low_command.hpp`](include/a2/low_command.hpp) | 每帧新建 STOP/DAMPING/POSITION HG LowCmd 并计算 CRC / Builds a fresh STOP/DAMPING/POSITION HG LowCmd and CRC every frame |

### 控制数据流 / Control data flow

```text
unitree_mujoco / A2 hardware
    │
    ├── rt/lowstate ───────────────┐
    └── rt/lf/mainboardstate ──────┤
                                   v
                         DDS callbacks + CRC/tick
                                   │
                                   v
                 RobotSnapshot + SafetySupervisor
                                   │
terminal cmd ─┐                    │
gamepad ──────┼──> [vx, vy, yaw] ──┤
high UDP ─────┘                    v
                          50 Hz control loop
                   state machine + 45D observation
                                   │
                                   v
                     TorchScript 45D -> 12D action
                                   │
                                   v
                         requested_q at 50 Hz
                                   │
                                   v
                          500 Hz command loop
                   position/rate/effort target filter
                                   │
                                   v
                         sent_q + fresh 35-slot LowCmd
                                   │
                                   └──> rt/lowcmd
```

DDS 接口固定为：

The DDS interfaces are fixed:

| 方向 / Direction | Topic | Message | CRC |
|---|---|---|---|
| subscribe | `rt/lowstate` | `unitree_hg::msg::dds_::LowState_` | 回调校验 trailing CRC；失败时锁存故障且不更新策略快照 / The callback validates the trailing CRC; failure latches a fault and does not update the policy snapshot |
| subscribe | `rt/lf/mainboardstate` | `unitree_hg::msg::dds_::MainBoardState_` | 该消息没有 LowState/LowCmd 使用的 CRC 字段 / This message has no LowState/LowCmd-style CRC field |
| publish | `rt/lowcmd` | `unitree_hg::msg::dds_::LowCmd_` | 每个 500 Hz 帧从零初始化并重新计算 CRC / Every 500 Hz frame is zero-initialized and receives a fresh CRC |

`requested_q` 是 50 Hz policy 或 STAND 插值产生的上层请求，在一个策略周期内保持不变。`sent_q` 是 500 Hz 线程经过位置、目标变化率和保守 PD 力矩限制后的最近发送目标。500 Hz 线程不会把 `sent_q` 反写成下一次 `requested_q`。

`requested_q` is the high-level request produced by the 50 Hz policy or STAND interpolation and remains fixed for one policy period. `sent_q` is the most recent 500 Hz target after position, target-slew, and conservative PD-effort limiting. The 500 Hz loop never feeds `sent_q` back into the next `requested_q`.

### 并发模型 / Concurrency model

| 执行上下文 / Context | 周期 / Trigger | 主要职责 / Responsibility |
|---|---:|---|
| main thread | 约 / about 20 ms | 仿真终端、high UDP、SIGINT/SIGTERM / Simulation terminal, high UDP, SIGINT/SIGTERM |
| runtime thread | 生命周期 / lifetime | `A2Controller::Run()`、初始化和 teardown / `A2Controller::Run()`, initialization, and teardown |
| DDS callbacks | 消息触发 / message-driven | 校验并更新 LowState/MainBoard 快照 / Validate and update LowState/MainBoard snapshots |
| control thread | 50 Hz | 状态机、导航命令、low-policy 推理和 CSV / State machine, navigation commands, low-policy inference, and CSV |
| command thread | 500 Hz | 安全复查、目标过滤、LowCmd 构造和发布 / Safety recheck, target filtering, LowCmd construction, and publishing |

## 策略与机器人合同 / Policy and Robot Contract

默认合同位于 [`params/a2.yaml`](params/a2.yaml)，程序启动时会拒绝维度、顺序、物理参数或安全参数不一致的配置。

The default contract is [`params/a2.yaml`](params/a2.yaml). At startup, the program rejects configurations with inconsistent dimensions, ordering, physical parameters, or safety parameters.

| 项目 / Item | 固定值 / Frozen value |
|---|---|
| Contract ID | `a2_45d_project_v0` |
| Model | `models/a2_45d_policy.jit` |
| SHA-256 | `324d851114f77bb848255026bd56d8d4ebe00a72a72aac54f4271cd644c6fb65` |
| Policy input/output | `float32 [1,45] -> float32 [1,12]` |
| Policy / command period | `0.02 s (50 Hz)` / `0.002 s (500 Hz)` |
| Action mapping | `q_target = default_q + 0.25 * raw_action` |
| Joint order | `FR, FL, RR, RL × hip, thigh, calf` |
| HG mapping | `motor_indices = [0,1,...,11]`; slots 12–34 remain STOP |
| Default pose | right `[0.1, 0.9, -1.8]`, left `[-0.1, 0.9, -1.8]` |
| PD per leg | `Kp=[100,100,150]`, `Kd=[4,4,6]` |
| Command envelope | `vx [-0.5,1.0] m/s`, `vy [-0.5,0.5] m/s`, `yaw [-1,1] rad/s` |

官方部分 47D 示例使用 `FL,FR,RL,RR` 顺序和 `[3,4,5,0,1,2,9,10,11,6,7,8]` 映射，不能用于本 45D 策略。输入/输出维度相同也不代表模型 ABI 兼容。

Some official 47D examples use `FL,FR,RL,RR` ordering and the `[3,4,5,0,1,2,9,10,11,6,7,8]` mapping. That mapping must not be used with this 45D policy. Matching tensor dimensions alone do not make two model ABIs compatible.

### 45D observation

该 observation 不包含 base linear velocity。

The observation does not contain base linear velocity.

| 索引 / Indices | 维度 / Dim | 内容 / Value | Scale |
|---|---:|---|---:|
| `[0:3]` | 3 | `[vx, vy, yaw_rate]` | `[1.0, 1.0, 0.25]` |
| `[3:6]` | 3 | projected gravity | `1.0` |
| `[6:9]` | 3 | body angular velocity / gyro | `0.25` |
| `[9:21]` | 12 | `q - default_q` | `1.0` |
| `[21:33]` | 12 | `dq` | `0.05` |
| `[33:45]` | 12 | previous raw policy action | `1.0` |

12 维关节/action block 的精确顺序为：

The exact order of every 12D joint/action block is:

```text
FR hip, FR thigh, FR calf,
FL hip, FL thigh, FL calf,
RR hip, RR thigh, RR calf,
RL hip, RL thigh, RL calf
```

## 安装与构建 / Installation and Build

### 获取源码 / Get the source

```bash
git clone https://github.com/Lurvelly/go2_deploy.git
cd go2_deploy
```

仓库目录名仍为 `go2_deploy`，但当前 CMake 项目和可执行程序名称均为 `a2_deploy`。

The repository directory is still named `go2_deploy`, while the current CMake project and executable are both named `a2_deploy`.

### 依赖 / Dependencies

- CMake 3.16 或更新版本 / CMake 3.16 or newer
- 支持 C++17 的编译器 / A C++17 compiler
- [LibTorch](https://pytorch.org/)（CPU 或匹配本机 CUDA 的构建 / CPU build or a CUDA build matching the host）
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- OpenSSL Crypto
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

项目不会主动启用 CUDA。CPU LibTorch 不需要 CUDA；CUDA LibTorch 的 `TorchConfig.cmake` 会要求匹配的 CUDA Toolkit。

The project does not enable CUDA on its own. CPU LibTorch does not require CUDA; a CUDA LibTorch package will require a matching CUDA Toolkit through `TorchConfig.cmake`.

### 使用 SDK 源码构建 / Build with an SDK source checkout

CMake 默认查找仓库旁的 `../unitree_sdk2`，也可以显式指定路径：

CMake looks for `../unitree_sdk2` next to this repository by default. The source path can also be provided explicitly:

```bash
cmake -S . -B build \
  -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2 \
  -DCMAKE_PREFIX_PATH=/path/to/libtorch

cmake --build build -j
ctest --test-dir build --output-on-failure
```

也可以使用标准的 `Torch_DIR=/path/to/libtorch/share/cmake/Torch`。

The standard `Torch_DIR=/path/to/libtorch/share/cmake/Torch` variable is also supported.

### 使用已安装 SDK / Build with an installed SDK

把 SDK 源码路径设为空，并将 SDK 与 LibTorch 的安装前缀加入 `CMAKE_PREFIX_PATH`：

Set the SDK source path to an empty value and add the SDK and LibTorch installation prefixes to `CMAKE_PREFIX_PATH`:

```bash
cmake -S . -B build \
  -DUNITREE_SDK2_ROOT= \
  -DCMAKE_PREFIX_PATH="/opt/unitree_robotics;/path/to/libtorch"

cmake --build build -j
```

### 安装 / Install

```bash
cmake --install build --prefix /desired/prefix
```

安装内容包括可执行程序、静态库、A2 头文件、配置、模型、仿真脚本和 A2 资产。构建会保留发现的非系统动态库搜索路径，因此安装后不要随意移动 LibTorch、SDK 或 yaml-cpp。

The installation contains the executable, static libraries, A2 headers, configuration, model, simulation scripts, and A2 assets. The build preserves discovered non-system runtime library paths, so do not move LibTorch, the SDK, or yaml-cpp after installation.

### 首次合同检查 / First contract check

该命令不初始化 DDS，也不会连接或驱动机器人：

This command does not initialize DDS and cannot connect to or command the robot:

```bash
./build/a2_deploy --check-contract --config params/a2.yaml
```

成功时会打印实际配置路径、模型路径、SHA-256，并确认 45D→12D 与 golden output。

On success, it prints the resolved configuration path, model path, SHA-256, and confirmation of the 45D→12D contract and golden output.

## 使用方式 / Usage

### 启动模式与 CLI / Runtime modes and CLI

```text
Usage:
  ./build/a2_deploy --check-contract [--config PATH]
  ./build/a2_deploy --interface IFACE [--domain ID] [--config PATH] [--log-dir PATH]
  ./build/a2_deploy --sim [--interface IFACE] [--domain ID] [--command-source SOURCE]
      [--high-command-port PORT] [--config PATH] [--log-dir PATH]
```

| 模式 / Mode | 默认值 / Defaults | 行为 / Behavior |
|---|---|---|
| `--check-contract` | 不启动 DDS / no DDS | 校验 YAML、SHA、shape、有限输出与 probe / Validates YAML, SHA, shape, finite output, and probe |
| `--sim` | interface `lo`, domain `1`, source `terminal` | 运行 HG DDS 控制闭环，跳过真机 MotionSwitcher RPC / Runs the HG DDS loop and skips hardware MotionSwitcher RPC |
| 真机 / Hardware | domain `0`, source `gamepad` | 必须显式提供 `--interface IFACE` / Requires an explicit `--interface IFACE` |

| 选项 / Option | 说明 / Description |
|---|---|
| `--check-contract` | 只校验配置和策略；按上方文档化形式仅需可选的 `--config` / Validates only the config and policy; the documented form above needs only the optional `--config` |
| `--sim` | 使用 A2 HG DDS 仿真端并跳过硬件 MotionSwitcher RPC / Uses an A2 HG DDS simulator and skips hardware MotionSwitcher RPC |
| `--interface IFACE` | DDS 网卡；仿真默认 `lo`，真机必须显式指定 / DDS interface; simulation defaults to `lo` and hardware requires an explicit value |
| `--domain ID` | DDS domain，整数范围 `[0,232]`；真机默认 0，仿真默认 1 / DDS domain in `[0,232]`; hardware defaults to 0 and simulation to 1 |
| `--config PATH` | 配置 YAML；省略时按下述顺序自动查找 / Contract YAML; resolved through the search order below when omitted |
| `--log-dir PATH` | runtime 时间戳 CSV 的父目录；不能用于 `--check-contract` / Parent directory for timestamped runtime CSV logs; invalid with `--check-contract` |
| `--command-source SOURCE` | `gamepad`、`terminal` 或 `high`；`terminal` 仅限 `--sim` / `gamepad`, `terminal`, or `high`; `terminal` is simulation-only |
| `--high-command-port PORT` | high UDP loopback 端口，范围 `[1024,65535]`，默认 15000 / High-policy loopback UDP port in `[1024,65535]`, default 15000 |
| `-h`, `--help` | 打印帮助后退出 / Prints help and exits |

`--command-source` 可取 `gamepad`、`terminal` 或 `high`。`terminal` 只允许在 `--sim` 中使用。这个选项只选择导航速度来源，不会关闭手柄或仿真终端的安全状态事件。

`--command-source` accepts `gamepad`, `terminal`, or `high`. `terminal` is valid only with `--sim`. This option selects only the navigation velocity source; it does not disable safety-state events from the gamepad or simulation terminal.

未指定 `--config` 时，程序按顺序查找：

When `--config` is omitted, the program searches in this order:

1. `./params/a2.yaml`
2. `<executable>/share/a2_deploy/params/a2.yaml`
3. `<executable>/../share/a2_deploy/params/a2.yaml`

启动日志中的 `Config:` 和 `Policy:` 才是本次实际加载的路径。

The `Config:` and `Policy:` startup lines are the authoritative paths loaded for that run.

### 方式 A：Python 单进程 MuJoCo / Path A: Standalone Python MuJoCo

该路径直接加载仓库中的 MJCF、YAML 和 TorchScript，不需要机器人、DDS 或另一个仿真进程。MuJoCo/PD 为 200 Hz，策略为 50 Hz。

This path loads the repository MJCF, YAML, and TorchScript directly. It requires neither a robot nor DDS nor a second simulator process. MuJoCo/PD runs at 200 Hz and the policy at 50 Hz.

安装 Python 依赖：

Install the Python dependencies:

```bash
python3 -m pip install numpy pyyaml torch mujoco
```

无窗口前进测试：

Headless forward-command test:

```bash
python3 sim/a2_mujoco.py \
  --headless --duration 5 \
  --vx 0.5 --vy 0 --yaw 0 \
  --config params/a2.yaml
```

其他常用工况：

Other common scenarios:

```bash
# Stand in place / 原地站立
python3 sim/a2_mujoco.py --headless --duration 2 \
  --vx 0 --vy 0 --yaw 0 --config params/a2.yaml

# Lateral motion / 横移
python3 sim/a2_mujoco.py --headless --duration 2 \
  --vx 0 --vy 0.3 --yaw 0 --config params/a2.yaml

# Turn in place / 原地转向
python3 sim/a2_mujoco.py --headless --duration 2 \
  --vx 0 --vy 0 --yaw 0.5 --config params/a2.yaml
```

去掉 `--headless` 会打开 MuJoCo viewer。成功时输出 `[a2_mujoco] ok: ...`；合同、哈希、命令范围、姿态或关节检查失败时输出 `[a2_mujoco] fault: ...` 并返回非零状态。

Remove `--headless` to open the MuJoCo viewer. Success prints `[a2_mujoco] ok: ...`. Contract, hash, command-envelope, orientation, or joint failures print `[a2_mujoco] fault: ...` and return a nonzero status.

此路径只验证模型合同、关节映射、策略和动力学，不覆盖 DDS、CRC、MainBoardState、MotionSwitcher、网络 watchdog 或真机急停。

This path validates the model contract, joint mapping, policy, and dynamics only. It does not exercise DDS, CRC, MainBoardState, MotionSwitcher, network watchdogs, or the hardware emergency stop.

### 方式 B：HG DDS Sim2Sim / Path B: HG DDS Sim2Sim

`--sim` 仍通过 HG DDS 连接独立的 `unitree_mujoco`。仿真器必须发布 CRC 正确、tick 递增、`mode_machine=1` 的 `LowState_` 和健康 MainBoardState；`a2_deploy` 通过 `rt/lowcmd` 控制 12 个 A2 执行器。

`--sim` still connects to a separate `unitree_mujoco` process through HG DDS. The simulator must publish CRC-correct `LowState_` messages with advancing ticks and `mode_machine=1`, plus healthy MainBoardState messages. `a2_deploy` controls the 12 A2 actuators through `rt/lowcmd`.

终端 1 启动支持 A2 HG 桥的 MuJoCo。终端控制不需要物理手柄，建议关闭 joystick：

In terminal 1, start a `unitree_mujoco` build that includes the A2 HG bridge. A physical gamepad is unnecessary for terminal control, so disabling the joystick is recommended:

```bash
cd /path/to/unitree_mujoco
./simulate/build/unitree_mujoco \
  --robot a2 --domain_id 1 --network lo --joystick 0
```

终端 2 启动控制器：

In terminal 2, start the controller:

```bash
cd /path/to/go2_deploy
./build/a2_deploy --sim
```

`--sim` 只跳过真机 MotionSwitcher RPC；LowState CRC、tick watchdog、MainBoardState 门禁、LowCmd CRC、500 Hz 命令和安全状态机仍然启用。看到 `A2 DDS inputs healthy; controller is ready to arm` 后，依次输入：

`--sim` skips only the hardware MotionSwitcher RPC. LowState CRC, tick watchdog, MainBoardState gating, LowCmd CRC, the 500 Hz command loop, and the safety state machine remain enabled. After `A2 DDS inputs healthy; controller is ready to arm` appears, enter:

```text
arm
stand
# Wait about 2 seconds for stand interpolation / 等待约 2 秒完成站立插值
ctrl
cmd 0.2 0 0
zero
quit
```

| 命令 / Command | 作用 / Effect |
|---|---|
| `arm` | PREARM → DAMPING，并等待 arm 后的新鲜零 MainBoardState / PREARM → DAMPING, then waits for a fresh zero MainBoardState after arming |
| `stand` | DAMPING → STAND，2 秒插值到默认姿态 / DAMPING → STAND, interpolating to the default pose over 2 seconds |
| `ctrl` | STAND 完成后进入 CTRL / Enters CTRL after STAND completes |
| `cmd vx vy yaw` | 提交 terminal navigation action / Submits a terminal navigation action |
| `zero` | 提交 raw zero；命令经 EMA 衰减而不是瞬间清空 / Submits raw zero; the command decays through the EMA rather than clearing instantly |
| `damp` | STAND/CTRL → DAMPING / STAND/CTRL → DAMPING |
| `rearm` | 输入恢复后从 FAULT 显式恢复 / Explicitly recovers from FAULT after inputs recover |
| `quit` | 进入受控停止 / Starts a controlled shutdown |

MuJoCo 约束求解器可能瞬时越过硬限位几毫弧度，因此 `--sim` 会使用 `sim.joint_position_tolerance_rad`，默认 0.005 rad、上限 0.01 rad。真机路径始终使用 0 容差。

The MuJoCo constraint solver can transiently overshoot a hard joint limit by a few milliradians. `--sim` therefore uses `sim.joint_position_tolerance_rad`, defaulting to 0.005 rad and capped at 0.01 rad. The hardware path always uses zero tolerance.

配套仿真器可能在 DAMPING 阶段暂停物理，因为 `kp=0` 的阻尼不足以抵抗重力；第一帧 CRC 正确且 `kp>0` 的 STAND 位置命令可触发自动启动。若所用仿真器未启用该行为，请按其说明手动启动。进入 DAMPING 后机器人会在重力下下蹲或倒下，它不是“冻结仿真”。

The matching simulator may pause physics during DAMPING because damping with `kp=0` cannot support the robot against gravity. The first CRC-correct STAND position command with `kp>0` can trigger automatic startup. If the simulator build does not enable this behavior, start it manually as documented by that simulator. Once physics is running, DAMPING lets the robot crouch or fall under gravity; it does not freeze the simulation.

使用手柄时可运行 `--sim --command-source gamepad`，并用 `--joystick 1` 启动仿真器。

For gamepad operation, run `--sim --command-source gamepad` and start the simulator with `--joystick 1`.

### 方式 C：High-policy 3D 接口 / Path C: High-policy 3D Interface

high policy 和手动 `cmd` 共用同一个 low policy、安全状态机和 LowCmd 链。high policy 只负责输出三维导航 action；observation、深度图、目标点和 recurrent state 由 high-policy 进程自行维护。

The high policy and manual `cmd` input share the same low policy, safety state machine, and LowCmd path. The high policy produces only the 3D navigation action; its observation, depth images, goals, and recurrent state remain inside the high-policy process.

启动接收端：

Start the receiver:

```bash
./build/a2_deploy \
  --sim \
  --command-source high \
  --config params/a2.yaml
```

仍在 `a2_deploy` 终端使用 `arm`、`stand`、`ctrl`、`damp` 和 `rearm` 管理安全状态机。另一个终端可用示例发送器验证接口：

Continue to use `arm`, `stand`, `ctrl`, `damp`, and `rearm` in the `a2_deploy` terminal to manage the safety state machine. In another terminal, validate the interface with the example sender:

```bash
python3 sim/a2_highcmd.py \
  --vx 0.2 --vy 0 --yaw 0 --duration 5
```

默认 wire format 和目标地址为：

The default wire format and destination are:

```text
UDP 127.0.0.1:15000
A2NAV1 <vx> <vy> <yaw_rate>
```

`--high-command-port` 可以修改监听端口，接收器只绑定 loopback。输入先裁剪到 `[-3,3]`，再经过 `alpha=0.5` 的 EMA 和 A2 物理命令范围裁剪。200 ms 没有新包时会清空 EMA 并把速度归零。若每轮收到的包达到 64 个，接收器会丢弃整段 backlog，直到 socket 排空，避免旧动作刷新 watchdog。

`--high-command-port` changes the listening port. The receiver binds only to loopback. Inputs are clamped to `[-3,3]`, passed through an `alpha=0.5` EMA, and clipped to the A2 physical command envelope. If no fresh packet arrives for 200 ms, the EMA is cleared and velocity becomes zero. If a poll reaches the 64-datagram drain limit, the receiver discards the backlog until the socket is empty so stale actions cannot refresh the watchdog.

Python high-policy 循环可以复用发送器：

A Python high-policy loop can reuse the publisher:

```python
import time

from sim.a2_highcmd import A2NavigationPublisher

period = 0.02
next_send = time.monotonic()

with A2NavigationPublisher() as publisher:
    while True:
        observation = get_high_policy_observation()
        action = high_policy(observation)  # [vx, vy, yaw_rate]
        publisher.publish(action)

        next_send += period
        delay = next_send - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_send = time.monotonic()
```

该调度不会补发积压帧。正式 high policy 必须与 `a2_deploy` 运行在同一台计算机，除非代码和安全边界被明确重新设计。

This scheduling does not replay missed frames. The production high policy must run on the same computer as `a2_deploy` unless the transport and safety boundary are explicitly redesigned.

### 方式 D：A2 真机 / Path D: A2 Hardware

真机端参考 Unitree [A2 SDK 基础服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/basic_service_interface)。当前合同仅支持标准 A2 form `"0"`。

The hardware runtime follows the Unitree [A2 SDK Basic Service Interface](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/basic_service_interface). The current contract supports only standard A2 form `"0"`.

#### 1. 先区分修改级别 / Decide What Kind of Change Is Required

Sim2Real 前先判断差异属于哪一层。当前 YAML 不是一个可以任意调参的配置文件：`A2Config::Validate()` 会把策略 ABI、机器人映射、PD、物理限制和安全阈值与编译期常量逐项比对。修改冻结字段时，只改 `params/a2.yaml` 会在启动阶段被拒绝；绕过校验则可能让策略 ABI 与真实机器人失配。

Before Sim2Real, classify each difference by layer. The YAML is not a freely tunable configuration file: `A2Config::Validate()` compares the policy ABI, robot mapping, PD values, physical limits, and safety thresholds against compile-time constants. Editing only `params/a2.yaml` for a frozen field will be rejected at startup; bypassing validation can silently mismatch the policy ABI and the real robot.

| 修改级别 / Change level | 可以调整的内容 / What may change | 做法 / Required action |
|---|---|---|
| 启动参数 / Runtime selection | 网卡、DDS domain、命令源、high UDP 端口、日志目录 / Interface, DDS domain, command source, high UDP port, log directory | 只改 CLI，不改代码 / Change CLI arguments only |
| 同 ABI 模型 / Same-ABI model | TorchScript 文件、SHA-256、golden output / TorchScript file, SHA-256, golden output | 用 `sim/a2_prepare_low_policy.py` 生成独立 YAML，并先运行 `--check-contract` / Generate a separate YAML with `sim/a2_prepare_low_policy.py` and run `--check-contract` first |
| 仿真专用 / Simulation-only | MJCF 路径、physics dt、base height、仿真关节容差 / MJCF path, physics dt, base height, simulation joint tolerance | 只影响 Python MuJoCo 或 `--sim` 容差；不能用来修复真机合同 / Affects Python MuJoCo or `--sim` tolerance only; it cannot repair a hardware-contract mismatch |
| 冻结合同 / Frozen contract | 45D/12D、频率、action/observation scale、关节顺序、motor slots、default pose、PD、limits、速度范围、安全时间和阈值、`mode_machine/form` | 必须同步修改 [`include/a2/contract.hpp`](include/a2/contract.hpp)、[`params/a2.yaml`](params/a2.yaml)、[`src/config.cpp`](src/config.cpp) 及相关测试；ABI 变化通常还要重新训练/导出模型 / Update the contract header, YAML, validator, and related tests together; an ABI change normally also requires retraining/exporting the model |
| 硬件协议 / Hardware protocol | DDS topic/message、CRC 布局、MainBoard bit 语义、MotionSwitcher API、遥控器字节布局 / DDS topics/messages, CRC layout, MainBoard bit semantics, MotionSwitcher API, remote payload layout | 按当前机器人固件和 SDK 修改 `src/controller.cpp`、`src/safety.cpp`、消息构造/CRC 代码及测试 / Update controller, safety, message/CRC code, and tests against the actual robot firmware and SDK |
| 导航边界 / Navigation boundary | 摇杆轴方向、command envelope、EMA、high timeout、UDP 协议 / Stick signs, command envelope, EMA, high timeout, UDP protocol | 修改 navigation/controller/runtime options，并补充 command、navigation、receiver 测试 / Update navigation/controller/runtime options and extend command, navigation, and receiver tests |

最常见的不需要改代码的真机命令是：

The most common hardware command requiring no code change is:

```bash
./build/a2_deploy \
  --interface enp3s0 \
  --domain 0 \
  --command-source gamepad \
  --config params/a2.yaml \
  --log-dir logs
```

如果下面任何一项与真机事实不一致，不要继续 START；先按上表修改对应合同并重新完成 CTest、合同检查和 DDS Sim2Sim。

If any item below disagrees with the actual robot, do not press START. Update the corresponding contract, then repeat CTest, the contract check, and DDS Sim2Sim.

#### 2. 上机前必须确认 / Must Confirm Before Hardware

机器人与 SDK / Robot and SDK:

- [ ] 机器人是标准 A2，不是 A2W；MotionSwitcher `CheckMode()` 返回 form `"0"`。
  The robot is a standard A2, not A2W, and MotionSwitcher `CheckMode()` returns form `"0"`.
- [ ] 真机固件、`unitree_sdk2` 版本和本仓编译时使用的 `unitree_hg` IDL 一致；`LowState_`/`LowCmd_` 均为 35 个 motor slots。
  Robot firmware, the `unitree_sdk2` version, and the compiled `unitree_hg` IDL agree; `LowState_`/`LowCmd_` both expose 35 motor slots.
- [ ] `ai_sport` 或其他高层运动服务可以被释放，而且不会被守护进程自动重新拉起并抢占 mode。
  `ai_sport` and other high-level motion services can be released and are not automatically restarted to reclaim the mode.
- [ ] 当前固件中 `MainBoardState.state[0] == 0` 表示健康；首次 FOC 前 `1` 只表示 pending，其他 bit 或 FOC 后任意非零都应视为故障。
  On the installed firmware, `MainBoardState.state[0] == 0` means healthy; `1` is only a pre-FOC pending state, while other bits—or any nonzero value after FOC—mean fault.

DDS 与网络 / DDS and network:

- [ ] 使用有线网卡，主机 IP/子网正确；已确认实际 interface 名称和 DDS domain（当前默认真机 domain 0）。
  A wired interface is used with the correct host IP/subnet; the actual interface and DDS domain are confirmed (hardware currently defaults to domain 0).
- [ ] 真机发布 `rt/lowstate` 和 `rt/lf/mainboardstate`，并接受 `rt/lowcmd`；topic 名、namespace 和消息类型没有被固件版本修改。
  The robot publishes `rt/lowstate` and `rt/lf/mainboardstate` and accepts `rt/lowcmd`; firmware has not changed the topic names, namespace, or message types.
- [ ] 不按 START 时可以稳定看到 `A2 DDS inputs healthy; controller is ready to arm`，没有 CRC、tick、100 ms freshness 或 `mode_machine` 故障。
  Without pressing START, `A2 DDS inputs healthy; controller is ready to arm` remains stable with no CRC, tick, 100 ms freshness, or `mode_machine` fault.
- [ ] 日志目录可写、磁盘空间充足，并能保存每次吊装验收的 CSV。
  The log directory is writable, disk space is sufficient, and every suspended acceptance run can retain its CSV.

机器人与策略合同 / Robot and policy contract:

- [ ] 12 关节反馈的实际顺序确实是 `FR, FL, RR, RL × hip, thigh, calf`，HG slot 映射确实是 `0..11` identity。
  The actual 12-joint feedback order is `FR, FL, RR, RL × hip, thigh, calf` and the HG slot mapping is identity `0..11`.
- [ ] 每个关节的正方向、零位、默认站姿、硬限位、速度/力矩上限和当前机器人机械版本一致。
  Every joint sign, zero, default pose, hard limit, velocity limit, and effort limit matches the actual mechanical revision.
- [ ] `Kp/Kd`、2 秒站立插值和 500 Hz target filter 适合当前机器人、负载、电池状态与地面条件。
  `Kp/Kd`, the 2-second stand interpolation, and the 500 Hz target filter are suitable for the robot, payload, battery condition, and floor.
- [ ] 模型文件 SHA-256、45D observation 顺序、scales、previous action 和 `q_target = q0 + 0.25 * action` 与训练导出完全一致。
  The model SHA-256, 45D observation order, scales, previous action, and `q_target = q0 + 0.25 * action` exactly match training/export.
- [ ] IMU 四元数在当前消息中是 `[w,x,y,z]`，gyro 是 body frame；站立时 projected gravity 和倾角合理。
  The current message supplies quaternion `[w,x,y,z]` and body-frame gyro; projected gravity and tilt are plausible while standing.
- [ ] 实际遥控器的 40-byte payload、`START/L1/R2/A/Y` 按键和 `LY/-LX/-RX` 轴方向与代码一致。
  The real remote's 40-byte payload, `START/L1/R2/A/Y` buttons, and `LY/-LX/-RX` axis signs match the code.

主机与独立安全链 / Host and independent safety path:

- [ ] 目标计算机在 CPU 模式下持续满足 20 ms policy 周期，没有热降频；单帧不达到 40 ms，也不会连续 3 帧超过 20 ms。
  The target computer sustains the 20 ms policy period on CPU without thermal throttling; no frame reaches 40 ms and no three consecutive frames exceed 20 ms.
- [ ] 机器人已可靠吊装，关节完整活动范围内没有吊带、线缆或支架干涉。
  The robot is reliably suspended, with no sling, cable, or fixture interference throughout the joint range.
- [ ] 物理急停由独立人员控制并实际验证；软件 DAMPING、`Ctrl+C` 和网线 watchdog 不能替代物理急停。
  A second operator controls and has physically verified the emergency stop; software DAMPING, `Ctrl+C`, and the Ethernet watchdog do not replace it.
- [ ] 若使用 high policy，它与 `a2_deploy` 在同一台主机，端口一致，且已单独验证 200 ms 断流自动归零。
  If a high policy is used, it runs on the same host with the matching port and its 200 ms stale-command zeroing has been independently verified.

#### 3. 上电前检查 / Preflight

- 先运行合同检查并确保所有 CTest 通过。
  Run the contract check and ensure all CTest tests pass first.
- 用网线直连 A2，确认上位机与机器人网络配置。
  Connect the host directly to the A2 over Ethernet and verify both network configurations.
- 吊装或可靠固定机器人，确认遥控器和物理急停可用。
  Suspend or firmly restrain the robot and verify the remote and physical emergency stop.
- 首次验收先验证 STOP、DAMPING 和 STAND，不要直接进入 CTRL。
  Validate STOP, DAMPING, and STAND before CTRL during first acceptance.

查询网卡名：

Find the wired interface name:

```bash
ip -br link
ip -br addr
```

建议把 SDK commit、机器人固件版本、机器人序列号、网卡/domain、配置路径和模型 SHA-256 写入本次验收记录。

Record the SDK commit, robot firmware version, robot serial number, interface/domain, configuration path, and model SHA-256 for this acceptance run.

#### 4. 启动控制器 / Start the controller

把 `enp3s0` 替换成实际有线网卡：

Replace `enp3s0` with the actual wired interface:

```bash
./build/a2_deploy \
  --interface enp3s0 \
  --config params/a2.yaml \
  --log-dir logs
```

真机默认使用手柄提供速度和状态事件。未来 high policy 上真机时，只用 `--command-source high` 替换速度来源；START/L1 组合键和物理急停仍属于独立安全链：

Hardware defaults to the gamepad for both velocity and state events. To use a future high policy on hardware, `--command-source high` replaces only the velocity source; START/L1 button combinations and the physical emergency stop remain an independent safety path:

```bash
./build/a2_deploy \
  --interface enp3s0 \
  --command-source high \
  --high-command-port 15000 \
  --config params/a2.yaml \
  --log-dir logs
```

启动时程序会检查并释放 `ai_sport`，直到 MotionSwitcher `CheckMode()` 返回空模式，最长等待 30 秒。首次 START 前会同步复查，运行期间约每秒复查一次。

At startup, the program checks and releases `ai_sport` until MotionSwitcher `CheckMode()` reports an empty mode, with a 30-second limit. It rechecks synchronously before the first START and approximately once per second during runtime.

#### 5. 遥控状态机 / Remote state machine

| 操作 / Input | 前置状态 / From | 结果 / Result |
|---|---|---|
| `START` | PREARM | 武装并进入 DAMPING；等待新的零 MainBoardState / Arms and enters DAMPING; waits for a new zero MainBoardState |
| `L1 + R2` | DAMPING | 进入 STAND，2 秒插值到默认姿态 / Enters STAND and interpolates to the default pose over 2 seconds |
| `L1 + A` | STAND 完成 / complete | 进入 CTRL，启用 low policy / Enters CTRL and enables the low policy |
| `L1 + Y` | DAMPING/STAND/CTRL | 返回或保持 DAMPING / Returns to or remains in DAMPING |
| `L1 + START` | FAULT，输入已恢复 / inputs recovered | 解除锁存，返回 DAMPING 并重新执行主板门禁 / Clears the latch, returns to DAMPING, and repeats mainboard gating |

#### 6. 分阶段验收与停止条件 / Phased Acceptance and Stop Conditions

每个阶段都必须在可靠吊装、物理急停可用且日志开启的条件下单独通过。任一关节方向错误、意外 FOC、剧烈抖动、CRC/tick/MainBoard/MotionSwitcher 故障或日志缺失，都应立即急停并回到“需要修改/确认”表，不得通过放宽阈值继续。

Run every stage separately with reliable suspension, a ready physical emergency stop, and logging enabled. Any wrong joint direction, unexpected FOC, severe oscillation, CRC/tick/MainBoard/MotionSwitcher fault, or missing log is a stop condition. Return to the change/confirmation tables; do not continue by weakening thresholds.

| 阶段 / Stage | 操作 / Action | 必须确认的通过条件 / Required evidence |
|---|---|---|
| 0. 离线 / Offline | build、6 项 CTest、`--check-contract`、Python MuJoCo、DDS Sim2Sim | 全部通过；记录 config/model 路径和 SHA；无合同或 limiter 异常 / All pass; record config/model paths and SHA; no contract or unexpected limiter issue |
| 1. 只接收状态 / State-only | 启动真机进程，不按 START，保持 PREARM 至少 30 秒 / Start hardware runtime and remain in PREARM for at least 30 seconds without START | ready 提示稳定；phase=PREARM；MainBoard=0；全 35 槽 STOP；无 CRC/tick/freshness fault / Stable ready message, PREARM, MainBoard=0, all 35 slots STOP, no CRC/tick/freshness fault |
| 2. DAMPING | 按 START，不按其他组合键 / Press START only | 只进入 DAMPING；arm 后出现新鲜 MainBoard=0；`kp=0`；没有跳变、主动站立或关节方向异常 / DAMPING only; fresh post-arm MainBoard=0; `kp=0`; no jump, active stand, or wrong joint direction |
| 3. STAND | 按 `L1+R2`，观察完整 2 秒插值 / Press `L1+R2` and observe the full 2-second interpolation | 12 关节朝预期方向平滑移动；最终姿态、`q`、`requested_target`、`sent_target` 和 limiter 行为合理 / All joints move smoothly in the expected direction; final pose, `q`, targets, and limiter behavior are plausible |
| 4. 零命令 CTRL / Zero-command CTRL | STAND 完成后按 `L1+A`，摇杆保持中立 / After STAND, press `L1+A` with neutral sticks | command 接近零，策略 50 Hz 稳定，`policy_time_ms` 满足 deadline，机器人无持续漂移或高频抖动 / Near-zero command, stable 50 Hz policy, deadline-compliant `policy_time_ms`, no sustained drift or high-frequency oscillation |
| 5. 小命令 / Small commands | 分别施加极小 `+vx/-vx/+vy/-vy/+yaw/-yaw`，每次回零 / Apply each small signed axis separately and return to zero | 实际运动方向与命令符号一致；命令裁剪、EMA、姿态和 limiter 可解释 / Motion direction matches command sign; clipping, EMA, attitude, and limiter activity are explainable |
| 6. 降级与退出 / Degrade and stop | 依次验证 `L1+Y`、`Ctrl+C`；使用 high 时验证 200 ms 断流；最后在吊装下验证断网和物理急停 / Validate `L1+Y` and `Ctrl+C`; validate 200 ms high timeout if used; finally test network loss and physical E-stop while suspended | DAMPING/STOP 顺序符合设计，不自动恢复 CTRL，必须显式 rearm；进程退出后保持 STOP / DAMPING/STOP sequencing matches design, CTRL never auto-resumes, explicit rearm is required, and STOP remains after exit |

每一阶段至少保留以下记录：

Retain at least the following evidence for every stage:

| 记录 / Record | 内容 / Content |
|---|---|
| 软件身份 / Software identity | Git commit/diff、SDK commit、编译器与 LibTorch 版本 / Git commit/diff, SDK commit, compiler, and LibTorch version |
| 硬件身份 / Hardware identity | A2 序列号、固件版本、form、负载、电池状态 / A2 serial, firmware, form, payload, and battery condition |
| 运行身份 / Runtime identity | 完整启动命令、interface/domain、config/model 绝对路径、模型 SHA / Full command, interface/domain, resolved config/model paths, and model SHA |
| 运行证据 / Runtime evidence | 完整控制台输出、`a2_runtime.csv`、必要的视频和急停/断网结果 / Complete console output, `a2_runtime.csv`, necessary video, and E-stop/network-loss results |
| 结论 / Decision | 通过、失败原因、下一项修改、复测日期与负责人 / Pass/failure reason, next change, retest date, and owner |

## 安全模型 / Safety Model

控制器同时维护“想执行的 phase”和“这一帧最多允许的 command authority”。界面显示 DAMPING 不代表已经发送 FOC；主板、LowState 或 MotionSwitcher 门禁未满足时，500 Hz 输出仍是 STOP。

The controller maintains both the requested runtime phase and the maximum command authority allowed for the current frame. Seeing DAMPING in the UI does not prove that FOC has been sent. If the mainboard, LowState, or MotionSwitcher gates are incomplete, the 500 Hz output remains STOP.

### 状态机 / State machine

```text
PREARM --arm/START--> DAMPING --stand/L1+R2--> STAND
   │                     ^                         │
   │                     └──── damp/L1+Y ─────────┤
   │                                               │ ctrl/L1+A after interpolation
   │                                               v
   │                                             CTRL
   │                                               │
   │                     <──── damp/L1+Y ──────────┘
   │
   └──────── safety fault ───────> FAULT
                                      │
                         rearm/L1+START after recovery
                                      v
                                   DAMPING

Any running phase --signal/quit/worker failure--> STOPPING --> STOPPED
```

| Phase | Low policy | `requested_q` | 健康时的输出 / Healthy output |
|---|---|---|---|
| PREARM | off | initial | 有可信 `mode_machine=1` 状态时全槽 STOP，否则不发布 / All-slot STOP with a trusted `mode_machine=1` state; otherwise no publish |
| DAMPING | off, reset previous action | current `q` | 12 路 FOC damping，`kp=0` / 12-channel FOC damping with `kp=0` |
| STAND | off | 2 s interpolation | 经过 hard-limit/rate/effort 过滤的位置 FOC / Position FOC through hard-limit/rate/effort filters |
| CTRL | 50 Hz | `default_q + 0.25 * action` | 经过 90% soft-limit/rate/effort 过滤的位置 FOC / Position FOC through 90% soft-limit/rate/effort filters |
| FAULT | off, reset previous action | retained but inactive | 通常 STOP；仅可信 external fault 可短时 damping；状态不可信时可能不发布 / Usually STOP; only trusted external faults may use bounded damping; an untrusted protocol state may prevent publishing |
| STOPPING | off | inactive | 最多 0.5 s damping，随后 STOP / Up to 0.5 s damping, then STOP |

### FOC 持续门禁 / Continuous FOC gates

- LowState CRC 正确；5 秒内收到首个可信状态；后续有效状态和 tick 推进不超过 100 ms。
  LowState CRC is valid; the first trusted state arrives within 5 seconds; subsequent valid state and tick progress remain within 100 ms.
- `mode_machine == 1`，IMU、关节和策略数据全部为有限值。
  `mode_machine == 1` and all IMU, joint, and policy values are finite.
- 四元数范数在 `[0.9,1.1]`，总倾角不超过 1.0 rad。
  Quaternion norm remains in `[0.9,1.1]` and total tilt does not exceed 1.0 rad.
- 关节位置和速度位于配置限制内。
  Joint positions and velocities remain within the configured limits.
- arm 后收到一帧新的 `MainBoardState.state[0] == 0`；运行状态不超过 2 秒未更新。
  A new `MainBoardState.state[0] == 0` frame arrives after arming; runtime mainboard state is no more than 2 seconds old.
- 真机 MotionSwitcher form 为 `"0"`、mode 为空且控制权已释放。
  Hardware MotionSwitcher form is `"0"`, mode is empty, and control authority has been released.
- 单次策略推理小于 40 ms，且不能连续 3 帧超过 20 ms。
  A policy inference takes less than 40 ms and cannot exceed 20 ms for three consecutive frames.

首次 FOC 前，精确的 `state[0] == 1` 可以作为主板 pending 状态等待清零，其他非零值会触发故障。控制器曾获得 FOC 权限后，任何非零 `state[0]` 都会立即锁存故障。

Before the first FOC authorization, exactly `state[0] == 1` may be treated as a pending mainboard state while waiting for zero; any other nonzero value faults. After the controller has obtained FOC authority once, any nonzero `state[0]` immediately latches a fault.

故障会锁存，健康数据重新出现不会自动恢复 CTRL。只有输入恢复、故障后的 damping/STOP 时间窗结束并收到显式 `rearm` 后，系统才回到 DAMPING；随后还会重新执行 MainBoard 门禁。

Faults are latched; healthy data returning does not automatically restore CTRL. The system returns to DAMPING only after inputs recover, the post-fault damping/STOP window completes, and an explicit `rearm` is received. It then repeats the MainBoard gate.

### 关节限制 / Joint limits

| 关节 / Joint | 硬位置范围 / Hard position (rad) | 速度上限 / Velocity (rad/s) | 力矩上限 / Effort (Nm) |
|---|---:|---:|---:|
| hip | `[-1.01, 1.01]` | 22 | 120 |
| front thigh | `[-2.34, 3.15]` | 22 | 120 |
| rear thigh | `[-1.56, 3.94]` | 22 | 120 |
| calf | `[-2.77, -0.54]` | 14.6667 | 180 |

CTRL 目标使用硬范围中间 90% 的软限位。每个 500 Hz tick 依次执行位置、目标变化率和保守 PD 力矩限制：

CTRL targets use the central 90% of each hard range as a soft limit. Every 500 Hz tick applies position, target-slew, and conservative PD-effort limits in that order:

```text
tau_est = kp * (q_target - q) - kd * dq
```

## 自定义 Low Policy / Custom Low Policy

自定义模型必须保持 45D observation、12D action、关节顺序、缩放和 `q_target = q0 + 0.25 * action` 合同。不要覆盖默认模型和 YAML；为每个模型生成独立配置。

A custom model must preserve the 45D observation, 12D action, joint order, scales, and `q_target = q0 + 0.25 * action` contract. Do not overwrite the bundled model or YAML; generate a separate configuration for each model.

```bash
python3 sim/a2_prepare_low_policy.py \
  --policy /absolute/path/to/my_low_policy.jit \
  --output params/my_low_policy.yaml

./build/a2_deploy \
  --check-contract \
  --config params/my_low_policy.yaml
```

生成器在 CPU 上验证 `float32 [1,45] -> float32 [1,12]`，并把模型绝对路径、SHA-256 和 probe 输出写入新 YAML。验证后，Sim2Sim 和真机都通过 `--config` 使用该文件：

The generator validates `float32 [1,45] -> float32 [1,12]` on CPU and writes the absolute model path, SHA-256, and probe output to the new YAML. After validation, both Sim2Sim and hardware use the file through `--config`:

```bash
./build/a2_deploy --sim --config params/my_low_policy.yaml

./build/a2_deploy \
  --interface enp3s0 \
  --config params/my_low_policy.yaml
```

训练 checkpoint 通常还包含 critic 和 optimizer，不能直接交给 LibTorch；必须先导出可执行 TorchScript actor。仓库默认 iteration-20000 actor 的来源、文件身份和 ABI 见 [模型说明](models/README.md)。

A training checkpoint usually also contains critic and optimizer state and cannot be passed directly to LibTorch. Export an executable TorchScript actor first. The bundled iteration-20000 actor's provenance, file identities, and ABI are documented in [Model Notes](models/README.md).

如果模型是 47D observation、不同腿序、不同 default pose 或不同 action scale，必须建立新的明确合同并相应修改代码和测试，不能只改维度或路径强行加载。

If a model uses a 47D observation, another leg order, another default pose, or another action scale, define a new explicit contract and update the code and tests accordingly. Do not force-load it by changing only dimensions or paths.

## 停止、日志与故障定位 / Shutdown, Logging, and Troubleshooting

### 受控停止 / Controlled shutdown

`Ctrl+C`、SIGTERM 或 `quit` 会进入受控停止：在状态仍可信且此前确实发布过 FOC 时，最多发送 0.5 秒 damping；随后持续发送 STOP，再停止线程并关闭 DDS。若状态或 authority 已不可信，则立即降级到 STOP。

`Ctrl+C`, SIGTERM, or `quit` starts a controlled shutdown. If state remains trusted and FOC was previously published, the controller may send damping for at most 0.5 seconds. It then sends STOP before stopping threads and closing DDS. If state or authority is no longer trusted, it downgrades directly to STOP.

在 MuJoCo 中，普通结束建议先 `zero` 再 `quit`。若要停留观察 DAMPING，先暂停仿真或使用可靠的虚拟吊带。

In MuJoCo, a normal exit should use `zero` followed by `quit`. To observe DAMPING without a fall, pause the simulator first or use a reliable virtual suspension.

### CSV 日志 / CSV logging

默认日志位置：

Default log location:

```text
logs/<YYYYmmdd-HHMMSS>/a2_runtime.csv
```

| 字段组 / Fields | 含义 / Meaning |
|---|---|
| `time_ms, phase, fault, reason` | 状态机和锁存故障 / State machine and latched fault |
| state ages / ticks | LowState、MainBoard 与 tick 新鲜度 / LowState, MainBoard, and tick freshness |
| `command0..2` | 策略实际使用的速度命令 / Velocity command consumed by the policy |
| `observation0..44` | 45D 策略输入 / 45D policy input |
| `action0..11` | 12D raw policy action / 12D raw policy action |
| `requested_target0..11` | 50 Hz policy/STAND 请求 / 50 Hz policy/STAND request |
| `sent_target0..11` | 50 Hz 日志采样到的最近 500 Hz 发送目标 / Latest 500 Hz sent target sampled by the 50 Hz logger |
| `limiter_count` | 最近采样的 FilterTarget 中触发任一限制的关节数 / Joints limited by the sampled FilterTarget result |
| `policy_time_ms` | 最近一次 policy 推理耗时，用于检查 20/40 ms deadline / Latest policy inference duration for checking the 20/40 ms deadlines |

CSV 以 50 Hz 异步采样 500 Hz telemetry，不保存每个 2 ms 命令帧。`limiter_count` 也不保证对应的计算结果最终通过二次安全复查并成功发送。详细判读方法见 [执行逻辑文档](docs/a2_deploy_execution_flow.md)。

The CSV samples 500 Hz telemetry asynchronously at 50 Hz and does not contain every 2 ms command frame. `limiter_count` also does not prove that the calculated target passed the second safety check and was successfully published. See the [execution-flow document](docs/a2_deploy_execution_flow.md) for detailed interpretation.

### 常见故障 / Common faults

| 原因 / Reason | 排查 / Action |
|---|---|
| `initial_low_state_timeout` | 5 秒内没有可信 LowState；检查网卡、网线、DDS domain 和 topic / No trusted LowState within 5 seconds; check interface, cable, DDS domain, and topic |
| `mainboard_prearm_timeout` | START 后 2 秒内没有新的零 MainBoardState / No new zero MainBoardState within 2 seconds after START |
| `motion_mode_not_released` | 高层运动服务仍占用控制权，或 form 不是 `"0"` / A high-level motion service still owns control, or form is not `"0"` |
| `invalid_low_state_crc` | LowState 数据损坏或双方消息 ABI 不一致 / Corrupted LowState or mismatched message ABI |
| `tick_not_advancing` | 状态流冻结、重复或倒退 / State stream is frozen, repeated, or moving backward |
| `excessive_tilt` | 机器人倾角越过 1.0 rad / Robot tilt exceeded 1.0 rad |
| `joint_position_limit` | 反馈位置越过硬限位；检查映射、姿态和仿真容差 / Feedback exceeded hard limits; check mapping, pose, and simulation tolerance |
| `joint_velocity_limit` | 反馈速度越过逐关节上限 / Feedback velocity exceeded a per-joint limit |
| `policy inference deadline failure` | 单帧达到 40 ms 或连续 3 帧超过 20 ms / One frame reached 40 ms or three consecutive frames exceeded 20 ms |

## 测试与验证边界 / Tests and Validation Boundaries

### C++ 测试 / C++ tests

```bash
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/a2_deploy --check-contract --config params/a2.yaml
```

当前测试目标：

Current test targets:

| Test | 覆盖 / Coverage |
|---|---|
| `test_contract` | observation/action、映射、配置和 500 Hz filter invariants |
| `test_safety` | watchdog、MainBoard、MotionSwitcher、fault/rearm authority |
| `test_policy` | 模型哈希、shape、finite/golden output 和推理 |
| `test_command` | fresh STOP/DAMPING/POSITION LowCmd 与 CRC |
| `test_navigation` | action clamp、EMA、command envelope、wire format |
| `test_high_command_receiver` | loopback UDP、latest packet 和 backlog fail-closed |

### 验证层级 / Validation levels

| 层级 / Level | 覆盖 / Covers | 不覆盖 / Does not cover |
|---|---|---|
| `--check-contract` | YAML、模型身份、45D→12D、probe | DDS、动力学、硬件 |
| Python MuJoCo | 合同、关节映射、策略、PD/动力学 | DDS、CRC、MainBoard、MotionSwitcher、网络 |
| DDS Sim2Sim | HG DDS、CRC、状态机、双频循环、MainBoard 门 | 真机 MotionSwitcher、真实网络/执行器、物理急停 |
| 吊装真机 / Suspended hardware | 实际 DDS、MotionSwitcher、主板、执行器和 watchdog | 无约束运动和最终场地安全 |

每一层通过只说明该层覆盖的合同成立，不能代替下一层验收。

Passing one level proves only the contracts covered at that level and does not replace acceptance at the next level.

## 相关项目 / Related Projects

| Repository | Relationship |
|---|---|
| [Legged-Nexus-Dev](https://github.com/InnoBot-Research/Legged-Nexus-Dev) | A2 low-policy training and bundled actor provenance / A2 low policy 训练及默认 actor 来源 |
| [SRU Navigation Learning](https://github.com/leggedrobotics/sru-navigation-learning) | SRU network and RL training framework; not bundled here / SRU 网络与 RL 训练框架，本仓不包含 |
| [SRU Navigation Sim](https://github.com/leggedrobotics/sru-navigation-sim) | Isaac Lab navigation environments; not a runtime dependency / Isaac Lab 导航环境，不是运行时依赖 |
| [SEA-Nav](https://github.com/11chens/SEA-Nav-Code) | Related high-level navigation research / 相关高层导航研究 |
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | DDS and MotionSwitcher runtime dependency / DDS 与 MotionSwitcher 运行时依赖 |
| [unitree_mujoco](https://github.com/lupinjia/unitree_mujoco) | External HG DDS simulator used by Sim2Sim / Sim2Sim 使用的外部 HG DDS 仿真器 |

## 来源、致谢与许可证 / Sources, Credits, and License

本项目基于原 `go2_deploy` 工作继续演进，部署代码采用 [BSD 3-Clause License](LICENSE)。版权所有者和完整条款以 LICENSE 为准。

This project continues the original `go2_deploy` work. The deployment code is distributed under the [BSD 3-Clause License](LICENSE). Refer to LICENSE for the copyright holder and complete terms.

- 默认 45D 模型来自 Legged-Nexus A2 训练流程；其 SHA-256、checkpoint/exported actor 关系和 ABI 见 [`models/README.md`](models/README.md)。
  The bundled 45D model comes from the Legged-Nexus A2 training workflow; see [`models/README.md`](models/README.md) for hashes, checkpoint/exported-actor relationships, and ABI details.
- A2 MJCF 和 mesh 来自 Unitree 资源；来源、commit、校验和与许可证见 [`assets/a2/SOURCE.md`](assets/a2/SOURCE.md)、[`assets/a2/SHA256SUMS`](assets/a2/SHA256SUMS) 和 [`assets/a2/LICENSE`](assets/a2/LICENSE)。
  The A2 MJCF and meshes originate from Unitree resources; provenance, commit, checksums, and license are recorded in [`assets/a2/SOURCE.md`](assets/a2/SOURCE.md), [`assets/a2/SHA256SUMS`](assets/a2/SHA256SUMS), and [`assets/a2/LICENSE`](assets/a2/LICENSE).
- Unitree SDK2、LibTorch、yaml-cpp、OpenSSL 和 MuJoCo 分别遵循其上游许可证；汇总说明见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。
  Unitree SDK2, LibTorch, yaml-cpp, OpenSSL, and MuJoCo remain under their upstream licenses; see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) for the consolidated notices.
