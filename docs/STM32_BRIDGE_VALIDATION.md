# STM32 bridge 软件交付与验证报告

软件验证日期：2026-10-16；发布授权日期：2026-10-17。**状态：代码实现已落地，验收部分通过；按用户明确要求提交并发布到 `rely-only`，不宣称全量验证通过或整机安全已验证。**

## 1. 基线与范围

- 仓库：`/home/zxy/rc2026_deom`（WSL Ubuntu-22.04）。
- 开始时 `rely-only` 工作区干净，与远程同 SHA。
- 修改前基线：`9b516334459092fdc6be5a489f1cb9ab30ac9cc6`。
- 开发分支：`codex/stm32-bridge`，从 `rely-only` 创建；2026-10-17 用户明确授权将现有实现提交并推送至 `rely-only`。
- 发布以快进方式更新 `rely-only`；`main` 保持 `9474d31de176f95a2e8aa5a088e1a9a6821a86d2`，不合并、不强推。
- 没有修改 STM32 同学的仓库，没有强制推送、hard reset、自动合并或开启实体运动输出。
- 本报告对应提交可用 `git log -1 --format=%H -- docs/STM32_BRIDGE_VALIDATION.md` 查询。此次提交/推送仅发布现有实现，以下失败记录、验收状态和默认禁用输出均保留。

本次实现单 UART 网关、协议 v1 草案、Nav2 与实测状态适配、Hurricane 共享运输层、实机仲裁配置和默认禁用的启动入口。没有实现 STM32 C 端、关节轨迹、电机使能或整机运动控制认证。

## 2. 安全与兼容结果

- 默认 `port=''`、`test_mode=true`、`enable_motion=false`、`calibration_confirmed=false`；不自动启动 Mission，默认也不启动 Hurricane Action。
- 物理速度只由 MotionArbiter 可选的仲裁后 `MotionCommand` 输出进入 UART，绑定 owner/mission/generation/stamp；原 Twist 保留诊断用途。
- MotionArbiter 仍唯一提供 control_state/authority/get_state/set_control_mode，未复制这些职责。核心授权、租约、停止与状态判断未放宽。
- STATUS 需要新序号和真实接收时间，ACK/发送记录/模式请求均不生成安全成功。断线、过期、旧任务、旧代次与重复技能请求不能自动恢复动作。
- MCU 未声明的技能返回 UNSUPPORTED；19/20 横移始终拒绝。WALK 零速度、HOLD 支撑与 STOP 卸力分开；跳跃中 HEARTBEAT 不覆盖轨迹。
- 两份 Nav2 的 Y 限幅/加减速全为 0，vy_samples=1；X/yaw 原数值经 YAML 对比保持一致。
- 非横移 PrecisionPoseCritic 解决侧方误差只能停车的兼容问题，保留原 holonomic 默认行为。
- 最后补上 NavVelocityAdapter 重复授权心跳不延长短租约的检查；这一小改后已重新执行最终 167 项针对性测试。

以下文件逐字节与基线相同：MissionFSM、MissionManager、ControlHandoff、ArbiterCore、LocalizationAdapter、preflight、SimulationBackend、原模拟 `arbiter.yaml`。传感器/FAST-LIO/GICP算法与行为树 XML 未改。

## 3. 修改文件清单与职责

| 类型 | 文件 | 修改目的/职责 |
| --- | --- | --- |
| 修改 | `README.md` | 更新自制 STM32 机器人硬件假设与入口索引，删除支持横移/旧串口的误导。 |
| 新增 | `common/go2_mission_backends/config/arbiter_real.yaml` | 新增实机私有输出与默认禁用配置；原 arbiter.yaml 保留不动。 |
| 修改 | `common/go2_mission_backends/go2_mission_backends/motion_arbiter.py` | 只增加可选的带租约最终输出；默认空，原授权/限幅/状态检查逻辑未改。 |
| 修改 | `common/go2_nav_bringup/config/autonomy/fast_lio_nav2.yaml` | 所有 DWB 与 smoother 禁横移，启用非横移精确接近；保留 X/yaw 数值。 |
| 修改 | `common/go2_nav_bringup/src/precision_pose_critic.cpp` | 增加可选非横移目标，修正侧方目标只能停车的问题；旧默认兼容。 |
| 修改 | `common/go2_nav_bringup/test/test_precision_pose.cpp` | 增加侧方/后方/最终朝向测试及采样速度的闭环收敛检查。 |
| 修改 | `common/robocon_mission/HURRICANE_PORT.md` | 真实流程改为共享网关，旧编码仅兼容/测试，更新反馈与启动说明。 |
| 修改 | `common/robocon_mission/launch/hurricane.launch.py` | 移除旧 serial_port launch 参数，保留默认 dry-run 预览。 |
| 新增 | `common/robocon_mission/robocon_mission/hurricane_gateway_transport.py` | 复用 SkillRunner 的 ROS 运输层，方向经仲裁，离散技能严格关联真实反馈。 |
| 修改 | `common/robocon_mission/robocon_mission/hurricane_server.py` | 真实模式使用网关，拒绝旧直连串口，增加真实技能可用性查询。 |
| 新增 | `common/robocon_mission/test/test_hurricane_gateway_transport.py` | 覆盖租约、能力、ACK/完成区分、过期/断线、错误关联与真实仲裁输入。 |
| 修改 | `common/robocon_mission/test/test_hurricane_server_ros.py` | 保留原测试并补网关模式 Action 成功/失败/取消/超时等场景。 |
| 修改 | `common/robocon_mission/test/test_simulation_launch.py` | 按最新无横移硬件要求更新断言，保留原精度/停稳/FSM检查。 |
| 修改 | `docs/ARCHITECTURE.md` | 链接新的 STM32 控制路径，保留原定位结构说明。 |
| 修改 | `docs/CONTROL_INTEGRATION.md` | 提示统一网关与带授权最终输出，指向具体协议。 |
| 修改 | `docs/REAL_ROBOT_GUIDE.md` | 纠正横移假设和旧反馈/串口说明，保留雷达/地图/定位指南。 |
| 修改 | `docs/REAL_ROBOT_QUICK_REFERENCE.md` | 加入新的 STM32 接入指南入口。 |
| 新增 | `docs/STM32_BRIDGE_GUIDE.md` | 完整构建、HELLO、部署、协议、操作顺序、C端协作与风险说明。 |
| 新增 | `docs/STM32_BRIDGE_VALIDATION.md` | 本次交付范围、逐文件职责、真实测试结果、失败证据和未验收项目。 |
| 新增 | `docs/STM32_FIRMWARE_AUDIT.md` | 固定参考固件 SHA，只读记录两板API、STOP/HOLD和真实状态缺口。 |
| 修改 | `real/go2_onboard_bringup/README.md` | 给现有实机诊断用户增加新控制链路入口。 |
| 修改 | `real/go2_onboard_bringup/config/nav2.template.yaml` | 机载 Nav2 全部禁止横移，保留其他速度与容差配置。 |
| 新增 | `real/go2_onboard_bringup/launch/stm32_bringup.launch.py` | 新增机载桥接入口，包含通信包启动；不自动启动 Mission。 |
| 修改 | `real/go2_onboard_bringup/package.xml` | 声明新的通信包运行依赖。 |
| 新增 | `real/quadruped_stm32_bridge/README.md` | 通信包职责与默认关闭运动说明。 |
| 新增 | `real/quadruped_stm32_bridge/config/stm32.yaml` | 集中配置串口、时序、标定、话题与速度限制，默认不开放运动。 |
| 新增 | `real/quadruped_stm32_bridge/launch/stm32_bringup.launch.py` | 组合网关、适配器、实机仲裁器和可选Hurricane，严格校验显式启用参数。 |
| 新增 | `real/quadruped_stm32_bridge/package.xml` | ROS 2/pyserial/接口/仲裁依赖。 |
| 新增 | `real/quadruped_stm32_bridge/protocol/test_vectors.json` | 提供 HELLO/WALK/ACK 字节测试向量供 C 端联调。 |
| 新增 | `real/quadruped_stm32_bridge/quadruped_stm32_bridge/__init__.py` | Python包标识。 |
| 新增 | `real/quadruped_stm32_bridge/quadruped_stm32_bridge/gateway_core.py` | ROS无关的会话、授权、队列、ACK、技能关联、超时与安全封锁逻辑。 |
| 新增 | `real/quadruped_stm32_bridge/quadruped_stm32_bridge/nav_velocity_adapter.py` | 将新鲜 Nav2 Twist 绑定当前 NAV lease，拒绝横移/超限，绝不直达串口。 |
| 新增 | `real/quadruped_stm32_bridge/quadruped_stm32_bridge/protocol.py` | 协议v1编码、CRC32、流解析与严格真实状态校验。 |
| 新增 | `real/quadruped_stm32_bridge/quadruped_stm32_bridge/serial_gateway.py` | 唯一UART拥有者，后台I/O、TIOCEXCL、有限重连和最终带授权命令校验。 |
| 新增 | `real/quadruped_stm32_bridge/quadruped_stm32_bridge/skill_transport.py` | 导出已有 GatewayTransport，不另建任务FSM或ActionServer。 |
| 新增 | `real/quadruped_stm32_bridge/quadruped_stm32_bridge/telemetry_adapter.py` | 新鲜真实STATUS映射为MotionTelemetry；不复制Arbiter的服务职责。 |
| 新增 | `real/quadruped_stm32_bridge/resource/quadruped_stm32_bridge` | ament索引标记。 |
| 新增 | `real/quadruped_stm32_bridge/setup.cfg` | ROS 2 Python可执行程序安装路径。 |
| 新增 | `real/quadruped_stm32_bridge/setup.py` | 注册3个节点、配置、launch和C向量；测试夹具不安装为运行节点。 |
| 新增 | `real/quadruped_stm32_bridge/test/test_gateway_ros.py` | PTY+真实ROS节点全链路、默认实际launch隔离；fake MCU仅测试使用。 |
| 新增 | `real/quadruped_stm32_bridge/test/test_launch_contract.py` | 默认禁用、显式启用条件、实机仲裁配置、Nav2无横移与参数名回归。 |
| 新增 | `real/quadruped_stm32_bridge/test/test_protocol_gateway.py` | 协议与核心授权/重放/技能/超时的无硬件测试。 |
| 新增 | `real/quadruped_stm32_bridge/test/test_protocol_vectors.py` | 逐字节验证公开给C端的测试向量。 |
| 新增 | `real/quadruped_stm32_bridge/test/test_ros_adapters.py` | 实际ROS速度/遥测适配、过期/重放拒绝和仲裁最终输出测试。 |
| 新增 | `real/quadruped_stm32_bridge/test/test_serial_worker.py` | PTY分包双向、独占、断线重连和次数上限测试。 |
| 修改 | `tools/install_real.bash` | 把新通信包纳入实机安装，更新默认禁用说明；不启动控制。 |
| 修改 | `tools/tests/test_install_real.py` | 安装测试夹具新增通信包目录，保留所有原安装安全断言。 |
| 修改 | `tools/verify_real.bash` | 构建与验证纳入通信包，保留原Mission/安全/模拟回归。 |

没有删除文件。保留的 `hurricane_protocol.py` 含旧编码/SerialTransport，正式 Hurricane 服务器不再构造它；兼容与测试仍可使用。SimulationBackend、旧 Mock 测试夹具及模拟仲裁配置保留用于原有安全回归，新实机 Launch 不包含这些节点。新增 fake MCU 仅存在于源码测试目录，不注册运行节点。

## 4. 实际环境与命令

环境：Windows 的 WSL Ubuntu 22.04、ROS 2 Humble，工作区 `/home/zxy/rc2026_release_ws`，apt pyserial 3.5；未连接实体 STM32、MID-360、电机或急停。ROS 测试使用 localhost DDS 与隔离 domain，PTY 使用 `/dev/pts/*`，不是实体 UART。

构建实际执行：

```bash
source /opt/ros/humble/setup.bash
source /home/zxy/rc2026_release_ws/install/local_setup.bash
cd /home/zxy/rc2026_release_ws
colcon build --executor sequential --base-paths /home/zxy/rc2026_deom \
  --packages-select go2_obstacle_interfaces go2_nav_bringup robocon_mission \
    go2_mission_backends quadruped_stm32_bridge go2_onboard_bringup \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

结果：6 包成功。原有传感器/GICP依赖使用先前已构建的同工作区安装，本次没有重编译未修改的全部第三方源码。重复构建有 colcon underlay override 提示，接口未改。当前为 symlink install，之后 Python/YAML 小修已由最终针对性测试读取源码验证。

两轮完整检查实际执行（各自独立报告目录）：

```bash
bash tools/verify_real.bash --workspace /home/zxy/rc2026_release_ws \
  --report-dir /home/zxy/rc2026_release_ws/log/stm32_bridge/full \
  --skip-build --include-wall-clock
bash tools/verify_real.bash --workspace /home/zxy/rc2026_release_ws \
  --report-dir /home/zxy/rc2026_release_ws/log/stm32_bridge/full_final \
  --skip-build --include-wall-clock
```

最终小范围验证实际执行：

```bash
python3 -m pytest -q real/quadruped_stm32_bridge/test \
  common/go2_fastlio_localization/test \
  real/go2_onboard_bringup/test/test_doctor_ros.py \
  real/go2_onboard_bringup/test/test_planning_graph.py \
  --junitxml=/home/zxy/rc2026_release_ws/log/stm32_bridge/targeted_frozen.xml
ctest --test-dir /home/zxy/rc2026_release_ws/build/go2_nav_bringup --output-on-failure
python3 tools/smoke_mission_backends.py --domain 176 \
  --output-dir /home/zxy/rc2026_release_ws/log/stm32_bridge/launch_smoke
```

失败项另实际单独复测：

```bash
python3 -m pytest -q \
  'common/go2_mission_backends/test/test_backend_mission.py::test_mission_two_distinct_obstacles_and_return[wall_clock]'
python3 -m pytest -q \
  'common/robocon_mission/test/test_mission_integration.py::test_existing_mock_skill_contract[success-FINISHED-5]' \
  common/robocon_mission/test/test_mission_integration.py::test_full_shipped_sim_route_with_stops_and_state_only_actions
```

## 5. 验证结果（重复覆盖不能相加）

| 检查 | 实际结果 |
| --- | --- |
| 修改/新增 Python 语法、package.xml、两个 Bash 脚本语法 | 通过；最终再次检查 |
| git diff whitespace检查、注册文件存在性、Launch默认值/参数/隔离 | 静态检查通过；安装注册回归也纳入完整测试 |
| 6个受影响功能包构建 | 通过，最后一次14.8s |
| 最终 bridge + 定位/doctor/planning 补充测试 | **167 passed / 11.64s，0fail/0skip**；其中bridge143项、补充24项 |
| Hurricane 全部测试（独立执行） | **156 passed / 9.98s**，含真实ROS Action运输验证 |
| Nav2 CTest | **6/6 passed / 1.96s**，包含非横移采样闭环 |
| 独立软件启动 smoke | **success=true / 24.42s**；两障碍+返航，launch及子进程已回收 |
| 3份受影响Launch --show-args | 通过：bridge、onboard wrapper、独立Hurricane |
| 首轮全量 verify_real | **726 passed、1 failed / 367.93s**，脚本退出1 |
| 第二轮全量 verify_real | **729 passed、2 failed / 372.62s**，脚本退出1 |
| 首轮 wall-clock失败项单独重测 | **1 failed**（改在NAV阶段安全中止） |
| 第二轮两个失败项单独重测 | **1 passed、1 failed / 15.19s**；29点巡航仍未通过 |
| 实体UART、STM32、电机、急停、实际导航/越障 | **未执行** |

完整脚本因 pytest 失败提前结束，没有执行其后续 CTest/smoke；上表这两项是另行独立实际执行，不能把脚本报为成功。所有适用原测试仍保留，没有删掉、deselect或改断言来隐藏失败。

首轮新增测试还发现两项测试夹具错误，均修复后重测：超时用例误把代次9的续约写成代次7（按同代次续约纠正），以及伪串口非阻塞读取在 select 后仍可能 EAGAIN（按正确I/O语义继续等待）。没有为此修改生产安全阈值；后者原始失败见 targeted_final.log，复测见 targeted_final_retry 与 targeted_frozen。

## 6. 失败原文、诊断与边界

首轮：

```text
test_mission_two_distinct_obstacles_and_return[wall_clock]
AssertionError: control ownership SKILL stale, lost or unknown
```

单独重测：

```text
AssertionError: control ownership NAVIGATION stale, lost or unknown
robot_state_adapter: VELOCITY_WARMUP
```

第二轮：

```text
test_existing_mock_skill_contract[success-FINISHED-5]
  ABORTED (WAIT_SKILL_SAFE: control ownership SKILL stale, lost or unknown)
test_full_shipped_sim_route_with_stops_and_state_only_actions
  ABORTED (WAIT_WAYPOINT_STOP: control ownership NAVIGATION stale, lost or unknown)
```

29点路线单独复测依然在后续点 WAIT_WAYPOINT_STOP 因 NAVIGATION 状态过期中止，不能标为通过。

已取得的独立证据：

1. 第一轮 wall-clock 重测中，相邻定位证据 steady 仅增加0.030422s，ROS stamp却增加1.929172s，净差1.898751s，随后触发过期。可确认时间不连续，尚不能断言具体是何种 WSL/系统服务机制。
2. 第一轮完整case出现sequence189→190但stamp倒退15.068ms。只读两线程复现直接调用与基线字节相同的 SimulationBackend._tick，在输入时钟严格递增时也能产生“seq递增、stamp倒退”：旧timer使用ReentrantCallbackGroup，采样/序号/发布不处于一个互斥事务。确认原测试后端存在重入风险，但没有唯一还原每次失败的现场原因。
3. 用 git show 提取基线旧 MotionArbiter，在临时Python进程替换导入、domain186单独跑原 wall-clock用例，1 passed / 23.86s。单次通过不能消除随机失败，也不能单凭它断言本次修改完全无关。
4. 第二轮两个障碍+返航的wall-clock和sim-clock主用例分别FINISHED（23.34s/23.18s）；其他原集成用例仍出现墙钟状态过期，故整体稳定性验收未过。

本次没有放宽任何门控，没有修改系统时间服务，也没有扩大修改 SimulationBackend。应在时钟稳定的原生机载Ubuntu复核，并单独修复测试后端并发发布风险，之后再跑完整验证。

原始日志/可重复脚本/JSON/JUnit保留在本机：

```text
/home/zxy/rc2026_release_ws/log/stm32_bridge/
  build.log, static_checks.json, protected_unchanged.json
  full/tests.log, full/tests.xml, full/backend_cases/
  retry_wall_clock.log, retry_wall_clock.xml, retry_wall_clock_cases/
  full_final/tests.log, full_final/tests.xml, full_final/backend_cases/
  targeted_final.log, targeted_final_retry.log, targeted_frozen.log/.xml
  retry_integration.log/.xml, nav_final.log, launch_smoke/
  baseline_wall.log/.xml, baseline_wall_cases/
  wall_clock_diagnosis.md, simulation_reentry_probe.json/.log
```

## 7. 仍需STM32和硬件团队完成

详见 [接入指南](STM32_BRIDGE_GUIDE.md) 与 [固定版本固件审查](STM32_FIRMWARE_AUDIT.md)。参考固件没有实现v1，不能宣称即插即用。必须协商UART/引脚/3.3V、实现C端CRC与去重/本地TTL、真实人工使能与急停、停稳/支撑/可释放证据、跳跃请求关联与安全取消；F429/H743能力并不相同。现有STOP会退出电机使能，不能作为HOLD。

实际速度正负号、full_scale与非线性、制动/步态、支撑承重、跳跃落地、MCU watchdog和物理急停均未验证。原机载Nav2仍有静态地图诊断costmap限制；动态避障、实际足迹、定位精度和比赛完赛能力不在本次已验证范围。

## 8. Git修改摘要

按 2026-10-17 用户要求，将开发分支的实现提交并快进发布到 `rely-only`，不修改或合并 `main`。以下为相对修改前基线的差异摘要（含本报告）：

```text
 README.md                                          |  12 +-
 .../go2_mission_backends/config/arbiter_real.yaml  |  22 +
 .../go2_mission_backends/motion_arbiter.py         |  12 +-
 .../config/autonomy/fast_lio_nav2.yaml             |  23 +-
 .../go2_nav_bringup/src/precision_pose_critic.cpp  |  29 +-
 .../go2_nav_bringup/test/test_precision_pose.cpp   |  49 ++
 common/robocon_mission/HURRICANE_PORT.md           |  37 +-
 common/robocon_mission/launch/hurricane.launch.py  |   2 -
 .../robocon_mission/hurricane_gateway_transport.py | 328 +++++++++++++
 .../robocon_mission/hurricane_server.py            | 101 +++-
 .../test/test_hurricane_gateway_transport.py       | 291 +++++++++++
 .../test/test_hurricane_server_ros.py              | 167 ++++++-
 .../robocon_mission/test/test_simulation_launch.py |   7 +-
 docs/ARCHITECTURE.md                               |   3 +
 docs/CONTROL_INTEGRATION.md                        |   3 +
 docs/REAL_ROBOT_GUIDE.md                           |  11 +-
 docs/REAL_ROBOT_QUICK_REFERENCE.md                 |   3 +
 docs/STM32_BRIDGE_GUIDE.md                         | 237 +++++++++
 docs/STM32_BRIDGE_VALIDATION.md                    | 269 +++++++++++
 docs/STM32_FIRMWARE_AUDIT.md                       |  97 ++++
 real/go2_onboard_bringup/README.md                 |   3 +
 real/go2_onboard_bringup/config/nav2.template.yaml |  21 +-
 .../launch/stm32_bringup.launch.py                 |  11 +
 real/go2_onboard_bringup/package.xml               |   2 +-
 real/quadruped_stm32_bridge/README.md              |  12 +
 real/quadruped_stm32_bridge/config/stm32.yaml      |  56 +++
 .../launch/stm32_bringup.launch.py                 |  59 +++
 real/quadruped_stm32_bridge/package.xml            |  15 +
 .../protocol/test_vectors.json                     |  41 ++
 .../quadruped_stm32_bridge/__init__.py             |   1 +
 .../quadruped_stm32_bridge/gateway_core.py         | 534 +++++++++++++++++++++
 .../quadruped_stm32_bridge/nav_velocity_adapter.py | 137 ++++++
 .../quadruped_stm32_bridge/protocol.py             | 167 +++++++
 .../quadruped_stm32_bridge/serial_gateway.py       | 259 ++++++++++
 .../quadruped_stm32_bridge/skill_transport.py      |   8 +
 .../quadruped_stm32_bridge/telemetry_adapter.py    | 125 +++++
 .../resource/quadruped_stm32_bridge                |   0
 real/quadruped_stm32_bridge/setup.cfg              |   4 +
 real/quadruped_stm32_bridge/setup.py               |  16 +
 .../test/test_gateway_ros.py                       | 508 ++++++++++++++++++++
 .../test/test_launch_contract.py                   |  75 +++
 .../test/test_protocol_gateway.py                  | 429 +++++++++++++++++
 .../test/test_protocol_vectors.py                  |  14 +
 .../test/test_ros_adapters.py                      | 154 ++++++
 .../test/test_serial_worker.py                     | 113 +++++
 tools/install_real.bash                            |   4 +-
 tools/tests/test_install_real.py                   |   2 +-
 tools/verify_real.bash                             |   6 +-
 48 files changed, 4392 insertions(+), 87 deletions(-)
```
