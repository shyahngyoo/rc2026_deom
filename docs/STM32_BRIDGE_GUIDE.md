# STM32 单串口接入指南（协议 v1 草案）

本模块面向自制四足机器人，不使用宇树 SDK，不使用 E34 或无线遥控。ROS 2 上位机不生成关节轨迹。**现有 STM32 参考固件没有实现本协议；目前交付的是可用伪串口验证的软件接入基础，不能插线后直接跑比赛。**

## 1. 已接入的软件路径

```mermaid
flowchart LR
  N[Nav2 /onboard/nav_cmd_raw] --> V[NavVelocityAdapter]
  V -->|MotionCommand NAVIGATION| A[MotionArbiter]
  H[Hurricane /execute_obstacle] -->|MotionCommand SKILL| A
  A -->|仲裁后 MotionCommand /stm32/arbiter_command| G[唯一 SerialGateway]
  H -->|离散技能请求，绑定租约| G
  G <-->|USB-TTL 协议 v1| S[STM32 固件，待实现]
  G -->|真实新鲜 STATUS| T[TelemetryAdapter]
  T -->|/motion/telemetry| A
  A --> C[/motion/authority、control_state、get_state]
```

- `MotionArbiter` 继续独占 `/motion/control_state`、`/motion/authority`、`/motion/get_state`、`/motion/set_control_mode`；状态适配器不重复提供这些服务。
- 普通 `/stm32/arbiter_cmd_vel` 为仲裁后 Twist 诊断输出。UART 只消费带 `owner/mission_id/generation/header` 的 `/stm32/arbiter_command`，避免旧 Twist 在新租约下误执行。
- `/stm32/skill_request`、`/stm32/status`、`/stm32/event` 是同机 ROS 内部 JSON 接口，不是 MCU 线上的原始数据。只有网关打开串口。
- `MissionFSM` 和 Mission 调用方式不变。动作完成后仍需实测停稳、安全支撑、控制权释放和新的出口定位验收。
- `vy` 必须严格等于零。两份 Nav2 中所有 DWB 控制器和 smoother 已禁止横移，速度适配器和网关再次拒绝非零 Y。`PrecisionPoseCritic.holonomic=false` 通过转向和前后移动修正位置后再对准终点朝向；旧插件使用方仍保留默认 holonomic 行为。
- 现有软件模拟后端保留用于回归测试，实机入口不包含它。

## 2. Ubuntu 22.04 + ROS 2 Humble 构建

以下命令以最新 `rely-only` 的完整源码已放入工作区为前提。构建前确认 `real/quadruped_stm32_bridge/package.xml` 存在；旧 checkout 需先更新到包含本模块的版本，不能把旧版本安装成功误认为通信包已经部署。

已有完整依赖工作区时（把路径替换为实际路径）：

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
cd ~/rc2026_ws
sudo apt-get install python3-serial
colcon build --symlink-install --executor sequential \
  --packages-up-to go2_onboard_bringup quadruped_stm32_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

新机首次安装依然使用仓库 `tools/install_real.bash --workspace ~/rc2026_ws`，它已包含新增通信包，不会启动电机。安装器会拒绝未知修改或旧 recipe 指纹；如果旧工作区报 `install recipe changed`，保留旧工作区，选一个新的专用 workspace 安装，不要删除元数据或重置第三方源码来绕过检查。

构建成功不代表固件已实现协议或整机安全验证通过。

## 3. 第一次 USB-TTL HELLO/OK（禁止运动）

先让 STM32 同学实现第 6 节协议的 HELLO/ACK/STATUS。由其确认可用 UART、RX/TX 引脚和 3.3 V TTL 电平。USB-TTL 的 TX 接 MCU RX，RX 接 TX，GND 共地；不要使用 RS-232 电平，不用串口模块给电机供电。参考源码的 USART1 目前只有调试 TX、UART8 仍属于 E34 驱动，直接插上并不会响应本协议。

确认设备与权限：

```bash
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$USER"
# 加组后重新登录；选稳定的 by-id 路径。
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
ros2 launch go2_onboard_bringup stm32_bringup.launch.py \
  port:=/dev/serial/by-id/REPLACE_WITH_YOUR_USB_TTL
```

此命令保留 `test_mode=true`、`enable_motion=false`、`calibration_confirmed=false`，只做 HELLO 与反馈观察，不启动 Hurricane 或 Mission，不发送 WALK/技能/自动电机使能。

另一个终端观察：

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
ros2 topic echo /stm32/status
ros2 topic echo /stm32/event
ros2 topic echo /motion/telemetry
ros2 service call /motion/get_state go2_obstacle_interfaces/srv/GetMotionState '{obstacle_id: ""}'
```

“OK”指收到 CRC 正确、同 session、匹配 HELLO 序号和类型的 ACK，`accepted=true`、`protocol_version=1`，且声明必需协议 features。不是接收到 ASCII 字符 `OK`。如果 MCU 暂时只实现 HELLO/STATUS 而尚未实现支撑 HOLD 等能力，应如实省略这些 feature；主机会报告 HANDSHAKE_UNSUPPORTED。此时可证明串口有双向收发，但不算完整握手通过，不得为了得到 OK 虚报能力。随后必须持续收到新 STATUS，才能得到真实状态。默认禁用仲裁器下 `/motion/get_state` 返回 unknown 是预期行为，不要为此伪造安全数据。

无响应检查：接线、电平、端口占用、波特率、固件接收 DMA/中断、协议版本及 CRC。`RECONNECT_EXHAUSTED` 后先修复，再人工重启整条桥接启动链，不会无限自动重连。不要同时开串口终端、旧 Hurricane 串口程序和网关。

## 4. 真实运动前需要哪些条件

仓库默认配置不是实机标定值。先完成固件协议、独立急停、人工使能、支撑与取消策略，并在台架上验证，再准备外部配置：

```bash
mkdir -p ~/.config/rc2026
cp "$(ros2 pkg prefix quadruped_stm32_bridge)/share/quadruped_stm32_bridge/config/stm32.yaml" \
  ~/.config/rc2026/stm32.yaml
```

测量 `vx_full_scale`（m/s 对应 normalized forward=1）和 `wz_full_scale`（rad/s 对应 normalized turn=1），确认 ROS 正 X/正 yaw 与 C 端符号一致；若方向相反，必须在固件协议适配或经审查的标定映射中纠正。当前是线性草案，不能把示例 0.5 m/s、1.0 rad/s 当成实测值。检查正负向是否对称；若明显非线性，先扩展并测试标定映射再启用。

`max_linear_speed/max_angular_speed` 必须与 MotionArbiter 的实机限幅协调，且不大于 full_scale。Hurricane 的 `skill_forward_speed/skill_turn_speed` 也要在这些上限内。未标定的技能文件仍会拒绝执行。

只有完成上述联调后，操作人员才可显式启用：

```bash
ros2 launch go2_onboard_bringup stm32_bringup.launch.py \
  port:=/dev/serial/by-id/REPLACE_WITH_YOUR_USB_TTL \
  params_file:=$HOME/.config/rc2026/stm32.yaml \
  test_mode:=false enable_motion:=true calibration_confirmed:=true \
  start_hurricane:=true skills_file:=/ABSOLUTE/PATH/calibrated_hurricane_skills.yaml
```

这三个软件开关不代替物理急停、人工使能或 MCU 本地保护。协议没有自动 ARM/ENABLE 消息。启用软件输出后，仍需真实 human_enable、无 estop/故障、实时状态和有效仲裁租约，才可能发出运动请求。只运行一个 Hurricane `/execute_obstacle` Server；使用组合启动时不要再运行独立 `hurricane.launch.py`。

## 5. 与定位、Nav2 和 Mission 一起运行

原 `real.launch.py config_dir:=... mode:=navigation` 仍启动 MID-360、FAST-LIO、GICP、状态与定位证据适配、Nav2；其中速度输出保持 `/onboard/nav_cmd_raw`。新桥接入口独立启动，便于先监测串口再接导航。

旧的 onboard doctor 与 `/mission/preflight` 保留诊断禁止运动的语义，不要将其返回值修改成 true。桥接联调阶段，需要另起已有的通用 preflight，使用新的仲裁后输出与实际路线，例如：

```bash
ros2 run go2_mission_backends preflight --ros-args \
  -r __node:=stm32_mission_preflight \
  -p service_name:=/stm32/preflight \
  -p route_file:=/ABSOLUTE/PATH/course.yaml \
  -p output_topic:=/stm32/arbiter_command \
  -p expected_output_node:=motion_arbiter \
  -p required_nav_nodes:="['/controller_server','/planner_server','/bt_navigator']"
ros2 service call /stm32/preflight std_srvs/srv/Trigger '{}'
ros2 service call /skills/ready std_srvs/srv/Trigger '{}'
```

`/skills/ready` 只列出经过现场标定、且 MCU 声明支持所需离散动作的技能。桥梁、高墙等 Hurricane 组合技能不等于 MCU 已实现同名技能；缺 mode 或 jump 能力时必须 UNSUPPORTED。

只有预检查真实通过后，启动既有 Mission（初始 IDLE，不自动开跑）：

```bash
ros2 launch robocon_mission mission.launch.py \
  route_file:=/ABSOLUTE/PATH/course.yaml \
  accepted_pose_topic:=/localization/evidence accepted_pose_type:=evidence \
  accepted_pose_body_frame:=livox_frame startup_check_service:=/stm32/preflight \
  use_sim_time:=false simulation_mode:=false autostart:=false
# 操作员确认场地、急停和路线后再手工开始。
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

不要将 `/onboard/nav_cmd_raw` 或普通 `/cmd_vel` 直接接到电机。模式切换规则：有当前 NAV lease 和实际执行中的 NavigateToPose goal 才请求 NAVIGATION 模式；STM32 确认 nav_active 后仲裁器才放非零速度。goal 结束/取消会请求 HOLD；真实报告 inactive + stopped + safe_stance + releasable 后，才允许原有 NAV→SKILL 切换。SKILL 执行时导航速度不能进入串口；离散跳跃中使用 HEARTBEAT 续约，不能用 WALK(0,0) 取消轨迹。

已有外部 profile 中生成过的 `nav2.yaml` 不会自动更新。请备份后按照新模板只修改 DWB/smoother 的 Y 配置与 `PrecisionPose.holonomic=false`，保留实测 footprint、地图和其余标定。不要对实机 profile 盲目 `--force` 覆盖。

当前 Nav2 onboard 模板仍为静态地图诊断 costmap，动态障碍物接入、实机足迹/雷达盲区、运动与定位效果均需验证。本通信模块交付不能解释为已可自动跑完整障碍赛。

## 6. 线协议 v1 草案

所有整数为 little-endian；串口默认 115200，8N1。无协议自动使能电机。最大 JSON payload 1024 bytes，最大帧 1046 bytes。

| Offset | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | magic `A5 5A` |
| 2 | 1 | version `01` |
| 3 | 1 | message type |
| 4 | 2 | UTF-8 JSON 字节数，不含 header/CRC |
| 6 | 4 | sequence，非零 uint32 |
| 10 | 8 | session，主机每次串口连接新生成的非零 uint64 |
| 18 | length | 紧凑 JSON object，不允许重复键、NaN/Inf |
| 18+length | 4 | CRC32(header+payload)，little-endian |

CRC 为 Python `zlib.crc32` 对应 CRC-32/ISO-HDLC，反射多项式 `0xEDB88320`，初值 `0xFFFFFFFF`、xorout `0xFFFFFFFF`。不是旧无线 CRC16。

| Type | 名称 | payload 要点 |
| --- | --- | --- |
| 1 | HELLO | `protocol_version:1`，host 标识；只协商，不使能 |
| 2 | SET_MODE | `mode`、`mission_id`、`generation`；模式接收不代表实际 active |
| 3 | WALK | `forward/turn` 归一化 `[-1,1]`；`owner/mission_id/generation/valid_for_ms` |
| 4 | SKILL_START | `command_id`、`request_id`、`runner_session`、`obstacle_id`、`step_index` |
| 5 | SKILL_CANCEL | 取消关联技能请求，结束必须另由状态确认 |
| 6 | HOLD | 请求保持支撑；不能用 motor STOP 假充 |
| 7 | STOP | 驱动停机/卸力请求，需当前有效 SKILL 授权，与站立支撑不同，不自动发送 |
| 8 | HEARTBEAT | 当前 owner/mission/generation/TTL；不修改正在执行的离散轨迹 |
| 128 | ACK | `acked_sequence/acked_type/accepted`，可带 reason；不是完成 |
| 129 | STATUS | 下表的真实状态与动作关联 |
| 130 | FAULT | `code`，收到后封锁运动 |

HELLO ACK 示例（ACK 帧仍有自己的 device sequence）：

```json
{"acked_sequence":1,"acked_type":1,"accepted":true,"protocol_version":1,"features":["HOLD","SET_MODE","WALK","STATUS","HEARTBEAT","SKILL_START","SKILL_CANCEL"]}
```

必需 `HOLD/SET_MODE/WALK/STATUS/HEARTBEAT`，缺少其中任意能力不会完成握手。额外技能能力还需 STATUS 的 capabilities 显式声明。

STATUS 必填字段：

| 字段 | 类型/语义 |
| --- | --- |
| known | bool，控制器自身状态可信 |
| mode | `HOLD/NAVIGATION/SKILL/DISABLED` |
| nav_active、skill_active | bool，真实执行器活动，不能同时 true；不是主机模式请求的回显 |
| stopped | bool，实际停稳，不是“上次收到零速度” |
| safe_stance | bool，已验证支撑，不能等同 static_support_requested |
| control_releasable | bool，实际允许交接；活动或危险阶段不得释放 |
| estop、human_enable | bool，独立急停和人工授权实测状态 |
| obstacle_id、fault_code | string；无故障用空字符串 |
| skill_state | `idle/accepted/running/succeeded/failed/canceled` |
| capabilities | unique int[]，实际支持的 Hurricane 离散编号 1..18；19/20 永不支持 |
| request_id、runner_session、step_index | 技能关联，必须回显当前请求，旧终态不能完成新动作 |

旧 Hurricane 1..4 只在 ROS 侧映射成受仲裁的前后/转向速度，不直接把旧编号原样当固件 API。5 对应 HOLD 请求，6/7 是两个待映射跳跃，8..18 均无预设固件支持。C 端应明确哪些离散编号支持哪种真实 API，不能假设 6=原地跳、7=前跃已经谈妥。

时序/防重放约定：

- 两个方向各自维护 sequence，同连接 session 下严格递增；到 uint32 上限关闭会话、重新 HELLO，不能绕回。
- MCU 对 `(session,sequence)` 重复帧只回放原 ACK，不重复起跳；未缓存且小于已接收最高 sequence 的旧命令拒绝。新 session 清除旧速度/租约/排队动作，进入受控安全策略，不恢复运行。
- 默认 status/authority timeout 为 0.3 s、command TTL 为 0.2 s；ACK timeout 0.15 s，最多 2 次重试；HELLO timeout 2 s，串口重连间隔 1 s，总尝试最多 5 次。以外部 yaml 为准，C 端必须设独立硬上限，不能任主机要求无限 TTL。
- ACK 重发保留原 sequence；START 只能执行一次；新速度取代旧未完成的速度发送。相同 runner/request 重复请求不得再次执行；旧 WALK/HEARTBEAT 重试不得续 TTL 或恢复已被新 sequence 取代的速度。
- STATUS 重复 sequence、错误 CRC、错误 session、ACK 和旧终态都不能刷新安全反馈；串口连接不等于 known，不等于停稳。
- 主机断线、过期、失去租约、故障后清理队列与 pending；重连换 session，必须等待新的控制权 generation 和新命令。原任务应取消并人工评估，不自动续跑。
- MCU 本地 watchdog 必须在主机进程死亡、USB拔出、通信停顿时独立触发经实机验证的支撑/落地/停机策略。主机尝试发 HOLD 不能替代这个机制。

稳定 HELLO 测试向量（只含 `protocol_version:1`）位于 `protocol.py:TEST_VECTOR_HELLO` 和安装后的 `protocol/test_vectors.json`。C 端必须按字节比较，不依赖 JSON 对象键顺序重新计算接收帧 CRC。可通过 `python3 -m pytest real/quadruped_stm32_bridge/test/test_protocol_gateway.py` 验证。

## 7. STM32 同学需要完成的工作

源码证据详见 [STM32 固件只读审查](STM32_FIRMWARE_AUDIT.md)。只读核对参考仓库 commit `bec442861ebd0d570e95d8a4d82873394d8f0f7c`，本任务没有修改该仓库。

1. 指定自动控制 UART 与接线；实现新帧接收、CRC、长度、session、sequence 和重复命令处理，不复用 E34 的无线地址/AUX/27-byte 限制。
2. 为 HELLO 提供真实 feature 协商，为 SET_MODE/WALK 提供显式授权与本地 TTL；`horse_app_set_motion` 接受的是归一化量，不能直接填 m/s、rad/s。
3. 实现真正支撑 HOLD 与安全取消。现有 `horse_app_request_stop` 会进入 STOPPING、发送电机 STOP；F429 的 `horse_app_set_static_support` 仅是支撑请求，H743 尚无等价 app API。
4. 增补停稳、支撑确认、可释放控制权、物理急停和人工使能的真实测量/判据。F429 `horse_app_get_snapshot` 目前不能直接给出这些完整证据；H743 还缺 snapshot 封装。
5. 把跳跃预检、运行和完成/失败/取消状态绑定本次 request/session/step。F429 `horse_app_get_jump_status` 的 COMPLETE 是历史状态，不能用于完成下一次新请求。
6. F429 有前跃 API，H743 未发现；桥梁/大坡/匍匐/高墙等旧模式没有同名现成固件接口，未知能力返回 UNSUPPORTED。
7. 本地通信 watchdog、故障保护、取消时落地/支撑策略和独立物理急停需要上电台架与整机验证。

## 8. 软件验证与实机验证分开记录

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
python3 -m pytest -q real/quadruped_stm32_bridge/test
bash tools/verify_real.bash --workspace ~/rc2026_ws --include-wall-clock
```

新增测试使用纯 Python、真实 ROS 消息及伪串口，不连接真实 STM32。现有 MissionFSM、控制权仲裁、定位和预检查测试继续执行。此次实际命令、测试结果与失败记录见 `docs/STM32_BRIDGE_VALIDATION.md`。

尚未验证：USB-TTL实物收发、引脚电平、STM32协议实现、速度映射、支撑承重、急停、MCU watchdog、取消落地、定位导航实机效果。不得据软件测试结果宣称实机运动安全。
