# Hurricane 移植与使用

来源为 2026-10-14 同步的 Hurricane `ros2_to_stm32` 上位机代码。移植的是 Unitree 版本的动作编号、八类障碍的航点／模式／跳跃设计，以及两点场地标定、障碍采点功能。没有迁入另一套 Mi 指令编号，也没有复制 FAST-LIO／Livox 驱动或下位机固件。仓库原有的 Mission FSM、ControlHandoff、独立运动状态检查、定位证据与出口验收流程继续使用。

## 已移植的功能

| 文件 | 职责 |
|---|---|
| `hurricane_protocol.py` | 原 `S编号E\r\n` 编码、115200 波特率、持久串口、显式端口与无硬件 DryRun |
| `hurricane_skills.yaml` | 木桥A/B、大斜坡、限高栏、沙坑、绕杆、高墙、台阶的原始动作设计 |
| `hurricane_skills.py` | 非阻塞技能执行、局部坐标转换、独立阶段与完成反馈门控 |
| `hurricane_server.py` | `/execute_obstacle` Action 服务、TF 与控制权检查、串口输出与反馈关联 |
| `hurricane_capture.py` | TF 采点面板、离线采点、八障碍路线与技能参数导出 |

Unitree 原表中前进=2、站立=5、小跳1/2=6/7、下台阶=8、正常模式=9，模式编号一直到台阶=18，左右平移=19/20。`repeat` 表示重复串口帧数，不是动作完成次数。发送成功只证明本机写入；`stand/stop` 仅对应原“站立”命令，不能替代硬件急停。

## 与原代码的必要差异

- 模式与每次跳跃都有独立阶段编号，跳前保留最短准备时间，跳后等待实际完成与安全站立反馈。原部分代码在 jump 后立即发站立，移植版不假定未知下位机需要这种线序；执行器团队必须核对协议与运动语义。本移植不是未知固件的逐字节实车认证。
- 固定 `sleep` 已改为 steady clock 轮询；经过五秒不会自动宣布跳跃完成。模式、跳跃和最终站立确认都需新鲜、匹配 session／step 的外部状态。
- 修复身后目标误到达、沙坑索引越界、同回调同时执行两个目标及共享障碍索引。
- Hurricane 场地坐标的 y 向右，控制 yaw 也被取负。位移和角度均转换为 ROS 右手系；源 yaw 单位 degree，导出 `map` 姿态为 rad。源 `x/y/yaw` 动态值用 null 锁定该航点入口值；源明确不验收 yaw 的首航点用 `require_yaw:false` 保留语义。
- 木桥A末两航点对木桥B起点的引用由实测采点重算。台阶原先混用全场 y 与局部 y 的缺陷已修正。
- 最终航点完成后仍要独立确认安全站立、技能输出释放，并由 Mission 对 `end_pose` 验收。出口只用于验证，绝不作为穿过障碍的 Nav2 目标。

## 无硬件预览

Python 模块预览只读取配置，不需要 ROS，不会打开串口：

```bash
cd /你的路径/robocon_mission
PYTHONPATH=$PWD python3 -m robocon_mission.hurricane_preview --skill bridge_a
```

安装后可用 `ros2 run robocon_mission hurricane_preview`。预览保留原参考坐标且 `calibrated:false`，不代表可直接用在现场。ROS 适配器同样默认 dry-run：

```bash
ros2 launch robocon_mission hurricane.launch.py dry_run:=true
```

它仅提供技能服务器，要求已有新鲜 `map→base_footprint` TF；不启动 Nav2、雷达或机器人。dry-run 的 Action 结果带 `error_code=DRY_RUN`、`safe_stance=false`，不能作为硬件任务完成证据。禁止同时启动 mock_obstacle_skill 或另一个 `/execute_obstacle` 服务器。

## 现场采点与路线导出

先在正式定位地图中完成采点。`start` 与 `finish` 是场地轴的两个标记，`finish` 不等于比赛终点；每类障碍还需 `begin`、`entry`、`exit`。总共 26 点，轴不能退化。begin 定义障碍局部原点，entry 是导航对准入口，exit 是越障结束的实测验收位置。参考模板不能代替采点。

```bash
ros2 run robocon_mission capture_panel --file /绝对路径/field.yaml
# 或录入已测量的 map 坐标：
ros2 run robocon_mission hurricane_capture --file /绝对路径/field.yaml \
  --label bridge_a.entry --pose 1.0 2.0 0.0
ros2 run robocon_mission hurricane_capture --file /绝对路径/field.yaml \
  --export-route /绝对路径/hurricane_route.yaml \
  --skills-profile /已安装路径/config/hurricane_skills.yaml \
  --export-skills /绝对路径/calibrated_skills.yaml
```

工具拒绝过期、未来和零时间戳 TF。导出路线包含八个独立障碍与返回起点；各任务有实测 `end_pose`，符合本仓库的严格路线格式。标定来源与 `calibration.warnings` 写入技能配置，不额外写入路线根字段。导出后仍需检查每项 source 航点与本场障碍尺寸是否一致。已知末航点与出口采点不一致会记录为警告，必须调整技能航点并再次核对。

## 下位机反馈契约与实机接入

实机技能服务器在入口与执行期间持续查询 `/mission/localization_ok`；服务缺失、失败或超时会阻止开始或请求站立并中止，不会仅以新鲜 TF 冒充定位质量有效。

原 UART 协议没有 ACK 或动作完成回包。仓库也不含接收编号的 STM32 固件／Go2 SDK 执行端。实机启动前，已有电控适配器必须把真实测量与执行结果发布到 `/hurricane/controller_feedback`（`std_msgs/String` JSON）；不要用固定定时器、串口 write 返回值或机器人目标位置生成成功反馈。

```json
{
  "stamp_ns": 1770000000000000000,
  "session_id": "当前 /hurricane/status 中的 session_id",
  "obstacle_id": "bridge_a",
  "step_index": 2,
  "event": "completed",
  "completed": true,
  "safe_stance": true,
  "control_released": false
}
```

时间戳必须使用同一 ROS 时钟，为实际测量时间、递增且不早于实际命令发送边界。反馈由当前 session 与 step 唯一关联；每阶段分别确认，不能重放上一阶段状态。`event=progress` 表示仍在执行，`event=failed`、`known=false`、非空 `error_code` 或否定安全状态会阻止推进并停车。最短等待结束时仍要求反馈新鲜，控制器需持续发布当前阶段真实状态。最终阶段 `step_index=len(steps)` 才能确认 `control_released=true`。该标志表示技能输出可释放，不代表仲裁器已经切换 NAVIGATION。

真实仲裁器另须提供 `SetControlMode` 服务 `/motion/set_control_mode` 和 `ControlState` 心跳 `/motion/control_state`，按 mission_id/request_id、expected_owner 与租约实际屏蔽未授权输出。使用本仓库的 `go2_obstacle_interfaces` 并统一重编译；旧 Action 类型缺少 safe_stance/error_code 时会拒绝硬件适配器。仅发零速度或修改 Python owner 不等于安全支撑；物理保护仍由独立底层承担。Nav2 输出与串口技能输出须实际纳入同一仲裁链路。

Hurricane 服务只消费 `/hurricane/controller_feedback`，不提供 `/motion/get_state`。原 Mission 在 Action 成功后仍独立调用 `GetMotionState`，要求真实的 known、安全站姿、技能停止、可释放与无故障状态，并核对 obstacle_id。电控接入须同时提供这两份真实反馈；不能拿 Action 结果代替独立运动状态。原有 MotionTelemetry、MotionAuthority 与 MotionCommand 的底层接入要求见任务包 README 和根 README。

当电控反馈、独立运动状态、仲裁、定位证据、预检查和现场参数均完成对接后，可按以下形式集成。占位参数须替换为真实部署端点：

```bash
ros2 launch robocon_mission hurricane.launch.py \
  skills_file:=/绝对路径/calibrated_skills.yaml \
  dry_run:=false serial_port:=/dev/serial/by-id/你的电控设备
ros2 launch robocon_mission mission.launch.py \
  route_file:=/绝对路径/hurricane_route.yaml \
  simulation_mode:=false use_sim_time:=false autostart:=false \
  startup_check_service:=/你的实机预检查服务 \
  accepted_pose_topic:=/你的真实定位证据话题 \
  accepted_pose_type:=evidence \
  accepted_pose_body_frame:=你的定位安装体坐标系 \
  accepted_pose_source_id:=你的定位证据来源
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

本仓库没有 `hardware_skills` 开关；真实模式沿用完整的任务门控。先确认 NAVIGATION 所有权，再定位与导航；入口停稳后获取 SKILL；动作结果与独立运动状态通过后转 SAFE_HOLD；获得新定位证据并验收实测出口后才恢复 NAVIGATION。未知终态、心跳丢失、错误出口与缺失反馈均不能完成当前障碍。现有机载入口不会自动启动这套 Mission；新增技能服务不改变这一部署边界。

## 验证范围

新增测试覆盖协议、阶段推进、场地导出、反馈关联，以及真实 ROS Action 的成功、错误 session 和未释放控制权分支。测试使用 fake 串口、合成 TF、测试控制心跳与测试执行反馈，无真实电机命令。原 Mission 测试继续验证既有任务门控。测试证明软件接口与失败分支，不证明下位机编号语义、实车定位精度或比赛完赛能力。
