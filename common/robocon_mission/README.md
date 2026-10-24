# robocon_mission

ROS 2 Humble 的上层任务调度器。Mission 负责路线、导航与越障 Action 的顺序、控制权交接、停稳和出口验收，不生成腿部轨迹、步态或关节命令。

本发布版的机载入口 `go2_onboard_bringup/real.launch.py` **不启动实机 Mission**。当前可部署范围为真实 MID-360 输入、FAST-LIO、GICP 定位、机器人状态适配、隔离的 Nav2 规划诊断和自检。安装与外部配置步骤见[根 README](../../README.md)，软件验证范围见[验证记录](../../docs/VALIDATION.md)。默认工作空间为 `~/rc2026_ws`。

主机侧已提供 NavVelocityAdapter、STM32 TelemetryAdapter 和 Hurricane 网关运输层，分别完成带租约速度适配、真实 STATUS 转换和越障请求关联。实机自动跑赛前仍须 STM32 实现协议、真实安全状态与技能执行，完成现场标定和联调；启动方式见 [STM32 接入指南](../../docs/STM32_BRIDGE_GUIDE.md)。软件仲裁器不能代替底层支撑控制或硬件急停；模拟状态和测试执行器不能用于替代实机反馈。

## FSM 与任务闭环

状态机位于 [mission_fsm.py](robocon_mission/mission_fsm.py)，ROS 适配位于 [mission_manager.py](robocon_mission/mission_manager.py)。异步命令带 token，过期结果不能推进后续任务。

```text
IDLE → WAIT_LOCALIZATION → RESTORING_NAV → SELECT_TASK
普通点：NAVIGATING → APPROACHING_WAYPOINT → WAIT_WAYPOINT_STOP
      → 停稳与原始目标位姿验收 → SELECT_TASK
障碍：NAVIGATING → APPROACHING → WAIT_STOP
    → ACQUIRING_CONTROL → EXECUTING_SKILL → WAIT_SKILL_SAFE
    → RELEASING_CONTROL → WAIT_LOCALIZATION → VERIFYING_END
    → RESTORING_NAV → 完成当前障碍 → SELECT_TASK
返航：RETURNING → APPROACHING_RETURN → WAIT_WAYPOINT_STOP → FINISHED
定位恢复：确认 SAFE_HOLD → RELOCALIZING → WAIT_LOCALIZATION → 原检查阶段
失败：ABORTED → 请求取消活动 Action 与 SAFE_HOLD
```

普通点与返航默认执行 transit、精确 approach、连续停稳和位姿验收。障碍的 `transit_pose` 是巡航目标，`approach_pose` 是技能入口，`end_pose` 仅用于越障后的独立验收；不会把出口作为越障导航目标，也不会将出口伪装成重定位初始位姿。

Skill 必须返回 Action `SUCCEEDED`，且 `completed=true`、`control_released=true`、`safe_stance=true`、`error_code` 为空。随后仍须独立查询运动状态、确认 SAFE_HOLD、获得本阶段的新定位证据并验证出口。恢复 NAVIGATION 的 RPC 和心跳确认完成后，才完成当前障碍并继续。执行中 `safe_stance=false` 不等同于底层故障；静止交接和技能结束时的安全站姿必须真实确认。

失败或未知终态会中止后续导航。Action 的取消接受响应不等于终态；本地 Future 取消不等于服务端撤销。超时使用单调时钟，暂停 ROS 时间不会无限延长任务预算。`FINISHED` 表示配置路线与返航流程完成，不代表裁判得分；状态中的 `finished=true` 也包含 `ABORTED`，须同时检查 `state`。

## ROS 接口

接口定义见 [go2_obstacle_interfaces](../go2_obstacle_interfaces)。Action/消息类型变化后，所有生产者与消费者须一起重编译。

| 默认名称 | 类型 | 用途 |
| --- | --- | --- |
| `/mission/start` | `std_srvs/srv/Trigger` | 只允许启动一次；配置预检查时必须有新鲜的成功结果 |
| `/mission/status` | `std_msgs/msg/String`，JSON | FSM、当前任务、token、完成列表、误差与失败原因 |
| `/mission/startup_status` | `std_msgs/msg/String`，JSON | 预检查是否配置、就绪、超时及原因 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 导航终态、取消结果及 BT 选择 |
| `/execute_obstacle` | `ExecuteObstacle` | 通过 `obstacle_id/skill_id` 调用技能 |
| `/motion/set_control_mode` | `SetControlMode` | `expected_owner`、Mission 会话、请求 ID 与租约交接 |
| `/motion/control_state` | `ControlState` | Mission 使用的控制所有权与健康心跳 |
| `/motion/get_state` | `GetMotionState` | 独立查询技能活动、安全站姿、可释放及故障 |
| `/mission/localization_ok` | `std_srvs/srv/Trigger` | 定位质量检查；仅有 TF 不足以成功 |
| `/localization/relocalize` | `Relocalize` | 仅 `enable_relocalization:=true` 时创建客户端；本分支默认关闭，健康检查失败转 SAFE_HOLD 后终止 |
| `/odom` | `nav_msgs/msg/Odometry` | 有效时间戳的运动测量与连续停稳检查 |
| `map → base_footprint` | TF2 | 独立测得的全局平面位姿 |

新后端另提供 `/motion/authority` 的 `MotionAuthority`，包含 `mission_id/generation`；仲裁输入 `MotionCommand` 必须匹配该会话和代次。真实底层反馈、最终命令路径及故障保护必须共同接入，不能只实现交接服务。机载规划诊断的原始速度保留在 `/onboard/nav_cmd_raw`，独立 `stm32_bringup.launch.py` 中的 NavVelocityAdapter 将其接入受租约保护的 `/motion/nav_command`，默认禁用运动。

## 参数与路线

ROS 参数默认值见 [mission_manager.py](robocon_mission/mission_manager.py)，部署覆盖见 [mission_params.yaml](config/mission_params.yaml)，launch 参数见 [mission.launch.py](launch/mission.launch.py)。长度单位 m，角度 rad，时间 s。路线严格要求 `frame_id: map`、非空任务列表、唯一 ID 和显式 `return.pose`；解析器会拒绝重复键、未知字段、非有限数值和非法时限。

| 参数组 | 当前默认或配置 | 作用 |
| --- | --- | --- |
| `route_file` | 必须提供审核后的路线 | 普通点 `pose`；障碍 `transit_pose/approach_pose/end_pose/skill_id` |
| `transit_bt_file/approach_bt_file` | 默认解析安装后的两棵 BT | 见 [BT 说明](../go2_nav_bringup/NAV2_BT.md) |
| `wait_at_waypoints/precise_waypoints` | `true/true` | 普通点与返航精确接近、停稳、验收 |
| `approach_xy_tolerance/approach_yaw_tolerance` | `0.10/0.10` | 停稳后的障碍入口验收 |
| `waypoint_xy_tolerance/waypoint_yaw_tolerance` | `0.10/0.10` | 普通点和返航验收 |
| `max_waypoint_alignment_retries` | `1` | 仅停稳后位姿超限可有限重新 approach |
| `stable_linear_velocity/stable_angular_velocity/stable_duration` | `0.03/0.05/1.0` | 连续停稳判据 |
| `verify_stable_duration/verify_max_sample_gap/verify_minimum_samples` | `0.5/0.4/3` | 独立出口稳定观察 |
| `accepted_pose_topic/type/body_frame/source_id` | 默认 topic 为空，类型 `covariance` | 实机须显式连接正确的定位证据与安装体坐标 |
| `control_state_timeout/control_lease_duration` | `0.5/2.0` | 控制心跳时效及租约 |
| `startup_check_service` | 默认空字符串，兼容旧调用 | 配置后，手动与自动启动均受预检查门控 |
| `startup_check_max_age/startup_check_timeout/startup_timeout` | `1.0/0.5/20.0` | 启动结果时效、RPC 超时与自动启动等待上限 |
| `mission_time_limit` | `180.0` | 整场单调时间预算；按实际赛规配置 |
| `autostart/simulation_mode` | `false/false` | 当前机载入口未启用 Mission 自动启动 |

障碍可逐项覆盖 `end_xy_tolerance`、`end_yaw_tolerance`、`verify_timeout`、`transit_timeout`、`approach_timeout`、`skill_timeout` 和 `max_retries`。未指定出口验收值时，路线数据类默认 `0.15 m/0.20 rad/5.0 s`。这些是软件配置值，实机对准能力和实际定位误差仍须测量。

## CLI 与调试入口

安装后先加载新工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
ros2 launch robocon_mission mission.launch.py --show-args
```

`mission.launch.py` 只启动真实 MissionManager，不再注册或启动 Mock 节点；`use_sim_time=false`、`simulation_mode=false`、`autostart=false` 为安全默认。路线、原子定位证据、预检查服务与任务时限参数继续保留。它不提供底层状态或运动驱动，当前应使用根 README 的机载定位/规划诊断入口。

已启动的软件测试环境可读取：

```bash
ros2 topic echo /mission/status
ros2 topic echo /mission/startup_status
```

`/mission/start` 的调用形式为 `ros2 service call /mission/start std_srvs/srv/Trigger '{}'`。配置 `startup_check_service` 后，手动调用也不能绕过预检查；空字符串是兼容行为，不应作为实机启动策略。自动启动只在等待上限内重试，失败原因发布至 `/mission/startup_status`。预检查通过后，运行中的定位、停稳、所有权和安全站姿监督继续由既有流程处理。

## Hurricane 障碍适配

提供 Hurricane 八类障碍技能模板、非阻塞 `/execute_obstacle` 服务和现场采点工具。真实执行通过共享 STM32 网关；旧指令编码仅保留用于兼容与测试。使用方法与反馈契约见 [Hurricane 移植说明](HURRICANE_PORT.md)。默认 dry-run，模板需现场标定；本任务包保留现有 FSM、控制交接、独立运动状态、定位与出口验收流程。Hurricane 实机路径与通用 obstacle_adapter 是不同执行后端，不能同时提供 `/execute_obstacle`。

## 验证范围

核心测试位于 [test](test)，后端集成测试位于 [go2_mission_backends/test](../go2_mission_backends/test)。当前证据与命令以[根验证记录](../../docs/VALIDATION.md)为准。模拟成功仅证明软件状态和消息契约，不能证明真实技能、机械支撑、赛场落点或整场实机自动化已完成。任务状态机不负责足端接触判分，规则验证接口见 [rule_validation.py](robocon_mission/rule_validation.py)。

## rely-only 清理说明

旧 Mock 运行注册、无使用方的兼容 launch 与演示配置已移除。集成测试需要的两个 Mock 已迁到 `test/fixtures`，不作为 ROS 可执行程序安装。仲裁、定位、预检查和故障回归仍使用显式软件测试后端；它不在实机启动链中。检查清单与结果见 [清理报告](../../docs/RELY_ONLY_CLEANUP_REPORT.md)。
