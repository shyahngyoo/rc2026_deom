# Mission 后端

本包为 robocon_mission 提供机器人状态、原子定位证据、控制交接和越障 Action 后端。MissionFSM 负责既有任务流程。

软件验证覆盖：导航 → 精确接近 → 入口停稳与位姿验收 → NAVIGATION/SAFE_HOLD/SKILL 交接 → 技能执行与独立安全释放确认 → 新定位证据和出口验收 → 下一障碍 → 返航。测试机器人根据仲裁器实际最终速度积分运动，Action 不设置出口位姿。

## 安装与验证

环境为 Ubuntu 22.04、ROS 2 Humble。先按仓库根 README 安装依赖和工作空间，再运行：

~~~bash
cd ~/rc2026_ws
bash src/rc2026_deom/tools/verify_real.bash
~~~

工作空间或源码目录不同时，可用 --workspace 指定安装工作空间。工具重新构建本仓库测试运行包并检查 ament 安装来源，拒绝引用其他工作空间。日志、JUnit 与闭环记录写入该工作空间的 log/verification/时间目录。

验证使用独立 localhost ROS domain、CycloneDDS 分片 profile 和受控 /clock。完整 backend_simulation 是软件验证入口：运行模拟 NavigateToPose、运动积分底层和模拟证据流。真实 GICP 另用完整1800点合成点云做独立 C++ 跨进程验证。工具不启动真实雷达、SDK 或电机。

旧 Gazebo 运行入口已清理，相关测试已调整为适用的启动契约与回归检查；保留 test/fixtures 中的巡航路线和地图，当前验证脚本不再忽略该历史 launch 测试模块。额外 wall_clock 完整闭环案例通过 --include-wall-clock 开启；默认受控时钟路径仍完整执行任务，时钟停止/回退的拒绝路径仍检查。历史开发环境成绩不作为新安装机器的验收结果。

实机配置入口见仓库根 README 与 go2_onboard_bringup。`real.launch.py` 保留 NOT_READY 诊断边界；独立 STM32 桥接入口已提供导航命令和真实 STATUS 适配，默认禁用运动。固件端状态与技能仍待实现和联调，见 [STM32 指南](../../docs/STM32_BRIDGE_GUIDE.md)。backend_simulation 不能作为实机部署入口。

## 节点与接口

| 节点 | 输入 | 输出或服务 |
|---|---|---|
| robot_state_adapter | /Odometry、测量时刻的 map→odom、已测量安装外参 | /odom、独立导航 TF 分支、/robot_state/status |
| localization_adapter | /gicp/evidence | /localization/evidence、Trigger /mission/localization_ok |
| motion_arbiter | 带身份的 nav/skill command、MotionTelemetry、/odom | authority、control_state、运动状态查询和紧急门控；按配置输出测试 Twist 或实机带授权 MotionCommand |
| obstacle_adapter | ExecuteObstacle、技能配置、authority、独立底层状态 | Action feedback/result、/skills/ready、带租约身份的技能输入 |
| preflight | 定位、TF、状态、控制权、导航和技能接口、路线 | Trigger /mission/preflight |
| simulation_backend | 隔离测试最终 Twist、技能事件、控制权 | 测试运动积分、IMU odom/TF、底层反馈、证据流、NavigateToPose、/clock |

保留 SetControlMode、GetMotionState、ControlState 和 ExecuteObstacle 协议。新增 LocalizationEvidence、MotionCommand、MotionAuthority、MotionTelemetry。GetMotionState 请求包含 obstacle_id，用于核对当前障碍。

MotionCommand 包含 owner、mission_id、generation 和时间戳。发送者必须使用当前授权身份；控制切换清空缓存、建立新时间边界，旧命令不能在再次获得同名 owner 后生效。租约、命令、里程计或底层状态超时，仲裁输出归零。

唯一发布者检查能发现旁路并停止本节点授权输出，但不能阻止其他 DDS 发布者。实机接线和启动配置必须保证最终速度发布链路唯一。

## 配置与参考系

- config/cyclonedds_sim.xml：软件验证进程的 UDP 消息1200 B、数据分片1100 B。保持完整 ROS 点云，不修改用户全局 DDS 或内核。
- config/robot_state.yaml：外参、速度坐标系、状态年龄和协方差。默认 identity 与模拟值均是示例；实机通过已测量的外部 profile 提供。
- config/localization.yaml：source pinning、ROS/单调时间年龄、未来时间和时钟停止检查。
- config/arbiter_real.yaml：STM32 实机仲裁配置，默认关闭输出，最终带授权命令为 `/stm32/arbiter_command`。
- config/arbiter.yaml：隔离测试输出、速度限制、命令/状态超时和停稳标准。该文件是验证配置，参数未标定实机。
- config/test_skills.yaml、backend_test_route.yaml、simulation.yaml：软件验证的技能、两个入口/出口分离的障碍和返航路线、受控时钟 Mission 配置。
- config/preflight.yaml：状态、接口、路线、唯一输出发布者和可选 Nav2 lifecycle 检查。实机规划阶段采用 onboarding 配置。

保留原 map→odom→livox_frame 分支，robot_state_adapter 新增导航分支，避免同一 IMU 有第二个父节点。未知协方差必须显式表达未知，不能解释为零误差。ground_z 是导航平面，实际安装外参、导航轮廓和高度需测量。

## 定位与越障接入

GICP 原子发布一次匹配的测量时间、source、epoch、sequence、accepted 和 pose。adapter 不通过 Bool 与独立 Pose 拼接证据。拒绝、乱序、过期或时钟故障清除 accepted；恢复和阶段推进必须等待新的测量。详见 docs/LOCALIZATION_EVIDENCE.md。

新 Mission 链路设置 accepted_pose_type=evidence、accepted_pose_topic=/localization/evidence、accepted_pose_body_frame=livox_frame、startup_check_service 指向实际生产预检查服务；STM32 指南使用 `/stm32/preflight`，原机载 `/mission/preflight` 保留诊断禁止运动边界。GICP 重启产生新 source，需要停止任务并显式重启对应 adapter/Mission；同一健康服务不能有多个不一致后端。

通用 obstacle_adapter 的扩展方式是实现 SkillExecutor 的 start/tick/cancel/close，并以 executor_factories 注册后端。当前 STM32/Hurricane 路径使用独立 hurricane_skill_server 和共享网关，不要求再实现或启动第二套 Action 后端。该通用 SPI 的运动命令仍经 MotionCommand 仲裁，cancel/close 确认需要真实执行器与独立底层状态。只有明确 simulation_mode=true 才允许测试 executor；硬件模式未配置后端时返回 NOT_CONFIGURED。Action 成功仍需独立安全释放和新出口定位，未确认取消、未知支撑和释放失败均禁止恢复导航。详见 docs/SKILL_EXECUTOR_ADAPTER.md。

固定地图初猜沿用真实 GICP，尚未提供全局候选搜索式重定位。主机已有 NavVelocityAdapter、TelemetryAdapter 与 Hurricane 网关运输层；尚需 STM32 提供真实协议执行与安全状态，并确认取消/完成关联。当前软件验证不代表真实运动或定位精度验收；最新测试通过项和未通过项见 [STM32 验证报告](../../docs/STM32_BRIDGE_VALIDATION.md)。


`rely-only` 保留 `simulation_backend` 与 `backend_simulation.launch.py`，供独立的软件安全回归使用；它们不被机载 real.launch 或正式 mission.launch 包含。测试后端的 `simulation_mode` 防护和 `/backend_sim/cmd_vel` 隔离输出保持不变，不能用测试状态替代 STM32 真实反馈。

## 节点重启后的编号

`motion_arbiter` 的授权 `generation` 与 `stm32_telemetry_adapter` 的输出 `sequence`
使用持久化编号区间。每次启动在 SQLite 事务中预留新的 `2^32` 个编号，之后在内存中递增，
不会每次发布都写磁盘。新实例的编号高于旧实例整个区间；旧实例不能再次分配区间。
仲裁器重启仍从 `SAFE_HOLD` 开始，不恢复旧租约或命令，需要重新申请授权。

默认状态文件为 `$XDG_STATE_HOME/rc2026/runtime_counters.sqlite3`；未设置该环境变量时，
使用 `~/.local/state/rc2026/runtime_counters.sqlite3`。两个节点均支持 ROS 参数
`counter_state_path` 指定文件。重启前后必须使用相同的持久化路径，不能将它放在每次清空的临时目录。
数据库损坏、不可写或编号耗尽会使节点启动失败，不会悄悄从低编号重新开始。

ROS 消息格式不变；`generation` 和遥测 `sequence` 原本就是 `uint64`，现在可超过 32 位。
这里的遥测序号不等于 UART 帧头的 32 位 `sequence`。更新时需同时重新构建
`go2_mission_backends` 和 `quadruped_stm32_bridge`。状态文件丢失或更换运行用户时，
应先停止整个控制链并重新初始化接收端；本机制不承诺跨机器、跨状态文件恢复旧租约。
