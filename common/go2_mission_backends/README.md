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

默认不收集旧 Gazebo 专用 test_simulation_launch 模块，保留迁移到 test/fixtures 的巡航路线和地图测试。额外 wall_clock 完整闭环案例通过 --include-wall-clock 开启；默认受控时钟路径仍完整执行任务，时钟停止/回退的拒绝路径仍检查。历史开发环境成绩不作为新安装机器的验收结果。

实机配置入口见仓库根 README 与 go2_onboard_bringup。当前实机阶段低层状态与真实技能尚未接入，启动检查保留 NOT_READY；Nav2 规划输出保持在私有诊断话题。backend_simulation 不能作为实机部署入口。

## 节点与接口

| 节点 | 输入 | 输出或服务 |
|---|---|---|
| robot_state_adapter | /Odometry、测量时刻的 map→odom、已测量安装外参 | /odom、独立导航 TF 分支、/robot_state/status |
| localization_adapter | /gicp/evidence | /localization/evidence、Trigger /mission/localization_ok |
| motion_arbiter | 带身份的 nav/skill command、MotionTelemetry、/odom | authority、control_state、运动状态查询和紧急门控；隔离测试最终速度 |
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
- config/arbiter.yaml：隔离测试输出、速度限制、命令/状态超时和停稳标准。该文件是验证配置，参数未标定实机。
- config/test_skills.yaml、backend_test_route.yaml、simulation.yaml：软件验证的技能、两个入口/出口分离的障碍和返航路线、受控时钟 Mission 配置。
- config/preflight.yaml：状态、接口、路线、唯一输出发布者和可选 Nav2 lifecycle 检查。实机规划阶段采用 onboarding 配置。

保留原 map→odom→livox_frame 分支，robot_state_adapter 新增导航分支，避免同一 IMU 有第二个父节点。未知协方差必须显式表达未知，不能解释为零误差。ground_z 是导航平面，实际安装外参、导航轮廓和高度需测量。

## 定位与越障接入

GICP 原子发布一次匹配的测量时间、source、epoch、sequence、accepted 和 pose。adapter 不通过 Bool 与独立 Pose 拼接证据。拒绝、乱序、过期或时钟故障清除 accepted；恢复和阶段推进必须等待新的测量。详见 docs/LOCALIZATION_EVIDENCE.md。

新 Mission 链路设置 accepted_pose_type=evidence、accepted_pose_topic=/localization/evidence、accepted_pose_body_frame=livox_frame、startup_check_service=/mission/preflight。GICP 重启产生新 source，需要停止任务并显式重启对应 adapter/Mission；同一健康服务不能有多个不一致后端。

越障团队实现 SkillExecutor 的 start/tick/cancel/close，并以 executor_factories 注册真实后端。所有命令仍经 MotionCommand 仲裁，cancel/close 确认需要真实执行器与独立底层状态。只有明确 simulation_mode=true 才允许测试 executor；硬件模式未配置后端时返回 NOT_CONFIGURED。Action 成功仍需独立安全释放和新出口定位，未确认取消、未知支撑和释放失败均禁止恢复导航。详见 docs/SKILL_EXECUTOR_ADAPTER.md。

固定地图初猜沿用真实 GICP，尚未提供全局候选搜索式重定位。实际 Nav2 Twist 到带 generation/session 的控制输入、真实 MotionTelemetry 和技能 executor 是后续实机接入工作；当前软件验证不代表真实运动或定位精度验收。


`rely-only` 保留 `simulation_backend` 与 `backend_simulation.launch.py`，供独立的软件安全回归使用；它们不被机载 real.launch 或正式 mission.launch 包含。测试后端的 `simulation_mode` 防护和 `/backend_sim/cmd_vel` 隔离输出保持不变，不能用测试状态替代 STM32 真实反馈。
