# Nav2 行为树与精确接近

本包包含 ROS 2 Humble 的两棵 BT 和 DWB 精确接近插件。机载部署、测量配置与验证范围见[根 README](../../README.md)和[验证记录](../../docs/VALIDATION.md)。当前机载 `navigation` 模式用于隔离的规划诊断，不启动实机 Mission，也不连接机器人速度控制口。

## BT 与控制器关系

`NavigateToPose.Goal.behavior_tree` 可指定安装后的 BT XML 绝对路径。Mission 的 `NavigationClient` 分别选用以下文件。通用 [nav2.launch.py](launch/nav2.launch.py) 和新机载规划入口均通过 `RewrittenYaml`，在启动时将安装后的 `nav_transit.xml` 绝对路径写入 `bt_navigator.ros__parameters.default_nav_to_pose_bt_xml`。模板及外部 profile 中该字段保留为空，实际加载的参数由 launch 运行时覆盖；Goal 未指定 BT 时使用项目巡航树。approach 目标由调用者提供，BT 不生成障碍入口或出口。

| 阶段 | BT | Planner | Controller | Goal Checker |
| --- | --- | --- | --- | --- |
| 巡航 | [nav_transit.xml](behavior_trees/nav_transit.xml) | `GridBased` | `FollowPath` | `transit_goal_checker` |
| 精确接近 | [nav_approach.xml](behavior_trees/nav_approach.xml) | `GridBasedApproach` | `FollowPathApproach` | `approach_goal_checker` |

`nav_transit.xml` 使用 `PipelineSequence`。远离目标时由 `RateController` 以 1 Hz 重规划；进入 0.25 m 范围后，`PauseReplanningNearGoal` 锁存暂停重规划，避免持续更新路径打断最后调向。即使起点已接近目标，也须先完成一次规划。新目标或父树 halt 解除锁存。实现见 [pause_replanning.cpp](src/pause_replanning.cpp)。

`nav_approach.xml` 使用 `Sequence`，先规划，再跟随路径；控制运行时不周期重规划。它使用精确入口 Planner，入口不可达时不主动将目标改为邻近点。

| 恢复层 | 巡航 | 精确接近 |
| --- | --- | --- |
| 规划局部恢复 | 清 global costmap，最多重试 1 次 | 清 global costmap，最多重试 1 次 |
| 控制局部恢复 | 清 local costmap，最多重试 1 次 | 清 local costmap，最多重试 1 次 |
| 系统恢复 | 最多 2 次；清 local/global costmap、等待 1 s、重试导航 | 无 |

次数对应各层 `RecoveryNode.number_of_retries`，不是整段导航总调用次数。Mission 的有限重试另行计数。两棵 XML 均不调用 `Spin`、`BackUp` 或 `DriveOnHeading`；Nav2 模板中加载这些插件不代表 BT 会执行它们。

## 精确接近与验收

新机载参数模板见 [nav2.template.yaml](../../real/go2_onboard_bringup/config/nav2.template.yaml)。它包含以下软件初值，尚未作为最终机器马的运动参数验收：

| 项目 | 巡航 | 精确接近 |
| --- | --- | --- |
| Goal Checker XY/yaw | `0.25 m/0.26 rad` | `0.08 m/0.08 rad` |
| Goal Checker | `SimpleGoalChecker`，`stateful=true` | `StoppedGoalChecker`，`stateful=false` |
| 速度停止判据 | 不独立检查 | 平面线速度 ≤`0.03 m/s`、yaw 角速度 ≤`0.05 rad/s` |
| NavFn 邻近端点容差 | `0.05 m` | `0.0 m` |
| 平移/yaw 速度上限 | `0.25 m/s/0.5 rad/s`，不生成横移 | `0.12 m/s/0.35 rad/s`，允许机体 X/Y 双向调整 |

接近 Goal Checker 的 `stateful=false` 持续检查 XY，避免先到 XY 后调向漂移仍被判为成功。它检查当前速度，不能替代连续停稳窗口。Mission 默认在精确接近后连续停稳，再按 `0.10 m/0.10 rad` 验收普通点、返航和障碍入口。Nav2 的 `0.08` 与 Mission 的 `0.10` 留出停车漂移余量；出口使用路线独立配置，见 [Mission 说明](../robocon_mission/README.md)。估计 TF 的合格不能替代独立的实物对准测量。

`FollowPathApproach` 使用 [PrecisionPoseCritic](src/precision_pose_critic.cpp)，连续计算机体 X/Y 和 yaw 的目标速度，同时纠正位置和朝向。距离超过 `goal_direct_radius=0.30 m` 时，使用路径前视 `path_lookahead=0.20 m`；靠近终点时直接使用精确终点。DWB 的 `BaseObstacle` 保留候选轨迹碰撞评分。该 controller 不使用巡航的 `RotateToGoal` 切换窗口。

共享 `SimpleProgressChecker` 初值为 `required_movement_radius=0.05 m`、`movement_time_allowance=30 s`，只统计平移。较长时间原地调向仍可能触发进度超时，应记录实际转向收敛与停稳漂移后再调参。

## 新机载 profile 的参数地址

模板只用于生成配置，不直接填充实机安装位置和机器人外形。使用根 README 中的 `configure_robot`，提供测量的 MID-360 IP、`T_base_IMU`、地图、初始位姿及 `base_footprint` 下 footprint。

| 文件 | 用途 |
| --- | --- |
| `~/.config/rc2026/profile.yaml` | 外部 profile 索引和运动禁用声明 |
| `~/.config/rc2026/nav2.yaml` | 由模板生成的 Nav2 参数与实测 footprint；默认 BT 路径由 launch 运行时覆盖 |
| `~/.config/rc2026/robot_state.yaml` | 独立 `nav_odom` 分支、IMU 安装外参及 `/odom` 适配 |
| `~/.config/rc2026/scan.yaml` | 点云转扫描的诊断参数 |

`config_dir` 可改为另一绝对目录。模板来源为 [go2_onboard_bringup/config](../../real/go2_onboard_bringup/config)，字段验证与生成见 [profile.py](../../real/go2_onboard_bringup/go2_onboard_bringup/profile.py)。全局 costmap 使用 `map`，局部 costmap 使用 `nav_odom`，机器人平面坐标为 `base_footprint`；身体姿态由独立 `base_link` TF 保留。

当前机载诊断只启用 static/inflation costmap，动态点云转 scan 输出到 `/onboard/scan_diagnostic`，未宣称具备实机动态避障安全能力。Nav2 控制与行为节点输出先到 `/onboard/nav_cmd_internal`，velocity smoother 输出到 `/onboard/nav_cmd_raw`。该输出不进入机器人 `/cmd_vel`；真实 `Twist → MotionCommand` 的租约与代次绑定入口仍待实现。组合和 remap 见 [launching.py](../../real/go2_onboard_bringup/go2_onboard_bringup/launching.py)。

## 客户端和测试入口

[nav_action_probe.py](scripts/nav_action_probe.py) 可依次发送 `transit` 和 `approach` 目标，等待 Action 终态，并在 `--check-accuracy` 下读取最终 TF 与 `/odom`。超时后请求取消；新的不同 BT 目标应在上一目标终态确认后发送。它会生成导航速度，当前只应用于隔离的诊断输出图。

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
ros2 run go2_nav_bringup nav_action_probe --help
```

[test_navigate_to_pose_client.py](test/test_navigate_to_pose_client.py) 验证客户端 BT 选择、顺序、失败和取消门控，测试进程内 Action server 不加载真实 BT，也不验证运动收敛。实际 Nav2 图和发布边界的验证以[根验证记录](../../docs/VALIDATION.md)为准。

障碍闭环由 [robocon_mission](../robocon_mission/README.md) 负责。真实底层状态、技能、命令仲裁入口及安全支撑接入之前，BT/Action 成功不能证明实机可以自动跑完整场比赛。
