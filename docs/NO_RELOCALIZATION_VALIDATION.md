# 无重定位适配验证记录

日期：2026-10-24。基线：`rely-only`，提交 `66b524695179fa8152083d8fa6179c9df309c660`。
环境：WSL Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10、CycloneDDS、localhost 隔离测试域。
使用已有 ROS 接口与原生依赖 underlay，本次三个 Python 包另建独立 overlay。未连接雷达或运动执行器。

## 结果

| 检查 | 结果 |
|---|---|
| colcon 构建 `robocon_mission`、`go2_mission_backends`、`go2_onboard_bringup` | 3 包通过 |
| 安装后的 `ros2 pkg executables go2_mission_backends` | 包含 `odometry_localization` |
| 安装后的 `no_relocalization.launch.py --show-args` | 通过，默认 navigation |
| 新增与相关回归，完整命令见下文 | 212 通过，6 失败；失败均为原有 `test_robot_state_ros.py` 用例 |
| 未修改基线的 `test_robot_state_ros.py` 对照 | 同样 6 失败、2 通过，失败用例名称完全一致 |
| Mission 定向 ROS 集成 | 7 通过 |
| `git diff --check` | 通过 |

本次新增的里程计变换、故障锁定、配置、互斥启动图、ROS evidence/health/TF 和无重定位 FSM 测试全部通过，无跳过。
新增 ROS 测试实际构造 `OdometryLocalizationNode`、`LocalizationAdapter` 和 `RobotStateAdapter`，验证非零 map/odom 对齐后 `map → base_footprint` 的位置、健康数据来源、断流失效和恢复输入后仍锁定故障。

## 回归范围与未通过项

在 source Humble 和工作空间后，将当前源码包放在 `PYTHONPATH` 前部，运行：

```bash
python3 -m pytest -q \
  real/go2_onboard_bringup/test/test_profile.py \
  real/go2_onboard_bringup/test/test_no_relocalization_launch.py \
  common/robocon_mission/test/test_mission_fsm.py \
  common/robocon_mission/test/test_precise_waypoints.py \
  common/robocon_mission/test/test_waypoint_stop.py \
  common/robocon_mission/test/test_route_manager.py \
  common/go2_mission_backends/test/test_odometry_localization.py \
  common/go2_mission_backends/test/test_odometry_localization_ros.py \
  common/go2_mission_backends/test/test_robot_state.py \
  common/go2_mission_backends/test/test_robot_state_ros.py \
  common/go2_mission_backends/test/test_localization_core.py \
  common/go2_mission_backends/test/test_localization_nodes.py tools/tests
```

6 个失败用例均在 `test_robot_state_ros.py`：

- `test_ros_actual_odom_and_tf_preserve_tilted_global_body`
- `test_ros_frame_timestamp_stale_and_watchdog_refuse_output`
- `test_ros_pose_derivation_ignores_zero_fastlio_twist`
- `test_ros_unknown_measured_twist_refused`
- `test_ros_sim_clock_pause_expires_and_rollback_requires_new_samples`
- `test_ros_global_tf_interruption_never_refreshes_old_map_transform`

这些用例在未修改基线上也失败，日志表现为观测端尚未收到预期 TF/状态、或收到前一状态时已执行断言。本次未修改其生产代码或放宽测试断言；它们仍是当前环境中的未通过项。

定向 Mission 集成命令如下，共 7 个参数化用例通过，覆盖完整任务、越障后定位故障、显式重定位兼容与独立安全状态确认：

```bash
python3 -m pytest -q \
  common/robocon_mission/test/test_mission_integration.py::test_complete_route_with_real_ros_actions \
  common/robocon_mission/test/test_mission_integration.py::test_localization_failure_after_skill_blocks_next_goal \
  common/robocon_mission/test/test_mission_integration.py::test_localization_loss_recovers_without_repeating_skill \
  common/robocon_mission/test/test_mission_integration.py::test_independent_stance_release_and_restore_confirmation
```

一次较大范围的 Mission 集成运行在 15 个用例通过后，`unstable` 用例因控制权确认时序失败；上述定向运行中该用例通过。因此不将一次通过等同于全部时序测试稳定。

更早的完整后端回归尝试出现多项失败后中止，没有完成全量测试。单独对照 `test_backend_mission.py::test_mission_two_distinct_obstacles_and_return[sim_clock]`：新分支与未修改基线都在初始定位健康等待阶段得到 `health_timeout:ACCEPTED`。基线随后尝试不可用的重定位服务；新分支按预期在 SAFE_HOLD 后报告重定位已禁用并终止。该完整后端用例未通过，不计入通过数。

未执行真实雷达数据、实际二维地图定位、STM32 固件联调或整场障碍赛；无重定位的累计漂移与起点重复摆放精度需按 [使用指南](NO_RELOCALIZATION.md) 做现场验收。
