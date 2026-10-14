# 机载入口：真实输入、关闭运动输出

此包适用于 Ubuntu 22.04 / ROS 2 Humble。安装与建图先使用 `sensor_only` 配置；不会启动 Gazebo、模拟底层、mock 技能或 Mission。当前缺少真实底层安全反馈和越障执行器，因此 `navigation` 仅用于规划诊断，不能让机器马运动。

所有安装外参必须测量后显式填写，`--imu-in-base` 是 `T_base_link_IMU` 的 XYZ（米）及 RPY（弧度）。`--initial-pose` 是 **IMU 在 map 中**的 XYZ/RPY，不是导航参考点的位置。下面数值只表示命令语法，不能直接当作实机标定值。

```bash
ros2 run go2_onboard_bringup configure_robot \
  --config-dir "$HOME/.config/rc2026" --stage sensor_only \
  --host-ip <接收机IPv4> --lidar-ip <MID360_IPv4> \
  --imu-in-base <x> <y> <z> <roll> <pitch> <yaw>
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=lidar
```

`mode:=odometry` 在驱动后启动 FAST-LIO 和机体坐标适配；无需地图，不认为局部里程计已经完成全局定位。准备实际 PCD 与固定起点后生成定位配置：

```bash
ros2 run go2_onboard_bringup configure_robot --force \
  --config-dir "$HOME/.config/rc2026" --stage localization \
  --host-ip <接收机IPv4> --lidar-ip <MID360_IPv4> \
  --imu-in-base <x> <y> <z> <roll> <pitch> <yaw> \
  --map-pcd /absolute/real_room.pcd \
  --initial-pose <map_x> <map_y> <map_z> <map_roll> <map_pitch> <map_yaw>
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=localization
ros2 run go2_onboard_bringup doctor --config-dir "$HOME/.config/rc2026" --mode localization
ros2 service call /onboard/doctor std_srvs/srv/Trigger '{}'
```

外置目录保存 profile、雷达网络、FAST-LIO、GICP、坐标适配、原子定位证据及 Nav2 参数。配置错误、地图缺失、坐标不一致会在启动真实节点之前拒绝。`--force` 只替换指定生成文件；不会删除其他文件。生成后如果修改网络或外参，必须同步 profile 与相关参数。

`navigation` 还必须显式提供 `--nav-map /absolute/map.yaml` 及 `--footprint '<测量的二维JSON多边形>'`。初始 Nav2 profile 使用静态地图 costmap；扫描投影 `/cloud_registered_body → /onboard/scan_diagnostic` 的高度范围只是诊断参数，未校准到真实机器马，不作为碰撞安全证明。所有控制/行为速度使用 `/onboard/nav_cmd_internal`，平滑后的最终速度使用 `/onboard/nav_cmd_raw`；不存在通往机器人 `/cmd_vel` 的连接，也没有打开电机输出的开关。

常驻 doctor 与真正的 Mission preflight 均会报告未接底层状态、未接技能等条件。doctor 的 `mission_allowed` 始终为 false；收到模拟遥测或外部成功响应也不能解锁。真实 footprint、运动限制、底层状态、安全支撑、急停和越障技能仍需独立实机集成。

测试可执行 `python3 -m pytest real/go2_onboard_bringup/test`。实际 Nav2 topic 隔离检查在 `test_planning_graph.py`，只启动诊断 Nav2 与测试 TF，绝不运行真实雷达驱动或电机。
