# 实机现场操作卡

> STM32 接入更新：自制机器人不支持横移。新增单串口网关默认禁用运动；Hurricane 实机改走共享网关。协议为待固件实现草案，见 [STM32 接入指南](STM32_BRIDGE_GUIDE.md)。下文原有定位/规划诊断入口继续保留。


适用于已经完成安装、实测配置、地图和起点核对的操作者。第一次使用请先按 [实机操作指南](REAL_ROBOT_GUIDE.md) 完成准备。当前 `navigation` 仍是私有输出诊断；真实运动先完成指南第 9 节的生产接入。

## 1. 每个终端加载环境

```bash
source /opt/ros/humble/setup.bash
source "$HOME/rc2026_ws/install/setup.bash"
export PYTHONNOUSERSITE=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$(ros2 pkg prefix go2_mission_backends)/share/go2_mission_backends/config/cyclonedds_sim.xml"
```

如现场另有约定 domain，所有终端一致替换 42。检查网卡、地图、安装状态与当前 IMU 初猜是否符合实际；改变起点后不能沿用旧初猜。

## 2. 终端 A 选择一个主模式

```bash
# 实际全局定位
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=localization
```

需要导航规划诊断时，将同一条命令的 `mode:=localization` 改成 `mode:=navigation`。需要重新建图时使用 `mode:=odometry`。切换前先退出上一主入口，不同时启动多套驱动。

## 3. 终端 B 检查定位

```bash
ros2 service call /mission/localization_ok std_srvs/srv/Trigger '{}'
ros2 service call /onboard/doctor std_srvs/srv/Trigger '{}'
ros2 run tf2_ros tf2_echo map base_footprint
```

定位健康服务应当前成功；doctor 看 `localization_ready` 和 `live_checks` 的实时字段。doctor 顶层 `NOT_READY`、Trigger `success:false` 和运动关闭在本入口中会保留，不表示定位必然失败。`tf2_echo` 用 Ctrl+C 退出。

## 4. 按需打开定位图或采点面板

以下需要可用的图形会话，并已加载第 1 节环境：

```bash
rviz2 -d "$(ros2 pkg prefix go2_real_localization)/share/go2_real_localization/rviz/localization.rviz"
```

```bash
mkdir -p "$HOME/rc2026_field"
ros2 run robocon_mission capture_panel \
  --file "$HOME/rc2026_field/field.yaml"
```

采点前人工移动至实测位置并停稳，确保定位当前有效。26 点采齐后按主指南第 8 节导出与核对；面板不驱动机器人运动。

## 5. 退出

有活动 Nav2 目标时，按 [主指南取消步骤](REAL_ROBOT_GUIDE.md#7-实机地图上的-nav2-规划诊断) 显式取消并确认终态，再在 A 按 Ctrl+C，等待子进程退出。没有活动目标时直接退出 A。以后接通真实底层后，必须先完成底层安全停止，退出 ROS 不等于已停稳。

| 现象 | 去哪里查 |
|---|---|
| 找不到包或多个终端互相看不到数据 | [环境与安装排错](TROUBLESHOOTING.md) |
| 雷达无数据、没有 `/odom` | [传感器与状态排错](TROUBLESHOOTING.md) |
| ICP 拒绝、定位超时或点云错位 | [定位排错](TROUBLESHOOTING.md) |
| 目标已发出但机器不走 | [导航诊断与生产接入边界](REAL_ROBOT_GUIDE.md#9-要让机器马真正自动走和越障还需完成的接线与接口) |
| 无法弹出采点窗口、标签不识别 | [现场采点排错](TROUBLESHOOTING.md) |
