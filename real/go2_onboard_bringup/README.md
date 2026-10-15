# 机载入口与配置接口

首次实机部署按[实机操作指南](../../docs/REAL_ROBOT_GUIDE.md)执行；已经配置完成时查[现场操作卡](../../docs/REAL_ROBOT_QUICK_REFERENCE.md)，失败时查[实机排错表](../../docs/TROUBLESHOOTING.md)。

此包组合真实传感器、FAST-LIO、GICP、机体状态适配和诊断。实机 `use_sim_time=false`，不自动启动 RViz、Mission、电机输出或 Hurricane 技能服务器。

## 配置命令

```bash
ros2 run go2_onboard_bringup configure_robot --help
ros2 run go2_onboard_bringup doctor --help
ros2 launch go2_onboard_bringup real.launch.py --show-args
```

| 参数 | 含义 |
|---|---|
| `--config-dir` | 外置配置目录，默认 `~/.config/rc2026` |
| `--host-ip`、`--lidar-ip` | 实际接收网卡与雷达 IPv4；不会替操作系统设置网卡 |
| `--imu-in-base` | 显式六个实测 XYZ/RPY，IMU 在 base_link 中的安装位姿，m/rad |
| `--stage sensor_only` | 首次采集/建图，不要求地图 |
| `--stage localization` | 实际 PCD 与当前 IMU 在地图中的初始位姿 |
| `--map-pcd`、`--initial-pose` | 实际地图绝对路径和六维 map IMU 初猜；不是地面投影坐标 |
| `--nav-map`、`--footprint` | navigation 额外需要实际二维图和实测 JSON 多边形 |
| `--force` | 重新生成指定文件；不是保留旧调参值的合并更新，先备份 |

`stage` 没有 navigation 选项。完整网络/地图/外参输入步骤见主指南第 3、6、7 节。

## 启动模式与验收

`real.launch.py` 的参数为 `config_dir`、`mode`；默认 mode 为 localization，第一次无地图必须显式选择 lidar/odometry。

| mode | 启动范围 | 观察 |
|---|---|---|
| lidar | 真实 MID-360、doctor | `/livox/lidar` CustomMsg、`/livox/imu` |
| odometry | 加 FAST-LIO、robot_state_adapter | `/Odometry`、`/odom`、注册点云；无全局定位属正常 |
| localization | 加真实 PCD GICP、定位适配、preflight | 健康定位、新证据、`map→base_footprint` |
| navigation | 加二维图、Nav2、扫描诊断 | lifecycle ACTIVE，速度保留在 `/onboard/nav_cmd_*` |

`navigation` 当前使用静态地图 costmap。`/cloud_registered_body → /onboard/scan_diagnostic` 只是诊断投影；高度范围和动态障碍能力未做实机标定。所有控制/行为速度使用 `/onboard/nav_cmd_internal`，平滑输出为 `/onboard/nav_cmd_raw`，没有连接机器人 `/cmd_vel`。

静态 doctor `configuration_valid=true` 和退出码 0 只代表配置有效；常驻 `/onboard/doctor` 或 `/onboard/doctor_status` 的 `live_checks`、`localization_ready` 才反映当前数据。顶层 `NOT_READY`、`mission_allowed=false`、`motion_enabled=false` 和 Trigger `success=false` 在本入口中保留，不会因测试遥测或外部成功响应解锁。

生产控制、底层安全反馈和 Hurricane 独立扩展接入见[控制集成说明](../../docs/CONTROL_INTEGRATION.md)与[Hurricane 文档](../../common/robocon_mission/HURRICANE_PORT.md)。不得把诊断速度直接重映射到电机绕过仲裁。
