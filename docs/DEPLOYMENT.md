# 机载部署与建图

首次实机操作请先按[实机操作指南](REAL_ROBOT_GUIDE.md)逐步执行；日常使用查[现场操作卡](REAL_ROBOT_QUICK_REFERENCE.md)，错误按[实机排错表](TROUBLESHOOTING.md)处理。本文保留参数与部署补充。

环境是原生 Ubuntu 22.04 / ROS 2 Humble。默认 workspace 为 `~/rc2026_ws`，源码为 `~/rc2026_deom`；可用安装器参数改变前者。安装脚本在子进程中清除继承 overlay，不依赖原工程。部署终端设置 `export PYTHONNOUSERSITE=1`，避免 `~/.local` 的旧 pip 包覆盖 Ubuntu / ROS 使用的 NumPy、SciPy 等系统包；它不删除用户库。

## 先检查网络、时钟和数据

- `ip -br addr` 确认接收 IP 已配置到真实网卡；设置的 `--host-ip` 不会自动替操作系统配置网卡。
- 使用单一 MID-360 驱动实例。FAST-LIO 需要 `xfer_format=1` 的 CustomMsg 和同雷达 IMU。
- 所有实时节点使用 `use_sim_time=false`。用 `timedatectl status` 检查机载时钟；运行中时钟回退会使证据失效。
- 控制台中的有效 odometry、accepted evidence、TF 与 doctor 需要共同检查，单个 topic 有数据不代表可以执行任务。

如果 clone 失败，恢复设备对 GitHub 的访问后重跑；安装器不切换非官方镜像，也不从其他工程复用未知版本。已有依赖被修改时先保留并处理，安装器不会丢弃这些修改。

## 采集地图

用根 README 的外参/网络向导生成 `sensor_only` 配置，启动 `mode:=odometry`，在人工控制下完成实际场地扫描。项目不会自动驱动机器马建图。记录启动参考位姿，避免扫描时剧烈姿态变化、雷达遮挡及缺乏静态几何的区域。

另开 source 过的终端：

```bash
ros2 run go2_fastlio_localization save_live_map   --ros-args -p output_dir:="$HOME/rc2026_maps" -p timeout_sec:=45.0
```

该命令等待 `/Laser_map`，保存已经累计的注册扫描为实际 PCD，并输出完整路径、点数及 frame。停止 odometry 后再用此 PCD 定位。不是闭环优化建图工具；大场地需检查累计漂移与重叠墙面。

二维图必须与 PCD 使用相同地图原点、坐标朝向及比例。已有投影工具可生成初稿：

```bash
read -r -p 'PCD 绝对路径: ' REAL_PCD
read -r -p '障碍切片 z_min（PCD坐标，米）: ' MAP_Z_MIN
read -r -p '障碍切片 z_max（PCD坐标，米）: ' MAP_Z_MAX
ros2 run go2_fastlio_localization pcd_to_nav2_map   "$REAL_PCD" "$HOME/rc2026_maps/field"   --resolution 0.05 --z-min "$MAP_Z_MIN" --z-max "$MAP_Z_MAX"
```

此工具只按高度投影点云；未观测区域也可能被输出为 free，不能直接作为比赛可行驶区域证明。必须检查并编辑边界、未知区域、足式可通行障碍、狭窄通道与机体包络，再用实际点位核对坐标。测试 fixture 地图只能回归，不能用于实机。

## 定位和规划诊断

根 README 的 `--initial-pose` 是 IMU 在 PCD map 中的固定起点。重启 LIO 后原点会重建，不能沿用与新位置不符的 initial pose。生成 `stage=localization` 配置后运行 localization，检查 `/gicp/evidence` 和 `/localization/evidence` 中的新 accepted 时间/epoch。

`navigation` 必须提供实际二维图和 footprint。可在 RViz 用 `map` 固定坐标显示地图、足点 TF 和规划路径。默认不启动 RViz，也不启动电机；在 RViz 发送目标仍只产生私有诊断速度。

```bash
ros2 run go2_onboard_bringup doctor --config-dir "$HOME/.config/rc2026" --mode navigation
ros2 service call /onboard/doctor std_srvs/srv/Trigger '{}'
ros2 topic info /onboard/nav_cmd_raw --verbose
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo map base_footprint
```

静态 doctor 退出码 0 表示配置可解析，不能用于自动开启 Mission。`mission_allowed` 和 `motion_enabled` 在本入口中均为 false。当前缺失低层安全反馈与技能服务属于预期 NOT_READY。

## DDS 与离线回归

实机多机通信需统一 `ROS_DOMAIN_ID` 与 RMW，允许网卡间发现；不要复制软件回归的 `ROS_LOCALHOST_ONLY=1` 到需要远程 RViz 的环境。验证工具为自身进程选择 CycloneDDS 和分片配置，不改系统防火墙。

软件回归可用：

```bash
bash tools/verify_real.bash --workspace "$HOME/rc2026_ws" --report-dir "$HOME/rc2026_reports"
```

测试结果不会证明真实雷达收包、实际姿态/制动/支撑或比赛定位精度。CPU/内存不够时安装用 `--jobs 1`，运行 GICP 再按资源调整 `num_threads`；不要通过放宽质量阈值掩盖错误外参或时钟问题。

## 长期运行

先用终端分阶段跑通、检查日志与真实数据，完成实际控制接口验收后再做无人值守服务。当前不安装自动开机行走服务。运行时配置和地图独立于 git；更新源码前备份 `~/.config/rc2026` 和实际地图，再重跑安装/回归。
