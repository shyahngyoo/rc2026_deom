# RC2026 机器马机载导航与 Mission 后端

部署目标：**Ubuntu 22.04 + ROS 2 Humble、Livox MID-360（使用自带 IMU）、支持 ROS 2 `/cmd_vel` 和横移的机器马**。CPU 架构尚未由设备确认；安装器支持 `amd64` / `arm64`，本次验证环境是 `amd64`。

这是从原开发工程整理出的实机发布仓库。包含实际雷达驱动、FAST-LIO、GICP 全局定位、机体状态适配、定位证据、Nav2 和 Mission 后端。没有 Gazebo 模型和仿真雷达依赖。

**当前可以直接部署采集、建图、定位和 Nav2 规划诊断。当前不能直接驱动机器马跑完障碍赛。** 底层还没有提供停稳、安全支撑及技能结束状态，真实越障执行器也未接入；机载入口因此不启动 Mission、不启动电机输出、不使用模拟成功状态。`navigation` 的速度只发布到 `/onboard/nav_cmd_*`，没有连接机器人 `/cmd_vel`。

## 1. 安装

在机载 Ubuntu 上操作，使用普通用户。需能访问 GitHub 和 Ubuntu / ROS 软件源。

```bash
sudo apt-get update
sudo apt-get install -y git
cd "$HOME"
git clone https://github.com/shyahngyoo/rc2026_deom.git
cd rc2026_deom
bash tools/install_real.bash
source /opt/ros/humble/setup.bash
source "$HOME/rc2026_ws/install/setup.bash"
export PYTHONNOUSERSITE=1
```

如果尚未安装 Humble，改用 `bash tools/install_real.bash --install-ros`。内存较少时加 `--jobs 1`；默认并发为 2，包按顺序构建。用 `--workspace /absolute/your_ws` 指定工作空间后，相应修改 `source` 路径。

安装器安装必要系统依赖、获取四个固定版本源码、校验并应用补丁、将 Livox SDK 安装到工作空间私有前缀，再构建实机包。固定版本见 [dependencies.repos](dependencies.repos) 与 [dependencies.lock.yaml](dependencies.lock.yaml)。未知修改或不同版本会明确报错，已有源码不会被 reset 或删除。`--dry-run` 可先查看命令。

## 2. 配置雷达及外参

先确认网卡确实拥有接收 IP：`ip -br addr`。接收机和雷达应在可互通的网络中；不要照抄示例 IP 或旧 WSL 地址。

```bash
read -r -p '机载网卡接收 IPv4: ' MID360_HOST_IP
read -r -p 'MID-360 IPv4: ' MID360_LIDAR_IP
read -r -a IMU_IN_BASE -p '测量的 T_base_link_IMU，x y z roll pitch yaw（米/弧度）: '
ros2 run go2_onboard_bringup configure_robot   --config-dir "$HOME/.config/rc2026" --stage sensor_only   --host-ip "$MID360_HOST_IP" --lidar-ip "$MID360_LIDAR_IP"   --imu-in-base "${IMU_IN_BASE[@]}"
ros2 run go2_onboard_bringup doctor   --config-dir "$HOME/.config/rc2026" --mode lidar
```

`T_base_link_IMU` 表示 IMU 在机体坐标下的安装位姿，必须测量；它与雷达内部的 LiDAR–IMU 外参是两回事。固定配置存放在用户目录，不修改依赖仓库。重复生成配置需显式加 `--force`；只替换指定生成文件。

## 3. 分阶段启动

每次新终端都先 source ROS 和本工作空间。只选一个模式运行，切换前先 `Ctrl+C` 停止上一个入口。

```bash
# 仅真实点云和雷达 IMU
ros2 launch go2_onboard_bringup real.launch.py   config_dir:="$HOME/.config/rc2026" mode:=lidar

# 真实雷达 + FAST-LIO + 机体里程计；不需要已有地图
ros2 launch go2_onboard_bringup real.launch.py   config_dir:="$HOME/.config/rc2026" mode:=odometry
```

可在另一终端查看 `ros2 topic hz /livox/imu`、`ros2 topic hz /Odometry`、`ros2 topic hz /odom`。`/livox/lidar` 使用 Livox `CustomMsg`；FAST-LIO 的 `/cloud_registered_body` 才是可投影的 `PointCloud2`。入口默认无 GUI，远程 SSH 可用；需要可视化时单独启动 RViz。

保存真实 PCD、制作并检查同坐标系的二维地图，见 [建图与部署细节](docs/DEPLOYMENT.md)。准备好实际 PCD 和固定起点后，在 source 过的终端读取实际数值并升级配置：

```bash
read -r -p '实际 PCD 绝对路径: ' REAL_PCD
read -r -a INITIAL_IMU_POSE -p 'IMU 在地图中起点 x y z roll pitch yaw（米/弧度）: '
# 若是新终端，先重新读取前面三个网络/外参变量。
ros2 run go2_onboard_bringup configure_robot --force   --config-dir "$HOME/.config/rc2026" --stage localization   --host-ip "$MID360_HOST_IP" --lidar-ip "$MID360_LIDAR_IP"   --imu-in-base "${IMU_IN_BASE[@]}"   --map-pcd "$REAL_PCD" --initial-pose "${INITIAL_IMU_POSE[@]}"
ros2 launch go2_onboard_bringup real.launch.py   config_dir:="$HOME/.config/rc2026" mode:=localization
```

`--initial-pose` 是 **IMU 在 map 中的位姿**，不是机体地面投影的位置。首次建立 FAST-LIO 地图时的参考原点也要记录。空地图、无效外参、混用坐标系都会导致配置或定位拒绝。

需要规划诊断时，再生成配置并显式加 `--nav-map /absolute/real_map.yaml` 和 `--footprint '[[x1,y1],[x2,y2],[x3,y3],...]'`，其余参数同上；多边形是测量的机体投影包络，所有顶点都要替换成实际数据。然后启动 `mode:=navigation`。当前 costmap 使用静态地图；实时扫描只是诊断输出，未作为实机避障安全依据。控制参数是待标定候选值。

```bash
ros2 run go2_onboard_bringup doctor   --config-dir "$HOME/.config/rc2026" --mode navigation
ros2 service call /onboard/doctor std_srvs/srv/Trigger '{}'
ros2 topic info /cmd_vel --verbose
```

合法配置的静态 doctor 可以返回退出码 0，但仍显示 `motion_enabled=false`、`mission_allowed=false`、`NOT_READY`。常驻 doctor 会补充数据和 TF 新鲜度。规划入口自身不会向 `/cmd_vel` 发布；若实际图中存在其他发布者，查明其来源后再进行任何底层集成。

## 4. 软件回归

```bash
# 回归需要测试依赖；runtime 安装器刻意不安装它们。
sudo apt-get install -y python3-pytest ros-humble-ament-cmake-pytest
cd "$HOME/rc2026_deom"
bash tools/verify_real.bash --workspace "$HOME/rc2026_ws"
```

这是独立的软件回归，包括真实 GICP 原生程序的合成点云测试，以及隔离 domain 中的 Mission 后端闭环。测试替身只在测试入口运行，不会启动真实雷达或向机器马发送命令。运行时不要连接其他机器人 DDS domain。实际 Nav2 topic 隔离还有单独图测试，见 [验证记录](docs/VALIDATION.md)。

## 5. 跑完整障碍赛前还需接入

1. 真实底层状态适配，提供时间戳、停稳、支撑、安全释放、技能活动及故障状态；不能通过发出的零速度推测已停稳。
2. 真实 `SkillExecutor`，把 Mission 的越障请求接到你们的动作控制器，并落实取消、超时和结束后安全释放。
3. 将 Nav2 速度接入带 owner / mission_id / generation 的 `MotionCommand` 通道，再由唯一仲裁器输出 `/cmd_vel`；验证暂停、失效和急停。
4. 用真实地图、机体尺寸、速度和制动数据校准定位、扫描过滤、动态障碍 costmap、到点精度及比赛路线。

接口与接入验收见 [CONTROL_INTEGRATION.md](docs/CONTROL_INTEGRATION.md)，运行结构见 [ARCHITECTURE.md](docs/ARCHITECTURE.md)。完成这些条件前，发布入口不提供运动解锁开关或模拟越障捷径。

第三方版权与声明见 [NOTICE.md](NOTICE.md)。实机地图、录包、网络配置和 build/install 结果不提交到本仓库。
