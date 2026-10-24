# 不使用重定位：固定坐标对齐＋FAST-LIO

本分支基于 `rely-only` 的 `66b524695179fa8152083d8fa6179c9df309c660`。
新增 `no_relocalization.launch.py`，不启动 GICP/ICP，不需要定位 PCD，不调用 `/initialpose`，不提供自动坐标重置。
Mission 默认 `enable_relocalization: false`，不创建 `Relocalize` Action client。

## 坐标约定

- FAST-LIO 继续输出 `/Odometry`：`odom → livox_frame`。
- 新节点发布固定 `map → odom`，以及 `/odometry/evidence`。此 evidence 仅表示连续里程计数据有效，不表示地图匹配成功；绝对定位协方差标为未知。
- 原有 `robot_state_adapter` 继续生成 `/odom` 和 `map → nav_odom → base_footprint → base_link`，保留 IMU 安装外参与机体速度计算。
- `/mission/localization_ok` 和 `/localization/evidence` 接口保持不变，数据源切换为里程计。Nav2 的全局地图、路线和采点仍使用 `map`。

`--odom-in-map X Y Z ROLL PITCH YAW` 是 **T_map_odom**，单位为米、弧度。它不是机器人机体起点，也不是 GICP 的 IMU 初始位姿。
若已知起跑时 IMU 在地图中的位姿 T_map_imu 和同一时刻 FAST-LIO 的 T_odom_imu，应计算：

```text
T_map_odom = T_map_imu × inverse(T_odom_imu)
```

只有地图与当前 FAST-LIO 原点、方向确实一致时，才可使用六个零。程序不会把接收到的第一帧自动吸附到路线起点。
每次重启 FAST-LIO，都必须核对起跑位置、朝向、初始化重力方向和固定变换。历史采点不能直接用于原点不同的新会话。
无重定位不能消除累计漂移；越障后位姿验收仍保留，但不具备独立地图匹配带来的绝对精度保证。

## 配置与启动

安装、雷达网络、安装外参测量和二维地图制作沿用 [实机操作指南](REAL_ROBOT_GUIDE.md)。切到本分支后重新构建：

```bash
git fetch origin
git switch codex/no-relocalization
bash tools/install_real.bash --jobs 2
source /opt/ros/humble/setup.bash
source "$HOME/rc2026_ws/install/local_setup.bash"
```

下面使用独立配置目录，避免覆盖已有 GICP 配置。**将示例 IP、安装外参、T_map_odom、轮廓和地图路径替换成现场测量值**：

```bash
ros2 run go2_onboard_bringup configure_robot \
  --config-dir "$HOME/.config/rc2026-no-relocalization" \
  --host-ip 192.168.1.50 --lidar-ip 192.168.1.111 \
  --imu-in-base 0.11 -0.07 0.08 0.01 0.02 -0.03 \
  --stage localization --localization-source odometry \
  --odom-in-map 1.0 2.0 0.0 0.0 0.0 0.6 \
  --nav-map "$HOME/maps/field.yaml" \
  --footprint '[[-0.4,-0.2],[0.4,-0.2],[0.4,0.2],[-0.4,0.2]]'

ros2 launch go2_onboard_bringup no_relocalization.launch.py \
  config_dir:="$HOME/.config/rc2026-no-relocalization" mode:=navigation
```

仅检查定位链路时使用 `mode:=localization`，无需 `--nav-map`、`--footprint`。导航仍需真实二维占据栅格地图与实测轮廓；本功能不是无地图导航。
配置中不要同时传入 `--map-pcd` 或 `--initial-pose`，否则拒绝生成。
生成目录中保留兼容用的 `gicp.yaml`，无重定位启动入口不会加载或启动 GICP。
旧配置缺少 `localization_source` 时保持 GICP 语义；专用入口会拒绝旧 GICP 配置，避免误启动。

检查：

```bash
ros2 topic echo /localization/evidence --once
ros2 service call /mission/localization_ok std_srvs/srv/Trigger '{}'
ros2 run tf2_ros tf2_echo map base_footprint
ros2 node list
```

预期：有 `odometry_localization`、无 `gicp_localizer`；健康数据的 `source_id` 以 `odometry:` 开头，`covariance_known` 为 false。
无重定位模式不会打开电机。Nav2 输出仍隔离在 `/onboard/nav_cmd_*`；STM32 及整场控制接入前提沿用原项目说明。

## Mission 对接

完成原有仲裁、STM32、安全状态与技能链路联调后，Mission 使用同一份 `map` 路线，并显式启用结构化 evidence：

```bash
ros2 launch robocon_mission mission.launch.py \
  route_file:="$HOME/field/competition_route.yaml" \
  enable_relocalization:=false \
  accepted_pose_topic:=/localization/evidence accepted_pose_type:=evidence \
  accepted_pose_body_frame:=livox_frame autostart:=false
```

定位检查失败时先确认 `SAFE_HOLD`，然后终止任务，不进入 `RELOCALIZING`，也不因重定位服务缺失而等待。
正常越障仍执行技能完成确认、连续停稳、最新定位数据与出口位姿验收，不重复执行技能。
`max_relocalization_retries: 0` 不是关闭开关：历史语义仍允许首次尝试；请使用 `enable_relocalization: false`。

超时、时间倒退、重复时间戳、非法位姿和超过连续性速度阈值的跳变会锁定故障。即使随后收到新数据也不自动恢复，须停机核对坐标并重启整套定位和 Mission。
默认时效阈值 0.5 秒，跳变阈值为 5 m/s、10 rad/s；这些是数据连续性检查，不是运动速度指令或绝对精度保证。缓慢漂移、无明显跳变的同原点重启不能仅靠里程计检测，禁止在任务中单独重启 FAST-LIO。

## 软件验证

新增测试覆盖固定变换、输入故障锁定、无 PCD 配置、互斥启动图、真实 ROS TF/evidence/健康接口，以及无重定位状态机的完成与失败路径。
在已构建、已 source 的 ROS 2 Humble 工作空间运行：

```bash
python3 -m pytest -q \
  common/go2_mission_backends/test/test_odometry_localization.py \
  common/go2_mission_backends/test/test_odometry_localization_ros.py \
  real/go2_onboard_bringup/test/test_profile.py \
  real/go2_onboard_bringup/test/test_no_relocalization_launch.py \
  common/robocon_mission/test/test_mission_fsm.py
```

软件测试使用合成输入，不代表实机 MID-360、STM32、累计漂移或整场障碍赛验收。
本次具体通过项、基线对照和未通过项见 [验证记录](NO_RELOCALIZATION_VALIDATION.md)。
