# 实机操作指南：从机载电脑安装到现场采点

> STM32 接入更新：自制机器人不支持横移。新增单串口网关默认禁用运动；Hurricane 实机改走共享网关。协议为待固件实现草案，见 [STM32 接入指南](STM32_BRIDGE_GUIDE.md)。下文原有定位/规划诊断入口继续保留。


适用设备：**机载 Ubuntu 22.04、ROS 2 Humble、MID-360 自带 IMU、自制 STM32 四足机器人（前后与转向，不支持横移）**。本文所有操作都面向实机部署，使用真实雷达、真实地图和实测坐标。

请先明确当前可操作的范围：按本文可以完成雷达收数、实际场地建图、全局定位、Nav2 路径与速度诊断、Hurricane 现场采点和技能配置导出。**仓库默认入口尚不能驱动机器马自动跑完整场障碍赛**：ROS 侧统一仲裁与串口桥接已新增，底层协议、安全状态和真实动作反馈仍需固件配合。原机载诊断入口保持独立，详见 STM32 接入指南。第 9 节列出具体缺项和接入后的操作顺序。

## 0. 先选当前要做的事情

| 你的现场状态 | 按顺序执行 | 完成后得到什么 |
|---|---|---|
| 小电脑刚装好系统 | 1 → 2 → 3 | 已安装的程序和真实雷达数据 |
| 雷达已经正常，没有现场地图 | 4 → 5 | 实际场地 PCD、地图起点记录和二维图初稿 |
| 已有本场地图，希望定位 | 6 | 新鲜定位证据、机体位置和 RViz 对齐图 |
| 希望检查导航规划 | 7 | 实际地图中的路径与私有诊断速度；机器马不会自动走 |
| 要设置 Hurricane 八类障碍 | 8 | 实测采点、审核后的路线和标定技能文件 |
| 希望开始整场实机自动运行 | 9 | 先完成明确列出的底层接口与联调验收 |

第一次使用从第 1 节开始。之后日常启动看第 10 节。失败时查 [实机排错表](TROUBLESHOOTING.md)。本文的 `read` 命令会停下来等待你在终端输入实际值；输完按 Enter，不要把提示文字当作要执行的命令。

## 1. 在机载 Ubuntu 上安装

### 1.1 确认操作地点和系统

以下命令在**机载 Ubuntu 的 Bash 终端**执行，可以本地操作，也可以 SSH 登录机载电脑执行。Windows 浏览器中的仓库页面不是运行环境。

```bash
lsb_release -ds
uname -m
```

应为 Ubuntu 22.04；`x86_64` 对应 amd64，`aarch64` 对应 arm64。项目安装器支持这两种架构，历史编译验证为 amd64。还需网络能够访问 GitHub 及 Ubuntu / ROS 软件源。

### 1.2 下载、安装、编译

```bash
sudo apt-get update
sudo apt-get install -y git
cd "$HOME"
git clone --branch rely-only --single-branch https://github.com/shyahngyoo/rc2026_deom.git
cd "$HOME/rc2026_deom"
bash tools/install_real.bash --jobs 2
```

如果终端明确提示 `ROS Humble is absent`，使用：

```bash
cd "$HOME/rc2026_deom"
bash tools/install_real.bash --install-ros --jobs 2
```

内存较少可改为 `--jobs 1`。安装器会询问 sudo 密码；输入密码时终端不显示字符是正常现象。安装完成后，源码在 `~/rc2026_deom`，构建与安装结果在 `~/rc2026_ws`。

成功判断：安装器最后输出 `Installed runtime only`，下面第 2 节能找到 `go2_onboard_bringup`。不要只凭最后一行以前没有红字判断编译成功。

**已有本仓库时**，不要在同目录再执行 clone：

```bash
cd "$HOME/rc2026_deom"
git status --short
git branch --show-current
git fetch origin rely-only
git switch rely-only
git pull --ff-only
bash tools/install_real.bash --jobs 2
```

确认工作分支为 `rely-only` 再运行安装器；切换分支前先保存自己的修改。若存在自己的修改、版本冲突或依赖指纹报错，保留修改并查排错表；不要用 `reset --hard` 或删除工作空间来覆盖现场配置。

## 2. 每个操作终端先加载同一套环境

建议准备两个终端：**A 运行主程序**，**B 观察数据、保存地图和采点**。有桌面时再开 C 运行 RViz。

每开一个新终端，都先复制执行这一块：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/rc2026_ws/install/setup.bash"
export PYTHONNOUSERSITE=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$(ros2 pkg prefix go2_mission_backends)/share/go2_mission_backends/config/cyclonedds_sim.xml"
```

这里选择单台机载电脑内部通信，SSH 终端也属于该电脑。XML 文件名中有 `sim`，但它只设置 DDS 传输与进程容量，不启动仿真、模拟定位或 `/clock`。不要同时 source 原开发工作空间。

如果团队已有固定 ROS domain，把所有终端的 `42` 一起改为该值。若需要另一台电脑直接运行 ROS / RViz，两边应使用相同 domain，将 `ROS_LOCALHOST_ONLY` 改为 `0`，并核对实际网卡与 DDS 发现；不要只改其中一个终端。

检查环境：

```bash
ros2 pkg prefix go2_onboard_bringup
ros2 run go2_onboard_bringup configure_robot --help
```

第一条应指向 `~/rc2026_ws/install` 下的新包，第二条应显示 `--host-ip`、`--lidar-ip`、`--imu-in-base` 等选项。出现 `Package not found` 时先解决安装/环境问题，再接下一步。

## 3. 接好 MID-360，先验证真实数据

### 3.1 准备网络和安装数据

按雷达硬件说明完成供电和以太网连接。先记下这些**实际值**：

| 项目 | 怎么确认 | 单位 / 含义 |
|---|---|---|
| 接收机 IP | `ip -br addr` 中连接雷达的真实网卡 IPv4 | 不能用 Wi-Fi 或旧 WSL 地址代替 |
| MID-360 IP | 雷达实际配置或设备网络信息 | 与接收机不同且网络互通 |
| IMU 安装平移 | IMU 参考点在 `base_link` 下的 x、y、z | 米，x 向前、y 向左、z 向上 |
| IMU 安装旋转 | IMU 坐标轴相对 `base_link` 的 roll、pitch、yaw | 弧度，右手系 |

`--imu-in-base` 是机体到 IMU 的安装关系，不是 MID-360 内部 LiDAR–IMU 外参，也不是机器人在地图中的初始位置。尚未测量时先补测，不能把六个零当作任意机器的标定值。

```bash
ip -br addr
```

如网卡没有所需接收 IP，先在 Ubuntu 的“设置 → 网络 → 有线 → IPv4”或团队的网络配置中设置正确地址与掩码。**下面的配置向导只写雷达程序配置，不会替操作系统设置网卡 IP。**

### 3.2 首次生成配置

在已执行第 2 节的终端 B 中：

```bash
read -r -p '机载接收网卡的 IPv4: ' MID360_HOST_IP
read -r -p 'MID-360 的实际 IPv4: ' MID360_LIDAR_IP
read -r -a IMU_IN_BASE -p '实测 IMU 安装 x y z roll pitch yaw（六个数，用空格分开）: '
ros2 run go2_onboard_bringup configure_robot \
  --config-dir "$HOME/.config/rc2026" \
  --stage sensor_only \
  --host-ip "$MID360_HOST_IP" --lidar-ip "$MID360_LIDAR_IP" \
  --imu-in-base "${IMU_IN_BASE[@]}"
ros2 run go2_onboard_bringup doctor \
  --config-dir "$HOME/.config/rc2026" --mode lidar
```

期望配置命令输出 `"status": "CONFIGURED"`；doctor 显示 `"configuration_valid": true`。doctor 仍显示 `NOT_READY`、`motion_enabled:false` 和 `mission_allowed:false`，这是本机载入口的整体运行边界，不等于雷达收数失败。

如果提示配置已存在，先备份：

```bash
cp -a "$HOME/.config/rc2026" "$HOME/.config/rc2026.backup.$(date +%Y%m%d_%H%M%S)"
```

再对确实需要更新的配置命令增加 `--force`。生成文件位于 `~/.config/rc2026`，源码与 `install` 目录中的示例配置无需修改。后续重开终端时，刚才输入的 Bash 变量不会自动保留；运行程序直接读取已经生成的文件。

### 3.3 启动与检查

终端 A：

```bash
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=lidar
```

终端 B：

```bash
ros2 topic list
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/imu
```

`/livox/lidar` 应为 `livox_ros_driver2/msg/CustomMsg`，`/livox/imu` 应为 `sensor_msgs/msg/Imu`；`hz` 持续显示采样统计时表示该订阅端正在收数据。用 Ctrl+C 结束 `hz`，不影响 A 的驱动。

如需确认激光消息到达，可单独运行 `ros2 topic hz /livox/lidar`；不要直接持续 echo 整帧点云刷满终端。**仅有 topic 名称或消息类型不能证明雷达实际收包。**

完成后在 A 按 Ctrl+C，等待所有子进程退出，再进入下一模式。不要另启第二个 MID-360 驱动。

## 4. 用真实场地建图，并记录地图起点

### 4.1 先保持机器人在可复现的起点

在现场标记建图起点与朝向，记录雷达安装状态。启动时保持机体静止，等待 FAST-LIO 初始化；之后由已有底层/人工控制在场地中移动采集。本项目在此阶段不自动驱动建图。

终端 A：

```bash
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=odometry
```

终端 B 逐条检查，每个 `hz` 用 Ctrl+C 结束后再执行下一条：

```bash
ros2 topic hz /Odometry
ros2 topic hz /odom
ros2 topic hz /cloud_registered_body
```

成功现象：原始 IMU 位姿 `/Odometry`、适配后的机体里程计 `/odom` 和注册点云持续更新。此时没有全局地图定位，`map→base_footprint` 不可用或 doctor 的定位状态为 false 可以是正常现象。

**第一次移动之前**，保存起点参考：

```bash
mkdir -p "$HOME/rc2026_maps"
ros2 topic echo /Odometry nav_msgs/msg/Odometry --once \
  > "$HOME/rc2026_maps/mapping_start_odom.yaml"
```

这记录的是 IMU 的实际参考位姿。PCD 使用建图时的参考坐标，不能只记机器人“站在某个地砖上”却忘记朝向和 IMU 位置。

### 4.2 在 RViz 观察建图

有桌面或已配置图形显示的终端 C：

```bash
rviz2 -d "$(ros2 pkg prefix go2_real_bringup)/share/go2_real_bringup/config/real_mapping.rviz"
```

RViz 使用真实时间（不要启用 Use Sim Time）。该视图的 Fixed Frame 为 `odom`，查看 `/cloud_registered`、`/Laser_map` 和 `/Odometry`。机载主入口默认不启动 RViz；纯 SSH 无图形显示时，先通过 B 的数据检查继续，不要把没有 GUI 当成程序失败。

### 4.3 保存 PCD

场地扫描完成后，**保持 A 运行**，在 B 执行：

```bash
ros2 run go2_fastlio_localization save_live_map \
  --ros-args -p output_dir:="$HOME/rc2026_maps" -p timeout_sec:=45.0
```

成功输出包含：

```text
SAVED /home/你的用户名/rc2026_maps/real_room_时间戳.pcd
POINTS 大于0的点数
BYTES 文件大小
FRAME odom
```

记录实际 `SAVED` 路径。文件名由时间生成，不能把上面的示意路径照抄到定位配置。该命令保存当前累计地图快照，不做全局闭环优化；墙面重影、走廊累计漂移需要检查实际地图质量。

保存成功后在 A 按 Ctrl+C。下一次定位必须加载本次保存的实际 PCD。

## 5. 制作同一坐标系的二维导航图

定位使用三维 PCD，Nav2 使用二维 `field.yaml + field.pgm`，两者都要准备。

在终端 B 输入实际路径和依据实测高度选择的障碍切片：

```bash
read -r -p '刚保存的 PCD 绝对路径: ' REAL_PCD
read -r -p '障碍切片 z_min（PCD 坐标，米）: ' MAP_Z_MIN
read -r -p '障碍切片 z_max（PCD 坐标，米，须大于 z_min）: ' MAP_Z_MAX
ros2 run go2_fastlio_localization pcd_to_nav2_map \
  "$REAL_PCD" "$HOME/rc2026_maps/field" \
  --resolution 0.05 --z-min "$MAP_Z_MIN" --z-max "$MAP_Z_MAX"
ls -lh "$HOME/rc2026_maps/field.yaml" "$HOME/rc2026_maps/field.pgm"
```

应输出两份文件。检查地面是否被误当障碍、通道是否保留、边界是否正确。投影工具可能把未观察区域也输出为 free，必须人工审核可通行范围及未知区域。不要独立平移/旋转 PGM 后仍沿用原 YAML 原点；PCD、二维图和采点必须使用同一 `map`。

## 6. 在真实地图中定位

### 6.1 准备地图中的 IMU 初始位姿

需要六个数：`map_x map_y map_z map_roll map_pitch map_yaw`，单位米/弧度。它们描述**当前 IMU 在地图中的位姿**。

如果机器人确实返回第 4 节保存时的相同 IMU 位置与朝向，且 PCD 未变更坐标，可从起点记录换算参考值：

```bash
python3 - "$HOME/rc2026_maps/mapping_start_odom.yaml" <<'PY'
from pathlib import Path
import math, sys, yaml
records = yaml.safe_load_all(Path(sys.argv[1]).read_text())
m = next(v for v in records if isinstance(v, dict) and "pose" in v)
p = m["pose"]["pose"]; q = p["orientation"]; t = p["position"]
x, y, z, w = (float(q[k]) for k in ("x", "y", "z", "w"))
n = math.sqrt(x*x + y*y + z*z + w*w)
if not math.isfinite(n) or n < 1e-9: raise SystemExit("起点四元数无效")
x, y, z, w = x/n, y/n, z/n, w/n
roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
print(" ".join(str(v) for v in (t["x"], t["y"], t["z"], roll, pitch, yaw)))
PY
```

机器人换了起点、朝向、雷达安装或地图坐标后，这个旧记录就不能代表当前初始位姿。Hurricane 入口采点保存的是 `base_footprint` 的平面位姿，也不能直接拿来填 IMU 六维初始位姿。

### 6.2 生成定位配置

以下块重新询问完整输入，适合在新终端直接执行。先确认要替换第 3 节配置，并保留备份。`--force` 会重建所有生成文件，不是合并更新；已手调的参数需另行记录，遗漏二维图或 footprint 会清空这些字段：

```bash
read -r -p '机载接收网卡 IPv4: ' MID360_HOST_IP
read -r -p 'MID-360 IPv4: ' MID360_LIDAR_IP
read -r -a IMU_IN_BASE -p '实测 IMU 安装 x y z roll pitch yaw: '
read -r -p '实际 PCD 绝对路径: ' REAL_PCD
read -r -a INITIAL_IMU_POSE -p '当前 IMU 在地图中 x y z roll pitch yaw: '
ros2 run go2_onboard_bringup configure_robot --force \
  --config-dir "$HOME/.config/rc2026" --stage localization \
  --host-ip "$MID360_HOST_IP" --lidar-ip "$MID360_LIDAR_IP" \
  --imu-in-base "${IMU_IN_BASE[@]}" \
  --map-pcd "$REAL_PCD" --initial-pose "${INITIAL_IMU_POSE[@]}"
ros2 run go2_onboard_bringup doctor \
  --config-dir "$HOME/.config/rc2026" --mode localization
```

配置验收仍看 `configuration_valid:true`。A 启动：

```bash
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=localization
```

### 6.3 判断定位是否有效

B：

```bash
ros2 service call /mission/localization_ok std_srvs/srv/Trigger '{}'
ros2 topic echo /localization/evidence go2_obstacle_interfaces/msg/LocalizationEvidence --once
ros2 run tf2_ros tf2_echo map base_footprint
```

结束 `tf2_echo` 用 Ctrl+C。还要查看常驻诊断：

```bash
ros2 service call /onboard/doctor std_srvs/srv/Trigger '{}'
```

成功判断：定位健康服务当前返回成功；新定位证据中的测量时间和序号在持续推进；doctor 的 `live_checks.odometry`、`map_to_footprint_tf`、`accepted_localization` 为 true，`localization_ready:true`。仅有一条旧 `accepted:true` 或一个 TF 不足以说明现在健康。

`/onboard/doctor` 的 Trigger `success:false` 与顶层 `NOT_READY` 在本入口中固定保留；这不是上面三个定位检查的结果。不要为了消除 NOT_READY 去关闭门控。详细字段解释见排错表。

C 查看实际地图与点云对齐：

```bash
rviz2 -d "$(ros2 pkg prefix go2_real_localization)/share/go2_real_localization/rviz/localization.rviz"
```

应看到参考地图与当前点云在同一 `map` 下对齐。数值健康还要结合地图对齐和实测位置核对；RMSE 不是机器马实际落点误差。持续 `correction too large` 或点云明显错位时停在此步排查初猜、外参、时钟和地图，先不采点、不调宽门限。

## 7. 实机地图上的 Nav2 规划诊断

这一阶段使用真实定位和实测地图，**控制速度仍隔离在 `/onboard/nav_cmd_*`，机器马不会按目标自行走**。它用于检查路径、TF、footprint 和规划入口。

先在 A 停止 localization。B 重新生成包含二维图与机体包络的完整配置：

```bash
read -r -p '机载接收网卡 IPv4: ' MID360_HOST_IP
read -r -p 'MID-360 IPv4: ' MID360_LIDAR_IP
read -r -a IMU_IN_BASE -p '实测 IMU 安装 x y z roll pitch yaw: '
read -r -p '实际 PCD 绝对路径: ' REAL_PCD
read -r -a INITIAL_IMU_POSE -p '当前 IMU 在地图中 x y z roll pitch yaw: '
read -r -p '二维地图 YAML 绝对路径: ' REAL_NAV_MAP
read -r -p '实测 footprint JSON（例如语法 [[x1,y1],[x2,y2],[x3,y3]]，输入实际数字）: ' REAL_FOOTPRINT
ros2 run go2_onboard_bringup configure_robot --force \
  --config-dir "$HOME/.config/rc2026" --stage localization \
  --host-ip "$MID360_HOST_IP" --lidar-ip "$MID360_LIDAR_IP" \
  --imu-in-base "${IMU_IN_BASE[@]}" \
  --map-pcd "$REAL_PCD" --initial-pose "${INITIAL_IMU_POSE[@]}" \
  --nav-map "$REAL_NAV_MAP" --footprint "$REAL_FOOTPRINT"
ros2 run go2_onboard_bringup doctor \
  --config-dir "$HOME/.config/rc2026" --mode navigation
```

footprint 是 `base_footprint` 下实测的二维外包络，单位米；按轮廓顺序输入至少三个不同顶点，不自交，不重复首顶点。不能照搬旧机器狗尺寸。

A：

```bash
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=navigation
```

B 逐条看状态：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
ros2 topic info /onboard/nav_cmd_raw --verbose
ros2 topic info /cmd_vel --verbose
```

Nav2 节点应达到 `active`。私有速度输出的发布节点应是 `velocity_smoother`；本入口不向 `/cmd_vel` 发布。现场若已有其他 `/cmd_vel` 发布者，要识别其真实来源，不能只看总数就断言来自本导航入口。

C 可以运行：

```bash
rviz2 -d "$(ros2 pkg prefix go2_nav_bringup)/share/go2_nav_bringup/rviz/navigation.rviz"
```

此通用视图中包含旧机器人模型、AMCL 和相机显示项；本入口没有这些数据时可以关闭相应显示。重点观察 `/map`、TF、footprint 和路径。当前 scan 诊断话题是 `/onboard/scan_diagnostic`，通用视图默认 `/scan` 要手动改成它；当前 costmap 只启用静态地图和 inflation 层。

用 RViz 的 Nav2 Goal 选择地图内已知可通行位置，或在 B 构造目标：

```bash
read -r -p '目标 map x（米）: ' GOAL_X
read -r -p '目标 map y（米）: ' GOAL_Y
read -r -p '目标 map yaw（弧度）: ' GOAL_YAW
GOAL_JSON="$(python3 - "$GOAL_X" "$GOAL_Y" "$GOAL_YAW" <<'PY'
import json, math, sys
x, y, yaw = map(float, sys.argv[1:])
if not all(math.isfinite(v) for v in (x, y, yaw)): raise SystemExit("目标须为有限数值")
print(json.dumps({"pose": {"header": {"frame_id": "map"}, "pose": {
    "position": {"x": x, "y": y, "z": 0.0},
    "orientation": {"z": math.sin(yaw/2), "w": math.cos(yaw/2)}}}}))
PY
)"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "$GOAL_JSON" --feedback
```

目标接受后可在另一终端观察 `/onboard/nav_cmd_raw`。诊断目标只在独立诊断入口发送，不与运行中的 Mission 竞争目标。由于运动路径还未接入，机器人不移动，目标可能因无进展失败；**此阶段不以物理到达目标为验收条件**。不要将该 topic 直接 remap 到底层 `/cmd_vel` 来试图“让它先走”。

如要取消当前 Nav2 目标，显式请求取消（请求当前所有 Nav2 goal）：

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
  '{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}'
ros2 topic echo /navigate_to_pose/_action/status action_msgs/msg/GoalStatusArray \
  --qos-durability transient_local --qos-reliability reliable
```

对照发送目标时打印的 UUID，持续观察该 goal 达到 `STATUS_CANCELED`（5），或已明确为 `SUCCEEDED`（4）/`ABORTED`（6），再 Ctrl+C 结束状态观察。不要将其他 goal 的历史终态或中间的 `CANCELING` 当作当前目标已结束。取消接受和终态是两件事。当前 Humble 的 `send_goal` 终端 Ctrl+C 会请求取消，仍须看到终态确认；CLI 超时退出不表示已经取消。上面的显式服务调用适合从另一个终端操作。

## 8. 在真实赛场采点，导出 Hurricane 路线

保持第 6 或第 7 节的真实全局定位正常；先取消诊断目标。采点时使用已有的人工/底层控制方式移动机体，本工具只记录坐标，不驱动它到点。

### 8.1 需要采什么

| 标签 | 含义 |
|---|---|
| `start`、`finish` | 用于确定场地轴的两个标记；`finish` 不是比赛任务终点 |
| 每个障碍 `.begin` | 障碍局部坐标原点 |
| 每个障碍 `.entry` | 导航停稳并对准的技能入口 |
| 每个障碍 `.exit` | 真实越障结束后用于独立验收的位置 |

八个障碍 ID 是 `bridge_a`、`bridge_b`、`big_slope`、`limit_pole`、`bunker`、`pole`、`high_wall`、`step`。共 26 点，记录时机器人需处在对应位置且定位有效。这里的点使用 `map→base_footprint`，不是 IMU 初猜。

### 8.2 打开采点面板

采点面板是 **Tk 图形窗口**，需要在机载桌面或已配置图形转发的 ROS 会话运行。它不是纯文本终端面板。若提示缺少 Tk，可安装 `sudo apt-get install python3-tk`；若只有纯 SSH，无显示环境，使用下方手动录入已测坐标的方式，不能假定该方式会自动读取 TF。

有图形显示的终端（同样先加载第 2 节环境）：

```bash
mkdir -p "$HOME/rc2026_field"
ros2 run robocon_mission capture_panel \
  --file "$HOME/rc2026_field/field.yaml"
```

在窗口中先点“采集起点 start”“采集场地轴终点 finish”，随后每个障碍依次点“原点 begin”“入口 entry”“出口 exit”；窗口应显示已采集数量直至 `26/26`。采点会立即写入指定 field.yaml。只有按要求采齐、核对坐标后才导出。修改地图或地图原点后需要重新检查全部点，不能继续复用旧场地坐标。

已有独立测量的地图点，也可逐点录入；以下会询问实际标签和实际坐标：

```bash
read -r -p '点标签（例如 bridge_a.entry）: ' POINT_LABEL
read -r -a POINT_POSE -p '实际 map x y yaw（米/弧度）: '
ros2 run robocon_mission hurricane_capture \
  --file "$HOME/rc2026_field/field.yaml" \
  --label "$POINT_LABEL" --pose "${POINT_POSE[@]}"
```

手动录入功能无法验证你给出的点是否实测，录入责任仍在现场操作人员。不要复制参考模板里的坐标充当已完成采点。

### 8.3 导出并检查

```bash
ros2 run robocon_mission hurricane_capture \
  --file "$HOME/rc2026_field/field.yaml" \
  --export-route "$HOME/rc2026_field/hurricane_route.yaml" \
  --skills-profile "$(ros2 pkg prefix robocon_mission)/share/robocon_mission/config/hurricane_skills.yaml" \
  --export-skills "$HOME/rc2026_field/calibrated_skills.yaml"
ros2 run robocon_mission hurricane_preview \
  --skills-file "$HOME/rc2026_field/calibrated_skills.yaml" --skill bridge_a
```

预览只读取配置，不发送串口动作。检查导出的各障碍入口、出口、朝向、阶段与 `calibration.warnings`，尤其检查技能末航点是否与实测出口匹配。这里的 `calibrated` 表示几何配置已导出，不表示已通过实机动作验收。

Hurricane 源坐标的左右方向与 ROS 不同，工具内部已转换；外部手工录入使用 ROS 地图坐标和 rad，不要再次自行把 y 或 yaw 取负。完整协议和反馈定义见 [HURRICANE_PORT.md](../common/robocon_mission/HURRICANE_PORT.md)。

## 9. 要让机器马真正自动走和越障，还需完成的接线与接口

以下是当前默认机载入口与整场自动运行之间的实际缺项，不能只靠填写地图解决。

| 需要交付的部分 | 必须有的真实数据 / 行为 | 当前操作判断 |
|---|---|---|
| 底层安全状态适配 | 新鲜 `/motion/telemetry`，真实停稳、安全支撑、技能活动与可释放；独立 `/motion/get_state` | 用户此前说明尚无这些状态，需电控/底层补接 |
| 生产导航命令适配 | 将 Nav2 速度绑定权威 `mission_id/generation` 转成 `MotionCommand`，由唯一仲裁器输出 `/cmd_vel` | 当前私有诊断输出没有自动连接 |
| Hurricane 执行端与反馈 | 电控实现新协议，提供匹配 request/session/step 的真实 STATUS；网关统一接入 | 串口写成功不是动作完成 |
| 统一控制权 | Nav2 与串口动作实际受同一仲裁链路控制；故障、取消、超时有真实安全收尾 | 不能仅改变 ROS owner 字段 |
| 实机启动检查配置 | 正确的唯一输出节点、真实低层状态、定位、技能、路线都通过 preflight | 默认机载 preflight 仍按“电机输出关闭”配置 |

先用下面命令与电控共同核对已接入的端点：

```bash
ros2 topic info /motion/telemetry --verbose
ros2 service type /motion/get_state
ros2 service type /motion/set_control_mode
ros2 topic info /motion/control_state --verbose
ros2 topic info /motion/authority --verbose
ros2 topic info /hurricane/controller_feedback --verbose
```

有这些名字不等于完成接入；还须核对数据新鲜度、来源、会话和实际物理状态。精确契约见 [CONTROL_INTEGRATION.md](CONTROL_INTEGRATION.md)。

Hurricane 独立预览默认 `dry_run:=true`。真实执行改用唯一 STM32 网关，不再发送旧 `S编号E` 串口帧，也不接受旧 feedback topic 作为实机完成证据。请按 STM32 接入指南完成协议、真实状态和仲裁联调。只读采点不需要打开串口。

接入完成后的现场顺序应为：底层与急停可用 → 唯一仲裁器及真实状态 → 雷达/定位 → 生产 Nav2 命令适配 → 已标定的真实技能服务 → preflight 成功 → Mission 手动启动。先逐项完成“普通点、单个技能、单障碍完整链”验收，再自动整场。

实机 Mission 必须显式使用 `simulation_mode:=false`、`use_sim_time:=false`、`autostart:=false`。连接本仓库 `/localization/evidence` 时还必须使用 **`accepted_pose_type:=evidence`**；默认 `covariance` 不能读取这个消息类型。不得省略实机 preflight 服务来绕过启动检查。

**本仓库尚未提供包含这些生产接口的一键全场 bringup。** 因此本指南不把独立启动 `mission_manager` 或把 `autostart` 改成 true 当作已经能实机跑赛。完整接口接通后，`/mission/start` 的标准调用为：

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

成功执行链应是：导航到入口 → 精确接近 → 连续停稳与入口验收 → 获取技能控制权 → 越障 → 独立确认安全结束 → 获取本阶段新定位 → 独立出口验收 → 恢复导航 → 下一障碍 → 返回点验收。

## 10. 日常操作速查

第一次已经完成安装、真实配置和现场地图后，每次使用不用重新 clone、建图或生成配置。

1. 连接设备、确认雷达安装没变，把机器人放到已知地图起点。
2. A、B 分别加载第 2 节环境。
3. 核对当前 `~/.config/rc2026/profile.yaml` 的地图和 IMU 起点是否符合实际位置；起点改变需重新生成配置。
4. A 启动 `mode:=localization`；需要规划检查则启动 `mode:=navigation`，只选一个。
5. B 检查定位健康与 doctor 动态字段，C 按需启动对应 RViz。
6. 需要采点时用第 8 节；需要真实运动时先满足第 9 节生产接入条件。
7. 退出诊断前显式取消活动 Nav2 goal，再在 A 按 Ctrl+C 等待子进程退出。

模式切换使用同一主入口的不同 mode，不是把 lidar、odometry、localization、navigation 四个 launch 同时开起来。以后接入真实运动后，停止操作还须先完成底层安全停止；退出 ROS 进程不等同于机器已经停稳。

| 现场文件 | 用途 | 更改时机 |
|---|---|---|
| `~/.config/rc2026/` | 雷达网络、安装外参、初猜、Nav2 等运行配置 | 网络、安装、起点或地图改变 |
| `~/rc2026_maps/` | 真实 PCD、二维图、建图起点记录 | 地图重建或审核修改 |
| `~/rc2026_field/field.yaml` | 现场 26 点 | 地图/场地/障碍位置改变 |
| `hurricane_route.yaml`、`calibrated_skills.yaml` | 审核后的任务与技能 | 重新采点或技能调整后重新导出 |

这些现场文件不在 git 源码里。更新软件前备份，改地图后重新核对定位初猜与采点。参数与构建细节查 [DEPLOYMENT.md](DEPLOYMENT.md)，按症状排查查 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。

## 说明核对范围

本文按 2026-10-15 仓库接口核对，并进行了命令语法、安装后 CLI 帮助、无动作技能预览、Nav2 消息字段与 RViz 文件路径检查。编写指南时没有连接真实雷达或执行真实动作；现场每步需以上述实际数据与物理观测验收。
