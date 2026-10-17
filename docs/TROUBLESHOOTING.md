# 实机排错手册：安装、雷达、定位与导航

本手册面向 Ubuntu 22.04／ROS 2 Humble 新手。正常使用顺序见[实机操作指南](REAL_ROBOT_GUIDE.md)，安装和部署背景见[根 README](../README.md)及[部署说明](DEPLOYMENT.md)。先排查当前阶段，不要同时启动多套雷达、定位或技能服务。

文中的默认源码目录是 `~/rc2026_deom`，工作空间是 `~/rc2026_ws`，配置目录是 `~/.config/rc2026`。如果安装时指定了其他路径，请相应替换。命令在机载 Ubuntu 的 Bash 中执行。持续输出的检查命令用 `Ctrl+C` 结束；`echo --once` 在没有消息时也可能一直等待。

## 1. 每个新终端先做这一步

```bash
source /opt/ros/humble/setup.bash
source "$HOME/rc2026_ws/install/setup.bash"
export PYTHONNOUSERSITE=1
ros2 pkg prefix go2_onboard_bringup
```

最后一行应指向本工作空间的 `install` 下。若提示 `Package not found`，先检查第二个 `source` 是否成功、是否使用了安装器的实际 `--workspace` 路径。不要继续执行 launch，也不要靠 source 另一个旧工作空间补齐包。

| 症状 | 检查 | 下一步 |
| --- | --- | --- |
| `ros2: command not found` | `/opt/ros/humble/setup.bash` 是否存在 | 已安装则重新 source；未安装按安装器的 `--install-ros` 流程处理 |
| 找不到包或可执行文件 | `ros2 pkg executables go2_onboard_bringup` | 应列出 `configure_robot`、`doctor`、`doctor_node`；回看安装末尾错误，修好后重跑安装器 |
| `rclpy`、NumPy、SciPy 导入错误 | `command -v python3`；`/usr/bin/python3 -c 'import rclpy,numpy,scipy,yaml; print("imports OK")'` | 使用系统 Python 的干净终端，退出 Conda；不要用 pip 安装另一套 rclpy；保留 `PYTHONNOUSERSITE=1` |
| 新终端中的 IP／外参数量为空 | `printf '%s\n' "$MID360_HOST_IP" "$MID360_LIDAR_IP"`；`declare -p IMU_IN_BASE` | shell 变量不会跨新终端继承，按使用指南重新读取；已有 profile 不需要每次重新生成 |

### 节点在运行，另一个终端却看不到

在启动终端和检查终端分别查看：

```bash
printf 'domain=%s\nrmw=%s\nlocalhost_only=%s\ncyclone_uri=%s\n' \
  "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-系统默认}" \
  "${ROS_LOCALHOST_ONLY:-未设置}" "${CYCLONEDDS_URI:-未设置}"
ros2 node list
```

先让两端使用一致的 domain 与 RMW 配置。远程 RViz 还需实际网卡间发现，不能沿用仅本机的软件测试 `ROS_LOCALHOST_ONLY=1` 和 loopback 配置。更改环境后若 CLI 仍显示旧图，在检查终端执行 `ros2 daemon stop`、`ros2 daemon start` 后重查；这不会重启已经运行的节点，也不会替它们改变环境。先修环境，再检查传感器。

## 2. 安装阶段失败

进入实际 clone 的源码目录再运行安装器。`git clone` 报 `git: command not found` 时，先安装 `git`；克隆报连接超时则先恢复 GitHub 访问。

```bash
cd "$HOME/rc2026_deom"
bash tools/install_real.bash --dry-run
```

预期：输出 Ubuntu／架构、目标 workspace、固定源码及构建命令，不创建 workspace、不执行 sudo 或下载。没有 Humble 时，预览也需显式加 `--install-ros`。正式安装不要带 `--dry-run`。

| 错误 | 原因与下一步 |
| --- | --- |
| `only Ubuntu 22.04 is supported`／`unsupported architecture` | 安装器只接受 Ubuntu 22.04 的 amd64／arm64；用 `cat /etc/os-release` 和 `dpkg --print-architecture` 核对 |
| `ROS Humble is absent` | 按使用指南使用 `--install-ros`；安装器不会无提示安装另一发行版 |
| `workspace path belongs to another checkout` | `workspace/src/rc2026_deom` 已指向另一仓库；核对目标路径，选择新的专用 workspace，保留原目录 |
| `revision mismatch`、`dependency modified`、`install recipe changed` | 固定依赖或安装记录与当前内容不同；保留修改与完整错误，交给维护者核对，或在新的空 workspace 安装；不要 `git reset --hard` 掩盖差异 |
| clone／rosdep／apt 下载失败 | 保存首个下载错误，检查所报官方地址和网络；安装器不自动换第三方镜像，不要复制旧工程未知版本顶替 |
| 编译显示 `Killed` | 可能是内存耗尽；检查 `free -h` 和 `journalctl -k -b --no-pager` 的相关记录；停掉不需要的负载，安装可改用 `--jobs 1`，不要忽略非零退出码 |

`--dry-run` 成功只证明本地检查和命令生成通过；安装日志出现最后的 Next steps 后，仍需完成下面的真实配置和数据检查。

## 3. `configure_robot`／profile 被拒绝

先只读检查现有配置，模式要与当前阶段相符：

```bash
ros2 run go2_onboard_bringup doctor \
  --config-dir "$HOME/.config/rc2026" --mode lidar
```

预期：`configuration_valid: true`，进程退出码为 0。`status: NOT_READY` 在这里是正常的，含义见第 7 节。`doctor` 不带 `--mode` 时默认检查 localization；还没有地图的 sensor_only 配置必须显式选 `lidar` 或 `odometry`。

| 拒绝信息或现象 | 下一步 |
| --- | --- |
| `configure_robot must create profile.yaml first` | 用使用指南的 `configure_robot` 生成配置；确认 `--config-dir` 与 launch 相同 |
| `profile files already exist` | 先备份已有配置；确需重新生成时显式使用 `--force`，它会重写指定生成文件，手改的调参值也可能被覆盖 |
| `imu_in_base must contain 6 numbers`／argparse 缺参 | `--imu-in-base` 后必须是六个独立数值；Bash 数组用 `"${IMU_IN_BASE[@]}"` 展开，不能将整串数字包成一个参数 |
| IP 地址无效／host 与 lidar 相同 | 填真实 IPv4；接收机 IP 和雷达 IP 是两个地址。`ip -br addr` 确认接收机 IP 已在网卡上；生成 JSON 不会自动修改网卡或雷达 IP |
| `map_pcd`／`nav_map` 缺失、空文件、相对路径 | 使用现存实际地图的绝对路径；同时保留二维地图 YAML 所引用的图片 |
| `footprint` 解析／面积／自交错误 | `--footprint` 是被单引号包围的 JSON 数组，如形状 `[[x,y],...]`；必须填实测数值，至少三个不同顶点，不重复首点、不交叉 |
| extrinsics／frames／topics 与 profile 不一致 | 不要只手改一份生成文件；对照 profile 与模板核对。重新生成前备份，不能把校验项删掉绕过 |

`--stage` 只有 `sensor_only` 和 `localization`，没有 `navigation`。规划诊断使用含实际 PCD、固定起点、二维图和 footprint 的 localization profile，再选择 launch 的 `mode:=navigation`。`--initial-pose` 是 **IMU 在 map 中的六维位姿**；`--imu-in-base` 是 **IMU 在 base_link 中的安装外参**，两者不能互换。

## 4. 雷达有无数据？LIO 和 `/odom` 是否正常？

只运行一个机载入口。初次 sensor_only 明确选择 `mode:=lidar` 或 `mode:=odometry`；`real.launch.py` 默认是 localization，没有地图时直接使用默认模式会被拒绝。

### 看不到雷达数据

在检查终端执行：

```bash
ros2 topic list -t
ros2 topic info /livox/lidar --verbose
ros2 topic info /livox/imu --verbose
ros2 topic hz /livox/imu
```

预期：存在实际发布者，IMU 连续更新；具体频率以驱动和现场测量为准。继续检查点云可用 `ros2 topic hz /livox/lidar`。不要直接长时间 `echo /livox/lidar`：它是大量点的 Livox `CustomMsg`，刷屏不能代替收包质量判断。需要看时间戳时只输出一次 header：

```bash
ros2 topic echo /livox/imu --field header --once
```

有 topic 名或有 publisher，不等于持续收到有效载荷。没有数据时按顺序查：启动日志、实际网卡 IP、线缆／供电、JSON 的 host/lidar IP、是否重复运行驱动。若 CLI 与节点的 QoS 不兼容，以 `topic info --verbose` 对照 publisher QoS；不要只根据 `hz` 的空输出断定雷达坏了。跨机器看不到 topic 时先回到第 1 节排 DDS。

### 有雷达，没有 `/Odometry`

`mode:=lidar` 只启动雷达，出现这种情况正常。需要 LIO 时先停止它，再改为 `mode:=odometry`，不要叠加第二个完整入口。

```bash
ros2 node list
ros2 topic info /Odometry --verbose
ros2 topic hz /Odometry
ros2 param get /fastlio_mapping use_sim_time
```

预期：FAST-LIO 节点运行、`/Odometry` 持续更新、实机 `use_sim_time` 为 `false`。初始化期间保持传感器稳定并查看 IMU 初始化日志；不承诺固定几秒完成。核对 FAST-LIO 输入 `/livox/lidar` 的类型为 `livox_ros_driver2/msg/CustomMsg`，输入 IMU 为 `/livox/imu`，日志没有持续时间戳／输入错误。不要把雷达改成仅 PointCloud2 的传输格式来满足 RViz，而破坏 LIO 的 CustomMsg 输入。

### 有 `/Odometry`，没有 `/odom` 或全局 TF

```bash
ros2 topic echo /robot_state/status --once
ros2 topic echo /Odometry --field header --once
ros2 topic echo /Odometry --field child_frame_id --once
ros2 topic info /odom --verbose
ros2 run tf2_ros tf2_echo nav_odom base_footprint
```

源帧应为 `odom → livox_frame`；适配后的 `/odom` 是 `nav_odom → base_footprint`。`pose_derived` 模式需要先积累新鲜的姿态样本，第一帧暂时不能产生可信速度。默认全零的 FAST-LIO twist 不能直接证明机器人停稳。

`local_odom_valid=true` 表示局部里程计可用，`global_ready=true` 还要求该测量时刻存在全局定位 TF。仅 odometry 模式没有 GICP，出现 `GLOBAL_TF_UNAVAILABLE`、`global_ready=false` 是预期；局部里程计仍可工作。定位模式持续如此则转到第 5 节。`INPUT_FRAME_MISMATCH`、`FAULT`、过期／回退时间戳必须先解决，不能通过静态发布假的 `map` TF 消除提示。

## 5. 定位迟迟不通过、位置偏得很大

先确认启动的是 `mode:=localization` 或 `mode:=navigation`，不是 odometry，并已用现场 PCD 和真实固定起点生成 profile。

```bash
ros2 topic echo /gicp/input_status --once
ros2 topic echo /gicp/status --once --qos-durability transient_local
ros2 topic echo /gicp/evidence --once
ros2 topic echo /localization/evidence --once
ros2 service call /mission/localization_ok std_srvs/srv/Trigger '{}'
ros2 run tf2_ros tf2_echo map base_footprint
```

预期：`cloud_count/odom_count/pair_count` 随新数据增加，GICP 持续产生 `accepted=true` 的新测量，定位 health 成功，全局 TF 能更新。连续查看状态时去掉 `--once`，不要只拿启动后的某一条旧 ACCEPTED 当作当前健康。

| GICP／适配器状态 | 怎么查，下一步是什么 |
| --- | --- |
| `WAITING_FOR_SYNCHRONIZED_INPUT` | 看 input_status：某侧 count 为 0 时查对应 publisher、QoS／DDS；两侧都有而 pair 不增加时，比较 `/cloud_registered_body` 与 `/Odometry` 的 header 时间戳和帧。实现要求同时间戳配对，不能把两个 topic 都有频率当作已配对 |
| `FRAME_MISMATCH` | 点云 body 必须为 `livox_frame`，Odometry 必须为 `odom → livox_frame`；核对是否接入另一套 LIO 或桥接 |
| `WAITING_FOR_INITIALPOSE` | 查看实际 profile 的 initial pose 与 GICP `initial_pose_enabled`；确认启动用了该配置。提供测量的 IMU 起点，不把目标点写成当前位置 |
| `STALE_INPUT`／`LOCALIZATION_EXPIRED` | 看数据是否中断、处理是否积压、机器时钟是否跳变；先查输入时间和计算资源，再查配准质量 |
| `TIME_RESET_NEEDS_INITIALPOSE`／`CLOCK_RESET_NEEDS_INITIALPOSE` | 当前估计已失效；先解决时间源，停止该入口，确认机器人真实起点后按正确配置重启。不要继续沿用旧成功状态 |
| `INSUFFICIENT_POINTS` | 核对真实 PCD、当前位置是否在图内、点云是否有效及有足够静态几何；不要靠降低最小点数掩盖空图／错图 |
| `REJECTED`／修正过大 | 核对地图、初始位姿坐标、LiDAR–IMU 与机体外参、遮挡和动态物体；高 inlier ratio 不能抵消过大的位姿修正。不要直接放宽修正／RMSE 门槛让它通过 |
| GICP 已重启，evidence health 仍不恢复 | 定位适配器会绑定来源身份；有意更换／重启定位源时，停止并重新启动完整机载入口，让适配器重新建立来源关联，不混用前后进程证据 |

PCD、二维图与采点必须同原点、同方向、同尺度。重新从另一位置启动 FAST-LIO 后，其局部原点会重建；initial pose 必须与**当前 IMU 在该 PCD map 中的实际位置**一致。`base_link` 或地面投影的坐标不能直接当作 IMU 起点。TF 看起来平滑也不证明实际定位准确，须以场地标志独立核对。

## 6. navigation 已启动，但没窗口／不走／没有 scan

没有 Gazebo、RViz 自动弹出是正常的。机载入口默认无 GUI；需要显示时在有图形环境的终端单独运行 `rviz2`，Fixed Frame 选 `map`，添加实际地图、TF 和路径。SSH 没有桌面时不要用 GUI 是否出现判断后台成功。

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list -t
ros2 topic info /onboard/nav_cmd_raw --verbose
ros2 topic info /cmd_vel --verbose
```

预期：Nav2 lifecycle 节点为 active，并有 `/navigate_to_pose` Action。只有发出诊断目标时才应期待非零诊断速度；即使 Action 工作，机器马也不会因此运动，当前路径不连接电机 `/cmd_vel`。仅运行该入口时 `/cmd_vel` 应没有发布者或不存在；若存在，查 verbose 中的节点名，停止意外的旧启动组，不要直接接线解锁。

若 lifecycle 节点不是 active，查看对应节点最早的配置错误：实际二维图图片是否存在、footprint 是否通过检查、`map → base_footprint` 是否可用、BT／DWB 插件库是否来自当前 workspace。模板的 BT 路径为空是正常的，onboard launch 在运行时覆盖成安装后的项目 `nav_transit.xml`；不要写某台电脑上的源码绝对路径。

诊断扫描话题是 `/onboard/scan_diagnostic`，不是 `/scan`。检查 `/cloud_registered_body` 为 PointCloud2，且到 `base_footprint` 的 TF 有效。当前 costmap 使用静态地图，scan 投影只是诊断；没有动态避障安全验收。规划路线穿过实物时，应核对二维图和机体包络，不能据此认定可以实机行走。

## 7. doctor 的 NOT_READY 到底算不算错误？

两种入口职责不同：

```bash
# 一次性静态配置检查；没有启动节点也可运行。
ros2 run go2_onboard_bringup doctor --config-dir "$HOME/.config/rc2026" --mode localization
# real.launch.py 在运行时才有以下常驻接口。
ros2 service call /onboard/doctor std_srvs/srv/Trigger '{}'
ros2 topic echo /onboard/doctor_status --once
```

| 字段／返回 | 如何理解 |
| --- | --- |
| 静态 `configuration_valid=true`、退出码 0 | 配置文件可解析且契约一致；不证明已收到雷达数据 |
| `PROFILE_INVALID:...`、`configuration_valid=false`、静态退出码 2 | 真正的配置问题，按 reason 修正；确认 doctor 模式和实际 profile 阶段 |
| `status=NOT_READY`、`motion_enabled=false`、`mission_allowed=false` | 本机载入口固定保留运动禁用边界，属于预期，不是解锁提示 |
| 常驻 Trigger 的 `success=false` | 当前实现始终为 false；详细检查在 `message` 的 JSON，不以该布尔值判断定位成功 |
| `live_checks.odometry` | 新鲜、有限且帧正确的 `/odom`；lidar 模式没有里程计时为 false 正常 |
| `live_checks.map_to_footprint_tf`／`accepted_localization` | 新鲜全局 TF／定位 health；lidar、odometry 模式未运行全局定位时为 false 正常；localization/navigation 模式持续 false 则查第 5 节 |
| `localization_ready=true` | 当前配置、里程计、TF、定位 health 同时通过；仍不表示可启动 Mission |
| `measured_low_level_state=false`、`preflight=false` | 未接入完整底层和运动链的机载诊断中是预期；不通过发布假状态使它们变 true |
| `direct_motor_publishers` 非空／`UNEXPECTED_DIRECT_MOTOR_PUBLISHER` | 有节点直接发布 `/cmd_vel`；核对它是谁，不能忽略 |

`checks` 和 `reasons` 中的 `REAL_LOW_LEVEL_STATE_NOT_CONNECTED`、`REAL_SKILL_EXECUTOR_NOT_CONNECTED`、`MOTION_OUTPUT_DISABLED` 是此机载入口的固定边界说明，不是对全仓库扩展的自动发现结果。看本次传感／定位是否正常，应看 `configuration_valid`、`localization_ready` 和 `live_checks`。

## 8. 实际采点和 Hurricane 接口

### 采点面板打不开／TF 采点被拒绝

`capture_panel` 是 Tk 图形面板，纯 SSH／无 DISPLAY 环境不能直接显示。先检查 `echo "$DISPLAY"`，在有桌面的终端使用面板；无 GUI 时按 [Hurricane 文档](../common/robocon_mission/HURRICANE_PORT.md)使用 `hurricane_capture` 命令行录入已经独立测量的 map 坐标：

```bash
ros2 run robocon_mission hurricane_capture --help
ros2 run tf2_ros tf2_echo map base_footprint
```

CLI 手动采点须同时提供 `--label` 和 `--pose x y yaw`；它记录人工输入，不会自动读取实时 TF。TF 采点来自面板，要求新鲜、非零、非未来时间戳。TF 不合格时先修定位，不修改采点时效去容忍旧位姿。

采点前确认实际地图和当前定位一致。`start/finish` 定义场地轴，`finish` 不是比赛终点；八类障碍还分别需要 begin、entry、exit，总共 26 点。每个出口是独立测量的验收位置。未采全、场地轴退化或技能末航点与实测出口不一致时，按导出错误／warnings 逐项核对，参考模板不能代替本场数据。

### 找不到 Hurricane 命令／想先核对技能内容

```bash
ros2 pkg executables robocon_mission
ros2 run robocon_mission hurricane_preview --skill bridge_a
```

预期：预览输出 JSON，含 `dry_run: true`、协议和动作步骤，不打开串口、不执行动作。参考技能中的 `calibrated:false` 表示未完成本场采点。命令缺失时核对当前源码版本与实际安装 workspace；新 clone 或更新源码后仅 source 不能代替安装构建。

### 串口技能适配已有，为什么仍不能整场自动运行？

仓库已有 Hurricane 串口技能适配扩展；它独立于 onboard 诊断入口，默认 dry-run，onboard 不自动启动它。干跑 Action 的 `error_code=DRY_RUN`、`safe_stance=false` 是有意保留的边界，不能作为真实动作成功。

完成正式电控对接时，可先只读检查接口是否存在：

```bash
ros2 action info /execute_obstacle
ros2 topic info /stm32/status --verbose
ros2 topic info /stm32/event --verbose
ros2 service type /motion/get_state
```

仅运行 onboard 时，这些接口可能不存在，属于尚未启动独立技能／底层适配的状态，不是雷达故障。已启动 Hurricane 扩展时，`/execute_obstacle` 应只有一个 Action server；不要并行启动 mock 或另一技能服务器。

若报串口设备缺失或权限错误，可只读检查 `ls -l /dev/serial/by-id` 和 `id -nG`：先确认实际设备存在、端点稳定、当前用户具备团队批准的访问权限。不要用无关串口替代，也不通过全局开放设备权限试动作。

真实技能执行需要实测标定、经电控确认的串口端点与协议、匹配当前 request／runner_session／step／串口 session 的 `/stm32/status`、独立 `/motion/get_state`、定位质量、实际仲裁和支撑保护。串口 write 成功不能证明动作完成，Hurricane Action 结果也不能代替独立运动状态。缺少反馈时不通过关闭 dry-run、补固定定时成功或改门控来排错。实机不订阅旧 `/hurricane/controller_feedback`；它仅保留给 dry-run/测试。具体反馈字段和接入契约见 [HURRICANE_PORT.md](../common/robocon_mission/HURRICANE_PORT.md)与[控制接入说明](CONTROL_INTEGRATION.md)。

## 9. 反馈问题时提供什么

保存失败阶段的启动命令、安装 workspace、doctor 完整 JSON、最早的错误日志、相关 `topic info --verbose` 和第 1 节的 DDS 环境。定位问题补充 `/gicp/input_status`、`/gicp/status`、`/robot_state/status`，说明地图来源、初始位姿是 IMU 还是机体、是否重启过 LIO／GICP。实机 Mission 接入问题另补充 `/mission/startup_status` 和 `/mission/status`；Hurricane 问题补充匹配的 session／step、反馈时间和失败阶段，不发送虚构的完成反馈。

不要只发送最后一句超时，也不需要把整个 CustomMsg 点云刷屏日志贴出来。实际地图、IP、串口信息和录包按团队需要单独保存，不提交进公共仓库。

无硬件的软件闭环与完整回归属于开发验证，不是上述实机操作步骤。需要时另见[验证记录](VALIDATION.md)；不要把其测试遥测、受控 `/clock` 或模拟成功状态当成实机数据；DDS 传输参数按主指南统一选择。
