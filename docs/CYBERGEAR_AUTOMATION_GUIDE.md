# 新版 CyberGear：按键通信与自动化实机联调

> **新版面板已支持模板选择、采点生成任务和单障碍/整路线运行：请优先用 [CyberGear 现场操作简版](CYBERGEAR_QUICK_START.md)。** 下文手动 YAML 流程仅为底层调试参考，不再是普通操作必做步骤。

适用固件：2026-10-26 提供的 CyberGear.zip 与 2.4G.zip。STM32 不改；机载电脑继续使用 rely-only。

**先按原[实机操作指南第 9 节](REAL_ROBOT_GUIDE.md#9-新版-cybergear从采点接到自动运行)执行完整流程。** 本文是通信参数、单键和排错参考，不是另一套必须重做的流程。统一终端：A 定位，B 串口与任务，C 观察和发启动/停止命令。新模式的任务文件与原配置关系、采点如何接入，已写入原指南第 9 节及新版简版指南。

## 1. 这次选哪个入口

- 新版按键固件：`cybergear_auto.launch.py`，按本指南操作。
- 旧 v1 JSON 协议：`stm32_bringup.launch.py`，仍保留，但不适用于这次固件。
- 不要同时启动两个串口入口。新入口不启动旧 Mission、Hurricane、Nav2、雷达或定位；自身包含按键任务执行器。不要同时运行另一套运动输出。

新入口支持：单次按键、按顺序自动执行模式/动作/等待、按定位走航点。默认只观察；加 `enable_motion:=true` 后可发动作。

第一轮只测通信和动作，不用地图。走地图航点时，再单独启动已有定位。新入口不需要二维 Nav2 地图。

## 2. 更新和编译（终端 B）

在机载电脑拉取 rely-only 后编译；工作区应已链接这个仓库。

```bash
cd ~/rc2026_deom
git switch rely-only
git pull --ff-only origin rely-only
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
cd ~/rc2026_ws
colcon build --symlink-install --executor sequential \
  --packages-up-to go2_onboard_bringup quadruped_stm32_bridge
source ~/rc2026_ws/install/setup.bash
ros2 pkg executables quadruped_stm32_bridge
```

输出应含 `cybergear_bridge`。工作区需已按原安装指南链接这个仓库。若缺依赖：

```bash
sudo apt-get install -y python3-serial python3-yaml ros-humble-tf2-ros ros-humble-std-srvs
```

以后每个新终端先执行（确实是两行）：

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
```

## 3. 接线及找串口

USB-TTL TX → STM32 PB11（USART3 RX）；USB-TTL RX → PB10（USART3 TX）；GND 共地。使用匹配 STM32 的 3.3 V TTL 电平；不要把 RS232 接到这些引脚。

```bash
ls -l /dev/serial/by-id/
```

优先使用该目录下的完整稳定路径。下面用 `/dev/ttyUSB0` 举例，替换成实际设备。权限不足时：

```bash
sudo usermod -aG dialout "$USER"
```

注销重新登录后生效。ToDesk 不影响这条串口通信。

## 4. 先看回传，不发运动

终端 B：

```bash
ros2 launch go2_onboard_bringup cybergear_auto.launch.py port:=/dev/ttyUSB0
```

机器人完成原有初始化后，把遥控器开关从手动拨到自动。上电时已经在自动位置的，要先拨回手动再拨到自动。

终端 C：

```bash
ros2 topic echo /cybergear/status
```

判断方法：

- `connected: true`：串口已打开，不等于 STM32 已回复。
- `status` 有内容、`status_age_s` 很小：已收到通过 CRC 的状态。
- `ready: true`：收到新鲜回包、已初始化（不是 z）、故障码为 0；不是动作完成/站稳证明。
- `motion_enabled: false`：当前只观察。
- `status.mode`：机器人选择的模式；`fault`、`motor_id`、`detail`：固件诊断信息。

固件只在自动打开时约每 80 ms 回传。无回包时检查遥控器开关、串口、交叉接线。这里没有 HELLO/ACK，不使用旧 `/stm32/status` 判断。

## 5. 直接调单次动作

终端 B 先 Ctrl+C 退出观察，再启动：

```bash
ros2 launch go2_onboard_bringup cybergear_auto.launch.py \
  port:=/dev/ttyUSB0 enable_motion:=true
```

终端 C 选择普通模式：

```bash
ros2 topic pub --once /cybergear/key std_msgs/msg/String '{data: "9"}'
```

查看状态，等 `status.mode` 为 `a` 且 `pending_mode` 为 null，再发前进：

```bash
ros2 topic pub --once /cybergear/key std_msgs/msg/String '{data: "2"}'
```

停止：

```bash
ros2 service call /cybergear/stop std_srvs/srv/Trigger '{}'
```

普通模式：1 左转、2 前进、3 右转、4 后退、5 停止站立。动作持续由固件决定，不是发一次只走一小步。6/7/8/19/20 的含义取决于模式，按电控动作表使用，不要按旧 v1 技能编号直接发送。

不要用 `ros2 topic pub -r` 连发跳跃键。手动切换模式后必须等待反馈；自动任务正在执行时拒绝普通手动按键，停止键仍可用。

| 模式 | 选择按键 | 回传字符 |
|---|---:|---|
| 普通 | 9 | a |
| 台阶 | 18 | b |
| 小坡 | 10 | c |
| 桥 B | 11 | d |
| 桥 A | 12 | e |
| 大坡 | 13 | f |
| 爬行 | 14 | g |
| 沙坑 | 15 | h |
| 绕杆 | 16 | j |
| 高墙 | 17 | k |

## 6. 不建图，先跑一段自动时序

准备个人配置，源码更新不会覆盖它：

```bash
mkdir -p ~/.config/rc2026
cp "$(ros2 pkg prefix quadruped_stm32_bridge)/share/quadruped_stm32_bridge/config/cybergear.yaml" ~/.config/rc2026/cybergear.yaml
cp "$(ros2 pkg prefix quadruped_stm32_bridge)/share/quadruped_stm32_bridge/config/cybergear_plan_example.yaml" ~/.config/rc2026/cybergear_plan.yaml
nano ~/.config/rc2026/cybergear_plan.yaml
```

自带示例只选普通模式并停止。要试普通模式前进 1 秒，把任务文件改为：

```yaml
steps:
  - {type: mode, key: 9, timeout_s: 5.0}
  - {type: key, key: 2, duration_s: 1.0}
  - {type: key, key: 5, duration_s: 0.5}
```

终端 B 停掉旧实例，再启动：

```bash
ros2 launch go2_onboard_bringup cybergear_auto.launch.py \
  port:=/dev/ttyUSB0 enable_motion:=true \
  params_file:="$HOME/.config/rc2026/cybergear.yaml" \
  plan_file:="$HOME/.config/rc2026/cybergear_plan.yaml"
```

终端 C 开始执行：

```bash
ros2 service call /cybergear/start std_srvs/srv/Trigger '{}'
```

`start` 每次从第 0 步开始，不自动启动、不自动续跑。任务结束显示 `task: completed`，并发送一次 5。修改 YAML 后重启入口再 start。

四种步骤：

| type | 必填数据 | 行为 |
|---|---|---|
| mode | key | 发模式键一次，等待新回包匹配；默认超时 5 秒 |
| key | key、duration_s | 发动作键一次，等待指定时长再进入下一步 |
| wait | duration_s | 只等待，不发送按键，也不自动停止上一动作 |
| waypoint | x、y | 跟随地图坐标；可填 yaw、mode_key、timeout_s |

`duration_s` 是上位机等待时间，不是传给 STM32 的动作时长。key 步骤结束不会自动发 5；需要停止就在下一步明确写 key 5。跳跃按键只发送一次，duration_s 按实机动作时间调足，随后继续下一步。等待时间不代表收到动作完成反馈。

## 7. 加入定位与采点，跑入口、出口

终端 A 按原指南完成建图、地图配置和初始定位，然后保持定位运行：

```bash
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=localization
```

新任务默认读取 `map → base_footprint`，与原采点参考系一致。先检查：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

从已采集的点中取 x、y、yaw 写入步骤；单位米、弧度。以下坐标仅为格式示例，必须换成现场采点：

```yaml
steps:
  # 普通路段：需要经过的中间点也依次写上
  - {type: waypoint, x: 1.0, y: 0.0, yaw: 0.0, mode_key: 9, timeout_s: 60.0}
  - {type: key, key: 5, duration_s: 0.5}
  # 桥 A：以桥 A 模式行走到实际出口
  - {type: waypoint, x: 2.0, y: 0.0, yaw: 0.0, mode_key: 12, timeout_s: 60.0}
  - {type: key, key: 5, duration_s: 0.5}
  # 下一段普通路段
  - {type: waypoint, x: 3.0, y: 1.0, mode_key: 9, timeout_s: 60.0}
```

每个 waypoint 自动选择 mode_key，等反馈匹配后再运动；默认普通模式 9。偏角大先转向，朝向合适前进，到点发送 5。yaw 省略时不要求终点朝向。左右键依照固件 1/3，首次实测其方向与定位坐标正方向是否一致。

原 `hurricane_route.yaml` / `calibrated_skills.yaml` **不直接作为 plan_file 加载**。普通操作可在场地窗口导入旧 field 采点并自动生成任务；下方手写步骤保留用于高级调试。采点流程不用重做，除非地图或参考点改变。

跟随器不规划避障，也不发 Nav2 目标。通过中间航点限定实际路线。调 `cybergear.yaml` 的 distance_tolerance（米）、heading_tolerance（弧度）、key_interval（秒）改善到点和转向切换；调整后重启。

## 8. 停止、故障和再次运行

```bash
ros2 service call /cybergear/stop std_srvs/srv/Trigger '{}'
```

该命令中止任务并尝试发 5；遥控器切回手动关闭自动。新鲜状态丢失、故障、定位缺失或步骤超时都会结束当前任务，不缓存续跑。故障排除后 start 从头执行；想从中间调试，复制任务文件，只保留要测试的步骤。

固件仍关闭上位机掉线 watchdog，且忙碌的一次性动作可能忽略 5。上位机的“停止发送”不等于物理停止；按现场既有急停方案处理。回包没有动作完成、停稳、busy 字段，本入口不伪造这些反馈。

## 9. 快速排错

| 现象 | 处理 |
|---|---|
| Package/executable not found | 编译两个包，重新 source install/setup.bash |
| connected=true，但无 status | 遥控器打开自动；核对 RX/TX、波特率、固件与设备路径 |
| status.mode=z | 先完成机器人初始化 |
| ready=false、fault 非零 | 按 motor_id/detail 检查电控诊断 |
| key 被拒绝 | 看终端日志：只观察模式、任务运行中、等待模式反馈或状态过期 |
| step_timeout | 模式没匹配，或航点超时；核对按键表与坐标 |
| localization_unavailable | 检查 map→base_footprint 更新、ROS 时钟和 pose_timeout |
| 跳完提前发下一键 | 增大该 key 的 duration_s；固件忙碌时会忽略动作键 |
| 修改配置没变化 | 停止任务，重启新入口，确认 params_file/plan_file 是个人文件 |

本入口是独立的 CyberGear 现场联调任务执行器，不使用旧 Mission/v1 的安全状态服务。旧方案入口和配置保持可用，两套方案不要同时输出。
