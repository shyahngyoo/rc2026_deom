# CyberGear 现场上手：选障碍、采点、点开始

适用新版 CyberGear / 2.4G 按键固件。**现在不用手抄坐标或从零写任务：面板内置 Hurricane 八类障碍模板，采点后生成当前障碍或整条路线。** 不修改 STM32，原 v1 入口保留。

## 0. 命令在哪里执行，按钮在哪里点

**下面命令全部在机载 Ubuntu 的终端执行，不是在 Windows PowerShell 执行。** 用 ToDesk 打开机载电脑桌面即可。正常使用只需一个终端开面板；另开一个终端可以看状态或发停止命令。

- 标记 **终端命令** 的代码块：复制到 Ubuntu 终端执行。
- 标记 **面板操作** 的步骤：点击已经打开的窗口，不需要再输入另一条命令。
- 面板已经启动某个功能时，不要再手动执行相同功能的启动命令。

## 1. 安装更新后，只开一个面板

先在机载电脑拉取 rely-only，再编译（工作区应已链接这个仓库）：

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
ros2 run go2_onboard_bringup operator_panel
```

**以后每次开机，终端 A 完整执行这三行即可打开面板：**

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
ros2 run go2_onboard_bringup operator_panel
```

这个终端保持运行。后面的建图、采点、任务生成和启动按钮都在面板里操作。ToDesk 桌面可用；缺 Tk 安装 `sudo apt-get install python3-tk`。先关闭其他终端正在运行的雷达、定位和串口实例。已有 IP、IMU 外参、footprint 保留；面板自动补齐缺少的 CyberGear 默认配置，不覆盖已调文件。

## 2. 先选择定位方式（面板操作，无需另外输入启动命令）

| 现场情况 | 点哪个按钮 | 要注意什么 |
|---|---|---|
| 赶时间，现场重新采点 | **快速里程计（无需地图）** | 无需保存 PCD；本次里程计不中断即可采点和运行；重启后需重采 |
| 已有 PCD，需要复用地图点 | 选 PCD，填当前 IMU 初始位姿，点 **③ 定位与采点** | 地图和初始位姿按原项目配置；在 RViz 核对定位 |
| 还想制作可复用地图 | **① 开始建图 → ② 保存地图 → ③ 定位与采点** | 建图时用遥控器走场，保存后 PCD 路径自动填入；初始位姿需按实际确认 |

快速模式使用 `nav_odom→base_footprint`，地图模式使用 `map→base_footprint`。不需要 Nav2 二维地图，不运行 Nav2 控制。快速模式没有历史地图重定位，也没有旧 Hurricane 的两点场地轴转换；每个障碍通过实测原点和方向建立独立局部坐标。主面板显示当前位置与朝向。

## 3. 创建本场障碍列表（面板操作）

点 **场地 / 障碍编排与采点**。第一次可直接添加；更换场地或坐标模式时点“新场地”，选择独立文件名。

- 下拉选择木桥 A、木桥 B、大斜坡、限高栏、沙坑、绕杆、高墙、台阶，点“添加障碍”。
- 重命名，例如“台阶1”“台阶2”；同类障碍可出现多次，参数各自独立。
- 用“上移 / 下移”设置比赛顺序。“启用/跳过”控制整条路线包含哪些障碍。
- 每个障碍的动作模板已经填好。无需复制另一障碍坐标，不依赖场上一定有桥 B。

**桥 A/B 的旧名字有交叉：** Hurricane 的 bridge_a 实际包含桥 B 模式与多次跳跃，bridge_b 实际使用桥 A 模式。面板保留实际流程并显示说明，选择时以模板动作和 STM 电控模式为准，不能只凭“A/B”名字选。

## 4. 每个障碍采三个点（面板操作）

遥控器手动移动机器人，选中对应障碍，依次点击：

1. **采原点＋模板 X 方向**：把机器人放到障碍模板的参考原点，机头沿模板局部 +X 方向，再点击。记录位置和方向；它不一定等于入口，也不一定沿越障方向。例如模板沿负 X 行走时，+X 朝向应与行走方向相反。
2. **采入口＋朝向**：站在希望自动接近并停下的位置，朝向实际进入障碍的方向。
3. **采出口＋朝向**：站在越障后的目标位置，朝向下一阶段需要的方向。

下方显示已采点，点击会更新并保存。绕路时，按实际经过顺序采“添加入口前过渡点”；这些点在该障碍入口前执行。程序不自动避障。

地图模式已有原 `field.yaml` 时，点“导入旧 field 采点”，输入标签前缀如 `bridge_a`。程序读取 begin/entry/exit，并由旧 start/finish 推导模板 +X 轴；不用手抄坐标。

不要求全场 26 点。测试一个障碍，只需该实例的三个点。快模式里程计重启后，旧点失效，面板会要求清空重采；地图模式使用相同地图时可保存复用。

## 5. 接串口，先调单动作

USB-TTL TX→PB11，RX→PB10，GND 共地，3.3 V TTL。

**终端 C 查实际串口：**

```bash
ls -l /dev/serial/by-id/
```

把输出中的完整设备路径填入主面板“串口设备”栏（如没有 by-id，再检查 `/dev/ttyUSB*` 或 `/dev/ttyACM*`）。**面板操作：**点“连接串口单动作调试”。不需要同时输入 ros2 launch。

**终端 C 查看详细回包（可选）：**

```bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/setup.bash
ros2 topic echo /cybergear/status
```

查看结束按 Ctrl+C，只关闭这个观察命令，不影响面板。

完成机器人初始化，遥控器由手动拨到自动；看到就绪为 true，选择模式键并点“发送模式键”，等回包模式变化后，点击前进、左右转、停止。动作6/7/8按电控的当前模式解释，每点一次只发送一次。

模式键：普通9、小坡10、桥B11、桥A12、大坡13、爬行14、沙坑15、绕杆16、高墙17、台阶18。上位机不能代替遥控器开启自动。

## 6. 跑一个障碍，再跑整条路线（面板操作）

场地窗口：

1. 选中障碍，点 **生成并载入当前障碍**。
2. 等主面板串口就绪、定位正常，点 **开始 / 从头重跑**。
3. 程序依次执行：入口前过渡点 → 入口 → 内置障碍步骤 → 出口 → 停止。
4. 用“停止任务”结束。修改参数后重新生成并载入，再从头运行。
5. 单障碍调好后，按顺序启用所需障碍，点 **生成并载入启用路线**，再点开始。

载入不自动发起任务，开始才运行。载入时保持已有定位/里程计不重启；无需每个动作人工确认。面板显示当前障碍、动作、步骤编号、底层模式与故障。单障碍会先走向采集的入口，不是跳过接近阶段直接触发跳跃。

## 7. 现场只需调这些

| 调什么 | 在哪里调 |
|---|---|
| 本场位置、方向、入口出口、连接路径 | 采点按钮、入口前过渡点 |
| 模板整体长度/宽度 | X距离倍率、Y距离倍率，默认1 |
| 一次性动作等待 | 动作等待倍率，默认1；或模板中的 duration_s |
| 单个起跳/转弯点、动作顺序 | 下方已填好的模板步骤：target 为局部 x/y（米）和角度（度）；null 保持进入步骤时对应分量 |
| 到点容差、朝向容差 | `~/.config/rc2026/cybergear.yaml`，修改后重新载入任务 |

模板来自 Hurricane 的既有流程，默认尺寸和时长是调试起点，不是你们赛场的实测值。不同台阶数量需增删对应模板步骤；倍率不能自动推断级数。底层腿部轨迹、力度、速度仍由电控组调整。

跳跃只触发一次，等待 duration_s 后推进；回包没有动作完成或安全站稳标志。固件忙碌时可能忽略停止键，继续使用现场已有急停手段。

## 8. 开始、停止和检查的终端命令

面板按钮和下面命令是**两种操作方式，任选一种，不必各做一次**。终端 C 每次新开先执行两行 source。需要看状态时，先结束正在运行的 echo，再在同一终端执行下一条。

**开始 / 从第 0 步重跑**（先在面板生成并载入任务）：

```bash
ros2 service call /cybergear/start std_srvs/srv/Trigger '{}'
```

返回 `success: true` 才是接受启动；失败原因在 message 中。它不是一键安装或启动所有节点的命令。

**停止当前任务并尝试发送停止键：**

```bash
ros2 service call /cybergear/stop std_srvs/srv/Trigger '{}'
```

**查看运行状态：**

```bash
ros2 topic echo /cybergear/status
```

重点看 `ready`、`obstacle`、`action`、`step`、`task`。`completed` 表示执行器步骤已结束，不是 STM32 动作完成 ACK。

**检查快速里程计位置：**

```bash
ros2 run tf2_ros tf2_echo nav_odom base_footprint
```

**检查地图定位位置：**

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

只执行与你当前模式对应的一条。关闭面板自己的整套程序请点击“停止本面板全部程序”，无需按名称批量杀进程。

## 9. 不使用面板时的手动命令（备用，不与面板重复启动）

以下 A、B、C 均先加载两行环境。只有想单独排查节点时才使用；**选择障碍、采点和生成模板任务仍推荐面板，当前没有独立的场地编辑 CLI。**

### A：只启动建图 / 实时里程计

```bash
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=odometry
```

B 显示地图：

```bash
rviz2 -d "$(ros2 pkg prefix go2_real_bringup)/share/go2_real_bringup/config/real_mapping.rviz"
```

C 在雷达继续运行时保存地图：

```bash
ros2 run go2_fastlio_localization save_live_map \
  --ros-args -p output_dir:="$HOME/rc2026_maps" -p timeout_sec:=45.0
```

成功会打印 `SAVED 实际路径`。**手动保存不自动更新定位配置**；自动填路径和备份配置是面板提供的功能。用面板保存和切换最省事。

### A：只启动地图定位（与 odometry 二选一）

先退出上面的 odometry，再执行；地图和初始位姿必须已经通过面板或原指南配置好：

```bash
ros2 launch go2_onboard_bringup real.launch.py \
  config_dir:="$HOME/.config/rc2026" mode:=localization
```

### B：只启动新版串口单动作联调

把 `/dev/ttyUSB0` 换成实际设备：

```bash
ros2 launch go2_onboard_bringup cybergear_auto.launch.py \
  port:=/dev/ttyUSB0 enable_motion:=true
```

C 先选普通模式，等待状态回传 `mode: a` 且 `pending_mode: null`：

```bash
ros2 topic pub --once /cybergear/key std_msgs/msg/String '{data: "9"}'
```

C 发送一次前进（会持续运动，随后需停止）：

```bash
ros2 topic pub --once /cybergear/key std_msgs/msg/String '{data: "2"}'
```

C 停止：

```bash
ros2 service call /cybergear/stop std_srvs/srv/Trigger '{}'
```

### B：运行已经生成的地图坐标任务

退出 B 的单动作实例后执行。文件需已在场地面板生成，A 的地图定位保持运行：

```bash
ros2 launch go2_onboard_bringup cybergear_auto.launch.py \
  port:=/dev/ttyUSB0 enable_motion:=true \
  params_file:="$HOME/.config/rc2026/cybergear.yaml" \
  plan_file:="$HOME/.config/rc2026/cybergear_selected.yaml"
```

整条路线把 `cybergear_selected.yaml` 换成 `cybergear_route.yaml`。C 再执行第 8 节 start 命令开始。

**快速里程计生成的任务请继续在原面板会话中载入运行**，不要关闭里程计后照抄此命令复用旧坐标；面板负责检查采点会话是否一致。

## 文件与后续工作

- 场地文件：默认 `~/.config/rc2026/cybergear_course.yaml`，保存障碍顺序、各自采点和调好的模板；可另存多个场地。
- 自动生成：`cybergear_selected.yaml`、`cybergear_route.yaml`；普通使用不手动编辑。
- 地图与日志：`~/rc2026_maps/`、`~/rc2026_logs/`。

**你现场还要完成：**同步并编译代码 → 核对真实串口回包和左右转方向 → 选定位方式 → 为本场障碍采点 → 根据实际尺寸和动作时间调一个障碍 → 串场验证。程序和模板已经提供，未替你们实测步态、完成真实越障或验证全场成功率。

旧的手动任务命令仍在[通信参考](CYBERGEAR_AUTOMATION_GUIDE.md)，建图详细说明仍在[实机指南](REAL_ROBOT_GUIDE.md)。日常优先使用本页的面板流程。
