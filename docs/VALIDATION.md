# 发布前软件验证，2026-10-14

此页保留 2026-10-14 发布前的历史软件验证记录，不代表当前分支的完整验收结果。最新 STM32 接入验证及未通过项见 [STM32_BRIDGE_VALIDATION.md](STM32_BRIDGE_VALIDATION.md)。`rely-only` 清理后的测试迁移及新结果见 [RELY_ONLY_CLEANUP_REPORT.md](RELY_ONLY_CLEANUP_REPORT.md)；当前验证脚本已不再忽略历史 launch 测试文件。

## 实际执行结果

| 检查 | 结果 | 范围 |
|---|---|---|
| 固定源码独立编译 | 11 个 ROS 包通过；7 分 39 秒 | 真实 driver、FAST-LIO、CPU fast_gicp、项目 8 包；SDK 另行编译 |
| 主软件回归 | **409 passed，1 deselected**；323.43 秒 | Mission、后端、原生 GICP、33 个配置测试、15 个安装器契约测试 |
| 导航 CTest | **6 / 6 通过**；1.91 秒 | 精确姿态 critic、旋转恢复、重规划 BT、Nav2 client、地图激活、插件环境 |
| 独立进程闭环 | **通过**；25.37 秒 | 两个不同障碍、返回点、最终零速、healthy SAFE_HOLD，7 个子进程全部回收 |
| 实际 Nav2 图 | **1 passed**；4.70 秒 | 真实 Nav2 lifecycle ACTIVE、非零私有速度、平滑器发布来源、无 `/cmd_vel` 发布者、目标取消 |
| 常驻 doctor | **1 passed**（0.18 秒）；与配置合计 34 项通过（1.84 秒） | 收到测试遥测也不会获得运动权限；不使用真实低层 |

测试环境为 WSL Ubuntu 22.04 / ROS 2 Humble，`amd64`。这是软件编译/接口验收，不是机载硬件或比赛完成验收。未测试 ARM 小电脑。

## 验证来源

构建在独立工作空间进行，清除旧 overlay，7 个关键 package prefix 均检查属于新的 install。原工程未修改。Livox SDK 安装到该工作空间私有前缀，driver 链接对应 SDK，CPU fast_gicp 不使用 CUDA。

本机直连 GitHub Git 端口不可用，因此本次干净构建使用官方 `codeload.github.com` 的固定 commit archive；版本与仓库 `dependencies.repos` 一致，按安装器相同补丁/ROSIDL 修复构建。没有使用旧 workspace vendor 源码或安装库。部署安装器的真实 apt + Git 全新冷安装未在本机完整执行；其 dry-run、15 个离线契约测试与相同依赖的干净源码构建已执行。GitHub Actions 提供手动验证 workflow，本次发布前未执行远端 workflow。

原生 GICP 测试启动真实 C++ 程序并发送合成点云/odom：要求多次递增的 accepted 测量时间；显式初猜后要求新 epoch 的 accepted。它验证程序和证据链，不能证明真实地图的定位精度。

主回归故意排除 5 个依赖原 Gazebo 工程的启动测试，保留巡航路线 fixture 和 Mission 软件闭环。默认额外 wall-clock 正向完整闭环用例为 1 deselected；受控 `/clock` 成功路径、时间暂停/回退与证据失效检查仍运行。稳定时钟环境可使用 `--include-wall-clock` 追加。

## 发布中修复并重测的问题

1. 初轮 Python 环境受用户 `~/.local` SciPy 覆盖影响。工具现显式设置 `PYTHONNOUSERSITE=1`；完整成绩来自系统 NumPy 1.21.5 / SciPy 1.8.0 的新运行。初轮中断日志另存，没有计作完成验收。
2. 实际 Nav2 多进程图超过 Cyclone 默认自动 participant index 范围。测试 profile 增加 `MaxAutoParticipantIndex=64` 后严格重测通过，未放宽生命周期/输出隔离/取消断言。配置仅影响显式选择它的进程。
3. 新机载入口直接使用 Nav2 navigation launch，避开旧入口对 Python ICP 话题的等待；由 GICP 提供唯一全局定位，并在运行时设置项目巡航 BT。实际图测试验证控制器/行为节点的速度均保持私有。

## 复现命令

```bash
sudo apt-get install -y python3-pytest ros-humble-ament-cmake-pytest
bash tools/verify_real.bash --workspace "$HOME/rc2026_ws"
source /opt/ros/humble/setup.bash
source "$HOME/rc2026_ws/install/local_setup.bash"
export PYTHONNOUSERSITE=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=169 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$PWD/common/go2_mission_backends/config/cyclonedds_sim.xml"
python3 -m pytest -q real/go2_onboard_bringup/test/test_doctor_ros.py real/go2_onboard_bringup/test/test_planning_graph.py
```

工具的报告默认保存在 workspace/log/verification，可指定 `--report-dir`。本次本地报告位置是独立 workspace 的 `log/verification/release` 与 `log/verification/onboard_graph`；运行日志不发布为源码。

## 仍需真实设备验收

真实 MID-360 收包与时间戳、LiDAR/IMU 及机体外参、实际地图初猜与定位误差、足式机体尺寸和运动控制（当前 STM32 实机配置禁止横移）、动态障碍扫描层、实际制动距离、安全支撑、急停、控制权交接及真实越障 action 均未在设备上验收。本文所测 `real.launch.py` 诊断入口保持 `mission_allowed=false`、运动输出关闭。后续新增的 STM32 独立入口默认禁用运动，配置及其硬件验收边界见 [STM32 接入指南](STM32_BRIDGE_GUIDE.md)。
