# RC2026 机器马机载导航与 Mission 后端

> STM32 接入更新：自制机器人不支持横移。新增单串口网关默认禁用运动；Hurricane 实机改走共享网关。协议为待固件实现草案，见 [STM32 接入指南](docs/STM32_BRIDGE_GUIDE.md)。下文原有定位/规划诊断入口继续保留。


`rely-only` 面向团队自制 STM32 四足机器狗，不使用 Go2 官方运动 SDK。新增 USB-TTL 单串口协议草案与 ROS 接入，默认不开启运动输出；STM32 固件端仍需实现并联调。此前 Mock 清理记录见 [清理报告](docs/RELY_ONLY_CLEANUP_REPORT.md)。

面向 **Ubuntu 22.04 / ROS 2 Humble、MID-360 自带 IMU、仅支持前后和转向、禁止横移的 STM32 四足机器人**。

## 从这里开始实机操作

- **第一次部署：[实机操作指南](docs/REAL_ROBOT_GUIDE.md)**。按接线、网络、安装、雷达检查、建图、定位、导航诊断、现场采点的顺序执行；每步写明终端、完整命令与成功判断。
- **已经配置好、准备每天使用：[现场操作卡](docs/REAL_ROBOT_QUICK_REFERENCE.md)**。
- **运行卡住或看不懂报错：[实机排错表](docs/TROUBLESHOOTING.md)**。包括 `NOT_READY`、雷达不收数、定位拒绝、Nav2 不动、Tk 面板与串口等情况。

第一次安装可从机载 Ubuntu 终端执行：

```bash
sudo apt-get update
sudo apt-get install -y git
cd "$HOME"
git clone --branch rely-only --single-branch https://github.com/shyahngyoo/rc2026_deom.git
cd "$HOME/rc2026_deom"
bash tools/install_real.bash --jobs 2
```

尚未装 Humble 时加 `--install-ros`，内存较少时用 `--jobs 1`。已有仓库请按指南更新，不重复 clone。安装后的 workspace 默认是 `~/rc2026_ws`；真实配置、地图和场地采点放在用户目录，更新源码前备份。

## 当前哪些入口可以使用

| 现场目标 | 入口 | 当前行为 |
|---|---|---|
| 检查真实 MID-360 数据 | `real.launch.py mode:=lidar` | 驱动、IMU、诊断；需要真实网络配置 |
| 实际建图与局部里程计 | `mode:=odometry` | FAST-LIO、机体状态适配，支持保存现场 PCD |
| 实际场地全局定位 | `mode:=localization` | 加载实际 PCD 和 IMU 初猜，GICP 与定位证据 |
| Nav2 路径/控制诊断 | `mode:=navigation` | 真实地图与定位；速度仅在 `/onboard/nav_cmd_*` |
| 八障碍采点与技能导出 | `capture_panel`、`hurricane_capture` | 现场 26 点、路线与技能配置；面板需要图形会话 |
| 串口握手与状态诊断 | `stm32_bringup.launch.py port:=...` | 新协议草案，默认关闭运动输出，不启动 Mission |
| Hurricane 技能适配 | `hurricane.launch.py` | 独立预览默认 dry-run；实机使用共享 STM32 网关 |

机载 `real.launch.py` 属于 `go2_onboard_bringup`，始终需要先生成真实配置。具体可复制命令均在实机指南中。

**当前默认入口尚不能驱动机器马自动跑完整场障碍赛。** ROS 侧 Nav2 命令适配、统一仲裁与单串口链路已提供；真实停稳/支撑反馈、STM32 协议与现场标定仍需联调。`navigation` 不发布机器人 `/cmd_vel`，不启动整场 Mission。指南第 9 节列出接入清单、实际端点与验收顺序。

## 进一步阅读

| 文档 | 什么时候查 |
|---|---|
| [实机操作指南](docs/REAL_ROBOT_GUIDE.md) | 第一次安装到现场采点的完整操作 |
| [现场操作卡](docs/REAL_ROBOT_QUICK_REFERENCE.md) | 配置完成后的日常开机、检查和退出 |
| [实机排错表](docs/TROUBLESHOOTING.md) | 看到错误或数据不符合预期时 |
| [部署与建图细节](docs/DEPLOYMENT.md) | 地图、网络、DDS、工作空间的补充说明 |
| [控制与底层接入](docs/CONTROL_INTEGRATION.md) | 电控、导航与技能团队对接真实接口 |
| [Hurricane 移植与协议](common/robocon_mission/HURRICANE_PORT.md) | 串口帧、八类技能、采点和反馈契约 |
| [系统结构](docs/ARCHITECTURE.md) | 定位、TF、状态适配和后端模块关系 |
| [Mission](common/robocon_mission/README.md) / [导航 BT](common/go2_nav_bringup/NAV2_BT.md) | 路线、任务状态流转、精确接近与恢复 |
| [软件验证记录](docs/VALIDATION.md) | 已执行检查及其边界；不替代实机验收 |

软件回归按验证记录执行。源码固定依赖见 [dependencies.repos](dependencies.repos) 与 [dependencies.lock.yaml](dependencies.lock.yaml)，版权说明见 [NOTICE.md](NOTICE.md)。
