# RC2026 机器马机载导航与 Mission 后端

面向 **Ubuntu 22.04 / ROS 2 Humble、MID-360 自带 IMU、支持 `/cmd_vel` 与横移的机器马**。

## 从这里开始实机操作

- **第一次部署：[实机操作指南](docs/REAL_ROBOT_GUIDE.md)**。按接线、网络、安装、雷达检查、建图、定位、导航诊断、现场采点的顺序执行；每步写明终端、完整命令与成功判断。
- **已经配置好、准备每天使用：[现场操作卡](docs/REAL_ROBOT_QUICK_REFERENCE.md)**。
- **运行卡住或看不懂报错：[实机排错表](docs/TROUBLESHOOTING.md)**。包括 `NOT_READY`、雷达不收数、定位拒绝、Nav2 不动、Tk 面板与串口等情况。

第一次安装可从机载 Ubuntu 终端执行：

```bash
sudo apt-get update
sudo apt-get install -y git
cd "$HOME"
git clone https://github.com/shyahngyoo/rc2026_deom.git
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
| Hurricane 技能适配 | `hurricane.launch.py` | 独立串口扩展，默认 dry-run；实际控制需确认协议、反馈与仲裁 |

机载 `real.launch.py` 属于 `go2_onboard_bringup`，始终需要先生成真实配置。具体可复制命令均在实机指南中。

**当前默认入口尚不能驱动机器马自动跑完整场障碍赛。** 真实停稳/支撑反馈、生产 Nav2 命令适配和统一控制权链路还需接入；Hurricane 适配器的存在不等于这些接口已接通。`navigation` 不发布机器人 `/cmd_vel`，不启动整场 Mission。指南第 9 节列出接入清单、实际端点与验收顺序。

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
