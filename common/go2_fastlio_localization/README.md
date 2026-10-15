# FAST-LIO 公共工具

机载入口见 [go2_onboard_bringup](../../real/go2_onboard_bringup/README.md)，部署说明见 [根 README](../../README.md)。实机默认全局定位使用 `go2_real_localization` 原生 GICP，机体状态使用 `go2_mission_backends/robot_state_adapter`。

- `fastlio.launch.py`：只启动 FAST-LIO，调用方显式传入参数文件和时钟。
- `save_live_map`：从 `/Laser_map` 保存累计注册点云，默认目录 `~/go2_maps`，可用 `output_dir` 改为实际地图目录。
- `pcd_to_nav2_map`：按高度将 PCD 投影为二维栅格，只是地图初稿，需检查实际可行驶区域。
- `check_stationary`：辅助观测位姿静止，不提供真实安全支撑判据。

Python `pcd_icp_localizer`、`fast_lio_odom_bridge` 与有效的 `icp.launch.py` 保留供算法检查，新的机载入口不启动它们。没有真实使用方的旧兼容 launch 已移除；实机使用 `go2_real_bringup` 的雷达/里程计入口。不要与原生 GICP/robot_state_adapter 同时启动造成重复 `map→odom`、`/odom` 或机体 TF。

实际配置由 `configure_robot` 生成到 `~/.config/rc2026`。工具命令及地图原点处理见 [DEPLOYMENT.md](../../docs/DEPLOYMENT.md)。定位 RMSE 和固定 covariance 都不能单独证明真实位置精度。
