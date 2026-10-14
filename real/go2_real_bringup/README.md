# 真实传感器基础入口

推荐使用经过配置检查的 [go2_onboard_bringup](../go2_onboard_bringup/README.md)，完整安装见 [根 README](../../README.md)。本包是它的传感器组合层，不提供整场 Mission 或机器人运动输出。

- `lidar.launch.py`：MID-360 驱动，真实 `/livox/lidar` CustomMsg 和 `/livox/imu`。
- `odometry.launch.py`：FAST-LIO；雷达驱动须已启动，`lio_params` 选择生成的实机配置。
- `localization.launch.py`：真实驱动、FAST-LIO、GICP；必须传实际 `map_path`，同时传 `user_config_path`、`lio_params` 和 `params_file`。
- `mapping.launch.py`：较低层兼容组合入口；新部署优先采用机载 `mode:=odometry`，避免直接使用未测量的默认外参。

包内配置是算法候选模板，雷达接收 IP 与机体外参由机载配置向导显式填写。地图不会从原仿真目录加载，运行数据保持在用户目录。默认时钟为真实时间，机载入口显式关闭 RViz。
