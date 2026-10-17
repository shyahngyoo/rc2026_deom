# quadruped_stm32_bridge

自制 STM32 四足机器人的单串口通信包。协议 v1 是需要双方确认并在 STM32 端实现的草案；不能直接兼容现有参考固件。

- `serial_gateway`：唯一 UART 所有者、帧解析、握手、ACK、重连和最后一道授权门控。
- `nav_velocity_adapter`：`/onboard/nav_cmd_raw` → 带有效租约的 `/motion/nav_command`。
- `telemetry_adapter`：真实新鲜 STATUS → `/motion/telemetry`；状态服务仍由 MotionArbiter 唯一提供。
- Hurricane 通过 ROS 请求共享网关；不再自行打开串口。

默认 `port=''`、`test_mode=true`、`enable_motion=false`、`calibration_confirmed=false`。默认 Launch 不启动 Hurricane、不启动 Mission、不使能电机。

构建、协议、USB-TTL 首次测试和待固件协作项见仓库 `docs/STM32_BRIDGE_GUIDE.md`。
