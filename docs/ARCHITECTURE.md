# 机载运行结构

> STM32 自制机器人的新通信链路见 [STM32 接入指南](STM32_BRIDGE_GUIDE.md)：禁止横移、唯一 UART 网关、带租约的仲裁后输出。协议仍待 C 端实现，默认关闭运动。


## 数据与坐标链

```text
MID-360 driver → /livox/lidar + /livox/imu
                         ↓
                    FAST-LIO
               /Odometry（IMU位姿）
               /cloud_registered_body
                    ↓        ↓
robot_state_adapter     GICP + 实际PCD地图
       ↓                 ↓              ↓
/odom（机体速度）     map→odom      /gicp/evidence
       ↓                               ↓
map→nav_odom→base_footprint→base_link  localization_adapter
       ↓                               ↓
       Nav2                       /localization/evidence
       ↓
/onboard/nav_cmd_internal → velocity_smoother → /onboard/nav_cmd_raw
```

FAST-LIO 的 `odom→livox_frame` 分支保持独立；`robot_state_adapter` 根据测量的 `T_base_link_IMU` 生成导航分支。不能再用另一组 static TF 给 `livox_frame` 添加父节点。`/odom` 的速度由连续测量位姿估计；其协方差和延迟仍需实机验证。

GICP 发布原子证据，包含 accepted、测量时间、源身份、epoch / sequence 和质量量；拒绝更新不会刷新 accepted。定位适配器验证顺序与新鲜度后提供 `/mission/localization_ok`。RMSE、inlier ratio 是点云匹配质量，不能直接等同全局定位厘米精度。

## 启动模式

| 模式 | 节点范围 | 地图 | 运动 |
|---|---|---|---|
| lidar | driver、doctor | 不需要 | 关闭 |
| odometry | 上述 + FAST-LIO、robot_state_adapter | 不需要 | 关闭 |
| localization | 上述 + GICP、localization_adapter、preflight | 实际 PCD、固定起点 | 关闭 |
| navigation | 上述 + scan 诊断、map_server、Nav2 lifecycle | PCD + 同坐标系二维图 + 测量 footprint | 私有 topic，关闭机器人输出 |

机载入口不启动 AMCL，`map→odom` 只有 GICP 提供。Nav2 使用 `nav_odom`，避免更改原始 LIO 链。扫描 `/cloud_registered_body→/onboard/scan_diagnostic` 在当前版本只供诊断。Nav2 costmap 只启用 static / inflation 层。

## Mission 后端

包 `go2_mission_backends` 含状态适配、定位证据适配、控制权仲裁、越障 SPI 和 preflight；Mission FSM 在 `robocon_mission`。软件测试包含导航、停稳、精确到点验收、越障、出口独立定位验收、继续导航及返回，并覆盖异常状态流转。

`real.launch.py` 只启动传感器、定位与隔离的规划诊断，不启动整场任务。独立 `stm32_bringup.launch.py` 已提供 `NavVelocityAdapter → MotionArbiter → SerialGateway` 的带租约输出链，以及 `STATUS → TelemetryAdapter → /motion/telemetry` 的反馈适配；默认不开串口、不允许运动，可显式配置真实 Hurricane 服务。STM32 固件协议、真实停稳/支撑反馈、技能能力与现场标定仍待接入和验收。Mission 需要另外配置预检查并手动启动，详见 [STM32 接入指南](STM32_BRIDGE_GUIDE.md)。测试 backend_sim 仅用于软件验证。

```text
/onboard/nav_cmd_raw → NavVelocityAdapter → /motion/nav_command
Hurricane → /motion/skill_command → MotionArbiter
MotionArbiter → /stm32/arbiter_command → SerialGateway → STM32
Hurricane → /stm32/skill_request → SerialGateway
STM32 STATUS → SerialGateway → TelemetryAdapter → /motion/telemetry
```

`/stm32/arbiter_command` 是带 owner/mission_id/generation 的最终 MotionCommand；普通 Twist 诊断输出不作为 UART 输入。定位、Nav2、通信和 Mission 是独立启动范围，不能把诊断入口的运动禁用状态理解为主机适配器不存在。

关键源码：

- `real/go2_onboard_bringup/go2_onboard_bringup/launching.py`：机载组合及私有速度 remap。
- `real/go2_onboard_bringup/go2_onboard_bringup/profile.py`：配置生成及启动前检查。
- `common/go2_mission_backends/go2_mission_backends/`：适配器、仲裁、技能 SPI、preflight。
- `real/go2_real_localization/src/gicp_localizer.cpp`：原生 GICP 及质量门控。
- `common/robocon_mission/`：路线 schema、Mission FSM、客户端和软件 E2E。
- `common/go2_nav_bringup/behavior_trees/`、`src/`：巡航/精确接近 BT 与 critic。
