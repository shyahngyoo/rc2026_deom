# 真实底层与越障接入契约

> STM32 自制机器人的新通信链路见 [STM32 接入指南](STM32_BRIDGE_GUIDE.md)：禁止横移、唯一 UART 网关、带租约的仲裁后输出。协议仍待 C 端实现，默认关闭运动。


主机已实现导航速度适配、MotionArbiter、单串口网关、真实 STATUS 适配和 Hurricane Action 运输层。尚待 STM32 实现 v1 协议、真实停稳/支撑/技能状态与执行能力，并完成现场标定。`real.launch.py` 保持规划诊断，独立 `stm32_bringup.launch.py` 默认禁用运动；以下说明当前接口和剩余固件责任。

## 1. 独立低层反馈

`go2_obstacle_interfaces/msg/MotionTelemetry.msg`：

| 字段 | 来源与要求 |
|---|---|
| header.stamp、source_id、sequence | 当前 STM32 适配路径保留新 STATUS 的主机接收时刻、固定源身份及递增适配序号；不伪造 MCU 采样时间 |
| known | 只有真实状态有效时为 true |
| stopped | 底层实际运动反馈，不是最后一条 `/cmd_vel` 为零 |
| safe_stance | 足/关节/支撑控制器提供的安全站姿；不能由雷达静止代替 |
| nav_active、skill_active、obstacle_id | 实际控制执行状态与当前技能身份 |
| control_releasable | 当前动作已经安全结束，允许交还控制权 |
| error_code | 真实故障与拒绝原因 |

当前 TelemetryAdapter 将网关收到的新鲜 STM32 STATUS 转换到 `/motion/telemetry`；其时间戳保留主机收到该次 STATUS 的时刻，不把缓存状态重新标记为新测量。STM32 必须提供真实底层证据，最终核对实机 `motion_arbiter` 参数。丢帧、时间过旧、时钟异常或 unknown 时必须停止并保留控制权。雷达位姿估计可辅助检查运动，不能填补安全支撑信号。

## 2. Nav2 命令入口与唯一输出

Nav2 平滑输出为 `/onboard/nav_cmd_raw`。已实现的 `quadruped_stm32_bridge/nav_velocity_adapter.py` 读取权威 lease，将新鲜速度包装为 `MotionCommand`：`header`、`owner`、`mission_id`、`generation`、`twist`，发布到 `/motion/nav_command`，交给 MotionArbiter。

适配器不能自行创造 generation、替 Mission 获取控制权，或在 lease 切换后重放旧速度。先启动时不绑定会话；授予 NAV 后只接受当次新命令；撤销、暂停、导航取消、定位失效或低层未知时清空缓存。

UART 只消费唯一 MotionArbiter 的 `/stm32/arbiter_command`（MotionCommand）；`/stm32/arbiter_cmd_vel` 只是 Twist 诊断输出。底层必须有本地 watchdog 和物理急停，DDS 发布者检查不能替代物理输出隔离。STM32 启动入口提供显式软件运动许可参数，默认禁用；这些开关不替代人工使能、真实状态和标定。

## 3. 越障执行器 SPI

公共动作 `ExecuteObstacle` 由 Mission 使用。当前 STM32 路径由 `hurricane_skill_server` 提供，经 GatewayTransport 请求唯一串口网关，并匹配真实 STATUS。通用 `obstacle_adapter` 仍提供 `SkillExecutor` SPI 供其他后端接入；使用 Hurricane 时无需再部署一个同名 Action server。通用 SPI 的字段、反馈、取消、幂等 identity 与安全释放流程见 [SKILL_EXECUTOR_ADAPTER.md](../common/go2_mission_backends/docs/SKILL_EXECUTOR_ADAPTER.md)。

实际执行器至少落实：按 obstacle_id / skill 类型分发、准备完成反馈、运行中反馈、成功/失败、取消及超时后安全停止。Action 成功不能单独等同允许交还控制权；必须等独立 telemetry 确认 stopped、safe_stance、control_releasable。

当前 TestExecutor 只允许软件测试模式，不能作为真实越障完成证据。

## 4. 路线与验收

建立实际赛场坐标下的入口、出口、普通点及返回点。Mission 的停稳后位姿验收和越障后的独立定位验收使用新鲜 accepted 证据；不能把同一条旧 TF 当作连续新样本。

完成接口后按以下顺序验收：

1. 零输出检查：仅启动底层和仲裁，unknown/掉线/急停都保持零及安全状态。
2. 有限速的普通点：执行 NAV 获取、暂停/恢复、取消、超时、精确接近、停稳、位姿验收。
3. 单个真实技能：控制权交接、成功、失败、取消、进程崩溃及独立安全释放。
4. 单障碍完整链：到入口→停稳→入口验收→越障→安全结束→新定位出口验收→恢复导航。
5. 全路线和比赛时间预算，记录实际误差、制动距离、数据新鲜度与故障收尾。

软件回归中已验证状态流转，实机需要验证物理事实及参数。固件接入后按 [STM32 指南](STM32_BRIDGE_GUIDE.md) 组合现有启动入口、配置 `/stm32/preflight` 并独立验收，不能依靠放宽 preflight 或把缺失状态填 true。线协议统一见 [STM32 Host UART v1](STM32_HOST_PROTOCOL_V1.md)。
