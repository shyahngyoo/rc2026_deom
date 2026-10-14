# 真实底层与越障接入契约

当前设备暂时没有停稳/安全支撑/技能结束 topic，越障 action 未接入。本仓库机载入口保持不输出运动；这里列出后续必须实现的生产接口。

## 1. 独立低层反馈

`go2_obstacle_interfaces/msg/MotionTelemetry.msg`：

| 字段 | 来源与要求 |
|---|---|
| header.stamp、source_id、sequence | 底层采样时刻、固定源身份、严格递增序号 |
| known | 只有真实状态有效时为 true |
| stopped | 底层实际运动反馈，不是最后一条 `/cmd_vel` 为零 |
| safe_stance | 足/关节/支撑控制器提供的安全站姿；不能由雷达静止代替 |
| nav_active、skill_active、obstacle_id | 实际控制执行状态与当前技能身份 |
| control_releasable | 当前动作已经安全结束，允许交还控制权 |
| error_code | 真实故障与拒绝原因 |

由适配节点将厂家已有状态转换成协议，独立采样；默认后端配置使用 `/motion/telemetry`，最终需核对实际 `motion_arbiter` 参数。丢帧、时间过旧、时钟异常或 unknown 时必须停止并保留控制权。雷达位姿估计可辅助检查运动，不能填补安全支撑信号。

## 2. Nav2 命令入口与唯一输出

真实 Nav2 当前只产生 `/onboard/nav_cmd_raw`。需要编写生产适配器读取**权威 lease**，将每条速度包装为 `MotionCommand`：`header`、`owner`、`mission_id`、`generation`、`twist`，交给 `motion_arbiter` 的 NAV 输入。

适配器不能自行创造 generation、替 Mission 获取控制权，或在 lease 切换后重放旧速度。先启动时不绑定会话；授予 NAV 后只接受当次新命令；撤销、暂停、导航取消、定位失效或低层未知时清空缓存。

最终 `/cmd_vel` 只允许一个仲裁器发布，机器人底层要有自身超时停机和急停。DDS 中其他发布者无法由此软件“拦截”，因此必须同时检查 topic 图和物理接线。机载发布入口没有生产适配器或打开电机的参数。

## 3. 越障执行器 SPI

公共动作 `ExecuteObstacle` 由 Mission 使用；后端通过 `SkillExecutor` SPI 连接实际控制器。完整字段、反馈、取消、幂等 identity 与安全释放流程见 [SKILL_EXECUTOR_ADAPTER.md](../common/go2_mission_backends/docs/SKILL_EXECUTOR_ADAPTER.md)。

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

软件回归中已验证状态流转，实机需要验证物理事实及参数。完成上述接入需修改生产入口并独立验收，不能依靠放宽 preflight 或把缺失状态填 true。
