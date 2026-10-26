# 旧 STM32 v1 / Mission 接入参考

仅适用于实现 v1 JSON 协议的固件，不适用于 2026-10-26 CyberGear 按键固件。新版按原[实机指南第 9 节](REAL_ROBOT_GUIDE.md#9-新版-cybergear从采点接到自动运行)执行。

## 9. 要让机器马真正自动走和越障，还需完成的接线与接口

以下是当前默认机载入口与整场自动运行之间的实际缺项，不能只靠填写地图解决。

| 需要交付的部分 | 必须有的真实数据 / 行为 | 当前操作判断 |
|---|---|---|
| 底层安全状态 | STM32 真实 STATUS 经已有 TelemetryAdapter 转成 `/motion/telemetry`，由仲裁器提供 `/motion/get_state` | 主机适配已实现，固件实测证据待接入 |
| 导航命令链 | 已有 NavVelocityAdapter 将速度绑定 `mission_id/generation`，由唯一仲裁器输出 `/stm32/arbiter_command` 给网关 | 独立 STM32 启动入口提供，默认禁用运动；需标定与联调 |
| Hurricane 执行端与反馈 | 电控实现新协议，提供匹配 request/session/step 的真实 STATUS；网关统一接入 | 串口写成功不是动作完成 |
| 统一控制权 | Nav2 与串口动作实际受同一仲裁链路控制；故障、取消、超时有真实安全收尾 | 不能仅改变 ROS owner 字段 |
| 实机启动检查配置 | 正确的唯一输出节点、真实低层状态、定位、技能、路线都通过 preflight | 默认机载 preflight 仍按“电机输出关闭”配置 |

先用下面命令与电控共同核对已接入的端点：

```bash
ros2 topic info /motion/telemetry --verbose
ros2 service type /motion/get_state
ros2 service type /motion/set_control_mode
ros2 topic info /motion/control_state --verbose
ros2 topic info /motion/authority --verbose
ros2 topic info /stm32/status --verbose
ros2 topic info /stm32/arbiter_command --verbose
```

有这些名字不等于完成接入；还须核对数据新鲜度、来源、会话和实际物理状态。精确契约见 [CONTROL_INTEGRATION.md](CONTROL_INTEGRATION.md)。

Hurricane 独立预览默认 `dry_run:=true`。真实执行改用唯一 STM32 网关，不再发送旧 `S编号E` 串口帧，也不接受旧 feedback topic 作为实机完成证据。请按 STM32 接入指南完成协议、真实状态和仲裁联调。只读采点不需要打开串口。

接入完成后的现场顺序应为：底层与急停可用 → 唯一仲裁器及真实状态 → 雷达/定位 → 已有 NavVelocityAdapter 与网关联调 → 已标定的真实技能服务 → preflight 成功 → Mission 手动启动。先逐项完成“普通点、单个技能、单障碍完整链”验收，再自动整场。

实机 Mission 必须显式使用 `simulation_mode:=false`、`use_sim_time:=false`、`autostart:=false`。连接本仓库 `/localization/evidence` 时还必须使用 **`accepted_pose_type:=evidence`**；默认 `covariance` 不能读取这个消息类型。不得省略实机 preflight 服务来绕过启动检查。

**本仓库尚未提供包含这些生产接口的一键全场 bringup。** 因此本指南不把独立启动 `mission_manager` 或把 `autostart` 改成 true 当作已经能实机跑赛。完整接口接通后，`/mission/start` 的标准调用为：

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

成功执行链应是：导航到入口 → 精确接近 → 连续停稳与入口验收 → 获取技能控制权 → 越障 → 独立确认安全结束 → 获取本阶段新定位 → 独立出口验收 → 恢复导航 → 下一障碍 → 返回点验收。
