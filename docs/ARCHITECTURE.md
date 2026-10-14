# 机载运行结构

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

当前机载入口仅使用不涉及运动的适配器和 preflight。没有真实低层 `MotionTelemetry`、真实 `SkillExecutor` 与带 lease 的 Nav2 命令适配，所以不会启动整场任务。这与测试 backend_sim 的可运行闭环是不同入口；后者只用于软件验证。

关键源码：

- `real/go2_onboard_bringup/go2_onboard_bringup/launching.py`：机载组合及私有速度 remap。
- `real/go2_onboard_bringup/go2_onboard_bringup/profile.py`：配置生成及启动前检查。
- `common/go2_mission_backends/go2_mission_backends/`：适配器、仲裁、技能 SPI、preflight。
- `real/go2_real_localization/src/gicp_localizer.cpp`：原生 GICP 及质量门控。
- `common/robocon_mission/`：路线 schema、Mission FSM、客户端和软件 E2E。
- `common/go2_nav_bringup/behavior_trees/`、`src/`：巡航/精确接近 BT 与 critic。
