# 原子定位证据与 Mission 适配

## 消息与发布者

`go2_obstacle_interfaces/msg/LocalizationEvidence` 将一次匹配的 verdict 和位姿放在同一个消息中：`header`, `source_id`, `epoch`, `sequence`, `accepted`, `covariance_known`, `pose`, `body_frame`, `reason`。

- `source_id` 标识一次发布进程，真实 GICP 启动时生成随机身份。
- 同一 epoch 内 sequence 必须严格递增；更高 epoch 可以重置 sequence。真实 GICP 本身全进程递增 sequence。
- GICP 接受匹配后先更新 map→odom、accepted measurement stamp 和 pose，随后发布 `/gicp/evidence`。accepted stamp 是同步传感器测量时间，不能用定时器或 TF 刷新时间替代。
- rejected 事件使用事件时间并标记 `accepted=false`，不刷新 accepted 位姿。新初猜、输入时间回退、ROS 时钟回退会增加 epoch。
- GICP 不提供经过标定的位姿协方差，`covariance_known=false`；消息中的零数组不可解读为零误差。
- 保留 `/gicp/pose`、`/gicp/valid`、`/gicp/status`，用于兼容和观测。适配器完全不通过 headerless Bool 与独立 Pose 配对构造可信证据。

## 适配节点

`localization_adapter` 默认输入 `/gicp/evidence`，输出 `/localization/evidence`（包含 accepted/rejected 事件），并提供 Trigger `/mission/localization_ok`。配置见 `config/localization.yaml`。

校验 frame/body frame、有限位姿和协方差、单位四元数、测量时间、source、epoch、sequence；墙钟与 ROS 时间均检查有效年龄。`use_sim_time=true` 时，重复但不前进的 `/clock` 包不能刷新时钟健康。失效检查定时器采用 STEADY_TIME，暂停仿真也会失效。

首个 source 会锁定，也可以预设 source_id。来源切换不是静默恢复：需要在停止任务后显式重启 adapter 与 Mission 或配置确定的新 source；迟到旧进程消息会报告 SOURCE_CONFLICT 并禁止推进。

任何拒绝、乱序、过期或来源冲突都会清除 accepted 缓存。恢复必须收到后续新的 accepted sequence，其测量时间不早于故障时间边界。已处于失效状态时，迟到旧序列/旧 epoch/故障前测量被拒绝但不持续推进恢复边界，避免传输延迟使正确的新测量永远追不上边界；新的真实拒绝和来源故障仍重新建立边界。时钟回退还需要更高 epoch，拒绝迟到旧 epoch。

## Mission 兼容接口

默认保留旧模式 `accepted_pose_type:=covariance`。新链路应设置：

```text
accepted_pose_topic:=/localization/evidence
accepted_pose_type:=evidence
accepted_pose_body_frame:=livox_frame
accepted_pose_source_id:=""
```

每次现有 Mission 调用 `require_new_accepted_pose()` 时，新模式记录 ROS 时间和 source/epoch/sequence 边界；只有测量时间位于当前阶段且身份序列更新的 accepted 才能通过。故障期间旧证据被丢弃，健康服务失败不能使旧证据在恢复后重新有效。不改变 Mission FSM。

单独启动此 adapter 前应停止旧 `localization_health`/mock Trigger 服务，不能让多个不同后端共同响应同一个健康服务名。

## 便携验证

先按仓库根 README 安装独立工作空间，再执行：

~~~bash
cd ~/rc2026_ws
bash src/rc2026_deom/tools/verify_real.bash
~~~

源码不在 WORKSPACE/src 时传 --workspace。验证工具使用 /opt/ros/humble 和当前 install/local_setup，检查 ament 中全部运行包来自当前 install，不引用开发者旧安装。native 测试通过 get_package_prefix("go2_real_localization") 查找真实 GICP 和 BUILD_TESTING=ON 的 synthetic_gicp_inputs。

只运行定位模块时，可用安装包 API 获取验证配置：

~~~bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/local_setup.bash
cd ~/rc2026_ws/src/rc2026_deom
export ROS_DOMAIN_ID=154 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$(/usr/bin/python3 -c 'from pathlib import Path; from ament_index_python.packages import get_package_share_directory; print((Path(get_package_share_directory("go2_mission_backends"))/"config/cyclonedds_sim.xml").as_uri())')"
/usr/bin/python3 -m pytest -q common/go2_mission_backends/test/test_localization_core.py \
  common/go2_mission_backends/test/test_localization_nodes.py
~~~

### 真实 GICP 测试边界

独立 C++ 输入进程读取临时生成的1800点 PCD，以已知 map→body X=1 m 表达原始点云，发布相同原测量时间的完整 PointCloud2 与 Odometry（odom→body X=0.3 m）。独立真实 gicp_localizer 运行原质量门控和 ExactTime，Python 仅观察输入 Odom stamp、input_status 和原子 evidence。

原10 s预算内必须至少3次 accepted，测量时间不同且严格递增，rawcloud/odom/pair计数均至少3；检查位姿、原测量时间和未知协方差。新 initialpose 后原5 s内必须收到更高 epoch、且测量时间晚于 initialpose 的 accepted。测试不合成 accepted、不减点云数据、不修改质量标准或强行配对。

只读 /gicp/input_status 包含 cloud_count、odom_count、pair_count、cloud_stamp_ns、odom_stamp_ns；原始回调只记录到达与时间，不修改传给 ExactTime 的消息。它可区分未到达、未配对和匹配质量拒绝。

验证进程使用 config/cyclonedds_sim.xml 的 MaxMessageSize=1200 B、FragmentSize=1100 B，处理曾在 WSL 大点云 UDP 传输中观察到的缺帧。保留完整 ROS 消息，只改变 DDS 分片；通过进程级 CYCLONEDDS_URI 同时传给 Python 与两个 C++ 进程，不修改全局 DDS 或内核。不能据此推断其他机载网络的根因或配置已经标定。

### 验收记录

本发行包不将旧开发工作空间的成绩或日志作为新安装机器的验收结果。每次 verify_real 生成当前工作空间 log/verification/时间目录下的 build.log、environment.log、tests.log、tests.xml 和独立闭环报告。只以该次实际执行结果判断通过；接口、时序和合成点云求解通过仍不能代替真实雷达精度、误匹配率、时间同步及计算负载验收。
