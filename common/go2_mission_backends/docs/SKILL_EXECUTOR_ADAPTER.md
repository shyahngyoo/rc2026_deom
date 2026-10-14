# 越障 Action 服务端与执行器适配

## 运行边界

`obstacle_adapter` 保留已有 `ExecuteObstacle` Action，默认 `/execute_obstacle`，不重写 Mission FSM，也不自行申请或切换运动控制权。执行前必须收到新鲜 `MotionAuthority`：owner=SKILL、mission_id 非空、generation 有效、lease 未到期；执行中身份改变、健康失效或墙钟心跳过期均失败并请求 `/motion/emergency_hold`。

`simulation_mode` 默认 false，`skills_file` 默认空。默认仅内置测试 executor，硬件模式明确拒绝该 executor，`/skills/ready` 返回 `NOT_CONFIGURED`。没有实现实际 SDK 或真实越障动作；不能把测试执行器当作实机能力。

## 配置与接入位置

配置样例 `config/skills.yaml`：`skills` 下面以 skill_id 为键，包含 executor、mode、duration、velocity（机体 vx/vy/wz）、execution_timeout、enabled。StrictLoader 拒绝重复 YAML 键，数值、模式、允许列表和 enabled 均校验。

- Action goal 仅包含现有 obstacle_id、skill_id。
- Feedback 保留现有 phase、progress。
- `/skills/ready` Trigger 返回 JSON：skills、simulation_mode、reason；用于启动预检查。
- 所有框架输出都是 `/motion/skill_command` 的 MotionCommand，绑定收到的 mission_id/generation，交给真实仲裁节点门控。框架不直接发布最终 `/cmd_vel`。
- 模拟状态事件 `/backend_sim/skill_state` 只在 simulation_mode=true 发布 String JSON：event、obstacle_id、skill_id、request_id、mission_id、generation、mode、error_code。event 为 running/finishing/released/fault。该事件仅供模拟底层；实机状态必须由真实底层提供。

## 安全结束协议

1. PRECHECK 查询 GetMotionState（obstacle_id 空，允许上一障碍 ID），必须确认 known、stopped、无故障、非 nav_active/skill_active、safe_stance、control_releasable。
2. PRECHECK 仅发送已授权零速心跳，不产生普通运动；若取消先到，不创建或启动 executor，必须取得取消之后新的独立安全状态才返回 canceled。无法确认则 abort，control_released=false。
3. executor.start 后持续 tick；命令经过仲裁，feedback 表示执行进度。Action 成功不表示已经到出口。
4. 内部完成/确认取消后发 finishing，并持续零速。
5. 等到事件之后新的 GetMotionState 响应，必须是本 obstacle_id、known、无故障、非 active、安全且可释放；之后才调用 executor.close。
6. close 明确确认输出已释放后发 released，再次取得新的独立 GetMotionState 确认，才 Result.control_released/safe_stance=true。
7. 成功时 owner 仍为 SKILL，由现有 FSM 接着切 SAFE_HOLD。任何失败、超时、取消未确认、close 失败、底层状态失效都不宣称安全释放，并发故障/零速、请求仲裁 emergency fence。

租约同一身份的重复 stamp 不刷新墙钟心跳；更高 generation 的同 stamp 控制事件可接收，但活动 Action 因身份变化必须 abort。旧 generation、乱序 stamp、非法 sec/nanosec 都不能恢复授权。GetMotionState RPC 完成超过 state_timeout 后的旧结果也不能刷新 measured state。

## 可替换执行器接口

`skill_executor.SkillExecutor` 定义 `start(now)`、`tick(now)->SkillUpdate`、`cancel(now)->bool`、`close(now)->bool`。这些调用必须非阻塞，超时由框架单调时钟监管。真实执行器应从真实后端确认 cancel/close；不能用本地赋值冒充真实释放。

```python
node = ObstacleAdapter(executor_factories={"team_backend": TeamExecutor}, ...)
```

factory 签名为 `(SkillConfig, simulation_mode=False)`。真实 factory 的 `simulation_only=False`（或未标记）可在硬件模式注册；默认测试 factory 标记 True，所以硬件模式拒绝。通过 YAML 选择该 registry key。真实实现必须遵守 MotionCommand 统一输出，不能绕过仲裁发最终速度；实际底层站稳、物理急停、控制模式及运动技能均需独立硬件验收。

测试 executor 支持 success/fail/timeout/cancel_unconfirmed/crash/release_failure/wrong_exit。wrong_exit 只发零速也可完成 Action，出口位置验收仍由现有 Mission 完成。

## 便携验证与边界

按仓库根 README 安装后，执行完整软件验证：

~~~bash
cd ~/rc2026_ws
bash src/rc2026_deom/tools/verify_real.bash
~~~

非标准布局使用 --workspace。工具重新构建当前工作空间测试运行包，检查 ament 安装来源。测试和子进程共同使用独立 localhost ROS domain 与进程级 CycloneDDS 分片配置，不修改全局中间件、不启动真实雷达或电机。

仅运行技能模块时：

~~~bash
source /opt/ros/humble/setup.bash
source ~/rc2026_ws/install/local_setup.bash
cd ~/rc2026_ws/src/rc2026_deom
export ROS_DOMAIN_ID=154 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$(/usr/bin/python3 -c 'from pathlib import Path; from ament_index_python.packages import get_package_share_directory; print((Path(get_package_share_directory("go2_mission_backends"))/"config/cyclonedds_sim.xml").as_uri())')"
/usr/bin/python3 -m pytest -q common/go2_mission_backends/test/test_skill_executor.py \
  common/go2_mission_backends/test/test_skill_nodes.py
~~~

测试包含真实跨进程 SIGKILL：MotionArbiter 根据独立 telemetry/odom 门控，确认最终 Twist 非零后强制结束 Action 进程，检查 command_timeout 将最终输出归零，客户端不能取得成功或 released。另覆盖冲突 goals、独立状态/服务失效、取消未确认、close 失败、错误障碍 ID、租约丢失、新 generation、迟到 RPC 和 PRECHECK 取消 start=0。

完整 backend_simulation 闭环是软件测试入口。Action完成后仍由现有 Mission 验收新出口定位，wrong_exit 模式不能借 Action 成功绕过出口检查。该模拟底层不提供物理支撑或实际低层控制状态。

本发行包不附带失效的旧开发日志路径，也不把旧测试计数作为新安装机器成绩。verify_real 将本次日志、JUnit 和闭环记录写入当前工作空间 log/verification/时间目录。真实 executor 尚待团队对接，硬件模式未配置时拒绝执行；真实稳定支撑、取消、控制释放与物理急停需要独立实机验收。
