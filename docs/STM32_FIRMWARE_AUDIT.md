# STM32 参考固件只读审查

日期：2026-10-16
参考仓库：https://github.com/alpha123ad/homework
固定审查 Commit：bec442861ebd0d570e95d8a4d82873394d8f0f7c
读取方式：GitHub connector GET；没有写入、提交、推送或改动参考仓库。
此参考仓库为 private，普通未认证 git ls-remote 未取得内容；connector 有读取权限。

## 接口事实

| 内容 | F429 | H743 | 证据 |
| --- | --- | --- | --- |
| 普通运动 | horse_app_set_motion(app, forward, turn)，归一化 [-1,1] | 同左 | 两工程 User/Horse/Inc/horse_app.h；Src/horse_app.c |
| 非有限输入 | forward/turn 同归零 | 同左 | F429 horse_app.c:133–151；H743 horse_app.c:104–121 |
| 显式使能 | horse_app_request_enable；使能可能 PENDING，需要 get_enable_result | request_enable；无 app 级 pending getter | 两工程 horse_app.h；F429 horse_app.c:154–172,272–279 |
| 原地跳 | horse_app_request_jump，仅 HORSE_ENABLE_JUMP | 同左 | 两工程 horse_app.h |
| 前跃 | horse_app_request_forward_jump | 不提供；旧无线明确 UNSUPPORTED | F429 horse_app.h:102；H743 Remote/Src/horse_remote_link.c:36 |
| 支撑请求 | horse_app_set_static_support | 无该 app API | F429 horse_app.c:187–203 |
| 状态快照 | horse_app_get_snapshot | 无该 app API，旧无线直接读 horse struct | 两工程 horse_app.h；H743 Remote/Src/horse_remote_link.c:177–240 |
| 跳跃状态 | horse_app_get_jump_status，phase/failure/checking等 | 无 app 级封装 | F429 horse_app.h:43–65；horse_app.c:361–385 |
| 独占动作 | commands_suspended 覆盖enable_pending、jump check与JUMPING | 覆盖JUMPING | 两工程 horse_app.c |
| 侧移 | 无 vy；仅 forward/turn | 无 vy | 两工程 horse_app.h set_motion签名 |

注意：horse_app_set_motion 不会替代人工使能，也不能当作 SI 单位速度。m/s、rad/s→归一化量必须实测标定。符号需轮架实测确认；旧遥控1/3键分别turn=-scale/+scale不能证明ROS yaw符号。

## STOP、归零与 HOLD 不能混用

- F429 horse_app.c:403–410 request_stop 清除运动后调用 horse_control_request_stop。
- F429 horse_control.c:454–480 取消enable/jump预检、enable_requested=false并进入STOPPING。
- H743 horse_control.c:1757–1793 同样清使能并STOPPING。
- H743 horse_control.c:995–1052 stop sequence 对所有腿排STOP；成功后返回 WAIT_ONLINE，重新进行配置使能。
- F429 User/Leg/Src/leg_control.c:354–378 leg_control_stop 清静态支撑力并调用两个 mi_motor_stop。
- F429 User/CyberGear/Src/mi_motor.c:152–159 mi_motor_stop 发送 MI_COMM_STOP。
- 因此现有STOP属于驱动STOP/退出使能语义；不能承诺站立保持，不能作为“正常停车 HOLD”的C API映射。
- F429旧无线clear_motion在Remote/Src/horse_remote_link.c:92–98归零并取消static_support请求；H743 :14–19仅归零。
- 正式新协议必须区分 WALK(0,0)、HOLD（维持安全支撑）、DISABLE/STOP（驱动STOP）；未实现HOLD应UNSUPPORTED，禁止以STOP假充HOLD。

## 安全支撑与停稳证据

F429 snapshot（horse_app.h:28–42，horse_app.c:291–340）包含：
state、runtime、applied高度/步幅/步频/抬腿、在线掩码、温度、电机故障、enable_requested/pending、ground参考、static_support_supported/requested。
没有 measured_stopped / support_confirmed / control_releasable，也没有底盘实测vx/wz。

static_support_requested 只是请求，不能映射成 safe_stance=true。
F429 horse_static_support.c:57–91 实际支撑还要求：
- forward_applied/turn_applied、target均0；
- REFERENCE gait，height渐变到位；
- 八电机online、run_state=2、无fault、新鲜反馈、有限位置速度；
- 位置/速度阈值内且关节指令速度约0；
- 连续settle_ms，再经过ramp使blend达到目标。
当前app snapshot没有公布blend、settling或真实确认，因此C侧须新增支持能力和经过物理定义的support_confirmed状态；上位机不能猜测。

主机ROS /odom速度可作为额外StopMonitor证据，不能替代STM32反馈缺失。机身打滑、悬空与关节静止之间并不等价。

## 跳跃完成证据

F429 horse_jump_control.c:333–365：
八电机run_state=2、反馈年龄、跟踪误差、有限关节速度检查；
末端位置/滤波速度满足条件后 horse_jump_update；
仅在phase=HORSE_JUMP_COMPLETE时归零forward/turn、reset gait、返回RUN。
horse_jump.h phase包括 IDLE,CROUCH,PUSH,FLIGHT,LAND,RECOVER,COMPLETE,FAILED。
F429 get_jump_status给phase/failure/checking，旧无线JA有此遥测。
但“当前phase=COMPLETE”不含ROS请求关联，不能用于完成后续新请求。
新协议必须含session、skill request sequence/run id，且完成事件只能对应实际发起的当前动作；
ACK/预检通过不是物理动作完成。
任意轨迹中取消如何保持支撑需要C端定义；现有request_stop会停止驱动，不适合作为“安全取消”保证。
障碍赛bridge_a/high_wall等整套技能在参考固件无同名自动技能API，默认UNSUPPORTED。

## 当前串口内容与引脚（仅源码事实，接线需确认）

F429 Core/Src/usart.c：
- USART1，115200 8N1，PA9/PA10，GPIO_AF7_USART1。
- UART8，115200 8N1，PE0/PE1，GPIO_AF8_UART8。
- USART3被动Yesense RX在PB11，波特率YESENSE_BAUD_RATE，RX-only，PB10故意不驱动。
H743 Core/Src/usart.c：
- USART1 115200 8N1 PA9/PA10。
- UART8 115200 8N1 PE0/PE1，注释说明E34模块已保存115200。

两板main.c（F429:184，H743:191）均horse_radio_init(&huart8)。
User/BSP/Src/bsp_debug_uart.c仅USART1中断TX调试输出；RxCplt回调交给horse_radio，并无上位机命令接收器。
User/Remote/Src/horse_radio.c为E34无线，含固定地址头、AUX、27字节RF载荷限制和包间隔。
旧协议是ASCII printable body+'*'+CRC16/CCITT hex+'\\n'，会话/请求/按键P/H/R；
它不是本次新二进制协议，也不是原Hurricane S<number>E命令协议。
接USB-TTL到已有USART1不会自动支持HELLO；直接接UART8绕过无线也需要C端去除无线地址/AUX/发送调度并增加协议解析。必须由STM32同事指定实际UART、接线及3.3V电平。

## 现有watchdog局限

旧无线 HORSE_REMOTE_MOTION_TIMEOUT_MS=350、HORSE_LINK_TIMEOUT_MS=500；
Remote/Src/horse_remote_input.c:132–146只在active_key!=0时归零；
Remote/Src/horse_remote_link.c在会话超时做clear_motion（F429还取消pending）。
这些保护属于旧无线输入链路，不可宣称已保护新USB自动控制链路。
新C侧必须为自动控制命令建立本地watchdog，并明确超时采取的支撑/急停策略，不能依赖上位机还活着发送STOP。
已有bsp_watchdog为MCU主循环饥饿重置保护，语义不同。
未发现独立physical estop_active和operator_enable标准遥测接口，不能凭串口在线/握手ACK推定已使能或急停正常。


软件接入方式、待固件协作项和验证边界见 [STM32 接入指南](STM32_BRIDGE_GUIDE.md)。未连接硬件，不能据此宣称实机通信或运动安全通过。
