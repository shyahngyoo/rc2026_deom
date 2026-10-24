# STM32 Host UART v1 协议规范

版本：v1；整理日期：2026-10-17；目标：Ubuntu 22.04 / ROS 2 Humble ↔ STM32F429。

实现基线：`shyahngyoo/rc2026_deom`，分支 `rely-only`，Commit `b3ed82fc10492907d007be14510ee591ad2de4a3`。

**本文件完整描述当前主机实现及其固件接入契约，不表示协议已经双方冻结或参考固件已经实现。** 当前源码仍标记 DRAFT。凡标为“待确认”的行为，均不能当作已经实现的底层能力。本次仅整理文档，没有运行测试、生成测试向量或连接硬件。文中的 JSON 是字段说明示例，不是新增测试向量；示例中的技能编号不构成能力声明。

## 1. 角色、线路与安全原则

Host（Ubuntu）运行 Mission、Nav2、MotionArbiter、Hurricane 和唯一串口网关 SerialGateway。STM32 接收经仲裁的归一化运动或离散技能请求，调用现有 `horse_app` 执行，并报告实际状态。Host 不生成关节轨迹。协议没有 ARM、ENABLE、故障清除或控制权 RELEASE 命令。

默认 UART：115200 baud，8 data bits，no parity，1 stop bit（8N1），无软/硬件流控。波特率可配置；其他串口格式由 `serial.Serial` 默认参数确定。使用 USB-TTL，实际 UART、引脚、3.3 V 电平和接线由固件同学确认。TX/RX 交叉、GND 共地；不能使用 RS-232 电平。调试文本不得混入 Host UART 协议输出。

Host 单进程、单 worker 独占串口（pyserial `exclusive=True` 和 Linux `TIOCEXCL`）。默认 `port=''` 不打开串口；打开后先发 HELLO，不能因连接成功自动使能。默认 `test_mode=true`、`enable_motion=false`、`calibration_confirmed=false`；只有三项显式满足运动许可，才可能请求运动。软件开关不能代替物理急停、人工使能、实时定位/预检查或 STM32 本地保护。

Host 控制权由 MotionArbiter 独占管理：`SAFE_HOLD`、`NAVIGATION`、`SKILL`。`SAFE_HOLD` 是 ROS 仲裁 owner，不是 UART `mode` 值；UART 安全保持模式叫 `HOLD`。最终速度必须经过 `/stm32/arbiter_command` 的带授权 `MotionCommand`；普通 `/cmd_vel`、Nav2 原始速度或诊断 Twist 都不能直达电机。STM32 还必须排除 E34 与 Host 同时控制普通运动，不能仅相信 Host 内部仲裁。

安全证据必须实测：ACK、串口在线、零速度请求、支撑请求、历史动作完成均不能替代实际停稳、安全支撑或控制权可释放。

## 2. 二进制帧与解析

### 2.1 字节布局

固定头长度 18 bytes，尾部 CRC 4 bytes。多字节二进制整数均 little-endian；JSON 内数字是文本，不使用二进制大小端。

| 偏移（byte） | 长度 | 类型 | 内容与范围 |
| --- | --- | --- | --- |
| 0 | 2 | bytes | 固定 `A5 5A` |
| 2 | 1 | uint8 | `version=1` |
| 3 | 1 | uint8 | 第 3 节 message type |
| 4 | 2 | uint16 LE | `payload_length=L`，JSON 的 UTF-8 字节数，最大 1024 |
| 6 | 4 | uint32 LE | `sequence`，1…4294967295 |
| 10 | 8 | uint64 LE | `session`，1…18446744073709551615 |
| 18 | L | UTF-8 | JSON object |
| 18+L | 4 | uint32 LE | CRC32，覆盖从偏移 0 到 17+L 的全部原始字节 |

总长度 `22+L`，最大 **1046 bytes**。长度字段可以编码 0，但空 payload 不是合法 JSON object，会被拒绝；最短合法 object 是 `{}`。协议没有换行终止符、E34 地址头、AUX 字段或旧无线 27-byte 限制。

CRC 使用 Python `zlib.crc32(header+payload) & 0xffffffff`，即 CRC-32/ISO-HDLC：width=32，正常多项式 `0x04C11DB7`，反射多项式 `0xEDB88320`，init=`0xFFFFFFFF`，RefIn/RefOut=true，xorout=`0xFFFFFFFF`。CRC 结果按小端存放。必须对收到的原始字节计算，不能解析 JSON 后重新排序/序列化再校验；STM32 CRC 外设不能未经核对直接采用默认算法。

### 2.2 JSON 与流式解析规则

Host 发送紧凑 JSON，键排序，非 ASCII 字符使用 JSON 转义（`ensure_ascii=True`），禁止发送 NaN/Infinity。接收允许合法 UTF-8 object；拒绝重复键（包括嵌套 object）、显式 NaN/Infinity 常量、非法 UTF-8、非 object、未知帧类型、错误版本、零 sequence/session、过长 payload 和错误 CRC。Host parser 按帧头重新同步，保留半帧，并能拆分粘连帧；UART 一次 read 不等于一帧。

接收 parser 不是所有消息的完整 schema 校验器。STATUS 有专用校验；ACK 为局部校验；FAULT 较宽松；额外 JSON 字段目前没有通用拒绝规则。未知字段不得作为新的运动能力解释。极端 JSON 数值、字符串长度、嵌套深度等 MCU 资源上限仍需双方确认，不能把 Python 可解析范围当成固件安全范围。

损坏帧不刷新 STATUS 新鲜度，不成为安全证据；Host 不对损坏帧发 ACK。CRC 是传输完整性校验，不提供身份认证或加密。

## 3. 消息总表与字段约定

| Type（十进制 / 十六进制） | 名称 | 方向 | ACK | 运动相关副作用 |
| --- | --- | --- | --- | --- |
| 1 / 0x01 | HELLO | Host → MCU | MCU 回复 | 协商；不得自动使能或开跑；新 session 撤销旧意图 |
| 2 / 0x02 | SET_MODE | Host → MCU | MCU 回复 | 改变执行器控制模式，不是位移命令 |
| 3 / 0x03 | WALK | Host → MCU | MCU 回复 | 前后/转向运动 |
| 4 / 0x04 | SKILL_START | Host → MCU | MCU 回复 | 启动离散动作 |
| 5 / 0x05 | SKILL_CANCEL | Host → MCU | MCU 回复 | 请求取消，可能触发安全收尾 |
| 6 / 0x06 | HOLD | Host → MCU | MCU 回复 | 请求停止并维持安全支撑 |
| 7 / 0x07 | STOP | Host → MCU | MCU 回复 | 显式驱动停机/卸力，不能冒充支撑 |
| 8 / 0x08 | HEARTBEAT | Host → MCU | MCU 回复 | 维持有效命令窗口，不替换离散轨迹 |
| 128 / 0x80 | ACK | MCU → Host | Host 不回复 ACK | 仅接受/拒绝信息，不证明执行完成 |
| 129 / 0x81 | STATUS | MCU → Host | Host 不回复 ACK | 遥测；可使主机推进状态或封锁运动 |
| 130 / 0x82 | FAULT | MCU → Host | Host 不回复 ACK | 故障通知；主机撤销运动意图 |

以下“必填”对 Host→MCU 指当前主机实际生成的字段，对 MCU→Host 指当前接收器要求的字段。MCU 接收条件列出的检查是既有安全契约要求；MCU 实现尚未交付，不能解释为固件已执行检查。每条 Host 请求都受帧/session/去重规则约束；拒绝必须不执行运动副作用。语义拒绝用 `accepted=false` ACK；无效帧和无法关联的旧会话不能当作有效请求执行。拒绝 reason 字符串没有冻结的枚举。

### 3.1 共用字段组（后续引用均包含此处全部约束）

| 字段组 | 字段 | 类型与当前主机范围 | 含义 |
| --- | --- | --- | --- |
| A：授权 | owner | string，`NAVIGATION` 或 `SKILL`；技能为 `SKILL` | 当前运动所有者 |
| A | mission_id | 非空 string，主机检查最长 128 个字符 | 仲裁任务标识；不是 UART session |
| A | generation | JSON integer；有效值须高于本机被隔离代次，并匹配当前租约；ROS 来源为 uint64 | 控制权代次；不是帧序号 |
| T：有效期 | valid_for_ms | JSON integer，`max(1,int(command_timeout*1000))` | 命令相对有效期（毫秒）；YAML 默认 200，代码兜底默认 300；未在网关配置中规定最大值 |
| C：技能关联 | request_id | 非空 string，最长 128 个字符 | 单次操作 ID；正常 transport 使用 UUID hex |
| C | runner_session | 非空 string，最长 128 个字符 | Hurricane runner 会话，不是 UART uint64 session |
| C | obstacle_id | 非空 string，最长 128 个字符 | 当前障碍标识 |
| C | step_index | JSON integer，0…2147483647 | runner 步骤索引 |

主机 `_safe_motion` 共用门控：运动软件开关全部允许；已连接且 HELLO 完成；新鲜 STATUS 且 `known=true`；`estop=false`、`human_enable=true`、`fault_code=""`；当前 owner/mission/generation 匹配、租约健康且未过期，授权反馈新鲜。切换模式另需真实停稳/支撑/可释放。

以上字段长度是 Python 字符数限制，整个 JSON 仍须 ≤1024 bytes；没有为 MCU 规定新的单字段字节数组长度。`generation` 在 core 没有独立 wire 上限校验，应区别 ROS uint64 来源与尚未冻结的固件范围。

运行中的 `motion_arbiter` 使用持久化区间分配 `generation`，节点重启后不回到 1，
其值可以超过 `UINT32_MAX`。固件解析和保存这个授权字段时应使用 `uint64_t`，不能截断为
32 位；这不改变 UART 帧头 `sequence` 的 uint32 格式。主机区间分配范围目前限制在
正 int64 内，仍属于 ROS uint64 的有效范围。

### 3.2 HELLO — 1

必填（当前 Host 发送）：`protocol_version`（integer，1），`host`（string，固定 `quadruped_stm32_bridge`）。无其他当前可选发送字段。既有最简示例/向量省略 host，只含版本；其兼容性差异见第 9 节。

```json
{"protocol_version":1,"host":"quadruped_stm32_bridge"}
```

Host 每次成功打开串口生成新 session 后发送。MCU 应识别支持的版本并建立当前 session，撤销旧速度、授权和待执行队列；不能重放先前动作。新会话到来时进行中的腾空动作如何安全收尾，仍待固件确认。

MCU 不支持版本或真实缺少 feature 时应拒绝/如实声明。HELLO ACK 除通用字段还必须含 `protocol_version=1` 和 `features` string array；必须包含 `HOLD`、`SET_MODE`、`WALK`、`STATUS`、`HEARTBEAT` 才能使当前 Host 握手成功。额外按实际实现声明 `SKILL_START`、`SKILL_CANCEL`、`STOP`。不能为得到握手成功而虚报能力。无运动使能副作用。

### 3.3 SET_MODE — 2

必填：`mode`（string，当前 Host 仅发 `NAVIGATION` 或 `SKILL`），`mission_id`、`generation`（见 A）。**不发送 owner 或 TTL。** 无当前可选字段。

```json
{"mode":"NAVIGATION","mission_id":"mission-1","generation":2}
```

Host 只有通过共用门控、没有待验收 HOLD，且 STATUS 同时满足 `stopped && safe_stance && control_releasable && !nav_active && !skill_active` 才新发模式请求。导航还需要有效 NavigateToPose goal；技能需要本次合法请求。模式 ACK 不直接设置 `nav_active/skill_active`。

MCU 必须拒绝不能安全交接、无效/过期授权或不支持的模式；实际如何记忆与比较 mission/generation、何时撤销模式见待确认项。ACK 只表示接受切换；完成须由真实 STATUS 报告。Host 不使用 `SET_MODE(HOLD)` 或 `SET_MODE(DISABLED)`；保持使用 HOLD，驱动停机使用 STOP，不能据 STATUS 枚举扩展可发送模式。

### 3.4 WALK — 3

必填：`forward`、`turn`（有限 JSON number，均在 [-1,1]），全部 A、T 字段。无当前可选字段。

```json
{"forward":0.16,"turn":0.0,"owner":"NAVIGATION","mission_id":"mission-1","generation":2,"valid_for_ms":200}
```

`forward=vx/vx_full_scale`，`turn=wz/wz_full_scale`；它们是归一化输入，不是 m/s 或 rad/s。符号、标定与实际增益待实测。主机拒绝越限、非有限值、非零 vy/vz/roll/pitch；协议没有横移字段。WALK 可以属于 NAVIGATION，也可以属于 SKILL 下的前后/转向步骤。

Host 必须有当前授权的唯一仲裁器新鲜输出，且 STATUS 的 `mode` 等于 owner、相应 active=true；NAV 还必须有活动 goal。HOLD 待确认时不发 WALK。离散动作 active 或 `skill_state=accepted/running` 时，非零速度被拒绝；零速度转为 HEARTBEAT，而不是用 WALK(0,0) 中断轨迹。

MCU 必须检查模式、授权、TTL、限幅、急停/人工使能、执行器冲突后接受，否则拒绝；相同/旧帧不得复活速度或延长有效期。ACK 不证明机器人已达到速度。WALK(0,0) 也不证明停稳或支撑。会产生运动副作用。

### 3.5 SKILL_START — 4

必填：`command_id`（integer，当前 UART 实际发送范围 6…18），全部 C、A、T 字段，其中 owner 固定 SKILL。无当前可选字段。

```json
{"command_id":6,"request_id":"start-1","runner_session":"runner-1","obstacle_id":"obstacle-a","step_index":0,"owner":"SKILL","mission_id":"mission-1","generation":3,"valid_for_ms":200}
```

示例仅说明格式；未确认 6 对应哪种固件动作。1…4 是 ROS 侧方向指令，通过仲裁后 WALK 执行；5 走 HOLD；19/20 被拒绝。`capabilities` 接收范围虽为 1…18，不能因此向 MCU 发送 SKILL_START(1…5)。

Host 要求 SKILL 授权、共用门控、feature `SKILL_START`、command_id 在当前 STATUS.capabilities 中，且无 deferred/active 技能、无 accepted/running 旧动作。请求可等待模式切换。**实现细节：`_try_skill` 最终入队条件检查 mode=SKILL，但未再次要求 skill_active=true；不能在固件侧仅依赖 active 标志来省略真实就绪判断。**

MCU 应拒绝未实现编号、未就绪/忙、无效授权、TTL 不合法或不安全的启动；不得自行定义未知编号的行为。ACK 表示请求接受，执行由 STATUS 的 accepted/running/终态说明，并回显全部 C。相同请求不能二次起跳。会产生运动副作用。

### 3.6 SKILL_CANCEL — 5

必填：全部 C 字段。无其他当前字段；**线上不含 command_id、owner、mission_id、generation、TTL，也没有 target_request_id。**

```json
{"request_id":"cancel-1","runner_session":"runner-1","obstacle_id":"obstacle-a","step_index":0}
```

当前 transport 为 cancel 新建 request_id；网关先发送带同一 C 和 `reason=SKILL_CANCEL` 的 HOLD，再发 SKILL_CANCEL。因此 cancel-1 不等于先前 start-1，不能要求 MCU 把它与 START.request_id 简单相等匹配。取消目标和终态关联仍是必须双方确认的缺口，见第 10 节。

Host 要求已连接、握手完成、软件运动开关允许、C 合法、feature SKILL_CANCEL；为允许请求安全收尾，不要求仍持有活租约。MCU 必须在安全策略内处理，不可将其解释成强制卸力；无法处理或关联时拒绝并如实反馈，不能返回虚假安全状态。ACK 不证明取消完成。可能改变执行轨迹或进入安全收尾；具体动作阶段行为待确认。

### 3.7 HOLD — 6

当前存在两种 payload 形态：

| 形态 | 必填 | 可选/不存在字段 |
| --- | --- | --- |
| 自动保护/导航结束/退出 | `reason`，string | 不含 C、授权或 TTL |
| 技能 hold/cancel 的前置 HOLD | 全部 C，`reason` string（`SKILL_HOLD` 或 `SKILL_CANCEL`） | 不含授权或 TTL |

```json
{"reason":"NAVIGATION_GOAL_TERMINAL"}
```

```json
{"request_id":"hold-1","runner_session":"runner-1","obstacle_id":"obstacle-a","step_index":1,"reason":"SKILL_HOLD"}
```

其他现有 reason 包括 HOST_SHUTDOWN、SAFE_HOLD_AUTHORITY、STATUS_STALE、COMMAND_TIMEOUT 和封锁原因；不是冻结的固件枚举。Host 自动 HOLD 只在连接、握手及软件运动许可满足时尽力发送；技能 HOLD 还验证 C 和真实 feature。不需要活租约才能请求支撑。

HOLD 清理主机未发/待 ACK 的 WALK、HEARTBEAT、SET_MODE、SKILL_START，阻止新运动直到真实验收。MCU 必须实现可保持支撑的处理，不能把 `horse_app_request_stop` 当作 HOLD；不支持应拒绝。拒绝 HOLD 会使 Host 设置 `handshake=false`、报 HOLD_UNSUPPORTED_OR_REJECTED。

ACK 不是支撑完成。网关解除 hold_pending 要求新 STATUS：`mode=HOLD`、stopped/safe_stance/control_releasable 全 true、两个 active 全 false；若 HOLD 带 C，还必须四字段全部匹配。Hurricane hold 步骤完成另要求 `skill_state=succeeded`，见第 6 节。HOLD 可能引发减速/落地/姿态保持等运动，其实现和物理判据未定义。

### 3.8 STOP — 7

必填：全部 C。无当前可选字段；线上不携带授权或 TTL。

```json
{"request_id":"stop-1","runner_session":"runner-1","obstacle_id":"obstacle-a","step_index":1}
```

只有显式内部 `op=stop`、有效 SKILL 授权及完全相等的 mission/generation、共用门控和 feature STOP 才能生成。Host 随即撤销本地运动意图，再发送 STOP。当前 GatewayTransport 没有普通技能到 STOP 的自动映射。

MCU 必须保留物理急停优先级，拒绝不支持或不符合本地安全策略的请求。由于线上缺少授权字段，固件如何用当前会话/已建立模式校验 STOP 需双方明确，不能宣称接收端能从此 payload 直接验证主机租约。

ACK 不代表电机实际停机，状态必须如实上报；网关没有专用“STOP 已完成”验收状态机。STOP 是驱动 STOP/退出使能/可能卸力，可能失去支撑，不能作为导航结束、HOLD 或超时安全保持的替代。协议不规定 STOP 后 mode 必定为 DISABLED，也不提供自动重新使能。

### 3.9 HEARTBEAT — 8

必填：全部 A、T 字段；当前发送 owner=SKILL。无当前可选字段。

```json
{"owner":"SKILL","mission_id":"mission-1","generation":3,"valid_for_ms":200}
```

只有经过当前仲裁器的零速度输出、处于离散 active 技能或 STATUS accepted/running 时，Host 将其转换成 HEARTBEAT。收到 SKILL_START 后、尚未收到首个 running STATUS 的窗口也使用此路径。不是一连接就周期无条件发送的链路保活；正常 WALK 本身更新运动指令。

MCU 检查当前 session、模式、授权、TTL 和安全门控；错误/旧帧拒绝，不刷新有效窗口。ACK 必需。不得启动新动作、替换轨迹或把它解释为 WALK(0,0)。它维持动作继续有效的窗口，故仍有运动安全影响。MCU 本地续期和到期行为待确认。

### 3.10 ACK — 128

| 字段 | 要求 | 类型/范围/含义 |
| --- | --- | --- |
| acked_sequence | 必填 | 精确 JSON integer，必须命中当前 session 主机 pending 请求序号；有效请求范围 1…4294967295 |
| acked_type | 必填 | 精确 JSON integer，必须等于该请求 type |
| accepted | 必填 | 精确 JSON boolean，不允许 0/1 或字符串 |
| reason | 普通 ACK 可选 | 拒绝说明；主机使用 `str(...)` 转换，缺失用 CONTROLLER_REJECTED；尚无严格 wire 类型/枚举检查，固件应返回可读字符串 |
| protocol_version | HELLO ACK 必填 | 精确 integer，1 |
| features | HELLO ACK 必填 | string array，需覆盖第 3.2 节必需项；当前 Host 允许重复项并转集合，也允许未知额外字符串，但不会因此新增命令 |

```json
{"acked_sequence":1,"acked_type":1,"accepted":true,"protocol_version":1,"features":["HOLD","SET_MODE","WALK","STATUS","HEARTBEAT"]}
```

```json
{"acked_sequence":7,"acked_type":4,"accepted":false,"reason":"UNSUPPORTED"}
```

ACK 帧头使用 MCU 自己的 sequence，不是 acked_sequence；session 必须等于当前会话。无匹配 pending、类型不符或字段类型错误均不接受。HELLO features 不满足报 HANDSHAKE_UNSUPPORTED。普通拒绝触发主机封锁；HOLD 拒绝另取消握手。ACK 不刷新 STATUS、不证明停稳/完成，Host 不回复 ACK。协议不通过 ACK 回显 C；主机用原 pending 请求恢复关联。

### 3.11 STATUS — 129

必填与可选字段完整列于第 6 节；无 timestamp、mission_id、generation 或额外嵌套 status 要求。以下为无动作且状态未知的完整 payload 示例，不伪造安全证据：

```json
{"known":false,"mode":"DISABLED","nav_active":false,"skill_active":false,"stopped":false,"safe_stance":false,"control_releasable":false,"estop":false,"human_enable":false,"obstacle_id":"","fault_code":"","skill_state":"idle","capabilities":[]}
```

示例 estop=false 只表示该示例假设急停未触发，不应用于硬件输入未知的情况；若状态不可信须 known=false，不能据其他标志放行。

Host 仅在握手后处理，要求新 MCU sequence、当前 session、正确帧和专用字段校验。两 active 同时 true、active 与 mode 冲突、非法布尔/枚举/能力字段等使 STATUS 无效并封锁运动。known=false、estop=true、human_enable=false 或非空 fault_code 同样阻断运动，即使结构有效。Host 不发送 ACK。新有效 STATUS 仅是当前实际状态的证据，不能直接驱动电机；ACK/重复旧 STATUS 均不能替代它。

### 3.12 FAULT — 130

指南约定字段 `code` 为故障字符串；**当前接收实现没有必填字段校验**，缺失时使用 CONTROLLER_FAULT，存在时 `str(payload['code'])` 转换。没有其他当前定义的字段；额外字段不被使用，也没有冻结的 code 枚举。

```json
{"code":"CONTROLLER_FAULT"}
```

Host 对当前 session、新 MCU sequence 的合法 FAULT 帧清空状态并封锁运动，即使 HELLO 尚未完成也会处理。故障后在可发送条件满足时尽力请求 HOLD；不自动发送电机 STOP。Host 不回复 ACK。错误 session/序号/帧拒绝；payload 类型已由 parser 要求为 object。固件应提供有意义的非空字符串，空值及故障锁存/清除规则需双方确认。FAULT 不是实际急停或安全支撑执行的替代。

## 4. session、sequence、去重和重传

### 4.1 会话生命周期

每次串口打开，Host 清输入/输出缓存，生成新的随机非零 uint64 session（保证与紧邻上一 session 不同），清 parser、命令队列、pending、技能和请求缓存；两个方向的接收/发送序号从零状态开始，第一帧为正数。MCU 所有响应使用该 session。MCU 重启后如何辨识旧 session、何时强制重新 HELLO，需固件实现约定。

新会话必须清除旧授权、速度和排队动作，不能重新执行先前任务。Host 会隔离本会话第一次观察到的授权代次，后续必须有更高 generation 和新命令才可能恢复。随机 session 是防止旧数据混入的标识，不能替代认证，也不保证跨无限历史绝对无碰撞。

### 4.2 序号与重复请求

两个发送方向各自维护严格递增非零 uint32 sequence。Host 消费 MCU 帧时只接受 `sequence > received_sequence`；ACK、STATUS、FAULT 共用 MCU 序列。新序号即使 payload 后续未通过语义校验，也已经推进接收最高序号。Host 对错误方向的已知类型没有业务处理，不产生 ACK。

Host 不绕回序号；达到上限、或 queue+pending 达到 128 时断开，错误 TX_LIMIT_OR_SEQUENCE_EXHAUSTED。MCU 方向也应在耗尽前重新建会话，不得回绕；主动要求重连的专门 wire 消息尚不存在。

MCU 需按 `(session,sequence)` 缓存原请求结果：相同请求重复到达，只重复确认原结果，不能重复动作、续 TTL 或恢复旧速度；未缓存且低于最高序号的请求按旧命令拒绝。同 sequence 不同内容的冲突处理细节和缓存容量/保留期限待冻结，不能把冲突当新动作。

**重发 ACK 应使用新的 MCU 帧头 sequence，同时保持原 acked_sequence/acked_type/accepted/结果。** 若逐字节重发旧 ACK 帧，Host 在已经接收更高 MCU 序号后会丢弃它。“回放原 ACK”指原确认结果，不保证旧帧头可重用。

除 UART 去重，Host 对技能内部 request_id 每连接缓存最多 2048 个请求：相同内容不再次执行，不同内容报 REQUEST_ID_CONFLICT；缓存满拒绝新请求。该缓存不是 MCU 持久去重实现，MCU 仍需处理重传和新旧会话。

### 4.3 ACK 超时重发

所有 1…8 请求进入 pending；默认 ACK 超时 0.15 s，最多重发 2 次（初发加最多两次重发）。重发使用相同 Host sequence/session 和原始帧，不创建新动作 ID。重发仍经过写出门控；授权类请求若创建后超过 command_timeout、授权改变或状态失效则不能写出，所以不能保证每条命令一定发满三次。

新 WALK/HEARTBEAT 会删除旧 WALK/HEARTBEAT 的队列与 pending，避免旧速度晚重放。HOLD 和封锁会进一步清理运动请求。MCU 不能因较迟的旧命令或重发重新延长 TTL。TTL 以首次接受命令的本地时间计、而非每次重复收包续期，是既有防重放契约；硬上限和各种消息的续期细节见第 10 节。

HELLO 最终 ACK 超时断连；其他非 HOLD 请求 ACK 超时封锁并尽力 HOLD；HOLD ACK 耗尽仅记录 ACK_TIMEOUT，不证明安全，也不自动改发 STOP。ACK 延迟累计受 worker 调度及 8N1 传输时间影响；这些参数不是精确实时截止保证。

## 5. 状态机与交互时序

### 5.1 连接与导航

1. Host 打开指定串口 → HELLO（新 session）。MCU 接受会话并回复 HELLO ACK，声明实际 features。握手不使能电机。
2. MCU 持续上报真实 STATUS。Host 等待 known、安全输入、无故障及新的有效控制权。默认软件输出禁用时停留在观察阶段。
3. 有活动 NavigateToPose goal、有效 NAVIGATION 授权，且执行器 inactive、真实停稳/支撑/可交接 → Host 发 SET_MODE(NAVIGATION)。
4. MCU ACK 接受，然后完成实际模式准备并通过 STATUS 报 mode=NAVIGATION、nav_active=true。不能仅回显请求；也不能等第一条非零 WALK 后才报 active，否则会与 Host 的发送门控形成互等。
5. Host 收到匹配当前代次的新鲜仲裁速度 → 连续 WALK。每条请求均有 ACK；持续新 STATUS 提供反馈。ACK 接受与运动执行中是两个不同事实。
6. goal 结束或取消中 → Host 发 HOLD；等真实 mode=HOLD、双 inactive、stopped/safe_stance/control_releasable=true。此时才具备交接证据。

### 5.2 HOLD → SKILL → HOLD → NAVIGATION

1. 原控制方停止并报告真实可释放；MotionArbiter 在既有安全逻辑下授予 SKILL 新租约。
2. Hurricane 提交本次技能请求，Host 发 SET_MODE(SKILL) 并等待状态；MCU 报真实技能执行器就绪。主机 `_try_skill` 的 mode-only 判定差异见第 9 节，固件不得省略就绪检查。
3. 对方向步骤，Host 发受仲裁 WALK；对已声明离散能力，Host 发 SKILL_START(C、授权、TTL)。MCU ACK 接受，后续 STATUS 带同一 C 报 accepted/running。
4. 离散动作期间，Host 用 HEARTBEAT 延续命令窗口，不反复 START，不用 WALK(0,0) 停轨迹。
5. 动作结束，MCU STATUS 带对应 C 报 succeeded/failed/canceled。**终态不等于停稳、安全支撑或控制权释放。** 仍须独立真实状态。
6. Hurricane 的 HOLD 步骤使用新的 C；Host 发 HOLD，MCU 完成安全收尾并用该 HOLD 的 C 上报真实状态。网关与 Hurricane 的验收条件见第 6 节。
7. MotionArbiter 完成安全交接后可授予 NAVIGATION 新租约；结合新的有效导航 goal，再发 SET_MODE(NAVIGATION) → STATUS ready → 新 WALK。UART 没有独立 RELEASE 消息，`control_releasable` 只是允许 Host 交接的证据，不直接分配 ROS 控制权。
8. Mission 的出口定位验收仍由定位系统及 Mission 完成，不能用 MCU 的 succeeded 或 stopped 代替“已到达出口”。

### 5.3 取消、故障与恢复

取消流程是 HOLD(C_cancel) → SKILL_CANCEL(C_cancel) → ACK 与后续真实 STATUS；取消使用新 request_id。如何对应被取消的 START、先 HOLD 后 CANCEL 的作用顺序、腾空时取消和最终应回报哪个关联 ID，必须双方冻结后才能完整联调。

出现状态过期、授权失效、命令过期、控制器拒绝/故障、急停、人工使能撤销、无效状态或检测到输出绕过等情况，Host 清理运动意图并隔离旧代次，在条件允许时尽力 HOLD。断线无法保证 HOLD 到达，因此 MCU 必须自行保护。Host 时钟回退也清状态、隔离旧意图；超时主要用单调时钟，ROS 原始消息时间另做新鲜度校验。

恢复需要故障原因消除、真实新鲜安全状态、必要时重连/HELLO、比被隔离值更高的授权代次及新命令。代码不提供 MCU 清故障命令，不能假定 ACK 或换 session 就清除物理故障。不要自动续跑旧任务；操作人员及上层任务流程应重新评估。物理重新使能、故障锁存和 MCU 重启策略属于待确认固件约定。

## 6. STATUS 全字段与安全语义

| 字段 | 必填性与类型/范围 | 语义与禁止推断 |
| --- | --- | --- |
| known | 必填，精确 bool | 控制器当前状态是否可信；未知不得伪装安全 |
| mode | 必填，string：HOLD/NAVIGATION/SKILL/DISABLED | 实际执行器控制模式，不是模式请求回显 |
| nav_active | 必填，精确 bool | 导航执行器已实际接管/可接受该模式命令；不应简单等于速度非零；物理判据待确认 |
| skill_active | 必填，精确 bool | 技能执行器实际接管/活动；与 nav_active 互斥；不等同 skill_state=running |
| stopped | 必填，精确 bool | 实际停稳，不能由最后一次零速度或 ACK 推断；速度/时间阈值待测 |
| safe_stance | 必填，精确 bool | 实际达到安全支撑，不等同 static_support_requested；确认判据待测 |
| control_releasable | 必填，精确 bool | 当前机械/执行状态允许交接；危险或活动阶段不能伪报释放 |
| estop | 必填，精确 bool | 真实物理急停状态；true 阻止 Host 运动 |
| human_enable | 必填，精确 bool | 真实人工运动许可；false 阻止 Host 运动；不能由 HELLO 成功置 true |
| fault_code | 必填，string | 无故障空字符串；非空封锁运动；枚举未冻结 |
| skill_state | 必填，string：idle/accepted/running/succeeded/failed/canceled | 本次关联操作状态；accepted 不等于执行完成，终态也不等于支撑完成 |
| capabilities | 必填，array | 长度 0…18，唯一精确 integer 1…18；仅声明实际实现，空数组合法；不支持 19/20 |
| obstacle_id | 必填，string | 无关联可为空；确认操作时必须与该请求完全相等 |
| request_id | parser 可选，出现须 string | 确认技能/关联 HOLD 时实际必需；完全匹配该次操作，不能用历史完成状态 |
| runner_session | parser 可选，出现须 string | 同上，必须匹配 runner 会话 |
| step_index | parser 可选，精确 integer ≥ -1 | 无步骤可用 -1；确认请求时须与有效请求 0…2147483647 完全匹配 |

STATUS parser 未为字符串指定最长长度，未为 step_index 指定上限；受整体 1024-byte payload 限制。附加字段目前保留但没有新控制语义。所有布尔必须是真正 JSON true/false；0/1、null、字符串都非法。

组合约束：不能 nav_active 与 skill_active 同时 true；nav_active=true 要求 mode=NAVIGATION；skill_active=true 要求 mode=SKILL。parser 没有对 stopped/support/releasable 的全部物理组合做一致性校验，固件必须如实测量，不能靠“能通过 parser”证明物理合理。

三层验收必须区别：

- **START 完成关联**：网关仅在四项 C 匹配 active_skill 的终态时清 active；对结束操作的 C 匹配，还要求 !skill_active 且 stopped。历史终态不完成新请求。
- **网关 HOLD 验收**：mode=HOLD、stopped/safe_stance/control_releasable=true、两个 active=false；带 C 的 HOLD 还必须四项匹配。此检查本身不要求 skill_state=succeeded；后续实际放行仍受 known/急停/人工使能/故障和授权门控。
- **Hurricane HOLD 步骤完成**：新的、当前串口 session 和授权下、四项 C 匹配的 STATUS，且 skill_state=succeeded；hold 还要求 stopped/safe_stance/control_releasable=true、skill_active=false。failed 或 canceled 被作为失败反馈。仅报告 HOLD+idle，虽然可解除网关 hold_pending，却不能完成 Hurricane 的 hold 步骤。两个层次判据不能混淆。

STATUS 不自带时间戳。Host 为每个新包记录原始 ROS 接收时间及本地单调接收时间，并生成内部 gateway_sequence；缓存快照不能刷新证据。`/stm32/status` 的 connected/handshake/session/gateway_sequence/stamp_ns/status/error_code 是 ROS 包装，不要在 UART STATUS 里照搬嵌套结构。TelemetryAdapter 只发布新鲜测量证据；无效状态不会把停稳/支撑/释放标志置 true。

## 7. 超时、TTL 与 watchdog

### 7.1 当前主机参数

单位除特别说明外均为秒。随包 YAML 会覆盖代码默认；实际部署还可被外部参数覆盖，不能把任一默认当作固件固定值。

| 参数 | GatewayCore 内置默认 | stm32.yaml 默认 | 作用 |
| --- | --- | --- | --- |
| port | 空 | 空 | 不打开任何串口 |
| baudrate | 115200 | 115200 | UART 波特率 |
| read_timeout | 0.02 | 0.02 | 单次串口 read 等待界限，不是断链判据 |
| write_timeout | 0.05 | 0.05 | 单次 write 等待界限 |
| reconnect_interval | 1.0 | 1.0 | 串口失败后的重连间隔 |
| max_reconnect_attempts | 5 | 5 | worker 生命周期总打开尝试次数；成功后也不归零 |
| handshake_timeout | 2.0 | 2.0 | 打开后未握手完成的时限；ACK 耗尽也可能提前断连 |
| status_timeout | **0.4** | **0.3** | STATUS 接收新鲜度 |
| authority_timeout | **0.4** | **0.3** | ROS 授权新鲜度，另受 lease_remaining 约束 |
| command_timeout | **0.3** | **0.2** | 新鲜速度/排队命令上限，生成 valid_for_ms |
| ack_timeout | 0.15 | 0.15 | pending ACK 重试间隔 |
| max_ack_retries | 2 | 2 | 最多两次重发 |
| status_publish_frequency | 20 Hz | 20 Hz | Host ROS 发布/检查 timer，**不是 MCU STATUS 发送频率** |
| enable_motion | false | false | 显式输出许可 |
| test_mode | true | true | true 阻止运动 |
| calibration_confirmed | false | false | 未标定阻止运动 |
| vx_full_scale | 0.5 m/s | 0.5 m/s | forward=1 的示例线性标定，未实测 |
| wz_full_scale | 1.0 rad/s | 1.0 rad/s | turn=1 的示例线性标定，未实测 |
| max_linear_speed | 0.5 m/s | 0.5 m/s | Host 速度限幅 |
| max_angular_speed | 1.0 rad/s | 1.0 rad/s | Host 角速度限幅 |

随包 NavVelocityAdapter 的 authority_timeout=0.3、command_timeout=0.2；TelemetryAdapter status_timeout=0.3；Hurricane gateway_status_timeout=0.3，技能方向速度示例 forward=0.08 m/s、turn=0.25 rad/s。GatewayTransport 自身 authority_timeout 默认 0.5，是独立上层检查，不能用它覆盖网关的更严格约束。

### 7.2 各层故障作用

| 机制 | 当前行为 | 不能保证的事项 |
| --- | --- | --- |
| Host 打开/连接异常 | 有限重连；失败撤销会话；达到总尝试数报告 RECONNECT_EXHAUSTED | 不能保证 MCU 收到最后 HOLD |
| HELLO 超时 | HANDSHAKE_TIMEOUT 后断连 | 不是固件运动 watchdog |
| STATUS 超时 | STATUS_STALE，封锁并尽力 HOLD；不一定关闭串口 | 有 ACK 也不能刷新状态；没有自动 MCU 安全确认 |
| Host 命令超时 | last_velocity 超时清意图，COMMAND_TIMEOUT；排队旧授权命令不写出 | 没有主机进程时这段逻辑不能运行 |
| wire valid_for_ms | WALK/START/HEARTBEAT 带相对 TTL | 固件本地最大值、起算/续期细节和过期收尾尚需实现 |
| ACK 超时 | 同序号有限重传，最终拒绝/封锁；HOLD 仅报超时 | ACK 不证明轨迹或停止完成 |
| 授权失效 | 租约到期、反馈陈旧/非法或安全状态变化后撤销旧命令 | 无单独 wire REVOKE；STM32 必须有自己的模式/授权记忆及保护 |
| MCU 本地通信 watchdog | 必须独立覆盖进程死亡、USB 拔出、卡死和停帧 | **当前 Host 代码不能提供 MCU 阈值或证明 MCU 已实现** |

MCU 本地 watchdog 的最大允许 TTL、最慢 STATUS 周期、心跳间隔、超时动作和停止/落地最大时间均未冻结。Host JSON 的 valid_for_ms 不得被允许无限延长固件硬上限。参考 E34 的 350 ms motion timeout、500 ms link timeout属于旧无线链路；MCU 主循环 watchdog 也不是 Host 指令 watchdog，均不能直接宣称保护新 UART。

## 8. STM32F429 适配接口与双方职责

### 8.1 固件侧接口要求

参考固件事实来自既有只读审查（参考 Commit `bec442861ebd0d570e95d8a4d82873394d8f0f7c`），不代表当前其他固件版本。保留 E34 遥控协议；Host UART 使用独立串口，不与 E34 共用 UART/解析器/发送调度。参考 UART8 属于 E34，USART1 当前为调试 TX，没有本协议接收器；选用后须处理原调试输出冲突，不可仅接线即用。

复用 `horse_app`：

| 接口/职责 | 当前事实 | 接入限制 |
| --- | --- | --- |
| horse_app_set_motion(app, forward, turn) | 输入归一化 [-1,1] | 对应受授权 WALK；不接受 SI 速度或 vy，不绕过执行器状态 |
| horse_app_request_jump | 存在，受构建能力影响 | command_id 映射未确认，不能直接默认编号 6 |
| horse_app_request_forward_jump | F429 参考实现存在 | command_id 映射未确认，不能直接默认编号 7 |
| horse_app_set_static_support | 支撑请求接口 | 不能凭调用成功宣称 safe_stance |
| horse_app_get_snapshot | 提供部分运行/电机/支撑请求状态 | 缺完整 measured_stopped/support_confirmed/releasable/急停/人工许可接口 |
| horse_app_get_jump_status | 阶段、失败与预检信息 | 历史 COMPLETE 必须绑定本次请求才能用于完成 |
| horse_app_request_stop | STOPPING、驱动 STOP/退出使能 | 不能当 HOLD 或通用安全取消 |
| horse_app_request_enable | 存在显式使能过程 | 本 UART v1 没有 ENABLE 命令，不可 HELLO 后自动调用 |

需要固件实现：帧接收/CRC/有界 JSON、会话/序号/重复请求、HELLO 与真实 features、模式互斥、授权和 TTL、技能关联、真实 STATUS、FAULT、支撑 HOLD、安全取消、本地 watchdog、物理急停与人工许可。未知技能编号必须拒绝；不自行实现一套主机关节轨迹，不绕过 horse_app 的状态和故障检查。

E34 与 Host 的普通运动输入必须互斥；物理急停始终优先。输入选择、人工接管、Host 断线后是否允许转回 E34、切换等待条件尚需双方确认，不能让两个接口轮流覆盖同一个运动目标。

### 8.2 职责边界

| Ubuntu Host | STM32 | 共同契约 |
| --- | --- | --- |
| 定位、导航、路线、任务与出口验收 | 实际电机/步态/技能执行 | ACK、执行中、完成、停稳、支撑、可释放六类事实分离 |
| 仲裁 owner/mission/generation 与新鲜度 | MCU 本地输入互斥、模式和故障门控 | 不复用旧会话、旧命令或历史完成状态 |
| SI 到归一化标定、限幅、禁止横移 | 归一化映射校核与实际能力声明 | 速度方向/幅度、TTL、安全判据双方确认 |
| 唯一串口 writer、有限重试、撤销旧队列 | 有界 parser、去重、ACK、STATUS、watchdog | 原始字节 CRC、长度、大小端、版本一致 |
| 只采纳真实新鲜反馈，不以 ACK 推断安全 | 实测急停/使能/停止/支撑/释放 | 无证据不放行；通信故障不能依赖 Host 还活着 |

## 9. 已发现的文档/实现差异与未闭合处

以下是基线源码位置；行号以基线为准，函数名可在后续版本定位。此次没有修正业务代码，也未悄悄选择一种行为覆盖冲突。

| 项目 | 差异/实际行为 | 对固件的影响与来源 |
| --- | --- | --- |
| 超时默认 | 原指南第 6 节的 0.3/0.3/0.2 对应 YAML；代码兜底是 0.4/0.4/0.3 | 必须说明启动参数来源；gateway_core.py:8 DEFAULTS；config/stm32.yaml |
| START 字段 | 原指南消息表只列 command_id 与 C；实际还发 owner/mission_id/generation/valid_for_ms | 完整发送字段见 gateway_core.py:357 `_try_skill` |
| STATUS 关联必填性 | 原指南将关联列入必填；parser 中 request_id/runner_session/step_index 可选 | 结构可通过不等于能完成任务；protocol.py:136 `validate_status`；gateway_core.py:405 `receive`；GatewayTransport:242 `_matches` |
| HELLO.host | 当前 connect 总发送 host；最简既有向量只含 protocol_version | MCU 对缺 host 的兼容接受策略未由实际固件确定；protocol.py:162 附近 TEST_VECTOR_HELLO；gateway_core.py:84 `connect` |
| FAULT.code | 原指南说 code；接收器允许缺失并转字符串 | 不能声称代码已有严格 schema；gateway_core.py:405 `receive` |
| active 判据 | 指南称真实执行器活动；Host 等 nav_active/skill_active 才发 WALK | 需要“实际已接管/就绪”而非“速度非零”的固件定义；gateway_core.py:197、216 |
| START 就绪 | `_ensure_mode` 要求相应 active；`_try_skill` 最后发 START 仅检查 mode=SKILL 等条件 | 固件必须检查实际就绪；本次不修改判定；gateway_core.py:357 |
| CANCEL 关联 | 指南说取消关联技能；实际 cancel 生成新 request_id，且没有被取消请求 ID | 必须共同确定取消目标和终态回显；GatewayTransport:144、218；gateway_core.py:280 |
| HOLD 完成 | 网关验收安全 HOLD 不要求 skill_state=succeeded；Hurricane hold 完成要求 succeeded 和真实释放 | 只报 idle 会卡在上层步骤；gateway_core.py:405；GatewayTransport:247 `_notify` |
| 重复 ACK | 指南“回放原 ACK”易被理解成整帧重放；Host 丢弃所有非递增 MCU 序号 | 重复确认使用新 MCU sequence 与原 acked_sequence；gateway_core.py:405 |
| STOP 授权 | Host 内部验证有效 SKILL 租约，但线上 STOP 只发 C | 不能声称固件从此帧直接检查 generation；gateway_core.py:280 |
| ACK/HOLD 超时 | 概括说超时封锁，但 HOLD ACK 耗尽仅报告 timeout，不自动 STOP/断连 | 保护依靠真实状态与 MCU 本地策略；gateway_core.py:490 `tick` |
| E34 关系 | 原指南首段“不使用 E34”指自动化路径，不应理解为删除固件 E34 | 本规范保留 E34，要求独立 UART 与互斥；既有 STM32_FIRMWARE_AUDIT.md |

## 10. 待双方确认的固件与安全约定

以下未定项不会通过写文档自动变成已实现能力；在确认前不能宣称整条实机动作链已闭合。

1. **UART 与电气接口**：F429 板卡实际型号/引脚、电平、可用独立 UART、DMA/中断与缓冲大小、调试口迁移、USB 串口复位/掉线行为。
2. **输入控制权**：E34/Host 的互斥来源、物理选择与人工接管规则、每次会话如何绑定 mission/generation、何时拒绝旧代次，以及线上不带授权的 HOLD/CANCEL/STOP 如何按安全意图处理。不能增加未实现字段冒充已解决。
3. **模式与 active**：NAV/SKILL 执行器实际就绪的定义、SET_MODE ACK 到 ready 的期限、接受 START 时必须满足的状态，以及 active 与 accepted/running 的关系，避免等待第一条 WALK 的互锁。
4. **取消/保持关联**：cancel 使用新 ID；如何唯一对应正在执行的 START；先 HOLD 后 CANCEL 的顺序与重复处理；原 START 终态、cancel 终态、后续 HOLD 成功如何分别回显 C。当前没有 target_request_id，不得虚构该字段已经存在。
5. **动作安全收尾**：预检、蹲伏、推蹬、腾空、落地、恢复各阶段对 HOLD/CANCEL/超时/新 session 的响应；是否允许延迟到安全阶段；失败如何上报。STOP 卸力必须与安全支撑明确区分。
6. **测量与安全判据**：stopped 的速度与稳定时长、支撑承重/触地/姿态和电机条件、control_releasable 的判据；physical estop 与 human_enable 的真实硬件来源。未知状态统一不能声称安全。
7. **技能能力映射**：6、7 及 8…18 哪些编号映射到哪个 horse_app 接口、编译配置、前置条件和失败码；不支持即不声明，不能推测桥梁/高墙等组合技能是固件已有单动作。
8. **时间契约**：MCU 最大 TTL、最慢 STATUS 周期/抖动、合法输入更新 watchdog 的精确规则、SET_MODE 无 TTL 时的保护、START 与 HEARTBEAT 如何续期、重复/迟到命令不能延长 TTL、无通信时安全动作硬上限。Host 的 20 Hz 不是 MCU 发送频率规定。
9. **会话与去重**：MCU 重启握手、序号耗尽重连、重复结果缓存大小和期限、同序号不同内容、跨新 session 动作未收尾如何处理；ACK 新帧序号需与 STATUS/FAULT 串行分配。
10. **schema 与错误表**：每字段 MCU 存储/数值上限、未知字段策略、非法/未知请求如何反馈、reason/code 的统一枚举、FAULT.code 是否严格必填、host 标识兼容、故障锁存/清除方式。当前没有故障清除命令。
11. **速度标定与真实动作验证**：forward/turn 的符号、非线性、正反向差异和安全上限；当前 0.5 m/s、1 rad/s 仅示例。支撑、取消、急停、watchdog、复位与实际通信均待硬件验证。

## 11. 源码追溯索引

本规范主体无需阅读源码即可理解；以下用于复核实现边界，不是额外必需的协议定义。

- [protocol.py](../real/quadruped_stm32_bridge/quadruped_stm32_bridge/protocol.py)：帧格式、CRC、流式 parser、STATUS schema（24、50、82、136 行附近）。
- [gateway_core.py](../real/quadruped_stm32_bridge/quadruped_stm32_bridge/gateway_core.py)：参数（8）、会话（84）、授权/门控（131–180）、模式/速度（197–264）、技能/HOLD（269–370）、发送/接收/ACK/超时（371–525）。
- [serial_gateway.py](../real/quadruped_stm32_bridge/quadruped_stm32_bridge/serial_gateway.py)：唯一串口 worker（58）、ROS 授权/速度边界（164、181）、时钟与发布（218）。
- [stm32.yaml](../real/quadruped_stm32_bridge/config/stm32.yaml)：随包参数覆盖值；与代码默认不同的三项已列于第 7 节。
- [hurricane_gateway_transport.py](../common/robocon_mission/robocon_mission/hurricane_gateway_transport.py)：request_id 构建（144）、方向/离散分流（174）、取消（218）、关联/完成（242–306）。
- [telemetry_adapter.py](../real/quadruped_stm32_bridge/quadruped_stm32_bridge/telemetry_adapter.py)：新鲜真实反馈到 MotionTelemetry，不生成模拟安全证据。
- [STM32 固件只读审查](STM32_FIRMWARE_AUDIT.md)：固定参考固件版本的 API、引脚和安全能力缺口。
- [STM32 接入指南](STM32_BRIDGE_GUIDE.md)：部署/运行说明；线协议定义统一引用本文件。
