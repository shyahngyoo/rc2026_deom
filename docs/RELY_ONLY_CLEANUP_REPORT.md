# rely-only 第一阶段清理审核报告

状态：本阶段的软件静态检查、构建及最终 ROS 2 回归通过，满足提交门槛。首轮失败和复测结果均保留；本报告不证明真实 STM32、传感器或电机的整机安全。

## 1. 分支与修改基线

- 仓库：`https://github.com/shyahngyoo/rc2026_deom`。
- 工作分支：远端已经存在的 `rely-only`，未新建远端分支。
- 修改前 Commit：`9474d31de176f95a2e8aa5a088e1a9a6821a86d2`。
- 修改前工作区干净；本地 `main` 和初始远端 `main` 均为该 SHA。
- 未使用 hard reset、强制推送或自动合并。
- 未开发 STM32 串口驱动，未使能真实运动输出。

依赖基线记录了 8 个功能包、4 份 Python 可执行注册、23 个 Launch 的声明/依赖以及 CMake 程序安装规则。65 个受保护的任务、仲裁、定位、接口、Nav2 插件与行为树文件逐一保留原内容；没有修改其核心行为或安全参数。

## 2. 修改文件清单

| 文件 | 修改与目的 |
|---|---|
| `common/robocon_mission/launch/mission.launch.py` | 只保留 MissionManager；删除 Mock 节点、条件与参数；真实时间、手动启动、安全模式为默认，继续支持路线、证据、预检查和时限覆盖。 |
| `common/robocon_mission/setup.py` | 删除 3 个旧 Mock/demo console 注册，保留真实 Mission、定位健康及 Hurricane 程序。 |
| `common/go2_nav_bringup/CMakeLists.txt` | 删除旧 Mock Action 脚本安装规则；保留控制器、BT 插件、导航检查工具及所有 CTest。 |
| `common/robocon_mission/test/test_mission_integration.py` | 从源码测试夹具加载 Mock，替代已安装运行程序路径；原有 30 个测试函数及其安全断言保留。 |
| `common/robocon_mission/test/test_simulation_launch.py` | 保留历史文件及 5 项契约，改为检查真实 Mission、当前实机 Nav2 模板和随仓库提供的路线夹具，不再依赖不存在的 Gazebo 包。 |
| `tools/verify_real.bash` | 取消对上述 5 项 Launch 契约的整组忽略。 |
| `tools/tests/test_real_only_cleanup.py` | 新增 17 项清理/安装注册检查，防止删除的程序重新进入运行链。 |
| `common/robocon_mission/test/test_real_launch_isolation.py` | 实际启动正式 Mission，检查默认参数、IDLE、缺失真实 preflight 时拒绝启动，以及没有模拟状态和电机输出。 |
| `README.md` | 说明自制四足/STM32 硬件范围，下载明确选择 `rely-only`，链接本报告。 |
| `common/robocon_mission/README.md` | 更新正式 Launch 的默认值及测试夹具边界。 |
| `common/go2_fastlio_localization/README.md` | 删除已经清理的兼容入口说明，指向真实 bringup。 |
| `common/go2_mission_backends/README.md` | 说明保留模拟后端的安全测试用途，实机入口不包含它。 |
| `common/robocon_mission/HURRICANE_PORT.md` | 删除已移除的 Mock 程序名，继续要求技能 Action server 唯一。 |
| `docs/REAL_ROBOT_GUIDE.md` | 删除已移除的参数；安装/更新明确使用 `rely-only`。 |
| `docs/DEPLOYMENT.md` | 说明旧安装残留与独立工作空间升级方法。 |
| `docs/VALIDATION.md` | 将旧成绩标明为历史发布记录，链接本次新验证，避免误用旧忽略策略。 |
| `docs/RELY_ONLY_CLEANUP_REPORT.md` | 本报告：记录依赖判断、变更、测试迁移、原始失败和最终验收。 |

各 `package.xml` 已检查，没有为了整洁删除仍有真实使用方的依赖。例如 `rosgraph_msgs` 仍被定位客户端和 Hurricane 使用，不能仅因删除演示节点就移除。

## 3. 移到测试夹具的文件

| 原位置 | 新位置 | 保留理由 |
|---|---|---|
| `common/robocon_mission/robocon_mission/mock_motion_services.py` | `common/robocon_mission/test/fixtures/mock_motion_services.py` | 集成测试仍需控制权、运动状态、心跳和重定位故障夹具。 |
| `common/go2_nav_bringup/scripts/mock_obstacle_skill.py` | `common/robocon_mission/test/fixtures/mock_obstacle_skill.py` | 保留 Action 成功、失败、取消和未释放控制权契约测试。 |

两文件内容按 Git 的 LF 规范化表示一致，仅迁移位置；测试夹具不注册为 ROS 可执行程序，也不作为运行包安装。

## 4. 删除文件清单与依赖判断

| 文件 | 删除原因 |
|---|---|
| `common/robocon_mission/robocon_mission/mock_localization_ok.py` | 仅由旧 Mock/demo 入口使用；删除无条件模拟定位成功服务。 |
| `common/robocon_mission/robocon_mission/mock_mission_demo.py` | 废弃演示，正式运行与安全回归均不依赖。 |
| `common/robocon_mission/config/mock_closed_loop_route.yaml` | 只有上述演示使用，随演示删除。 |
| `common/robocon_mission/launch/simulation.launch.py` | 无真实使用方，转发到未提供的仿真包。 |
| `common/go2_nav_bringup/launch/localization.launch.py` | 同上；真实 GICP/bringup 入口保留。 |
| `common/go2_nav_bringup/launch/mapping.launch.py` | 同上；真实建图入口保留。 |
| `common/go2_nav_bringup/launch/navigation.launch.py` | 同上；真实 Nav2 启动与插件保留。 |
| `common/go2_fastlio_localization/launch/localization.launch.py` | 无真实使用方的仿真兼容转发。 |
| `common/go2_fastlio_localization/launch/mapping.launch.py` | 同上。 |
| `common/go2_fastlio_localization/launch/odometry.launch.py` | 历史仿真配置入口，默认仿真时间并引用未提供的配置；实际里程计使用 real bringup。 |
| `common/go2_fastlio_localization/launch/real_fastlio.launch.py` | 无调用方的过期实机别名；实际 FAST-LIO Launch 保留。 |
| `common/go2_fastlio_localization/launch/real_lidar.launch.py` | 无调用方的过期别名；真实 MID-360 Launch 保留。 |
| `common/go2_fastlio_localization/launch/real_mapping.launch.py` | 无调用方的过期别名；真实 mapping/odometry 保留。 |
| `real/go2_real_localization/launch/bringup.launch.py` | 无调用方的过期转发；真实 localization 及组合入口保留。 |

这些文件经过代码、安装、测试和文档引用检查后删除。保留的真实链为：机载 real.launch → 实际 MID-360/FAST-LIO/GICP bringup → 状态与定位适配；navigation 另外组合真实 Nav2，但保持私有速度输出。

## 5. 仍保留的 simulation/mock/test 内容

- `simulation_backend.py`、`backend_simulation.launch.py` 及其注册：原有仲裁、时钟、定位、越障故障与独立闭环安全测试依赖，保留显式 simulation 防护和 `/backend_sim/cmd_vel` 隔离。正式 Mission 与机载入口不包含它们。
- `test/fixtures/mock_*.py`、`patrol_mock_actions.yaml`、合成地图：仅测试夹具，未作为真实状态或现场路线部署。
- `test_simulation_launch.py`：名称保留用于追溯，本分支运行的是迁移后的实机默认值/精度/路线契约，已经取消整组忽略。
- `test_live_mock_feedback_cannot_unlock_real_motion`：有价值的反向测试，证明伪造输入不会解锁机载运动，不能因名字有 mock 就删除。
- `cyclonedds_sim.xml`：DDS 分片与进程容量配置，实机也使用；不启动模拟节点或仿真时钟，内容未修改。
- `BUILD_TESTING` 下的 Nav2/GICP 原生测试与源码安全测试全部保留。

残留引用分类的机器可读结果保存在验证目录 `residual_references_final.json`。除显式隔离的安全测试后端外，没有正式运行入口依赖已删除模块或仿真包。

## 6. 原有测试如何迁移

修改前直接运行历史 Launch 测试，5 项全部因外部仿真包不存在而失败；旧验证脚本会整组忽略。现在保留 5 项检查并重新纳入验证：

| 原意图 | 现在的检查 |
|---|---|
| 巡航/精确接近容差、旋转与速度一致性 | 原精度、停稳、控制器和 planner 断言全部保留，参数源改为当前实机模板。 |
| 手动启动不被其他启动流程覆盖 | 检查正式入口默认不自动启动、显式调用方覆盖仍可配置；正式入口不再包含嵌套 Nav2/仿真启动器。 |
| 导航/定位未就绪不得开始任务 | 检查 preflight/证据参数传递，并新增实际 ROS 测试证明 preflight 不存在时拒绝 `/mission/start`。 |
| 不能重复启动替身服务 | 正式入口只能有 MissionManager，不能包含 Mock 或模拟机器人。 |
| 巡航几何与三个出口验收 | 使用仓库自带 29 点夹具保留几何与三个出口断言；原独立路线测试和完整巡航集成测试继续运行。 |

30 个原 Mission 集成测试函数均保留，仅替换两个测试夹具的加载路径。没有删除失败的安全用例，没有改 MissionFSM、MotionArbiter、ControlHandoff 或定位门限。

## 7. 验证状态

### 最终结果

| 检查 | 实际结果 |
|---|---|
| Python 语法检查 | 116 个 Python 文件通过，不执行模块或创建节点。 |
| 核心完整性 | 65 个受保护文件 SHA-256 与修改前一致；核心行为、接口、Nav2 和定位算法未修改。 |
| package.xml / 安装注册 | 8 份 XML 可解析；所有 console 目标与 CMake 程序存在；安装后的 8 包均无已撤销 Mock 注册。 |
| Launch 参数加载 | 剩余 12 个 Launch 的 `--show-args` 全部返回成功，包括正式 Mission 和实机入口；不会启动设备。 |
| 针对性清理/隔离检查 | 23 passed，0 failed，0 skipped，包含迁移的 5 项、17 项新增静态契约及 1 项实际 ROS 隔离测试。 |
| 完整验证脚本最终复测 | **555 passed，0 failed，0 errors，0 skipped**；369.78 s；包含 wall-clock 与受控时钟用例。 |
| Nav2 CTest | **6/6 通过**，1.93 s，含 3 个原生 C++ 与 3 个 Python 契约。 |
| 定位工具与机载补充测试 | **24 passed，0 failed，0 skipped**，4.54 s；包括原 ICP/里程计测试、doctor 反向测试和实际 Nav2 图隔离测试。 |
| 独立进程闭环 smoke | 通过，24.87 s；两障碍、返航、FINISHED、SAFE_HOLD、最终零速；7 个子进程全部回收。 |
| 正式 Mission 运行隔离 | 实际节点只有 manager/probe；默认真实时间、非模拟、非自动；缺少真实 preflight 时拒绝启动；电机发布者为 0。 |
| 实际 Nav2 图 | 原有测试确认 lifecycle ACTIVE、私有输出非零、平滑器来源、没有 `/cmd_vel` 发布者以及取消终态。 |
| `git diff --check` | 通过。 |
| STM32 / 电机 / 真实雷达硬件 | **未执行**；本次没有连接硬件，未发实际运动命令。 |

555 项和补充 24 项为不同的 pytest 集合；针对性 23 项包含在完整集合中，不重复计总数。Nav2 图使用真实 Nav2 程序与合成 TF/odom，并非真实场地运动。

### 原始失败与复测

已知并保留的原始记录：

- 修改前历史 Launch 测试：5 项失败，原因是所需外部仿真包不存在。
- 新增测试的初次执行发现两个测试探针问题：显式 rclpy Context 未绑定独立 Executor；全局服务名图误将客户端服务名当作服务端。只修测试探针，保留安全断言，原始失败日志已归档。
- 针对性清理/正式 Launch 测试修正后：23 项通过。
- 11 个功能包及依赖构建通过；安装后的 8 包注册检查无旧 Mock 程序；剩余 12 个 Launch 的 `--show-args` 全部通过。
- 完整回归首轮：552 通过、3 失败。失败分别为 wall-clock 正向闭环、wrong_exit 注入和技能超时取消。wall-clock 记录出现约 1.42 秒实时时钟跳变；另外两项在进入预期测试阶段前安全中止。
- 未修改原有断言或超时，单独复测上述 3 项均通过；完整回归仍须以最终整套结果为准，不能将首轮宣称为通过。

完整复测使用同一份源代码和安全断言，没有对失败的旧用例加 skip、xfail 或重试装饰器，也没有放宽超时、运动状态或定位阈值。复测监督记录仍观察到本机实时时钟与单调时钟的差异，不能据本次通过认定宿主时序问题已经消失。

构建和日志均在 WSL Ubuntu 22.04 / ROS 2 Humble 执行。旧生成物先备份再重新生成，未删除源码或覆盖未提交修改。11 个 ROS 包构建包含项目 8 包及相关第三方 ROS 依赖；Livox SDK 复用此前已安装的固定版本前缀。本次不是一次全新系统 apt 安装验收。

### 实际执行命令

本次仓库为 `/home/zxy/rc2026_deom`，验证工作空间为 `/home/zxy/rc2026_release_ws`。以下是构建脚本中的实际参数：

```bash
repo=/home/zxy/rc2026_deom
ws=/home/zxy/rc2026_release_ws
sdk="$ws/install/livox-sdk2"
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1 MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "$ws/log" build \
  --base-paths "$repo/common" "$repo/real" \
    "$ws/src/livox_ros_driver2" "$ws/src/FAST_LIO" "$ws/src/fast_gicp" \
  --build-base "$ws/build" --install-base "$ws/install" \
  --symlink-install --executor sequential \
  --packages-up-to go2_onboard_bringup go2_nav_bringup go2_mission_backends \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DBUILD_VGICP_CUDA=OFF -DBUILD_apps=OFF -DBUILD_test=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF -DDISTRO_ROS=humble \
    "-DLIVOX_LIDAR_SDK_LIBRARY=$sdk/lib/liblivox_lidar_sdk_shared.so" \
    "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$sdk/include" "-DCMAKE_INSTALL_RPATH=$sdk/lib"
```

`BUILD_test=OFF` 是第三方 fast_gicp 的数据集测试选项；项目原生测试由 `BUILD_TESTING=ON` 保留并实际执行。

完整验证首轮及最终复测：

```bash
bash tools/verify_real.bash \
  --workspace /home/zxy/rc2026_release_ws \
  --report-dir /home/zxy/rc2026_release_ws/log/verification/rely_only_cleanup_20261015/full \
  --include-wall-clock
bash tools/verify_real.bash \
  --workspace /home/zxy/rc2026_release_ws \
  --report-dir /home/zxy/rc2026_release_ws/log/verification/rely_only_cleanup_20261015/full_retry \
  --skip-build --include-wall-clock
```

`--skip-build` 仅复用已通过的同份构建；最终复测没有跳过任何 pytest 用例，CTest 和独立 smoke 仍由工具执行。

另外在已 source 的验证终端中，使用 localhost CycloneDDS、独立 domain 执行：

```bash
python3 -m pytest -q \
  common/robocon_mission/test/test_simulation_launch.py \
  common/robocon_mission/test/test_real_launch_isolation.py \
  tools/tests/test_real_only_cleanup.py
python3 -m pytest -q \
  common/go2_fastlio_localization/test \
  real/go2_onboard_bringup/test/test_doctor_ros.py \
  real/go2_onboard_bringup/test/test_planning_graph.py
ctest --test-dir /home/zxy/rc2026_release_ws/build/go2_nav_bringup --output-on-failure
```

每个剩余 Launch 都执行了 `ros2 launch <package> <file> --show-args`。实际正式 Mission 运行隔离由 `test_real_launch_isolation.py` 执行，没有启动雷达或底层控制器。

完整原始日志在验证工作空间 `log/verification/rely_only_cleanup_20261015/`，其中：

- `baseline.json`、`dependency_inventory_baseline.json`、`protected_core_baseline.json`：修改前基线。
- `baseline_obsolete_launch_tests.log`：原有 5 项失败原文。
- `targeted_first_failure/`、`targeted_second_failure/`：新增探针问题与原始错误。
- `full/`、`full_verify_console.log`：首轮完整失败原文、JUnit 与后端观测。
- `failed_case_recheck.*`：3 项独立复测与时钟采样。
- `full_retry/`：最终 555 项、CTest 与 smoke 的完整结果。
- `supplemental_tests.xml`、`supplemental_console.log`：额外 24 项结果。
- `installed_registry.json`、`launch_arguments.json`、`residual_references_final.json`、`verification_summary.json`：注册、参数与分类审核。

日志没有作为现场数据提交进公开仓库。

## 8. 升级与剩余风险

从旧分支复用同一个 `build/install` 时，colcon 不保证清除以前安装的可执行程序和 Launch；只更新源码不足以保证没有残留。本次验证先备份受影响的 4 个包的 build/install 再构建。现场建议安装到新的专用工作空间，保留旧安装以便回退：

```bash
cd "$HOME/rc2026_deom"
git branch --show-current
bash tools/install_real.bash --workspace "$HOME/rc2026_rely_only_ws" --jobs 2
source /opt/ros/humble/setup.bash
source "$HOME/rc2026_rely_only_ws/install/setup.bash"
```

确认分支是 `rely-only`，后续所有终端使用所选工作空间，不能混合 source 旧安装。现场地图与外置配置先备份。

STM32 真实状态适配、生产速度命令入口、唯一仲裁输出、真实技能反馈与完整实机 bringup 仍未接通。当前 Nav2 保持私有诊断输出；本次清理不代表机器人已具备整场自主运行或经过运动安全认证。

## 9. Git 差异与提交

提交前已核对仅 `rely-only` 工作树有本次修改，`main` 引用仍为基线 SHA。以下 diff 统计包含本报告自身；最终 Commit SHA 在提交后的交付答复与 `git log -1` 中提供。未执行自动合并或强制推送。

<!-- DIFF_STAT_BEGIN -->
```text
 README.md                                          |   4 +-
 common/go2_fastlio_localization/README.md          |   2 +-
 .../launch/localization.launch.py                  |  17 --
 .../launch/mapping.launch.py                       |  17 --
 .../launch/odometry.launch.py                      |  17 --
 .../launch/real_fastlio.launch.py                  |  17 --
 .../launch/real_lidar.launch.py                    |  17 --
 .../launch/real_mapping.launch.py                  |  17 --
 common/go2_mission_backends/README.md              |   2 +
 common/go2_nav_bringup/CMakeLists.txt              |   2 -
 .../go2_nav_bringup/launch/localization.launch.py  |  17 --
 common/go2_nav_bringup/launch/mapping.launch.py    |  17 --
 common/go2_nav_bringup/launch/navigation.launch.py |  17 --
 common/robocon_mission/HURRICANE_PORT.md           |   2 +-
 common/robocon_mission/README.md                   |   6 +-
 .../config/mock_closed_loop_route.yaml             |  22 --
 common/robocon_mission/launch/mission.launch.py    |  27 +--
 common/robocon_mission/launch/simulation.launch.py |  17 --
 .../robocon_mission/mock_localization_ok.py        |  32 ---
 .../robocon_mission/mock_mission_demo.py           | 145 ------------
 common/robocon_mission/setup.py                    |   3 -
 .../fixtures}/mock_motion_services.py              |   0
 .../test/fixtures}/mock_obstacle_skill.py          |   0
 .../test/test_mission_integration.py               |  15 +-
 .../test/test_real_launch_isolation.py             |  88 +++++++
 .../robocon_mission/test/test_simulation_launch.py | 133 +++++------
 docs/DEPLOYMENT.md                                 |   2 +
 docs/REAL_ROBOT_GUIDE.md                           |   9 +-
 docs/RELY_ONLY_CLEANUP_REPORT.md                   | 258 +++++++++++++++++++++
 docs/VALIDATION.md                                 |   2 +
 .../go2_real_localization/launch/bringup.launch.py |  17 --
 tools/tests/test_real_only_cleanup.py              | 125 ++++++++++
 tools/verify_real.bash                             |   2 +-
 33 files changed, 569 insertions(+), 499 deletions(-)
```
<!-- DIFF_STAT_END -->
