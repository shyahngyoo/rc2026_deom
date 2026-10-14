# MID-360 的固定 PCD GICP 定位

本包运行 fast_gicp::FastGICP（CPU），使用同步点云和 FAST-LIO 里程计，在给定 PCD 上求解 map→odom。定位入口不启动电机或越障执行器。

## 安装与构建

环境为 Ubuntu 22.04、ROS 2 Humble。先按仓库根 README 运行 tools/install_real.bash，准备固定版本 fast_gicp 及实机工作空间依赖。fast_gicp 的固定提交和来源记录在 fast_gicp.repos。

以下工作空间是示例，使用实际安装目录：

~~~bash
cd ~/rc2026_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --packages-up-to go2_real_localization --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/local_setup.bash
~~~

BUILD_TESTING=ON 同时构建 synthetic_gicp_inputs，供无硬件验收。测试通过 ament 查找安装位置，不依赖用户名或原开发工作空间。

## 输入与启动

驱动和 FAST-LIO 参数存于 go2_real_bringup。已有驱动和 FAST-LIO 在运行时，可以只启动定位：

~~~bash
ros2 launch go2_real_localization localization.launch.py \
  map_path:=$HOME/maps/field.pcd
~~~

map_path 必须是用户已有且坐标明确的 PCD，仓库不附带比赛地图。雷达静止完成 IMU 初始化后，用 RViz 的 2D Pose Estimate 提供初猜。不同高度或倾斜姿态应通过 initial_pose=[x,y,z,roll,pitch,yaw] 和 initial_pose_enabled 配置初猜。PCD 的 z=0 不一定是地面，初猜描述 IMU 机体在地图中的位姿。

| 接口 | 含义 |
|---|---|
| /cloud_registered_body | 已去畸变的 IMU 坐标 PointCloud2 |
| /Odometry | odom→livox_frame 的 Odometry |
| /initialpose | map 中的 IMU 初猜 |
| /gicp/evidence | 原子 LocalizationEvidence，包含来源、epoch、sequence、测量时间、accepted 和位姿 |
| /gicp/input_status | 原始 cloud/odom 到达计数、同步计数和最后测量时间 |
| /gicp/map、/gicp/aligned_cloud | 参考地图和接受后的对齐扫描 |
| /gicp/pose、/gicp/status、/gicp/valid | 兼容观测接口 |

两路输入按原测量时间 ExactTime 配对。求解 T_map_body 后发布 T_map_odom = T_map_body × inverse(T_odom_body)。TF 链为 map→odom→livox_frame，机体与 IMU 外参需在实机上测量。

Mission 使用 /gicp/evidence 经 localization_adapter 的原子证据和新鲜度门控。独立 Bool 与 Pose 不能组合成可信位姿。GICP 没有标定协方差，covariance_known=false，零协方差不能解释为零误差。初猜变化和时间回退增加 epoch，accepted 使用原测量时间。

## 质量检查与故障

体素、配准周期、内点比例、RMSE、修正量和输入年龄见 config/gicp.yaml。RMSE 是内点最近邻距离的均方根，不能视为定位精度。匹配需要收敛、足够内点、误差与修正量合格且输入未过期。重复几何仍可能产生误匹配，需要真实场地验收。

输入过期、时钟回退、frame 错误和匹配拒绝发布失效证据。时钟回退后重新给初猜，FAST-LIO 重启时应重启定位节点。通过 input_status 区分无点云、无里程计、未配对与质量拒绝，不通过放宽质量阈值掩盖输入问题。

## 无硬件验证

在已安装的工作空间运行：

~~~bash
bash src/rc2026_deom/tools/verify_real.bash
~~~

非标准源码目录可传 --workspace。工具只加载当前工作空间安装，使用专属 localhost ROS domain 和进程级 CycloneDDS 分片配置，不启动真实雷达、SDK 或电机。

真实 native GICP 用例临时生成1800点 PCD，独立 C++ 输入进程持续发布同步原测量时间的完整点云和里程计。真实 GICP 必须接受至少三次不同测量，并在新初猜后接受新 epoch 和初猜之后的测量。测试还覆盖 Mission、停稳、控制交接、越障模拟执行器和独立出口验收。合成几何与软件底层只证明接口和时序，真实定位精度、计算负载、机载网络和物理运动仍需实机验证。
