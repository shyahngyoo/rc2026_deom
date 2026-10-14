# 源码与依赖来源

本发行从 icecream127/go2-mid360-fastlio-nav2 当前工作树整理实机包，并加入 Mission 后端及机载部署工具。保留各文件现有版权头；各 ROS 包的许可声明以其 package.xml 和文件头为准（BSD-3-Clause / Apache-2.0）。新增 Mission 后端声明 Apache-2.0，文本见 LICENSES/Apache-2.0.txt。

外部算法和驱动不在本仓库中打包源码，由固定版本安装：Livox-SDK/Livox-SDK2、Livox-SDK/livox_ros_driver2、XjuHurricaneQuadVision/FAST_LIO、koide3/fast_gicp。其许可按上游仓库执行；版本与项目补丁在 dependencies.repos 和 patches 中列出。

测试目录中的小型二维地图是原仿真示例测试数据，禁止作为默认真实场地地图。本仓库不包含真实场地 PCD、雷达录包、凭据或本机编译结果。
