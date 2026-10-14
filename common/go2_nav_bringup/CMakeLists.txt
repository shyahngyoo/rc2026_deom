cmake_minimum_required(VERSION 3.5)
project(go2_nav_bringup)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(pluginlib REQUIRED)
add_library(go2_goal_critics SHARED src/recoverable_rotate_to_goal.cpp)
ament_target_dependencies(go2_goal_critics dwb_core dwb_critics pluginlib)
pluginlib_export_plugin_description_file(dwb_core dwb_critics.xml)
install(TARGETS go2_goal_critics LIBRARY DESTINATION lib)
add_library(go2_metric_goal_critic SHARED src/metric_goal_distance.cpp)
ament_target_dependencies(go2_metric_goal_critic dwb_core pluginlib)
pluginlib_export_plugin_description_file(dwb_core metric_goal_critic.xml)
install(TARGETS go2_metric_goal_critic LIBRARY DESTINATION lib)
add_library(go2_precision_pose_critic SHARED src/precision_pose_critic.cpp)
target_compile_features(go2_precision_pose_critic PRIVATE cxx_std_17)
ament_target_dependencies(go2_precision_pose_critic dwb_core pluginlib nav2_util)
pluginlib_export_plugin_description_file(dwb_core precision_pose_critic.xml)
install(TARGETS go2_precision_pose_critic LIBRARY DESTINATION lib)


add_library(pause_replanning_near_goal_bt_node SHARED src/pause_replanning.cpp)
target_compile_features(pause_replanning_near_goal_bt_node PRIVATE cxx_std_17)
target_compile_definitions(pause_replanning_near_goal_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(pause_replanning_near_goal_bt_node
  behaviortree_cpp_v3 rclcpp tf2_ros nav2_util geometry_msgs)
install(TARGETS pause_replanning_near_goal_bt_node LIBRARY DESTINATION lib)
ament_export_libraries(pause_replanning_near_goal_bt_node)
install(DIRECTORY launch config maps rviz behavior_trees DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/nav_action_probe.py DESTINATION lib/${PROJECT_NAME}
  RENAME nav_action_probe)
install(PROGRAMS scripts/mock_obstacle_skill.py DESTINATION lib/${PROJECT_NAME}
  RENAME mock_obstacle_skill)
install(PROGRAMS scripts/mission_handoff_example.py DESTINATION lib/${PROJECT_NAME}
  RENAME mission_handoff_example)
install(FILES NAV2_BT.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_precision_pose test/test_precision_pose.cpp)
  ament_target_dependencies(test_precision_pose dwb_core pluginlib)
  add_test(NAME precision_pose_contract COMMAND test_precision_pose)

  add_executable(test_recoverable_rotate_to_goal test/test_recoverable_rotate_to_goal.cpp)
  ament_target_dependencies(test_recoverable_rotate_to_goal dwb_core dwb_critics pluginlib)
  add_test(NAME recoverable_rotate_to_goal_contract COMMAND test_recoverable_rotate_to_goal)

  add_executable(test_pause_replanning test/test_pause_replanning.cpp)
  target_compile_features(test_pause_replanning PRIVATE cxx_std_17)
  ament_target_dependencies(test_pause_replanning
    behaviortree_cpp_v3 rclcpp tf2_ros nav2_util geometry_msgs)
  add_test(NAME pause_replanning_contract COMMAND test_pause_replanning
    $<TARGET_FILE:pause_replanning_near_goal_bt_node>)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigate_to_pose_client test/test_navigate_to_pose_client.py
    TIMEOUT 90)
  ament_add_pytest_test(map_activation test/test_map_activation.py TIMEOUT 30)
  ament_add_pytest_test(bt_plugin_environment test/test_bt_plugin_environment.py TIMEOUT 30)
endif()
ament_package()
