#include <cmath>
#include <iostream>
#include <stdexcept>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"

void require(bool condition, const char * message)
{if (!condition) {throw std::runtime_error(message);}}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
      "dwb_core", "dwb_core::TrajectoryCritic");
    auto critic = loader.createSharedInstance(
      "go2_nav_bringup::RecoverableRotateToGoalCritic");
    auto node = std::make_shared<nav2_util::LifecycleNode>("goal_critic_contract");
    node->declare_parameter("Approach.xy_goal_tolerance", 0.08);
    node->declare_parameter("Approach.trans_stopped_velocity", 0.03);
    critic->initialize(node, "RotateToGoal", "Approach", nullptr);
    geometry_msgs::msg::Pose2D pose, goal;
    nav_2d_msgs::msg::Twist2D velocity;
    nav_2d_msgs::msg::Path2D plan;
    goal.theta = 1.0;
    dwb_msgs::msg::Trajectory2D translating, rotating;
    translating.velocity.x = 0.1;
    translating.poses.push_back(pose);
    rotating.velocity.theta = 0.2;
    rotating.poses.push_back(pose);
    pose.x = 0.12;
    critic->prepare(pose, velocity, goal, plan);
    require(critic->scoreTrajectory(translating) == 0.0, "far translation rejected");
    pose.x = 0.06;
    critic->prepare(pose, velocity, goal, plan);
    bool rejected = false;
    try {critic->scoreTrajectory(translating);}
    catch (const dwb_core::IllegalTrajectoryException &) {rejected = true;}
    require(rejected, "near translation must stop");
    require(std::abs(critic->scoreTrajectory(rotating) - 1.0) < 1e-9,
      "final yaw must be scored");
    pose.x = 0.11;
    critic->prepare(pose, velocity, goal, plan);
    require(critic->scoreTrajectory(translating) == 0.0,
      "drifted robot cannot resume translation");
    pose.x = 0.07;
    critic->prepare(pose, velocity, goal, plan);
    rejected = false;
    try {critic->scoreTrajectory(translating);}
    catch (const dwb_core::IllegalTrajectoryException &) {rejected = true;}
    require(rejected, "return to window must restore rotation");
    critic->reset();
    pose.x = 0.12;
    critic->prepare(pose, velocity, goal, plan);
    require(critic->scoreTrajectory(translating) == 0.0, "new plan retained latch");
    // The same near-goal phase must decelerate before it forces pure rotation.
    pose.x = 0.06;
    velocity.x = 0.1;
    critic->prepare(pose, velocity, goal, plan);
    translating.velocity.x = 0.05;
    require(std::isfinite(critic->scoreTrajectory(translating)),
      "decelerating translation must remain legal");
    auto distance = loader.createSharedInstance("go2_nav_bringup::MetricGoalDistanceCritic");
    distance->initialize(node, "GoalDist", "Approach", nullptr);
    distance->prepare(pose, velocity, goal, plan);
    translating.poses.back().x = 0.079;
    translating.poses.back().y = 0.0;
    require(std::abs(distance->scoreTrajectory(translating) - 0.079) < 1e-9,
      "endpoint inside a grid cell must retain metric error");
    translating.poses.back().x = 0.04;
    require(std::abs(distance->scoreTrajectory(translating) - 0.04) < 1e-9,
      "moving closer must continuously reduce cost");
    dwb_msgs::msg::Trajectory2D empty;
    rejected = false;
    try {distance->scoreTrajectory(empty);}
    catch (const dwb_core::IllegalTrajectoryException &) {rejected = true;}
    require(rejected, "empty trajectory must be rejected through the DWB contract");
    std::cout << "PASS: plugin loading, near-goal rotation, drift recovery, reset, deceleration\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
