#include <cmath>
#include <array>
#include <limits>
#include <iostream>
#include <stdexcept>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"
void require(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core", "dwb_core::TrajectoryCritic");
    auto critic = loader.createSharedInstance("go2_nav_bringup::PrecisionPoseCritic");
    auto node = std::make_shared<nav2_util::LifecycleNode>("precision_pose_contract");
    critic->initialize(node, "PrecisionPose", "Approach", nullptr);
    geometry_msgs::msg::Pose2D pose, goal;
    nav_2d_msgs::msg::Twist2D velocity;
    nav_2d_msgs::msg::Path2D plan;
    dwb_msgs::msg::Trajectory2D correct, wrong;
    correct.poses.push_back(pose);wrong.poses.push_back(pose);
    goal.y=0.1;goal.theta=0.5;
    critic->prepare(pose, velocity, goal, plan);
    correct.velocity.y=0.07;correct.velocity.theta=0.35;
    wrong.velocity.y=-0.07;wrong.velocity.theta=-0.35;
    require(critic->scoreTrajectory(correct)<1e-12, "simultaneous lateral and yaw correction");
    require(critic->scoreTrajectory(wrong)>critic->scoreTrajectory(correct), "wrong direction preferred");
    for (double distance : {0.079, 0.081}) {
      goal.y=distance;goal.theta=0.5;
      critic->prepare(pose, velocity, goal, plan);
      correct.velocity.y=0.7*distance;
      require(critic->scoreTrajectory(correct)<1e-12,
        "crossing old XY boundary must not disable or reverse yaw correction");
    }
    pose.theta=M_PI/2;goal.theta=M_PI/2;goal.y=0.1;
    critic->prepare(pose, velocity, goal, plan);
    correct.velocity.x=0.07;correct.velocity.y=0.0;correct.velocity.theta=0.0;
    require(critic->scoreTrajectory(correct)<1e-12, "world error must rotate into body coordinates");
    pose.theta=M_PI-0.05;goal.x=goal.y=0.;goal.theta=-M_PI+0.05;
    critic->prepare(pose, velocity, goal, plan);
    correct.velocity.x=0.;correct.velocity.theta=0.08;
    require(critic->scoreTrajectory(correct)<1e-12, "yaw must use shortest angular distance");
    pose.theta=0.;goal.x=2.;goal.y=0.;goal.theta=0.;
    geometry_msgs::msg::Pose2D path_pose;
    for (double y : {0.0,0.1,0.3}) {path_pose.y=y;plan.poses.push_back(path_pose);}
    path_pose.x=2.;plan.poses.push_back(path_pose);path_pose.y=0.;plan.poses.push_back(path_pose);
    critic->prepare(pose, velocity, goal, plan);
    correct.velocity.y=0.12;correct.velocity.theta=0.;
    require(critic->scoreTrajectory(correct)<1e-12, "long approach must use path lookahead");
    dwb_msgs::msg::Trajectory2D empty;bool rejected=false;
    try {critic->scoreTrajectory(empty);} catch (const dwb_core::IllegalTrajectoryException &) {rejected=true;}
    require(rejected,"empty trajectory contract");
    auto differential = loader.createSharedInstance("go2_nav_bringup::PrecisionPoseCritic");
    node->declare_parameter("Differential.PrecisionPose.holonomic", false);
    differential->initialize(node, "PrecisionPose", "Differential", nullptr);
    pose = geometry_msgs::msg::Pose2D();goal = geometry_msgs::msg::Pose2D();plan.poses.clear();
    goal.y=0.1;
    differential->prepare(pose, velocity, goal, plan);
    correct.velocity.x=0.;correct.velocity.y=0.;correct.velocity.theta=0.35;
    wrong.velocity.x=0.;wrong.velocity.y=0.;wrong.velocity.theta=0.;
    require(differential->scoreTrajectory(correct)<1e-12,
      "nonholonomic lateral error requires steering, not a stationary minimum");
    require(differential->scoreTrajectory(correct)<differential->scoreTrajectory(wrong),
      "steering must beat stopping beside the goal");
    goal.x=-0.1;goal.y=0.;
    differential->prepare(pose, velocity, goal, plan);
    correct.velocity.x=-0.07;correct.velocity.theta=0.;
    require(differential->scoreTrajectory(correct)<1e-12, "nonholonomic reverse correction");
    goal.x=0.02;goal.theta=0.5;
    differential->prepare(pose, velocity, goal, plan);
    correct.velocity.x=0.;correct.velocity.theta=0.35;
    require(differential->scoreTrajectory(correct)<1e-12, "final heading inside position tolerance");
    // Closed-loop software convergence with no lateral samples, including
    // position targets initially perpendicular to the body heading.
    for (const auto & target : {std::array<double,3>{0.0,0.2,0.6},
      std::array<double,3>{0.0,-0.2,-0.6}, std::array<double,3>{-0.2,0.1,0.8}})
    {
      pose = geometry_msgs::msg::Pose2D();goal.x=target[0];goal.y=target[1];goal.theta=target[2];
      bool reached=false;
      for (int tick=0; tick<2000 && !reached; ++tick) {
        differential->prepare(pose, velocity, goal, plan);
        double best=std::numeric_limits<double>::infinity();
        dwb_msgs::msg::Trajectory2D chosen;
        for (int ix=-8; ix<=8; ++ix) {
          for (int iw=-7; iw<=7; ++iw) {
            dwb_msgs::msg::Trajectory2D candidate;candidate.poses.push_back(pose);
            candidate.velocity.x=ix*0.015;candidate.velocity.y=0.0;candidate.velocity.theta=iw*0.05;
            const double score=differential->scoreTrajectory(candidate);
            if (score<best) {best=score;chosen=candidate;}
          }
        }
        pose.x += std::cos(pose.theta)*chosen.velocity.x*0.05;
        pose.y += std::sin(pose.theta)*chosen.velocity.x*0.05;
        pose.theta += chosen.velocity.theta*0.05;
        const double yaw_error=std::atan2(std::sin(goal.theta-pose.theta),std::cos(goal.theta-pose.theta));
        reached=std::hypot(goal.x-pose.x,goal.y-pose.y)<0.08 && std::abs(yaw_error)<0.08;
      }
      require(reached,"nonholonomic sampled closed loop must converge in position and heading");
    }
    std::cout<<"PASS: simultaneous XY/yaw, boundary continuity, body frame, angle wrapping, route lookahead, invalid trajectory\n";
  } catch (const std::exception & e) {std::cerr<<e.what()<<"\n";rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
