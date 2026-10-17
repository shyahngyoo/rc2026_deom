#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <string>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_nav_bringup
{
// Optional body X/Y correction for holonomic platforms; the explicit
// nonholonomic mode steers toward positional targets before final heading.
class PrecisionPoseCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("PrecisionPose node expired");}
    auto parameter = [&](const std::string & key, double fallback) {
      const auto name = dwb_plugin_name_ + "." + name_ + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(fallback));
      const double value = node->get_parameter(name).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(name + " must be positive and finite");
      }
      return value;
    };
    const auto holonomic_name = dwb_plugin_name_ + "." + name_ + ".holonomic";
    nav2_util::declare_parameter_if_not_declared(node, holonomic_name,
      rclcpp::ParameterValue(true));
    holonomic_ = node->get_parameter(holonomic_name).as_bool();
    position_tolerance_ = parameter("position_tolerance", 0.05);
    xy_gain_ = parameter("xy_gain", 0.7);
    yaw_gain_ = parameter("yaw_gain", 0.8);
    max_xy_ = parameter("max_xy_speed", 0.12);
    max_yaw_ = parameter("max_yaw_speed", 0.35);
    angular_weight_ = parameter("angular_weight", 0.5);
    direct_radius_ = parameter("goal_direct_radius", 0.3);
    lookahead_ = parameter("path_lookahead", 0.2);
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & plan) override
  {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.theta))
    {throw dwb_core::IllegalTrajectoryException(name_, "Non-finite pose");}
    auto target = goal;
    // Retain the planned route for a longer approach. Near the endpoint, use
    // the exact metric goal so grid-cell potentials cannot create a dead zone.
    if (std::hypot(goal.x - pose.x, goal.y - pose.y) > direct_radius_ &&
      !plan.poses.empty())
    {
      auto nearest = std::min_element(plan.poses.begin(), plan.poses.end(),
        [&](const auto & a, const auto & b) {
          return std::hypot(a.x-pose.x, a.y-pose.y) < std::hypot(b.x-pose.x, b.y-pose.y);
        });
      auto cursor = nearest;
      double length = 0.0;
      while (std::next(cursor) != plan.poses.end() && length < lookahead_) {
        const auto next = std::next(cursor);
        length += std::hypot(next->x-cursor->x, next->y-cursor->y);
        cursor = next;
      }
      target = *cursor;
    }
    const double dx = xy_gain_ * (target.x-pose.x);
    const double dy = xy_gain_ * (target.y-pose.y);
    const double speed = std::hypot(dx, dy);
    const double factor = speed > max_xy_ ? max_xy_/speed : 1.0;
    desired_x_ = factor * (std::cos(pose.theta)*dx + std::sin(pose.theta)*dy);
    desired_y_ = factor * (-std::sin(pose.theta)*dx + std::cos(pose.theta)*dy);
    const double error = std::atan2(std::sin(goal.theta-pose.theta),
      std::cos(goal.theta-pose.theta));
    desired_yaw_ = std::clamp(yaw_gain_*error, -max_yaw_, max_yaw_);
    if (!holonomic_) {
      // A lateral velocity objective stalls a nonholonomic robot beside its
      // goal. Steer toward the positional target first, then align final yaw.
      desired_y_ = 0.0;
      if (std::hypot(goal.x-pose.x, goal.y-pose.y) > position_tolerance_) {
        double bearing = std::atan2(target.y-pose.y, target.x-pose.x)-pose.theta;
        bearing = std::atan2(std::sin(bearing), std::cos(bearing));
        double direction = 1.0;
        if (std::abs(bearing) > M_PI/2.0) {
          direction = -1.0;
          bearing = std::atan2(std::sin(bearing+M_PI), std::cos(bearing+M_PI));
        }
        desired_x_ = direction*std::min(speed, max_xy_)*std::max(0.0, std::cos(bearing));
        desired_yaw_ = std::clamp(yaw_gain_*bearing, -max_yaw_, max_yaw_);
      } else {
        desired_x_ = 0.0;
      }
    }
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (trajectory.poses.empty()) {
      throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory");
    }
    const auto & v = trajectory.velocity;
    return std::pow(v.x-desired_x_, 2) + std::pow(v.y-desired_y_, 2) +
      angular_weight_*std::pow(v.theta-desired_yaw_, 2);
  }
private:
  bool holonomic_{true};
  double position_tolerance_{0.05};
  double xy_gain_, yaw_gain_, max_xy_, max_yaw_, angular_weight_, direct_radius_, lookahead_;
  double desired_x_{0.0}, desired_y_{0.0}, desired_yaw_{0.0};
};
}  // namespace go2_nav_bringup
PLUGINLIB_EXPORT_CLASS(go2_nav_bringup::PrecisionPoseCritic, dwb_core::TrajectoryCritic)
