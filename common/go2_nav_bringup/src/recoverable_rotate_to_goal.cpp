// Re-enter translation if a stopped, rotating robot drifts out of the XY window.
#include <cmath>
#include <stdexcept>
#include "dwb_critics/rotate_to_goal.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_nav_bringup
{
class RecoverableRotateToGoalCritic : public dwb_critics::RotateToGoalCritic
{
public:
  void onInit() override
  {
    dwb_critics::RotateToGoalCritic::onInit();
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("RotateToGoal node expired");}
    node->get_parameter(dwb_plugin_name_ + ".xy_goal_tolerance", tolerance_);
    if (!std::isfinite(tolerance_) || tolerance_ <= 0.0) {
      throw std::invalid_argument("xy_goal_tolerance must be positive and finite");
    }
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & plan) override
  {
    // Upstream latches both flags for the entire path. A non-stateful goal
    // checker cannot accept a drifted pose, so that latch can prevent recovery.
    // Retain upstream deceleration and rotation scoring inside the window.
    if (std::hypot(pose.x - goal.x, pose.y - goal.y) > tolerance_) {
      dwb_critics::RotateToGoalCritic::reset();
    }
    return dwb_critics::RotateToGoalCritic::prepare(pose, vel, goal, plan);
  }
private:
  double tolerance_{0.08};
};
}  // namespace go2_nav_bringup
PLUGINLIB_EXPORT_CLASS(
  go2_nav_bringup::RecoverableRotateToGoalCritic, dwb_core::TrajectoryCritic)
