#include <cmath>
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace go2_nav_bringup {
// Use metric distance rather than a costmap cell potential at precision entries.
class MetricGoalDistanceCritic : public dwb_core::TrajectoryCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D &) override
  {goal_ = goal;return true;}
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override
  {
    if (traj.poses.empty()) {throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory");}
    const auto & end = traj.poses.back();
    return std::hypot(end.x - goal_.x, end.y - goal_.y);
  }
private:
  geometry_msgs::msg::Pose2D goal_;
};
}
PLUGINLIB_EXPORT_CLASS(go2_nav_bringup::MetricGoalDistanceCritic, dwb_core::TrajectoryCritic)
