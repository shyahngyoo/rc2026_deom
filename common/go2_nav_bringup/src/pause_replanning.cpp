#include <cmath>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

// Preserve the planner/recovery subtree while finishing yaw near the goal.
// Always finish an initial path, then latch the final-yaw phase. A new goal
// or a parent halt (cancellation/recovery/completion) releases the latch.
// A one-child ControlNode is intentional. BT::DecoratorNode::executeTick()
// resets a successful child's status to IDLE, which would reset the nested
// RateController timer and accidentally increase distant replanning to BT rate.
class PauseReplanningNearGoal : public BT::ControlNode
{
public:
  PauseReplanningNearGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("distance", 0.25, "Distance to enter final yaw phase"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot frame")};
  }

  void halt() override
  {
    planned_ = false;
    frozen_ = false;
    have_goal_ = false;
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 1) {
      throw BT::RuntimeError("PauseReplanningNearGoal requires exactly one child");
    }
    auto * child_node_ = children_nodes_.front();
    geometry_msgs::msg::PoseStamped goal;
    double distance;
    std::string robot_frame;
    if (!getInput("goal", goal) || !getInput("distance", distance) ||
      !getInput("robot_base_frame", robot_frame) ||
      !std::isfinite(distance) || distance <= 0.0 || goal.header.frame_id.empty())
    {
      throw BT::RuntimeError("PauseReplanningNearGoal: invalid goal/frame/distance");
    }
    // Ignore stamp-only updates; a changed pose or frame is a new destination.
    if (!have_goal_ || goal.pose != goal_.pose || goal.header.frame_id != goal_.header.frame_id) {
      if (have_goal_) { child_node_->halt(); }
      goal_ = goal;
      have_goal_ = true;
      planned_ = false;
      frozen_ = false;
    }
    if (!node_) {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    }
    setStatus(BT::NodeStatus::RUNNING);
    // RateController can be RUNNING while just waiting for its next period.
    // Check its planner subtree to distinguish waiting from an in-flight plan.
    auto * rate = dynamic_cast<BT::DecoratorNode *>(child_node_);
    auto * planner_subtree = rate ? rate->child() : child_node_;
    if (planned_ && !frozen_ && planner_subtree->status() != BT::NodeStatus::RUNNING) {
      geometry_msgs::msg::PoseStamped current;
      if (nav2_util::getCurrentPose(current, *tf_, goal.header.frame_id, robot_frame, 0.2)) {
        const double xy_error = std::hypot(
          current.pose.position.x - goal.pose.position.x,
          current.pose.position.y - goal.pose.position.y);
        if (xy_error <= distance) {
          frozen_ = true;
          child_node_->halt();
          RCLCPP_INFO(node_->get_logger(),
            "REPLAN_FROZEN xy_error=%.4f threshold=%.3f goal=(%.3f,%.3f)",
            xy_error, distance, goal.pose.position.x, goal.pose.position.y);
        }
      }
    }
    if (frozen_) { return BT::NodeStatus::SUCCESS; }
    const auto result = child_node_->executeTick();
    if (result == BT::NodeStatus::SUCCESS) { planned_ = true; }
    return result;
  }

  bool planned_{false}, frozen_{false}, have_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<PauseReplanningNearGoal>("PauseReplanningNearGoal");
}
