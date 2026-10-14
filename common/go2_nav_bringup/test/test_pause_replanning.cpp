#include <iostream>
#include <stdexcept>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

class MockPlan : public BT::ActionNodeBase
{
public:
  MockPlan(const std::string & name, const BT::NodeConfiguration & config, int & plans, bool & running)
  : BT::ActionNodeBase(name, config), plans_(plans), running_(running) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override
  {
    ++plans_;
    return running_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  void halt() override { setStatus(BT::NodeStatus::IDLE); }
private:
  int & plans_;
  bool & running_;
};

void require(bool value, const char * message)
{
  if (!value) { throw std::runtime_error(message); }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_pause_replanning");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  auto pose = [&](double x) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = x;
    t.transform.rotation.w = 1.0;
    tf->setTransform(t, "test", true);
  };
  auto bb = BT::Blackboard::create();
  bb->set("node", node);
  bb->set("tf_buffer", tf);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.0;
  goal.pose.orientation.w = 1.0;
  bb->set("goal", goal);
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(argv[1]);
  int plans = 0;
  bool running_plan = false;
  factory.registerBuilder<MockPlan>("Plan", [&](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<MockPlan>(name, config, plans, running_plan);
  });
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">"
    "<PauseReplanningNearGoal goal=\"{goal}\" distance=\"0.25\">"
    "<Plan/></PauseReplanningNearGoal></BehaviorTree></root>", bb);
  pose(0.8);
  tree.tickRoot();
  require(plans == 1, "near-start goal still needs an initial path");
  tree.tickRoot();
  require(plans == 1, "must freeze after initial path when near goal");
  pose(0.65);
  tree.tickRoot();
  require(plans == 1, "drift must not unfreeze the final-yaw phase");
  goal.header.stamp.sec = 1;
  bb->set("goal", goal);
  tree.tickRoot();
  require(plans == 1, "stamp-only change must not reset the latch");
  goal.pose.orientation.z = 1.0;
  goal.pose.orientation.w = 0.0;
  bb->set("goal", goal);
  tree.tickRoot();
  require(plans == 2, "yaw-only goal update must generate a new path");
  tree.haltTree();
  tree.tickRoot();
  require(plans == 3, "recovery/cancellation halt must release the latch");
  pose(0.0);
  tree.tickRoot();
  require(plans == 4, "far from goal must continue planning");
  tree.haltTree();
  running_plan = true;
  pose(0.8);
  require(tree.tickRoot() == BT::NodeStatus::RUNNING, "initial planning can run asynchronously");
  require(tree.tickRoot() == BT::NodeStatus::RUNNING, "in-flight initial plan must not be skipped");
  running_plan = false;
  tree.tickRoot();
  int finished_plans = plans;
  tree.tickRoot();
  require(plans == finished_plans, "freeze after in-flight initial plan completes");
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_rate_controller_bt_node.so");
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
  int follower_ticks = 0;
  bool follower_running = true;
  factory.registerBuilder<MockPlan>("Follower", [&](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<MockPlan>(name, config, follower_ticks, follower_running);
  });
  plans = 0;
  pose(0.0);
  auto pipeline = factory.createTreeFromText(
    "<root main_tree_to_execute=\"P\"><BehaviorTree ID=\"P\"><PipelineSequence>"
    "<PauseReplanningNearGoal goal=\"{goal}\" distance=\"0.25\">"
    "<RateController hz=\"1.0\"><Plan/></RateController></PauseReplanningNearGoal>"
    "<Follower/></PipelineSequence></BehaviorTree></root>", bb);
  for (int i = 0; i < 10; ++i) { pipeline.tickRoot(); }
  require(plans == 1, "wrapper must preserve the real RateController 1 Hz timer");
  require(follower_ticks > 1, "rate waiting must not block a running follower");
  pose(0.8);
  pipeline.tickRoot();
  pose(0.6);
  for (int i = 0; i < 10; ++i) { pipeline.tickRoot(); }
  require(plans == 1, "freeze must latch even when RateController is waiting");
  std::cout << "PASS: initial path, frozen drift, new goal, halt reset, async plan, real 1 Hz rate and pipeline\n";
  rclcpp::shutdown();
}
