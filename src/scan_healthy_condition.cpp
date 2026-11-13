// Behavior tree condition: SUCCESS when /scan arrived recently and has finite beams, FAILURE otherwise.
// Usage in a tree: <ScanHealthy max_age="2.0" min_finite="10"/>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_robot_cpp/logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav_robot_cpp
{

class ScanHealthy : public BT::ConditionNode
{
public:
  ScanHealthy(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf)
  {
    node_ = std::make_shared<rclcpp::Node>("scan_healthy_bt_node");
    exec_.add_node(node_);
    sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        last_ = node_->now();
        finite_ = countFinite(m->ranges);
        seen_ = true;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age", 2.0, "seconds since the last scan"),
      BT::InputPort<int>("min_finite", 10, "minimum finite beams")};
  }

  BT::NodeStatus tick() override
  {
    exec_.spin_some();
    double max_age = 2.0;
    int min_finite = 10;
    getInput("max_age", max_age);
    getInput("min_finite", min_finite);
    if (!seen_ || (node_->now() - last_).seconds() > max_age || finite_ < min_finite) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  int finite_{0};
  bool seen_{false};
};

}  // namespace nav_robot_cpp

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav_robot_cpp::ScanHealthy>("ScanHealthy");
}
