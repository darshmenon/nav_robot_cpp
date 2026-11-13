// Waypoint task executor: log the arrival and pause for `pause_seconds` before the follower moves on.
#include <chrono>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav_robot_cpp
{

class LogWaypointTask : public nav2_core::WaypointTaskExecutor
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name) override
  {
    node_ = parent.lock();
    logger_ = node_->get_logger();
    nav2_util::declare_parameter_if_not_declared(node_, plugin_name + ".pause_seconds", rclcpp::ParameterValue(1.0));
    node_->get_parameter(plugin_name + ".pause_seconds", pause_);
  }

  bool processAtWaypoint(const geometry_msgs::msg::PoseStamped & pose, const int & index) override
  {
    RCLCPP_INFO(
      logger_, "arrived at waypoint %d (%.2f, %.2f), pausing %.1f s", index + 1,
      pose.pose.position.x, pose.pose.position.y, pause_);
    std::this_thread::sleep_for(std::chrono::duration<double>(pause_));
    return true;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("LogWaypointTask")};
  double pause_{1.0};
};

}  // namespace nav_robot_cpp

PLUGINLIB_EXPORT_CLASS(nav_robot_cpp::LogWaypointTask, nav2_core::WaypointTaskExecutor)
