// Global planner: a straight line from start to goal, refused when it crosses an occupied or unknown-lethal cell.
// Deliberately simple (no search), useful as a baseline and as a template for a real planner plugin.
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_robot_cpp/logic.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav_robot_cpp
{

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    name_ = name;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".lethal_threshold", rclcpp::ParameterValue(200));
    node_->get_parameter(name_ + ".interpolation_resolution", step_);
    node_->get_parameter(name_ + ".lethal_threshold", lethal_);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = global_frame_;
    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
      throw nav2_core::PlannerException("start and goal must be in the " + global_frame_ + " frame");
    }
    const auto pts = interpolate(
      start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y, step_);
    for (const auto & p : pts) {
      unsigned int mx, my;
      if (!costmap_->worldToMap(p.first, p.second, mx, my)) {
        throw nav2_core::PlannerException("straight line leaves the costmap");
      }
      if (costmap_->getCost(mx, my) >= lethal_) {
        throw nav2_core::PlannerException("straight line is blocked");
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.first;
      pose.pose.position.y = p.second;
      pose.pose.orientation = goal.pose.orientation;
      path.poses.push_back(pose);
    }
    return path;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_, global_frame_;
  double step_{0.05};
  int lethal_{200};
};

}  // namespace nav_robot_cpp

PLUGINLIB_EXPORT_CLASS(nav_robot_cpp::StraightLinePlanner, nav2_core::GlobalPlanner)
