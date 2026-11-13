// Controller: minimal pure pursuit for a differential-drive robot. Picks the first path point at least
// `lookahead` away from the robot, drives toward it with curvature 2y/L^2, slows in tight turns and near
// the goal, and rotates in place for large heading errors. A readable template, not a replacement for MPPI.
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_robot_cpp/logic.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_robot_cpp
{

class PurePursuitController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    tf_ = tf;
    name_ = name;
    costmap_ros_ = costmap_ros;
    auto declare = [&](const std::string & k, double v) {
        nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + k, rclcpp::ParameterValue(v));
        double out;
        node_->get_parameter(name_ + "." + k, out);
        return out;
      };
    lookahead_ = declare("lookahead", 0.6);
    max_v_ = declare("max_linear_speed", 0.4);
    max_w_ = declare("max_angular_speed", 1.5);
    rotate_threshold_ = declare("rotate_in_place_angle", 0.9);
    slow_radius_ = declare("slow_down_radius", 0.8);
    min_v_ = declare("min_linear_speed", 0.05);
  }

  void cleanup() override {plan_.poses.clear();}
  void activate() override {}
  void deactivate() override {}

  void setPlan(const nav_msgs::msg::Path & path) override {plan_ = path;}

  void setSpeedLimit(const double & limit, const bool & percentage) override
  {
    // limit == 0 means "no limit" in Nav2
    if (limit <= 0.0) {
      speed_scale_ = 1.0;
    } else if (percentage) {
      speed_scale_ = std::min(1.0, limit / 100.0);
    } else {
      speed_scale_ = std::min(1.0, limit / max_v_);
    }
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override
  {
    if (plan_.poses.empty()) {
      throw nav2_core::PlannerException("pure pursuit received an empty plan");
    }
    // plan poses into the robot's frame
    std::vector<geometry_msgs::msg::PoseStamped> local;
    for (const auto & p : plan_.poses) {
      geometry_msgs::msg::PoseStamped in = p, out;
      in.header.frame_id = plan_.header.frame_id;
      in.header.stamp = rclcpp::Time(0);
      if (!tf_->canTransform(pose.header.frame_id, in.header.frame_id, tf2::TimePointZero)) {
        throw nav2_core::PlannerException("cannot transform plan into " + pose.header.frame_id);
      }
      tf_->transform(in, out, pose.header.frame_id);
      local.push_back(out);
    }
    // closest point, then the first one at least `lookahead` further along
    size_t closest = 0;
    double best = 1e9;
    for (size_t i = 0; i < local.size(); ++i) {
      const double d = std::hypot(
        local[i].pose.position.x - pose.pose.position.x, local[i].pose.position.y - pose.pose.position.y);
      if (d < best) {best = d; closest = i;}
    }
    size_t target = local.size() - 1;
    for (size_t i = closest; i < local.size(); ++i) {
      const double d = std::hypot(
        local[i].pose.position.x - pose.pose.position.x, local[i].pose.position.y - pose.pose.position.y);
      if (d >= lookahead_) {target = i; break;}
    }
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double dx = local[target].pose.position.x - pose.pose.position.x;
    const double dy = local[target].pose.position.y - pose.pose.position.y;
    const double lx = std::cos(yaw) * dx + std::sin(yaw) * dy;     // target in the robot frame
    const double ly = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double heading_err = std::atan2(ly, lx);
    const double dist_goal = std::hypot(
      local.back().pose.position.x - pose.pose.position.x, local.back().pose.position.y - pose.pose.position.y);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = pose.header.frame_id;
    cmd.header.stamp = node_->now();
    if (dist_goal < 0.12) {
      // at the goal position: turn to the goal heading
      const double gyaw = tf2::getYaw(local.back().pose.orientation);
      cmd.twist.angular.z = std::clamp(normalizeAngle(gyaw - yaw) * 1.5, -max_w_, max_w_);
      return cmd;
    }
    if (std::fabs(heading_err) > rotate_threshold_) {
      cmd.twist.angular.z = std::clamp(heading_err * 2.0, -max_w_, max_w_);
      return cmd;
    }
    double v = max_v_ * speed_scale_;
    if (dist_goal < slow_radius_) {v *= std::max(0.2, dist_goal / slow_radius_);}
    v *= std::max(0.3, 1.0 - std::fabs(heading_err));     // slower in turns
    v = std::max(min_v_, v);
    cmd.twist.linear.x = v;
    cmd.twist.angular.z = std::clamp(v * pursuitCurvature(lx, ly), -max_w_, max_w_);
    return cmd;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  nav_msgs::msg::Path plan_;
  double lookahead_{0.6}, max_v_{0.4}, max_w_{1.5}, rotate_threshold_{0.9}, slow_radius_{0.8}, min_v_{0.05};
  double speed_scale_{1.0};
};

}  // namespace nav_robot_cpp

PLUGINLIB_EXPORT_CLASS(nav_robot_cpp::PurePursuitController, nav2_core::Controller)
