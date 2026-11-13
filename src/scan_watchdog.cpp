// Warns when the lidar goes blind (no finite beams) or silent. A blind lidar makes Nav2 report goals as
// succeeded while the robot never moves.
#include <memory>

#include "nav_robot_cpp/logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanWatchdog : public rclcpp::Node
{
public:
  ScanWatchdog()
  : Node("scan_watchdog")
  {
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        last_ = now();
        finite_ = nav_robot_cpp::countFinite(m->ranges);
        seen_ = true;
      });
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {check();});
  }

private:
  void check()
  {
    if (!seen_) {
      RCLCPP_WARN(get_logger(), "no /scan received yet");
    } else if ((now() - last_).seconds() > 3.0) {
      RCLCPP_ERROR(get_logger(), "/scan has gone silent for more than 3 s");
    } else if (finite_ == 0) {
      RCLCPP_ERROR(get_logger(), "LIDAR BLIND: every beam is inf. Restart the sim; Nav2 results are not trustworthy.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  int finite_{0};
  bool seen_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanWatchdog>());
  rclcpp::shutdown();
  return 0;
}
