// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#ifndef ROBOTINO_DRIVER_WEBOTS_PLUGIN_HPP
#define ROBOTINO_DRIVER_WEBOTS_PLUGIN_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include <map>
#define TIME_STEP 32
namespace robotino_driver {
class RobotinoDriver : public webots_ros2_driver::PluginInterface {
public:
  ~RobotinoDriver();
  using TimeStamp = builtin_interfaces::msg::Time;
  static constexpr double WHEEL_RADIUS = 0.063;
  static constexpr double WHEEL_DISTANCE = 0.1826;
  static constexpr double GEER_RATIO = 16.0;
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  void read_data();
  void publish_data();
  void publish_odom(const TimeStamp &t, const double &time_diff);
  void publish_odom_from_sensors(const TimeStamp &t);
  void publish_joint_state(const TimeStamp &t);
  void transform_pose(const TimeStamp &t);
  void publish_ir(const TimeStamp &t);
  void publish_laser(const TimeStamp &t);
  void publish_imu(const TimeStamp &t);
  std::vector<double> inverse_kinematics(const double &w0, const double &w1,
                                         const double &w2);
  std::vector<double> kinematics();

  double get_time();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Twist cmd_vel_msg;

  WbDeviceTag gps_;
  WbDeviceTag inertial_unit_;
  WbDeviceTag gyro_;
  WbDeviceTag accelerometer_;

  double prev_wheel0_ticks_ = 0.0;
  double prev_wheel1_ticks_ = 0.0;
  double prev_wheel2_ticks_ = 0.0;
  double prev_odom_x_ = 0.0;
  double prev_odom_y_ = 0.0;
  double prev_odom_omega_ = 0.0;
  double last_sample_time_ = 0.0;

  std::vector<std::string> imu_names_;
  std::vector<std::vector<double>> imu_pos_;
  std::vector<std::vector<double>> imu_ori_;
  std::vector<std::string> laser_names_;
  std::vector<std::vector<double>> laser_pos_;
  std::vector<std::vector<double>> laser_ori_;
  std::vector<std::string> ir_sensor_names_;
  std::vector<std::vector<double>> ir_sensor_pos_;
  std::vector<std::vector<double>> ir_sensor_ori_;
  std::string tf_prefix_ = "robotinobase1";

  std::mutex vel_msg_mutex_;
  bool shutdown_ = false;

  webots_ros2_driver::WebotsNode *node_;

  std::vector<WbDeviceTag> motors_;
  std::vector<WbDeviceTag> motor_sensors_;
  std::vector<double> motor_pos_;

  std::thread act_thread_;
  double act_frequency_ = 10.0;

  std::string odom_source_ = "gps";
};
} // namespace robotino_driver
#endif
