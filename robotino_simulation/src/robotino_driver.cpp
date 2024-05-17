// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "robotino_simulation/robotino_driver.hpp"

#include <cstdio>
#include <functional>
#include <webots/accelerometer.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045

namespace robotino_driver {
RobotinoDriver::~RobotinoDriver() {
  shutdown_ = true;
  act_thread_.join();
}
void RobotinoDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {
  node_ = node;
  motors_ = {wb_robot_get_device("wheel0_joint"),
             wb_robot_get_device("wheel1_joint"),
             wb_robot_get_device("wheel2_joint")};

  motor_sensors_ = {wb_robot_get_device("wheel0_joint_sensor"),
                    wb_robot_get_device("wheel1_joint_sensor"),
                    wb_robot_get_device("wheel2_joint_sensor")};
  motor_pos_ = {0.0, 0.0, 0.0};

  for (const auto &sensor : motor_sensors_) {
    wb_position_sensor_enable(sensor, TIME_STEP);
  }
  last_sample_time_ = wb_robot_get_time();

  // Set positions and velocities for motors
  for (const auto &motor : motors_) {
    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, 0.0);
  }
  gps_ = wb_robot_get_device("gps");
  wb_gps_enable(gps_, TIME_STEP);

  inertial_unit_ = wb_robot_get_device("inertial unit");

  gyro_ = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro_, TIME_STEP);

  accelerometer_ = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer_, TIME_STEP);

  std::string namespace_param = parameters["namespace"];
  tf_prefix_ = namespace_param;
  act_frequency_ = std::stod(parameters["frequency"]);
  odom_source_ = parameters["odom_source"];
  cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      namespace_param + "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(vel_msg_mutex_);
        this->cmd_vel_msg = *msg;
      });
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      namespace_param + "/odom", rclcpp::SensorDataQoS().reliable());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
      namespace_param + "/joint_states", 1);

  ir_sensor_names_ = {
      "ir1_sensor", "ir2_sensor", "ir3_sensor", "ir4_sensor", "ir5_sensor",
      "ir6_sensor", "ir7_sensor", "ir8_sensor", "ir9_sensor", "irsensor_merge",
  };
  ir_sensor_pos_ = {
      {0, -0.2264, 0.0566},       {0.1454, -0.1736, 0.0566},
      {0.223, -0.0394, 0.0566},   {0.1963, 0.114, 0.0566},
      {0.0777, 0.2131, 0.0566},   {-0.0772, 0.2133, 0.0566},
      {-0.196, 0.1133, 0.0566},   {-0.2229, -0.0394, 0.0566},
      {-0.1455, -0.1735, 0.0566}, {0, 0, 0.0566},
  };
  ir_sensor_ori_ = {
      {0, 0, -0.7071068, 0.7071068}, {0, 0, -0.4226183, 0.9063078},
      {0, 0, -0.0871557, 0.9961947}, {0, 0, 0.258819, 0.96592581},
      {0, 0, 0.5735764, 0.819152},   {0, 0, 0.819152, 0.5735764},
      {0, 0, 0.9659258, 0.258819},   {0, 0, 0.9961947, -0.0871557},
      {0, 0, 0.9063078, -0.4226183}, {0, 0, 0, 1},
  };

  laser_names_ = {"SickLaser_Rear", "SickLaser_Front", "laser_link"};
  laser_pos_ = {{0.12, 0, 0.3}, {-0.13, 0, 0.35}, {0, 0, 0.3}};
  laser_ori_ = {{0, 0, 0, 1}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  imu_names_ = {"imu_link"};
  imu_pos_ = {{0.0, -0.1, 0.2}};
  imu_ori_ = {{0, 0, 1, 0}};

  // Create a thread and start it with a lambda function
  act_thread_ = std::thread([&]() {
    while (!shutdown_) {
      auto angular_velocity = kinematics();
      wb_motor_set_velocity(motors_[2], angular_velocity[0]);
      wb_motor_set_velocity(motors_[0], angular_velocity[1]);
      wb_motor_set_velocity(motors_[1], angular_velocity[2]);

      // Uncomment the following code to use the odometry from the wheel sensors
      if (odom_source_ == "encoders") {
        double curr_time_ = wb_robot_get_time();
        double time_diff_ = curr_time_ - last_sample_time_;
        int sec_ = static_cast<int>(curr_time_);
        int nanosec_ = static_cast<int>((curr_time_ - sec_) * 1e9);
        TimeStamp time_stamp_;
        time_stamp_.sec = sec_;
        time_stamp_.nanosec = nanosec_;
        read_data();
        publish_odom(time_stamp_, time_diff_);
        last_sample_time_ = curr_time_;
        }
      {
        std::lock_guard<std::mutex> lock(vel_msg_mutex_);
        this->cmd_vel_msg.linear.x = 0.0;
        this->cmd_vel_msg.linear.y = 0.0;
        this->cmd_vel_msg.angular.z = 0.0;
      }
      std::chrono::duration<double, std::ratio<1>> period{1.0 / act_frequency_};
      std::this_thread::sleep_for(period);
    }
  });
}

double RobotinoDriver::get_time() {
  return (double)node_->get_clock()->now().nanoseconds();
}

void RobotinoDriver::read_data() {
  // Set positions and velocities for motors
  for (size_t i = 0; i < 3; i++) {
    motor_pos_[i] = wb_position_sensor_get_value(motor_sensors_[i]);
  }
}

void RobotinoDriver::publish_data() {
  //read_data();
  double curr_time = wb_robot_get_time();

  // Convert double time to seconds and nanoseconds
  int sec = static_cast<int>(curr_time);
  int nanosec = static_cast<int>((curr_time - sec) * 1e9);
  TimeStamp time_stamp;
  time_stamp.sec = sec;
  time_stamp.nanosec = nanosec;

  if (odom_source_ == "gps"){
    publish_odom_from_sensors(time_stamp);
  }
  //publish_odom_from_sensors(time_stamp);
  publish_joint_state(time_stamp);
  publish_ir(time_stamp);
  publish_laser(time_stamp);
  publish_imu(time_stamp);
}

void RobotinoDriver::publish_laser(const TimeStamp &time_stamp) {

  for (size_t i = 0; i < laser_names_.size(); ++i) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time_stamp;
    tf.header.frame_id = tf_prefix_ + "/base_link";
    tf.child_frame_id = tf_prefix_ + "/" + laser_names_[i];
    tf.transform.translation.x = laser_pos_[i][0];
    tf.transform.translation.y = laser_pos_[i][1];
    tf.transform.translation.z = laser_pos_[i][2];
    tf.transform.rotation.x = laser_ori_[i][0];
    tf.transform.rotation.y = laser_ori_[i][1];
    tf.transform.rotation.z = laser_ori_[i][2];
    tf.transform.rotation.w = laser_ori_[i][3];
    tf_broadcaster_->sendTransform(tf);
  }
}

void RobotinoDriver::publish_imu(const TimeStamp &time_stamp) {
  for (size_t i = 0; i < imu_names_.size(); ++i) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time_stamp;
    tf.header.frame_id = tf_prefix_ + "/base_link";
    tf.child_frame_id = tf_prefix_ + "/" + imu_names_[i];
    tf.transform.translation.x = this->imu_pos_[i][0];
    tf.transform.translation.y = this->imu_pos_[i][1];
    tf.transform.translation.z = this->imu_pos_[i][2];
    tf.transform.rotation.x = this->imu_ori_[i][0];
    tf.transform.rotation.y = this->imu_ori_[i][1];
    tf.transform.rotation.z = this->imu_ori_[i][2];
    tf.transform.rotation.w = this->imu_ori_[i][3];
    this->tf_broadcaster_->sendTransform(tf);
  }
}
void RobotinoDriver::publish_odom_from_sensors(const TimeStamp &time_stamp) {

  auto q = wb_inertial_unit_get_quaternion(inertial_unit_);
  auto velocity = wb_gps_get_speed_vector(gps_);
  auto pose = wb_gps_get_values(gps_);
  auto gyro = wb_gyro_get_values(gyro_);
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = time_stamp;
  odom_msg.header.frame_id = tf_prefix_ + "/odom";
  odom_msg.child_frame_id = tf_prefix_ + "/base_link";
  odom_msg.twist.twist.linear.x = velocity[0];
  odom_msg.twist.twist.linear.y = velocity[1];
  odom_msg.twist.twist.linear.z = velocity[2];
  odom_msg.twist.twist.angular.x = gyro[0];
  odom_msg.twist.twist.angular.y = gyro[1];
  odom_msg.twist.twist.angular.z = gyro[2];
  odom_msg.pose.pose.position.x = pose[0];
  odom_msg.pose.pose.position.y = pose[1];
  odom_msg.pose.pose.position.z = pose[2];
  odom_msg.pose.pose.orientation.x = q[0];
  odom_msg.pose.pose.orientation.y = q[1];
  odom_msg.pose.pose.orientation.z = q[2];
  odom_msg.pose.pose.orientation.w = q[3];

  odom_pub_->publish(odom_msg);
  // geometry_msgs::msg::TransformStamped tf_msg;
  // tf_msg.header.stamp = time_stamp;
  // tf_msg.header.frame_id = (tf_prefix_ + "/odom");
  // tf_msg.child_frame_id = (tf_prefix_ + "/base_link");
  // tf_msg.transform.translation.x = pose[0];
  // tf_msg.transform.translation.y = pose[1];
  // tf_msg.transform.translation.z = pose[2];
  // tf_msg.transform.rotation.x = q[0];
  // tf_msg.transform.rotation.y = q[1];
  // tf_msg.transform.rotation.z = q[2];
  // tf_msg.transform.rotation.w = q[3];

  // tf_broadcaster_->sendTransform(tf_msg);
}

void RobotinoDriver::publish_odom(const TimeStamp &time_stamp,
                                  const double &time_diff) {
  double wheel0_ticks = motor_pos_[2];
  double wheel1_ticks = motor_pos_[0];
  double wheel2_ticks = motor_pos_[1];

  double w0 = (wheel0_ticks - prev_wheel0_ticks_) / (time_diff);
  double w1 = (wheel1_ticks - prev_wheel1_ticks_) / (time_diff);
  double w2 = (wheel2_ticks - prev_wheel2_ticks_) / (time_diff);

  auto velocity = inverse_kinematics(w0, w1, w2);

  double phi = prev_odom_omega_ + (velocity[2] * time_diff);
  double x = prev_odom_x_ + (((velocity[0] * cos(phi)) - (velocity[1] * sin(phi))) * time_diff);
  //double x = prev_odom_x_ + (velocity[0]*time_diff);
  //RCLCPP_INFO(node_->get_logger(), "Position_x: %f ", x);
  double y = prev_odom_y_ + (((velocity[0] * sin(phi)) + (velocity[1] * cos(phi))) * time_diff);
  //double y = prev_odom_y_ + (velocity[1]*time_diff);
  //RCLCPP_INFO(node_->get_logger(), "Position_y: %f", y);

  std::vector<double> q = {0.0, 0.0, sin(phi / 2.), cos(phi / 2.)};
  prev_odom_x_ = x;
  prev_odom_y_ = y;
  prev_odom_omega_ = phi;
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = time_stamp;
  odom_msg.header.frame_id = tf_prefix_ + "/odom";
  odom_msg.child_frame_id = tf_prefix_ + "/base_link";
  odom_msg.twist.twist.linear.x = velocity[0];
  odom_msg.twist.twist.linear.y = velocity[1];
  odom_msg.twist.twist.angular.z = velocity[2];
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.x = q[0];
  odom_msg.pose.pose.orientation.y = q[1];
  odom_msg.pose.pose.orientation.z = q[2];
  odom_msg.pose.pose.orientation.w = q[3];

  odom_pub_->publish(odom_msg);
  prev_wheel0_ticks_ = wheel0_ticks;
  prev_wheel1_ticks_ = wheel1_ticks;
  prev_wheel2_ticks_ = wheel2_ticks;

  // odom_pub_->publish(odom_msg);
  // geometry_msgs::msg::TransformStamped tf_msg;
  // tf_msg.header.stamp = time_stamp;
  // tf_msg.header.frame_id = (tf_prefix_ + "/odom");
  // tf_msg.child_frame_id = (tf_prefix_ + "/base_link");
  // tf_msg.transform.translation.x = x;
  // tf_msg.transform.translation.y = y;
  // tf_msg.transform.translation.z = 0.0;
  // tf_msg.transform.rotation.x = q[0];
  // tf_msg.transform.rotation.y = q[1];
  // tf_msg.transform.rotation.z = q[2];
  // tf_msg.transform.rotation.w = q[3];

  // tf_broadcaster_->sendTransform(tf_msg);
}

void RobotinoDriver::publish_ir(const TimeStamp &time_stamp) {
  for (size_t i = 0; i < ir_sensor_names_.size(); ++i) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time_stamp;
    tf.header.frame_id =
        tf_prefix_ +
        "/base_link"; // Assuming the frame_id is fixed as "base_link"
    tf.child_frame_id = tf_prefix_ + "/" + ir_sensor_names_[i];
    tf.transform.translation.x = ir_sensor_pos_[i][0];
    tf.transform.translation.y = ir_sensor_pos_[i][1];
    tf.transform.translation.z = ir_sensor_pos_[i][2];
    tf.transform.rotation.x = ir_sensor_ori_[i][0];
    tf.transform.rotation.y = ir_sensor_ori_[i][1];
    tf.transform.rotation.z = ir_sensor_ori_[i][2];
    tf.transform.rotation.w = ir_sensor_ori_[i][3];
    this->tf_broadcaster_->sendTransform(tf);
  }
}

void RobotinoDriver::publish_joint_state(const TimeStamp &time_stamp) {

  std::vector<std::string> motor_names = {"wheel0_joint", "wheel1_joint",
                                          "wheel2_joint"};
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = time_stamp;
  std::vector<double> zeros(0.0, motors_.size());
  joint_state_msg.velocity = zeros;
  joint_state_msg.effort = zeros;
  joint_state_msg.name = motor_names;
  joint_state_msg.position = motor_pos_;
  joint_state_pub_->publish(joint_state_msg);
}

std::vector<double> RobotinoDriver::kinematics() {
  // Algorithm1: Linear kinematic model for omniwheel drive (Angular velocity
  // from linear velocity)
  double v_x = this->cmd_vel_msg.linear.x;
  double v_y = this->cmd_vel_msg.linear.y;
  double omega = this->cmd_vel_msg.angular.z;

  double k = (60.0*GEER_RATIO*0.009375)/(2.0*M_PI*WHEEL_RADIUS);

  omega = omega * WHEEL_DISTANCE;
  double m1 = (((sqrt(3.) / 2.) * v_x) - (0.5 * v_y) - omega) * k;
  double m2 = (v_y - omega) * k;
  double m3 = (-((sqrt(3.) / 2.) * v_x) - (0.5 * v_y) - omega) * k;

  return {m1, m2, m3};
}

std::vector<double> RobotinoDriver::inverse_kinematics(const double &w0,
                                                       const double &w1,
                                                       const double &w2) {
  double k_inv = (2*M_PI*WHEEL_RADIUS)/(60*GEER_RATIO*0.009375);

  double vx = ((w0 - w2) / sqrt(3.0)) * k_inv;
  double vy = (-((1.0 / 3.0) * w0) + ((2.0 / 3.0) * w1)- ((1.0 / 3.0) * w2)) * k_inv;
  double const_val = 3.0 * WHEEL_DISTANCE;
  double omega = (-((1.0 / const_val) * w0) - ((1.0 / const_val) * w1) -
                  ((1.0 / const_val) * w2)) * k_inv;
  return {vx, vy, omega};
}

void RobotinoDriver::step() { publish_data(); }
} // namespace robotino_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotino_driver::RobotinoDriver,
                       webots_ros2_driver::PluginInterface)
