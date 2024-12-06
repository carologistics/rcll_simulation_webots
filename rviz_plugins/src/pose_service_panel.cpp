// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "rviz_plugins/pose_service_panel.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace custom_rviz_plugins {

PoseServicePanel::PoseServicePanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  // Initialize the ROS 2 node
  node_ = std::make_shared<rclcpp::Node>("pose_service_panel_node");

  // Create service clients
  client_get_pose_ = node_->create_client<robotino_interface::srv::GetInitPose>(
      "/robotinobase1/get_init_pose");
  client_set_pose_ = node_->create_client<nav2_msgs::srv::SetInitialPose>(
      "/robotinobase1/set_initial_pose");

  // Setup UI
  layout_ = new QVBoxLayout;
  label_ = new QLabel("SetPoseServicePanel");
  layout_->addWidget(label_);

  button_ = new QPushButton("Set_Pose");
  layout_->addWidget(button_);
  connect(button_, &QPushButton::clicked, this,
          &PoseServicePanel::onButtonClicked);

  setLayout(layout_);
}

void PoseServicePanel::onButtonClicked() {
  // Ensure both services are available
  if (!client_get_pose_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "/get_init_pose service is not available.");
    return;
  }
  if (!client_set_pose_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "/set_initial_pose service is not available.");
    return;
  }

  // Create a request for /get_init_pose
  auto get_pose_request =
      std::make_shared<robotino_interface::srv::GetInitPose::Request>();

  // Send the request
  auto get_pose_future = client_get_pose_->async_send_request(get_pose_request);

  // Process the response
  if (rclcpp::spin_until_future_complete(node_, get_pose_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = get_pose_future.get();
    RCLCPP_INFO(node_->get_logger(), "Received pose from /get_init_pose.");

    // Use the response to create a request for /set_initial_pose
    auto set_pose_request =
        std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = node_->get_clock()->now();
    pose_msg.header.frame_id = "map"; // Change if needed
    pose_msg.pose.pose = response->robot_pose;

    set_pose_request->pose = pose_msg;

    // Send the set initial pose request
    auto set_pose_future =
        client_set_pose_->async_send_request(set_pose_request);

    if (rclcpp::spin_until_future_complete(node_, set_pose_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "Successfully set the initial pose.");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call /set_initial_pose.");
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call /get_init_pose.");
  }
}

// Destructor definition
PoseServicePanel::~PoseServicePanel() {
  // Cleanup (if necessary)
}
} // namespace custom_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_rviz_plugins::PoseServicePanel,
                       rviz_common::Panel)
