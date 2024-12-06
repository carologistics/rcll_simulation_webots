// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#ifndef POSE_SERVICE_PANEL_HPP
#define POSE_SERVICE_PANEL_HPP

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <memory>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotino_interface/srv/get_init_pose.hpp>
#include <rviz_common/panel.hpp>

namespace custom_rviz_plugins {

class PoseServicePanel : public rviz_common::Panel {
  Q_OBJECT

public:
  PoseServicePanel(QWidget *parent = nullptr);
  ~PoseServicePanel() override;

private Q_SLOTS:
  void onButtonClicked();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<robotino_interface::srv::GetInitPose>::SharedPtr
      client_get_pose_;
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr client_set_pose_;

  QVBoxLayout *layout_;
  QPushButton *button_;
  QLabel *label_;
};

} // namespace custom_rviz_plugins

#endif // POSE_SERVICE_PANEL_HPP
