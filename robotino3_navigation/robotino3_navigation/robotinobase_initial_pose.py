#!/usr/bin/env python3
from time import sleep

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPose(Node):
    def __init__(self):
        super().__init__("robotinobase_initial_pose")
        self.mypub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.create_timer(1, self.mytimercallback)

    def mytimercallback(self):
        if self.count_subscribers("/initialpose") > 0:
            sleep(1.0)
            mymsg = PoseWithCovarianceStamped()
            mymsg.header.stamp = self.get_clock().now().to_msg()
            mymsg.header.frame_id = "map"
            mymsg.pose.pose.position.x = 1.0
            mymsg.pose.pose.position.y = 0.0
            mymsg.pose.pose.position.z = 0.0
            mymsg.pose.pose.orientation.x = 0.0
            mymsg.pose.pose.orientation.y = 0.0
            mymsg.pose.pose.orientation.z = 1.0
            mymsg.pose.pose.orientation.w = 1.0
            self.mypub.publish(mymsg)
            self.destroy_node()
            exit()


def main(args=None):
    rclpy.init(args=args)
    pose = InitialPose()

    try:
        rclpy.spin(pose)
    except KeyboardInterrupt:
        pose.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
