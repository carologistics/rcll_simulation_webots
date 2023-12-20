#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Robotino3ScanRemap(Node):

    def __init__(self):
        super().__init__('robotino3_laserscan_republisher')
        self.create_subscription(LaserScan, '/robotino/SickLaser_Front', self.FrontScan_cb, 10)
        self.create_subscription(LaserScan, '/robotino/SickLaser_Rear', self.RearScan_cb, 10)
        self.Front_publisher= self.create_publisher(LaserScan, '/robotino/SickLaser_Front_Remaped', 10)
        self.Rear_publisher= self.create_publisher(LaserScan, '/robotino/SickLaser_Rear_Remaped', 10)
        
    # callback function to publish data over cmd_vel topic based on joy_pad inputs
    def FrontScan_cb(self, msg):
        scan_f = LaserScan()
        scan_f.header.stamp = self.get_clock().now().to_msg()
        scan_f.header.frame_id = msg.header.frame_id
        scan_f.angle_min = (msg.angle_max)
        scan_f.angle_max = (msg.angle_min)
        scan_f.angle_increment = -(msg.angle_increment)
        scan_f.time_increment = msg.time_increment
        scan_f.scan_time = msg.scan_time
        scan_f.range_min = msg.range_min
        scan_f.range_max = msg.range_max
        scan_f.ranges = [float()]*180
        var_len = len(msg.ranges)
        #self.get_logger().info(f'range length: {var_len}')
        for i in range(var_len):
            scan_f.ranges[179-i] = msg.ranges[i]
        self.Front_publisher.publish(scan_f)
        
    def RearScan_cb(self, msg_r):
        scan_r = LaserScan()
        scan_r.header.stamp = self.get_clock().now().to_msg()
        scan_r.header.frame_id = msg_r.header.frame_id
        scan_r.angle_min = (msg_r.angle_max)
        scan_r.angle_max = (msg_r.angle_min)
        scan_r.angle_increment =  -(msg_r.angle_increment)
        scan_r.time_increment = msg_r.time_increment
        scan_r.scan_time = msg_r.scan_time
        scan_r.range_min = msg_r.range_min
        scan_r.range_max = msg_r.range_max
        scan_r.ranges = [float()]*180
        var_len = len(scan_r.ranges)
        for i in range(var_len):
            scan_r.ranges[179-i] = msg_r.ranges[i]
        self.Rear_publisher.publish(scan_r)

def main():
    rclpy.init()
    teleop_node = Robotino3ScanRemap()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
