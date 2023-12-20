
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

l1tf_received = False
l2tf_received = False
l1scan_received = False
l2scan_received = False


class Robotino3LaserScanMerger(Node):

    def __init__(self):
        super().__init__('robotino3_laserscanmerger')
        self.get_parameters()
        self.subscription_laserscan2 = self.create_subscription(LaserScan, self.scantopic2, self.LaserScan2_cb, 10)
        self.subscription_laserscan1 = self.create_subscription(LaserScan, self.scantopic1, self.LaserScan1_cb, 10)
        self.mergescanpub = self.create_publisher(LaserScan,'/scan',10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    def get_parameters(self):
        # Get User defined parameters from Config file 
        self.scantopic1 = self.declare_parameter('scantopic_1', '/robotino/SickLaser_Front_Remaped').get_parameter_value().string_value
        self.laser1_tfframe = self.declare_parameter('laser1_tfframe', 'SickLaser_Front').get_parameter_value().string_value
        
        self.scantopic2 = self.declare_parameter('scantopic_2', '/robotino/SickLaser_Front_Remaped').get_parameter_value().string_value
        self.laser2_tfframe = self.declare_parameter('laser2_tfframe', 'SickLaser_Rear').get_parameter_value().string_value
        
        self.targetframe = self.declare_parameter('targetframe', 'laser_link').get_parameter_value().string_value
        
        self.mergescan_minangle = self.declare_parameter('mergescan_minangle', 3.1416).get_parameter_value().double_value
        self.mergescan_maxangle = self.declare_parameter('mergescan_maxangle', -3.1416).get_parameter_value().double_value
       
    def LaserScan2_cb(self, data):
        global l2scan_received
        if not l2scan_received: 
            self.laser_scan2_ = data
            if len(self.laser_scan2_.ranges) > 0:
                l2scan_received = True
            else:
                l2scan_received = False
         
    def LaserScan1_cb(self, data):
        global l1scan_received
        if not l1scan_received:
            self.laser_scan1_ = data
            if len(self.laser_scan1_.ranges) > 0:
                l1scan_received = True
                self.TfListener_l1()
            else:
                l1scan_received = False
            
    def TfListener_l1(self):
        global l1tf_received
        if not l1tf_received:
            from_frame_rel = self.laser1_tfframe
            to_frame_rel = self.targetframe
            try:
                laser1_t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
                l1tf_received = True 
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return
            self.xoff_l1 = laser1_t.transform.translation.x
            self.yoff_l1 = laser1_t.transform.translation.y
            r = R.from_quat([laser1_t.transform.rotation.x, laser1_t.transform.rotation.y, laser1_t.transform.rotation.z, laser1_t.transform.rotation.w])
            rot_array = r.as_euler('zyx', degrees=True)
            self.yaw_l1 = rot_array[0]
            #self.get_logger().info(f'laser1 x_off: {self.xoff_l1}')
            #self.get_logger().info(f'laser1 y_off: {self.yoff_l1}')
            #self.get_logger().info(f'laser1 yaw: {self.yaw_l1}')
            if l1tf_received:
                self.TfListener_l2()
            else:
                l1tf_received = False
        
    def TfListener_l2(self):
        global l2tf_received
        if not l2tf_received:
            from_frame_rel = self.laser2_tfframe
            to_frame_rel = self.targetframe
            try:
                laser2_t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
                l2tf_received = True 
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return
            self.xoff_l2 = laser2_t.transform.translation.x
            self.yoff_l2 = laser2_t.transform.translation.y
            r = R.from_quat([laser2_t.transform.rotation.x, laser2_t.transform.rotation.y, laser2_t.transform.rotation.z, laser2_t.transform.rotation.w])
            rot_array = r.as_euler('zyx', degrees=True)
            self.yaw_l2 = rot_array[0]
            #self.get_logger().info(f'laser2 x_off: {self.xoff_l2}') 
            #self.get_logger().info(f'laser2 y_off: {self.yoff_l2}')
            #self.get_logger().info(f'laser2 yaw: {self.yaw_l2}')
            if l2tf_received:
                self.ScanMerger()
            else:
                l2tf_received = False
        
    def ScanMerger(self):
        global l1tf_received, l2tf_received, l1scan_received, l2scan_received
        
        # Assuming Min range, max range and angle increment for both lidar scan is identical 
        laserscanmerge = LaserScan()
        laserscanmerge.header.frame_id = self.targetframe
        laserscanmerge.header.stamp = self.get_clock().now().to_msg()
        laserscanmerge.angle_min = self.mergescan_minangle
        laserscanmerge.angle_max = self.mergescan_maxangle
        laserscanmerge.angle_increment = self.laser_scan1_.angle_increment
        laserscanmerge.range_min = self.laser_scan1_.range_min
        laserscanmerge.range_max = self.laser_scan1_.range_max
        laserscanmerge.ranges = self.laser_scan1_.ranges + self.laser_scan2_.ranges
        
        pos_minangle = 0
        pos_angleancr = np.rad2deg(self.laser_scan1_.angle_increment)
        
        for i in range(len(self.laser_scan1_.ranges)-90):
            pos_x = (self.laser_scan1_.ranges[i+90]*np.cos(pos_minangle)) + abs(self.xoff_l1)
            pos_y = (self.laser_scan1_.ranges[i+90]*np.sin(pos_minangle)) + self.yoff_l1
            pos_minangle = pos_minangle - pos_angleancr
            res = np.sqrt(pos_x**2+pos_y**2)
            laserscanmerge.ranges[i] = res
            
        for i in range(len(self.laser_scan2_.ranges)):
            pos_x = self.laser_scan2_.ranges[i]*np.cos(pos_minangle) + abs(self.xoff_l2)
            pos_y = self.laser_scan2_.ranges[i]*np.sin(pos_minangle) + self.yoff_l2
            pos_minangle = pos_minangle + pos_angleancr
            res = np.sqrt(pos_x**2+pos_y**2)
            laserscanmerge.ranges[i+90] = res
            
        for i in range(len(self.laser_scan1_.ranges)-90):
            pos_x = self.laser_scan1_.ranges[i]*np.cos(pos_minangle) + abs(self.xoff_l1)
            pos_y = self.laser_scan1_.ranges[i]*np.sin(pos_minangle) + self.yoff_l1
            pos_minangle = pos_minangle - pos_angleancr
            res = np.sqrt(pos_x**2+pos_y**2)
            laserscanmerge.ranges[i+270] = res
            
        self.mergescanpub.publish(laserscanmerge)
        l1tf_received = False
        l2tf_received = False
        l1scan_received = False
        l2scan_received = False

        
def main():
    rclpy.init()
    scanmerger_node = Robotino3LaserScanMerger()
    try:
        rclpy.spin(scanmerger_node)
    except KeyboardInterrupt:
        pass
    scanmerger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
