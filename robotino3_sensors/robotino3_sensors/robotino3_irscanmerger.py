
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs.msg import Range
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

num_ir_sensors = 9
robotino_base_radius = 0.225

class Robotino3IrScanMerger(Node):

    def __init__(self):
        super().__init__('robotino3_irscanmerger', namespace='' )
        
        self.Laserscan_qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=5)
        
        self.declare_parameter('frame_prefix', 'robotinobase1')
        
        for i in range(num_ir_sensors):
            self.subscribers = [] 
            self.topic = self.get_namespace()+f"/ir{i+1}"
            self.callback_fcn = getattr(self, f"IrScan_cb_{i+1}")
            subscriber = self.create_subscription(Range, self.topic, self.callback_fcn, 10)
            self.subscribers.append(subscriber)
            self.get_logger().info(f"Subscriber {i} created for topic {self.topic}")
            # self.ir_scan_range_init = getattr(self, f"ir{i+1}_scan_range")
            # self.ir_scan_range_init = 0.0
        self.publisher = self.create_publisher(LaserScan, self.get_namespace()+'/ir_scan_merged', qos_profile=self.Laserscan_qos_profile)    
        self.ir1_scan_range = 0.0 
        self.ir2_scan_range = 0.0
        self.ir3_scan_range = 0.0
        self.ir4_scan_range = 0.0
        self.ir5_scan_range = 0.0
        self.ir6_scan_range = 0.0
        self.ir7_scan_range = 0.0
        self.ir8_scan_range = 0.0
        self.ir9_scan_range = 0.0       
        self.timer = self.create_timer(0.0333 , self.On_Timer)
        
    def IrScan_cb_1(self, msg):
        self.ir1_scan_range = msg.range
        
    def IrScan_cb_2(self, data):
        self.ir2_scan_range = data.range

    def IrScan_cb_3(self, data):
        self.ir3_scan_range = data.range
        
    def IrScan_cb_4(self, data):
        self.ir4_scan_range = data.range
        
    def IrScan_cb_5(self, data):
        self.ir5_scan_range = data.range
         
    def IrScan_cb_6(self, data):
        self.ir6_scan_range = data.range
        
    def IrScan_cb_7(self, data):
        self.ir7_scan_range = data.range 
        
    def IrScan_cb_8(self, data):
        self.ir8_scan_range = data.range

    def IrScan_cb_9(self, data):
        self.ir9_scan_range = data.range
        
    def On_Timer(self):
        msg = LaserScan()
        msg.header.frame_id = self.get_parameter('frame_prefix').get_parameter_value().string_value+'/'+'irsensor_merge'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min       = 0.0
        msg.angle_max       = 2 * math.pi
        msg.angle_increment = 0.6982
        msg.range_min       = 0.02
        msg.range_max       = 0.5+robotino_base_radius
        for i in range(num_ir_sensors):
            self.ir_scan_range = getattr(self, f"ir{i+1}_scan_range")
            msg.ranges.append(self.ir_scan_range + robotino_base_radius)
        self.publisher.publish(msg)
        
def main():
    rclpy.init()
    scanmerger_node = Robotino3IrScanMerger()
    try:
        rclpy.spin(scanmerger_node)
    except KeyboardInterrupt:
        pass
    scanmerger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()