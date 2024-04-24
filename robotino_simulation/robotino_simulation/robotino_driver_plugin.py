# Author: Saurabh Borse(saurabh.borse@alumni.fh-aachen.de)
#  MIT License
#  Copyright (c) 2023 Saurabh Borse
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
import math
from math import cos
from math import sin

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# Define robot parameters
wheel_distance = 0.1826
wheel_radius = 0.063
gear_ratio = 1


class Robotino3Driver:
    def init(self, webots_node, properties):
        # Initialize webots_node to initiate robot instance
        self.__robot = webots_node.robot
        self.robotini3_node = self.__robot.getFromDef("robotino")
        self.__robot.timestep = 32

        # Define motors for joints/ position sensors and set intitial position and velocity
        self.motor_names = ["wheel0_joint", "wheel1_joint", "wheel2_joint"]
        self.motors = []
        for motor_name in self.motor_names:
            self.motors.append(self.__robot.getDevice(motor_name))
        self.motor_posSensor_names = ["wheel0_joint_sensor", "wheel1_joint_sensor", "wheel2_joint_sensor"]
        self.motor_sensors = []
        self.motors_pos = []
        for idx, sensor_name in enumerate(self.motor_posSensor_names):
            self.motor_sensors.append(self.__robot.getDevice(sensor_name))
            self.motor_sensors[idx].enable(self.__robot.timestep)
            self.motors_pos.append(0.0)
        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        # Initialize ROS2_Node and list the no.of devices
        rclpy.init(args=None)
        self.drive_node = rclpy.create_node("robotino_driver_plugin")
        n_devices = self.__robot.getNumberOfDevices()
        self.drive_node.get_logger().info(f"num_devices: {n_devices}")
        index = 0
        while index < n_devices:
            self.drive_node.get_logger().info("index: " + str(index) + " " + str(self.__robot.getDeviceByIndex(index)))
            index += 1

        self.device_names = self.__robot.devices
        self.drive_node.get_logger().info(f"num_devices: {self.device_names}")

        # Initialize GPS sensor
        self.gps_sensor = self.__robot.getDevice("gps")
        self.gps_sensor.enable(self.__robot.timestep)

        # Initialize IMU sensor
        self.inertial_unit = self.__robot.getDevice("inertial unit")
        self.inertial_unit.enable(self.__robot.timestep)

        # Initialize Gyro sensor
        self.gyro = self.__robot.getDevice("gyro")
        self.gyro.enable(self.__robot.timestep)

        # Initialize Accelerometer sensor
        self.acc_sensor = self.__robot.getDevice("accelerometer")
        self.acc_sensor.enable(self.__robot.timestep)

        self.__target_twist = Twist()
        self.tfb_ = TransformBroadcaster(self.drive_node)

        self.drive_node.get_logger().info("Init_RobotinoDriver")

        # Initialize subscription:/cmd_vel and publishers:/odom & /joint_states
        self.drive_node.create_subscription(Twist, "/cmd_vel", self.CmdVel_cb, 1)
        self.odom_pub = self.drive_node.create_publisher(Odometry, "/odom", 1)
        self.joint_state_pub = self.drive_node.create_publisher(JointState, "/joint_states", 1)

        self.prev_wheel0_ticks = 0.0
        self.prev_wheel1_ticks = 0.0
        self.prev_wheel2_ticks = 0.0
        self.prev_odom_x = 0.0
        self.prev_odom_y = 0.0
        self.prev_odom_omega = 0.0
        self.last_odometry_sample_time = self.get_time()

    def get_time(self):
        # Time in seconds
        return float(self.drive_node.get_clock().now().to_msg()._sec) + (
            float(self.drive_node.get_clock().now().to_msg()._nanosec) * 1e-9
        )

    def CmdVel_cb(self, msg):
        self.__target_twist = msg

    def TransformAndOdometry_wheelodom(self):
        # Initialze, calculate and publidh wheel odometry
        time_stamp = self.drive_node.get_clock().now().to_msg()
        time_diff = self.get_time() - self.last_odometry_sample_time

        for idx, motor_sensor in enumerate(self.motor_sensors):
            self.motors_pos[idx] = motor_sensor.getValue()

        wheel0_rad_s = (self.motors_pos[0] - self.prev_wheel0_ticks) / time_diff
        wheel1_rad_s = (self.motors_pos[1] - self.prev_wheel1_ticks) / time_diff
        wheel2_rad_s = (self.motors_pos[2] - self.prev_wheel2_ticks) / time_diff

        wheel0_rpm = wheel0_rad_s * 9.5493
        wheel1_rpm = wheel1_rad_s * 9.5493
        wheel2_rpm = wheel2_rad_s * 9.5493

        m1 = wheel2_rpm
        m2 = wheel0_rpm
        m3 = wheel1_rpm

        # Convert from RPM to mm/s
        k = 60.0 * gear_ratio / (2 * math.pi * wheel_radius)
        vx = -(m3 - m1) / math.sqrt(3.0) / k
        vy = -((2.0 / 3.0) * (m1 + 0.5 * (m3 - m1) - m2) / k)
        vw = -vy + m2 / k
        omega = -(vw / wheel_distance)
        res_phi = self.prev_odom_omega + omega * time_diff
        x = self.prev_odom_x + (vx * cos(res_phi) - vy * sin(res_phi)) * time_diff
        y = self.prev_odom_y + (vx * sin(res_phi) + vy * cos(res_phi)) * time_diff
        q = [0.0, 0.0, sin(res_phi / 2), cos(res_phi / 2)]

        # Pack & publish odometry
        msg = Odometry()
        msg.header.stamp = time_stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Velocity
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = omega
        msg.twist.covariance[0] = 0.1  # <x
        msg.twist.covariance[7] = 0.1  # <y
        msg.twist.covariance[35] = 0.1  # <yaw

        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Uncertainty in position
        # major representation of the 6x6 covariance matrix -> 36 values
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[1] = 0.0
        msg.pose.covariance[2] = 0.0
        msg.pose.covariance[3] = 0.0
        msg.pose.covariance[4] = 0.0
        msg.pose.covariance[5] = 0.0
        msg.pose.covariance[6] = 0.0
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[8] = 0.0
        msg.pose.covariance[9] = 0.0
        msg.pose.covariance[10] = 0.0
        msg.pose.covariance[11] = 0.0
        msg.pose.covariance[12] = 0.0
        msg.pose.covariance[13] = 0.0
        msg.pose.covariance[14] = 0.1
        msg.pose.covariance[15] = 0.0
        msg.pose.covariance[16] = 0.0
        msg.pose.covariance[17] = 0.0
        msg.pose.covariance[18] = 0.0
        msg.pose.covariance[19] = 0.0
        msg.pose.covariance[20] = 0.0
        msg.pose.covariance[21] = 0.1
        msg.pose.covariance[22] = 0.0
        msg.pose.covariance[23] = 0.0
        msg.pose.covariance[24] = 0.0
        msg.pose.covariance[25] = 0.0
        msg.pose.covariance[26] = 0.0
        msg.pose.covariance[27] = 0.0
        msg.pose.covariance[28] = 0.1
        msg.pose.covariance[29] = 0.1
        msg.pose.covariance[30] = 0.1
        msg.pose.covariance[31] = 0.1
        msg.pose.covariance[32] = 0.1
        msg.pose.covariance[33] = 0.1
        msg.pose.covariance[34] = 0.1
        msg.pose.covariance[35] = 0.1

        self.odom_pub.publish(msg)

        self.prev_wheel0_ticks = self.motors_pos[0]
        self.prev_wheel1_ticks = self.motors_pos[1]
        self.prev_wheel2_ticks = self.motors_pos[2]

        # Compose and publish trnasform:odom
        tfs = TransformStamped()
        tfs.header.stamp = time_stamp
        tfs.header.frame_id = "odom"
        tfs._child_frame_id = "base_link"
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.translation.z = 0.0
        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]
        self.tfb_.sendTransform(tfs)

        self.prev_odom_x = x
        self.prev_odom_y = y
        self.prev_odom_omega = res_phi

        # Compose and publish:Joint_states
        joint_state = JointState()
        joint_state.header.stamp = time_stamp
        joint_state.name = []
        joint_state.name.extend(self.motor_names)
        joint_state.position = []
        joint_state.position.extend(self.motors_pos)
        qty = len(self.motor_names)
        joint_state.velocity = [0.0 for _ in range(qty)]
        joint_state.effort = [0.0 for _ in range(qty)]
        self.joint_state_pub.publish(joint_state)

        self.last_odometry_sample_time = self.get_time()

    def TransformIrsensor(self):

        # Publish transform for Ir_sensor w.r.t to base link
        self.ir_sensor_list = [
            "ir1_sensor",
            "ir2_sensor",
            "ir3_sensor",
            "ir4_sensor",
            "ir5_sensor",
            "ir6_sensor",
            "ir7_sensor",
            "ir8_sensor",
            "ir9_sensor",
        ]

        self.ir_sensor_pos = [
            [0, -0.2264, 0.0566],
            [0.1454, -0.1736, 0.0566],
            [0.223, -0.0394, 0.0566],
            [0.1963, 0.114, 0.0566],
            [0.0777, 0.2131, 0.0566],
            [-0.0772, 0.2133, 0.0566],
            [-0.196, 0.1133, 0.0566],
            [-0.2229, -0.0394, 0.0566],
            [-0.1455, -0.1735, 0.0566],
        ]

        self.ir_sensor_orientation = [
            [0, 0, -0.7071068, 0.7071068],
            [0, 0, -0.4226183, 0.9063078],
            [0, 0, -0.0871557, 0.9961947],
            [0, 0, 0.258819, 0.96592581],
            [0, 0, 0.5735764, 0.819152],
            [0, 0, 0.819152, 0.5735764],
            [0, 0, 0.9659258, 0.258819],
            [0, 0, 0.9961947, -0.0871557],
            [0, 0, 0.9063078, -0.4226183],
        ]

        time_stamp = self.drive_node.get_clock().now().to_msg()

        for i in range(len(self.ir_sensor_list)):
            tf = TransformStamped()
            tf.header.stamp = time_stamp
            tf.header.frame_id = "base_link"
            tf._child_frame_id = self.ir_sensor_list[i]
            tf.transform.translation.x = float(self.ir_sensor_pos[i][0])
            tf.transform.translation.y = float(self.ir_sensor_pos[i][1])
            tf.transform.translation.z = float(self.ir_sensor_pos[i][2])
            tf.transform.rotation.x = float(self.ir_sensor_orientation[i][0])
            tf.transform.rotation.y = float(self.ir_sensor_orientation[i][1])
            tf.transform.rotation.z = float(self.ir_sensor_orientation[i][2])
            tf.transform.rotation.w = float(self.ir_sensor_orientation[i][3])
            self.tfb_.sendTransform(tf)

    def TrnsformLaserSensor(self):

        self.laser_sensor_list = ["SickLaser_Rear", "SickLaser_Front", "laser_link"]

        self.lidar_pos = [[0.12, 0, 0.3], [-0.13, 0, 0.35], [0, 0, 0.3]]  # SickLaser_Rear  # SickLaser_Front

        self.lidar_orientation = [[0, 0, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]]  # SickLaser_Rear  # SickLaser_Front

        for i in range(len(self.laser_sensor_list)):
            tf = TransformStamped()
            tf.header.stamp = self.drive_node.get_clock().now().to_msg()
            tf.header.frame_id = "base_link"
            tf._child_frame_id = self.laser_sensor_list[i]
            tf.transform.translation.x = float(self.lidar_pos[i][0])
            tf.transform.translation.y = float(self.lidar_pos[i][1])
            tf.transform.translation.z = float(self.lidar_pos[i][2])
            tf.transform.rotation.x = float(self.lidar_orientation[i][0])
            tf.transform.rotation.y = float(self.lidar_orientation[i][1])
            tf.transform.rotation.z = float(self.lidar_orientation[i][2])
            tf.transform.rotation.w = float(self.lidar_orientation[i][3])
            self.tfb_.sendTransform(tf)

    def TransformImuSensor(self):

        self.imusensor = "Inertial_Unit"
        self.imu_pose = [0.0, -0.1, 0.2]
        self.imu_orientation = [0, 0, 1, 0]

        tf = TransformStamped()
        tf.header.stamp = self.drive_node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf._child_frame_id = self.imusensor
        tf.transform.translation.x = float(self.imu_pose[0])
        tf.transform.translation.y = float(self.imu_pose[1])
        tf.transform.translation.z = float(self.imu_pose[2])
        tf.transform.rotation.x = float(self.imu_orientation[0])
        tf.transform.rotation.y = float(self.imu_orientation[1])
        tf.transform.rotation.z = float(self.imu_orientation[2])
        tf.transform.rotation.w = float(self.imu_orientation[3])
        self.tfb_.sendTransform(tf)

    def step(self):

        rclpy.spin_once(self.drive_node, timeout_sec=0)

        # Compute motor velocity based on linear and angular component of /cmd_vel
        # set individual motor velocity based on calculation
        v_x = self.__target_twist.linear.x
        v_y = self.__target_twist.linear.y
        omega = self.__target_twist.angular.z
        v_x = v_x / wheel_radius
        v_y = v_y / wheel_radius
        omega = omega * (wheel_distance / wheel_radius)
        self._v_x = v_x
        self._omega = omega
        self.motors[0].setVelocity(v_y - omega)
        self.motors[1].setVelocity(-math.sqrt(0.75) * v_x - 0.5 * v_y - omega)
        self.motors[2].setVelocity(math.sqrt(0.75) * v_x - 0.5 * v_y - omega)

        # Publish Transformation/ Odomebtry and and Joint_states
        self.TransformAndOdometry_wheelodom()
        self.TransformIrsensor()
        self.TrnsformLaserSensor()
        self.TransformImuSensor()
