
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.clock import Clock
from tf2_ros import TransformBroadcaster
import math
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from math import cos, dist, sin
from builtin_interfaces.msg import Time

wheel_distance = 0.1826
wheel_radius = 0.063
gear_ratio = 1
robotino_spwanpose_x = 5.5
robotino_spawanpose_y = 3.65

class Robotino3Driver:
    def init(self, webots_node, properties):
        # Initialize webots_node to initiate robot instance
        self.__robot = webots_node.robot
        self.robotini3_node = self.__robot.getFromDef("robotinobase1")
        self.__robot.timestep = 32
        
        # Define motors for joints/ position sensors and set intitial position and velocity
        self.motor_names = ['wheel0_joint', 'wheel1_joint', 'wheel2_joint']
        self.motors = []
        for motor_name in self.motor_names:
            self.motors.append(self.__robot.getDevice(motor_name))
        self.motor_posSensor_names = ['wheel0_joint_sensor', 'wheel1_joint_sensor', 'wheel2_joint_sensor']
        self.motor_sensors = []
        self.motors_pos = []
        for idx, sensor_name in enumerate(self.motor_posSensor_names):
            self.motor_sensors.append(self.__robot.getDevice(sensor_name))
            self.motor_sensors[idx].enable(self.__robot.timestep)
            self.motors_pos.append(0.)
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
                
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
        
        # INitialize ROS2_Node and list the no.of devices
        rclpy.init(args=None)
        self.ros_clock = Clock()
        self.drive_node = Node('robotinobase1_driver_plugin', namespace='robotinobase1')
        
        self.device_names = self.__robot.devices
        self.drive_node.get_logger().info(f'num_devices: {self.device_names}')
        
        n_devices = self.__robot.getNumberOfDevices()
        index = 0
        while index < n_devices:
            self.drive_node.get_logger().info("index: "+ str(index)+" "+ str(self.__robot.getDeviceByIndex(index)))
            index += 1
        self.drive_node.get_logger().info(f'num_devices: {n_devices}')
        
        self.drive_node.get_logger().info('Init_RobotinoDriver')
        
        self.__target_twist = Twist()
        self.tfb_ = TransformBroadcaster(self.drive_node)        

        # Initialize subscription:/cmd_vel and publishers:/odom & /joint_states
        self.drive_node.create_subscription(Twist, self.drive_node.get_namespace()+'/cmd_vel', self.CmdVel_cb, 1)
        self.odom_pub = self.drive_node.create_publisher(Odometry, self.drive_node.get_namespace()+'/odom', 1)
        self.joint_state_pub = self.drive_node.create_publisher(JointState, self.drive_node.get_namespace()+'/joint_states', 1)
        
        self.prev_wheel0_ticks = 0.0
        self.prev_wheel1_ticks = 0.0
        self.prev_wheel2_ticks = 0.0
        self.prev_odom_x = 0.0
        self.prev_odom_y = 0.0
        self.prev_odom_omega = 0.0
        self.last_odometry_sample_time = self.__robot.getTime()
    
    def get_time(self):
    # Time in seconds
        return float(self.drive_node.get_clock().now().to_msg()._sec) + (float(self.drive_node.get_clock().now().to_msg()._nanosec) * 1e-9)

    def CmdVel_cb(self, msg):
        self.__target_twist = msg
        
    def TransformAndOdometry_wheelodom(self):
        
        # Initialize time stamp and calaculate time dtep for odometry calculations 
        current_time = self.__robot.getTime()
        time_stamp = Time()
        time_stamp.sec = int(current_time)
        time_stamp.nanosec = int((current_time % 1) * 1e9)
        
        time_diff = current_time - self.last_odometry_sample_time
        self.last_odometry_sample_time = self.get_time()
        
        for idx, motor_sensor in enumerate(self.motor_sensors):
            self.motors_pos[idx] = motor_sensor.getValue()
        
        wheel0_ticks = self.motors_pos[0]
        wheel1_ticks = self.motors_pos[1]
        wheel2_ticks = self.motors_pos[2]

        wheel0_rad_s = (self.motors_pos[0] - self.prev_wheel0_ticks) / time_diff
        wheel1_rad_s = (self.motors_pos[1] - self.prev_wheel1_ticks) / time_diff
        wheel2_rad_s = (self.motors_pos[2] - self.prev_wheel2_ticks) / time_diff
          
        self.prev_wheel0_ticks = wheel0_ticks
        self.prev_wheel1_ticks = wheel1_ticks
        self.prev_wheel2_ticks = wheel2_ticks
        
        w0 = wheel0_rad_s
        w1 = wheel1_rad_s
        w2 = wheel2_rad_s
                
        velocity = self.InverseKinematic_Calc(w0,w1,w2)
        res_phi = self.prev_odom_omega + (velocity[2] * time_diff)
        
        x = self.prev_odom_x + ((velocity[0] * cos(res_phi)) - (velocity[1] * sin(res_phi)) * time_diff)
        y = self.prev_odom_y + ((velocity[0]  * sin(res_phi)) + (velocity[1] * cos(res_phi)) * time_diff)
        q = [0.0, 0.0, sin(res_phi/2), cos(res_phi/2)]
        
        self.prev_odom_x = x
        self.prev_odom_y = y
        self.prev_odom_omega = res_phi
    
        # Pack & publish odometry
        msg = Odometry()
        msg.header.stamp = time_stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # Velocity
        msg.twist.twist.linear.x = velocity[0]
        msg.twist.twist.linear.y = velocity[1]
        msg.twist.twist.angular.z = velocity[2]
        #Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(msg)
        
        # Compose and publish trnasform:odom
        tfs = TransformStamped()
        tfs.header.stamp = time_stamp
        tfs.header.frame_id= self.drive_node.get_namespace()+'/'+"odom"
        tfs._child_frame_id = self.drive_node.get_namespace()+'/'+"base_link"
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.translation.z = 0.0
        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]
        self.tfb_.sendTransform(tfs)
          
        # Compose and publish:Joint_states  
        joint_state = JointState()
        joint_state.header.stamp = time_stamp
        joint_state.name = []
        joint_state.name.extend(self.motor_names)
        joint_state.position = []
        joint_state.position.extend(self.motors_pos) 
        qty = (len(self.motor_names))
        joint_state.velocity = [0. for _ in range(qty)]
        joint_state.effort = [0. for _ in range(qty)]
        self.joint_state_pub.publish(joint_state)
        
    def TransformAndOdometry_imugps(self):
        #Initilize gps/imu and gyro sensor
        gps = self.gps_sensor.getValues()
        linear_twist = self.gps_sensor.getSpeedVector()
        imu = self.inertial_unit.getRollPitchYaw()
        gyro = self.gyro.getValues()
        
        current_time = self.__robot.getTime()
        time_stamp = Time()
        time_stamp.sec = int(current_time)
        time_stamp.nanosec = int((current_time % 1) * 1e9)
        
        # Compose and publish trnasform:odom
        tfs = TransformStamped()
        tfs.header.stamp = time_stamp
        tfs.header.frame_id= self.drive_node.get_namespace()+'/'+"odom"
        tfs._child_frame_id = self.drive_node.get_namespace()+'/'+"base_link"
        tfs.transform.translation.x = gps[0] + robotino_spwanpose_x
        tfs.transform.translation.y = gps[1] - robotino_spawanpose_y
        tfs.transform.translation.z = gps[2]
        r = R.from_euler('xyz',[imu[0],imu[1],imu[2]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        # Compose and publish Odometry
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = time_stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = gps[0]
        odom.pose.pose.position.y = gps[1]
        odom.pose.pose.position.z = gps[2]
        r = R.from_euler('xyz',[imu[0],imu[1],imu[2]])
        odom.pose.pose.orientation.x = r.as_quat()[0]
        odom.pose.pose.orientation.y = r.as_quat()[1]
        odom.pose.pose.orientation.z = r.as_quat()[2]
        odom.pose.pose.orientation.w = r.as_quat()[3]
        odom.twist.twist.linear.x = linear_twist[0]
        odom.twist.twist.linear.y = linear_twist[1]
        odom.twist.twist.linear.z = linear_twist[2]
        odom.twist.twist.angular.x = gyro[0]
        odom.twist.twist.angular.y = gyro[1]
        odom.twist.twist.angular.z = gyro[2]
        self.odom_pub.publish(odom)
        
        for idx, motor_sensor in enumerate(self.motor_sensors):
            self.motors_pos[idx] = motor_sensor.getValue()
          
        # Compose and publish Joint_states  
        joint_state = JointState()
        joint_state.header.stamp = time_stamp
        joint_state.name = []
        joint_state.name.extend(self.motor_names)
        joint_state.position = []
        joint_state.position.extend(self.motors_pos) 
        qty = (len(self.motor_names))
        joint_state.velocity = [0. for _ in range(qty)]
        joint_state.effort = [0. for _ in range(qty)]
        self.joint_state_pub.publish(joint_state)
        
    def TransformIrsensor(self):
        
        # Publish transform for Ir_sensor w.r.t to base link 
        self.ir_sensor_list = ['ir1_sensor', 'ir2_sensor', 'ir3_sensor', 
                               'ir4_sensor', 'ir5_sensor', 'ir6_sensor', 
                               'ir7_sensor', 'ir8_sensor', 'ir9_sensor', 'irsensor_merge'] 
        
        self.ir_sensor_pos = [[0, -0.2264, 0.0566],
                              [0.1454, -0.1736, 0.0566],
                              [0.223, -0.0394, 0.0566],
                              [0.1963, 0.114, 0.0566],
                              [0.0777, 0.2131, 0.0566],
                              [-0.0772, 0.2133, 0.0566],
                              [-0.196, 0.1133, 0.0566],
                              [-0.2229, -0.0394, 0.0566],
                              [-0.1455, -0.1735, 0.0566], 
                              [0, 0 , 0.0566]] 
         
        self.ir_sensor_orientation = [[0, 0, -0.7071068, 0.7071068],
                                      [0, 0, -0.4226183, 0.9063078],
                                      [0, 0, -0.0871557, 0.9961947],
                                      [0, 0, 0.258819, 0.96592581],
                                      [0, 0, 0.5735764, 0.819152],
                                      [0, 0, 0.819152, 0.5735764],
                                      [0, 0, 0.9659258, 0.258819],
                                      [0, 0, 0.9961947, -0.0871557],
                                      [0, 0, 0.9063078, -0.4226183],
                                      [0, 0 , 0, 1]] 
        
        current_time = self.__robot.getTime()
        time_stamp = Time()
        time_stamp.sec = int(current_time)
        time_stamp.nanosec = int((current_time % 1) * 1e9)
        
        for i in range(len(self.ir_sensor_list)):
            tf = TransformStamped()
            tf.header.stamp = time_stamp
            tf.header.frame_id= self.drive_node.get_namespace()+'/'+"base_link"
            tf._child_frame_id = self.drive_node.get_namespace()+'/'+self.ir_sensor_list[i]
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
        
        self.lidar_pos = [[0.12, 0, 0.3],#SickLaser_Rear
                          [-0.13, 0, 0.35],
                          [0, 0 ,0.3]]#SickLaser_Front
        
        self.lidar_orientation = [[0, 0, 0, 1],#SickLaser_Rear
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]]#SickLaser_Front
        current_time = self.__robot.getTime()
        time_stamp = Time()
        time_stamp.sec = int(current_time)
        time_stamp.nanosec = int((current_time % 1) * 1e9)
        
        for i in range(len(self.laser_sensor_list)):
            tf = TransformStamped()
            tf.header.stamp = time_stamp
            tf.header.frame_id= self.drive_node.get_namespace()+'/'+"base_link"
            tf._child_frame_id = self.drive_node.get_namespace()+'/'+self.laser_sensor_list[i]
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
        
        current_time = self.__robot.getTime()
        time_stamp = Time()
        time_stamp.sec = int(current_time)
        time_stamp.nanosec = int((current_time % 1) * 1e9)
        
        tf = TransformStamped()
        tf.header.stamp = time_stamp
        tf.header.frame_id= self.drive_node.get_namespace()+'/'+"base_link"
        tf._child_frame_id = self.drive_node.get_namespace()+'/'+self.imusensor
        tf.transform.translation.x = float(self.imu_pose[0])
        tf.transform.translation.y = float(self.imu_pose[1])
        tf.transform.translation.z = float(self.imu_pose[2])
        tf.transform.rotation.x = float(self.imu_orientation[0])
        tf.transform.rotation.y = float(self.imu_orientation[1])
        tf.transform.rotation.z = float(self.imu_orientation[2])
        tf.transform.rotation.w = float(self.imu_orientation[3])
        self.tfb_.sendTransform(tf)
        
    def Kinematic_Calc(self):
        # Algorithm1: Linear kinematic model for omniwheel drive (Angular velocity from linear velocity) 
        v_x = self.__target_twist.linear.x
        v_y = self.__target_twist.linear.y
        omega = self.__target_twist.angular.z
        v_x = v_x / wheel_radius
        v_y = v_y / wheel_radius
        omega = (omega * wheel_distance)/ wheel_radius
        self._v_x = v_x
        self._omega = omega
        
        m1=(((math.sqrt(3)/2)* v_x) - (0.5* v_y) - omega)
        m2=(v_y - omega)
        m3=(-((math.sqrt(3)/2)* v_x) - (0.5* v_y) - omega) 
         
        return[m1,m2,m3]
    
    def InverseKinematic_Calc(w0, w1, w2):
        # Algorithm 1: Inverse kimeatic calculation (linear velocity from angular velocity)
        vx = ((-w1+w2)/math.sqrt(3.0))*wheel_radius
        vy = (((2/3)*w0) - ((1/3)*w1) - ((1/3)*w2))*wheel_radius
        const = 3*wheel_distance
        omega  = (-((1/const)*w0)-((1/const)*w1)-((1/const)*w2))*wheel_radius
        return[vx, vy, omega]
        
    def step(self):
        
        rclpy.spin_once(self.drive_node, timeout_sec=0)
    
        angular_velocity = self.Kinematic_Calc()
        
        self.motors[2].setVelocity(angular_velocity[0])
        self.motors[0].setVelocity(angular_velocity[1])
        self.motors[1].setVelocity(angular_velocity[2])
        
        # Publish Transformation/ Odomebtry and and Joint_states 
        #self.TransformAndOdometry_wheelodom()
        self.TransformAndOdometry_imugps()
        self.TransformIrsensor()
        self.TrnsformLaserSensor()
        self.TransformImuSensor()
        