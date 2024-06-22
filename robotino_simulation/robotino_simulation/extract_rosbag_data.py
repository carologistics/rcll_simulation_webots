#!/usr/bin/python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
from decimal import Decimal

import matplotlib.pyplot as plt
import pandas as pd
import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DataExtractorNode(Node):
    def __init__(self):
        super().__init__("data_extractor")
        self.declare_parameter("namespace", "robotinobase1")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.goal_start_time = None
        self.goal_end_time = None

        # Subscribe to rosout topic
        self.rosout_sub = self.create_subscription(Log, "/rosbag_rosout", self.rosout_callback, 10)

        # Subscribe to namespace plan topic
        # self.plan_sub = self.create_subscription(Path, self.namespace+'/plan', self.plan_callback, 10)

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.namespace + "/base_link"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the lookup table for the poses
        self.machine_pose = {
            "C-BS-I": (5.50, 2.50, 90),
            "C-BS-O": (5.50, 4.50, 270),
            "C-RS2-I": (3.50, 5.50, 180),
            "C-RS2-O": (5.50, 5.50, 0),
            "C-CS1-I": (0.50, 5.50, 180),
            "C-CS1-O": (2.50, 5.50, 0),
            "C-CS2-I": (0.50, 1.50, 180),
            "C-CS2-O": (2.50, 1.50, 0),
            "C-DS-I": (0.50, 0.50, 180),
            "C-DS-O": (2.50, 0.50, 0),
            "C-RS1-I": (1.50, 4.50, 270),
            "C-RS1-O": (1.50, 2.50, 90),
            "C-SS-I": (4.50, 1.50, 90),
            "C-SS-O": (4.50, 3.50, 270),
            "M-BS-I": (-5.50, 2.50, 90),
            "M-BS-O": (-5.50, 4.50, 270),
            "M-RS2-I": (-3.50, 5.50, 0),
            "M-RS2-O": (-5.50, 5.50, 180),
            "M-CS1-I": (-0.50, 5.50, 0),
            "M-CS1-O": (-2.50, 5.50, 180),
            "M-CS2-I": (-0.50, 1.50, 0),
            "M-CS2-O": (-2.50, 1.50, 180),
            "M-DS-I": (-0.50, 0.50, 0),
            "M-DS-O": (-2.50, 0.50, 180),
            "M-RS1-I": (-1.50, 4.50, 270),
            "M-RS1-O": (-1.50, 2.50, 90),
            "M-SS-I": (-4.50, 1.50, 90),
            "M-SS-O": (-4.50, 3.50, 270),
        }

        self.columns = [
            "pose_start_x",
            "pose_start_y",
            "pose_end_x",
            "pose_end_y",
            "machine_start",
            "machine_end",
            "time_start",
            "time_end",
            "time_diff",
        ]
        self.table = pd.DataFrame(columns=self.columns)

        self.columns_ = ["pose_x", "pose_y"]
        self.table_ = pd.DataFrame(columns=self.columns_)

        self.machine_start = "None"
        self.machine_end = "None"
        self.counter = 0

        self.robot_pose_x = []
        self.robot_pose_y = []

        self.plot_data = False
        self.record_data = False

    def rosout_callback(self, msg):
        if msg.name == self.namespace + ".bt_navigator":
            if self.msg_check(msg.msg, "Begin navigating"):
                self.goal_start_time = self.get_time(msg)
                self.pose_data = self.machine_pose_check(msg.msg)
                self.machine_end = self.pose_data[4]

                data_row = {
                    "pose_start_x": [self.pose_data[0]],
                    "pose_start_y": [self.pose_data[1]],
                    "pose_end_x": [self.pose_data[2]],
                    "pose_end_y": [self.pose_data[3]],
                    "machine_start": [self.machine_start],
                    "machine_end": [self.machine_end],
                    "time_start": [self.goal_start_time],
                }
                self.table = pd.concat([self.table, pd.DataFrame(data_row)], ignore_index=True)

            if msg.msg == "Goal succeeded":
                self.goal_end_time = self.get_time(msg)
                self.table.at[self.counter, "time_end"] = self.goal_end_time
                # self.table.at[self.counter, "time_diff"] = self.goal_end_time - self.goal_start_time
                self.machine_start = self.machine_end
                self.counter += 1
                if self.counter in [1, 5, 9, 13, 17]:
                    self.record_data = True
                elif self.counter in [4, 8, 12, 16, 20]:
                    self.record_data = False
                    self.plot_data = True
                # if self.counter == 20:
                #     self.table.to_csv(self.namespace+"_pose_data.csv", index=False)
                #     self.get_logger().info(f'Pose data saved, check rosbag status and shutdown the node')

    def msg_check(self, message, check):
        words = message.split()
        first_two = " ".join(words[:2])
        return first_two == check

    def machine_pose_check(self, message):
        words = message.split()
        start_pose_x = float(words[5][1:-1])
        start_pose_y = float(words[6][:-1])
        end_pose_x = float(words[8][1:-1])
        end_pose_y = float(words[9][:-1])
        for key, value in self.machine_pose.items():
            if end_pose_x == value[0] and end_pose_y == value[1]:
                machine_end = key
                self.get_logger().info(f"machine end: {machine_end}")
                return [start_pose_x, start_pose_y, end_pose_x, end_pose_y, machine_end]

    def get_time(self, msg):
        return Decimal(msg.stamp.sec) + (Decimal(msg.stamp.nanosec) * Decimal(1e-9))

    def on_timer(self):
        if self.record_data:
            try:
                t = self.tf_buffer.lookup_transform("map", self.target_frame, rclpy.time.Time())

            except TransformException as ex:
                self.get_logger().info(f"Could not transform {self.target_frame} to map: {ex}")
                return
            self.robot_pose_x.append(t.transform.translation.x)
            self.robot_pose_y.append(t.transform.translation.y)

            data_row = {"pose_x": [t.transform.translation.x], "pose_y": [t.transform.translation.y]}
            self.table_ = pd.concat([self.table_, pd.DataFrame(data_row)], ignore_index=True)

        if self.plot_data:
            quo, rem = divmod(self.counter, 4)

            plt.figure(quo)

            plt.clf()
            plt.plot(self.robot_pose_x, self.robot_pose_y, "-b")
            plt.xlabel("X_Position")
            plt.ylabel("Y_Position")
            plt.title(self.namespace + "_iteration" + str(quo) + "_followed_Path")
            plt.grid(True)
            plt.xlim(-6, 6)
            plt.ylim(0, 6)
            plt.gca().set_aspect("equal")

            plt.text(self.robot_pose_x[0], self.robot_pose_y[0], "Start", fontsize=12, color="green", ha="right")
            plt.text(self.robot_pose_x[-1], self.robot_pose_y[-1], "End", fontsize=12, color="red", ha="right")

            plt.savefig(
                self.namespace + "_iteration" + str(quo) + "_followed_path" + ".png"
            )  # Save the plot as PNG file
            self.table_.to_csv(self.namespace + "_iteration" + str(quo) + "_pose_data.csv", index=False)

            self.columns_ = ["pose_x", "pose_y"]
            self.table_ = pd.DataFrame(columns=self.columns_)

            self.get_logger().info(f"Plotting followed path data for iteration {quo}")

            plt.close()

            self.robot_pose_x = []
            self.robot_pose_y = []
            self.plot_data = False

    # def plan_callback(self, msg):
    #     #self.get_logger().info(f'Plan callback called:{msg}')
    #     if self.machine_start != "None":
    #         self.plan_pose_x.append(msg.poses[1].pose.position.x)
    #         self.plan_pose_y.append(msg.poses[1].pose.position.y)

    #         quo, rem = divmod(self.counter, 4)

    #         if not self.plot_plan_data and rem == 0:
    #             #if rem == 0:
    #             self.plot_plan_data = True

    #             plt.figure(quo+6)
    #             plt.clf()
    #             plt.plot(self.plan_pose_x, self.plan_pose_y, '-b')
    #             plt.xlabel('X_Position')
    #             plt.ylabel('Y_Position')
    #             plt.title(self.namespace+'_iteration'+str(quo)+'_planned_Path')
    #             plt.grid(True)
    #             plt.xlim(-6, 6)
    #             plt.ylim(0, 6)
    #             plt.savefig(self.namespace+'_iteration'+str(quo)+'_planned_path'+'.png')  # Save the plot as PNG file

    #             self.get_logger().info(f'plotx:{self.plan_pose_x}')

    #             self.get_logger().info(f'Plotting plan data, wait')

    #             plt.close()

    #             self.plan_pose_x = []
    #             self.plan_pose_y = []


def main(args=None):
    rclpy.init(args=args)
    node = DataExtractorNode()
    try:
        node.get_logger().info("Beginning data extraction node, shut down with CTRL-C")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
