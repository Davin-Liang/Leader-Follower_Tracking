#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import numpy as np

import rclpy
from rclpy.node import Node
from threading import Thread
import tf_transformations

from lpc import Lpc_Controller

class Lft_Onni_Robots(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("I am %s!" % name)
        self.leader_robot_odom_sub_ = self.create_subscription(Odometry, "/leader/odom", self.leader_robot_odom_callback_, 10)
        self.slave_robot_odom_sub_ = self.create_subscription(Odometry, "/slave/odom", self.slave_robot_odom_callback_, 10)
        self.slave_robot_cmd_vel_pub_ = self.create_publisher(Twist, "/slave/cmd_vel", 5)

        self.slave_move_cmd_ = Twist()

        self.leader_yaw = 0.0
        self.slave_yaw = 0.0

        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus below------------------------------------------------
        # ----------------------------------------------------------------------------------------------------

        self.Lpc = Lpc_Controller(m_p=4, radius=1, tol=0.1, m=10.2)

        # Variables in Paper. TODO
        self.x1       = np.array([10.0, 1.0, 0.0, 0.0]) # state variable of leader robot
        self.x2       = np.array([1.0, 1.0, 0.0, 0.0]) # state variable of slave robot

        self.Lpc.controller_initial_(x1=self.x1, x2=self.x2)

        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus above------------------------------------------------
        # ----------------------------------------------------------------------------------------------------

        self.work_timer = self.create_timer(0.002, self.timer_work_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()


    def turn_output_into_cmd_vel_(self, output):
        self.slave_move_cmd_.linear.x = output[0]
        self.slave_move_cmd_.linear.y = output[1]
        self.slave_move_cmd_.angular.z = (self.leader_yaw - self.slave_yaw) * 10.0

        self.slave_robot_cmd_vel_pub_.publish(self.slave_move_cmd_)

    def leader_robot_odom_callback_(self, msg):
        self.x1[0] = msg.pose.pose.position.x
        self.x1[1] = msg.pose.pose.position.y
        self.x1[2] = msg.twist.twist.linear.x
        self.x1[3] = msg.twist.twist.linear.y

        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)

        # 将四元数转换为欧拉角 (roll, pitch, yaw)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.leader_yaw = euler[2]
        # print("leader_yaw = ", euler[2])



    def slave_robot_odom_callback_(self, msg):
        self.x2[0] = msg.pose.pose.position.x
        self.x2[1] = msg.pose.pose.position.y
        self.x2[2] = msg.twist.twist.linear.x
        self.x2[3] = msg.twist.twist.linear.y

        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)

        # 将四元数转换为欧拉角 (roll, pitch, yaw)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.slave_yaw = euler[2]
        # print("slave_yaw = ", euler[2])

    def spin_task_(self):
        """ Spin node in class. """
        rclpy.spin(self)

    def timer_work_(self):
        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus below------------------------------------------------
        # ----------------------------------------------------------------------------------------------------

        # Use controller there!!!!!!TODO
        output = self.Lpc.lpc_calculate(x1=self.x1, x2=self.x2)
        self.Lpc.calculate_distance(x1=self.x1, x2=self.x2)

        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus above------------------------------------------------
        # ----------------------------------------------------------------------------------------------------
        self.turn_output_into_cmd_vel_(output)

def main():
    rclpy.init()
    try:
        node = Lft_Onni_Robots("Lft_Onni_Robots")
        while 1:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()