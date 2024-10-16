#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import numpy as np

import rclpy
from rclpy.node import Node
from threading import Thread

class Lft_Onni_Robots(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("I am %s!" % name)
        self.leader_robot_odom_sub_ = self.create_subscription(Odometry, "/leader/odom", self.leader_robot_odom_callback_, 10)
        self.slave_robot_odom_sub_ = self.create_subscription(Odometry, "/slave/odom", self.slave_robot_odom_callback_, 10)
        self.slave_robot_cmd_vel_pub_ = self.create_publisher(Twist, "/slave/cmd_vel", 5)

        self.slave_move_cmd_ = Twist()

        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus below------------------------------------------------
        # ----------------------------------------------------------------------------------------------------
        # Variables in Paper. TODO
        self.x_1_       = np.array([[0.0], [0.0], [0.0], [0.0]]) # state variable of leader robot
        self.x_2_       = np.array([[0.0], [0.0], [0.0], [0.0]]) # state variable of slave robot
        self.r_1_       = np.array([[0.0], [0.0]])
        self.r_2_       = np.array([[0.0], [0.0]])
        self.e_         = np.array([[0.0], [0.0]])
        self.output_    = np.array([[0.0], [0.0]])
        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus above------------------------------------------------
        # ----------------------------------------------------------------------------------------------------

        self.work_timer = self.create_timer(0.002, self.timer_work_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()

    def turn_output_into_cmd_vel_(self, output):
        # self.slave_move_cmd_.linear.x = output[0, 0]
        # self.slave_move_cmd_.linear.y = output[1, 0]
        self.slave_move_cmd_.linear.x = self.x_1_[2, 0]
        self.slave_move_cmd_.linear.y = self.x_1_[3, 0]
        self.slave_robot_cmd_vel_pub_.publish(self.slave_move_cmd_)

    def leader_robot_odom_callback_(self, msg):
        self.r_1_[0, 0] = msg.pose.pose.position.x
        self.r_1_[1, 0] = msg.pose.pose.position.y

        self.x_1_[0, 0] = msg.pose.pose.position.x
        self.x_1_[1, 0] = msg.pose.pose.position.y
        self.x_1_[2, 0] = msg.twist.twist.linear.x
        self.x_1_[3, 0] = msg.twist.twist.linear.y



    def slave_robot_odom_callback_(self, msg):
        self.r_2_[0, 0] = msg.pose.pose.position.x
        self.r_2_[1, 0] = msg.pose.pose.position.y

        self.x_2_[0, 0] = msg.pose.pose.position.x
        self.x_2_[1, 0] = msg.pose.pose.position.y
        self.x_2_[2, 0] = msg.twist.twist.linear.x
        self.x_2_[3, 0] = msg.twist.twist.linear.y

    def spin_task_(self):
        """ Spin node in class. """
        rclpy.spin(self)

    def timer_work_(self):
        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus below------------------------------------------------
        # ----------------------------------------------------------------------------------------------------

        # Use controller there!!!!!!TODO

        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus above------------------------------------------------
        # ----------------------------------------------------------------------------------------------------
        self.turn_output_into_cmd_vel_(self.output_)

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