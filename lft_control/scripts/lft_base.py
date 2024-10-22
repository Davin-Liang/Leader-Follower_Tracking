#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import numpy as np

import rclpy
from rclpy.node import Node
from threading import Thread

from lpc import Lpc_Controller

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

        self.Lpc = Lpc_Controller
        self.m = 2

        # Variables in Paper. TODO
        self.x1       = np.array([[0.0], [0.0], [0.0], [0.0]]) # state variable of leader robot
        self.x2       = np.array([[2.0], [0.0], [0.0], [0.0]]) # state variable of slave robot
        self.r1      = np.array([[0.0], [0.0]])
        self.r2       = np.array([[0.0], [0.0]])
        self.e         = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.output    = np.array([[0.0], [0.0]])

        for i in range(self.Lpc.m_p):
            self.Lpc.dist_l.append(np.linalg.norm(self.x2 - self.x1 - self.Lpc.dl[:, i]))

        self.Lpc.mv = min(self.Lpc.dist_l)
        self.Lpc.mi = self.Lpc.dist_l.index(self.Lpc.mv)
        self.Lpc.d = self.Lpc.dl[:, self.Lpc.mi]

        # 误差计算与控制律的设计
        self.e = self.x2 - self.x1 - self.Lpc.d

        a = max(-m * (self.e[2]) / self.e[0], 1)
        b = max(-m * (self.e[3]) / self.e[1], 1)

        lambda_matrix = np.diag([a, b])
        k2 = -2 * lambda_matrix
        k1 = np.dot(lambda_matrix, (k2 + lambda_matrix)) / self.m

        self.k_lin = np.hstack((k1, k2))  # 线性反馈控制器

        # ----------------------------------------------------------------------------------------------------
        # -----------------------------------------Focus above------------------------------------------------
        # ----------------------------------------------------------------------------------------------------

        self.work_timer = self.create_timer(0.002, self.timer_work_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()

    def use_controller(self):
        u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), self.x1) + np.array([np.sin(self.Lpc.t), np.cos(self.Lpc.t)])
        # u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.cos(t), np.sin(t)])
        # 更新 x1
        # x1 = x1 + h * (np.dot(A, x1) + np.dot(B, u1))

        # 误差计算
        self.e = self.x2 - self.x1 - self.Lpc.d
        # 控制律 u2
        self.u2 = np.dot(self.k_lin, self.e)
        # 更新 x2
        # self.x2 = x2 + h * (np.dot(A, x2) + np.dot(B, u2))

        # 计算 x2 和 x1 到安全点的距离
        for i in range(self.Lpc.m_p):
            self.Lpc.dist_l.append(np.linalg.norm(self.x2 - self.x1 - self.Lpc.dl[:, i]))

        self.Lpc.mv = min(self.Lpc.dist_l)
        self.Lpc.mi = self.Lpc.dist_l.index(self.Lpc.mv)
        self.Lpc.d = self.Lpc.dl[:, self.Lpc.mi]

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