import numpy as np
import matplotlib.pyplot as plt

class Lpc_Controller():
    def __init__(self, m_p=4, radius=1, tol=0.1, m=10.2):
        self.m = m
        self.A = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
        self.B = np.block([[np.zeros((2, 2))], [np.eye(2) * (1 / self.m)]])
        self.m_p    = 4 # 安全点的个数
        self.radius = 5 # 安全区的半径
        self.tol    = 0.1
        self.dl     = [] # 4 个安全的坐标

    def controller_initial_(self, x1, x2):
        center = x1[:2]
        self.dl = []
        for i in range(self.m_p):
            d = -1 * self.radius * np.array([np.cos(2 * np.pi * i / self.m_p), np.sin(2 * np.pi * i / self.m_p)]) + center
            self.dl.append(np.concatenate([d, [0, 0]]))
        self.dl = np.array(self.dl).T

        self.dist_l = []
        for i in range(self.m_p):
            self.dist_l.append(np.linalg.norm(x2 - x1 - self.dl[:, i]))

        self.mv = min(self.dist_l)
        self.mi = self.dist_l.index(self.mv)
        self.d = self.dl[:, self.mi]

        # 误差计算与控制律的设计
        self.e = x2 - x1 - self.d

        a = max(-self.m * (self.e[2]) / self.e[0], 1)
        if e[1] != 0:
            b = max(-m * (e[3]) / e[1], 1)
        else:
            b = 1
        # b = max(-self.m * (self.e[3]) / self.e[1], 1)

        lambda_matrix = np.diag([a, b])
        k2 = -2 * lambda_matrix
        k1 = np.dot(lambda_matrix, (k2 + lambda_matrix)) / self.m

        self.k_lin = np.hstack((k1, k2))  # 线性反馈控制器

    def lpc_calculate(self, x1, x2):
        # 控制律 u1
        # u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.sin(t), np.cos(t)])
        # u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.cos(t), np.sin(t)])
        # 更新 x1
        # x1 = x1 + h * (np.dot(A, x1) + np.dot(B, u1))

        # 误差计算
        self.e = x2 - x1 - self.d
        # 控制律 u2
        u2 = np.dot(self.k_lin, self.e)
        # 更新 x2
        goal_x2 = x2 + 0.02 * (np.dot(self.A, x2) + np.dot(self.B, u2))
        # print(goal_x2)
        result = []
        result.append(goal_x2[2])
        result.append(goal_x2[3])
        return result


    def calculate_distance(self, x1, x2):
        # 计算 x2 和 x1 到安全点的距离
        center = x1[:2]
        self.dl = []
        for i in range(self.m_p):
            d = -1 * self.radius * np.array([np.cos(2 * np.pi * i / self.m_p), np.sin(2 * np.pi * i / self.m_p)]) + center
            self.dl.append(np.concatenate([d, [0, 0]]))
        self.dl = np.array(self.dl).T

        self.dist_l = []
        for i in range(self.m_p):
            self.dist_l.append(np.linalg.norm(x2 - x1 - self.dl[:, i]))

        self.mv = min(self.dist_l)
        self.mi = self.dist_l.index(self.mv)
        self.d = self.dl[:, self.mi]