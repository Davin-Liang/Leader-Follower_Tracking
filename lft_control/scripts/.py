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
        if self.ee[1] != 0:
            b = max(-self.m * (self.e[3]) / self.e[1], 1)
        else:
            b = 1  # 或者其他默认值
        # b = max(-self.m * (self.e[3]) / self.e[1], 1)

        lambda_matrix = np.diag([a, b])
        k2 = -2 * lambda_matrix
        k1 = np.dot(lambda_matrix, (k2 + lambda_matrix)) / self.m

        self.k_lin = np.hstack((k1, k2))  # 线性反馈控制器

        # -------------------------齐次控制below--------------------------------
        K0, G0, P, nu_min, nu_max = lpc2hpc(self.A, self.B, self.k_lin)
        nu = nu_min
        # nu = 0
        Gd = np.eye(4) + nu * G0
        # -------------------------齐次控制above--------------------------------

    def lpc_calculate(self, x1, x2):
        # 控制律 u1
        # u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.sin(t), np.cos(t)])
        # u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.cos(t), np.sin(t)])
        # 更新 x1
        # x1 = x1 + h * (np.dot(A, x1) + np.dot(B, u1))

        # 误差计算
        self.e = x2 - x1 - self.d

        # -------------------------齐次控制below--------------------------------
        # 控制律 u2
        nx = hnorm(self.e, Gd, P)
        nx_clipped = np.clip(nx, 0.1, 1) # 限制 nx 的范围在 [0.1, 1]

        # 定义变量
        min_val = max(min(1, nx), 0.1)  # 计算 min(1, nx) 和 0.1 的最大值
        exponent = 1 + nu  # 计算指数部分
        k_term = min_val ** exponent  # 计算 k_term 的幂

        # 将 k_lin 和 k_term 分别处理为标量
        k_lin_k_term = k_lin * k_term

        # 计算矩阵指数
        Gd_term = Gd * (1 - np.log(min_val))  # 计算 Gd 的乘积
        exp_Gd = expm(Gd_term)  # 计算矩阵的指数
        # 计算最终结果
        u2 = k_lin_k_term @ (exp_Gd @ e)
        # -------------------------齐次控制above--------------------------------

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

        # -------------------------齐次控制below--------------------------------
        if self.mv + self.tol < np.linalg.norm(x2 - x1 - self.d):
            self.d = self.dl[:, self.mi]  # 选择矩阵 dl 的第 mi 列
            self.e = x2 - x1 - self.d  # 计算误差 e

            # 计算 a 和 b，确保最小值为 4
            a = max(-self.m * (self.e[2]) / self.e[0], 4)
            b = max(-self.m * (self.e[3]) / self.e[1], 4)

            # 创建对角矩阵 lambda
            lambda_matrix = np.diag([a, b])

            # 计算 k1 和 k2
            k2 = -2 * lambda_matrix
            k1 = lambda_matrix @ (k2 + lambda_matrix) / m

            # 将 k1 和 k2 拼接为一个矩阵
            k_lin = np.hstack([k1, k2])

            # 调用 lpc2hpc 函数，假设返回 K0, G0, P, nu_min, nu_max
            K0, G0, P, nu_min, nu_max = lpc2hpc(A, B, k_lin)

            # 选择 nu_min，并计算 Gd
            nu = nu_min
            Gd = np.eye(4) + nu * G0
        # -------------------------齐次控制above--------------------------------