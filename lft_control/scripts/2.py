import numpy as np
import matplotlib.pyplot as plt
from lpc2hpc import lpc2hpc
from hnorm import hnorm
from scipy.linalg import expm
import time

class Lpc_Controller():
    def __init__(self, m_p=4, radius=1, tol=0.1, m=2):
        self.m = m
        self.A = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
        self.B = np.block([[np.zeros((2, 2))], [np.eye(2) * (1 / self.m)]])
        self.m_p    = 4 # 安全点的个数
        self.radius = 1 # 安全区的半径
        self.tol    = 0.1
        self.dl     = [] # 4 个安全点的坐标
        self.safe_distance = 4
        self.max_distance = 10.0 

    def controller_initial_(self, x1, x2):
        # center = x1[:2]
        self.dl = []
        for i in range(self.m_p):
            d = -1 * self.radius * np.array([np.cos(2 * np.pi * i / self.m_p), np.sin(2 * np.pi * i / self.m_p)])
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
        # self.e = x2 - x1
        print("in_e:",self.e)
        self.distance = np.linalg.norm(self.e)
    
        a = max(-self.m * (self.e[2]) / self.e[0], 1)
        if self.e[1] != 0:
            b = max(-self.m * (self.e[3]) / self.e[1], 1)
        else:
            b = 1  # 或者其他默认值
        # b = max(-self.m * (self.e[3]) / self.e[1], 1)

        lambda_matrix = np.diag([a, b])
        k2 = -2 * lambda_matrix
        k1 = np.dot(lambda_matrix, (k2 + lambda_matrix)) / self.m

        self.k_lin = np.hstack((k1, k2))  # 线性反馈控制器
        print("in_klin:",self.k_lin)

        # -------------------------齐次控制below--------------------------------
        K0, G0, self.P, nu_min, nu_max = lpc2hpc(self.A, self.B, self.k_lin)
        self.nu = nu_min
        # nu = 0
        self.Gd = np.eye(4) + self.nu * G0
        print("in_nu,Gd:",self.nu, self.Gd)
        print("K0, G0, self.P, nu_min, nu_max, self.Gd, self.nu:", K0, G0, self.P, nu_min, nu_max, self.Gd, self.nu)
        # -------------------------齐次控制above-------------------------------
   
   
    def lpc_calculate(self, x1, x2): # TODO:
        h = 0.01
        
        # 控制律 u1
        # u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.sin(t), np.cos(t)])
        u1 = np.array([1.0, 0.0])
        # # 更新 x1
        x1 = x1 + h * (np.dot(self.A, x1) + np.dot(self.B, u1))
        # print("x1:",x1)

        # 误差计算
        self.e = x2 - x1 - self.d
        # self.e = x2 - x1
        print("e:",self.e)

        # -------------------------齐次控制below--------------------------------
        # 控制律 u2
        nx = hnorm(self.e, self.Gd, self.P)
        print("nx:",nx)
        # 定义变量
        min_val = max(min(1, nx), 0.1)  # 计算 min(1, nx) 和 0.1 的最大值
        exponent = 1 + self.nu  # 计算指数部分
        k_term = min_val ** exponent  # 计算 k_term 的幂

        # 将 k_lin 和 k_term 分别处理为标量
        k_lin_k_term = self.k_lin * k_term

        # 计算矩阵指数
        Gd_term = self.Gd * (1 - np.log(min_val))  # 计算 Gd 的乘积
        exp_Gd = expm(Gd_term)  # 计算矩阵的指数

        # # 根据距离生成减速因子
        # if self.distance > self.safe_distance:
        #     # 根据距离线性缩放控制信号
        #     # 当距离较大时，减速因子接近1，当距离接近safe_distance时，减速因子接近0
        #     slowdown_factor = (self.distance - self.safe_distance) / (self.max_distance - self.safe_distance)
        #     slowdown_factor = np.clip(slowdown_factor, 0.0, 1.0)  # 将减速因子限制在 [0, 1] 范围内
        # else:
        #     slowdown_factor = 0.1

        # 计算控制信号，并乘以减速因子
        # u2 = k_lin_k_term @ (exp_Gd @ self.e) * slowdown_factor
        u2 = k_lin_k_term @ (exp_Gd @ self.e)
        print("u2:",u2)

        # nx_clipped = np.clip(nx, 0.1, 1)
        # 计算 u2
        # u2 = (nx_clipped ** (1 + nu)) * k_lin * expm(Gd * (1 - np.log(nx_clipped))) @ e
        # -------------------------齐次控制above--------------------------------

        # 更新 x2
        goal_x2 = x2 + h * (np.dot(self.A, x2) + np.dot(self.B, u2))
        print("goal_x2:",goal_x2)

        result = []
        result.append(goal_x2[2])
        result.append(goal_x2[3])
        return result

    def calculate_distance(self, x1, x2):
        # 计算 x2 和 x1 到安全点的距离
        self.dist_l = []
        for i in range(self.m_p):
            self.dist_l.append(np.linalg.norm(x2 - x1 - self.dl[:, i]))

        self.mv = min(self.dist_l)
        self.mi = self.dist_l.index(self.mv)


def main():
    controller = Lpc_Controller()

    # Initial positions (example values)
    x1 = np.array([10.0, 1.0, 0.0, 0.0])  # Leader position (x1)
    x2 = np.array([1.0, 1.0, 0.0, 0.0])  # Follower position (x2)

    start=time.time()

    # Initialize controller
    controller.controller_initial_(x1, x2)
    

    # Perform one step of calculation
    goal_x2 = controller.lpc_calculate(x1, x2)
    print("Updated goal_x2:", goal_x2)

    # Calculate distance
    controller.calculate_distance(x1, x2)
    end=time.time()
    print('程序运行时间为: %s Seconds'%(end-start))


if __name__ == "__main__":
    main()