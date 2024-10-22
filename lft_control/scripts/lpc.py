import numpy as np
import matplotlib.pyplot as plt

# 参数设置
Tmax = 60  # 最大仿真时间
h = 0.01  # 采样时间
m = 2  # 质量
t = 0

class Lpc_Controller():
    def __init__(self, name):
        self.A = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
        self.B = np.block([[np.zeros((2, 2))], [np.eye(2) * (1 / m)]])
        self.m_p = 4
        self.radius = 1
        self.tol = 0.1
        self.dl = []

        self.mv = None
        self.mi = None
        self.d = None
        self.dist_l = []
        self.t = 0

    def safe_point_initial(self):
        pass

# -------------------------------------------------------------------------------------

# 时间步长与数据存储
tl = []
xl1 = []
ul1 = []
xl2 = []
ul2 = []
el = []
lambdal = []
ul = []
u1l = []

# 主循环
while t < Tmax:
    # 控制律 u1
    u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.sin(t), np.cos(t)])
    # u1 = -np.dot(np.hstack([np.eye(2), np.eye(2)]), x1) + np.array([np.cos(t), np.sin(t)])
    # 更新 x1
    x1 = x1 + h * (np.dot(A, x1) + np.dot(B, u1))

    # 误差计算
    e = x2 - x1 - d
    # 控制律 u2
    u2 = np.dot(k_lin, e)
    # 更新 x2
    x2 = x2 + h * (np.dot(A, x2) + np.dot(B, u2))

    # 计算 x2 和 x1 到安全点的距离
    dist_l = []
    for i in range(m_p):
        dist_l.append(np.linalg.norm(x2 - x1 - dl[:, i]))

    # 找到最小距离的安全点
    mv = min(dist_l)
    mi = dist_l.index(mv)

    # 更新时间
    t += h
    tl.append(t)  # 保存时间
    xl1.append(x1)  # 保存 x1
    ul1.append(u1)  # 保存 u1
    xl2.append(x2)  # 保存 x2
    ul2.append(u2)  # 保存 u2
    el.append(e)  # 保存误差 e
    u1l.append(u1)  # 保存控制量 u1

    lambdal.append(lambda_matrix)  # 保存 lambda 矩阵

# E = []
# for i in range(3000):
#     E.append(np.sqrt(el[i][0]**2 + el[i][1]**2))



# # 数据转换为numpy数组
# tl = np.array(tl)
# xl1 = np.array(xl1).T
# xl2 = np.array(xl2).T
# ul2 = np.array(ul2).T
# el = np.array(el).T

# # 绘制图像
# plt.figure(1)
# plt.plot(tl, xl1[0], 'r', label='$x_1$', linewidth=2)
# plt.plot(tl, xl2[0], 'b', label='$x_2$', linewidth=2)
# plt.xlim([0, 30])
# plt.xlabel('$t(s)$', fontsize=20)
# plt.ylabel('$x$', fontsize=20)
# plt.legend(fontsize=20)
# plt.grid()

# plt.figure(2)
# plt.plot(tl, xl1[1], 'r', label='$y_1$', linewidth=2)
# plt.plot(tl, xl2[1], 'b', label='$y_2$', linewidth=2)
# plt.xlim([0, 30])
# plt.ylim([-0.8, 1.2])
# plt.xlabel('$t(s)$', fontsize=20)
# plt.ylabel('$y$', fontsize=20)
# plt.legend(fontsize=20)
# plt.grid()

# plt.figure(3)
# plt.plot(xl1[0], xl1[1], 'r', label='$r_1$', linewidth=2)
# plt.plot(xl2[0], xl2[1], 'b', label='$r_2$', linewidth=2)
# plt.ylim([-0.8, 1.2])
# plt.xlabel('$x$', fontsize=20)
# plt.ylabel('$y$', fontsize=20)
# plt.legend(fontsize=20)
# plt.grid()

# plt.figure(4)
# plt.plot(tl, ul2[0], 'r', label='$u_x$', linewidth=2)
# plt.plot(tl, ul2[1], 'b', label='$u_y$', linewidth=2)
# plt.ylim([-12, 6])
# plt.xlabel('$t$', fontsize=20)
# plt.ylabel('$u$', fontsize=20)
# plt.legend(fontsize=20)
# plt.grid()

# plt.figure(5)
# plt.plot(tl, el[0], 'r', label='$e_x$', linewidth=2)
# plt.plot(tl, el[1], 'b', label='$e_y$', linewidth=2)
# plt.xlabel('$t$', fontsize=20)
# plt.ylabel('$\ell$', fontsize=20)
# plt.legend(fontsize=20)
# plt.grid()

# # 计算误差幅值
# E = np.sqrt(el[0] ** 2 + el[1] ** 2)

# plt.figure(6)
# plt.plot(tl, E, 'r', label='$e_{lin}$', linewidth=2)
# plt.ylim([0, 3.5])
# plt.xlabel('$t(s)$', fontsize=20)
# plt.ylabel('$e$', fontsize=20)
# plt.legend(fontsize=20)
# plt.grid()

# plt.show()
