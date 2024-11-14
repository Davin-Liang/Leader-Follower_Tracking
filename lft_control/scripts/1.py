import numpy as np
from scipy.linalg import expm

x = np.array([-8, 0, -0.005, 0])
Gd = np.array([[2., 0., 0., 0.],
               [0., 2., 0., 0.],
               [0., 0., 1., 0.],
               [0., 0., 0., 1.]])
P = np.array([[5.25, -0.0, 4.0, -0.0],
                [-0.0, 5.25, -0.0, 4.0],
                [4.0, 0.0, 5.0, 0.0],
                [0.0, 4.0, 0.0, 5.0]])
alpha = None
beta = None
Nmax = 20
    # Initialize parameters and tolerances
tol = 1e-6
if np.linalg.norm(x) > 1e-16:
    a = -1
    q = 0
    y = expm(-Gd * a).dot(x)

    # Iteratively adjust 'a' until condition is satisfied or limit is reached
    while (y.T @ P @ y < 1) and (a > -746):
        a *= 2
        y = expm(-Gd * a).dot(x)

    # Find a lower bound for 'b'
    if y.T @ P @ y > 1:
        b = 1
        y = expm(-Gd * b).dot(x)
        while (y.T @ P @ y > 1) and (b < 710):
            b *= 2
            y = expm(-Gd * b).dot(x)

        # Use binary search to refine 'c'
        if y.T @ P @ y < 1:
            c = (a + b) / 2
            y = expm(-Gd * c).dot(x)
            Qf = y.T @ P @ y - 1
            i = 0
            # 二分法寻找最优的 c，使得 y.T @ P @ y - 1 足够接近 0
            while (abs(Qf) > tol) and (i < Nmax):
                i += 1
                if Qf > 0:
                    a = c
                else:
                    b = c
                c = (a + b) / 2
                y = expm(-Gd * c).dot(x)
                Qf = y.T @ P @ y - 1
        else:
            c = b
    else:
        c = a
    q = np.exp(c)
else:
    q = 0

# Apply upper and lower bounds if provided
if beta is not None:
    q = min(beta, q)
if alpha is not None:
    q = max(alpha, q)

print("q=",q)
