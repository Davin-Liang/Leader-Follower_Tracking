import numpy as np
from scipy.linalg import inv, eig, sqrtm, kron, solve,null_space

def trans_con(A, B):
    """
    The function computes an orthogonal matrix T for a controllable pair {A, B}.

    Parameters:
        A (np.ndarray): system matrix (n x n)
        B (np.ndarray): control matrix (n x m)

    Returns:
        T (np.ndarray): orthogonal transformation matrix
        nt (list): list containing sizes of the blocks
    """
    n = A.shape[0]
    nt = []
    T = None

    # Check if A is square and dimensions of A and B match
    if A.shape[0] != A.shape[1]:
        print('Error: The matrix A is not square.')
        return T, nt
    if A.shape[0] != B.shape[0]:
        print('Error: The number of rows of the matrix A does not coincide with the number of rows of the matrix B.')
        return T, nt

    # ---- Controllability check ----
    U = np.empty((n, 0)) # Initialize an empty controllability matrix U
    Ak = np.eye(n)  # Initialize Ak as the identity matrix of size n

    for i in range(n):
        U = np.hstack([U, np.dot(Ak, B)])  # Append Ak * B to U
        Ak = np.dot(Ak, A)  # Update Ak by multiplying with A

    if np.linalg.matrix_rank(U) < n:
        print('The pair {A,B} is not controllable.')
        return T, nt

    T = np.eye(n)
    Ak = A.copy()
    Bk = B.copy()
    l = 0

    while np.linalg.matrix_rank(Bk) < Ak.shape[0]:

        nt.insert(0, np.linalg.matrix_rank(Bk))  # Prepend the rank of Bk to nt
        B_ort = np.linalg.svd(Bk.T)[2][np.linalg.matrix_rank(Bk):] # Compute orthogonal complement of Bk
        B_p = np.linalg.svd(B_ort)[2][np.linalg.matrix_rank(B_ort):]  # Compute orthogonal complement of B_ort

        print("B_ort shape:", B_ort.shape)
        print("Ak shape:", Ak.shape)
        print("B_p shape:", B_p.shape)

        if Ak.shape[0] < n:
            T_update = np.vstack([
                np.hstack([np.vstack([B_ort, B_p]), np.zeros((Ak.shape[0], l))]),
                np.hstack([np.zeros((l, Ak.shape[0])), np.eye(l)])
            ])
            T = np.dot(T_update, T)
        else:
            T = np.vstack([B_ort, B_p])

        l += np.linalg.matrix_rank(Bk)
        Bk = np.dot(np.dot(B_ort, Ak), B_p.T)
        Ak = np.dot(np.dot(B_ort, Ak), B_ort.T)

    nt.insert(0, np.linalg.matrix_rank(Bk))  # Prepend the final rank of Bk to nt
    return T, nt

def block_con(A, B):
    """
    This function needs to be implemented to return the block decomposition
    matrices for A and B. The output should be a transformation matrix T
    and block sizes nt.
    """
    T = None
    nt = []

    # Orthogonal Transformation
    T, nt = trans_con(A, B)
    if T is None:
        return T, nt

    k = len(nt)
    ni = [1]  # Initialize the list of indices
    j = 1

    for i in range(k - 1):
        j += nt[i]
        ni.append(j)

    # Transform A using the orthogonal matrix T
    A = T @ A @ T.T  # Use the @ operator for matrix multiplication

    #----Triangular transformation
    n = A.shape[0]
    Phi = np.eye(n)

    for i in range(k - 1):
        # Extract submatrix temp_A from A using Python slicing (0-based index)
        temp_A = A[ni[i] - 1:ni[i] + nt[i] - 1, ni[i] + nt[i] - 1:ni[i] + nt[i] + nt[i + 1] - 1]

        # Calculate temp_block using matrix operations
        temp_block = np.hstack([
            np.dot(np.dot(np.linalg.inv(np.dot(temp_A, temp_A.T)), temp_A.T),
                   A[ni[i] - 1:ni[i] + nt[i] - 1, :ni[i] + nt[i] - 1]),
            np.eye(nt[i + 1]),
            np.zeros((nt[i + 1], n - ni[i] - nt[i] - nt[i + 1] + 1))
        ])

        # Create temp_T and update it
        temp_T = np.eye(n)
        temp_T[ni[i + 1] - 1:ni[i + 1] + nt[i + 1] - 1, :] = temp_block

        # Update Phi and A
        Phi = np.dot(temp_T, Phi)
        A = np.dot(np.dot(temp_T, A), np.linalg.inv(temp_T))

    # Update T
    T = np.dot(Phi, T)

    return T, nt


def lpc2hpc(A, B, K):
    """
    The function upgrades the linear proportional controller (LPC) to
    Homogeneous Proportional Controller (HPC).

    Parameters:
        A (np.ndarray): system matrix (n x n)
        B (np.ndarray): control matrix (n x m)
        K (np.ndarray): gain matrix (m x n) of LPC such that A+B*K is Hurwitz

    Returns:
        K0 (np.ndarray): feedback gain (m x n) of the linear component of HPC
        G0 (np.ndarray): matrix (n x n) that defines the generator Gd=eye(n)+mu*G0
        P (np.ndarray): positive definite matrix which defines the canonical homogeneous norm
        mu_min (float): minimal admissible degree of HPC
        mu_max (float): maximal admissible degree of HPC
    """

    K0, G0, P, mu_min, mu_max = 0, 0, 0, 0, 0
    tol = 1e-5  # tolerance for numerical stability

    # Check dimensions
    sA = A.shape
    sB = B.shape
    sK = K.shape

    if sA[0] != sA[1]:
        print('Error: the system matrix must be square')

    if sB[0] != sA[1]:
        print('Error: dimensions of system and control matrices must agree')
        
    if sK[0] != sB[1]:
        print('Error: dimensions of control and gain matrices must agree')

    n, m = sB[0], sB[1]

    # Controllability check
    U = B
    tA = A
    k = 0
    if np.linalg.matrix_rank(U) == n:
        k = 1
    i = 1
    while k == 0 and i <= n - 1:
        i += 1
        U = np.hstack([U, np.dot(tA, B)])
        if np.linalg.matrix_rank(U) == n:
            k = i
        tA = np.dot(A, tA)

    if k == 0:
        print('Error: The system is not controllable')
        
    if np.linalg.det(np.dot(U, U.T)) < tol:
        print('Warning: The system is weakly controllable. Parameters may be badly tuned due to computations issues.')

    # 计算闭环系统的最大实部特征值，rho 用来衡量系统的稳定性裕度
    rho = -max(np.real(eig(A + np.dot(B, K))[0])) * 0.001
    if rho < tol:
        print(
            'Error: The linear control system does not have a sufficient stability margin. The upgrade is impossible.')
    
    # If B is full rank, compute K0, K, Gd, and P
    if np.linalg.matrix_rank(B) == n:
        K0 = -np.dot(B.T, inv(np.dot(B, B.T))).dot(A)
        K = -np.dot(B.T, inv(np.dot(B, B.T)))
        Gd = np.eye(n)
        P = np.eye(n)
        Gd = P

    # Block decomposition
    T, nt = block_con(A, B)
    if T is None or np.all(T == 0):
        K0 = 0
        K = 0
        P = 0
        Gd = 0
        print('Error: A block decomposition is impossible.')
        return K0, G0, P, Gd

    Anew = np.dot(np.dot(T, A), inv(T))
    n, m = B.shape
    k = len(nt)  # Python equivalent to MATLAB's size(nt, 1)

    # Creation of an array of indices
    s = 0  # Start index, 0-based for Python
    n_ind = [s]
    for i in range(1, k):
        s = s + nt[i - 1]
        n_ind.append(s)

    Bnew = np.dot(T, B)
    B0 = Bnew[n_ind[k - 1]:n, :m]
    K0 = -np.dot(np.dot(B0.T, inv(np.dot(B0, B0.T))), Anew[n_ind[k-1]:n, :n])
    A0 = Anew + np.dot(Bnew, K0)
    K0 = np.dot(K0, T)

    # Create vG0 array
    #vG0 = []
    #vG0 = np.concatenate([(k - i) * np.ones(nt[i - 1]) for i in range(1, k + 1)])
    vG0 = np.hstack([(k - i) * np.ones(nt[i - 1], dtype=int) for i in range(1, k + 1)])

    # G0 calculation
    G0 = -np.dot(inv(T), np.dot(np.diag(vG0), T))

    # Compute P from Lyapunov equation
    I_n = np.eye(n)
    W0 = np.kron(I_n, (A + np.dot(B, K)).T) + np.kron((A + np.dot(B, K)).T, I_n)
    zet0 = -2 * I_n.flatten()

    # 求解线性方程 W0 * v_P = zet0
    v_P = np.linalg.solve(W0, zet0)

    # 重新排列为 n x n 的矩阵 P
    P = np.reshape(v_P[:n ** 2], (n, n))

    # Calculation of admissible mu
    # 计算 admissible mu
    matrix_expr = np.dot(sqrtm(P), np.dot(G0, inv(sqrtm(P)))) + np.dot(inv(sqrtm(P)), np.dot(G0.T, sqrtm(P)))

    # 计算特征值的最小和最大实部
    eigenvalues = np.linalg.eigvals(matrix_expr)
    lambda_min = np.min(np.real(eigenvalues))
    lambda_max = np.max(np.real(eigenvalues))

    if lambda_max > tol:
        mu_min = max(-1, -1 / lambda_max + tol)
    else:
        mu_min = -1

    if lambda_min < -tol:
        mu_max = min(1 / k, -1 / lambda_min)
    else:
        mu_max = 1 / k
    return K0, G0, P, mu_min, mu_max











