import numpy as np
from scipy.linalg import solve_continuous_are

# 定义小车倒立摆物理性质
R = 0.151 / 2               # 车轮的半径
D = 0.42                    # 左轮、右轮两个轮子间的距离
l = 0.125                   # 摆杆质心到转轴距离  (修改)
m = 0.4842                  # 车轮的质量
M = 10.6316                 # 摆杆质量
I = 0.5 * m * R**2          # 车轮的转动惯量 
Jz = (1/3) * M * l**2       # 机器人机体对 z 轴的运动时产生的转动惯量(俯仰方向)
Jy = (1/12) * M * D**2      # 机器人机体对 y 轴的运动时产生的转动惯量(偏航方向)
g = 9.8

Q_eq = Jz * M + (Jz + M * l**2) * (2 * m + (2 * I) / R**2)

# 系统矩阵 A
A_23 = -(M**2 * l**2 * g) / Q_eq
A_43 = M * l * g * (M + 2 * m + (2 * I) / R**2) / Q_eq

A = np.array([
    [0, 1, 0, 0],
    [0, 0, A_23, 0],
    [0, 0, 0, 1],
    [0, 0, A_43, 0]
])

# 输入矩阵 B
B_21 = (Jz + M * l**2 + M * l * R) / (Q_eq * R)
B_41 = -((M * l / R) + M + 2 * m + (2 * I) / R**2) / Q_eq

B = np.array([
    [0],
    [2 * B_21],
    [0],
    [2 * B_41]
])

# 权重矩阵 Q 和 R
Q = np.diag([100, 1, 1000, 1])
R = np.array([[5]])

# 求解连续代数黎卡提方程 CARE
P = solve_continuous_are(A, B, Q, R)

# 计算反馈增益矩阵 K
K = np.linalg.inv(R) @ B.T @ P

print("LQR反馈增益矩阵 K =")
print(K)
