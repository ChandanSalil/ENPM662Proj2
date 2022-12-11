import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

a, d, alpha, theta, theta1, theta2, theta3, theta4, theta5, theta6, t = sp.symbols(
    'a d alpha theta theta1 theta2 theta3 theta4 theta5 theta6 t')

M = sp.Matrix([[sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
               [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
               [0, sp.sin(alpha), sp.cos(alpha), d],
               [0, 0, 0, 1]])

M01 = M.subs([(d, 0.089159), (a, 0), (alpha, sp.pi / 2), (theta, theta1)])
M12 = M.subs([(d, 0), (a, -0.425), (alpha, 0), (theta, theta2)])
M23 = M.subs([(d, 0), (a, -0.39225), (alpha, 0), (theta, theta3)])
M34 = M.subs([(d, 0.10915), (a, 0), (alpha, sp.pi / 2), (theta, theta4)])
M45 = M.subs([(d, 0.09465), (a, 0), (alpha, -sp.pi / 2), (theta, theta5)])
M56 = M.subs([(d, 0.0823), (a, 0), (alpha, 0), (theta, theta6)])

M02 = M01 * M12
M03 = M01 * M12 * M23
M04 = M01 * M12 * M23 * M34
M05 = M01 * M12 * M23 * M34 * M45
M06 = M01 * M12 * M23 * M34 * M45 * M56

J0 = sp.Matrix([[sp.diff(M06[0, 3], theta1), sp.diff(M06[0, 3], theta2), sp.diff(M06[0, 3], theta3),
                 sp.diff(M06[0, 3], theta4), sp.diff(M06[0, 3], theta5), sp.diff(M06[0, 3], theta6)],
                [sp.diff(M06[1, 3], theta1), sp.diff(M06[1, 3], theta2), sp.diff(M06[1, 3], theta3),
                 sp.diff(M06[1, 3], theta4), sp.diff(M06[1, 3], theta5), sp.diff(M06[1, 3], theta6)],
                [sp.diff(M06[2, 3], theta1), sp.diff(M06[2, 3], theta2), sp.diff(M06[2, 3], theta3),
                 sp.diff(M06[2, 3], theta4), sp.diff(M06[2, 3], theta5), sp.diff(M06[2, 3], theta6)],
                [M01[0, 2], M02[0, 2], M04[0, 2], M05[0, 2], M06[0, 2], M06[0, 2]],
                [M01[1, 2], M02[1, 2], M04[1, 2], M05[1, 2], M06[1, 2], M06[1, 2]],
                [M01[2, 2], M02[2, 2], M04[2, 2], M05[2, 2], M06[2, 2], M06[2, 2]]
                ])

omega = (2 * 3.14) / 5
dt = 0.125
time = np.arange(0, 5, dt)

x_dot = sp.Matrix([[0], [omega * 0.1 * sp.cos(omega * t)], [-omega * 0.1 * sp.sin(omega * t)], [0], [0], [0]])
# initializing with joint angles
q = sp.Matrix([[sp.pi / 2], [-sp.pi / 2], [sp.pi / 2], [-sp.pi], [-sp.pi / 2], [0]])

# setting up the 3d plot

fig = plt.figure(figsize=(10, 7))
ax = plt.axes(projection="3d")
plt.title("End effector trajectory")
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(1, 0)
ax.set_ylim(-0.4, 0.4)
ax.set_zlim(0.4, 1)
ax.set_xlabel('X_AXIS')
ax.set_ylabel('Y_AXIS')
ax.set_zlabel('Z_AXIS')

for i in range(0, 40):
    M06_x_y_z = M06.subs([(theta1, q[0]), (theta2, q[1]), (theta3, q[2]), (theta4, q[3]), (theta5, q[4]),
                          (theta6, q[5])])  # substituting the joint angles in final transformation matrix
    ax.scatter3D(M06_x_y_z[0, -1], M06_x_y_z[1, -1], M06_x_y_z[2, -1], color="blue")
    J0_inv = J0.subs([(theta1, q[0]), (theta2, q[1]), (theta3, q[2]), (theta4, q[3]), (theta5, q[4]), (theta6, q[5])])
    J0_inv = (J0_inv.evalf()).inv()

    X_ = x_dot.subs([(t, time[i])]).evalf()
    q_dot = (J0_inv * X_).evalf()
    q = q + q_dot * dt
    plt.pause(0.1)
plt.show()
