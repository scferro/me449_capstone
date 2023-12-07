import numpy as np
import modern_robotics as mr
import csv

def FeedbackControl(Tse, Tse_d, Tse_d_next, Tse_error_int, Kp, Ki, dt=0.01):

    Kp = Kp * np.identity(6)
    Ki = Ki * np.identity(6)

    Tse_error = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Tse) @ Tse_d))

    Vd =  mr.se3ToVec((1/dt) * mr.MatrixLog6((mr.TransInv(Tse_d) @ Tse_d_next)))

    V_ee = (mr.Adjoint(mr.TransInv(Tse) @ Tse_d) @ Vd) + (Kp @ Tse_error) + (Ki @ Tse_error_int)

    Tse_error_int_new = Tse_error * dt + Tse_error_int

    return V_ee, Tse_error_int_new


def FindTse(currentConfig, Tb0, M0e, BList):
    """
    Decription:
        Calculates Tse based on the current configuration of the robot
    Args:
        • currentConfig: The current configuration of the robot ([phi_b, x_b, y_b, armjoint1, armjoint2, armjoint3, armjoint4, armjoint5, wheel1, wheel2, wheel3, wheel4, gripper_state])
    Returns:
        • Tse: The position of the EE realtive to the space frame (Transformation matrix)
    """
    phi = currentConfig[0]
    x = currentConfig[1]
    y = currentConfig[2]
    thetaList = [currentConfig[3], currentConfig[4], currentConfig[5], currentConfig[6], currentConfig[7]]

    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]
                    ])

    T0e = mr.FKinBody(M0e, BList, thetaList)

    Ts0 = np.matmul(Tsb, Tb0)
    Tse = np.matmul(Ts0, T0e)

    return Tse, Tb0, T0e


# currentConfig = [0, 0, 0, 0, 0, 0.2, -1.6, 0]

# Tse_d = np.array([[0, 0, 1, 0.5],
#                   [0, 1, 0, 0],
#                   [-1, 0, 0, 0.5],
#                   [0, 0, 0, 1]
#                   ])
# Tse_d_next = np.array([[0, 0, 1, 0.6],
#                        [0, 1, 0, 0],
#                        [-1, 0, 0, 0.3],
#                        [0, 0, 0, 1]
#                        ])
# Tb0 = np.array([[1, 0, 0, 0.1662],
#                 [0, 1, 0, 0],
#                 [0, 0, 1, 0.0026],
#                 [0, 0, 0, 1]
#                 ])

# M0e = np.array([[1, 0, 0, 0.033],
#                 [0, 1, 0, 0],
#                 [0, 0, 1, 0.6546],
#                 [0, 0, 0, 1]
#                 ])
# Blist = np.array([[0, 0, 1, 0, 0.033, 0],
#                     [0, -1, 0, -0.5076, 0, 0],
#                     [0, -1, 0, -0.3526, 0, 0],
#                     [0, -1, 0, -0.2176, 0, 0],
#                     [0, 0, 1, 0, 0, 0]]).T
# Kp = 0
# Ki = 0
# Tse_error_int_init = 0


# Tse, Tb0, T0e = FindTse(currentConfig, Tb0, M0e, Blist)
# V_ee, Tse_error_int_new = FeedbackControl(Tse, Tse_d, Tse_d_next, Tse_error_int_init, Kp, Ki)
# print('V_ee')
# print(V_ee)
# print('Tse_error')
# print(Tse_error)
# print('Tse_error_int_new')
# print(Tse_error_int_new)


# # Jacobian of arm
# J_arm = mr.JacobianBody(Blist, currentConfig[3:-1])
# J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ F6
# Je = np.hstack((J_base, J_arm))