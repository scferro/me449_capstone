import numpy as np
import modern_robotics as mr
import csv

def FeedbackControl(Tse, Tse_d, Tse_d_next, Tse_error_int, Kp_in=0, Ki_in=0, dt=0.01):
    """
    Decription:
        Feedback controller for robot motion based on planned trajectory
    Args:
        • Tse: The current EE configuration in space
        • Tse_d: The desired EE configuration in space
        • Tse_d_next: The next desired EE configuration in space (for feed forward control)
        • Tse_error_int: The intergal of the joint error
        • Kp: Proportional gain for feebback controller
        • Ki: Integral gain for feebback controller
        • dt: The time step
    Returns:
        • V_ee: Twist in the EE frame
        • Tse_error: Current EE pose error
        • Tse_error_int_new: New integral of Tse_error 
    """
    # Convert gains to matrices
    Kp = Kp_in * np.identity(6)
    Ki = Ki_in * np.identity(6)

    # Calculate Tse error based on current and desired EE pose
    Tse_error = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Tse) @ Tse_d))

    # Calculate EE twist
    Vd =  (1/dt) * mr.se3ToVec(mr.MatrixLog6((mr.TransInv(Tse_d) @ Tse_d_next)))
    V_ee = (mr.Adjoint(mr.TransInv(Tse) @ Tse_d) @ Vd) + (Kp @ Tse_error) + (Ki @ Tse_error_int)

    # Calculate new error integral
    Tse_error_int_new = Tse_error * dt + Tse_error_int

    return V_ee, Tse_error, Tse_error_int_new


def FindTse(currentConfig, Tb0, M0e, BList):
    """
    Decription:
        Calculates Tse based on the current configuration of the robot
    Args:
        • currentConfig: The current configuration of the robot ([phi_b, x_b, y_b, armjoint1, armjoint2, armjoint3, armjoint4, armjoint5, wheel1, wheel2, wheel3, wheel4, gripper_state])
        • Tb0: The transform from the base of the robot to the base link of the arm
        • M0e: The home end effector frame
        • BList: The robot screw axis at the home position
    Returns:
        • Tse: The position of the EE realtive to the space frame (Transformation matrix)
    """
    # Extract variables from current configuration
    phi = currentConfig[0]
    x = currentConfig[1]
    y = currentConfig[2]
    thetaList = currentConfig[3:8]

    # Calculate Tsb based on current robot x, y, and phi
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]
                    ])

    # Find the transform from the base link to the EE using FK
    T0e = mr.FKinBody(M0e, BList, thetaList)

    # Calculate the EE position in the space frame
    Tse = Tsb @ Tb0 @ T0e

    return Tse, T0e


### UNCOMMENT CODE BELOW FOR TESTING ###

# currentConfig = [0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0]

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
# BList = np.array([[0, 0, 1, 0, 0.033, 0],
#                     [0, -1, 0, -0.5076, 0, 0],
#                     [0, -1, 0, -0.3526, 0, 0],
#                     [0, -1, 0, -0.2176, 0, 0],
#                     [0, 0, 1, 0, 0, 0]]).T
# Kp = 1
# Ki = 0
# Tse_error_int_init = [0, 0, 0, 0, 0, 0]

# l = 0.235       # the distance from the center of the robot to the front/rear axle (see p.521 of Modern Robotics)
# w = 0.15        # the distance from the center of the robot to the LH/RH wheel centerline (see p.521 of Modern Robotics)
# r = 0.0475      # the wheel radii

# base_geometry = (r / 4) * np.array([[0, 0, 0, 0],
#                                     [0, 0, 0, 0],
#                                     [-1.0/(l+w), 1.0/(l+w), 1.0/(l+w), -1.0/(l+w)],
#                                     [    1.0    ,     1.0  ,     1.0  ,     1.0   ],
#                                     [   -1.0    ,     1.0  ,    -1.0  ,     1.0   ],
#                                     [0, 0, 0, 0]])

# Tse, T0e = FindTse(currentConfig, Tb0, M0e, BList)
# V_ee, Tse_error, Tse_error_int_new = FeedbackControl(Tse, Tse_d, Tse_d_next, Tse_error_int_init, Kp, Ki)

# thetaList = currentConfig[3:8]
# J_arm = mr.JacobianBody(BList, thetaList)
# J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ base_geometry
# J = np.hstack((J_base, J_arm))
# J_inv = np.linalg.pinv(J)
# speedCommands = J_inv @ V_ee

# print('speedCommands')
# print(speedCommands)