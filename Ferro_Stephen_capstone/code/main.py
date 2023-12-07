import numpy as np
import modern_robotics as mr
import csv
import time
from milestone1 import NextState, write_to_csv
from milestone2 import TrajectoryGenerator
from milestone3 import FeedbackControl, FindTse

### INPUTS ###

Tse_d_init = np.array([[0,0,1,0],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]
                    ])
Tsc_init = np.array([[1,0,0,1],
                    [0,1,0,0],
                    [0,0,1,0.025],
                    [0,0,0,1]
                    ])
Tsc_final = np.array([[0,1,0,0],
                    [-1,0,0,-1],
                    [0,0,1,0.025],
                    [0,0,0,1]
                    ])
initial_config = np.array([0.5, 1, -1,
                            0, 0, 0, 0, 0,
                            0, 0.2, -1.6, 0,
                            0])

Kp = 5
Ki = 0.1

filename_csv = 'Ferro_Stephen_capstone.csv'

### ROBOT CONFIGURATION VARIABLES ###

Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]
                ])
M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]
                ])
BList = np.array([[0, 0, 1, 0, 0.033, 0],
                [0, -1, 0, -0.5076, 0, 0],
                [0, -1, 0, -0.3526, 0, 0],
                [0, -1, 0, -0.2176, 0, 0],
                [0, 0, 1, 0, 0, 0]]).T

l = 0.235       # the distance from the center of the robot to the front/rear axle (see p.521 of Modern Robotics)
w = 0.15        # the distance from the center of the robot to the LH/RH wheel centerline (see p.521 of Modern Robotics)
r = 0.0475      # the wheel radii

base_geometry = (r / 4) * np.array([[0, 0, 0, 0],
                                    [0, 0, 0, 0],
                                    [-1.0/(l+w), 1.0/(l+w), 1.0/(l+w), -1.0/(l+w)],
                                    [    1.0    ,     1.0  ,     1.0  ,     1.0   ],
                                    [   -1.0    ,     1.0  ,    -1.0  ,     1.0   ],
                                    [0, 0, 0, 0]])


### MAIN CODE ###

target_trajectory = TrajectoryGenerator(Tse_d_init, Tsc_init, Tsc_final)

write_to_csv(target_trajectory, 'traj.csv')

Tse, Tb0, T0e = FindTse(initial_config, Tb0, M0e, BList)
config = initial_config

Tse_error_int = [0, 0, 0, 0, 0, 0]
Tse_error_vector = []
config_array = np.array([config])
speed_array = np.array([0,0,0,0,0,0,0,0,0])

count = 0
max = len(target_trajectory)

while count < (max - 1):
    config_d = target_trajectory[count]
    config_d_next = target_trajectory[count+1]

    Tse_d = np.array([[config_d[0], config_d[1], config_d[2], config_d[9]], 
                       [config_d[3], config_d[4], config_d[5], config_d[10]], 
                       [config_d[6], config_d[7], config_d[8], config_d[11]],
                       [0, 0, 0, 1]])
    Tse_d_next = np.array([[config_d_next[0], config_d_next[1], config_d_next[2], config_d_next[9]], 
                       [config_d_next[3], config_d_next[4], config_d_next[5], config_d_next[10]], 
                       [config_d_next[6], config_d_next[7], config_d_next[8], config_d_next[11]],
                       [0, 0, 0, 1]])
    
    V_ee, Tse_error, Tse_error_int = FeedbackControl(Tse, Tse_d, Tse_d_next, Tse_error_int, Kp, Ki)

    thetaList = config[3:8]
    J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ base_geometry
    J_arm = mr.JacobianBody(BList, thetaList)
    J = np.hstack((J_base, J_arm))

    J_inv = np.linalg.pinv(J)
    speedCommands = J_inv @ V_ee

    config = NextState(config, speedCommands, config_d[-1])

    config_array = np.vstack((config_array, config))
    speed_array = np.vstack((speed_array, speedCommands))

    Tse_error_vector.append(Tse_error)

    count += 1
    print(count)
    
write_to_csv(config_array, filename_csv)
write_to_csv(speed_array, 'speeds.csv')
write_to_csv(Tse_error_vector, 'error.csv')

