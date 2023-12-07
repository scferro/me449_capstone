import numpy as np
import modern_robotics as mr
import csv
from milestone1 import NextState, write_to_csv
from milestone2 import TrajectoryGenerator, extract_variables
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
initial_config = [0, 0, 0,
                  0, 0, 0, 0, 0,
                  0, 0.2, -1.6, 0]
Kp = 0
Ki = 0


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


### MAIN CODE ###

target_trajectory = TrajectoryGenerator(Tse_d_init, Tsc_init, Tsc_final)

Tse, Tb0, T0e = FindTse(initial_config, Tb0, M0e, BList)
print(Tse)
config = initial_config

count = 0
max = len(target_trajectory)
Tse_error_int = 0
Tse_error_vector = []

while count < (max - 1):
    config_d = target_trajectory[count]
    config_d_next = target_trajectory[count+1]

    Tse_d = np.array([[config_d[0], config_d[1], config_d[2], config_d[3]], 
                       [config_d[4], config_d[5], config_d[6], config_d[7]], 
                       [config_d[8], config_d[9], config_d[10], config_d[11]],
                       [0, 0, 0, 1]])
    Tse_d_next = np.array([[config_d_next[0], config_d_next[1], config_d_next[2], config_d_next[3]], 
                            [config_d_next[4], config_d_next[5], config_d_next[6], config_d_next[7]], 
                            [config_d_next[8], config_d_next[9], config_d_next[10], config_d_next[11]],
                            [0, 0, 0, 1]])
    
    V_ee, Tse_error, Tse_error_int = FeedbackControl(Tse, Tse_d, Tse_d_next, Tse_error_int, Kp, Ki)

    speedCommands = 

    Tse_error_vector.append(Tse_error)

    config = NextState(config, speedCommands, config_d[-1])

    print(count)
    count += 1

