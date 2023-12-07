import numpy as np
import matplotlib.pyplot as plt
import modern_robotics as mr
import csv
import time
from milestone1 import NextState, write_to_csv
from milestone2 import TrajectoryGenerator
from milestone3 import FeedbackControl, FindTse

### INPUTS ###

Tse_d_init = np.array([[0, 0, 1, 0],
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0.5],
                    [0, 0, 0, 1]
                    ])
Tsc_init = np.array([[1, 0, 0, 1],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]
                    ])
Tsc_final = np.array([[0, 1, 0, 0],
                    [-1, 0, 0, -1],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]
                    ])
initial_config = np.array([0, 1, 0,
                            0, 0, 0, 0, 0,
                            0, 0, 0, 0,
                            0])

Kp = 10
Ki = 0.1

filename_csv = 'Ferro_Stephen_capstone.csv'
filename_dat = 'Ferro_Stephen_capstone.dat'

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

# Generate the goal trajectory
target_trajectory = TrajectoryGenerator(Tse_d_init, Tsc_init, Tsc_final)
write_to_csv(target_trajectory, 'traj.csv')

# Set the current configuration equal to the intial_config
config = initial_config

# Initialize error data integral and array
Tse_error_int = [0, 0, 0, 0, 0, 0]
Tse_error_array = np.array([0, 0, 0, 0, 0, 0])
config_array = np.array([config])

count = 0
max = len(target_trajectory)

# Interate through trajectory commands
while count < (max - 1):
    # Determien the current and next desired poses
    Tse_vec_d = target_trajectory[count]
    Tse_vec_d_next = target_trajectory[count+1]

    # Convert configurations to transforms
    Tse, T0e = FindTse(config, Tb0, M0e, BList)
    Tse_d = np.array([[Tse_vec_d[0], Tse_vec_d[1], Tse_vec_d[2], Tse_vec_d[9]],
                      [Tse_vec_d[3], Tse_vec_d[4], Tse_vec_d[5], Tse_vec_d[10]],
                      [Tse_vec_d[6], Tse_vec_d[7], Tse_vec_d[8], Tse_vec_d[11]],
                      [0, 0, 0, 1]
                      ])
    Tse_d_next = np.array([[Tse_vec_d_next[0], Tse_vec_d_next[1], Tse_vec_d_next[2], Tse_vec_d_next[9]],
                            [Tse_vec_d_next[3], Tse_vec_d_next[4], Tse_vec_d_next[5], Tse_vec_d_next[10]],
                            [Tse_vec_d_next[6], Tse_vec_d_next[7], Tse_vec_d_next[8], Tse_vec_d_next[11]],
                            [0, 0, 0, 1]
                            ])
            
    # Calculate EE twist using FeedbackControl
    V_ee, Tse_error, Tse_error_int_new = FeedbackControl(Tse, Tse_d, Tse_d_next, Tse_error_int, Kp, Ki)
    Tse_error_int = Tse_error_int_new

    # Find the Jacobian of the robot
    thetaList = config[3:8]
    J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ base_geometry
    J_arm = mr.JacobianBody(BList, thetaList)
    J = np.hstack((J_base, J_arm))

    # Calculate speed commands based on Jacobian and V_ee
    speedCommands = np.linalg.pinv(J) @ V_ee
    wheelSpeeds = speedCommands[0:4]
    jointSpeeds = speedCommands[4:]

    # Find the new robot configuration using NewState
    new_config = NextState(config, wheelSpeeds, jointSpeeds, Tse_vec_d[-1])
    config = new_config

    config_array = np.vstack((config_array, config))
    Tse_error_array = np.vstack((Tse_error_array, Tse_error))

    count += 1
    
write_to_csv(config_array, filename_csv)
print('Writing error file to ' + filename_dat)
Tse_error_array.tofile(filename_dat, sep=" ", format="%s")


# Generate x axis data
x = np.linspace(0, (max)*0.01, (max))

# Define functions for y values
y1 = []
y2 = []
y3 = []
y4 = []
y5 = []
y6 = []

for vector in Tse_error_array:
    y1.append(vector[0])
    y2.append(vector[1])
    y3.append(vector[2])
    y4.append(vector[3])
    y5.append(vector[4])
    y6.append(vector[5])

# Plot the six lines on the chart
plt.plot(x, y1, label='Pitch Error (rad)')
plt.plot(x, y2, label='Roll Error (rad)')
plt.plot(x, y3, label='Yaw Error (rad)')
plt.plot(x, y4, label='x Error (m)')
plt.plot(x, y5, label='y Error (m)')
plt.plot(x, y6, label='z Error (m)')

# Add labels and a legend
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.title('End Effoector Pose Error vs Time')
plt.legend()

# Show the plot
plt.show()

