import numpy as np
import modern_robotics as mr
import csv

def NextState(currentConfig, currentSpeeeds, dt=0.01, maxAngSpeed=2*np.pi):

    l =             # the distance from the center of the robot to the front/rear axle (see p.521 of Modern Robotics)
    w =             # the distance from the center of the robot to the LH/RH wheel centerline (see p.521 of Modern Robotics)
    r =             # the wheel radii

    for speed in currentSpeeeds:
        if speed > maxAngSpeed:
            speed = maxAngSpeed

    angleChange = currentSpeeeds * dt

    u = np.array([currentSpeeeds[5], currentSpeeeds[6], currentSpeeeds[7], currentSpeeeds[8]]).T

    dimensionArray = np.array([-l-w, 1, -1, 
                               l+w, 1, 1,
                               l+w, 1, -1,
                               -l-w, 1, 1])
    
    dimensionArrayInv = np.linalg.inv(dimensionArray)
    
    chassisVel = r * np.matmul(dimensionArrayInv, u)

    newConfig = np.array([chassisVel[0], chassisVel[1], chassisVel[2],                                                                                                                              # chassis state: phi, x, y 
                    currentConfig[3]+angleChange[0], currentConfig[4]+angleChange[1], currentConfig[5]+angleChange[2], currentConfig[6]+angleChange[3], currentConfig[7]+angleChange[4],            # arm joint positions: theta1, theta2, theta3, theta4, theta5
                    currentConfig[8]+angleChange[5], currentConfig[9]+angleChange[6], currentConfig[10]+angleChange[7], currentConfig[11]+angleChange[8]])                                          # wheel positions: wheel1, wheel2, wheel3, wheel4
    
    return newConfig


def write_to_csv(array, file_name):
    """
    Decription:
        Writes an array to a csv file
    Inputs:
        • array: The array to be written to the file
        • file_name: The filename to be used
    """
    with open(file_name, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(array)


u1 = np.array([0, 0, 0, 0, 0, 10, 10, 10, 10])
u2 = np.array([0, 0, 0, 0, 0, -10, 10, -10, 10])
u3 = np.array([0, 0, 0, 0, 0, -10, 10, 10 , -10])

config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

time = 0.0
dt = 0.01

filename = 'milestone1.csv'
configArray = np.array([config])

while time <= 1.0:
    config = NextState(config, u1, dt)
    configArray = np.vstack(configArray, config)
    time += dt

print(configArray)
write_to_csv(configArray, filename)

