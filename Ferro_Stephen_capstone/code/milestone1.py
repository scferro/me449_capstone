import numpy as np
import modern_robotics as mr
import csv

def NextState(currentConfig, wheelSpeeds, jointSpeeds, newGripperState, dt=0.01, maxAngSpeed=5*np.pi):
    """
    Decription:
        Predicts the next state of the robot based on the current state and current joint velocities
    Args:
        • currentConfig: The current configuration of the robot ([phi_b, x_b, y_b, armjoint1, armjoint2, armjoint3, armjoint4, armjoint5, wheel1, wheel2, wheel3, wheel4, gripper_state])
        • speedCommands: The current rotational speeds of each joint ([armjoint1_vel, armjoint2_vel, armjoint3_vel, armjoint4_vel, armjoint5_vel, wheel1_vel, wheel2_vel, wheel3_vel, wheel4_vel])
        • newGripperState: The state of the gripper in the new command
        • dt: The simulation timestep (float)
        • maxAngSpeed: The maximum rotational speed of all joints and wheels (float)
    Returns:
        • newConfig: The new configuration of the robot ([phi_b, x_b, y_b, armjoint1, armjoint2, armjoint3, armjoint4, armjoint5, wheel1, wheel2, wheel3, wheel4, gripper_state])
    """

    # Define robot variables
    l = 0.235       # the distance from the center of the robot to the front/rear axle (see p.521 of Modern Robotics)
    w = 0.15        # the distance from the center of the robot to the LH/RH wheel centerline (see p.521 of Modern Robotics)
    r = 0.0475      # the wheel radii

    # Limit joint speeds to +/- maxAngSpeed
    wheelSpeedsLimited = np.array([])
    for speed in wheelSpeeds:
        if speed > maxAngSpeed:
            speed = maxAngSpeed
        elif speed < -maxAngSpeed:
            speed = -maxAngSpeed
        wheelSpeedsLimited = np.hstack((wheelSpeedsLimited, speed))
    jointSpeedsLimited = np.array([])
    for speed in jointSpeeds:
        if speed > maxAngSpeed:
            speed = maxAngSpeed
        elif speed < -maxAngSpeed:
            speed = -maxAngSpeed
        jointSpeedsLimited = np.hstack((jointSpeedsLimited, speed))

    # Calculate the change in angle at each joint
    angleChangeWheel = wheelSpeedsLimited * dt
    angleChangeJoint = jointSpeedsLimited * dt

    # Calculate the change in chassis position
    u = angleChangeWheel.T
    dimensionArray = np.array([[-l-w, 1, -1], 
                               [l+w, 1, 1],
                               [l+w, 1, -1],
                               [-l-w, 1, 1]])
    dimensionArrayInv = np.linalg.pinv(dimensionArray)
    chassisVel = r * np.matmul(dimensionArrayInv, wheelSpeedsLimited.T)
    chassisChange = chassisVel * dt

    # Generate the new configuration based on the previous config and the joint speeds
    newConfig = np.array([currentConfig[0]+chassisChange[0], currentConfig[1]+chassisChange[1], currentConfig[2]+chassisChange[2],                                                                                              # chassis state: phi, x, y 
                        currentConfig[3]+angleChangeJoint[0], currentConfig[4]+angleChangeJoint[1], currentConfig[5]+angleChangeJoint[2], currentConfig[6]+angleChangeJoint[3], currentConfig[7]+angleChangeJoint[4],           # arm joint positions: theta1, theta2, theta3, theta4, theta5
                        currentConfig[8]+angleChangeWheel[0], currentConfig[9]+angleChangeWheel[1], currentConfig[10]+angleChangeWheel[2], currentConfig[11]+angleChangeWheel[3],                                               # wheel positions: wheel1, wheel2, wheel3, wheel4
                        newGripperState])                                                                                                                                                                                       # gripper state                    
    
    return newConfig


def write_to_csv(array, file_name):
    """
    Decription:
        Writes an array to a csv file
    Args:
        • array: The array to be written to the file
        • file_name: The filename to be used
    """
    print("Writing to CSV file %s" % (file_name))
    with open(file_name, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(array)


### UNCOMMENT CODE BELOW FOR TESTING ###

# u1_wheel = np.array([10, 10, 10, 10])
# u1_joint = np.array([1, 0, 0, 0, 0])
# u2_wheel = np.array([-10, 10, -10, 10])
# u2_joint = np.array([0, 0, 0, 0, 0])
# u3_wheel = np.array([-10, 10, 10 , -10])
# u3_joint = np.array([0, 0, 0, 0, 0])

# config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# time = 0.0
# dt = 0.01

# filename = 'Ferro_Stephen_milestone1.csv'
# configArray = np.array(config)

# maxTime = 5

# while time <= maxTime:
#     config = NextState(config, u1_wheel, u1_joint, 0)
#     configArray = np.vstack((configArray, config))
#     time += dt

# print("Final Configuration: %s" % (configArray[-1]))
# write_to_csv(configArray, filename)

