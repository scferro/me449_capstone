import numpy as np
import modern_robotics as mr
import csv

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, k=1):
    """
    Decription:
        Generate a trajectory for the robot arm to grasp a cube and move it to a new position
    Args:
        • Tse_init: The initial position of the EE realtive to the space frame (Transformation matrix)
        • Tsc_init: The initial position of the cube realtive to the space frame (Transformation matrix)
        • Tsc_final: The desired final position of the cibe realtive to the space frame (Transformation matrix)
        • Tce_grasp: The position of the EE realtive to the cube frame when grasping and releasing the cube (Transformation matrix)
        • Tce_standoff: The position of the EE realtive to the cube frame before grasping and after releasing (Transformation matrix)
        • k: The number of trajectory positions per 0.01 seconds (int)
    Returns:
        • traj_out: A list of rotation and position vectors for each point of the robot's full trajectory (list of lists)
    """

    # Define standoff ans grasp Transformations relative to cube
    angle_grasp = 2
    Tce_grasp = np.array([[np.cos(angle_grasp), 0, np.sin(angle_grasp), 0],
                        [0,1,0,0],
                        [-np.sin(angle_grasp),0,np.cos(angle_grasp),0],
                        [0,0,0,1]
                        ])
    Tce_standoff = np.array([[0,0,1,0],
                        [0,1,0,0],
                        [-1,0,0,0.5],
                        [0,0,0,1]
                        ])

    # Set movement time for each action and the number of points to be calculated for each motion
    timeMove = 2
    numPoints = timeMove / (0.01 / k)

    # Generate transformation matrices relative to the spaceframe for each position of the gripper
    Tse_standoff_init = Tsc_init @ Tce_standoff
    Tse_grasp_init = Tsc_init @ Tce_grasp
    Tse_standoff_final = Tsc_final @ Tce_standoff
    Tse_grasp_final = Tsc_final @ Tce_grasp
    
    # Find the trajectory from the home position to the standoff from the cube at its initial position 
    traj1 = extract_variables(mr.CartesianTrajectory(Tse_init, Tse_standoff_init, timeMove, numPoints, 5), 0)
    # Find the trajectory from the standoff from the cube at its initial position to grasping the cube at its initial position
    traj2 = extract_variables(mr.CartesianTrajectory(Tse_standoff_init, Tse_grasp_init, timeMove, numPoints, 5), 0)
    # Close the gripper on the cube
    counter3 = 0
    traj3 = []
    vec3 = traj2[-1]
    vec3[-1] = 1
    while counter3 < numPoints:
        traj3.append(vec3)
        counter3 += 1
    # Find the trajectory from the graps position to the initial standoff position
    traj4 = extract_variables(mr.CartesianTrajectory(Tse_grasp_init, Tse_standoff_init, timeMove, numPoints, 5), 1)
    # Find the trajectory from the initial standoff position to the final standoff position
    traj5 = extract_variables(mr.CartesianTrajectory(Tse_standoff_init, Tse_standoff_final, timeMove, numPoints, 5), 1)
    # Find the trajectory from the final standoff position to the final cube position
    traj6 = extract_variables(mr.CartesianTrajectory(Tse_standoff_final, Tse_grasp_final, timeMove, numPoints, 5), 1)
    # Release the cube
    counter7 = 0
    traj7 = []
    vec7 = traj6[-1]
    vec7[-1] = 0
    while counter7 < numPoints:
        traj7.append(vec7)
        counter7 += 1
    # Find the trajectory from the release position back to the final standoff position
    traj8 = extract_variables(mr.CartesianTrajectory(Tse_grasp_final, Tse_standoff_final, timeMove, numPoints, 5), 0)

    # Combine the individual trajectories into a single output trajectory
    traj_out = np.vstack((traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8))

    #write_to_csv(traj_out,'traj.csv')
    
    return traj_out

def extract_variables(T, gripper_var):
    """
    Decription:
        Extracts rotation and position variables from a transformation matrix and adds a variables that conttrols position of the gripper
    Args:
        • T: An array of transformation matrices along a robot trajectory (Array of transformation matrices)
        • gripper_var: The desired state of the gripper; 1 for closed, 0 for open (int)
    Returns:
        • output: A list of rotation and position vectors for each point of the robot's trajectory (list of lists)
    """
    output = []
    for array in T:
        # Add variables from transformation matrix to a vector and add the vector to the output list of vectors
        vec = [array[0,0], array[0,1], array[0,2], array[1,0], array[1,1], array[1,2], array[2,0], array[2,1], array[2,2], array[0,3], array[1,3], array[2,3], gripper_var]
        output.append(vec)
    return output

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

# Initialize Transformaations

Tse_init = np.array([[0,0,1,0],
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

# Plan trajectory using TrajectoryGenerator() function

traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final)

print(traj)
    

