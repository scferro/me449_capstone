# me449_capstone, milestone 2

# Decription:
    Use the TrajectoryGenerator() to generate a trajectory for the robot arm to grasp a cube and move it to a new position. Inputs and outputs are given below.
# Inputs:
    • Tse_init: The initial position of the EE realtive to the space frame (Transformation matrix)
    • Tsc_init: The initial position of the cube realtive to the space frame (Transformation matrix)
    • Tsc_final: The desired final position of the cibe realtive to the space frame (Transformation matrix)
    • Tce_grasp: The position of the EE realtive to the cube frame when grasping and releasing the cube (Transformation matrix)
    • Tce_standoff: The position of the EE realtive to the cube frame before grasping and after releasing (Transformation matrix)
    • k: The number of trajectory positions per 0.01 seconds (int)
# Outputs:
    • traj_out: A list of rotation and position vectors for each point of the robot's full trajectory (list of lists). This output trajectory will also be written to the CSV file "trajectory.csv" for usage with CopelliaSim