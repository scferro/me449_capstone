# ME449 Capstone Project

## Author: 
- Stephen Ferro

## How to Use:
1. To run, run the 'main.py' file in the '/code' directory. 
2. Values in the input section at the top can be adjusted to explore different scenarios

##  Results:
The task was completed in several different ways:
1. With an optimal controller, which minimized overshoot and error while completing the task. This was accomplished using the feedforward controller with PI control. 
2. With a controller that induces overshoot. The overshoot controller was a simple PI controller with poorly tuned gains. It was still able to pick up the cube.
3. With joint constraints turned off. This allows the robot to collide with itself.
4. A new task. For this scenario, the robot picks up and moves the block to a new location, different from the other scenarios.

Results for all of these scenarios, including videos and graphs of error over time, can be viewed in the 'Results' directory.

## Discussion:
Overall the results of the assignment were not very surprising. The feedforward controller with tightly tuned gains performs best, while the simpler PI controller with sloppy gains performs the worst. Though the overshoot scenario was less precise than the optimal case, it did still manage to pick up and move the cube. Through the new task scenario, the robot demonstrates the ability to plan for new goals. In all cases, error increases slightly around the time the robot picks up the block. This makes sense, as turn around after picking up the block is one of the more complex maneuvers in this routine, resulting in more error for a short period. 

## Joint Limits:
Joint limits are monitored by the CheckJointLimits function in milestone3. This function checks if the next joint positions are outside the allowable range for the robot. If they are, the function sets the corresponding line of the Jacobian equal to 0. This ensure that the joint at it's limits is not updated and does not pass the limits. The new Jacobian is returned and used to find the next set of velocity commands for the robot. From the scenario with joint limits turned off, we can see that the robot is more likely to enter strange poses and collide with itself when collision is turned off. That being said, the robot is also able to stay closer to the target Tse_d when it is allowed to ignore joint constraints. Obviously this would not be practical in the real world. 