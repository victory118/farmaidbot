#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self):
        # for local exponential stability:
        # kp > 0; kb < 0; ka > kp
        # strong stability condition:
        # ka + 5/3*kb - 2/pi*kp > 0
        # kb > (2/pi*kp - ka)*5/3
        self.kp = 3
        self.ka = 5
        self.kb = 0
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """

        # state is the current base_link pose in map frame
        # goal is the desired base_link pose in map frame
        state = state.flatten()
        goal = goal.flatten()

        dx = goal[0] - state[0] # robot to goal along world frame x-axis
        dy = goal[1] - state[1] # robot to goal along world frame y-axis
        theta = state[2] # orientation of robot
        theta_goal = goal[2]
	    
        rho = np.sqrt(dx*dx + dy*dy) # distance to goal

        alpha = np.arctan2(dy,dx) - theta # angle to goal relative to heading of robot in world frame
        beta = theta_goal - theta - alpha # angle between the robot's position and the goal position plus the goal angle in world frame

        alpha = np.arctan2(np.sin(alpha), np.cos(alpha)) # wrap angle from angle to [-pi, pi]
        beta = np.arctan2(np.sin(beta), np.cos(beta))

        # https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/move_to_pose/move_to_pose.py
        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        # Without restricting the angle to [-pi, pi], the trajectory can become unstable
        # alpha = (np.arctan2(dy,dx) - theta + np.pi) % (2*np.pi) - np.pi
        # beta = (theta_goal - theta - alpha + np.pi) % (2*np.pi) - np.pi

        # v = 0.3 # set velocity constant for testing
        v = self.kp*rho
        omega = self.ka*alpha + self.kb*beta

        # This allows the velocity to be negative if the goal is behind the robot.
        # In practice, this makes the total trajectory shorter.
        # However, note that RobotSim restricts the velocity to be non-negative so if the
        # commanded velocity is negative it gets set to zero. The robot will probably do
        # a pure rotation until it's heading is facing the goal, then proceed to drive towards it.
        
        # Note: The simulation RobotSim.py prevents negative velocity commands. This *may* mess up 
        # the Kalman filter because it uses the velocity command in the prediction step.
        # In other words, the Kalman filter may use a negative velocity command in the
        # prediction model but the velocity was actually zero.
        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        at_goal = False

        if abs(rho) < 0.05:

            v = 0.0
            omega = 0.0
            at_goal = True

            # print 'in here'
            # print "v = ", v
            # print "omega = ", omega
            # print "done = ", done

            return v, omega, at_goal
             
        return v, omega, at_goal
