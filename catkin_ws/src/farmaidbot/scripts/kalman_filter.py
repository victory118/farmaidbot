#!/usr/bin/python
import numpy as np

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, pose_init, wheelbase, wheel_radius):
        """
        Initialize all necessary components for Kalman Filter

        Input: 
        pose_init - (3,) numpy array with initial pose estimate (x, y, theta)
        wheelbase - wheelbase of the robot in meters
        wheel_radius - wheel radius of the robot in meters
        """

        self._Q_t = np.diag([0.035, 0.035]) # angle in radians rotated noise variance (process noise)
        self._R_t = 0.001*np.diag([0.00127, 0.00127, 2.0*np.pi/180]) # camera Apriltag noise variance (measurement noise)
        self._s_t = pose_init # initialize state variables
        self._P_t = np.diag([10, 10, 10]) # initialize covariance matrix

        self._prev_wheel_angle_left = 0
        self._prev_wheel_angle_right = 0

        self._wheelbase = wheelbase
        self._wheel_radius = wheel_radius

    def prediction(self, odom_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        odom_meas - a (2,) numpy array consistening of the the current wheel angles
                    in radians (wheel_angle_left, wheel_angle_right)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the prediction of the state
        predicted_covariance - a 3 by 3 numpy array of the prediction of the
            covariance
        """
        drad_left = odom_meas[0] - self._prev_wheel_angle_left
        drad_right = odom_meas[1] - self._prev_wheel_angle_right

        self._prev_wheel_angle_left = odom_meas[0]
        self._prev_wheel_angle_right = odom_meas[1]

        x_pre = self._s_t[0] + self._wheel_radius*(drad_left + drad_right)/2.0*np.cos(self._s_t[2])
        y_pre = self._s_t[1] + self._wheel_radius*(drad_left + drad_right)/2.0*np.sin(self._s_t[2])
        th_pre = self._s_t[2] + self._wheel_radius/self._wheelbase*(drad_right - drad_left)
        th_pre = np.arctan2(np.sin(th_pre), np.cos(th_pre)) # wrap angle to [-pi, pi]
        
        dfdx = np.array([[1., 0., -self._wheel_radius*(drad_left + drad_right)/2.0*np.sin(self._s_t[2])]]) # derivative of prediction model with respect to x position
        dfdy = np.array([[0., 1., self._wheel_radius*(drad_left + drad_right)/2.0*np.cos(self._s_t[2])]]) # derivative of prediction model with respect to y position
        dfdth = np.array([[0., 0., 1.]]) # deriv. wrt theta
        dfds = np.concatenate((dfdx, dfdy, dfdth), axis=0) # Jacobian of prediction model wrt states (3,3)
        
        dfxdn = np.array([[self._wheel_radius/2.0*np.cos(self._s_t[2]), self._wheel_radius/2.0*np.cos(self._s_t[2])]]) # deriv. of x position wrt process noise
        dfydn = np.array([[self._wheel_radius/2.0*np.sin(self._s_t[2]), self._wheel_radius/2.0*np.sin(self._s_t[2])]]) # deriv. of x position wrt process noise
        dfthdn = np.array([[self._wheel_radius/self._wheelbase, self._wheel_radius/self._wheelbase]]) # deriv. of theta orientation wrt process noise
        dfdn = np.concatenate((dfxdn, dfydn, dfthdn), axis=0) # deriv. of prediction model wrt noise (3,2)
        
        self._s_t = np.array([x_pre, y_pre, th_pre]) # predicted state
        self._P_t = dfds.dot(self._P_t).dot(dfds.T) + dfdn.dot(self._Q_t).dot(dfdn.T) # predicted covariance
        
        return self._s_t, self._P_t

    def update(self, tag_meas):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        tag_meas - (N, 3) Numpy array of the calculated base_link pose(s) in the map frame
                   based on Apriltag detections: (x, y, theta)
                   Will be None value if measurement is not available.
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        
        dh = np.eye(3) # Jacobian of measurement model wrt states

        for pose_map2base in tag_meas: # iterate through robot poses

            K = self._P_t.dot(dh.T).dot(np.linalg.inv(dh.dot(self._P_t).dot(dh.T) + self._R_t)) # Kalman gain
            
            self._s_t = self._s_t + K.dot(pose_map2base - self._s_t) # update pose estimate
            self._s_t[2] = np.arctan2(np.sin(self._s_t[2]), np.cos(self._s_t[2])) # wrap angle to [-pi, pi]
            self._P_t = (np.eye(3) - K.dot(dh)).dot(self._P_t) # update covariance matrix

        return self._s_t, self._P_t
        
    def step_filter(self, odom_meas, tag_meas):
        """
        Perform step in filter, called every iteration
        Inputs:
        odom_meas - (2,) numpy array of the the current wheel angles
                    in radians (wheel_angle_left, wheel_angle_right)
                    Will be None value if values are not available.
        tag_meas - (N, 3) Numpy array of the calculated base_link pose(s) in the map frame
                   based on Apriltag detections: (x, y, theta)
                    Will be None value if measurement is not available.
        Outputs:
        s_t - current estimate of the state
        """
        if odom_meas is not None:
            self._s_t, self._P_t = self.prediction(odom_meas)
        
        if tag_meas is not None:
            self._s_t, self._P_t = self.update(tag_meas)
            
        return self._s_t
            