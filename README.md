The following document contains instructions for running the Farmaidbot.

### Prerequisites:

* Computer runs Ubuntu 16.04 OS with ROS Kinetic installed. This machine will be called the **master** machine because it will be running the ROS master node (roscore).
* Raspberry Pi runs Ubuntu MATE OS with ROS Kinetic installed. This machine will be called the **slave** machine and will be running some auxiliary ROS nodes.
* Both master and slave should have the **fiducials** ROS package installed. Follow the instructions here: http://wiki.ros.org/fiducials. I have only tested the fiducials package on ROS Kinetic. The fiducials package is not officially supported on ROS Melodic. I have tried and failed to make it work on ROS Melodic.
* Master machine has the Arduino IDE installed and the ability to upload code to the Arduino Mega2560.
* Both master and slave machines are connected to the same wireless network.

### Instructions:

First clone the repository onto the master machine:

```bash
$ cd ~
$ git clone git@github.com:victory118/farmaidbot.git
```

Go into the catkin workspace directory and build the ROS packages:

```bash
$ cd ~/farmaidbot/catkin_ws
$ catkin_make
```

Source the workspace:

```bash
$ cd ~/farmaidbot/catkin_ws
$ source devel/setup.bash
```

Repeat the same procedure as above on the slave machine so that the same catkin workspace exists on both the master and slave.

Connect the Arduino Mega2560 to the master machine using a USB serial cable. Make sure the power to the motors is disconnected until you are ready for the robot to start moving. Upload the Arduino sketch called **farmaidbot.ino** onto the master machine which is located in the directory `/catkin_ws/src/farmaidbot/src/farmaidbot`. Disconnect the serial cable from the master machine and connect it to the one USB ports on the RPi.

On the master, SSH into the slave machine:

```bash
$ ssh victor@192.168.1.11
```

The RPi's user name is "victor" and password is "hello". Here, assume the IP address of the RPi is 192.168.1.11, although this will change depending on the network.

On the slave machine, export the ROS_MASTER_URI and ROS_IP:

```bash
$ export ROS_MASTER_URI=http://192.168.1.8:11311
$ export ROS_IP=`hostname -I`
```

Here, assume that the IP address of the master machine is 192.168.1.8, although this will change depending on the network. Run the command `hostname -I` in the terminal to get the IP address. To check that the ROS variables are set correctly use the following commands:

```bash
$ echo $ROS_IP
$ echo $ROS_MASTER_URI
```

On the master machine, export the ROS_IP:

```bash
$ export ROS_IP=`hostname -I`
```

On the master machine, run the ROS master node:

```bash
$ roscore
```

On the slave machine, execute the first launch file:

```bash
$ roslaunch farmaidbot usb_cam_aruco.launch
```

This launch file executes two other launch files (camera_rect.launch and aruco_detect.launch) and the **base_controller** node. camera_rect.launch is responsible for calling another launch file camera.launch that runs the **usb_cam_node**. aruco_detect.launch executes another launch file aruco_detect.launch from the aruco_detect package, which is part of the fiducials package. When an Aruco marker is detected by the camera, it publishes the pose of the tag with respect to camera to the /fiducial_transforms topic. The base_controller node sets up the serial communication between the RPi and Arduino. The base_controller node subscribes to the /cmd_vel topic and sends these commands via serial communication to the Arduino. The Arduino sends odometry information via serial communication to the base_controller node, which then publishes it to the /wheel_angle topic.

On the master machine, execute the second launch file:

```bash
$ roslaunch farmaidbot aruco_navigation.launch
```

This loads some parameters and executes the **aruco_navigation** node. The aruco_navigation node is responsible for reading the odometry and camera (Aruco marker) measurements at regular intervals and applying sensor fusion using an extended Kalman filter to estimate the pose of the robot in the map frame. This node is also responsible to broadcasting all of the static transforms, such as the pose of the camera in the base_link frame and the pose of each Aruco marker in the map frame.

On the master machine, open a new terminal and run the teleop node:

```bash
$ rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/cmd_vel
```

Now you should be able to control the robot using keyboard commands. Make sure that the power to the motors is connected.

To visualize the published messages on the master machine, you can run RViz. Select /map as the parent frame. You should see the static transforms of the Aruco markers such as /apriltag_100 and /apriltag_101, which are the pre-defined transforms of each tag in the /map frame. You should also see the /base_link and /base_link_ekf frames. The /base_link frame is the estimated pose of the robot based **only** on the Aruco markers detections. This transform tends to be jumpy due to the camera noise, but it provides a global pose information. The /base_link_ekf frame is estimated pose of the robot based on fusing the odometry measurements and Aruco marker detections. This transform tends to be smoother because the odometry measurements are locally very accurate, but tend to drift over time.

 ### To-dos:

* Implement a time-out functionality in the Arduino, so that the commanded velocity goes to zero if it doesn't receive a velocity command in some specified duration.
* 

