# cozmo driver for ROS

This is **unofficial** ROS node for Anki cozmo.

## Requirements

This is tested on Kinetic/Ubuntu16.04 and Android only. Latest version is tested on Indigo/Ubuntu14.04.

 * Ubuntu 14.04/16.04
 * ROS Indigo/Kinetic
 * Python3.5
 * (Android)
 * (Cozmo SDK 0.10+)

Use rosdep to install the required ROS packages:
```sh
rosdep install cozmo_driver
```

### Note

Cozmo SDK will become idle mode if the message is not sent to cozmo for a few minutes. To avoid idle mode, cozmo_driver.py is sending /cmd_vel repeatedly in 10[Hz].

## Super hack to run rospy from python3

**Note: Be careful installing python3, you should update python3 alternative to python3.5 before installing and using pip3!**

This is not recommended as usual, but it seems difficult to run rospy from python3 normally. (It requires full recompile of all ROS packages.)
Below hack will allow run python3, at least for cozmo_driver.py.
I don't know the true risk to do that.

```bash
sudo apt-get install python3-yaml
sudo pip3 install rospkg catkin_pkg
```

Note: To get further python3 compat a local version of "transformations.py" is included!

## TODO

Feel free to provide new ToDo's at Githubs Issues page!

## Pub / Sub / Tf

### Publish

 * /cozmo_camera/image (sensor_msgs/Image) : camera image from cozmo. This is gray scale, but the format is rgb8.
 * /cozmo_camera/camera_info (sensor_msgs/CameraInfo) : The camera info from Cozmo's camera ( **Note: I did a very poor calibration!** ).
 * /joint_states (sensor_msgs/JointState) : This contains the head angle [rad] and the lift height [m]
 * /imu (sensor_msgs/Imu) : Imu mounted on cozmo head
 * /battery (sensor_msgs/BatteryState) : battery voltage and charging status
 * /diagnostics (diagnostic_msgs/DiagnosticArray) : Several robot diagnostics.

### Subscribe

 * /cmd_vel (geometry_msgs/Twist) : command velocity as usual. ( **Note: velocity is correct now!** )
 * /say (std_msgs/String) : cozmo says this text
 * /head_angle (std_msgs/Float64) : command head angle [rad]
 * /lift_height (std_msgs/Float64) : command lift height (value between 0 and 1, which is autoscaled by Cozmo SDK) [float]
 * /backpack_led (std_msgs/ColorRGBA) : led color on backpack

### Transformations provided

 * odom_frame -> footprint_frame
 * footprint_frame -> base_frame
 * base_frame -> head_frame
 * head_frame -> camera_frame
 * camera_frame -> camera_optical_frame
 * odom_frame -> cube_xx (for detected cubes)

## Install cozmo SDK on Ubuntu16.04 and Android

**Note: The current driver whas tested on Ubuntu14.04 with ROS Indigo.**

Please follow the [original document](http://cozmosdk.anki.com/docs/install-linux.html#install-linux). Below is a quick hacky installation for only me.

```bash
sudo apt-get update
sudo apt-get install python3 python3-pip python3-pil.imagetk default-jre adb
pip3 install --user 'cozmo[camera]'
```

## Hardware configuration

It is normal for Cozmo SDK, but it has below hardware configuration. ROS is running on your **PC** not on Cozmo.

**Cozmo** (WiFi station) <-- WiFi --> **Phone** (Android) <-- USB cable --> **PC** (Cozmo official SDK <-> ROS)


## Demo Applications

### Cozmo Driver

This demo application starts the Cozmo ROS driver, a keyboard teleoperation node and a twist multiplexing node. Additionally rviz is started for visiualization.
Optionally joystick input can be used with argument "use_joy".

```sh
roslaunch cozmo_driver demo.launch
```

To enable joystick use:
```
roslaunch cozmo_driver demo.launch use_joy:=true
```

### RatSLAM

**Note:** To get the RatSLAM demo working you need to build the sources from [here](https://github.com/sem23/ratslam.git) in your ROS workspace.

This demo application starts a SLAM system utilizing a biological inspired SLAM, called RatSLAM. In a few words, it is a
monocular visiual SLAM building an experience map from odometry and image information. The provided version supports a simple
navigation system using a naive path follow controller. So be careful while using it, as neither Cozmo nor the path controller
provide collision avoidance!

```sh
roslaunch cozmo_driver ratslam.launch
```

