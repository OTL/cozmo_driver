# cozmo driver for ROS

This is **unofficial** ROS node for Anki cozmo.

## Requirements

This is tested on Kinetic/Ubuntu16.04 and Android only.

 * Ubuntu 16.04
 * ROS Kinetic
 * Python3.5
 * (Android)
 * (Cozmo SDK 0.10)

### Note

Cozmo SDK will become idle mode if the message is not send to cozmo for a few minutes.
You need send some messages repeatedly if you don't want to use idle mode.
I mean that when the camera has been stopped, try to send some command like /cmd_vel.

## Super hack to run rospy from python3

This is not recommended as usual, but it seems difficult to run rospy from python3 normally. (It requires full recompile of all ROS packages.)
Below hack will allow run python3, at least for cozmo_driver.py.
I don't know the true risk to do that.

```bash
sudo apt-get install python3-yaml
sudo pip3 install rospkg catkin_pkg
```

## TODO

* difficult to use tf2 on python3, it has tf2.so. How can we use?
transform_stamped_to_tf2.py will convert the transform message to tf2 message.
* use trajectory_msgs to command head angle and lift height.

## Pub/Sub

### Publish

 * /image (sensor_msgs/Image) : camera image from cozmo. This is gray scale, but the format is rgb8.
 * /joint_states (sensor_msgs/JointState) : This contains head angle and lift height
 * /transforms (geometry_msgs/TransformStamped) : poses of visialbe cubes and cozmo. if you need /tf, use transform_stamped_to_tf2.py
 * /imu (sensor_msgs/Imu) : Imu mounted on cozmo head
 * /battery (sensor_msgs/BatteryState) : battery voltage and charging status

### Subscribe

 * /cmd_vel (geometry_msgs/Twist) : command velocity as usual. (velocity is not correct)
 * /say (std_msgs/String) : cozmo says this text
 * /head_angle (std_msgs/Float64) : command head angle [rad]
 * /lift_height (std_msgs/Float64) : command lift height [m]
 * /backpack_led (std_msgs/ColorRGBA) : led color on backpack


## Install cozmo SDK on Ubuntu16.04 and Android

Please follow [original document](http://cozmosdk.anki.com/docs/install-linux.html#install-linux). Below is super quick hacky installation for only me.

```bash
sudo apt-get update
sudo apt-get install python3 python3-pip python3-pil.imagetk default-jre adb
pip3 install --user 'cozmo[camera]'
```
