# cozmo driver for ROS

This is ROS node for Anki cozmo.

I'm using Kinetic/Ubuntu16.04.

## Super hack to run rospy from python3

This is not recommended as usual, but it seems difficult to run rospy from python3 normally.
Below hack will allow run python3, at least for cozmo_driver.py.
I don't know the true risk to do that.

```bash
sudo apt-get install python3-yaml
sudo pip3 install rospkg catkin_pkg
```

### TODO

* It is difficult to use tf2 on python3, it has tf2.so. How can we use?
transform_stamped_to_tf2.py will convert the transform message to tf2 message.
* camera stops after a few minutes.
* use trajectory_msgs to command head angle and lift height.
* publish battery state

## Pub/Sub

### Publish

 * /image (sensor_msgs/Image) : camera image from cozmo. This is gray scale, but the format is rgb8.
 * /joint_states (sensor_msgs/JointState) : This contains head angle and lift height
 * /transforms (geometry_msgs/TransformStamped) : poses of visialbe cubes and cozmo

### Subscribe

 * /cmd_vel (geometry_msgs/Twist) : command velocity as usual. (velocity is not correct)
 * /say (std_msgs/String) : cozmo says this text
 * /head_angle (std_msgs/Float64) : command head angle [rad]
 * /lift_height (std_msgs/Float64) : command lift height [m]
 * /backpack_led (std_msgs/ColorRGBA) : led color on backpack
