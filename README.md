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
