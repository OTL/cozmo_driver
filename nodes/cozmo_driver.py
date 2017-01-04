#!/usr/bin/env python3

import sys
import time

import cozmo
from cozmo.util import radians
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image


class CozmoRos(object):
    '''Cozmo ros node
    '''
    def __init__(self, coz):
        self._cozmo = coz
        self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback)
        self._say_sub = rospy.Subscriber('say', String, self._say_callback)
        self._head_sub = rospy.Subscriber('head', Float64, self._move_head)
        self._lift_sub = rospy.Subscriber('lift', Float64, self._move_lift)
        self._image_pub = rospy.Publisher('image', Image, queue_size=1)

    def _move_head(self, cmd):
        action = self._cozmo.set_head_angle(radians(cmd.data), duration=0.1, in_parallel=True)
#        action.wait_for_completed()

    def _move_lift(self, cmd):
        action = self._cozmo.set_lift_height(height=cmd.data * 10, duration=0.1, in_parallel=True)
#        action.wait_for_completed()

    def _twist_callback(self, cmd):
        # TODO: convert m/s to motor speed
        lv = (cmd.linear.x + cmd.angular.z) * 100.0
        rv = (cmd.linear.x - cmd.angular.z) * 100.0
        self._cozmo.drive_wheels(lv, rv)

    def _say_callback(self, msg):
        self._cozmo.say_text(msg.data).wait_for_completed()

    def _publish_image(self):
        camera_image = self._cozmo.world.latest_image
        if camera_image is not None:
            img = camera_image.raw_image
            ros_img = Image()
            ros_img.encoding = 'rgb8'
            ros_img.width = img.size[0]
            ros_img.height = img.size[1]
            ros_img.step = 3 * ros_img.width
            ros_img.data = img.tobytes()
            ros_img.header.frame_id = 'cozmo_camera'
            cozmo_time = camera_image.image_recv_time
            ros_img.header.stamp = rospy.Time.from_sec(cozmo_time)
#            ros_img.header.stamp = rospy.Time.now()
            self._image_pub.publish(ros_img)

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish_image()
            r.sleep()
        self._cozmo.stop_all_motors()


def run(coz_conn):
    coz = coz_conn.wait_for_robot()
    coz.camera.image_stream_enabled = True
    coz_ros = CozmoRos(coz)
    coz_ros.spin()


if __name__ == '__main__':
    rospy.init_node('cozmo_driver')
    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
