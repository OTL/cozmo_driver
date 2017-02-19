#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This script implements a ROS node, which is used to
control cozmo's head angle and lift height via joy-pad.

Copyright {2017} {Peter Rudolph}

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

MIN_HEAD_ANGLE = -25.0
MAX_HEAD_ANGLE = 44.5
SUM_HEAD_ANGLE = MAX_HEAD_ANGLE - MIN_HEAD_ANGLE

MIN_LIFT_HEIGHT = 32.0
MAX_LIFT_HEIGHT = 92.0
SUM_LIFT_HEIGHT = MAX_LIFT_HEIGHT - MIN_LIFT_HEIGHT


class HeadLiftJoy(object):
    """
    The HeadLiftJoy object used to handle the ROS I/O.

    It subscribes to "joy" topic, checks if head/tilt
    button is pressed and sends the transformed value
    to corresponding "head_angle/lift_height" topic.

    The min/max parameters are taken from cozmo SDK.

    """

    def __init__(self):
        """
        Create HeadLiftJoy object.

        """
        # params
        self._head_axes = rospy.get_param('~head_axes', 3)
        self._lift_axes = rospy.get_param('~lift_axes', 3)
        self._head_button = rospy.get_param('~head_button', 4)
        self._lift_button = rospy.get_param('~lift_button', 5)

        # pubs
        self._head_pub = rospy.Publisher('head_angle', Float64, queue_size=1)
        self._lift_pub = rospy.Publisher('lift_height', Float64, queue_size=1)

        # subs
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_cb, queue_size=1)

    def _joy_cb(self, msg):
        """
        The joy/game-pad message callback.

        :type   msg:    Joy
        :param  msg:    The incoming joy message.

        """
        if msg.buttons[self._head_button] == 1:
            angle_deg = ((msg.axes[self._head_axes] + 1.0) / 2.0) * SUM_HEAD_ANGLE + MIN_HEAD_ANGLE
            rospy.logdebug('Send head angle: {} degrees.'.format(angle_deg))
            self._head_pub.publish(data=angle_deg)

        if msg.buttons[self._lift_button] == 1:
            lift_mm = abs(msg.axes[self._lift_axes]) * SUM_LIFT_HEIGHT + MIN_LIFT_HEIGHT
            rospy.logdebug('Send lift height: {} mm.'.format(lift_mm))
            self._lift_pub.publish(data=abs(msg.axes[self._lift_axes]))

    @staticmethod
    def run():
        """
        This function just keeps the node alive.

        """
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cozmo_head_lift_joy_node')
    hlj = HeadLiftJoy()
    hlj.run()
