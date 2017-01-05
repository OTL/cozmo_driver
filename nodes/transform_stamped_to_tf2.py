#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TransformBroadcaster(object):
    '''This is hack to avoid tf2_ros on python3
    '''
    def __init__(self):
        self._sub = rospy.Subscriber('transforms', TransformStamped,
                                     self.broadcast_tf)
        self._br = tf2_ros.TransformBroadcaster()

    def broadcast_tf(self, transform_msg):
        self._br.sendTransform(transform_msg)

if __name__ == '__main__':
    rospy.init_node('transform_stamped_to_tf2')
    br = TransformBroadcaster()
    rospy.spin()
