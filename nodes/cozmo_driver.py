#!/usr/bin/env python3

import sys
import time

import cozmo
from cozmo.util import radians

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState


class CozmoRos(object):
    '''Cozmo ros node
    '''
    def __init__(self, coz):
        self._cozmo = coz
        self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback)
        self._say_sub = rospy.Subscriber('say', String, self._say_callback)
        self._head_sub = rospy.Subscriber('head_angle', Float64, self._move_head,
                                          queue_size=1)
        self._lift_sub = rospy.Subscriber('lift_height', Float64, self._move_lift,
                                          queue_size=1)
        # TODO: better to use array (length=5)
        self._backpack_led_sub = rospy.Subscriber(
            'backpack_led', ColorRGBA, self._set_backpack_led, queue_size=1)
        self._image_pub = rospy.Publisher('image', Image, queue_size=100)
        self._joint_state_pub = rospy.Publisher('joint_states', JointState,
                                                queue_size=100)
        self._imu_pub = rospy.Publisher('imu', Imu, queue_size=100)
        self._battery_pub = rospy.Publisher('battery', BatteryState, queue_size=100)
        self._tf_br = rospy.Publisher('transforms', TransformStamped,
                                      queue_size=100)

    def _move_head(self, cmd):
        action = self._cozmo.set_head_angle(radians(cmd.data), duration=0.1,
                                            in_parallel=True)
        action.wait_for_completed()

    def _move_lift(self, cmd):
        action = self._cozmo.set_lift_height(height=cmd.data * 10, duration=0.2,
                                             in_parallel=True)
        action.wait_for_completed()

    def _set_backpack_led(self, msg):
        light = cozmo.lights.Light(cozmo.lights.Color(
            rgba=[int(x * 255) for x in [msg.r, msg.g, msg.b, msg.a]]),
                                   on_period_ms=1)
        self._cozmo.set_all_backpack_lights(light)

    def _twist_callback(self, cmd):
        # TODO: convert m/s to motor speed actually
        lv = (cmd.linear.x + cmd.angular.z) * 100.0
        rv = (cmd.linear.x - cmd.angular.z) * 100.0
        self._cozmo.drive_wheels(lv, rv)

    def _say_callback(self, msg):
        self._cozmo.say_text(msg.data).wait_for_completed()

    def _publish_objects(self):
        def convert_pose_to_tf_msg(pose):
            transform_msg = Transform()
            transform_msg.translation.x = pose.position.x * 0.001
            transform_msg.translation.y = pose.position.y * 0.001
            transform_msg.translation.z = pose.position.z * 0.001
            transform_msg.rotation.w = pose.rotation.q0
            transform_msg.rotation.x = pose.rotation.q1
            transform_msg.rotation.y = pose.rotation.q2
            transform_msg.rotation.z = pose.rotation.q3
            return transform_msg

        def convert_pose_to_ros_msg(pose, name):
            # TODO: use TransformStamped with tf2
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = 'map'
            transform_msg.child_frame_id = name
            transform_msg.transform = convert_pose_to_tf_msg(pose)
            return transform_msg

        for obj in self._cozmo.world.visible_objects:
            self._tf_br.publish(
                convert_pose_to_ros_msg(obj.pose, 'cube_' + str(obj.object_id)))
        self._tf_br.publish(convert_pose_to_ros_msg(self._cozmo.pose, 'cozmo'))

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
#            ros_img.header.stamp = rospy.Time.from_sec(cozmo_time)
            ros_img.header.stamp = rospy.Time.now()
            self._image_pub.publish(ros_img)

    def _publish_joint_state(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'cozmo'
        js.name = ['head', 'lift']
        js.position = [self._cozmo.head_angle.radians,
                       self._cozmo.lift_height.distance_mm * 0.001]
        js.velocity = [0.0, 0.0]
        js.effort = [0.0, 0.0]
        self._joint_state_pub.publish(js)

    def _publish_imu(self):
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'cozmo'
        imu.orientation.w = self._cozmo.pose.rotation.q0
        imu.orientation.x = self._cozmo.pose.rotation.q1
        imu.orientation.y = self._cozmo.pose.rotation.q2
        imu.orientation.z = self._cozmo.pose.rotation.q3
        imu.angular_velocity.x = self._cozmo.gyro.x
        imu.angular_velocity.y = self._cozmo.gyro.y
        imu.angular_velocity.z = self._cozmo.gyro.z
        imu.linear_acceleration.x = self._cozmo.accelerometer.x * 0.001
        imu.linear_acceleration.y = self._cozmo.accelerometer.y * 0.001
        imu.linear_acceleration.z = self._cozmo.accelerometer.z * 0.001
        self._imu_pub.publish(imu)

    def _publish_battery(self):
        battery = BatteryState()
        battery.header.stamp = rospy.Time.now()
        battery.voltage = self._cozmo.battery_voltage
        battery.present = True
        if self._cozmo.is_on_charger: # is_charging always return False
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        self._battery_pub.publish(battery)

    def spin(self, spin_hz=10.0):
        r = rospy.Rate(spin_hz)
        while not rospy.is_shutdown():
            self._publish_image()
            self._publish_objects()
            self._publish_joint_state()
            self._publish_imu()
            self._publish_battery()
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
