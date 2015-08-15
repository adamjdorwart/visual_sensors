#!/usr/bin/env python

import roslib
roslib.load_manifest("visual_sensors")
import rospy
import tf
import math

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from dynamic_reconfigure.server import Server as DynamicReconfigureServer

if __name__ == '__main__':
	rospy.init_node('three_rgbd_plate')

	left_rgbd_base_name = rospy.get_param("~left_rgbd_base_name","left_rgbd")
	left_yaw_setting = float(rospy.get_param("~left_yaw_setting",50.0))
	left_pitch_setting = float(rospy.get_param("~left_pitch_setting",0.0))
	left_rgbd_position = rospy.get_param("~left_rgbd_position",(-0.1275,0.23,0.0))
	left_rgbd_rotation = rospy.get_param("~left_rgbd_rotation",(-1.57079633 ,0,-1.57079633))
	left_rgbd_rotation_qt = tf.transformations.quaternion_from_euler(left_rgbd_rotation[0] - math.radians(left_pitch_setting),
																		left_rgbd_rotation[1],
																		left_rgbd_rotation[2] + math.radians(left_yaw_setting))

	center_rgbd_base_name = rospy.get_param("~center_rgbd_base_name","center_rgbd")
	center_pitch_setting = float(rospy.get_param("~center_pitch_setting",0.0))
	center_rgbd_position = rospy.get_param("~center_rgbd_position",(0,0,0))
	center_rgbd_rotation = rospy.get_param("~center_rgbd_rotation",(-1.57079633,0,-1.57079633))
	center_rgbd_rotation_qt = tf.transformations.quaternion_from_euler(center_rgbd_rotation[0] - math.radians(center_pitch_setting),
																		center_rgbd_rotation[1],
																		center_rgbd_rotation[2])

	right_rgbd_base_name = rospy.get_param("~right_rgbd_base_name","right_rgbd")
	right_yaw_setting = float(rospy.get_param("~right_yaw_setting",50.0))
	right_pitch_setting = float(rospy.get_param("~right_pitch_setting",0.0))
	right_rgbd_position = rospy.get_param("~right_rgbd_position",(-0.1275,-0.23,0.0))
	right_rgbd_rotation = rospy.get_param("~right_rgbd_rotation",(-1.57079633 ,0,-1.57079633 ))
	right_rgbd_rotation_qt = tf.transformations.quaternion_from_euler(right_rgbd_rotation[0] - math.radians(right_pitch_setting),
																		right_rgbd_rotation[1],
																		right_rgbd_rotation[2] - math.radians(right_yaw_setting))

	plate_base_name = rospy.get_param("~plate_base_name","three_rgbd_plate")

	left_rgbd_tf = tf.TransformBroadcaster()
	center_rgbd_tf = tf.TransformBroadcaster()
	right_rgbd_tf = tf.TransformBroadcaster()

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		left_rgbd_tf.sendTransform(left_rgbd_position,
			left_rgbd_rotation_qt,
			rospy.Time.now(),
			left_rgbd_base_name + "_link",
			plate_base_name)
		center_rgbd_tf.sendTransform(center_rgbd_position,
			center_rgbd_rotation_qt,
			rospy.Time.now(),
			center_rgbd_base_name + "_link",
			plate_base_name)
		right_rgbd_tf.sendTransform(right_rgbd_position,
			right_rgbd_rotation_qt,
			rospy.Time.now(),
			right_rgbd_base_name + "_link",
			plate_base_name)

		rate.sleep()
