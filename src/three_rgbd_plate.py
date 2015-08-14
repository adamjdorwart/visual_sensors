#!/usr/bin/env python

import roslib
roslib.load_manifest("visual_sensors")
import rospy
import tf

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from dynamic_reconfigure.server import Server as DynamicReconfigureServer

if __name__ == '__main__':
	rospy.init_node('three_rgbd_plate')

	left_rgbd_serial = rospy.get_param("~left_rgbd_serial_id","")
	left_rgbd_base_name = rospy.get_param("~left_rgbd_base_name","left_rgbd")
	left_rgbd_calibration = rospy.get_param("~left_rgbd_calibration","")
	left_rgbd_position = rospy.get_param("~left_rgbd_position",(-0.1275,0.2275,0.0))
	left_rgbd_rotation = rospy.get_param("~left_rgbd_rotation",(-1.57079633 ,0,-1.57079633))
	left_rgbd_rotation_qt = tf.transformations.quaternion_from_euler(left_rgbd_rotation[0],
																		left_rgbd_rotation[1],
																		left_rgbd_rotation[2])

	center_rgbd_serial = rospy.get_param("~center_rgbd_serial_id","")
	center_rgbd_base_name = rospy.get_param("~center_rgbd_base_name","center_rgbd")
	center_rgbd_calibration = rospy.get_param("~center_rgbd_calibration","")
	center_rgbd_position = rospy.get_param("~center_rgbd_position",(0.0,0.0,0.0))
	center_rgbd_rotation = rospy.get_param("~center_rgbd_rotation",(-0.872664626,0.0,-1.57079633))
	center_rgbd_rotation_qt = tf.transformations.quaternion_from_euler(center_rgbd_rotation[0],
																		center_rgbd_rotation[1],
																		center_rgbd_rotation[2])

	right_rgbd_serial = rospy.get_param("~right_rgbd_serial_id","")
	right_rgbd_base_name = rospy.get_param("~right_rgbd_base_name","right_rgbd")
	right_rgbd_calibration = rospy.get_param("~right_rgbd_calibration","")
	right_rgbd_position = rospy.get_param("~right_rgbd_position",(-0.1275,-0.2275,0.0))
	right_rgbd_rotation = rospy.get_param("~right_rgbd_rotation",(-2.26892803,0.0,-1.57079633))
	right_rgbd_rotation_qt = tf.transformations.quaternion_from_euler(right_rgbd_rotation[0],
																		right_rgbd_rotation[1],
																		right_rgbd_rotation[2])

	plate_base_name = rospy.get_param("~plate_base_name","three_rgbd_plate")

	left_rgbd_tf = tf.TransformBroadcaster()
	center_rgbd_tf = tf.TransformBroadcaster()
	right_rgbd_tf = tf.TransformBroadcaster()

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		left_rgbd_tf.sendTransform(left_rgbd_position,
			left_rgbd_rotation_qt,
			rospy.Time.now(),
			plate_base_name,
			left_rgbd_base_name + "_link")
		center_rgbd_tf.sendTransform(center_rgbd_position,
			center_rgbd_rotation_qt,
			rospy.Time.now(),
			plate_base_name,
			center_rgbd_base_name + "_link")
		right_rgbd_tf.sendTransform(right_rgbd_position,
			right_rgbd_rotation_qt,
			rospy.Time.now(),
			plate_base_name,
			right_rgbd_base_name + "_link")

		rate.sleep()
