#!/usr/bin/env python2

"""
ROS Node for republishing images received from airsim with proper stereo 
calibration
"""

from __future__ import division, print_function, absolute_import

# Import libraries
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class StereoPublisher(object):
	def __init__(self):

		#Subscribe to airsim's image topics
		self.left_img_sub = rospy.Subscriber(
			'/airsim_node/drone/front_left_custom/Scene',Image, self.left_img_cb)
		self.right_img_sub = rospy.Subscriber(
			'/airsim_node/drone/front_right_custom/Scene',Image, self.right_img_cb)

		#Subcribe to airsim's camera info topics
		self.left_info_sub = rospy.Subscriber(
			'/airsim_node/drone/front_left_custom/Scene/camera_info',CameraInfo, self.left_info_cb)
		self.right_info_sub = rospy.Subscriber(
			'/airsim_node/drone/front_right_custom/Scene/camera_info',CameraInfo, self.right_info_cb)

		#Publish new camera topics
		self.left_img_pub = rospy.Publisher(
			'/airsim_interface/left_camera_img', Image, queue_size=32)
		self.right_img_pub = rospy.Publisher(
			'/airsim_interface/right_camera_img', Image, queue_size=32)

		#publish new camera info topics
		self.left_info_pub = rospy.Publisher(
			'/airsim_interface/left_camera_info', CameraInfo, queue_size=32)
		self.right_info_pub = rospy.Publisher(
			'/airsim_interface/right_camera_info', CameraInfo, queue_size=32)

		#Place holders
		self.left_info_msg = CameraInfo()
		self.right_info_msg = CameraInfo()
		self.left_img = Image()
		#img sequence
		self.idx = 1
		
	def left_info_cb(self, msg):
		#store left camera info
		self.left_info_msg = msg

	def right_info_cb(self, msg):
		#store right camera info
		self.right_info_msg = msg

	def left_img_cb(self, msg):
		#store left image
		self.left_img = msg

	def right_img_cb(self, msg):
		"""
		Publish All topics once right image is received, and set ZED Camera
		calibration
		"""

		#publish right image
		msg.header.seq = self.idx
		msg.header.stamp = rospy.Time.now()
		self.right_img_pub.publish(msg)

		#publish left image
		self.left_img.header.seq = self.idx
		self.left_img.header.stamp = msg.header.stamp
		self.left_img_pub.publish(self.left_img)
		self.idx += 1

		#publish right cam info
		self.right_info_msg.header.stamp = msg.header.stamp
		self.right_info_msg.D = [0.0, 0.0, 0.0,0.0,0.0]
		self.right_info_msg.R = [1.0, 0.0, 0.0,0.0,1.0, 0.0, 0.0, 0.0, 1.0]
		self.right_info_msg.P = [336.0, 0.0, 336.0, -336.0*0.12, 0.0, 336.0, 
					188.0, 0.0, 0.0, 0.0, 1.0, 0.0]
		self.right_info_pub.publish(self.right_info_msg)

		#publish left cam info
		self.left_info_msg.header.stamp = msg.header.stamp
		self.left_info_msg.D = [0.0, 0.0, 0.0,0.0,0.0]
		self.left_info_msg.R = [1.0, 0.0, 0.0,0.0,1.0, 0.0, 0.0, 0.0, 1.0]
		self.left_info_pub.publish(self.left_info_msg)


if __name__ == '__main__':
    rospy.init_node('airsim_stereo_publisher')
    s =  StereoPublisher()
    rospy.spin()
