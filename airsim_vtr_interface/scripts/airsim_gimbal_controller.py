#!/usr/bin/env python2

"""
ROS Node for controlling gimbal in airsim
"""

from __future__ import division, print_function, absolute_import

# Import libraries
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from dji_osdk_ros.srv import Activation, ActivationResponse
from dji_osdk_ros.msg import Gimbal
import tf
import airsim

class GimbalPub(object):
	def __init__(self):

		#Simulate Publishing the dji_sdk gimbal angle, and drone attitude
		self.gimbal_angle_pub = rospy.Publisher('/dji_sdk/gimbal_angle', 
				Vector3Stamped, queue_size=32)
		self.vehicle_attitude_pub = rospy.Publisher('/dji_sdk/attitude', 
				QuaternionStamped, queue_size=32)

		#Simulate activation service for dji sdk
		self.activation_srv = rospy.Service('/dji_sdk/activation',Activation, 
											self.activation_cb)

		#Subscribe to gimbal angle cmd topic
		self.gimbal_cmd_sub = rospy.Subscriber('/dji_sdk/gimbal_angle_cmd', 
										Gimbal, self.gimbal_angle_cmd_cb)

		#in rads, closeness threhold between cmd and actual
		self.angle_threshold = rospy.get_param("/gimbal_pub/gimbal_angle_thres")
		#gimbal motion speed in rad/s
		self.angle_speed = rospy.get_param("/gimbal_pub/gimbal_angle_speed")

		# connect to the AirSim simulator
		self.client = airsim.MultirotorClient()
		self.client.confirmConnection()
		self.client.enableApiControl(True, vehicle_name='drone')
		self.client.armDisarm(True,  vehicle_name='drone')

		#Variables to store actual gimbal position
		self.pitch = 0.0
		self.roll = 0.0
		self.yaw = 0.0

		#variables to store gimbal angle commands
		self.pitch_cmd = 0.52
		self.roll_cmd = 0.0
		self.yaw_cmd = 0.0

		#All fixed transforms for ronin gimbal
		self.T_leftcamFRD_sensorFRD = np.identity(4)
		self.T_leftcamFRD_sensorFRD[0:3,3] = np.array([0.0, -0.06, 0.0])

		self.T_rightcamFRD_sensorFRD = np.identity(4)
		self.T_rightcamFRD_sensorFRD[0:3,3] = np.array([0.0, 0.06, 0.0])

		self.T_sensor_pitch = np.identity(4)
		self.T_sensor_pitch[0:3,3] = np.array([0.05, -0.06, -0.02])

		self.T_link_control = np.identity(4)
		self.T_link_control[0:3,3] = np.array([0.0, 0.0, -0.04])

		self.T_rot = np.array([[1, 0, 0 ,0],
								  [0,-1, 0, 0],
								  [0, 0,-1, 0],
								  [0, 0, 0, 1]])


	def activation_cb(self,msg):
		"""Simulate callback for dji sdk activation
		"""
		resp = ActivationResponse()
		resp.result = True
		return resp

	def gimbal_angle_cmd_cb(self, msg):
		"""Calback for gimbal angle cmd topic and store commands
		"""
		self.pitch_cmd = -msg.pitch
		self.roll_cmd = msg.roll
		self.yaw_cmd = -msg.yaw

	def publish_vehicle_attitude(self):
		"""Function for publishing the vehicle attitude
		"""
		T_rot = np.array([[1, 0, 0 ,0],
								  [0,-1, 0, 0],
								  [0, 0,-1, 0],
								  [0, 0, 0, 1]])

		#get pose from airsim relative (NED -> FRD)
		pose_NED_FRD = self.client.simGetVehiclePose(vehicle_name='drone')

		#extract quaternion components
		w = pose_NED_FRD.orientation.w_val
		x = pose_NED_FRD.orientation.x_val
		y = pose_NED_FRD.orientation.y_val
		z = pose_NED_FRD.orientation.z_val

		#extract rotation matrix
		R_NED_FRD = tf.transformations.quaternion_matrix([x,y,z,w])

		#conver rotation to (NWU -> FLU)
		R_NWU_FLU = T_rot.dot(R_NED_FRD).dot(T_rot)

		#convert to quaternion
		q_NWU_FLU = tf.transformations.quaternion_from_matrix(R_NWU_FLU)

		#publish attitude
		msg = QuaternionStamped()
		msg.quaternion.x = q_NWU_FLU[0]
		msg.quaternion.y = q_NWU_FLU[1]
		msg.quaternion.z = q_NWU_FLU[2]
		msg.quaternion.w = q_NWU_FLU[3]
		self.vehicle_attitude_pub.publish(msg)

		#get euler angles
		euler_angles = tf.transformations.euler_from_matrix(R_NWU_FLU,'ryxz')

		return euler_angles[0], euler_angles[1], euler_angles[2]


	def gimbal_controller(self):
		"""Function for controlling gimbal in airsim
		"""

		#Control update rate
		Freq = 10
		rate = rospy.Rate(Freq)

		#Control loop
		while not rospy.is_shutdown():

			#increment roll pitch yaw
			if abs(self.pitch - self.pitch_cmd) > self.angle_threshold:
				self.pitch += self.angle_speed/Freq * \
				(self.pitch_cmd - self.pitch)/abs(self.pitch - self.pitch_cmd)

			if abs(self.roll - self.roll_cmd) > 0.02:
				self.roll += self.angle_speed/Freq * \
				(self.roll_cmd - self.roll)/abs(self.roll - self.roll_cmd)

			if abs(self.yaw - self.yaw_cmd) > 0.02:
				self.yaw += self.angle_speed/Freq * \
				   (self.yaw_cmd - self.yaw)/abs(self.yaw - self.yaw_cmd)

			#Calculate roll pitch yaw rotation matrices
			T_pitch_roll = np.array([[np.cos(self.pitch),0,np.sin(self.pitch),0.16],
									 [0,1,0,0.11],
									 [-np.sin(self.pitch),0, np.cos(self.pitch),0],
									 [0,0,0,1]])

			T_roll_yaw = np.array([[1,0,0,-0.087],
								   [0,np.cos(self.roll),-np.sin(self.roll),0],
								   [0,np.sin(self.roll),np.cos(self.roll),-0.2],
								   [0,0,0,1]])

			T_yaw_link = np.array([[np.cos(self.yaw), -np.sin(self.yaw), 0, 0],
								   [np.sin(self.yaw), np.cos(self.yaw), 0, 0],
								   [0,0,1,-0.021],
								   [0,0,0,1]])
			
			#Calculate camera poses relative to control frame
			T_sensorFRD_ctrlFRD = self.T_rot.dot(self.T_link_control) \
					.dot(T_yaw_link).dot(T_roll_yaw).dot(T_pitch_roll) \
					.dot(self.T_sensor_pitch).dot(self.T_rot)
			T_leftcamFRD_ctrlFRD = T_sensorFRD_ctrlFRD \
									.dot(self.T_leftcamFRD_sensorFRD)
			T_rightcamFRD_ctrlFRD = T_sensorFRD_ctrlFRD \
									.dot(self.T_rightcamFRD_sensorFRD)

			#Convert rotations to euler
			left_euler = tf.transformations.euler_from_matrix(T_leftcamFRD_ctrlFRD,'ryxz')
			left_trans = T_leftcamFRD_ctrlFRD[0:3,3]
			right_euler = tf.transformations.euler_from_matrix(T_rightcamFRD_ctrlFRD,'ryxz')
			right_trans = T_rightcamFRD_ctrlFRD[0:3,3]

			#Send camera poses to airsim
			camera_pose = airsim.Pose(airsim.Vector3r(left_trans[0], 
													left_trans[1],
													left_trans[2]),
											airsim.to_quaternion(left_euler[0],
																left_euler[1] , 
																left_euler[2]))
			self.client.simSetCameraPose("front_left_custom", camera_pose)
			camera_pose = airsim.Pose(airsim.Vector3r(right_trans[0], 
													right_trans[1],
													right_trans[2]), 
											airsim.to_quaternion(right_euler[0],
																right_euler[1] , 
																right_euler[2]))
			self.client.simSetCameraPose("front_right_custom", camera_pose)

			#publish vehicle attitude
			vehicle_roll, vehicle_pitch, vehicle_yaw = self.publish_vehicle_attitude()

			#Publish gimbal state add vehicle pitch and roll to simulate ronin gimbal output
			gimbal_angle_msg = Vector3Stamped()
			gimbal_angle_msg.vector.z = -self.yaw*180/np.pi
			gimbal_angle_msg.vector.y = (self.roll + vehicle_roll)*180/np.pi
			gimbal_angle_msg.vector.x = (-self.pitch + vehicle_pitch)*180/np.pi

			self.gimbal_angle_pub.publish(gimbal_angle_msg)

			

			rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gimbal_pub')
    s =  GimbalPub()
    s.gimbal_controller()
    rospy.spin()



