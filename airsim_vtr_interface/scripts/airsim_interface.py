#!/usr/bin/env python2

"""
AirSim Interface with VT&R

"""

from __future__ import division, print_function, absolute_import

# Import libraries
import rospy
import numpy as np
import time
import copy
import airsim
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from asrl__messages.msg import TrackingStatus
from dji_osdk_ros.srv import SDKControlAuthority, SDKControlAuthorityResponse


class AirsimInterface(object):
	#Class for AirSim & VT&R Interface
	def __init__(self):

		#Simulate DJI sdk control authority service
		self.control_auth_srv = rospy.Service('/dji_sdk/sdk_control_authority',
			                  SDKControlAuthority, self.set_control_authority)
		

		#Subscribering to dji's control setpoint point 
		self.sub_ctrl = rospy.Subscriber('/dji_sdk/flight_control_setpoint_generic', 
										Joy, self.ctrl_cb)
		#Subscribe to vtr tracking status 
		self.sub_nav = rospy.Subscriber('/ShamNav/out/tracker_status',
                                    TrackingStatus,
                                    self.update_nav_status)

		#Simulate Display mode setting for M600
		self.display_mode_pub = rospy.Publisher('/dji_sdk/display_mode', 
													UInt8, queue_size=30)

		# connect to the AirSim simulator
		self.client = airsim.MultirotorClient()
		self.client.confirmConnection()
		self.client.enableApiControl(True, vehicle_name='drone')
		self.client.armDisarm(True,  vehicle_name='drone')

		#capture vt&r commands
		self.roll = 0
		self.pitch = 0
		self.yaw_rate = 0
		self.z_rate = 0

		#capture vt&R state
		self.state = ""

		#Parameters for authorizing control
		self.control_authorized = False
		self.return_started = False

		#Set Controller Gains
		self.update_anglerate_gains()
		self.update_angle_gains()
		self.update_velocity_gains()
		self.update_position_gains()


	def update_anglerate_gains(self):
		""" Set Angle Rate Gains (roll rate, pitch rate, yaw rate)
		Tuned for DJI M600
		"""
		gains = airsim.AngleRateControllerGains(
						roll_gains = airsim.PIDGains(kp=1.0, ki=0.0, kd=0.0),
                       pitch_gains = airsim.PIDGains(kp=1.0, ki=0.0, kd=0.0),
                       yaw_gains = airsim.PIDGains(kp=10.0, ki=0.0, kd=0.1))

		self.client.setAngleRateControllerGains(gains,vehicle_name="drone")


	def update_angle_gains(self):
		""" Set Angle Level Gains (roll, pitch, yaw)
		Tuned for DJI M600
		"""
		gains = airsim.AngleLevelControllerGains(
						roll_gains = airsim.PIDGains(kp=2.0, ki=10.0, kd=0.1),
                       pitch_gains = airsim.PIDGains(kp=2.0, ki=10.0, kd=0.1),
                       yaw_gains = airsim.PIDGains(kp=2.0, ki=10.0, kd=0.1))

		self.client.setAngleLevelControllerGains(gains,vehicle_name="drone")


	def update_velocity_gains(self):
		""" Set Velocity Level Gains (x, y, z)
		Tuned for DJI M600
		"""
		gains = airsim.VelocityControllerGains(
						x_gains = airsim.PIDGains(kp=0.2, ki=0.2, kd=0.0),
                       y_gains = airsim.PIDGains(kp=0.2, ki=0.2, kd=0.0),
                       z_gains = airsim.PIDGains(kp=0.2, ki=2.0, kd=0.0))

		self.client.setVelocityControllerGains(gains,vehicle_name="drone")


	def update_position_gains(self):
		""" Set Position Level Gains (x, y, z)
		Tuned for DJI M600
		"""
		gains = airsim.PositionControllerGains(
						x_gains = airsim.PIDGains(kp=0.002, ki=0.0, kd=0.001),
                       y_gains = airsim.PIDGains(kp=0.002, ki=0.0, kd=0.001),
                       z_gains = airsim.PIDGains(kp=0.1, ki=0.0, kd=0.1))

		self.client.setPositionControllerGains(gains,vehicle_name="drone")


	def ctrl_cb(self, msg):
		"""Callback function for /dji_sdk/flight_control_setpoint_generic topic
		Sets control inputs
		"""
		self.return_started = True

		self.roll = msg.axes[0]
		self.pitch = msg.axes[1]
		self.z_rate = - msg.axes[2]
		self.yaw_rate = msg.axes[3]

	def update_nav_status(self, msg):
		#callback for navigation status
		self.state = msg.state

	def set_control_authority(self,req):
		""" Function simulates the callback from /dji_sdk/sdk_control_authority 
		service
		sets control_authorized parameter, which authorises control of drone
		"""
		if req.control_enable:
			self.control_authorized = True
		else:
			self.control_authorized = False

		response = SDKControlAuthorityResponse()
		response.result = True
		return response


	def takoff(self):
		"""Move drone to inital position before commanding it
		Intial position is defined in the NED frame
		"""
		x = 0
		y = 0
		z = -10
		speed = 5.0

		self.client.moveToPositionAsync(x, y, z, speed, vehicle_name="drone")
		
		#sleep wait till vehicle takes off and settles at hovers at new position
		rospy.sleep(15)


	def move_drone_position(self, x, y, z, speed):
		"""Move drone to certain position before commanding it
		Position is defined in the NED frame
		"""
		self.client.moveToPositionAsync(x, y, z, speed, vehicle_name="drone")


	def move_drone_velocity(self):
		"""Function that moves commands the drone and moves it in an arc
		The frame of reference used is NED
		"""
		arc_angle = 0.3 #rad
		speed = 1.0 #m/s
		radius = 30.0 #m
		sim_speed = 0.25
		yaw_rate = speed/radius*180/np.pi

		ang = 0.0 #inital angle, updated every iteration

		#Update Freq
		Freq = 50
		rate = rospy.Rate(Freq)

		print('Commanding Drone Using Velocity')

		while not rospy.is_shutdown():
			vx = speed*np.cos(ang) #velocity in x
			vy = speed*np.sin(ang) #velocity in y

			self.client.moveByVelocityAsync(vx, vy, 0, 1/50.0, 
							yaw_mode=airsim.YawMode(True,yaw_rate))

			if ang < arc_angle:
				# increment angle, divide by Freq and multipy by sim speed
				ang += speed/radius/Freq*sim_speed
			else:
				print('Reached end of arc')
				self.client.hoverAsync(vehicle_name="drone")
				break

			rate.sleep()


	def return_follow(self):
		"""Function that controls the drone in the return phase of vt&r
		"""
		print("Initiating Return Phase Control Loop")

		#Set Control Update Freq
		rate = rospy.Rate(50)

		#Control Loop
		while not rospy.is_shutdown():

			#Set diplay mode to allow control from safety monitor
			self.display_mode_pub.publish(17)

			#move according to commands from vt&r
			self.client.moveByRollPitchYawrateZrateAsync(self.roll, 
															self.pitch, 
															self.yaw_rate, 
															self.z_rate, 
															float(1.0/50.0))

			#Hover when reach end of path
			if self.state == "::Hover::MetricLocalize":
				#reached end of return path
				print('Reached End, Hovering')
				self.client.hoverAsync(vehicle_name="drone")
				break

			rate.sleep()

	def run_vtr(self):
		self.takoff()
		#Teach phase: Move according to mission plan (arc in this case)
		self.move_drone_velocity()
		#Repeat phase: Move according to controls from vt&r
		self.return_follow()




if __name__ == '__main__':
    # Initiate airsim interface node
    rospy.init_node('airsim_interface')
    inter = AirsimInterface()
    inter.run_vtr()
    rospy.spin()
