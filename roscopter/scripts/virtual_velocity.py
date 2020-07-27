#!/usr/bin/env python

import numpy as np
import rospy

from ublox.msg import PosVelEcef


class VirtualVelocity():


	def __init__(self):


		#messages
		self.virtual_posvelecef = PosVelEcef()


		#history
		self.prev_pos = np.zeros(3)
		self.prev_time = 0.0
		self.prev_vel = np.zeros(3)

		#flags
		self.first_message = True

		#parameters
		self.sigma = 1.0

		# Publishers
		self.virtual_posvelecef_pub_ = rospy.Publisher('virtual_PosVelEcef', PosVelEcef, queue_size=5, latch=True)		

		#subscribers
		# self.posvelecef_sub_ = rospy.Subscriber('PosVelEcef', PosVelEcef, self.pveCallback, queue_size=5)	
		self.sub_ = rospy.Subscriber('PosVelEcef', PosVelEcef, self.pveCallback, queue_size=5)
		print('after subscriber')

		while not rospy.is_shutdown():
			# wait for new messages and call the callback when they arrive
			rospy.spin()


	def pveCallback(self, msg):

		self.virtual_posvelecef = msg

		position, time = self.get_position_time(msg.position, msg.header.stamp)
		self.virtual_posvelecef.velocity = self.calc_velocity(position, time)

		self.virtual_posvelecef_pub_.publish(self.virtual_posvelecef)


	def get_position_time(self, pos, time_stamp):

		pos_array = np.array(pos)

		time = time_stamp.secs+time_stamp.nsecs*1E-9

		return(pos_array, time)


	def calc_velocity(self, new_pos, new_time):

		if self.first_message:
			self.prev_pos = new_pos
			self.prev_time = new_time
			self.first_message = False
			return [0.0, 0.0, 0.0]

		dx = new_pos - self.prev_pos
		dt = new_time - self.prev_time
		new_vel = dx/dt
		new_vel_lpf = self.lpf(new_vel, self.prev_vel, dt, self.sigma)

		self.prev_pos = new_pos
		self.prev_time = new_time
		self.prev_vel = new_vel_lpf

		return new_vel_lpf


	def lpf(self, xt, x_prev, dt, sigma):

		#low pass filter
		x_lpf = xt*dt/(sigma+dt) + x_prev*sigma/(sigma+dt)
		return x_lpf



if __name__ == '__main__':

	rospy.init_node('virtual_velocity', anonymous=True)
	try:
		virtual_velocity = VirtualVelocity()
	except:
		rospy.ROSInterruptException
	pass