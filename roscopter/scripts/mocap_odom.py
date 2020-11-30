#!/usr/bin/env python3

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class MocapOdom():
	def __init__(self):
		self.odom = Odometry()

		self.x_prev = 0.0
		self.y_prev = 0.0
		self.z_prev = 0.0
		self.tau = 0.05

		self.first_message = True

		self.mocap_odom_pub_ = rospy.Publisher('mocap_odom', Odometry, queue_size=5, latch=True)		

		self.pose_sub_ = rospy.Subscriber('pose', PoseStamped, self.poseStampedCallback, queue_size=5)

		while not rospy.is_shutdown():
			rospy.spin()


	def poseStampedCallback(self, msg):
		if self.first_message:
			self.first_message = False
		else:
			self.odom.header = msg.header
			dt = self.get_dt()
			self.odom.pose.pose = msg.pose
			u_prev = self.odom.twist.twist.linear.x
			v_prev = self.odom.twist.twist.linear.y
			w_prev = self.odom.twist.twist.linear.z
			self.odom.twist.twist.linear.x = self.calc_velocity(msg.pose.position.x,self.x_prev,u_prev,dt)
			self.odom.twist.twist.linear.y = self.calc_velocity(msg.pose.position.y,self.y_prev,v_prev,dt)
			self.odom.twist.twist.linear.z = self.calc_velocity(msg.pose.position.z,self.z_prev,w_prev,dt)
			self.mocap_odom_pub_.publish(self.odom)

		self.time_prev = msg.header.stamp.secs+msg.header.stamp.nsecs*1E-9
		self.x_prev = msg.pose.position.x
		self.y_prev = msg.pose.position.y
		self.z_prev = msg.pose.position.z

	def get_dt(self):
		time = self.odom.header.stamp.secs+self.odom.header.stamp.nsecs*1E-9
		dt = time - self.time_prev
		self.time_prev = time
		return dt

	def calc_velocity(self,pos,pos_prev,vel_prev,dt):
		dx = pos - pos_prev
		vel = (2.0 * self.tau - dt) / (2.0 * self.tau + dt) * vel_prev + 2.0 / (2.0 * self.tau + dt) * dx
		return vel

if __name__ == '__main__':

	rospy.init_node('mocap_odom', anonymous=True)
	try:
		mocap_odom = MocapOdom()
	except:
		rospy.ROSInterruptException
	pass