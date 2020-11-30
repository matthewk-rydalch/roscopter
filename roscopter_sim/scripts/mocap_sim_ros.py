#!/usr/bin/env python3

import numpy as np
import rospy
import std_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from rosflight_msgs.msg import GNSS
# from rosflight_msgs.msg import GNSSRaw
from ublox.msg import RelPos
from ublox.msg import PosVelEcef


class MocapSimManager():


    def __init__(self):

        self.antenna_offset = rospy.get_param('~waypoint_manager/antenna_offset', [0.0, 0.0, 0.0])

        self.base_pos = np.zeros(3)
        self.rover_pos = np.zeros(3)
        self.rover_virtual_mocap_ned = PoseStamped()
        self.base_virtual_mocap_ned = PoseStamped()
        self.base_quat = Quaternion()

        self.origin_set = False
        self.origin = np.zeros(3)

        mocap_rate = 100 #hz        

        # # Set Up Publishers and Subscribers
        self.rover_virtual_mocap_ned_pub_ = rospy.Publisher('rover_mocap', PoseStamped, queue_size=5, latch=True)
        self.base_virtual_mocap_ned_pub_ = rospy.Publisher('base_mocap', PoseStamped, queue_size=5, latch=True)
        self.base_odom_sub_ = rospy.Subscriber('platform_odom', Odometry, self.baseOdomCallback, queue_size=5)
        self.rover_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.roverOdomCallback, queue_size=5)
    
        rate = rospy.Rate(mocap_rate)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            # rospy.spin()
            rate.sleep()


    def baseOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        base_pos_no_offset = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
        self.base_quat = msg.pose.pose.orientation
        base_euler = self.quat2euler(self.base_quat)
        yaw_deg = base_euler[2]*180/np.pi
        self.base_pos = base_pos_no_offset - self.Rz(yaw_deg)@np.array(self.antenna_offset)

        self.publish_base_virtual_mocap_ned()


    def roverOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.rover_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
        if self.origin_set == False:
            self.origin = self.rover_pos
            self.origin_set = True

        self.publish_rover_virtual_mocap_ned()


    def publish_rover_virtual_mocap_ned(self):
        rover_virtual_mocap_ned_array = self.rover_pos - self.origin

        self.rover_virtual_mocap_ned.header.stamp = rospy.Time.now()
        self.rover_virtual_mocap_ned.pose.position.x = rover_virtual_mocap_ned_array[0]
        self.rover_virtual_mocap_ned.pose.position.y = rover_virtual_mocap_ned_array[1]
        self.rover_virtual_mocap_ned.pose.position.z = rover_virtual_mocap_ned_array[2]
                
        self.rover_virtual_mocap_ned_pub_.publish(self.rover_virtual_mocap_ned)


    def publish_base_virtual_mocap_ned(self):
        base_virtual_mocap_ned_array = self.base_pos - self.origin
        self.base_virtual_mocap_ned
        self.base_virtual_mocap_ned.header.stamp = rospy.Time.now()
        self.base_virtual_mocap_ned.pose.position.x = base_virtual_mocap_ned_array[0]
        self.base_virtual_mocap_ned.pose.position.y = base_virtual_mocap_ned_array[1]
        self.base_virtual_mocap_ned.pose.position.z = base_virtual_mocap_ned_array[2]
        
        self.base_virtual_mocap_ned.pose.orientation = self.base_quat

        self.base_virtual_mocap_ned_pub_.publish(self.base_virtual_mocap_ned)

    def quat2euler(self, quat):
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (quat.w * quat.x + quat.y * quat.z)
        cosr_cosp = 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x)
        pitch = np.arcsin(sinp)
        if abs(sinp) >= 1:
            pitch = np.pi*np.sign(sinp) / 2.0 # use 90 degrees if out of range

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        euler = [roll, pitch, yaw]

        return euler

    def Rz(self, theta):
        theta = theta*np.pi/180.0
        st = np.sin(theta)
        ct = np.cos(theta)
        rotz = np.array([[ct, st, 0.0], \
                        [-st, ct, 0.0], \
                        [0.0, 0.0, 1.0]])

        return rotz

if __name__ == '__main__':
    rospy.init_node('mocap_sim_manager', anonymous=True)
    try:
        mocap_sim_manager = MocapSimManager()
    except:
        rospy.ROSInterruptException
    pass
