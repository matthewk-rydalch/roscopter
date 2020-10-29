#!/usr/bin/env python3

import numpy as np
import rospy
# from IPython.core.debugger import set_trace


from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.msg import RelativePose
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped


class WaypointManager():
    def __init__(self):

        #load parameters
        self.load_set_parameters()

        # Publishers
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.use_feed_forward_pub_ = rospy.Publisher('use_base_feed_forward_vel', Bool, queue_size=5, latch=True)
        self.is_landing_pub_ = rospy.Publisher('is_landing', Bool, queue_size=5, latch=True)
        self.landed_pub_ = rospy.Publisher('landed', Bool, queue_size=5, latch=True)
        self.error_pub_ = rospy.Publisher('error', Pose, queue_size=5, latch=True)

        #Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.plt_relPos_sub_ = rospy.Subscriber('base_relative_pos', PointStamped, self.pltRelPosCallback, queue_size=5)
        self.base_odom_sub_ = rospy.Subscriber('base_odom', Odometry, self.baseOdomCallback, queue_size=5)
        
        # Wait a second before we publish the first waypoint
        while (rospy.Time.now() < rospy.Time(2.)):
            pass

        # publish first waypoint
        current_waypoint = np.array(self.waypoint_list[0])
        self.new_waypoint(current_waypoint)

        while not rospy.is_shutdown():
            rospy.spin()

    def odometryCallback(self, msg):
        
        # Get error between waypoint and current state
        self.drone_odom = np.array([msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z])
        #waypoints are in neu
        current_position_neu = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
 
        if self.mission_state == 1:
            self.rendevous(current_position_neu)
        elif self.mission_state == 2:
            self.center(current_position_neu)
        elif self.mission_state == 3 or self.mission_state == 4:
            self.descend(current_position_neu)
        elif self.mission_state == 10:
            self.land(current_position_neu)
        else:
            self.mission(current_position_neu)   


    def pltRelPosCallback(self, msg):
        #TODO: implement time for the plt_relPos message?

        antenna_offset = np.matmul(self.Rz(self.base_orient[2]), self.antenna_offset)
                
        #flip to NEU and add antenna offset
        # self.plt_pos[0] = msg.point.x + self.drone_odom[0] - antenna_offset[0]
        # self.plt_pos[1] = msg.point.y + self.drone_odom[1] - antenna_offset[1]
        # self.plt_pos[2] = -msg.point.z - self.drone_odom[2] + antenna_offset[2]   
        

    def baseOdomCallback(self, msg):
        self.plt_pos[0] = msg.pose.pose.position.x
        self.plt_pos[1] = -msg.pose.pose.position.y
        self.plt_pos[2] = msg.pose.pose.position.z

        # yaw from quaternion
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        [roll, pitch, yaw] = self.get_euler(qw, qx, qy, qz)

        self.base_orient[0] = roll
        self.base_orient[1] = pitch
        self.base_orient[2] = yaw + self.base_yaw0  


    #this function works in ipython
    def get_euler(self, qw, qx, qy, qz):                       
        
        euler = np.array([np.arctan2(2.0*(qw*qx+qy*qz),1.0-2.0*(qx**2+qy**2)),
                          np.arcsin(2.0*(qw*qy-qz*qx)),
                          np.arctan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy**2+qz**2))])                                                       
        
        return euler     

    
    def mission(self, current_position):

        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

        self.publish_error(current_position, current_waypoint)
        error = np.linalg.norm(current_position - current_waypoint[0:3])
        
        #may want to implement heading error at some point
        # heading_error = np.abs(self.wrap(current_waypoint[3] - y))

        if error < self.threshold:
            print('reached waypoint ', self.current_waypoint_index + 1)
            # Get new waypoint index
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.waypoint_list) and self.auto_land == True:
                self.mission_state = 1 #switch to rendevous state
                print('rendevous state')
                return

            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)            
            elif self.current_waypoint_index == len(self.waypoint_list):
                self.current_waypoint_index -=1

            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            self.new_waypoint(next_waypoint)

    def rendevous(self, current_position):

        waypoint = self.plt_pos + np.array([0.0, 0.0, self.begin_descent_height])
        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)

        self.new_waypoint(waypoint)

        if error < self.rendevous_threshold:
            self.mission_state = 2 #switch to center state
            print('center state')

    def center(self, current_position):

        self.use_feed_forward_pub_.publish(True) #this will signal a switch to the ff controller

        waypoint = self.plt_pos + np.array([0.0, 0.0, self.begin_descent_height])
        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)

        self.new_waypoint(waypoint)

        if error < self.center_threshold:
            self.mission_state = 3 #switch to descent state
            print('descend state')


    def descend(self, current_position):

        waypoint = self.plt_pos + np.array([0.0, 0.0, self.begin_landing_height])
        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)

        self.new_waypoint(waypoint)

        base_roll = self.base_orient[0]
        base_pitch = self.base_orient[1]

        if error < self.landing_threshold and base_roll < self.landing_orient_threshold and base_pitch < self.landing_orient_threshold:
            self.mission_state = 4 #switch to land state
            self.is_landing_pub_.publish(True)
            # print('land state')


    def land(self, current_position):

        waypoint =self.plt_pos
        if self.is_landing == 0:
            self.new_waypoint(waypoint)
            self.is_landing = 1
            # self.is_landing_pub_.publish(True)

        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)
        alt_error = current_position[2]-waypoint[2]
        if alt_error < self.landing_threshold:
            self.landed_pub_.publish(True)
            #TODO find a way to disarm after reaching the waypoint


    def new_waypoint(self, waypoint):

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.x = waypoint[0]
        self.cmd_msg.y = waypoint[1]
        self.cmd_msg.F = waypoint[2]

        if len(waypoint) > 3:
            self.cmd_msg.z = waypoint[3]
        else:
            self.cmd_msg.z = 0.

        self.cmd_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(self.cmd_msg)

        # rospy.sleep(10)    

    def publish_error(self, current_position, current_waypoint):
        error_msg = Pose()
        error_msg.position.x = current_position[0] - current_waypoint[0]
        error_msg.position.y = current_position[1] - current_waypoint[1]
        error_msg.position.z = current_position[2] - current_waypoint[2]
        self.error_pub_.publish(error_msg)

    
    def Rz(self, theta):

        c_th = np.cos(theta)
        s_th = np.sin(theta)

        Rot_z = np.array([[c_th, -s_th, 0.0],
                          [s_th, c_th, 0.0],
                          [0.0, 0.0, 1.0]])

        return Rot_z

    
    def load_set_parameters(self):
        try:
            self.waypoint_list = rospy.get_param('~waypoints') #params are loaded in launch file
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')
        self.threshold = rospy.get_param('~threshold', 0.5)
        self.landing_threshold = rospy.get_param('~landing_threshold', 0.2)
        self.rendevous_threshold = rospy.get_param('~rendevous_threshold', 0.5)
        self.center_threshold = rospy.get_param('~center_threshold', 0.3)
        landing_orient_threshold = rospy.get_param('~landing_orient_threshold', 10)
        self.landing_orient_threshold = landing_orient_threshold*np.pi/180.0
        self.begin_descent_height = rospy.get_param('~begin_descent_height', 2)
        self.begin_landing_height = rospy.get_param('~begin_landing_height', 0.2)
        self.cyclical_path = rospy.get_param('~cycle', False)
        self.auto_land = rospy.get_param('~auto_land', False)
        self.print_wp_reached = rospy.get_param('~print_wp_reached', True)
        self.antenna_offset = rospy.get_param('~antenna_offset', [0.36, -0.36, -0.12])
        base_yaw0_deg = rospy.get_param('~base_yaw0', 0.0)
        self.base_yaw0 = base_yaw0_deg*np.pi/180.0
        print('waypoints = ', self.waypoint_list)

        #calculate parameters
        self.len_wps = len(self.waypoint_list)
        self.prev_time = rospy.Time.now()

        #other variables and arrays
        self.drone_odom = np.zeros(3)
        self.plt_pos = np.zeros(3)
        self.base_orient = np.zeros(3)
        self.current_waypoint_index = 0
        self.mission_state = 0 #0: mission
                               #1: rendevous
                               #2: descend
                               #3: land
        self.plt_prev_time = 0.0
        self.is_landing = 0
        self.current_waypoint_index = 0

        #message types
        self.cmd_msg = Command()


if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
