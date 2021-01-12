#!/usr/bin/env python3

import numpy as np
import rospy
# from IPython.core.debugger import set_trace


from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from geometry_msgs.msg import Pose

class WaypointManager():
    def __init__(self):

        #load parameters
        self.load_set_parameters()

        # Publishers
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.use_feed_forward_pub_ = rospy.Publisher('use_base_feed_forward_vel', Bool, queue_size=5, latch=True)
        self.is_landing_pub_ = rospy.Publisher('is_landing', Bool, queue_size=5, latch=True)
        self.error_pub_ = rospy.Publisher('error', Pose, queue_size=5, latch=True)

        #Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
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
        current_position_neu = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])
 
        if self.mission_state == 1:
            self.rendevous(current_position_neu)
        elif self.mission_state == 2:
            self.descend(current_position_neu)
        elif self.mission_state == 3:
            self.land(current_position_neu)
        else:
            self.mission(current_position_neu)   
        

    def baseOdomCallback(self, msg):
        antenna_offset = np.matmul(self.Rz(self.base_orient[2]), self.antenna_offset)

        self.plt_pos[0] = msg.pose.pose.position.x + antenna_offset[0]
        self.plt_pos[1] = msg.pose.pose.position.y + antenna_offset[1]
        self.plt_pos[2] = msg.pose.pose.position.z + antenna_offset[2]

        # yaw from quaternion
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        [roll, pitch, yaw] = self.get_euler_radians(qw, qx, qy, qz)

        self.base_orient[0] = roll
        self.base_orient[1] = pitch
        self.base_orient[2] = yaw 


    #this function works in ipython
    def get_euler_radians(self, qw, qx, qy, qz):                       
        euler = np.array([np.arctan2(2.0*(qw*qx+qy*qz),1.0-2.0*(qx**2+qy**2)),
                          np.arcsin(2.0*(qw*qy-qz*qx)),
                          np.arctan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy**2+qz**2))])                                                       
        
        return euler     

    
    def mission(self, current_position):

        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

        self.publish_error(current_position, current_waypoint)
        error = np.linalg.norm(current_position - current_waypoint[0:3])

        if error < self.mission_threshold:
            print('reached waypoint ', self.current_waypoint_index + 1)
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.waypoint_list) and self.auto_land == True:
                self.mission_state = 1
                print('rendevous state')
                return

            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)            
            elif self.current_waypoint_index == len(self.waypoint_list):
                self.current_waypoint_index -=1

            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            self.new_waypoint(next_waypoint)

    def rendevous(self, current_position):
        self.use_feed_forward_pub_.publish(True)
        waypoint = self.plt_pos + np.array([0.0, 0.0, self.begin_descent_height])
        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)
        self.new_waypoint(waypoint)

        if error < self.rendevous_threshold:
            self.mission_state = 2
            print('descend state')

    def descend(self, current_position):
        waypoint = self.plt_pos + np.array([0.0, 0.0, self.begin_landing_height])
        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)
        self.new_waypoint(waypoint)

        if error < self.landing_threshold and self.base_orient[0] < self.landing_orient_threshold and self.base_orient[1] < self.landing_orient_threshold:
            self.mission_state = 3
            self.is_landing_pub_.publish(True)
            print('land state')


    def land(self, current_position):

        waypoint = self.plt_pos
        self.new_waypoint(waypoint)
        self.publish_error(current_position, waypoint)

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

        #rospy.sleep(20)    

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
        self.cyclical_path = rospy.get_param('~cycle', False)
        self.auto_land = rospy.get_param('~auto_land', False)
        self.mission_threshold = rospy.get_param('~mission_threshold', 0.5)
        self.rendevous_threshold = rospy.get_param('~rendevous_threshold', 0.5)
        self.landing_threshold = rospy.get_param('~landing_threshold', 0.2)
        landing_orient_threshold_deg = rospy.get_param('~landing_orient_threshold_deg', 10)
        self.landing_orient_threshold = landing_orient_threshold_deg*np.pi/180.0
        self.begin_descent_height = rospy.get_param('~begin_descent_height', 2)
        self.begin_landing_height = rospy.get_param('~begin_landing_height', 0.2)
        self.antenna_offset = rospy.get_param('~antenna_offset', [0.36, -0.36, -0.12])
        print('waypoints = ', self.waypoint_list)

        #other variables and arrays
        self.plt_pos = np.zeros(3)
        self.base_orient = np.zeros(3)
        self.current_waypoint_index = 0
        self.mission_state = 0 #0: mission
                               #1: rendevous
                               #2: descend
                               #3: land
        #message types
        self.cmd_msg = Command()


if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
