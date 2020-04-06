#!/usr/bin/env python

import numpy as np
import rospy
import csv

from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.msg import RelativePose
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile, ListWaypoints, ClearWaypoints, Hold, Release


class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints')
        except KeyError:
            rospy.logfatal('[waypoint_manager] waypoints not set')
            rospy.signal_shutdown('[waypoint_manager] Parameters not set')

        self.current_waypoint_index = 0
        self.y = 0
        self.hold = False
        self.no_command = False
        if len(self.waypoint_list) == 0:
            self.holdPose()
            self.no_command = True
        self.halt_waypoint = [0, 0, 0, 0]

        # how close does the MAV need to get before going to the next waypoint?
        self.pos_threshold = rospy.get_param('~threshold', 5)
        self.heading_threshold = rospy.get_param('~heading_threshold', 0.035)  # radians
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.print_wp_reached = rospy.get_param('~print_wp_reached', True)

        # Set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.removeWaypointCallback)
        self.set_waypoints_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.setWaypointsFromFileCallback)
        self.list_waypoints_service = rospy.Service('list_waypoints', ListWaypoints, self.listWaypointsCallback)
        self.clear_waypoints_service = rospy.Service('clear_waypoints', ClearWaypoints, self.clearWaypointsCallback)
        self.hold_service = rospy.Service('hold', Hold, self.holdCallback)
        self.release_service = rospy.Service('release', Release, self.releaseCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_cmd_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.relPose_pub_ = rospy.Publisher('relative_pose', RelativePose, queue_size=5, latch=True)

        # Wait a second before we publish the first waypoint
        rospy.sleep(2)

        # Create the initial relPose estimate message
        relativePose_msg = RelativePose()
        relativePose_msg.x = 0
        relativePose_msg.y = 0
        relativePose_msg.z = 0
        relativePose_msg.F = 0
        self.relPose_pub_.publish(relativePose_msg)

        # Create the initial command message
        self.cmd_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])
        self.publish_command(current_waypoint)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(self, req):
        # This Function adds a waypoint to the waypoint list at the specified index.
        new_waypoint = [req.x, req.y, req.z, req.psi]
        if req.index == -1:
            index = len(self.waypoint_list)
        elif req.index > len(self.waypoint_list) or req.index < -1:
            rospy.logwarn("[waypoint_manager] Waypoint Index Out of Range")
            return False
        #Valid index
        else:
            index = req.index
        self.waypoint_list.insert(index, new_waypoint)
        if self.current_waypoint_index >= index:
            self.current_waypoint_index += 1
        rospy.loginfo("[waypoint_manager] Added New Waypoint")
        self.no_command = False
        return True

    def removeWaypointCallback(self, req):
        # Remove a waypoint at the requested index
        if req.index >= len(self.waypoint_list):
            rospy.logwarn("[waypoint_manager] Waypoint Index Out of Range")
            return False
        del self.waypoint_list[req.index] # Remove the waypoint
        removed_str = '[waypoint_manager] Waypoint {} Removed'.format(req.index)
        rospy.loginfo(removed_str) # Send loginfo
        # If the current waypoint was removed:
        if req.index == self.current_waypoint_index:
            last_waypoint_bool = self.current_waypoint_index == len(self.waypoint_list)
            if not last_waypoint_bool:
                current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                self.publish_command(current_waypoint)
                return True
            elif self.cyclical_path and len(self.waypoint_list) != 0:
                self.current_waypoint_index = 0
                current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                self.publish_command(current_waypoint)
                return True
            else:  # If the current waypoint was the only waypoint and was removed,
                   # or if it was the last and the list is not cyclical
                self.no_command = True
                return True
        # If the removed waypoint is before the current waypoint
        elif req.index < self.current_waypoint_index:
            self.current_waypoint_index -=1
        return True

    def setWaypointsFromFileCallback(self, req):
        # Sets waypoint list from .csv or .txt file
        if req.Filename.endswith('.csv'):
            with open(req.Filename) as file_wp_list:
                self.waypoint_list = list(csv.reader(file_wp_list, quoting=csv.QUOTE_NONNUMERIC))

        else:
            file = open(req.Filename, 'r')
            file_lines = file.readlines()
            file_wp_list = []
            for waypoint in file_lines:
                file_wp_list.append(map(float, waypoint.strip().split(',')))
            self.waypoint_list = file_wp_list

        # Set index to 0 and publish command to first waypoint
        self.current_waypoint_index = 0
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        self.publish_command(current_waypoint)
        rospy.loginfo("[waypoint_manager] Waypoints Set from File")
        self.no_command = False
        return True

    def listWaypointsCallback(self, req):
        # Returns the waypoint list
        rospy.loginfo('[waypoint_manager] Waypoints:')
        i = 0 # Start index at 0
        for waypoint in self.waypoint_list:
            if i == self.current_waypoint_index:
                waypoint_str = '[waypoint_manager] {}: {} (current_waypoint)'.format(i, waypoint)
            else:
                waypoint_str = '[waypoint_manager] {}: {}'.format(i, waypoint)
            rospy.loginfo(waypoint_str)
            i += 1
        return True

    def clearWaypointsCallback(self, req):
        # Clears all waypoints, except current
        self.no_command = True
        self.waypoint_list = []
        self.current_waypoint_index = 0
        return True

    def holdCallback(self, req):
        # stop and hold current pose
        self.holdPose()
        return True

    def holdPose(self):
        self.hold = True
        rospy.loginfo("[waypoint_manager] Multirotor's pose is held - release to continue")

    def releaseCallback(self, req):
        if len(self.waypoint_list) == 0:
            rospy.loginfo("[waypoint_manager] Cannot release - Zero Waypoints")
            return False
        else:
            self.hold = False
            current_waypoint = self.waypoint_list[self.current_waypoint_index]
            self.publish_command(current_waypoint)
            rospy.loginfo("[waypoint_manager] Released hold - Multirotor path in progress")
            return True

    def odometryCallback(self, msg):
        ###### Retrieve and Publish the Current Pose ######
        # get current position
        current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
        # orientation in quaternion form
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        # yaw from quaternion
        self.y = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        # publish the relative pose estimate
        relativePose_msg = RelativePose()
        relativePose_msg.x = current_pose[0]
        relativePose_msg.y = current_pose[1]
        relativePose_msg.z = self.y
        relativePose_msg.F = current_pose[2]
        self.relPose_pub_.publish(relativePose_msg)

        ###### Hold Pose - if commanded, or if there are zero waypoints ######
        if self.hold:
            self.publish_command(self.halt_waypoint)
            return

        elif

        ###### Check Waypoint Arrival Status & Update to Next Waypoint #######
        else:
            # Calculate error between current pose and commanded waypoint
            self.halt_waypoint = [current_pose[0] , current_pose[1] , current_pose[2] , self.y ]
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

            position_error = np.linalg.norm(current_waypoint[0:3] - current_pose)
            heading_error = np.abs(self.wrap(current_waypoint[3] - self.y))
            #if error is within the threshold
            if position_error < self.pos_threshold and heading_error < self.heading_threshold:
                #Print if we did not already print
                if self.print_wp_reached:
                    idx = self.current_waypoint_index
                    wp_str = '[waypoint_manager] Reached waypoint {}'.format(idx)
                    rospy.loginfo(wp_str)
                # stop iterating over waypoints if cycle==false and last waypoint reached
                if not self.cyclical_path and self.current_waypoint_index == len(self.waypoint_list)-1:
                    self.print_wp_reached = False
                    return
                # Get new waypoint index
                else:
                    # Get new waypoint index
                    self.print_wp_reached = True
                    self.current_waypoint_index += 1
                    self.current_waypoint_index %= len(self.waypoint_list)
                    current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                    self.publish_command(current_waypoint)
                    return

    def publish_command(self, current_waypoint):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.x = current_waypoint[0]
        self.cmd_msg.y = current_waypoint[1]
        self.cmd_msg.F = current_waypoint[2]

        if len(current_waypoint) > 3:
            self.cmd_msg.z = current_waypoint[3]
        else:
            next_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoint_list)
            next_waypoint = self.waypoint_list[next_waypoint_index]
            delta = next_waypoint - current_waypoint
            self.cmd_msg.z = np.arctan2(delta[1], delta[0])

        self.cmd_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_cmd_pub_.publish(self.cmd_msg)

    def wrap(self, angle):
        angle -= 2*np.pi * np.floor((angle + np.pi) / (2*np.pi))
        return angle

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
