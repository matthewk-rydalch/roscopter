#!/usr/bin/env python3

import numpy as np
import rospy

from rosflight_msgs.msg import Command

class HlcContinuousNed():

    def __init__(self):

        self.Ts = 0.1 #seconds
        self.highLevelCommand = Command()

        # Publishers
        self.hlc_continuous_ned_pub_ = rospy.Publisher('hlc_cont_ned', Command, queue_size=5, latch=True)
        
        # Subscribers
        self.hlc_sub_ = rospy.Subscriber('high_level_command', Command, self.hlcCallback, queue_size=5)
        
        # Timer
        self.hlc_rate_timer_ = rospy.Timer(rospy.Duration(self.Ts), self.hlcRateCallback)


        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def hlcCallback(self, msg):
        
        #switch from NEU to NED frame
        self.highLevelCommand.x = msg.x
        self.highLevelCommand.y = msg.y
        self.highLevelCommand.z = -msg.z
    
    def hlcRateCallback(self, event):
        
        self.highLevelCommand.header.stamp = rospy.Time.now()
        self.hlc_continuous_ned_pub_.publish(self.highLevelCommand)

if __name__ == '__main__':
    rospy.init_node('hlcContNed', anonymous=True)
    try:
        hlcContNed = HlcContinuousNed()
    except:
        rospy.ROSInterruptException
    pass
