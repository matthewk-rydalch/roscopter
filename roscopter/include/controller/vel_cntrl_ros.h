#ifndef VEL_CNTRL_ROS_H
#define VEL_CNTRL_ROS_H

#include <iostream>

#include "controller/vel_cntrl.h"
#include "controller/controller_ros.h"

class Vel_Cntrl_Ros : public Controller_Ros
{
public:
    Vel_Cntrl_Ros();

protected:
    
    Vel_Cntrl vel_cntrl;

    rosflight_msgs::Command v_cmd_;
    
    ros::Subscriber vel_model_input_sub_;
    void velModelCallback(const rosflight_msgs::CommandConstPtr &msg);

    void getVelocityCommand();
};

#endif