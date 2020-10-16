#ifndef VEL_CNTRL_ROS_H
#define VEL_CNTRL_ROS_H

#include <iostream>

#include "controller/vel_cntrl.h"
#include "controller/controller_ros.h"

class Vel_Cntrl_Ros : public Controller_Ros
{
public:
    Vel_Cntrl_Ros();
};

#endif