#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <iostream>
#include <controller/controller.h>

class Controller_Ros
{

public:

  Controller_Ros();

private:

  controller::Controller control;
  
};

#endif
