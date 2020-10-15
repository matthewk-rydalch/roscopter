#include <controller/controller_ros.h>

Controller_Ros::Controller_Ros() :
  nh_(ros::NodeHandle()), nh_private_("~")
{
  std::cout << "In Controller_Ros \n";


  //sets a variable to the parameter file path and name
  std::string roscopter_path = ros::package::getPath("roscopter");
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ragnarok.yaml");
  control.load(parameter_filename);
}