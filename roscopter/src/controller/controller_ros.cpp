#include <controller/controller_ros.h>

Controller_Ros::Controller_Ros() :
  nh_(ros::NodeHandle()), nh_private_("~")
{
  std::cout << "In Controller_Ros \n";

  //sets a variable to the parameter file path and name
  std::string roscopter_path = ros::package::getPath("roscopter");
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ragnarok.yaml");
  control.load(parameter_filename);

  _func = boost::bind(&Controller_Ros::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);
}

void Controller_Ros::reconfigure_callback(roscopter::ControllerConfig& config,
                                      uint32_t level)
{
  double P, I, D, tau;
  tau = config.tau;
  P = config.x_dot_P;
  I = config.x_dot_I;
  D = config.x_dot_D;
  control.setPIDXDot(P, I, D, tau);

  P = config.y_dot_P;
  I = config.y_dot_I;
  D = config.y_dot_D;
  control.setPIDYDot(P, I, D, tau);

  P = config.z_dot_P;
  I = config.z_dot_I;
  D = config.z_dot_D;
  control.setPIDZDot(P, I, D, tau);

  P = config.north_P;
  I = config.north_I;
  D = config.north_D;
  control.max_.n_dot = config.max_n_dot;
  control.setPIDN(P, I, D, tau);

  P = config.east_P;
  I = config.east_I;
  D = config.east_D;
  control.max_.e_dot = config.max_e_dot;
  control.setPIDE(P, I, D, tau);

  P = config.down_P;
  I = config.down_I;
  D = config.down_D;
  control.max_.d_dot = config.max_d_dot;
  control.setPIDD(P, I, D, tau);

  P = config.psi_P;
  I = config.psi_I;
  D = config.psi_D;
  control.setPIDPsi(P, I, D, tau);

  control.max_.roll = config.max_roll;
  control.max_.pitch = config.max_pitch;
  control.max_.yaw_rate = config.max_yaw_rate;
  control.max_.throttle = config.max_throttle;

  control.max_.n_dot = config.max_n_dot;
  control.max_.e_dot = config.max_e_dot;
  control.max_.d_dot = config.max_d_dot;

  control.throttle_eq_ = config.equilibrium_throttle;

  ROS_INFO("new gains");

  control.resetIntegrators();
}