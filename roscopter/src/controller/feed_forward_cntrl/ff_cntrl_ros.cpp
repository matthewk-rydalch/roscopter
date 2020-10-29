#include <controller/ff_cntrl_ros.h>

Ff_Cntrl_Ros::Ff_Cntrl_Ros() :
  nh_(ros::NodeHandle()), nh_private_("~")
{
  if (debug_Ff_Cntrl_Ros_)
    std::cout << "In Ff_Cntrl_Ros::Ff_Cntrl_Ros!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";

  init_controller();

  _func = boost::bind(&Ff_Cntrl_Ros::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);

  state_sub_ = nh_.subscribe("estimate", 1, &Ff_Cntrl_Ros::stateCallback, this);
  is_flying_sub_ = nh_.subscribe("is_flying", 1, &Ff_Cntrl_Ros::isFlyingCallback, this);
  cmd_sub_ = nh_.subscribe("waypoint", 1, &Ff_Cntrl_Ros::cmdCallback, this);
  status_sub_ = nh_.subscribe("status", 1, &Ff_Cntrl_Ros::statusCallback, this);

  use_feed_forward_sub_ = nh_.subscribe("use_base_feed_forward_vel", 1, &Ff_Cntrl_Ros::useFeedForwardCallback, this);
  is_landing_sub_ = nh_.subscribe("is_landing", 1, &Ff_Cntrl_Ros::isLandingCallback, this);
  add_integrator_sub_ = nh_.subscribe("add_integrator", 1, &Ff_Cntrl_Ros::addIntegratorCallback, this);
  landed_sub_ = nh_.subscribe("landed", 1, &Ff_Cntrl_Ros::landedCallback, this);
  target_estimate_sub_ = nh_.subscribe("target_estimate", 1, &Ff_Cntrl_Ros::targetEstimateCallback, this);
}

void Ff_Cntrl_Ros::init_controller()
{
  if (debug_init_controller_)
    std::cout << "In Ff_Cntrl_Ros::init_controller!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  control.MODE_PASS_THROUGH_ = rosflight_msgs::Command::MODE_PASS_THROUGH;
  control.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE_ = rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  control.MODE_ROLL_PITCH_YAWRATE_THROTTLE_ = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  control.MODE_ROLL_PITCH_YAWRATE_ALTITUDE_ = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE;
  control.MODE_XPOS_YPOS_YAW_ALTITUDE_ = rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE;
  control.MODE_XVEL_YVEL_YAWRATE_ALTITUDE_ = rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  control.MODE_XACC_YACC_YAWRATE_AZ_ = rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ;
}

void Ff_Cntrl_Ros::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if (debug_stateCallback_)
    std::cout << "In Ff_Cntrl_Ros::stateCallback!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";

  static double prev_time = 0;
  if(prev_time == 0)
  {
    prev_time = msg->header.stamp.toSec();
    return;
  }

  // Calculate time
  double now = msg->header.stamp.toSec();
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0)
    return;

  // This should already be coming in NED
  control.xhat_.pn = msg->pose.pose.position.x;
  control.xhat_.pe = msg->pose.pose.position.y;
  control.xhat_.pd = msg->pose.pose.position.z;

  control.xhat_.u = msg->twist.twist.linear.x;
  control.xhat_.v = msg->twist.twist.linear.y;
  control.xhat_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(control.xhat_.phi, control.xhat_.theta, control.xhat_.psi);

  control.xhat_.p = msg->twist.twist.angular.x;
  control.xhat_.q = msg->twist.twist.angular.y;
  control.xhat_.r = msg->twist.twist.angular.z;
  
  if(is_flying_ && armed_ && received_cmd_)
  {
    ROS_WARN_ONCE("CONTROLLER ACTIVE");
    control.computeFeedForwardControl(dt);
    publishCommand();
  }
  else
  {
    control.resetIntegrators();
    prev_time_ = msg->header.stamp.toSec();
  }
}

void Ff_Cntrl_Ros::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  if (debug_isFlyingCallback_)
    std::cout << "In Ff_Cntrl_Ros::isFlyingCallback!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  is_flying_ = msg->data;
}

void Ff_Cntrl_Ros::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
{
  if (debug_statusCallback_)
    std::cout << "In Ff_Cntrl_Ros::statusCallback!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  armed_ = msg->armed;
}

void Ff_Cntrl_Ros::cmdCallback(const rosflight_msgs::CommandConstPtr &msg)
{
  if (debug_cmdCallback_)
    std::cout << "In Ff_Cntrl_Ros::cmdCallback!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";

  switch(msg->mode)
  {
    case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      control.xc_.pn = msg->x;
      control.xc_.pe = msg->y;
      control.xc_.pd = -msg->F;
      control.xc_.psi = msg->z;
      control.control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
      control.xc_.x_dot = msg->x;
      control.xc_.y_dot = msg->y;
      control.xc_.pd = -msg->F;
      control.xc_.r = msg->z;
      control.control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
      control.xc_.ax = msg->x;
      control.xc_.ay = msg->y;
      control.xc_.az = msg->F;
      control.xc_.r = msg->z;
      control.control_mode_ = msg->mode;
      break;
    default:
      ROS_ERROR("roscopter/controller: Unhandled command message of type %d",
                msg->mode);
      break;
  }

  if (!received_cmd_)
    received_cmd_ = true;
}

void Ff_Cntrl_Ros::reconfigure_callback(roscopter::ControllerConfig& config,
                                      uint32_t level)
{
  if (debug_reconfigure_callback_)
    std::cout << "In Ff_Cntrl_Ros::reconfigure_callback!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  
  control.max_.roll = config.max_roll;
  control.max_.pitch = config.max_pitch;
  control.max_.yaw_rate = config.max_yaw_rate;
  control.max_.throttle = config.max_throttle;

  control.min_altitude_ = config.min_altitude;

  control.throttle_eq_ = config.equilibrium_throttle;

  control.max_accel_z_ = 1.0 / control.throttle_eq_;
  control.max_accel_xy_ = sin(acos(control.throttle_eq_)) / control.throttle_eq_ / sqrt(2.);

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

  ROS_INFO("new gains");

  control.resetIntegrators();
}

void Ff_Cntrl_Ros::targetEstimateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  control.target_hat_.u = msg->twist.twist.linear.x;
  control.target_hat_.v = msg->twist.twist.linear.y;
  control.target_hat_.w = msg->twist.twist.linear.z;
}

void Ff_Cntrl_Ros::useFeedForwardCallback(const std_msgs::BoolConstPtr &msg)
{
  control.use_feed_forward_ = msg->data;
  double Pn_ff_{0.5};
  double In_ff_{0.05};
  double Dn_ff_{0.1};
  double Pe_ff_{0.5};
  double Ie_ff_{0.05};
  double De_ff_{0.1};
  double tau_ff_{0.05};
  double sigma_ff_{2.0};
  control.switchControllers(Pn_ff_,In_ff_,Dn_ff_,Pe_ff_,Ie_ff_,De_ff_,tau_ff_,sigma_ff_);
}

void Ff_Cntrl_Ros::isLandingCallback(const std_msgs::BoolConstPtr &msg)
{
  std::cout << "in is landing callback \n";
  control.is_landing_ = msg->data;
}

void Ff_Cntrl_Ros::addIntegratorCallback(const std_msgs::BoolConstPtr &msg)
{
  control.add_integrator_ = msg->data;
}

void Ff_Cntrl_Ros::landedCallback(const std_msgs::BoolConstPtr &msg)
{
  control.landed_ = msg->data;
}

void Ff_Cntrl_Ros::publishCommand()
{
  if (debug_publishCommand_)
    std::cout << "In Ff_Cntrl_Ros::publishCommand!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  command_.header.stamp = ros::Time::now();
  command_.mode = control.mode_flag_;
  command_.F = control.xc_.throttle;
  command_.x = control.xc_.phi;
  command_.y = control.xc_.theta;
  command_.z = control.xc_.r;
  command_pub_.publish(command_);
}