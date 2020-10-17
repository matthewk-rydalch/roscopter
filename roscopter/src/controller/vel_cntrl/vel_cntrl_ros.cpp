#include "controller/vel_cntrl_ros.h"

Vel_Cntrl_Ros::Vel_Cntrl_Ros()
{
    Vel_Cntrl vel_cntrl;

    vel_model_input_sub_ = nh_.subscribe("vel_model_input", 1, &Vel_Cntrl_Ros::velModelCallback, this);
}

void Vel_Cntrl_Ros::velModelCallback(const rosflight_msgs::CommandConstPtr &msg)
{
    int a = 1;
//   if (debug_cmdCallback_)
//     std::cout << "In Controller_Ros::cmdCallback!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";

//   switch(msg->mode)
//   {
//     case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
//       control.xc_.pn = msg->x;
//       control.xc_.pe = msg->y;
//       control.xc_.pd = -msg->F;
//       control.xc_.psi = msg->z;
//       control.control_mode_ = msg->mode;
//       break;
//     case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
//       control.xc_.x_dot = msg->x;
//       control.xc_.y_dot = msg->y;
//       control.xc_.pd = -msg->F;
//       control.xc_.r = msg->z;
//       control.control_mode_ = msg->mode;
//       break;
//     case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
//       control.xc_.ax = msg->x;
//       control.xc_.ay = msg->y;
//       control.xc_.az = msg->F;
//       control.xc_.r = msg->z;
//       control.control_mode_ = msg->mode;
//       break;
//     default:
//       ROS_ERROR("roscopter/controller: Unhandled command message of type %d",
//                 msg->mode);
//       break;
//   }

//   if (!received_cmd_)
//     received_cmd_ = true;
}