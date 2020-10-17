#include "controller/vel_cntrl_ros.h"

Vel_Cntrl_Ros::Vel_Cntrl_Ros()
{
    Vel_Cntrl vel_cntrl;

    vel_model_input_sub_ = nh_.subscribe("vel_model_input", 1, &Vel_Cntrl_Ros::velModelCallback, this);
}

// void Vel_Cntrl_Ros::velModelCallback(const rosflight_msgs::CommandConstPtr &msg)
// {
//     std::cout << "11111111111111111111111111111 \n";
//     if(msg->mode == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
//     {
//         std::cout << "msg->header = " << msg->header << std::endl;
//         v_cmd_ = new rosflight_msgs::Command;
//         v_cmd_->header = msg->header;
//         std::cout << "3333333333333333333333333333 \n";
//         vel_cntrl.xc_.pn = msg->x;
//         vel_cntrl.xc_.pe = msg->y;
//         vel_cntrl.xc_.psi = msg->z;
//         std::cout << "44444444444444444444444444444 \n";
//         vel_cntrl.xhat_ = control.xhat_;
//         std::cout << "55555555555555555555555555555 \n";
//         vel_cntrl.computeVelocityCommand();
//         std::cout << "66666666666666666666666666666 \n";
//         getVelocityCommand();
//         std::cout << "77777777777777777777777777 \n";
//         v_cmd_->F = msg->F;
//         std::cout << "v_cmd_ pointer = " << v_cmd_ << std::endl;
//         std::cout << "v_cmd_ = " << *v_cmd_ << std::endl; 
//         // cmdCallback(v_cmd_);
//     }
//     else
//     {
//         cmdCallback(msg);
//     }
// }

void Vel_Cntrl_Ros::velModelCallback(const rosflight_msgs::CommandConstPtr &msg)
{
  switch(msg->mode)
  {
    case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      v_cmd_ = new rosflight_msgs::Command;
      v_cmd_->header = msg->header;
      vel_cntrl.xc_.pn = msg->x;
      vel_cntrl.xc_.pe = msg->y;
      vel_cntrl.xc_.psi = msg->z;
      vel_cntrl.xhat_ = control.xhat_;
      vel_cntrl.computeVelocityCommand();
      getVelocityCommand();
      v_cmd_->F = msg->F;

      control.xc_.x_dot = v_cmd_->x;
      control.xc_.y_dot = v_cmd_->y;
      control.xc_.pd = -v_cmd_->F;
      control.xc_.r = v_cmd_->z;
      control.control_mode_ = v_cmd_->mode;
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

void Vel_Cntrl_Ros::getVelocityCommand()
{
    v_cmd_->mode = vel_cntrl.mode_flag_;
    v_cmd_->x = vel_cntrl.xc_.x_dot;
    v_cmd_->y = vel_cntrl.xc_.y_dot;
    v_cmd_->z = vel_cntrl.xc_.r;
}