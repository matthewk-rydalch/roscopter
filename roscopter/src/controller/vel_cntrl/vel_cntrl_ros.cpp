#include "controller/vel_cntrl_ros.h"

Vel_Cntrl_Ros::Vel_Cntrl_Ros()
{
    Vel_Cntrl vel_cntrl;

    vel_model_input_sub_ = nh_.subscribe("vel_model_input", 1, &Vel_Cntrl_Ros::velModelCallback, this);
}

void Vel_Cntrl_Ros::velModelCallback(const rosflight_msgs::CommandConstPtr &msg)
{
    if(msg->mode == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
    {
        v_cmd_.header = msg->header;
        vel_cntrl.xc_.pn = msg->x;
        vel_cntrl.xc_.pe = msg->y;
        vel_cntrl.xc_.psi = msg->z;
        vel_cntrl.xhat_ = control.xhat_;
        vel_cntrl.computeVelocityCommand();
        getVelocityCommand();
        v_cmd_.F = msg->F;
        cmdCallback(v_cmd_);
    }
    else
    {
        cmdCallback(msg);
    }
}

void Vel_Cntrl_Ros::getVelocityCommand()
{
    v_cmd_.mode = vel_cntrl.mode_flag_;
    v_cmd_.x = vel_cntrl.xc_.x_dot;
    v_cmd_.y = vel_cntrl.xc_.y_dot;
    v_cmd_.z = vel_cntrl.xc_.r;
}