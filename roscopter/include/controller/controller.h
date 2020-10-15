#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <stdint.h>
#include <controller/simple_pid.h>
#include "roscopter_utils/yaml.h"

namespace controller
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;
} state_t;

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double x_dot;
  double y_dot;
  double z_dot;

  double r;

  double ax;
  double ay;
  double az;

  double throttle;
} command_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double n_dot;
  double e_dot;
  double d_dot;
} max_t;

class Controller
{
//TODO: determine what should be private and protected
public:

  Controller();
  void load(const std::string &filename);
  void setPIDXDot(double P, double I, double D, double tau);
  void setPIDYDot(double P, double I, double D, double tau);
  void setPIDZDot(double P, double I, double D, double tau);
  void setPIDN(double P, double I, double D, double tau);
  void setPIDE(double P, double I, double D, double tau);
  void setPIDD(double P, double I, double D, double tau);
  void setPIDPsi(double P, double I, double D, double tau);

  max_t max_ = {};

  // Paramters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  double max_accel_xy_;
  double max_accel_z_;
  double min_altitude_;
  float throttle_down_ = 0.95;
  bool is_flying_;
  bool armed_;
  bool received_cmd_;

// protected:

  uint8_t control_mode_;
  uint8_t mode_flag_;

// private:

  // Node handles, publishers, subscribers
  // ros::NodeHandle nh_;
  // ros::NodeHandle nh_private_;

  // PID Controllers
  controller::SimplePID PID_x_dot_;
  controller::SimplePID PID_y_dot_;
  controller::SimplePID PID_z_dot_;
  controller::SimplePID PID_n_;
  controller::SimplePID PID_e_;
  controller::SimplePID PID_d_;
  controller::SimplePID PID_psi_;

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  command_t xc_ = {}; // command

  // Functions
  void computeControl(double dt);
  void resetIntegrators();
  double saturate(double x, double max, double min);

  uint8_t MODE_PASS_THROUGH_;
  uint8_t MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE_;
  uint8_t MODE_ROLL_PITCH_YAWRATE_THROTTLE_;
  uint8_t MODE_ROLL_PITCH_YAWRATE_ALTITUDE_;
  uint8_t MODE_XPOS_YPOS_YAW_ALTITUDE_;
  uint8_t MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
  uint8_t MODE_XACC_YACC_YAWRATE_AZ_;
};
} //namespace controller

#endif
