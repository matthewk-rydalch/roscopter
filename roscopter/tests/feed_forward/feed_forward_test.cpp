#include "tests/feed_forward_test.h"

TEST(ff_test, stationaryBaseWNoError)
{
    int argc;
    char** argv;
    ros::init(argc, argv, "ff_test");
    ros::NodeHandle nh;
    controller::Controller cntrl;

    setup_control_object(cntrl);
    
    double tolerance{0.001};
    double dt = 0.001;
    double expected_x_dot{0.0};

    cntrl.computeControl(dt);

    EXPECT_NEAR(cntrl.xc_.x_dot,expected_x_dot,tolerance);
}


void setup_control_object(controller::Controller &cntrl)
{
    cntrl.control_mode_ = rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE;
    cntrl.xc_.pn = 0.0;
    cntrl.xc_.pe = 0.0;
    cntrl.xc_.pd = 0.0;
    cntrl.xc_.psi = 0.0;
    cntrl.xhat_.pn = 0.0;
    cntrl.xhat_.pe = 0.0;
    cntrl.xhat_.pd = 0.0;
    cntrl.xhat_.phi = 0.0;
    cntrl.xhat_.theta = 0.0;
    cntrl.xhat_.psi = 0.0;
    cntrl.xhat_.u = 0.0;
    cntrl.xhat_.v = 0.0;
    cntrl.xhat_.w = 0.0;
    cntrl.use_feed_forward_ = false;
    cntrl.landed_ = false;
    cntrl.is_landing_ = false;
    cntrl.max_.roll = 0.0;
    cntrl.max_.pitch = 0.0;
    cntrl.max_.yaw_rate = 0.0;
    cntrl.base_hat_.u = 0.0;
    cntrl.base_hat_.v = 0.0;
    cntrl.base_hat_.w = 0.0;
    cntrl.throttle_eq_ = 1.0;
    cntrl.throttle_down_ = 0.99;
    cntrl.max_.throttle = 2.0;
    cntrl.min_altitude_ = 0.0;
}
//       state_t xhat_ = {}; // estimate
//   state_t base_hat_ = {}; //base(frame) state
//   max_t max_ = {};
//   rosflight_msgs::Command command_;
//   command_t xc_ = {}; // command
//   double prev_time_;
//   uint8_t control_mode_;

//   double pn;
//   double pe;
//   double pd;

//   double phi;
//   double theta;
//   double psi;

//   double u;
//   double v;
//   double w;

//   double p;
//   double q;
//   double r;



//   double pn;
//   double pe;
//   double pd;

//   double phi;
//   double theta;
//   double psi;

//   double x_dot;
//   double y_dot;
//   double z_dot;

//   double r;

//   double ax;
//   double ay;
//   double az;

//   double throttle;



//   double roll;
//   double pitch;
//   double yaw_rate;
//   double throttle;
//   double n_dot;
//   double e_dot;
//   double d_dot;

// }