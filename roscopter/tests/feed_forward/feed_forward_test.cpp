#include "tests/feed_forward_test.h"

TEST(ff_test, getBoatVelocity)
{
    int argc;
    char** argv;
    ros::init(argc, argv, "ff_test");
    ros::NodeHandle nh;
    controller::Ff_Cntrl cntrl;

    // cntrl.target_hat_.phi = 0.0;
    // cntrl.target_hat_.theta = 0.0;
    // cntrl.target_hat_.psi = 0.0;
    // cntrl.xhat_.psi = 0.0;

    // cntrl.target_hat_.u = 0.0;
    // cntrl.target_hat_.v = 0.0;
    // cntrl.target_hat_.w = 0.0;

    // Eigen::Vector3d base_velocity_rover_v1_frame{cntrl.getBoatVelocity()};
    // std::cout << "boat velocity = " << base_velocity_rover_v1_frame << std::endl;
}

// TEST(ff_test, stationaryBaseWNoError)
// {
//     int argc;
//     char** argv;
//     ros::init(argc, argv, "ff_test");
//     ros::NodeHandle nh;
//     controller::Controller cntrl;

//     setup_control_object(cntrl);
    
//     std::array<double,3> expectedV1Velocities{0.0,0.0,0.0};

//     test_velocities_vs_expected_velocities(cntrl, expectedV1Velocities);
// }

// TEST(ff_test, BaseVelEquals1_NoError_NoHeadingDifference)
// {
//     int argc;
//     char** argv;
//     ros::init(argc, argv, "ff_test");
//     ros::NodeHandle nh;
//     controller::Controller cntrl;

//     setup_control_object(cntrl);
    
//     cntrl.use_feed_forward_ = true;
//     cntrl.base_hat_.u = 1.0;

//     std::array<double,3> expectedV1Velocities{1.0,0.0,0.0};

//     test_velocities_vs_expected_velocities(cntrl, expectedV1Velocities);
// }

// TEST(ff_test, BaseVelEquals1_NoError_BaseMinusRoverHeadingEqualsPiHalves)
// {
//     int argc;
//     char** argv;
//     ros::init(argc, argv, "ff_test");
//     ros::NodeHandle nh;
//     controller::Controller cntrl;

//     setup_control_object(cntrl);
    
//     cntrl.use_feed_forward_ = true;
//     cntrl.base_hat_.u = 1.0;
//     cntrl.base_hat_.psi = M_PI/2.0;

//     std::array<double,3> expectedV1Velocities{0.0,1.0,0.0};

//     test_velocities_vs_expected_velocities(cntrl, expectedV1Velocities);
// }

// TEST(ff_test,variedValuesExpectCorrectVelocityCommand)
// {
//     int argc;
//     char** argv;
//     ros::init(argc, argv, "ff_test");
//     ros::NodeHandle nh;
//     controller::Controller cntrl;

//     setup_control_object(cntrl);
    
//     cntrl.use_feed_forward_ = true;
//     cntrl.xhat_.u = 3.0;
//     cntrl.xhat_.v = -1.9;
//     cntrl.xhat_.w = 0.1;
//     cntrl.xhat_.psi = M_PI;
//     cntrl.base_hat_.u = 2.0;
//     cntrl.base_hat_.v = 1.3;
//     cntrl.base_hat_.w = -0.2;
//     cntrl.base_hat_.psi = M_PI/2.2;
//     cntrl.xc_.pn = 1.2;
//     cntrl.xc_.pe = 0.3;
//     cntrl.xc_.pd = -3.2;
//     cntrl.xc_.psi = 2.0;
//     cntrl.xhat_.pn = 1.0;
//     cntrl.xhat_.pe = 0.5;
//     cntrl.xhat_.pd = -3.1;
//     cntrl.xhat_.phi = 1.8;

//     std::array<double,3> expectedV1Velocities{2.5021, -0.6647, -0.3};

//     test_velocities_vs_expected_velocities(cntrl, expectedV1Velocities);
// }

// // TEST(ff_test, roverChasingFasterThanBoat)
// // {
// //     int argc;
// //     char** argv;
// //     ros::init(argc, argv, "ff_test");
// //     ros::NodeHandle nh;
// //     controller::Controller cntrl;

// //     setup_control_object(cntrl);
    
// //     cntrl.use_feed_forward_ = true;
// //     cntrl.base_hat_.u = 1.0;
// //     cntrl.xc_.pn = 1.0;

// //     test_chasing_velocities_greater_than_boat_velocity(cntrl);
// // }

// void test_velocities_vs_expected_velocities(controller::Controller &cntrl, std::array<double,3> expectedV1Velocities)
// {
//     double tolerance{0.001};
//     double dt = 0.001;

//     cntrl.computeControl(dt);

//     EXPECT_NEAR(cntrl.xc_.x_dot,expectedV1Velocities[0],tolerance);
//     EXPECT_NEAR(cntrl.xc_.y_dot,expectedV1Velocities[1],tolerance);
//     EXPECT_NEAR(cntrl.xc_.z_dot,expectedV1Velocities[2],tolerance);
// }

// // void test_chasing_velocities_greater_than_boat_velocity(controller::Controller &cntrl)
// // {
// //     double tolerance{0.001};
// //     double dt = 0.001;

// //     cntrl.computeControl(dt);

// //     EXPECT_GE(cntrl.xc_.x_dot,cntrl.base_hat_.u)
// // }

// void setup_control_object(controller::Controller &cntrl)
// {
//     cntrl.control_mode_ = rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE;
//     cntrl.xc_.pn = 0.0;
//     cntrl.xc_.pe = 0.0;
//     cntrl.xc_.pd = 0.0;
//     cntrl.xc_.psi = 0.0;
//     cntrl.xhat_.pn = 0.0;
//     cntrl.xhat_.pe = 0.0;
//     cntrl.xhat_.pd = 0.0;
//     cntrl.xhat_.phi = 0.0;
//     cntrl.xhat_.theta = 0.0;
//     cntrl.xhat_.psi = 0.0;
//     cntrl.xhat_.u = 0.0;
//     cntrl.xhat_.v = 0.0;
//     cntrl.xhat_.w = 0.0;
//     cntrl.base_hat_.u = 0.0;
//     cntrl.base_hat_.v = 0.0;
//     cntrl.base_hat_.w = 0.0;
//     cntrl.use_feed_forward_ = false;
//     cntrl.landed_ = false;
//     cntrl.is_landing_ = false;
    
//     cntrl.max_.roll = 0.196;
//     cntrl.max_.pitch = 0.196;
//     cntrl.max_.yaw_rate = 0.785;
//     cntrl.throttle_eq_ = 0.5;
//     cntrl.throttle_down_ = 0.99;
//     cntrl.max_.throttle = 0.85;
//     cntrl.min_altitude_ = 0.0;
//     cntrl.max_.n_dot = 1.5;
//     cntrl.max_.e_dot = 1.5;
//     cntrl.max_.d_dot = 0.5;

//     double max_accel_z_ = 1.0 / cntrl.throttle_eq_;
//     double max_accel_xy_ = sin(acos(cntrl.throttle_eq_)) / cntrl.throttle_eq_ / sqrt(2.);

//     double P, I, D, tau;
//     tau = 0.05;
//     P = 0.5;
//     I = 0.0;
//     D = 0.05;
//     cntrl.PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

//     P = 0.5;
//     I = 0.0;
//     D = 0.05;
//     cntrl.PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

//     P = 0.6;
//     I = 0.25;
//     D = 0.1;
//     // set max z accelerations so that we can't fall faster than 1 gravity
//     cntrl.PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);

//     P = 1.5;
//     I = 0.0;
//     D = 0.6;
//     cntrl.PID_n_.setGains(P, I, D, tau, cntrl.max_.n_dot, -cntrl.max_.n_dot);

//     P = 1.5;
//     I = 0.0;
//     D = 0.6;
//     cntrl.PID_e_.setGains(P, I, D, tau, cntrl.max_.e_dot, -cntrl.max_.e_dot);

//     P = 1.0;
//     I = 0.0;
//     D = 0.0;
//     cntrl.PID_d_.setGains(P, I, D, tau, cntrl.max_.d_dot, -cntrl.max_.d_dot);

//     P = 2.0;
//     I = 0.0;
//     D = 0.0;
//     cntrl.PID_psi_.setGains(P, I, D, tau);
// }