#include "tests/feed_forward_test.h"

TEST(ff_test, stationaryBaseWNoError)
{
    int argc;
    char** argv;
    ros::init(argc, argv, "ff_test");
    ros::NodeHandle nh;
    controller::Controller cntrl;

    setup_control_object(cntrl);
    
    std::array<double,3> expectedV1Velocities{0.0,0.0,0.0};

    test_velocities_vs_expected_velocities(cntrl, expectedV1Velocities);
}

TEST(ff_test, BaseVelEquals1_NoError_NoHeadingDifference)
{
    int argc;
    char** argv;
    ros::init(argc, argv, "ff_test");
    ros::NodeHandle nh;
    controller::Controller cntrl;

    setup_control_object(cntrl);
    
    cntrl.use_feed_forward_ = true;
    cntrl.base_hat_.u = 1.0;

    std::array<double,3> expectedV1Velocities{1.0,0.0,0.0};

    test_velocities_vs_expected_velocities(cntrl, expectedV1Velocities);
}

TEST(ff_test, BaseVelEquals1_NoError_BaseMinusRoverHeadingEqualsPiHalves)
{
    int argc;
    char** argv;
    ros::init(argc, argv, "ff_test");
    ros::NodeHandle nh;
    controller::Controller cntrl;

    setup_control_object(cntrl);
    
    cntrl.use_feed_forward_ = true;
    cntrl.base_hat_.u = 1.0;
    cntrl.base_hat_.psi = M_PI/2.0;

    std::array<double,3> expectedV1Velocities{0.0,1.0,0.0};

    test_velocities_vs_expected_velocities(cntrl, expectedV1Velocities);
}

void test_velocities_vs_expected_velocities(controller::Controller &cntrl, std::array<double,3> expectedV1Velocities)
{
    double tolerance{0.001};
    double dt = 0.001;

    cntrl.computeControl(dt);

    EXPECT_NEAR(cntrl.xc_.x_dot,expectedV1Velocities[0],tolerance);
    EXPECT_NEAR(cntrl.xc_.y_dot,expectedV1Velocities[1],tolerance);
    EXPECT_NEAR(cntrl.xc_.z_dot,expectedV1Velocities[2],tolerance);
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
    cntrl.base_hat_.u = 0.0;
    cntrl.base_hat_.v = 0.0;
    cntrl.base_hat_.w = 0.0;
    cntrl.use_feed_forward_ = false;
    cntrl.landed_ = false;
    cntrl.is_landing_ = false;
    
    cntrl.max_.roll = 0.196;
    cntrl.max_.pitch = 0.196;
    cntrl.max_.yaw_rate = 0.785;
    cntrl.throttle_eq_ = 0.5;
    cntrl.throttle_down_ = 0.99;
    cntrl.max_.throttle = 0.85;
    cntrl.min_altitude_ = 0.0;
    cntrl.max_.n_dot = 1.5;
    cntrl.max_.e_dot = 1.5;
    cntrl.max_.d_dot = 0.5;
}