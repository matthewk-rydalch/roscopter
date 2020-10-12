#include "tests/feed_forward_test.h"

TEST(ff_test, stationaryBaseWNoError)
{
    int argc;
    char** argv;
    ros::init(argc, argv, "ff_test");
    ros::NodeHandle nh;
    controller::Controller cntrl;

    double dt = 0.001;
    cntrl.computeControl(dt);
}