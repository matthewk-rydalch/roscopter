#include <iostream>
#include "gtest/gtest.h"
#include "ekf/ekf_ros.h"


TEST(rtkCompassingCallback, GivenRelPosMsgExpectHeading)
{
  int argc;
  char** argv;
  ros::init(argc, argv, "estimator");
  ublox::RelPos message;
  message.relPosNED[0] = 2.3;
  message.relPosHeading = 2.0; //radians
  ublox::RelPosConstPtr* msg = new ublox::RelPosConstPtr{&message};

  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();
  estimator.gnssCallbackRelPos(*msg);

  EXPECT_NEAR(estimator.base_relPos_msg_.point.x, -message.relPosNED[0], 0.001);
  EXPECT_NEAR(estimator.compassing_heading, message.relPosHeading, 0.001);
}