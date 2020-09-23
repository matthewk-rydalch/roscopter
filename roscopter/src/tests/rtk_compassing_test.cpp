#include <iostream>
#include "gtest/gtest.h"
#include "ekf/ekf_ros.h"


TEST(rtkCompassingCallback, GivenRelPosMsgExpectHeading)
{
  double relativeXPosition = 2.3;
  double rtkHeading = 2.0; //radians
  double rtkHeadingAccuracy = 0.1; //radians

  int argc;
  char** argv;
  ros::init(argc, argv, "estimator");
  ublox::RelPos message;
  message.relPosNED[0] = relativeXPosition;
  message.relPosHeading = rtkHeading;
  message.accHeading = rtkHeadingAccuracy;
  ublox::RelPosConstPtr* msg = new ublox::RelPosConstPtr{&message};

  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();
  estimator.gnssCallbackRelPos(*msg);
d
  double testCompassing_R = rtkHeadingAccuracy * rtkHeadingAccuracy;

  EXPECT_NEAR(estimator.base_relPos_msg_.point.x, -relativeXPosition, 0.001);
  EXPECT_NEAR(estimator.compassing_heading, rtkHeading, 0.001);
  EXPECT_NEAR(estimator.compassing_R_, testCompassing_R, 0.001);

  //** make sure that manual_compassing is turned off in ekf.yaml
}