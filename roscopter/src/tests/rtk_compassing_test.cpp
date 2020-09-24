#include <iostream>
#include "gtest/gtest.h"
#include "ekf/ekf_ros.h"

struct Quaternion
{
    double w, x, y, z;
};

Quaternion euler_2_quaternion(double yaw, double pitch, double roll);

TEST(rtkCompassingCallback, GivenRelPosMsgExpectHeading)
{
  double relativeXPosition = 2.3;
  double rtkHeading = 1.3; //radians
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

  double testCompassing_R = rtkHeadingAccuracy * rtkHeadingAccuracy;

  double testEstimatedYaw = 0.0;
  Quaternion testEstimatedQuaternion = euler_2_quaternion(0.0,0.0,testEstimatedYaw);
  //TODO check the order of this quaternion
  estimator.ekf_.x().q[0] = testEstimatedQuaternion.w;
  estimator.ekf_.x().q[1] = testEstimatedQuaternion.x;
  estimator.ekf_.x().q[2] = testEstimatedQuaternion.y;
  estimator.ekf_.x().q[3] = testEstimatedQuaternion.z;
  double testResYaw = rtkHeading-testEstimatedYaw;
  Quaternion testResQuaternion = euler_2_quaternion(0.0,0.0,testResYaw);

  estimator.gnssCallbackRelPos(*msg);

  EXPECT_NEAR(estimator.base_relPos_msg_.point.x, -relativeXPosition, 0.001);
  EXPECT_NEAR(estimator.compassing_heading, rtkHeading, 0.001);
  EXPECT_NEAR(estimator.compassing_R_, testCompassing_R, 0.001);
  EXPECT_NEAR(estimator.ekf_.q_res[0], testResQuaternion.w, 0.001);
  EXPECT_NEAR(estimator.ekf_.q_res[1], testResQuaternion.x, 0.001);
  EXPECT_NEAR(estimator.ekf_.q_res[2], testResQuaternion.y, 0.001);
  EXPECT_NEAR(estimator.ekf_.q_res[3], testResQuaternion.z, 0.001);
  EXPECT_NEAR(estimator.ekf_.testCompassingR, testCompassing_R, 0.001);

  //** make sure that manual_compassing is turned off in ekf.yaml
}

Quaternion euler_2_quaternion(double roll, double pitch, double yaw)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}