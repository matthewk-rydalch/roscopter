#include <iostream>
#include "gtest/gtest.h"

#define private public //not the best practise.  This may cause problems
#include "ekf/ekf_ros.h"

struct Quaternion
{
    double w, x, y, z;
};

struct Heading_Struct
{
  Quaternion expectedQuat;
  Quaternion quat;

  Eigen::Matrix4d expectedCovariance;
  Eigen::Matrix4d covariance;
};

Heading_Struct setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingEstimate, double rtkHeadingAccuracy);
Quaternion euler_2_quaternion(double yaw, double pitch, double roll);

TEST(rtkCompassingCallback, GivenRelPosMsgExpectHeading)
{
  double rtkHeading = 1.3; //radians
  double rtkHeadingEstimate = 0.0;
  double rtkHeadingAccuracy = 0.1; //radians

  Heading_Struct heading = setup_given_relpos_expect_heading_test(rtkHeading, rtkHeadingEstimate, rtkHeadingAccuracy);

  EXPECT_NEAR(heading.quat.w, heading.expectedQuat.w, 0.001);
  EXPECT_NEAR(heading.quat.x, heading.expectedQuat.x, 0.001);
  EXPECT_NEAR(heading.quat.y, heading.expectedQuat.y, 0.001);
  EXPECT_NEAR(heading.quat.z, heading.expectedQuat.z, 0.001);
  // EXPECT_NEAR(heading.covariance, heading.expectedCovariance, 0.001);
  // EXPECT_NEAR(heading.covariance.x, heading.expectedCovariance.x, 0.001);
  // EXPECT_NEAR(heading.covariance.y, heading.expectedCovariance.y, 0.001);
  // EXPECT_NEAR(heading.covariance.z, heading.expectedCovariance.z, 0.001);

  //** make sure that manual_compassing is turned off in ekf.yaml
}

// TEST(rtkCompassingUpdate, GivenAnRTKCompassingUpdateExpectCorrectEstimate)
// {
//   double rtkHeading = 1.3; //radians
//   // double rtkHeadingEstimate = 0.0;
//   double rtkHeadingAccuracy = 0.1; //radians

//   Heading_Struct heading = setup_given_relpos_expect_heading_test(rtkHeading, rtkHeadingEstimate, rtkHeadingAccuracy);

//   EXPECT_NEAR(heading.quat.w, heading.expectedQuat.w, 0.001);
//   EXPECT_NEAR(heading.quat.x, heading.expectedQuat.x, 0.001);
//   EXPECT_NEAR(heading.quat.y, heading.expectedQuat.y, 0.001);
//   EXPECT_NEAR(heading.quat.z, heading.expectedQuat.z, 0.001);
// }

Heading_Struct setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingEstimate, double rtkHeadingAccuracy)
{
  int argc;
  char** argv;
  ros::init(argc, argv, "estimator");
  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();

  // Eigen::Matrix<double, SIZE, 1> arr;
  // Eigen::Map<Vector6d> x;
  // Eigen::Map<Eigen::Vector3d> p;
  // Eigen::Map<Eigen::Vector3d> q;
  // Eigen::Map<Eigen::Vector3d> v;
  // Eigen::Map<Eigen::Vector3d> ba;
  // Eigen::Map<Eigen::Vector3d> bg;
  // double& bb; // bias for barometer measurements
  // double& ref; // reference global altitude of NED frame
  estimator.ekf_.x().p[0] = 1.2;
  estimator.ekf_.x().p[1] = -3.7;
  estimator.ekf_.x().p[2] = -5.2;
  estimator.ekf_.x().v[0] = 3.7;
  estimator.ekf_.x().v[1] = 0.0;
  estimator.ekf_.x().v[2] = -0.01;
  estimator.ekf_.x().ba[0] = 0.01;
  estimator.ekf_.x().ba[1] = -0.003;
  estimator.ekf_.x().ba[2] = -0.06;
  estimator.ekf_.x().bg[0] = 0.0002;
  estimator.ekf_.x().bg[1] = 0.07;
  estimator.ekf_.x().bg[2] = -0.01;
  estimator.ekf_.x().bb = 1.01;
  estimator.ekf_.x().ref = -1200.2;
  

  ublox::RelPos message;
  message.relPosHeading = rtkHeading;
  message.accHeading = rtkHeadingAccuracy;
  ublox::RelPosConstPtr* msg = new ublox::RelPosConstPtr{&message};
  double stdev = rtkHeadingAccuracy * rtkHeadingAccuracy;
  // Eigen::Matrix4d rtkcompassing_R_ << stdev, 0.0, 0.0, 0.0,
  //                              0.0, stdev, 0.0, 0.0,
  //                              0.0, 0.0, stdev, 0.0,
  //                              0.0, 0.0, 0.0, stdev;
  Quaternion rtkQuaternionEstimate = euler_2_quaternion(0.0,0.0,rtkHeadingEstimate);
  estimator.ekf_.x().q[0] = rtkQuaternionEstimate.w;
  estimator.ekf_.x().q[1] = rtkQuaternionEstimate.x;
  estimator.ekf_.x().q[2] = rtkQuaternionEstimate.y;
  estimator.ekf_.x().q[3] = rtkQuaternionEstimate.z;
  double rtkResHeading = rtkHeading-rtkHeadingEstimate;
  Quaternion rtkResQuaternion = euler_2_quaternion(0.0,0.0,rtkResHeading);

  estimator.gnssCallbackRelPos(*msg);

  Heading_Struct headingStruct;

  headingStruct.quat.w = estimator.ekf_.q_res[0];
  headingStruct.quat.x = estimator.ekf_.q_res[1];
  headingStruct.quat.y = estimator.ekf_.q_res[2];
  headingStruct.quat.z = estimator.ekf_.q_res[3];
  headingStruct.expectedQuat.w = rtkResQuaternion.w;
  headingStruct.expectedQuat.x = rtkResQuaternion.x;
  headingStruct.expectedQuat.y = rtkResQuaternion.y;
  headingStruct.expectedQuat.z = rtkResQuaternion.z;
  // headingStruct.covariance = estimator.ekf_.testCompassingR;
  // headingStruct.expectedCovariance = rtkCompassing_R;

  return headingStruct;
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