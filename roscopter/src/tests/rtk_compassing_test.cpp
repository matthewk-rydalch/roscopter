#include <iostream>
#include <array>
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

struct State
{
  double x = 1.2;
  double y = -3.7;
  double z = -5.2;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double u = 3.7;
  double v = 0.0;
  double w = -0.01;
  double ba_x = 0.01;
  double ba_y = -0.003;
  double ba_z = -0.06;
  double bg_x = 0.0002;
  double bg_y = 0.07;
  double bg_z = -0.01;
  double bb = 1.01;
  double ref = -1200.2;
};

struct Compare_states
{
  State state;
  State expectedState;
};

Compare_states setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingAccuracy);
Quaternion euler_2_quaternion(double yaw, double pitch, double roll);

TEST(rtkCompassingUpdate, GivenAnRTKCompassingUpdateExpectCorrectEstimate)
{
  double rtkHeading = 1.3; //radians
  double rtkHeadingAccuracy = 0.1; //radians

  Compare_states compare = setup_given_relpos_expect_heading_test(rtkHeading,rtkHeadingAccuracy);

  EXPECT_NEAR(compare.expectedState.x, compare.state.x, 0.001);
  EXPECT_NEAR(compare.expectedState.y, compare.state.y, 0.001);
  EXPECT_NEAR(compare.expectedState.z, compare.state.z, 0.001);
  EXPECT_NEAR(compare.expectedState.qw, compare.state.qw  , 0.001);
  EXPECT_NEAR(compare.expectedState.qx, compare.state.qx , 0.001); 
  EXPECT_NEAR(compare.expectedState.qy, compare.state.qy, 0.001);  
  EXPECT_NEAR(compare.expectedState.qz, compare.state.qz, 0.001);  
  EXPECT_NEAR(compare.expectedState.u, compare.state.u, 0.001);
  EXPECT_NEAR(compare.expectedState.v,compare.state.v, 0.001);
  EXPECT_NEAR(compare.expectedState.w, compare.state.w, 0.001);
  EXPECT_NEAR(compare.expectedState.ba_x, compare.state.ba_x, 0.001); 
  EXPECT_NEAR(compare.expectedState.ba_y, compare.state.ba_y, 0.001);
  EXPECT_NEAR(compare.expectedState.ba_z, compare.state.ba_z, 0.001);
  EXPECT_NEAR(compare.expectedState.bg_x, compare.state.bg_x, 0.001);
  EXPECT_NEAR(compare.expectedState.bg_y, compare.state.bg_y, 0.001);
  EXPECT_NEAR(compare.expectedState.bg_z, compare.state.bg_z, 0.001);
  EXPECT_NEAR(compare.expectedState.bb, compare.state.bb, 0.001);
  EXPECT_NEAR(compare.expectedState.ref, compare.state.ref, 0.001);
}

Compare_states setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingAccuracy)
{
  int argc;
  char** argv;
  ros::init(argc, argv, "estimator");
  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();

  State expectedState;
  State initialState;

  ublox::RelPos message;
  message.relPosHeading = rtkHeading;
  message.accHeading = rtkHeadingAccuracy;
  ublox::RelPosConstPtr* msg = new ublox::RelPosConstPtr{&message};  //Todo: Delete new memory or allocate differently?

  estimator.ekf_.x().p[0] = initialState.x;
  estimator.ekf_.x().p[1] = initialState.y;
  estimator.ekf_.x().p[2] = initialState.z;
  estimator.ekf_.x().q[0] = initialState.qw;
  estimator.ekf_.x().q[1] = initialState.qx;
  estimator.ekf_.x().q[2] = initialState.qy;
  estimator.ekf_.x().q[3] = initialState.qz;
  estimator.ekf_.x().v[0] = initialState.u;
  estimator.ekf_.x().v[1] = initialState.v;
  estimator.ekf_.x().v[2] = initialState.w;
  estimator.ekf_.x().ba[0] = initialState.ba_x;
  estimator.ekf_.x().ba[1] = initialState.ba_y;
  estimator.ekf_.x().ba[2] = initialState.ba_z;
  estimator.ekf_.x().bg[0] = initialState.bg_x;
  estimator.ekf_.x().bg[1] = initialState.bg_y;
  estimator.ekf_.x().bg[2] = initialState.bg_z;
  estimator.ekf_.x().bb = initialState.bb;
  estimator.ekf_.x().ref = initialState.ref;

  estimator.gnssCallbackRelPos(*msg);

  Compare_states compare;
  compare.state.x = estimator.ekf_.x().p[0];
  compare.state.y = estimator.ekf_.x().p[1];
  compare.state.z = estimator.ekf_.x().p[2];
  compare.state.qw = estimator.ekf_.x().q[0];
  compare.state.qx = estimator.ekf_.x().q[1];
  compare.state.qy = estimator.ekf_.x().q[2];
  compare.state.qz = estimator.ekf_.x().q[3];
  compare.state.u = estimator.ekf_.x().v[0];
  compare.state.v = estimator.ekf_.x().v[1];
  compare.state.w = estimator.ekf_.x().v[2];
  compare.state.ba_x = estimator.ekf_.x().ba[0];
  compare.state.ba_y = estimator.ekf_.x().ba[1];
  compare.state.ba_z = estimator.ekf_.x().ba[2];
  compare.state.bg_x = estimator.ekf_.x().bg[0];
  compare.state.bg_y = estimator.ekf_.x().bg[1];
  compare.state.bg_z = estimator.ekf_.x().bg[2];
  compare.state.bb = estimator.ekf_.x().bb;
  compare.state.ref = estimator.ekf_.x().ref;
  
  compare.expectedState = expectedState;

  return compare;
}

// TEST(rtkCompassingCallback, GivenRelPosMsgExpectHeading)
// {
//   double rtkHeading = 1.3; //radians
//   double rtkHeadingEstimate = 0.0;
//   double rtkHeadingAccuracy = 0.1; //radians

//   Heading_Struct heading = setup_given_relpos_expect_heading_test(rtkHeading, rtkHeadingEstimate, rtkHeadingAccuracy);

//   EXPECT_NEAR(heading.quat.w, heading.expectedQuat.w, 0.001);
//   EXPECT_NEAR(heading.quat.x, heading.expectedQuat.x, 0.001);
//   EXPECT_NEAR(heading.quat.y, heading.expectedQuat.y, 0.001);
//   EXPECT_NEAR(heading.quat.z, heading.expectedQuat.z, 0.001);
//   // EXPECT_NEAR(heading.covariance, heading.expectedCovariance, 0.001);
//   // EXPECT_NEAR(heading.covariance.x, heading.expectedCovariance.x, 0.001);
//   // EXPECT_NEAR(heading.covariance.y, heading.expectedCovariance.y, 0.001);
//   // EXPECT_NEAR(heading.covariance.z, heading.expectedCovariance.z, 0.001);

//   //** make sure that manual_compassing is turned off in ekf.yaml
// }

// Heading_Struct setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingEstimate, double rtkHeadingAccuracy)
// {
//   int argc;
//   char** argv;
//   ros::init(argc, argv, "estimator");
//   roscopter::ekf::EKF_ROS estimator;
//   estimator.initROS();

//   estimator.ekf_.x().p[0] = 1.2;
//   estimator.ekf_.x().p[1] = -3.7;
//   estimator.ekf_.x().p[2] = -5.2;
//   estimator.ekf_.x().v[0] = 3.7;
//   estimator.ekf_.x().v[1] = 0.0;
//   estimator.ekf_.x().v[2] = -0.01;
//   estimator.ekf_.x().ba[0] = 0.01;
//   estimator.ekf_.x().ba[1] = -0.003;
//   estimator.ekf_.x().ba[2] = -0.06;
//   estimator.ekf_.x().bg[0] = 0.0002;
//   estimator.ekf_.x().bg[1] = 0.07;
//   estimator.ekf_.x().bg[2] = -0.01;
//   estimator.ekf_.x().bb = 1.01;
//   estimator.ekf_.x().ref = -1200.2;
  

//   ublox::RelPos message;
//   message.relPosHeading = rtkHeading;
//   message.accHeading = rtkHeadingAccuracy;
//   ublox::RelPosConstPtr* msg = new ublox::RelPosConstPtr{&message};
//   double stdev = rtkHeadingAccuracy * rtkHeadingAccuracy;
//   // Eigen::Matrix4d rtkcompassing_R_ << stdev, 0.0, 0.0, 0.0,
//   //                              0.0, stdev, 0.0, 0.0,
//   //                              0.0, 0.0, stdev, 0.0,
//   //                              0.0, 0.0, 0.0, stdev;
//   Quaternion rtkQuaternionEstimate = euler_2_quaternion(0.0,0.0,rtkHeadingEstimate);
//   estimator.ekf_.x().q[0] = rtkQuaternionEstimate.w;
//   estimator.ekf_.x().q[1] = rtkQuaternionEstimate.x;
//   estimator.ekf_.x().q[2] = rtkQuaternionEstimate.y;
//   estimator.ekf_.x().q[3] = rtkQuaternionEstimate.z;
//   double rtkResHeading = rtkHeading-rtkHeadingEstimate;
//   Quaternion rtkResQuaternion = euler_2_quaternion(0.0,0.0,rtkResHeading);

//   estimator.gnssCallbackRelPos(*msg);

//   Heading_Struct headingStruct;

//   headingStruct.quat.w = estimator.ekf_.q_res[0];
//   headingStruct.quat.x = estimator.ekf_.q_res[1];
//   headingStruct.quat.y = estimator.ekf_.q_res[2];
//   headingStruct.quat.z = estimator.ekf_.q_res[3];
//   headingStruct.expectedQuat.w = rtkResQuaternion.w;
//   headingStruct.expectedQuat.x = rtkResQuaternion.x;
//   headingStruct.expectedQuat.y = rtkResQuaternion.y;
//   headingStruct.expectedQuat.z = rtkResQuaternion.z;
//   // headingStruct.covariance = estimator.ekf_.testCompassingR;
//   // headingStruct.expectedCovariance = rtkCompassing_R;

//   return headingStruct;
// }

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