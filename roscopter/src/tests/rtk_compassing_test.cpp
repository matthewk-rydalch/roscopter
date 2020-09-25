#include <iostream>
#include <array>
#include "gtest/gtest.h"

#define private public //not the best practise.  This may cause problems
#include "ekf/ekf_ros.h"

struct Quaternion
{
    double w, x, y, z;
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

Compare_states setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingAccuracy, double expectedHeadingEstimateChange);
Quaternion euler_2_quaternion(double yaw, double pitch, double roll);
std::array<double,3> quaternion_2_euler(double qw, double qx, double qy, double qz);

TEST(rtkCompassingUpdate, GivenAnRTKCompassingUpdateExpectCorrectEstimates)
{
  double rtkHeading = 1.3; //radians
  double rtkHeadingAccuracy = 0.1; //radians
  double expectedHeadingEstimateChange = 0.65;

  Compare_states compare = setup_given_relpos_expect_heading_test(rtkHeading,rtkHeadingAccuracy, expectedHeadingEstimateChange);

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

Compare_states setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingAccuracy, double expectedHeadingEstimateChange)
{
  int argc;
  char** argv;
  ros::init(argc, argv, "estimator");
  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();

  State expectedState;
  State initialState;
  std::array<double,3> initialEuler = quaternion_2_euler(initialState.qw,initialState.qx,initialState.qy,initialState.qz);
  double expectedHeadingEstimate = expectedHeadingEstimateChange + initialEuler[2];
  Quaternion expectedQuaternionEstimate = euler_2_quaternion(initialEuler[0],initialEuler[1],expectedHeadingEstimate);
  expectedState.qw = expectedQuaternionEstimate.w;
  expectedState.qx = expectedQuaternionEstimate.x;
  expectedState.qy = expectedQuaternionEstimate.y;
  expectedState.qz = expectedQuaternionEstimate.z;

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
  estimator.start_time_.sec = 1.0; //Much of the code won't run if time = 0

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

std::array<double,3> quaternion_2_euler(double qw, double qx, double qy, double qz)
{
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    double pitch = std::asin(sinp);
    if (std::abs(sinp) >= 1)
        double pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    std::array<double,3> euler{roll, pitch, yaw};

    return euler;
} 