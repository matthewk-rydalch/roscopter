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

Compare_states setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingAccuracy, double expectedHeadingEstimateChange, double initialHeading);
void setup_given_relpos_expect_covariance_test(double rtkHeading, double rtkHeadingAccuracy, double expectedHeadingEstimate, double initialHeading);
Quaternion euler_2_quaternion(double yaw, double pitch, double roll);
std::array<double,3> quaternion_2_euler(double qw, double qx, double qy, double qz);

void test_quaternion(Compare_states compare, double expectedHeadingEstimate)
{
  double tolerance = 0.001;
  EXPECT_NEAR(compare.expectedState.qw, compare.state.qw, tolerance);
  EXPECT_NEAR(compare.expectedState.qx, compare.state.qx, tolerance); 
  EXPECT_NEAR(compare.expectedState.qy, compare.state.qy, tolerance);  
  EXPECT_NEAR(compare.expectedState.qz, compare.state.qz, tolerance);

  std::array<double,3> euler = quaternion_2_euler(compare.state.qw, compare.state.qx, compare.state.qy, compare.state.qz);
  EXPECT_NEAR(expectedHeadingEstimate, euler[2], tolerance);
}

void test_all_states(Compare_states compare, double expectedHeadingEstimate)
{
  double tolerance = 0.001;
  EXPECT_NEAR(compare.expectedState.x, compare.state.x, tolerance);
  EXPECT_NEAR(compare.expectedState.y, compare.state.y, tolerance);
  EXPECT_NEAR(compare.expectedState.z, compare.state.z, tolerance);
  EXPECT_NEAR(compare.expectedState.qw, compare.state.qw, tolerance);
  EXPECT_NEAR(compare.expectedState.qx, compare.state.qx, tolerance); 
  EXPECT_NEAR(compare.expectedState.qy, compare.state.qy, tolerance);  
  EXPECT_NEAR(compare.expectedState.qz, compare.state.qz, tolerance);
  EXPECT_NEAR(compare.expectedState.u, compare.state.u, tolerance);
  EXPECT_NEAR(compare.expectedState.v, compare.state.v, tolerance);
  EXPECT_NEAR(compare.expectedState.w, compare.state.w, tolerance);
  EXPECT_NEAR(compare.expectedState.ba_x, compare.state.ba_x, tolerance);
  EXPECT_NEAR(compare.expectedState.ba_y, compare.state.ba_y, tolerance);
  EXPECT_NEAR(compare.expectedState.ba_z, compare.state.ba_z, tolerance);
  EXPECT_NEAR(compare.expectedState.bg_x, compare.state.bg_x, tolerance);
  EXPECT_NEAR(compare.expectedState.bg_y, compare.state.bg_y, tolerance);
  EXPECT_NEAR(compare.expectedState.bg_z, compare.state.bg_z, tolerance);
  EXPECT_NEAR(compare.expectedState.bb, compare.state.bb, tolerance);
  EXPECT_NEAR(compare.expectedState.ref, compare.state.ref, tolerance);

  std::array<double,3> euler = quaternion_2_euler(compare.state.qw, compare.state.qx, compare.state.qy, compare.state.qz);
  EXPECT_NEAR(expectedHeadingEstimate, euler[2], tolerance);
}

TEST(rtkCompassingUpdate, Test0Change)
{
  double rtkHeading = 0.0; //radians
  double rtkHeadingAccuracy = 0.1; //radians
  double expectedHeadingEstimate = 0.0;
  double initialHeading = 0.0;

  Compare_states compare = setup_given_relpos_expect_heading_test(rtkHeading,rtkHeadingAccuracy, expectedHeadingEstimate, initialHeading);

  test_quaternion(compare, expectedHeadingEstimate); 
}

TEST(rtkCompassingUpdate, TestStartNegative)
{
  double rtkHeading = 1.3; //radians
  double rtkHeadingAccuracy = 0.1; //radians
  double expectedHeadingEstimate = 0.15;
  double initialHeading = -1.0;

  Compare_states compare = setup_given_relpos_expect_heading_test(rtkHeading,rtkHeadingAccuracy, expectedHeadingEstimate, initialHeading);

  test_quaternion(compare, expectedHeadingEstimate); 
}

TEST(rtkCompassingUpdate, TestNegativeMeasurement)
{
  double rtkHeading = -0.9; //radians
  double initialHeading = 0.5;
  double rtkHeadingAccuracy = 0.2;
  double expectedHeadingEstimate = 0.22;

  Compare_states compare = setup_given_relpos_expect_heading_test(rtkHeading,rtkHeadingAccuracy, expectedHeadingEstimate, initialHeading);

  test_quaternion(compare, expectedHeadingEstimate); 
}

TEST(rtkCompassingUpdate, TestInaccurateMeasurement)
{
  double rtkHeading = 2.1; //radians
  double initialHeading = 0.8;
  double rtkHeadingAccuracy = 2.0;
  double expectedHeadingEstimate = 0.8032;

  Compare_states compare = setup_given_relpos_expect_heading_test(rtkHeading,rtkHeadingAccuracy, expectedHeadingEstimate, initialHeading);

  test_quaternion(compare, expectedHeadingEstimate); 
}

TEST(rtkCompassing, TestAllStates)
{
  double rtkHeading = 1.3; //radians
  double initialHeading = 0.0;
  double rtkHeadingAccuracy = 0.1;
  double expectedHeadingEstimate = 0.65;

  Compare_states compare = setup_given_relpos_expect_heading_test(rtkHeading, rtkHeadingAccuracy, expectedHeadingEstimate, initialHeading);

  test_all_states(compare, expectedHeadingEstimate);
}

// TEST(rtkCompassingUpdate, TestCovariance)
// {
//   double rtkHeading = 1.3; //radians
//   double rtkHeadingAccuracy = 0.1; //radians
//   double expectedHeadingEstimate = 0.65;
//   double initialHeading = 0.0;

//   setup_given_relpos_expect_covariance_test(rtkHeading, rtkHeadingAccuracy, expectedHeadingEstimate, initialHeading);

//   double tolerance = 0.001;
//   // EXPECT_NEAR(compare.expectedState.x, compare.state.x, tolerance);

// }

Compare_states setup_given_relpos_expect_heading_test(double rtkHeading, double rtkHeadingAccuracy, double expectedHeadingEstimate, double initialHeading)
{
  int argc;
  char** argv;
  ros::init(argc, argv, "estimator");
  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();

  State expectedState;
  State initialState;
  std::array<double,3> initialRollPitch = quaternion_2_euler(initialState.qw,initialState.qx,initialState.qy,initialState.qz);
  Quaternion initialQuat = euler_2_quaternion(initialRollPitch[0],initialRollPitch[1],initialHeading);
  initialState.qw = initialQuat.w;
  initialState.qx = initialQuat.x;
  initialState.qy = initialQuat.y;
  initialState.qz = initialQuat.z;
  Quaternion expectedQuaternionEstimate = euler_2_quaternion(initialRollPitch[0],initialRollPitch[1],expectedHeadingEstimate);
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

// void setup_given_relpos_expect_covariance_test(double rtkHeading, double rtkHeadingAccuracy, double expectedHeadingEstimate, double initialHeading)
// {
//   int argc;
//   char** argv;
//   ros::init(argc, argv, "estimator");
//   roscopter::ekf::EKF_ROS estimator;
//   estimator.initROS();

//   State initialState;
//   std::array<double,3> initialRollPitch = quaternion_2_euler(initialState.qw,initialState.qx,initialState.qy,initialState.qz);
//   Quaternion initialQuat = euler_2_quaternion(initialRollPitch[0],initialRollPitch[1],initialHeading);
//   initialState.qw = initialQuat.w;
//   initialState.qx = initialQuat.x;
//   initialState.qy = initialQuat.y;
//   initialState.qz = initialQuat.z;

//   ublox::RelPos message;
//   message.relPosHeading = rtkHeading;
//   message.accHeading = rtkHeadingAccuracy;
//   ublox::RelPosConstPtr* msg = new ublox::RelPosConstPtr{&message};  //Todo: Delete new memory or allocate differently?

//   estimator.ekf_.x().p[0] = initialState.x;
//   estimator.ekf_.x().p[1] = initialState.y;
//   estimator.ekf_.x().p[2] = initialState.z;
//   estimator.ekf_.x().q[0] = initialState.qw;
//   estimator.ekf_.x().q[1] = initialState.qx;
//   estimator.ekf_.x().q[2] = initialState.qy;
//   estimator.ekf_.x().q[3] = initialState.qz;
//   estimator.ekf_.x().v[0] = initialState.u;
//   estimator.ekf_.x().v[1] = initialState.v;
//   estimator.ekf_.x().v[2] = initialState.w;
//   estimator.ekf_.x().ba[0] = initialState.ba_x;
//   estimator.ekf_.x().ba[1] = initialState.ba_y;
//   estimator.ekf_.x().ba[2] = initialState.ba_z;
//   estimator.ekf_.x().bg[0] = initialState.bg_x;
//   estimator.ekf_.x().bg[1] = initialState.bg_y;
//   estimator.ekf_.x().bg[2] = initialState.bg_z;
//   estimator.ekf_.x().bb = initialState.bb;
//   estimator.ekf_.x().ref = initialState.ref;
//   estimator.start_time_.sec = 1.0; //Much of the code won't run if time = 0

//   estimator.gnssCallbackRelPos(*msg);

//   // compare.state.x = estimator.ekf_.x().p[0];
//   std::cout << "Got here" << "\n";
//   std::cout << "P() = " << estimator.ekf_.P() << "\n";
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