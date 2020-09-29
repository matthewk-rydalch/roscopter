#include "tests/rtk_compassing_test.h"

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