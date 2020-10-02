#include "tests/rtk_compassing_test.h"

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

double setup_given_relpos_expect_covariance_test(double rtkHeading, double rtkHeadingAccuracy, double initialHeading, double expectedCovariance[][17], int covarianceSize)
{
  int argc;
  char** argv;
  ros::init(argc, argv, "estimator");
  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();

  State initialState;
  std::array<double,3> initialRollPitch = quaternion_2_euler(initialState.qw,initialState.qx,initialState.qy,initialState.qz);
  Quaternion initialQuat = euler_2_quaternion(initialRollPitch[0],initialRollPitch[1],initialHeading);
  initialState.qw = initialQuat.w;
  initialState.qx = initialQuat.x;
  initialState.qy = initialQuat.y;
  initialState.qz = initialQuat.z;

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

  double covarianceError = 0.0;
  for (int i=0;i<covarianceSize;i++)
  {
    for (int j=0;j<covarianceSize;j++)
    {
      covarianceError = covarianceError + estimator.ekf_.P()(i,j) - expectedCovariance[i][j];
    }
  }
  
  return covarianceError;
}