#include "tests/rtk_compassing_test.h"

TEST(rtkCompassingUpdate, TestCovariance)
{
  double rtkHeading = 1.3; //radians
  double rtkHeadingAccuracy = 0.1; //radians
  double initialHeading = 0.0;
  int covarianceSize = 17;
  double expectedCovariance[covarianceSize][17] = {0.0};
  fill_expected_covariance(expectedCovariance,covarianceSize);

  setup_given_relpos_expect_covariance_test(rtkHeading, rtkHeadingAccuracy, initialHeading, expectedCovariance);

  double tolerance = 0.001;
  // EXPECT_NEAR(compare.expectedState.x, compare.state.x, tolerance);

}

void fill_expected_covariance(double expectedCovariance[][17], int covarianceSize)
{
  expectedCovariance[0][0] = 10.0;
  expectedCovariance[1][1] = 10.0;
  expectedCovariance[2][2] = 10.0;
  expectedCovariance[3][3] = 0.01;
  expectedCovariance[4][4] = 0.01;
  expectedCovariance[5][5] = 0.005;
  expectedCovariance[6][6] = 0.001;
  expectedCovariance[7][7] = 0.001;
  expectedCovariance[8][8] = 0.001;
  expectedCovariance[9][9] = 0.01;
  expectedCovariance[10][10] = 0.01;
  expectedCovariance[11][11] = 0.01;
  expectedCovariance[12][12] = 0.001;
  expectedCovariance[13][13] = 0.001;
  expectedCovariance[14][14] = 0.001;
  expectedCovariance[15][15] = 9.0;
  expectedCovariance[16][16] = 100.0;  
}

void setup_given_relpos_expect_covariance_test(double rtkHeading, double rtkHeadingAccuracy, double initialHeading, double expectedCovariance[][17])
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

  // std::cout << "expected Covariance = " << expectedCovariance << std::endl;
  std::cout << "Covariance = " << estimator.ekf_.P() << std::endl;

  // TODO find a way to test P
  // compare.state.x = estimator.ekf_.x().p[0];
}