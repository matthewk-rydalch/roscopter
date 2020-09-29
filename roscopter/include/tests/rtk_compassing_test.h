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
void test_quaternion(Compare_states compare, double expectedHeadingEstimate);
void test_all_states(Compare_states compare, double expectedHeadingEstimate);

void setup_given_relpos_expect_covariance_test(double rtkHeading, double rtkHeadingAccuracy, double expectedHeadingEstimate, double initialHeading[][17]);
void fill_expected_covariance(double expectedCovariance[][17], int covarianceSize);

Quaternion euler_2_quaternion(double yaw, double pitch, double roll);
std::array<double,3> quaternion_2_euler(double qw, double qx, double qy, double qz);