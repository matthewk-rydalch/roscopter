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