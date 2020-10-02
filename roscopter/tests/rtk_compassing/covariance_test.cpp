#include "tests/rtk_compassing_test.h"

TEST(rtkCompassingUpdate, TestCovariance1)
{
  double rtkHeading = 1.3; //radians
  double rtkHeadingAccuracy = 0.1; //radians
  double initialHeading = 0.0;
  int covarianceSize = 17;
  double expectedCovariance[covarianceSize][17] = {0.0};
  fill_expected_covariance(expectedCovariance,covarianceSize);
  double covarianceError = setup_given_relpos_expect_covariance_test(rtkHeading, rtkHeadingAccuracy, initialHeading, expectedCovariance, covarianceSize);

  double tolerance = 0.001;
  EXPECT_NEAR(covarianceError,0.0,tolerance);
}

TEST(rtkCompassingUpdate, TestCovariance2)
{
  double rtkHeading = -1.8; //radians
  double rtkHeadingAccuracy = 0.3; //radians
  double initialHeading = -1.0;
  int covarianceSize = 17;
  double expectedCovariance[covarianceSize][17] = {0.0};
  fill_expected_covariance(expectedCovariance,covarianceSize);
  expectedCovariance[5][5] = 0.009;
  double covarianceError = setup_given_relpos_expect_covariance_test(rtkHeading, rtkHeadingAccuracy, initialHeading, expectedCovariance, covarianceSize);

  double tolerance = 0.001;
  EXPECT_NEAR(covarianceError,0.0,tolerance);
}

TEST(rtkCompassingUpdate, TestCovariance3)
{
  double rtkHeading = 0.23427; //radians
  double rtkHeadingAccuracy = 0.11123; //radians
  double initialHeading = -0.0732;
  int covarianceSize = 17;
  double expectedCovariance[covarianceSize][17] = {0.0};
  fill_expected_covariance(expectedCovariance,covarianceSize);
  expectedCovariance[5][5] = 0.00553;
  double covarianceError = setup_given_relpos_expect_covariance_test(rtkHeading, rtkHeadingAccuracy, initialHeading, expectedCovariance, covarianceSize);

  double tolerance = 0.001;
  EXPECT_NEAR(covarianceError,0.0,tolerance);
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