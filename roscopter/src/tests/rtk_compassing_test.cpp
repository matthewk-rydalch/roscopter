#include <iostream>
#include "gtest/gtest.h"
// #include "ekf.h"

// void test_rgb_from_max255_to_max1(std::array<int,3> rgbMax255, std::array<double,3> expectedRGBMax1)
// {
//   const double tolerance{0.001};

//   std::array<double,3> rgbMax1 = convert_rgb_from_max255_to_max1(rgbMax255);

//   EXPECT_NEAR(expectedRGBMax1[0], rgbMax1[0], tolerance);
//   EXPECT_NEAR(expectedRGBMax1[1], rgbMax1[1], tolerance);
//   EXPECT_NEAR(expectedRGBMax1[2], rgbMax1[2], tolerance);
// }

// TEST(RGBConversion, givenRGBMax255EqualsBlackExpectRGBMax1IsCorrect)
// {
//   std::array<int,3> rgbMax255Black{0,0,0};
//   std::array<double,3> expectedRGBMax1{0.0,0.0,0.0};

//   test_rgb_from_max255_to_max1(rgbMax255Black, expectedRGBMax1);
// }

TEST(gpsCompassingCallback, placeHolder)
{
  EXPECT_NEAR(1.0, 1.0, 0.001);
}

// int main()
// {
//   std::cout << "In gps compassing test.cpp \n";

//   return 0;
// }