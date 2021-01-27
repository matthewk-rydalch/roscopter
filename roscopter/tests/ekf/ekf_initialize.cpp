#include "tests/ekf/ekf_initialize.h"

namespace roscopter::ekf
{

TEST_F(EkfInitialize,testAll)
{
    double expectedTime{time_};
    xform::Xformd expectedX0;
    expectedX0 = ekf_.x0_;
    Eigen::Vector3d expectedV;
    expectedV << 0.0,0.0,0.0;
    Eigen::Vector3d expectedBa;
    expectedBa << 0.0,0.0,0.0;
    Eigen::Vector3d expectedBg;
    expectedBg << 0.0,0.0,0.0;
    double expectedBb{0.0};
    double expectedRef{0.0};
    Eigen::Vector3d expectedA;
    expectedA << 0.0,0.0,-9.80665;  //This removes gravity, which is always felt by imu, even when stationary.
    Eigen::Vector3d expectedW;
    expectedW << 0.0,0.0,0.0;
    bool expectedIsFlyging{false};
    bool expectedArmed{false};


    EXPECT_EQ(expectedTime,ekf_.x().t);
    EXPECT_EQ(expectedX0.arr(),ekf_.x().x.arr());
    EXPECT_EQ(expectedV,ekf_.x().v);
    EXPECT_EQ(expectedBa,ekf_.x().ba);
    EXPECT_EQ(expectedBg,ekf_.x().bg);
    EXPECT_EQ(expectedBb,ekf_.x().bb);
    EXPECT_EQ(expectedRef,ekf_.x().ref);
    EXPECT_EQ(expectedA,ekf_.x().a);
    EXPECT_EQ(expectedW,ekf_.x().w);
    EXPECT_EQ(expectedIsFlyging,ekf_.is_flying_);
    EXPECT_EQ(expectedArmed,ekf_.armed_);
}

}