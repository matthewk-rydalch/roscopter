#include "tests/ekf/ekf_load.h"

namespace roscopter::ekf
{
TEST_F(EkfLoadTest, loadPB2g)
{
    Eigen::Vector3d expected_p_b2g_{0, 0, 0};        
    EXPECT_EQ(expected_p_b2g_,ekf_.p_b2g_);
}

TEST_F(EkfLoadTest, loadQx)
{
    Eigen::Matrix<double, 17, 17> expected_Qx;
    expected_Qx = matrixZeros17x17_;
    expected_Qx(6,6) = 0.00001;
    expected_Qx(7,7) = 0.00001;
    expected_Qx(8,8) = 0.00001;
    expected_Qx(9,9) = 0.0001;
    expected_Qx(10,10) = 0.0001;
    expected_Qx(11,11) = 0.0001;
    expected_Qx(12,12) = 0.0001;
    expected_Qx(13,13) = 0.0001;
    expected_Qx(14,14) = 0.0001;
    expected_Qx(15,15) = 0.000001;
    expected_Qx(16,16) = 1.0;

    EXPECT_EQ(expected_Qx,ekf_.Qx_);
}

TEST_F(EkfLoadTest, loadP)
{
    Eigen::Matrix<double, 17, 17> expected_P;
    expected_P = matrixZeros17x17_;
    expected_P(0,0) = 10.0;
    expected_P(1,1) = 10.0;
    expected_P(2,2) = 10.0;
    expected_P(3,3) = 0.01;
    expected_P(4,4) = 0.01;
    expected_P(5,5) = 0.01;
    expected_P(6,6) = 0.001;
    expected_P(7,7) = 0.001;
    expected_P(8,8) = 0.001;
    expected_P(9,9) = 0.01;
    expected_P(10,10) = 0.01;
    expected_P(11,11) = 0.01;
    expected_P(12,12) = 0.001;
    expected_P(13,13) = 0.001;
    expected_P(14,14) = 0.001;
    expected_P(15,15) = 9.0;
    expected_P(16,16) = 100.0;

    EXPECT_EQ(expected_P,ekf_.P());
}

TEST_F(EkfLoadTest, loadP0Yaw)
{
    double expectedP0Yaw{0.01};
    EXPECT_EQ(expectedP0Yaw,ekf_.P()(5,5));
}

TEST_F(EkfLoadTest, loadRZeroVel)
{
    Eigen::Matrix<double, 4, 4> expectedRZeroVel;
    expectedRZeroVel = matrixZeros4x4_;
    expectedRZeroVel(0,0) = 0.001;
    expectedRZeroVel(1,1) = 0.001;
    expectedRZeroVel(2,2) = 0.001;
    expectedRZeroVel(3,3) = 0.001;
    EXPECT_EQ(expectedRZeroVel,ekf_.R_zero_vel_);
}

TEST_F(EkfLoadTest, loadMeasurementFlags)
{
    bool expectedUseMocap = false;
    bool expectedUseGnss = true;
    bool expectedUseBaro = false;
    bool expectedUseCompassing = false;
    bool expectedUseZeroVel = false;

    EXPECT_EQ(expectedUseMocap,ekf_.use_mocap_);
    EXPECT_EQ(expectedUseGnss,ekf_.use_gnss_);
    EXPECT_EQ(expectedUseBaro,ekf_.use_baro_);
    EXPECT_EQ(expectedUseCompassing,ekf_.use_compassing_);
    EXPECT_EQ(expectedUseZeroVel,ekf_.use_zero_vel_);
}

TEST_F(EkfLoadTest, loadArmedCheckParams)
{
    bool expectedEnableArmCheck{false};
    double expectedIsFlyingThreshold{10.5};

    EXPECT_EQ(expectedEnableArmCheck,ekf_.enable_arm_check_);
    EXPECT_EQ(expectedIsFlyingThreshold,ekf_.is_flying_threshold_);
}

TEST_F(EkfLoadTest, loadInitialState)
{
    //TODO:: Struggling to get ref_heading test to work.
    // double ref_heading{0.0};
    // std::vector<double> expectedQ_n2I{0.0,0.0,0.0,1.0};
    //TODO:: Include test for manual ref lla set up.  For now, we don't really use manual ref lla.
    xform::Xformd expectedX0;
    expectedX0.arr() << 0.0,0.0,0.0,1.0,0.0,0.0,0.0;

    // EXPECT_EQ(expectedQ_n2I[0],ekf_.q_n2I_(0));
    EXPECT_EQ(expectedX0.arr(),ekf_.x0_.arr());
}

TEST_F(EkfLoadTest, loadFinalParams)
{
    double expected_update_baro_vel_thresh{0.5};
    EXPECT_EQ(expected_update_baro_vel_thresh,ekf_.update_baro_vel_thresh_);
}
}
