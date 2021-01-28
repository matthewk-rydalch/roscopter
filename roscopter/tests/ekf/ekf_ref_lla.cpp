#include "tests/ekf/ekf_ref_lla.h"

namespace roscopter::ekf
{

void EkfRefLla::test_xform(xform::Xformd expected,xform::Xformd actual,double tolerance)
{
    test_vector_3d(expected.t(),actual.t(),tolerance);
    test_quaternion(expected.q(),actual.q(),tolerance);
}

void EkfRefLla::test_vector_3d(Eigen::Vector3d expected,Eigen::Vector3d actual,double tolerance)
{
    for (int i{0};i<3;i++)
    {
        EXPECT_NEAR(expected[i],actual[i],tolerance);
    }
}

void EkfRefLla::test_quaternion(quat::Quatd expected,quat::Quatd actual,double tolerance)
{
    EXPECT_NEAR(expected.w(),actual.w(),tolerance);
    EXPECT_NEAR(expected.x(),actual.x(),tolerance);
    EXPECT_NEAR(expected.y(),actual.y(),tolerance);
    EXPECT_NEAR(expected.z(),actual.z(),tolerance);
}

TEST_F(EkfRefLla,setRefLla)
{
    ekf_.ref_lla_set_ = false;

    Eigen::Vector3d ref_lla;
    ref_lla << 40.267320, -111.635629, 1387.0;

    ekf_.setRefLla(ref_lla);

    double expectedRefLat{ref_lla[0]*M_PI/180.0};
    double expectedRefLon{ref_lla[1]*M_PI/180.0};
    double expectedRefAlt{ref_lla[2]};
    bool expectedRefLlaSet{true};

    double tolerance{0.001};
    EXPECT_NEAR(expectedRefLat,ekf_.ref_lat_radians_,tolerance);
    EXPECT_NEAR(expectedRefLon,ekf_.ref_lon_radians_,tolerance);
    EXPECT_NEAR(expectedRefAlt,ekf_.x().ref,tolerance);
    EXPECT_EQ(expectedRefLlaSet,ekf_.ref_lla_set_);

}

TEST_F(EkfRefLla,IfRefLlaSetDoNothing)
{
    //Possible issue.  The lat and lon don't reset on their own between tests (and they shouldn't).  The x().Ref does (and it shouldn't).
    ekf_.ref_lat_radians_ = 0.0;
    ekf_.ref_lon_radians_ = 1824124.0;
    ekf_.x().ref = -22.0;
    ekf_.ref_lla_set_ = true;

    Eigen::Vector3d ref_lla;
    ref_lla << 40.267320, -111.635629, 1387.0;

    ekf_.setRefLla(ref_lla);

    double expectedRefLat{0.0};
    double expectedRefLon{1824124.0};
    double expectedRefAlt{-22.0};
    bool expectedRefLlaSet{true};

    double tolerance{0.001};
    EXPECT_NEAR(expectedRefLat,ekf_.ref_lat_radians_,tolerance);
    EXPECT_NEAR(expectedRefLon,ekf_.ref_lon_radians_,tolerance);
    EXPECT_NEAR(expectedRefAlt,ekf_.x().ref,tolerance);
    EXPECT_EQ(expectedRefLlaSet,ekf_.ref_lla_set_);
}

TEST_F(EkfRefLla,x_e2I)
{
    ekf_.ref_lla_set_ = false;

    Eigen::Vector3d ref_lla;
    ref_lla << 40.267320, -111.635629, 1387.0;

    ekf_.setRefLla(ref_lla);

    Eigen::Vector3d ecef;
    ecef << -1797290.29923633,-4531202.79031687,4101575.56190053;
    quat::Quatd quat;
    quat.setW(0.2362495);
    quat.setX(-0.7505615);
    quat.setY(-0.50974);
    quat.setZ(-0.3478632);
    xform::Xformd expectedX_e2I;
    expectedX_e2I.t() = ecef;
    expectedX_e2I.q() = quat;

    double tolerance{0.001};
    test_xform(expectedX_e2I,ekf_.x_e2I_,tolerance);
}

}