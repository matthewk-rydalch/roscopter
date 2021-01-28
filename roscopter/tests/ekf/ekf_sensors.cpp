#include "tests/ekf/ekf_sensors.h"

namespace roscopter::ekf
{

void EkfSensorsTest::test_state(State expected,State actual)
{
    double tolerance{0.001};

    EXPECT_NEAR(expected.t,actual.t,tolerance);
    test_vector_3d(expected.x.t(),actual.x.t(),tolerance);
    test_quaternion(expected.x.q(),actual.x.q(),tolerance);
    test_vector_3d(expected.p,actual.p,tolerance);
    test_quaternion(expected.q,actual.q,tolerance);
    test_vector_3d(expected.v,actual.v,tolerance);
    test_vector_3d(expected.ba,actual.ba,tolerance);
    test_vector_3d(expected.bg,actual.bg,tolerance);
    EXPECT_NEAR(expected.bb,actual.bb,tolerance);
    EXPECT_NEAR(expected.ref,actual.ref,tolerance);
    test_vector_3d(expected.a,actual.a,tolerance);
    test_vector_3d(expected.w,actual.w,tolerance);
    test_vector_6d(expected.imu,actual.imu,tolerance);
}

void EkfSensorsTest::test_vector_3d(Eigen::Vector3d expected,Eigen::Vector3d actual,double tolerance)
{
    for (int i{0};i<3;i++)
    {
        EXPECT_NEAR(expected[i],actual[i],tolerance);
    }
}

void EkfSensorsTest::test_quaternion(quat::Quatd expected,quat::Quatd actual,double tolerance)
{
    EXPECT_NEAR(expected.w(),actual.w(),tolerance);
    EXPECT_NEAR(expected.x(),actual.x(),tolerance);
    EXPECT_NEAR(expected.y(),actual.y(),tolerance);
    EXPECT_NEAR(expected.z(),actual.z(),tolerance);
}

void EkfSensorsTest::test_vector_6d(Vector6d expected,Vector6d actual,double tolerance)
{
    for (int i{0};i<6;i++)
    {
        EXPECT_NEAR(expected[i],actual[i],tolerance);
    }
}
void EkfSensorsTest::test17x17Matrix(dxMat expected,dxMat actual)
{
    double tolerance{0.0001};
    for (int i{0}; i<17; i++)
    {
        for (int j{0}; j<17; j++)
        {
            EXPECT_NEAR(expected(i,j),actual(i,j),tolerance);
        }
    }
}

// TEST_F(EkfSensorsTest,firstTest)
// {
//     Vector6d z;
//     z << 1.0,0.0,3.0,-2.0,0.7,3.4;
//     Matrix6d R;
//     R.setZero();
//     for (int i{0};i<6;i++)
//     {
//         R(i,i) = 1.0;
//     }
//     R(2,2) = 2.0;
//     R(3,4) = 0.4;
//     R(0,4) = 0.1;
//     ekf_.gnssCallback(time_,z,R);

//     State expectedState = initialState_;
//     expectedState.p << 0.8455,0.0,2.5;
//     expectedState.v << -0.0023,0.0007,0.0034;
//     dxMat expectedP;
//     expectedP <<     0.9091,         0,         0,         0,         0,         0,         0,         0.0001,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,    0.9091,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,    1.6667,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,    0.0100,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,    0.0100,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,    0.0100,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,    0.0010,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,    0.0010,         0,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,    0.0010,         0,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,    0.0100,            0,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,         0,       0.0100,         0,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,    0.0100,         0,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,    0.0010,         0,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,    0.0010,         0,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,    0.0010,         0,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,    9.0000,         0,
//                           0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,  100.0000;
//     test_state(expectedState,ekf_.x());
//     test17x17Matrix(expectedP,ekf_.P());
// }

TEST_F(EkfSensorsTest,testResidual)
{
    ekf_.ref_lla_set_ = false;

    Eigen::Vector3d ref_lla;
    ref_lla << 40.267320, -111.635629, 1387.0;

    ekf_.setRefLla(ref_lla);

    Vector6d z;
    z << -1797292.0,-4531202.0,4101581.5,1.2,-2.3,0.9;
    Eigen::Vector3d gps_pos_I;
    gps_pos_I << 0.2,4.4,-12.7;
    Eigen::Vector3d gps_vel_I;
    gps_vel_I << 8.3,-9.9,10.0;
    Vector6d r{ekf_.gnssUpdateGetResidual(z,gps_pos_I,gps_vel_I)};

    Vector6d expectedR;
    expectedR << -2.2655,11.3003,-2.4232,5.6112,-18.0297,1.0303;

    double tolerance{0.001};
    test_vector_6d(expectedR,r,tolerance);
}

TEST_F(EkfSensorsTest,testH)
{
    ekf_.ref_lla_set_ = false;

    Eigen::Vector3d ref_lla;
    ref_lla << 40.267320, -111.635629, 1387.0;

    ekf_.setRefLla(ref_lla);

    Eigen::Matrix<double,6,17> H;
    H.setZero();
}

}