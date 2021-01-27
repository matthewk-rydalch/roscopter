#include "tests/ekf/ekf_update.h"

namespace roscopter::ekf
{

void EkfUpdateTest::test_state(State expected,State actual)
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

void EkfUpdateTest::test_vector_3d(Eigen::Vector3d expected,Eigen::Vector3d actual,double tolerance)
{
    for (int i{0};i<3;i++)
    {
        EXPECT_NEAR(expected[i],actual[i],tolerance);
    }
}

void EkfUpdateTest::test_quaternion(quat::Quatd expected,quat::Quatd actual,double tolerance)
{
    EXPECT_NEAR(expected.w(),actual.w(),tolerance);
    EXPECT_NEAR(expected.x(),actual.x(),tolerance);
    EXPECT_NEAR(expected.y(),actual.y(),tolerance);
    EXPECT_NEAR(expected.z(),actual.z(),tolerance);
}

void EkfUpdateTest::test_vector_6d(Vector6d expected,Vector6d actual,double tolerance)
{
    for (int i{0};i<6;i++)
    {
        EXPECT_NEAR(expected[i],actual[i],tolerance);
    }
}
void EkfUpdateTest::test17x17Matrix(dxMat expected,dxMat actual)
{
    double tolerance{0.001};
    for (int i{0}; i<17; i++)
    {
        for (int j{0}; j<17; j++)
        {
            EXPECT_NEAR(expected(i,j),actual(i,j),tolerance);
        }
    }
}

TEST_F(EkfUpdateTest, noInputs)
{
    Vector6d res;
    res << 0.0,0.0,0.0,0.0,0.0,0.0;
    Matrix6d R;
    R.setZero();
    for (int i{0};i<6;i++)
    {
        R(i,i) = 1.0;
    }
    Eigen::Matrix<double,6,17> H;
    H.setZero();
    for (int i{0};i<6;i++)
    {
        H(i,i) = 1.0;
    }

    ekf_.measUpdate(res,R,H);

    State expectedState = initialState_;
    dxMat expectedP;
    expectedP <<     0.9091,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,    0.9091,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,    0.9091,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,    0.0099,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,    0.0099,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,    0.0099,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,    0.0010,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,    0.0010,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,    0.0010,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,    0.0100,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,       0.0100,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,    0.0100,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,    0.0010,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,    0.0010,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,    0.0010,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,    9.0000,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,  100.0000;

    test_state(expectedState,ekf_.x());
    test17x17Matrix(expectedP,ekf_.P());
}

TEST_F(EkfUpdateTest, simpleRes)
{
    Vector6d res;
    res << 1.0,0.0,3.0,-2.0,0.7,3.4;
    Matrix6d R;
    R.setZero();
    for (int i{0};i<6;i++)
    {
        R(i,i) = 1.0;
    }
    Eigen::Matrix<double,6,17> H;
    H.setZero();
    for (int i{0};i<3;i++)
    {
        H(i,i) = 1.0;
    }
    for (int i{6};i<9;i++)
    {
        H(i-3,i) = 1.0;
    }

    ekf_.measUpdate(res,R,H);

    State expectedState = initialState_;
    expectedState.p << 0.9091,0.0,2.7273;
    expectedState.v << -0.002,0.0007,0.0034;
    dxMat expectedP;
    expectedP <<     0.9091,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,    0.9091,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,    0.9091,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,    0.0100,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,    0.0100,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,    0.0100,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,    0.0010,         0,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,    0.0010,         0,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,    0.0010,         0,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,    0.0100,            0,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,       0.0100,         0,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,    0.0100,         0,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,    0.0010,         0,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,    0.0010,         0,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,    0.0010,         0,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,    9.0000,         0,
                          0,         0,         0,         0,         0,         0,         0,         0,         0,         0,            0,         0,         0,         0,         0,         0,  100.0000;

    test_state(expectedState,ekf_.x());
    test17x17Matrix(expectedP,ekf_.P());
}

} //roscopter::ekf