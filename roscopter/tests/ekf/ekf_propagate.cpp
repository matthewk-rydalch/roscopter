#include "tests/ekf/ekf_propagate.h"

namespace roscopter::ekf
{
void EkfPropagateTest::test_state(State expected,State actual)
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

void EkfPropagateTest::test_vector_3d(Eigen::Vector3d expected,Eigen::Vector3d actual,double tolerance)
{
    for (int i{0};i<3;i++)
    {
        EXPECT_NEAR(expected[i],actual[i],tolerance);
    }
}

void EkfPropagateTest::test_quaternion(quat::Quatd expected,quat::Quatd actual,double tolerance)
{
    EXPECT_NEAR(expected.w(),actual.w(),tolerance);
    EXPECT_NEAR(expected.x(),actual.x(),tolerance);
    EXPECT_NEAR(expected.y(),actual.y(),tolerance);
    EXPECT_NEAR(expected.z(),actual.z(),tolerance);
}

void EkfPropagateTest::test_vector_6d(Vector6d expected,Vector6d actual,double tolerance)
{
    for (int i{0};i<6;i++)
    {
        EXPECT_NEAR(expected[i],actual[i],tolerance);
    }
}

State EkfPropagateTest::initialize_test_zero_input()
{
    double t = 0.1;
    Vector6d imu;
    imu << 0.0,0.0,0.0,0.0,0.0,0.0;
    ekf_.propagate(t,imu,I_6x6_);

    State expectedState;
    expectedState = initialState_;
    expectedState.t = t;
    expectedState.v << 0.0,0.0,0.980665;
    expectedState.imu << imu;

    return expectedState;
}

void EkfPropagateTest::test17x17Matrix(dxMat expected,dxMat actual)
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
void EkfPropagateTest::test17x6Matrix(dxuMat expected,dxuMat actual)
{
    double tolerance{0.001};
    for (int i{0}; i<17; i++)
    {
        for (int j{0}; j<6; j++)
        {
            EXPECT_NEAR(expected(i,j),actual(i,j),tolerance);
        }
    }
}

TEST_F(EkfPropagateTest, StatePropZeroInputTime)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    EXPECT_NEAR(expectedState.t,ekf_.x().t,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputX)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_3d(expectedState.x.t(),ekf_.x().x.t(),tolerance);
    test_quaternion(expectedState.x.q(),ekf_.x().x.q(),tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputP)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_3d(expectedState.p,ekf_.x().p,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputQ)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_quaternion(expectedState.q,ekf_.x().q,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputV)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_3d(expectedState.v,ekf_.x().v,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputBa)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_3d(expectedState.ba,ekf_.x().ba,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputBg)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_3d(expectedState.bg,ekf_.x().bg,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputBb)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    EXPECT_NEAR(expectedState.bb,ekf_.x().bb,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputRef)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    EXPECT_NEAR(expectedState.ref,ekf_.x().ref,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputA)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_3d(expectedState.a,ekf_.x().a,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputW)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_3d(expectedState.w,ekf_.x().w,tolerance);
}
TEST_F(EkfPropagateTest, StatePropZeroInputImu)
{
    State expectedState = initialize_test_zero_input();
    double tolerance{0.001};
    test_vector_6d(expectedState.imu,ekf_.x().imu,tolerance);
}

TEST_F(EkfPropagateTest, StatePropWAccelAndVel)
{
    double t{0.1};
    ekf_.x().v << 2.0,-1.0,1.0;
    Vector6d imu;
    imu << 0.5,-0.5,9.80665,0.0,0.0,0.0;
    State expectedState{ekf_.x()};
    ekf_.propagate(t,imu,I_6x6_);
    expectedState.t = t;
    expectedState.p << 0.2,-0.1,0.1;
    expectedState.v << 2.05,-1.05,2.96133;
    expectedState.imu = imu;
    test_state(expectedState,ekf_.x());
}

TEST_F(EkfPropagateTest, StatePropWOmega)
{
    double t{0.1};
    Vector6d imu;
    imu << 0.0,0.0,0.0,1.0,0.5,-0.7;
    ekf_.x().q.setW(0.8876263); // euler angles 30,-30,45 
    ekf_.x().q.setX(0.135299);
    ekf_.x().q.setY(-0.3266407);
    ekf_.x().q.setZ(0.2951603);
    State expectedState{ekf_.x()};
    ekf_.propagate(t,imu,I_6x6_);
    expectedState.q.setW(0.8918568);
    expectedState.q.setX(0.2011605);;
    expectedState.q.setY(-0.313559);
    expectedState.q.setZ(0.2565279);    //Quaternions are not passing.
    expectedState.v << 0.64698, 0.04645, 0.73550; //Possible issue. passive and active rotations may be the issue in multiple spots.
    expectedState.t = t;
    expectedState.imu = imu;
    test_state(expectedState,ekf_.x());
}

TEST_F(EkfPropagateTest, covariancePropZerosTestAAndB)
{
    double t{0.1};
    Vector6d imu;
    imu << 0.0,0.0,0.0,0.0,0.0,0.0;
    Matrix6d R;
    R = I_6x6_;

    ekf_.propagate(t,imu,R);

    test17x17Matrix(expectedAGivenNoInputs_,ekf_.A_);
    test17x6Matrix(expectedBGivenNoInputs_,ekf_.B_);
}

TEST_F(EkfPropagateTest, covariancePropZerosTestP)
{
    double t{0.1};
    Vector6d imu;
    imu << 0.0,0.0,0.0,0.0,0.0,0.0;
    Matrix6d R;
    R = I_6x6_;

    ekf_.propagate(t,imu,R);

    dxMat expectedP;
    expectedP = ekf_.A_*initialP_*ekf_.A_.transpose()+ekf_.B_*R*ekf_.B_.transpose()+ekf_.Qx_*0.01;
    test17x17Matrix(expectedP,ekf_.P());
}

} //namespace