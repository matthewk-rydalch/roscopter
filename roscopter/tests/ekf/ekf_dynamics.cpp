#include "tests/ekf/ekf_dynamics.h"

namespace roscopter::ekf
{

void EkfDynamicsTest::test_eigen_vectors_3d(Eigen::Vector3d expected, Eigen::Vector3d actual)
{
    double tolerance{0.001};
    for(int i{0};i<3;i++)
    {
        EXPECT_NEAR(expected(i),actual(i),tolerance);
    } 
}

TEST_F(EkfDynamicsTest,testZeroCase)
{
    Vector6d u;
    u << 0.0,0.0,0.0,0.0,0.0,0.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxP;
    expectedDxP << 0.0,0.0,0.0;
    Eigen::Vector3d expectedDxQ;
    expectedDxQ << 0.0,0.0,0.0;
    Eigen::Vector3d expectedDxV;
    expectedDxV << 0.0,0.0,9.80665;
    Eigen::Vector3d expectedDxBa;
    expectedDxBa << 0.0,0.0,0.0;
    Eigen::Vector3d expectedDxBg;
    expectedDxBg << 0.0,0.0,0.0;

    test_eigen_vectors_3d(expectedDxP,ekf_.dx_.p);
    test_eigen_vectors_3d(expectedDxQ,ekf_.dx_.q);
    test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
    test_eigen_vectors_3d(expectedDxBa,ekf_.dx_.ba);
    test_eigen_vectors_3d(expectedDxBg,ekf_.dx_.bg);
}

TEST_F(EkfDynamicsTest,testZerosWImuInputs)
{
    Vector6d u;
    u << 1.0,2.0,-1.2,-0.2,1.0,3.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxP;
    expectedDxP << 0.0,0.0,0.0;
    Eigen::Vector3d expectedDxQ;
    expectedDxQ << -0.2,1.0,3.0;
    Eigen::Vector3d expectedDxV;
    expectedDxV << 1.0,2.0,8.60665;
    Eigen::Vector3d expectedDxBa;
    expectedDxBa << 0.0,0.0,0.0;
    Eigen::Vector3d expectedDxBg;
    expectedDxBg << 0.0,0.0,0.0;

    test_eigen_vectors_3d(expectedDxP,ekf_.dx_.p);
    test_eigen_vectors_3d(expectedDxQ,ekf_.dx_.q);
    test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
    test_eigen_vectors_3d(expectedDxBa,ekf_.dx_.ba);
    test_eigen_vectors_3d(expectedDxBg,ekf_.dx_.bg);
}

void EkfDynamicsNonZeroTest::initialize()
{
        ekf_.x().t = 30.2;
        // last four values of x are a quaternion that equals 10, 0, -45 deg in euler angles.
        ekf_.x().x.arr() << 1.0,2.0,-3.0,-0.2477,0.8374,0.4672,0.1382;
        ekf_.x().v << -1.0,0.2,-0.4;
        ekf_.x().ba << 0.0,0.52,-1.1;
        ekf_.x().bg << 0.01,-0.2,-0.72;
        ekf_.x().bb = 0.2;
        ekf_.x().ref = 4000.0;
        ekf_.x().a << 0.0,0.0,-9.80665;
        ekf_.x().w << -1.0,0.0,0.3;
        ekf_.is_flying_ = false;
        ekf_.armed_ = false;
}

// TEST_F(EkfDynamicsNonZeroTest,testFromNonZeroInitialization)
// {
//     Vector6d u;
//     u << 1.0,2.0,-1.2,-0.2,1.0,3.0;

//     ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

//     Eigen::Vector3d expectedDxP;
//     expectedDxP << 0.0,0.0,0.0;
//     Eigen::Vector3d expectedDxQ;
//     expectedDxQ << -0.2,1.0,3.0;
//     Eigen::Vector3d expectedDxV;
//     expectedDxV << 1.0,2.0,8.60665;
//     Eigen::Vector3d expectedDxBa;
//     expectedDxBa << 0.0,0.0,0.0;
//     Eigen::Vector3d expectedDxBg;
//     expectedDxBg << 0.0,0.0,0.0;

//     test_eigen_vectors_3d(expectedDxP,ekf_.dx_.p);
    // test_eigen_vectors_3d(expectedDxQ,ekf_.dx_.q);
    // test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
//     test_eigen_vectors_3d(expectedDxBa,ekf_.dx_.ba);
//     test_eigen_vectors_3d(expectedDxBg,ekf_.dx_.bg);
// }

TEST_F(EkfDynamicsTest,testBiasCancelling)
{
    Vector6d u;
    u << 0.0,0.52,-1.1,0.01,-0.2,-0.72;
    ekf_.x().ba << 0.0,0.52,-1.1;
    ekf_.x().bg << 0.01,-0.2,-0.72;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxQ;
    expectedDxQ << 0.0,0.0,0.0;
    Eigen::Vector3d expectedDxV;
    expectedDxV << 0.0,0.0,9.80665;

    test_eigen_vectors_3d(expectedDxQ,ekf_.dx_.q);
    test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
}

TEST_F(EkfDynamicsTest,testPGivenQAndVSimple)
{
    Vector6d u;
    u << 0.0,0.0,0.0,0.0,0.0,0.0;
    ekf_.x().x.arr() << 0.0,0.0,0.0,0.707,0.0,0.0,0.707;
    ekf_.x().v << 1.0,0.0,0.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxP;
    expectedDxP << 0.0,1.0,0.0;

    test_eigen_vectors_3d(expectedDxP,ekf_.dx_.p);
}

TEST_F(EkfDynamicsTest,testPGivenQAndVSimple2)
{
    Vector6d u;
    u << 0.0,0.0,0.0,0.0,0.0,0.0;
    ekf_.x().x.arr() << 0.0,0.0,0.0,0.5,0.5,-0.5,0.5;
    ekf_.x().v << 1.0,0.0,0.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxP;
    expectedDxP << 0.0,0.0,1.0;

    test_eigen_vectors_3d(expectedDxP,ekf_.dx_.p);
}

TEST_F(EkfDynamicsTest,testPGivenQAndV)
{
    Vector6d u;
    u << 0.0,0.0,0.0,0.0,0.0,0.0;
    ekf_.x().x.arr() << 0.0,0.0,0.0,0.8363,-0.3266,0.1353,-0.4190; //euler angles -30,30,-45 deg
    ekf_.x().v << -1.0,0.2,-0.4;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxP;
    expectedDxP << -0.6899,0.7031,-0.4793;

    test_eigen_vectors_3d(expectedDxP,ekf_.dx_.p);
}

TEST_F(EkfDynamicsTest,testVGivenUQAndVSimple1)
{
    Vector6d u;
    u << 1.0,-0.5,2.0,0.0,0.0,0.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxV;
    expectedDxV << 1.0,-0.5,11.80665;

    test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
}

TEST_F(EkfDynamicsTest,testVGivenUQAndVSimple2)
{
    Vector6d u;
    u << 1.0,-0.5,2.0,0.0,0.0,0.0;
    ekf_.x().x.arr() << 0.0,0.0,0.0,0.9659258,0.0,0.258819,0.0; //euler angles 0,30,0 deg

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxV;
    expectedDxV << -3.9033,-0.5000,10.4928;

    test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
}

TEST_F(EkfDynamicsTest,testVGivenUQAndVSimple3)
{
    Vector6d u;
    u << 1.0,-0.5,2.0,0.0,0.0,0.0;
    ekf_.x().x.arr() << 0.0,0.0,0.0,0.8923991,0.0990458,0.2391176,0.3696438; //euler angles 0,30,45 deg

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxV;
    expectedDxV << -2.4672,2.9672,10.4928;

    test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
}

TEST_F(EkfDynamicsTest,testVGivenUQAndV)
{
    Vector6d u;
    u << 1.0,-0.5,2.0,2.0,-1.0,0.0;
    ekf_.x().x.arr() << 0.0,0.0,0.0,0.8923991,0.0990458,0.2391176,0.3696438; //euler angles 0,30,45 deg
    ekf_.x().v << 1.0,0.0,-2.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,false);

    Eigen::Vector3d expectedDxV;
    expectedDxV << -4.4672,-1.0328,9.4928;

    test_eigen_vectors_3d(expectedDxV,ekf_.dx_.v);
}
}