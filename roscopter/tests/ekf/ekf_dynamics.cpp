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

void EkfDynamicsTest::test_matrix_17d(dxMat expected,dxMat actual)
{
    double tolerance{0.001};
    for(int i{0};i<17;i++)
    {
        for(int j{0};j<17;j++)
        {
            EXPECT_NEAR(expected(i,j),actual(i,j),tolerance);
        }
    } 
}

void EkfDynamicsTest::test_matrix_17x6d(dxuMat expected,dxuMat actual)
{
    double tolerance{0.001};
    for(int i{0};i<17;i++)
    {
        for(int j{0};j<6;j++)
        {
            EXPECT_NEAR(expected(i,j),actual(i,j),tolerance);
        }
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

TEST_F(EkfDynamicsTest,testJacobianCalcAZeros)
{
    Vector6d u;
    u << 0.0,0.0,0.0,0.0,0.0,0.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,true);
    Eigen::Matrix3d RTranspose;
    RTranspose << 1.0,0.0,0.0,
         0.0,1.0,0.0,
         0.0,0.0,1.0;
    Eigen::Matrix3d skewGravity;
    skewGravity << 0.0,-9.80665,0.0,
                   9.80665,0.0,0.0,
                   0.0,0.0,0.0;
    dxMat expectedA;
    expectedA = matrixZeros17x17_;
    expectedA.block<3,3>(0,6) = RTranspose;
    expectedA.block<3,3>(3,12) = -I_3x3;
    expectedA.block<3,3>(6,3) = skewGravity;
    expectedA.block<3,3>(6,9) = -I_3x3;  

    test_matrix_17d(expectedA,ekf_.A_);
}

TEST_F(EkfDynamicsTest,testJacobianAGivenRVAndOmega)
{
    Vector6d u;
    u << 0.0,0.0,0.0,1.0,-1.0,2.0;
    ekf_.x().v << -1.0,1.0,-2.0;
    ekf_.x().x.arr() << 0.0,0.0,0.0,0.8923991,0.0990458,0.2391176,0.3696438; //euler angles 0,30,45 deg
    // ekf_.x().x.arr() << 0.0,0.0,0.0,0.7071,0.0,0.,0.7071; //euler angles 0,0,45 deg
    // ekf_.x().x.arr() << 0.0,0.0,0.0,0.9659258,0.0,0.258819,0.0; //euler angles 0,30,0 deg

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,true);

    Eigen::Matrix3d R;
    R << 0.6124,0.7071,-0.3536,
         -0.6124,0.7071,0.3536,
         0.5000,0.0,0.8660;
    Eigen::Matrix3d skewV;
    skewV << 0.0,2.0,1.0,
             -2.0,0.0,1.0,
             -1.0,-1.0,0.0;
    Eigen::Matrix3d skewOmega;
    skewOmega << 0.0,-2.0,-1.0,
                 2.0,0.0,-1.0,
                 1.0,1.0,0.0;
    Eigen::Matrix3d skewGravityRotated;
    skewGravityRotated << 0.0,-8.4928,0.0,
                          8.4928,0.0,4.9033,
                          0.0,-4.9033,0.0; 
    dxMat expectedA;
    expectedA = matrixZeros17x17_;
    expectedA.block<3,3>(0,3) = -R.transpose()*skewV;
    expectedA.block<3,3>(0,6) = R.transpose();
    expectedA.block<3,3>(3,3) = -skewOmega;
    expectedA.block<3,3>(3,12) = -I_3x3;
    expectedA.block<3,3>(6,6) = -skewOmega;
    expectedA.block<3,3>(6,3) = skewGravityRotated; // Possible Issue.
    expectedA.block<3,3>(6,9) = -I_3x3;
    expectedA.block<3,3>(6,12) = -skewV;

    test_matrix_17d(expectedA,ekf_.A_);
}

TEST_F(EkfDynamicsTest,testJacobianCalcBZeros)
{
    Vector6d u;
    u << 0.0,0.0,0.0,0.0,0.0,0.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,true);

    dxuMat expectedB;
    expectedB = matrixZeros17x6_;
    expectedB.block<3,3>(3,3) = I_3x3;
    expectedB.block<3,3>(6,0) = I_3x3;

    test_matrix_17x6d(expectedB,ekf_.B_);
}

TEST_F(EkfDynamicsTest,testJacobianCalcBGivenV)
{
    Vector6d u;
    u << 0.0,0.0,0.0,0.0,0.0,0.0;
    ekf_.x().v << -1.0,1.0,-2.0;

    ekf_.dynamics(ekf_.x(),u,ekf_.dx_,true);

    Eigen::Matrix3d skewV;
    skewV << 0.0,2.0,1.0,
             -2.0,0.0,1.0,
             -1.0,-1.0,0.0;
    dxuMat expectedB;
    expectedB = matrixZeros17x6_;
    expectedB.block<3,3>(3,3) = I_3x3;
    expectedB.block<3,3>(6,0) = I_3x3;
    expectedB.block<3,3>(6,3) = skewV;

    test_matrix_17x6d(expectedB,ekf_.B_);
}

}