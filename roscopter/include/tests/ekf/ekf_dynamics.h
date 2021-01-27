#include "gtest/gtest.h"

#include "ekf/ekf.h"

namespace roscopter::ekf
{

class EkfDynamicsTest : public testing::Test
{
public:
    EKF ekf_;

    double time_{0.0};

protected:
    void SetUp() override
    {
        std::string filename{"/home/matt/ws_rtkflight/src/roscopter/roscopter/params/ekf.yaml"};
        ekf_.load(filename);
        ekf_.initialize(time_);
    }
    void TearDown() override
    {

    }

    void test_eigen_vectors_3d(Eigen::Vector3d expected, Eigen::Vector3d actual);

};

class EkfDynamicsNonZeroTest : public EkfDynamicsTest
{
public:


protected:
    void SetUp() override
    {
        std::string filename{"/home/matt/ws_rtkflight/src/roscopter/roscopter/params/ekf.yaml"};
        ekf_.load(filename);
        initialize();
    }

    void initialize();
};

}