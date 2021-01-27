#include "gtest/gtest.h"

#include "ekf/ekf.h"

namespace roscopter::ekf
{
class EkfPropagateTest : public testing::Test
{
public:
    EKF ekf_;
    double time_{0.0};

    State initialState_;
    Matrix6d I_6x6_;

protected:
    void SetUp() override
    {
        std::string filename{"/home/matt/ws_rtkflight/src/roscopter/roscopter/params/ekf.yaml"};
        ekf_.load(filename);
        ekf_.initialize(time_);
        initialState_ = ekf_.x();
        I_6x6_ << 1.0,0.0,0.0,0.0,0.0,0.0,
                 0.0,1.0,0.0,0.0,0.0,0.0,
                 0.0,0.0,1.0,0.0,0.0,0.0,
                 0.0,0.0,0.0,1.0,0.0,0.0,
                 0.0,0.0,0.0,0.0,1.0,0.0,
                 0.0,0.0,0.0,0.0,0.0,1.0;
    }
    void TearDown() override
    {

    }

    void test_state(State expected,State actual);
    void test_vector_3d(Eigen::Vector3d expected,Eigen::Vector3d actual,double tolerance);
    void test_quaternion(quat::Quatd expected,quat::Quatd actual,double tolerance);
    void test_vector_6d(Vector6d expected,Vector6d actual,double tolerance);
    State initialize_test_zero_input();
};
}