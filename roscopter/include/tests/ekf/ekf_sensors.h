#include "gtest/gtest.h"
#include "ekf/ekf.h"

namespace roscopter::ekf
{

class EkfSensorsTest : public testing::Test
{
public:
    EKF ekf_;
    double time_{34.0};
    Matrix6d I6x6_;
    State initialState_;
    dxMat initialP_;

protected:
    void SetUp() override
    {
        std::string filename{"/home/matt/ws_rtkflight/src/roscopter/roscopter/params/ekf.yaml"};
        ekf_.load(filename);
        ekf_.initialize(time_);
        I6x6_ << 1.0,0.0,0.0,0.0,0.0,0.0,
                 0.0,1.0,0.0,0.0,0.0,0.0,
                 0.0,0.0,1.0,0.0,0.0,0.0,
                 0.0,0.0,0.0,1.0,0.0,0.0,
                 0.0,0.0,0.0,0.0,1.0,0.0,
                 0.0,0.0,0.0,0.0,0.0,1.0;
        initialState_ = ekf_.x();
        initialP_ = ekf_.P();
    }

    void test_state(State expected,State actual);
    void test_vector_3d(Eigen::Vector3d expected,Eigen::Vector3d actual,double tolerance);
    void test_quaternion(quat::Quatd expected,quat::Quatd actual,double tolerance);
    void test_vector_6d(Vector6d expected,Vector6d actual,double tolerance);
    void test17x17Matrix(dxMat expected,dxMat actual);
};

}