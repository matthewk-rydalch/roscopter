#include "gtest/gtest.h"
#include "ekf/ekf.h"

namespace roscopter::ekf
{

class EkfRefLla : public testing::Test
{
public:
    EKF ekf_;
    double time_{3.2};
protected:
    void SetUp() override
    {
        std::string filename{"/home/matt/ws_rtkflight/src/roscopter/roscopter/params/ekf.yaml"};
        ekf_.load(filename);
        ekf_.initialize(time_);
    }
    void test_xform(xform::Xformd expected,xform::Xformd actual,double tolerance);
    void test_vector_3d(Eigen::Vector3d expected,Eigen::Vector3d actual,double tolerance);
    void test_quaternion(quat::Quatd expected,quat::Quatd actual,double tolerance);
};

}