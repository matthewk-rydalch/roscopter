#include "gtest/gtest.h"

#include "ekf/ekf.h"

namespace roscopter::ekf
{

class EkfInitialize : public testing::Test
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
};

}