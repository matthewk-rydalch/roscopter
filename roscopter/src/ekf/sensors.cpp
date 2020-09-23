#include "ekf/ekf.h"

namespace roscopter::ekf
{
void EKF::compassingCallback(const double& t, const double& z, const double& R)
{
  std::cout << "in compassing callback";

  // if (enable_out_of_order_)
  // {
  //   mocap_meas_buf_.push_back(meas::Mocap(t, z, R));
  //   meas_.insert(meas_.end(), &mocap_meas_buf_.back());
  // }
  // else
  //   mocapUpdate(meas::Mocap(t, z, R));


  // if (enable_log_)
  // {
  //   logs_[LOG_REF]->log(t);
  //   logs_[LOG_REF]->logVectors(z.arr(), z.q().euler());
  // }
}
}