#include "ekf/ekf.h"

using namespace Eigen;

namespace roscopter::ekf
{
void EKF::compassingCallback(const double& t, const double& z, const double& R)
{

  if (enable_out_of_order_)
  {
    std::cout << "ERROR OUT OF ORDER COMPASS NOT IMPLEMENTED" << std::endl;
  }
  else
    compassingUpdate(meas::Compass(t, z, R));
}

void EKF::compassingUpdate(const meas::Compass &z)
{
  double yaw = z.z(0);
  const Vector1d yaw_hat(x().q.yaw()); 
  using Vector1d = Eigen::Matrix<double, 1, 1>;
  Vector1d yaw_res = z.z - yaw_hat;

  xform::Xformd zhat = x().x;

  typedef ErrorState E;
  Matrix<double, 1, E::NDX> H;
  H.setZero();
  H(0, E::DQ+2) = 1.0;

  if (use_compassing_)
    measUpdate(yaw_res, z.R, H);
}

}
