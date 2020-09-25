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
    compassUpdate(meas::Compass(t, z, R));
}

void EKF::compassUpdate(const meas::Compass &z)
{
  double yaw = z.z(0);
  // double yaw_hat = x().q.yaw();
  const Vector1d yaw_hat(x().q.yaw()); 
  using Vector1d = Eigen::Matrix<double, 1, 1>;
  Vector1d yaw_res = z.z - yaw_hat;

  //Todo rename q_res to r and remove testCompassingR.  Move test to check x()
  // q_res = quat::Quatd::from_euler(0.0, 0.0, yaw_res);
  testCompassingR = z.R(0);

  xform::Xformd zhat = x().x;
  // Vector4d r;
  // r(0) = q_res[0];
  // r(1) = q_res[1];
  // r(2) = q_res[2];
  // r(3) = q_res[3];

  typedef ErrorState E;
  Matrix<double, 1, E::NDX> H;
  H.setZero();
  H(0, E::DQ+2) = 1.0;

  if (use_compassing_)
    measUpdate(yaw_res, z.R, H);
}

}
