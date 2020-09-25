#include "ekf/ekf.h"

using namespace Eigen;

namespace roscopter::ekf
{
void EKF::compassingCallback(const double& t, const double& z, const Matrix4d& R)
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
  double yaw_hat = x().q.yaw();
  double yaw_res = yaw - yaw_hat;

  //Todo rename q_res to r and remove testCompassingR.  Move test to check x()
  q_res = quat::Quatd::from_euler(0.0, 0.0, yaw_res);
  testCompassingR = z.R(0);

  xform::Xformd zhat = x().x;
  Vector4d r;
  r(0) = q_res[0];
  r(1) = q_res[1];
  r(2) = q_res[2];
  r(3) = q_res[3];

  typedef ErrorState E;
  Matrix<double, 4, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DQ) = I_3x3;
  H(3, E::DQ+3) = 1.0;

  if (use_compassing_)
    measUpdate(r, z.R, H);
}

}
