#include "ekf/ekf.h"

#define T transpose()

using namespace Eigen;

namespace roscopter::ekf
{

void EKF::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{

  if (!is_flying_)
    checkIsFlying();

  propagate(t, z, R);
  if (!is_flying_)
    zeroVelUpdate(t);

  if (enable_log_)
  {
    logs_[LOG_IMU]->log(t);
    logs_[LOG_IMU]->logVectors(z);
  }

}

void EKF::baroCallback(const double &t, const double &z, const double &R,
                       const double &temp)
{
    baroUpdate(meas::Baro(t, z, R, temp));
}

void EKF::baroUpdate(const meas::Baro &z)
{
  if (!this->groundTempPressSet())
  {
    return;
  }
  else if (!update_baro_ || !is_flying_)
  {
    // Take the lowest pressure while I'm not flying as ground pressure
    // This has the effect of hopefully underestimating my altitude instead of
    // over estimating.
    if (z.z(0) < ground_pressure_)
    {
      ground_pressure_ = z.z(0);
      std::cout << "New ground pressure: " << ground_pressure_ << std::endl;
    }

    // check if we should start updating with the baro yet based on
    // velocity estimate
    if (x().v.norm() > update_baro_vel_thresh_)
      update_baro_ = true;

    return;
  }

  using Vector1d = Eigen::Matrix<double, 1, 1>;

  // // From "Small Unmanned Aircraft: Theory and Practice" eq 7.8
  const double g = 9.80665; // m/(s^2) gravity 
  const double R = 8.31432; // universal gas constant
  const double M = 0.0289644; // kg / mol. molar mass of Earth's air

  const double altitude = -x().p(2);
  const double baro_bias = x().bb;

  // From "Small Unmanned Aircraft: Theory and Practice" eq 7.9
  // const double rho = M * ground_pressure_ / R / ground_temperature_;
  const double rho = M * ground_pressure_ / R / z.temp;

  const double press_hat = ground_pressure_ - rho * g * altitude + baro_bias;

  const Vector1d zhat(press_hat);
  Vector1d r = z.z - zhat;

  typedef ErrorState E;

  Matrix<double, 1, E::NDX> H;
  H.setZero();
  H(0, E::DP + 2) = rho * g;
  H(0, E::DBB) = 1.;

  /// TODO: Saturate r
  if (use_baro_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_BARO_RES]->log(z.t);
    logs_[LOG_BARO_RES]->logVectors(r, z.z, zhat);
    logs_[LOG_BARO_RES]->log(z.temp);
  }

}

void EKF::gnssCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{

  if (!ref_lla_set_)
    return;

  gnssUpdate(meas::Gnss(t, z, R));

  if (enable_log_)
  {
    logs_[LOG_LLA]->log(t);
    logs_[LOG_LLA]->logVectors(ecef2lla((x_e2I_ * x().x).t()));
    logs_[LOG_LLA]->logVectors(ecef2lla(z.head<3>()));
  }
}

void EKF::gnssUpdate(const meas::Gnss &z)
{

  const Vector3d w = x().w - x().bg;
  const Vector3d gps_pos_I = x().p + x().q.rota(p_b2g_);
  const Vector3d gps_vel_b = x().v + w.cross(p_b2g_);
  const Vector3d gps_vel_I = x().q.rota(gps_vel_b);

  // Update ref_lla based on current estimate
  Vector3d ref_lla(ref_lat_radians_, ref_lon_radians_, x().ref);
  xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
  x_e2I_.t() = x_e2n.t();
  x_e2I_.q() = x_e2n.q() * q_n2I_;

  Vector6d zhat;
  zhat << x_e2I_.transforma(gps_pos_I),
          x_e2I_.rota(gps_vel_I);
  const Vector6d r = z.z - zhat; // residual

  const Matrix3d R_I2e = x_e2I_.q().R().T;
  const Matrix3d R_b2I = x().q.R().T;
  const Matrix3d R_e2b = R_I2e * R_b2I;

  const double sin_lat = sin(ref_lat_radians_);
  const double cos_lat = cos(ref_lat_radians_);
  const double sin_lon = sin(ref_lon_radians_);
  const double cos_lon = cos(ref_lon_radians_);
  const Vector3d dpEdRefAlt(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);

  typedef ErrorState E;

  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = R_I2e; // dpE/dpI
  H.block<3,3>(0, E::DQ) = -R_e2b * skew(p_b2g_);
  H.block<3, 1>(0, E::DREF) = dpEdRefAlt;
  H.block<3,3>(3, E::DQ) = -R_e2b * skew(gps_vel_b); // dvE/dQI
  H.block<3,3>(3, E::DV) = R_e2b;
  H.block<3,3>(3, E::DBG) = R_e2b * skew(p_b2g_);

  /// TODO: Saturate r
  if (use_gnss_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_GNSS_RES]->log(z.t);
    logs_[LOG_GNSS_RES]->logVectors(r, z.z, zhat);
  }
}

void EKF::mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R)
{
    mocapUpdate(meas::Mocap(t, z, R));
}

void EKF::mocapUpdate(const meas::Mocap &z)
{

  xform::Xformd zhat = x().x;

  // TODO Do we need to fix "-" operator for Xformd?
  // Right now using piecewise subtraction
  // on position and attitude separately. This may be correct though because
  // our state is represented as R^3 x S^3 (position, quaterion) not SE3.
  // calculate residual
  Vector6d r;
  r.segment<3>(0) = z.z.t_ - zhat.t_;
  r.segment<3>(3) = z.z.q_ - zhat.q_;

  typedef ErrorState E;
  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = I_3x3;
  H.block<3,3>(3, E::DQ) = I_3x3;


  /// TODO: Saturate r
  if (use_mocap_)
  {
    measUpdate(r, z.R, H);
  }

  if (enable_log_)
  {
    logs_[LOG_MOCAP_RES]->log(z.t);
    logs_[LOG_MOCAP_RES]->logVectors(r, z.z.arr(), zhat.arr());
  }
}

void EKF::compassingCallback(const double& t, const double& z, const double& R)
{
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
