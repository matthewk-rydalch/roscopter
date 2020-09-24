#include "ekf/ekf.h"

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
    std::cout << "in compassing update \n";

    x().q[0] = 1.0;
    x().q[1] = 0.0;
    x().q[2] = 0.0;
    x().q[3] = 0.0;
    double yaw = z.z(0);
    double yaw_hat = x().q.yaw();

    double yaw_res = yaw - yaw_hat;

    quat::Quatd q_res = quat::Quatd::from_euler(0.0, 0.0, yaw_res);

    std::cout << "yaw_res = " << yaw_res << "\n";
    std::cout << "q_res = " << q_res << "\n";
    std::cout << "q_hat = " << x().q << "\n";
    

    // test_res = psi-psihat;
// ///////////////////
//range update

      // Assume that the earth is flat and that the range sensor is rigidly attached
  // to the UAV, so distance is dependent on the attitude of the UAV.
  // TODO this assumes that the laser is positioned at 0,0,0 in the body frame
  // of the UAV
  // TODO this also only updates if the UAV is pretty close to level and the
  // measurement model jacobian assumes that the measurement is not dependent
  // on roll or pitch at all
//   using Vector1d = Eigen::Matrix<double, 1, 1>;

//   const double altitude = -x().p(2);
//   const double roll = x().q.roll();
//   const double pitch = x().q.pitch();

//   // const double level_threshold = 2. * M_PI / 180.; // 1 degree

//   // Only update if UAV is close to level
//   // if ((abs(roll) > level_threshold) || (abs(pitch) > level_threshold))
//     // return;

//   const Vector1d zhat(altitude / cos(roll) / cos(pitch)); // TODO roll/ pitch of drone
//   Vector1d r = z.z - zhat; // residual

//   // std::cout << "Laser Update: " << std::endl;
//   // std::cout << "Altitude meas: " << z.z(0) << std::endl;
//   // std::cout << "Altitude est: " << altitude << std::endl;

//   typedef ErrorState E;

//   Matrix<double, 1, E::NDX> H;
//   H.setZero();
//   H(0, E::DP + 2) = -1.;

//   // Vector1d r_saturated
//   double r_sat = 0.1;
//   if (abs(r(0)) > r_sat)
//   {
//     double r_sign = (r(0) > 0) - (r(0) < 0);
//     r(0) = r_sat * r_sign;
//   }

//   // TODO: Saturate r
//   if (use_range_)
//     measUpdate(r, z.R, H);

//   if (enable_log_)
//   {
//     logs_[LOG_RANGE_RES]->log(z.t);
//     logs_[LOG_RANGE_RES]->logVectors(r, z.z, zhat);
//   }


  ////////////////////// 
  //end of range update
}

}
