#include "ekf/ekf.h"
#define T transpose()
using namespace Eigen;

namespace roscopter::ekf
{
  const dxMat EKF::I_BIG = dxMat::Identity();

  EKF::EKF() :
    xbuf_(100)
  {}

  EKF::~EKF()
  {
    for (int i = 0; i < NUM_LOGS; i++)
      delete logs_[i];
  }

  void EKF::load(const std::string &filename)
  {
    // Constant Parameters
    get_yaml_eigen("p_b2g", filename, p_b2g_);
    get_yaml_diag("Qx", filename, Qx_);
    get_yaml_diag("P0", filename, P());
    P0_yaw_ = P()(ErrorState::DQ + 2, ErrorState::DQ + 2);
    get_yaml_diag("R_zero_vel", filename, R_zero_vel_);

    // Measurement Flags
    get_yaml_node("use_mocap", filename, use_mocap_);
    get_yaml_node("use_gnss", filename, use_gnss_);
    get_yaml_node("use_baro", filename, use_baro_);
    get_yaml_node("use_compassing", filename, use_compassing_);
    get_yaml_node("use_zero_vel", filename, use_zero_vel_);

    // Armed Check
    get_yaml_node("enable_arm_check", filename, enable_arm_check_);
    get_yaml_node("is_flying_threshold", filename, is_flying_threshold_);

    // load initial state
    double ref_heading;
    get_yaml_node("ref_heading", filename, ref_heading);
    q_n2I_ = quat::Quatd::from_euler(0, 0, M_PI/180.0 * ref_heading);

    ref_lla_set_ = false;
    bool manual_ref_lla;
    get_yaml_node("manual_ref_lla", filename, manual_ref_lla);
    if (manual_ref_lla)
    {
      std::cout << "common ref lla won't work with manual ref lla" << std::endl;
      Vector3d ref_lla;
      get_yaml_eigen("ref_lla", filename, ref_lla);
      std::cout << "Set ref lla: " << ref_lla.transpose() << std::endl;
      ref_lla.head<2>() *= M_PI/180.0; // convert to rad
      xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
      x_e2I_.t() = x_e2n.t();
      x_e2I_.q() = x_e2n.q() * q_n2I_; //??? not sure what this does.

      // initialize the estimated ref altitude state
      // x().ref = ref_lla(2);
      ref_lat_radians_ = ref_lla(0);
      ref_lon_radians_ = ref_lla(1);

      ref_lla_set_ = true;
    }

    ground_pressure_ = 0.;
    ground_temperature_ = 0.;
    update_baro_ = false;
    get_yaml_node("update_baro_velocity_threshold", filename, update_baro_vel_thresh_);

    get_yaml_eigen("x0", filename, x0_.arr());

    initLog(filename);
  }

  void EKF::initLog(const std::string &filename)
  {
    get_yaml_node("enable_log", filename, enable_log_);
    get_yaml_node("log_prefix", filename, log_prefix_);

    std::experimental::filesystem::create_directories(log_prefix_);

    logs_.resize(NUM_LOGS);
    for (int i = 0; i < NUM_LOGS; i++)
      logs_[i] = new Logger(log_prefix_ + "/" + log_names_[i] + ".bin");
  }

  void EKF::initialize(double t)
  {
    x().t = t;
    x().x = x0_;
    x().v.setZero();
    x().ba.setZero();
    x().bg.setZero();
    x().bb = 0.;
    // if (ref_lla_set_)
    //   x().ref = x().ref;
    // else
    //   x().ref = 0.;
    x().a = -gravity;
    x().w.setZero();
    is_flying_ = false;
    armed_ = false;
  }

  void EKF::propagate(const double &t, const Vector6d &imu, const Matrix6d &R)
  {
    if (std::isnan(x().t))
    {
      initialize(t);
      return;
    }

    double dt = t - x().t;
    // TODO figure out the assert statement
    // assert(dt >= 0);
    // is it okay to skip propagation rather than break the ekf?
    if (dt < 0.0)
      std::cerr << "dt < 0.0";
    if (dt < 1e-6)
      return;
    dynamics(x(), imu, dx_, true);

    // do the state propagation
    xbuf_.next().x = x() + dx_ * dt;
    xbuf_.next().x.t = t;
    xbuf_.next().x.imu = imu;

    // discretize jacobians (first order)
    A_ = I_BIG + A_*dt;
    B_ = B_*dt;
    CHECK_NAN(P());
    CHECK_NAN(A_);
    CHECK_NAN(B_);
    CHECK_NAN(Qx_);
    xbuf_.next().P = A_*P()*A_.T + B_*R*B_.T + Qx_*dt*dt; // covariance propagation
    CHECK_NAN(xbuf_.next().P);
    xbuf_.advance();
    Qu_ = R; // copy because we might need it later.

    if (enable_log_)
    {
      logs_[LOG_STATE]->logVectors(x().arr, x().q.euler());
      logs_[LOG_COV]->log(x().t);
      logs_[LOG_COV]->logVectors(P());
      logs_[LOG_IMU]->log(t);
      logs_[LOG_IMU]->logVectors(imu);
    }
  }

  bool EKF::measUpdate(const VectorXd &res, const MatrixXd &R, const MatrixXd &H)
  {
    int size = res.rows();
    auto K = K_.leftCols(size);
    ///TODO: perform covariance gating
    MatrixXd innov = (H*P()*H.T + R).inverse();
    CHECK_NAN(H); CHECK_NAN(R); CHECK_NAN(P());
    K = P() * H.T * innov;
    CHECK_NAN(K);

    x() += K * res;
    dxMat ImKH = I_BIG - K*H;
    P() = ImKH*P()*ImKH.T + K*R*K.T;
    CHECK_NAN(P());
    return true;
  }

  void EKF::setRefLla(Vector3d ref_lla)
  {
    if (!ref_lla_set_)
    {
      std::cout << "Set ref lla: " << ref_lla.transpose() << std::endl;
      ref_lla.head<2>() *= M_PI/180.0; // convert to rad
      xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
      x_e2I_.t() = x_e2n.t();
      x_e2I_.q() = x_e2n.q() * q_n2I_;

      // initialize the estimated ref altitude state
      x().ref = ref_lla(2);
      ref_lat_radians_ = ref_lla(0);
      ref_lon_radians_ = ref_lla(1);

      ref_lla_set_ = true;
    }
  }

  void EKF::setGroundTempPressure(const double& temp, const double& press)
  {
    ground_temperature_ = temp;
    ground_pressure_ = press;
  }

  void EKF::checkIsFlying()
  {
    bool okay_to_check = enable_arm_check_ ? armed_ : true;
    if (okay_to_check && x().a.norm() > is_flying_threshold_)
    {
      std::cout << "Now Flying!  Go Go Go!" << std::endl;
      is_flying_ = true;
    }
  }
}
