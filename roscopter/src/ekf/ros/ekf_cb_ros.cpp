
#include <ros/package.h>

#include "ekf/state.h"
#include "ekf/ekf_ros.h"
#include "roscopter_utils/yaml.h"
#include "roscopter_utils/gnss.h"

using namespace Eigen;

namespace roscopter::ekf
{
  void EKF_ROS::imuCallback(const sensor_msgs::ImuConstPtr &msg)
  {
    if (start_time_.sec == 0)
    {
      start_time_ = msg->header.stamp;
    }

    Vector6d z;
    z << msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z,
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z;

    double t = (msg->header.stamp - start_time_).toSec();
    ekf_.imuCallback(t, z, imu_R_);

    if(ros_initialized_)
      publishEstimates(msg);
  }

  void EKF_ROS::baroCallback(const rosflight_msgs::BarometerConstPtr& msg)
  {

    const double pressure_meas = msg->pressure;
    const double temperature_meas = msg->temperature;

    if (!ekf_.groundTempPressSet())
    {
      std::cout << "Set ground pressure and temp" << std::endl;
      std::cout << "press: " << pressure_meas << std::endl;
      ekf_.setGroundTempPressure(temperature_meas, pressure_meas);
    }

    if (start_time_.sec == 0)
      return;

    const double t = (msg->header.stamp - start_time_).toSec();
    ekf_.baroUpdate(t, pressure_meas, baro_R_, temperature_meas);
  }

  void EKF_ROS::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    xform::Xformd z;
    z.arr_ << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z;

    mocapCallback(msg->header.stamp, z);
  }

  void EKF_ROS::odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    xform::Xformd z;
    z.arr_ << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z,
              msg->pose.pose.orientation.w,
              msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z;

    mocapCallback(msg->header.stamp, z);
  }

  void EKF_ROS::mocapCallback(const ros::Time &time, const xform::Xformd &z)
  {

    if (start_time_.sec == 0)
      return;

    double t = (time - start_time_).toSec();
    ekf_.mocapUpdate(t, z, mocap_R_);
  }

  void EKF_ROS::compassingCallback(const ros::Time &time, const double &z)
  {
    if (start_time_.sec == 0)
      return;

    const double t = (time - start_time_).toSec();
    ekf_.compassingUpdate(t, z, compassing_R_);
  }

  void EKF_ROS::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
  {

    if (msg->armed)
    {
      ekf_.setArmed();
    }
    else
    {
      ekf_.setDisarmed();
    }
  }

  void EKF_ROS::commonRefLlaCallback(const rosflight_msgs::GNSSConstPtr &msg)
  {
	  std::cout<<"In common ref lla callback \n";
	  Eigen::Vector3d ref_lla{msg->position[0],msg->position[1],msg->position[2]};
    ekf_.setRefLla(ref_lla);
  }


  void EKF_ROS::gnssCallback(const rosflight_msgs::GNSSConstPtr &msg)
  {

    Vector6d z;
    z << msg->position[0],
        msg->position[1],
        msg->position[2],
        msg->velocity[0],
        msg->velocity[1],
        msg->velocity[2];

    // rotate covariance into the ECEF frame
    Vector6d Sigma_diag_NED;
    if (manual_gps_noise_)
    {
      Sigma_diag_NED << gps_horizontal_stdev_,
                        gps_horizontal_stdev_,
                        gps_vertical_stdev_,
                        gps_speed_stdev_,
                        gps_speed_stdev_,
                        gps_speed_stdev_;
    }
    else
    {
      Sigma_diag_NED << msg->horizontal_accuracy,
                    msg->horizontal_accuracy,
                    msg->vertical_accuracy,
                    msg->speed_accuracy,
                    msg->speed_accuracy,
                    msg->speed_accuracy;
    }
    Sigma_diag_NED = Sigma_diag_NED.cwiseProduct(Sigma_diag_NED);

    Matrix3d R_e2n = q_e2n(ecef2lla(z.head<3>())).R();

    Matrix6d Sigma_ecef;
    Sigma_ecef << R_e2n.transpose() * Sigma_diag_NED.head<3>().asDiagonal() * R_e2n, Matrix3d::Zero(),
                  Matrix3d::Zero(), R_e2n.transpose() *  Sigma_diag_NED.tail<3>().asDiagonal() * R_e2n;

    if (!ekf_.refLlaSet())
    {
      // set ref lla to first gps position
      Eigen::Vector3d ref_lla = ecef2lla(z.head<3>());
      // Convert radians to degrees
      ref_lla.head<2>() *= 180. / M_PI;
      ekf_.setRefLla(ref_lla);
      rosflight_msgs::GNSS common_ref_lla;
      common_ref_lla.position[0] = ref_lla[0];
      common_ref_lla.position[1] = ref_lla[1];
      common_ref_lla.position[2] = ref_lla[2];
      ref_lla_pub_.publish(common_ref_lla);
    }

    if (start_time_.sec == 0)
      return;

    double t = (msg->header.stamp - start_time_).toSec();
    ekf_.gnssCallback(t, z, Sigma_ecef);
  }

  double EKF_ROS::wrap(double psi, double wrapAngle)
  {
    double psiWrapped{std::fmod(psi,2.0*M_PI)};
    if (psiWrapped > wrapAngle)
        psiWrapped = -(2.0*M_PI-psiWrapped);
    return psiWrapped;
  }

  #ifdef UBLOX
  void EKF_ROS::gnssCallbackUblox(const ublox::PosVelEcefConstPtr &msg)
  {

    //only uses the data if gnss is fixed as 2D or 3D
    if (msg->fix == ublox::PosVelEcef::FIX_TYPE_2D
        || msg->fix == ublox::PosVelEcef::FIX_TYPE_3D)
    {
      rosflight_msgs::GNSS rf_msg;
      rf_msg.header.stamp = msg->header.stamp;
      rf_msg.position = msg->position;
      rf_msg.velocity = msg->velocity;
      rf_msg.horizontal_accuracy = msg->horizontal_accuracy;
      rf_msg.vertical_accuracy = msg->vertical_accuracy;
      rf_msg.speed_accuracy = msg->speed_accuracy;
      gnssCallback(boost::make_shared<rosflight_msgs::GNSS>(rf_msg));
    }
    else
    {
      ROS_WARN_THROTTLE(1., "Ublox GPS not in fix");
    }
  }

  void EKF_ROS::gnssCallbackRelPos(const ublox::RelPosConstPtr &msg)
  {
    //TODO:: put in logic to only use measurements if a flag of 311, 279, 271, or ... xxx, is found
    //TODO:: maybe put in logic to only move forward if in a landing state
    base_relPos_msg_.header = msg->header;
    
    // negate relPos message to go from rover to base rather than base to rover
    base_relPos_msg_.point.x = -msg->relPosNED[0];
    base_relPos_msg_.point.y = -msg->relPosNED[1];
    base_relPos_msg_.point.z = -msg->relPosNED[2];  
    // //TODO:: could add in the high precision (portion less than a mm)
    // //TODO:: could add in the accuracy of the NED measurment to update covariance

    double accHeading = msg->accHeading;  //in radians
    if(!manual_compassing_noise_)
    {
      compassing_R_ = accHeading * accHeading;
    }
    double z = wrap(msg->relPosHeading, M_PI);
    

    //make some of these variables scoped to this function only.

    compassingCallback(msg->header.stamp, z);
    base_relPos_pub_.publish(base_relPos_msg_);

  }
  #endif

  #ifdef INERTIAL_SENSE
  void EKF_ROS::gnssCallbackInertialSense(const inertial_sense::GPSConstPtr &msg)
  {

    if (msg->fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_2D_FIX
        || msg->fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_3D_FIX)
    {
      rosflight_msgs::GNSS rf_msg;
      rf_msg.header.stamp = msg->header.stamp;
      rf_msg.position[0] = msg->posEcef.x;
      rf_msg.position[1] = msg->posEcef.y;
      rf_msg.position[2] = msg->posEcef.z;
      rf_msg.velocity[0] = msg->velEcef.x;
      rf_msg.velocity[1] = msg->velEcef.y;
      rf_msg.velocity[2] = msg->velEcef.z;
      rf_msg.horizontal_accuracy = msg->hAcc;
      rf_msg.vertical_accuracy = msg->vAcc;
      rf_msg.speed_accuracy = 0.3;
      gnssCallback(boost::make_shared<rosflight_msgs::GNSS>(rf_msg));
    }
    else
    {
      ROS_WARN_THROTTLE(1., "Inertial Sense GPS not in fix");
    }
  }
  #endif
}
