// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief The septentrio_gnss_driver publishes various DO-NOT-USE values in its output, which are usually -2*10^10.
 * Such values are unsuitable for downstream ROS processing. This node filters these values out and replaces them with
 * more suitable values, or throws away whole messages if they would be invalid from the ROS point of view.
 *
 * It also creates alternative message types from the attitude message: compass_msgs/Azimuth (for generic azimuth
 * processing nodes) and sensor_msgs/Imu (for robot_localization). The IMU message contains only ENU heading (and maybe
 * pitch), but the third angle is always reported as zero. Covariance of the invalid angles is set to (2*pi)^2. Also,
 * the angular velocities are added to this IMU message. Accelerations are not used.
 */

#include <limits>
#include <memory>
#include <string>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <gps_common/GPSFix.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gprmc.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <septentrio_gnss_driver/AIMPlusStatus.h>
#include <septentrio_gnss_driver/AttEuler.h>
#include <septentrio_gnss_driver/AttCovEuler.h>
#include <septentrio_gnss_driver/PosCovGeodetic.h>
#include <septentrio_gnss_driver/PVTGeodetic.h>
#include <septentrio_gnss_driver/RFStatus.h>
#include <septentrio_gnss_driver/VelCovGeodetic.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace cras
{
struct SeptentrioProcess : public cras::Nodelet
{
  void onInit() override
  {
    this->nh = this->getNodeHandle();
    this->pnh = this->getPrivateNodeHandle();

    auto params = this->privateParams();

    // These maximum errors are used instead of do-not-use values or too large errors.
    this->maxPosErr = params->getParam("max_position_error", this->maxPosErr, "m");
    this->maxAltErr = params->getParam("max_altitude_error", this->maxAltErr, "m");
    this->maxAngErr = params->getParam("max_orientation_error", this->maxAngErr, "rad");
    this->maxAngCov = this->maxAngErr * this->maxAngErr;
    this->maxHorzVelErr = params->getParam("max_horizontal_velocity_error", this->maxHorzVelErr, "m/s");
    this->maxVertVelErr = params->getParam("max_vertical_velocity_error", this->maxVertVelErr, "m/s");
    // Max clock bias converted to error in velocity determination
    this->maxClkBiasErr = params->getParam("max_clock_bias_error", this->maxClkBiasErr, "m/s");

    // These thresholds decide acceptable errors for letting the data through marked as valid for navigation.
    this->validPosErrorThreshold = params->getParam(
      "valid_position_error_threshold", this->validPosErrorThreshold, "m");
    this->validHeadingErrorThresholdDeg = params->getParam(
      "valid_heading_error_threshold_deg", this->validHeadingErrorThresholdDeg, "deg");

    this->publishInvalidFix = params->getParam("publish_invalid_fix", this->publishInvalidFix);
    this->publishInvalidHeading = params->getParam("publish_invalid_heading", this->publishInvalidHeading);

    this->aimPlusStatusPub = this->nh.advertise<septentrio_gnss_driver::AIMPlusStatus>("aimplusstatus", 10);
    this->aimPlusStatusSub = this->nh.subscribe(
      "raw/aimplusstatus", 10, &SeptentrioProcess::processAimPlusStatus, this);

    this->attEulerPub = this->nh.advertise<septentrio_gnss_driver::AttEuler>("atteuler", 10);
    this->attEulerSub = this->nh.subscribe("raw/atteuler", 10, &SeptentrioProcess::processAtt, this);

    this->attCovEulerPub = this->nh.advertise<septentrio_gnss_driver::AttCovEuler>("attcoveuler", 10);
    this->attCovEulerSub = this->nh.subscribe("raw/attcoveuler", 10, &SeptentrioProcess::processAttCov, this);

    this->fixPub = this->nh.advertise<sensor_msgs::NavSatFix>("fix", 10);
    this->fixSub = this->nh.subscribe("raw/fix", 10, &SeptentrioProcess::processFix, this);

    this->fixDetailPub = this->nh.advertise<gps_common::GPSFix>("fix_detail", 10);
    this->fixDetailSub = this->nh.subscribe("raw/fix_detail", 10, &SeptentrioProcess::processFixDetail, this);

    this->gpggaPub = this->nh.advertise<nmea_msgs::Gpgga>("gpgga", 10);
    this->gpggaSub = this->nh.subscribe("raw/gpgga", 10, &SeptentrioProcess::processGpgga, this);

    this->gprmcPub = this->nh.advertise<nmea_msgs::Gprmc>("gprmc", 10);
    this->gprmcSub = this->nh.subscribe("raw/gprmc", 10, &SeptentrioProcess::processGprmc, this);

    this->gpstPub = this->nh.advertise<sensor_msgs::TimeReference>("gpst", 10);
    this->gpstSub = this->nh.subscribe("raw/gpst", 10, &SeptentrioProcess::processGpst, this);

    this->posCovGeodeticPub = this->nh.advertise<septentrio_gnss_driver::PosCovGeodetic>("poscovgeodetic", 10);
    this->posCovGeodeticSub = this->nh.subscribe(
      "raw/poscovgeodetic", 10, &SeptentrioProcess::processPosCovGeodetic, this);

    this->pvtGeodeticPub = this->nh.advertise<septentrio_gnss_driver::PVTGeodetic>("pvtgeodetic", 10);
    this->pvtGeodeticSub = this->nh.subscribe("raw/pvtgeodetic", 10, &SeptentrioProcess::processPVTGeodetic, this);

    this->rfStatusPub = this->nh.advertise<septentrio_gnss_driver::RFStatus>("rfstatus", 10);
    this->rfStatusSub = this->nh.subscribe("raw/rfstatus", 10, &SeptentrioProcess::processRFStatus, this);

    this->velCovGeodeticPub = this->nh.advertise<septentrio_gnss_driver::VelCovGeodetic>("velcovgeodetic", 10);
    this->velCovGeodeticSub = this->nh.subscribe(
      "raw/velcovgeodetic", 10, &SeptentrioProcess::processVelCovGeodetic, this);

    this->azimuthPub = this->nh.advertise<compass_msgs::Azimuth>("azimuth", 10);
    this->imuPub = this->nh.advertise<sensor_msgs::Imu>("azimuth_imu", 10);
  }

  template<typename T>
  bool fixNan(T& val) const
  {
    if (!std::isnan(val) && val < -1000000000.0)
      val = std::numeric_limits<T>::quiet_NaN();
    return !std::isfinite(val);
  }

  template<typename T>
  bool fixCov(T& val, const T& maxCov) const
  {
    if (std::isnan(val) || val < -1000000000.0)
      val = maxCov;
    return val >= maxCov;
  }

  template<typename T>
  bool fixLinCov(T& val, const T& maxVal) const
  {
    return fixCov(val, maxVal * maxVal);
  }

  template<typename T>
  bool fixAngCov(T& val) const
  {
    return fixCov(val, static_cast<T>(maxAngCov));
  }

  template<typename T>
  bool fixLLACov(boost::array<T, 9>& val) const
  {
    // lat, lon, alt covariance is in meters even for geodetic coordinates
    bool invalid {false};
    invalid |= fixCov(val[0], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[1], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[2], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[3], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[4], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[5], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[6], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[7], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[8], static_cast<T>(maxAltErr * maxAltErr));
    return invalid;
  }

  void processAimPlusStatus(const septentrio_gnss_driver::AIMPlusStatus& msg) const
  {
    this->updateThreadName();
    this->aimPlusStatusPub.publish(msg);
  }

  void processAtt(const septentrio_gnss_driver::AttEulerConstPtr& msg) const
  {
    this->updateThreadName();
    // Synchronize atteuler and attcoveuler topics: both come almost at the same time and at low frequency, so the
    // standard message_filters approximate policy synchronizer would introduce very large delays. This is a much
    // more trivial synchronization, but it should work well for the type of data coming from Septentrio.
    if (this->lastAttCov != nullptr && msg->block_header.tow == this->lastAttCov->block_header.tow)
    {
      this->processHeading(msg, this->lastAttCov);
      this->lastAttCov.reset();
    }
    else
      this->lastAtt = msg;
  }

  void processAttCov(const septentrio_gnss_driver::AttCovEulerConstPtr& msg) const
  {
    this->updateThreadName();
    if (this->lastAtt != nullptr && msg->block_header.tow == this->lastAtt->block_header.tow)
    {
      this->processHeading(this->lastAtt, msg);
      this->lastAtt.reset();
    }
    else
      this->lastAttCov = msg;
  }

  void processHeading(const septentrio_gnss_driver::AttEulerConstPtr& att,
    const septentrio_gnss_driver::AttCovEulerConstPtr& cov) const
  {
    this->updateThreadName();
    bool invalid {false};
    auto outAtt = *att;
    invalid |= fixNan(outAtt.heading);
    fixNan(outAtt.heading_dot);
    fixNan(outAtt.roll);
    fixNan(outAtt.roll_dot);
    fixNan(outAtt.pitch);
    fixNan(outAtt.pitch_dot);

    auto outCov = *cov;
    invalid |= fixAngCov(outCov.cov_headhead);
    fixAngCov(outCov.cov_headroll);
    fixAngCov(outCov.cov_headpitch);
    fixAngCov(outCov.cov_pitchpitch);
    fixAngCov(outCov.cov_pitchroll);
    fixAngCov(outCov.cov_rollroll);

    invalid |= (outCov.cov_headhead > validHeadingErrorThresholdDeg * validHeadingErrorThresholdDeg);

    if (!this->publishInvalidHeading && invalid)
      return;

    this->attEulerPub.publish(outAtt);
    this->attCovEulerPub.publish(outCov);

    // Create the alternative heading messages

    const auto conv = M_PI * M_PI / (180.0 * 180.0);

    compass_msgs::Azimuth az;
    az.header = att->header;
    az.azimuth = outAtt.heading * M_PI / 180.0;
    az.orientation = compass_msgs::Azimuth::ORIENTATION_ENU;
    az.reference = compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC;
    az.unit = compass_msgs::Azimuth::UNIT_RAD;
    az.variance = outCov.cov_headhead * conv;

    this->azimuthPub.publish(az);

    sensor_msgs::Imu imu;
    imu.header = att->header;
    imu.orientation_covariance.fill(maxAngCov);
    // Imu message spec says -1 means not measured, 0 means covariance unknown
    imu.angular_velocity_covariance.fill(0);
    imu.linear_acceleration_covariance.fill(-1);

    imu.angular_velocity.x = outAtt.pitch_dot * M_PI / 180.0;
    imu.angular_velocity.y = outAtt.roll_dot * M_PI / 180.0;
    imu.angular_velocity.z = outAtt.heading_dot * M_PI / 180.0;

    // roll angle is not measured
    tf2::Quaternion q;
    q.setRPY(
      0,
      std::isnan(outAtt.pitch) ? 0 : outAtt.pitch * M_PI / 180.0,
      std::isnan(outAtt.heading) ? 0 : outAtt.heading * M_PI / 180.0);
    imu.orientation = tf2::toMsg(q);

    imu.orientation_covariance[0] = outCov.cov_pitchpitch * conv;
    imu.orientation_covariance[1] = this->maxAngErr;
    imu.orientation_covariance[2] = outCov.cov_headpitch * conv;
    imu.orientation_covariance[3] = this->maxAngErr;
    imu.orientation_covariance[4] = this->maxAngErr;
    imu.orientation_covariance[5] = this->maxAngErr;
    imu.orientation_covariance[6] = outCov.cov_headpitch * conv;
    imu.orientation_covariance[7] = this->maxAngErr;
    imu.orientation_covariance[8] = outCov.cov_headhead * conv;

    this->imuPub.publish(imu);
  }

  void processFix(const sensor_msgs::NavSatFix& msg) const
  {
    this->updateThreadName();
    auto outMsg = msg;
    bool invalid {false};
    invalid |= fixNan(outMsg.latitude);
    invalid |= fixNan(outMsg.longitude);
    invalid |= fixNan(outMsg.altitude);
    fixLLACov(outMsg.position_covariance);

    const auto maxCov = validPosErrorThreshold * validPosErrorThreshold;
    invalid |= (outMsg.position_covariance[0] > maxCov) || (outMsg.position_covariance[4] > maxCov);

    // Consumers of fix messags should check status and if it is NO_FIX, they should not use the message.
    if (invalid)
    {
      outMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      if (!this->publishInvalidFix)
        return;
    }

    this->fixPub.publish(outMsg);
  }

  void processFixDetail(const gps_common::GPSFix& msg) const
  {
    this->updateThreadName();
    bool invalid {false};
    auto outMsg = msg;
    invalid |= fixNan(outMsg.latitude);
    invalid |= fixNan(outMsg.longitude);
    invalid |= fixNan(outMsg.altitude);
    fixNan(outMsg.track);
    fixNan(outMsg.speed);
    fixNan(outMsg.climb);
    fixNan(outMsg.pitch);
    fixNan(outMsg.roll);
    invalid |= fixNan(outMsg.dip);
    fixNan(outMsg.time);
    fixNan(outMsg.gdop);
    fixNan(outMsg.pdop);
    fixNan(outMsg.hdop);
    fixNan(outMsg.vdop);
    fixNan(outMsg.tdop);
    invalid |= fixLinCov(outMsg.err, maxPosErr);
    invalid |= fixLinCov(outMsg.err_horz, maxPosErr);
    invalid |= fixLinCov(outMsg.err_vert, maxAltErr);
    fixLinCov(outMsg.err_track, 10.0);
    fixLinCov(outMsg.err_speed, 10.0);
    fixLinCov(outMsg.err_climb, 10.0);
    fixLinCov(outMsg.err_time, ros::Time::now().toSec());
    fixAngCov(outMsg.err_pitch);
    fixAngCov(outMsg.err_roll);
    invalid |= fixAngCov(outMsg.err_dip);
    invalid |= fixLLACov(outMsg.position_covariance);

    const auto maxCov = validPosErrorThreshold * validPosErrorThreshold;
    invalid |= (outMsg.position_covariance[0] > maxCov) || (outMsg.position_covariance[4] > maxCov);

    if (invalid && !this->publishInvalidFix)
      return;

    this->fixDetailPub.publish(outMsg);
  }

  void processGpgga(const nmea_msgs::Gpgga& msg) const
  {
    this->updateThreadName();
    auto outMsg = msg;
    fixNan(outMsg.lat);
    fixNan(outMsg.lon);
    fixNan(outMsg.alt);
    fixNan(outMsg.hdop);
    fixNan(outMsg.undulation);

    this->gpggaPub.publish(outMsg);
  }

  void processGprmc(const nmea_msgs::Gprmc& msg) const
  {
    this->updateThreadName();
    auto outMsg = msg;
    fixNan(outMsg.lat);
    fixNan(outMsg.lon);
    fixNan(outMsg.speed);
    fixNan(outMsg.track);
    fixNan(outMsg.mag_var);

    this->gprmcPub.publish(outMsg);
  }

  void processPosCovGeodetic(const septentrio_gnss_driver::PosCovGeodetic& msg) const
  {
    this->updateThreadName();
    auto outMsg = msg;
    const auto MAX_BIAS_ERR = ros::Time::now().toSec();
    fixCov(outMsg.cov_latlat, static_cast<float>(maxPosErr * maxPosErr));
    fixCov(outMsg.cov_bb, static_cast<float>(MAX_BIAS_ERR * MAX_BIAS_ERR));
    fixCov(outMsg.cov_latlon, static_cast<float>(maxPosErr * maxPosErr));
    fixCov(outMsg.cov_lathgt, static_cast<float>(maxPosErr * maxAltErr));
    fixCov(outMsg.cov_latb, static_cast<float>(maxPosErr * MAX_BIAS_ERR));
    fixCov(outMsg.cov_lonhgt, static_cast<float>(maxPosErr * maxAltErr));
    fixCov(outMsg.cov_lonb, static_cast<float>(maxPosErr * MAX_BIAS_ERR));
    fixCov(outMsg.cov_hb, static_cast<float>(maxAltErr * MAX_BIAS_ERR));

    this->posCovGeodeticPub.publish(outMsg);
  }

  void processPVTGeodetic(const septentrio_gnss_driver::PVTGeodetic& msg) const
  {
    this->updateThreadName();
    auto outMsg = msg;
    fixNan(outMsg.latitude);
    fixNan(outMsg.longitude);
    fixNan(outMsg.height);
    fixNan(outMsg.undulation);
    fixNan(outMsg.vn);
    fixNan(outMsg.ve);
    fixNan(outMsg.vu);
    fixNan(outMsg.cog);
    fixNan(outMsg.rx_clk_bias);
    fixNan(outMsg.rx_clk_drift);

    this->pvtGeodeticPub.publish(outMsg);
  }

  void processGpst(const sensor_msgs::TimeReference& msg) const
  {
    this->updateThreadName();
    this->gpstPub.publish(msg);
  }

  void processRFStatus(const septentrio_gnss_driver::RFStatus& msg) const
  {
    this->updateThreadName();
    this->rfStatusPub.publish(msg);
  }

  void processVelCovGeodetic(const septentrio_gnss_driver::VelCovGeodetic& msg) const
  {
    this->updateThreadName();
    auto outMsg = msg;
    fixCov(outMsg.cov_vnvn, static_cast<float>(maxHorzVelErr * maxHorzVelErr));
    fixCov(outMsg.cov_veve, static_cast<float>(maxHorzVelErr * maxHorzVelErr));
    fixCov(outMsg.cov_vuvu, static_cast<float>(maxVertVelErr * maxVertVelErr));
    fixCov(outMsg.cov_dtdt, static_cast<float>(maxClkBiasErr * maxClkBiasErr));
    fixCov(outMsg.cov_vnve, static_cast<float>(maxHorzVelErr * maxHorzVelErr));
    fixCov(outMsg.cov_vnvu, static_cast<float>(maxHorzVelErr * maxVertVelErr));
    fixCov(outMsg.cov_vndt, static_cast<float>(maxHorzVelErr * maxClkBiasErr));
    fixCov(outMsg.cov_vevu, static_cast<float>(maxHorzVelErr * maxVertVelErr));
    fixCov(outMsg.cov_vedt, static_cast<float>(maxHorzVelErr * maxClkBiasErr));
    fixCov(outMsg.cov_vudt, static_cast<float>(maxVertVelErr * maxClkBiasErr));

    this->velCovGeodeticPub.publish(outMsg);
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  ros::Subscriber aimPlusStatusSub;
  ros::Publisher aimPlusStatusPub;

  ros::Subscriber attEulerSub;
  ros::Publisher attEulerPub;

  ros::Subscriber attCovEulerSub;
  ros::Publisher attCovEulerPub;

  ros::Subscriber fixSub;
  ros::Publisher fixPub;

  ros::Subscriber fixDetailSub;
  ros::Publisher fixDetailPub;

  ros::Subscriber gpggaSub;
  ros::Publisher gpggaPub;

  ros::Subscriber gprmcSub;
  ros::Publisher gprmcPub;

  ros::Subscriber gpstSub;
  ros::Publisher gpstPub;

  ros::Subscriber posCovGeodeticSub;
  ros::Publisher posCovGeodeticPub;

  ros::Subscriber pvtGeodeticSub;
  ros::Publisher pvtGeodeticPub;

  ros::Subscriber rfStatusSub;
  ros::Publisher rfStatusPub;

  ros::Subscriber velCovGeodeticSub;
  ros::Publisher velCovGeodeticPub;

  ros::Publisher azimuthPub;
  ros::Publisher imuPub;

  mutable septentrio_gnss_driver::AttEulerConstPtr lastAtt;
  mutable septentrio_gnss_driver::AttCovEulerConstPtr lastAttCov;

  // These maximum errors are used instead of do-not-use values or too large errors.

  double maxPosErr {40000000};  //!< Maximum error in cartesian position [m]. Default is 40 000 km, Earth circumference.
  double maxAltErr {100000};  //!< Maximum error in altitude [m]. Default is 100 km.
  double maxAngErr {2 * M_PI};  //!< Maximum error in orientation angles [rad].
  double maxAngCov {maxAngErr * maxAngErr};

  double maxHorzVelErr {10.0};  //!< Maximum error in horizontal velocity [m/s].
  double maxVertVelErr {10.0};  //!< Maximum error in vertical velocity [m/s].
  double maxClkBiasErr {10.0};  //!< Maximum error in clock bias converted to velocity error [m/s].

  // These thresholds decide acceptable errors for letting the data through marked as valid for navigation.

  double validPosErrorThreshold {0.15};  //!< Position measurements with larger error (in meters) will be discarded.
  double validHeadingErrorThresholdDeg {5.0};  //!< Heading measurements with larger error will be discarded.

  bool publishInvalidFix {true};  //!< Whether to publish invalid fix messages (with NO_FIX status or NaNs).
  bool publishInvalidHeading {false};  //!< Whether to publish invalid heading messages (with NaNs or high covariance).
};

}

PLUGINLIB_EXPORT_CLASS(cras::SeptentrioProcess, nodelet::Nodelet)
