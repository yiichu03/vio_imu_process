#include "simul/curves.h"

#include <ceres/internal/autodiff.h>

#include <okvis/kinematics/sophus_operators.hpp>

namespace simul {
bool EnumFromString(std::string description, SimulatedTrajectoryType *e) {
  std::transform(description.begin(), description.end(), description.begin(),
                 ::toupper);
  std::unordered_map<std::string, SimulatedTrajectoryType> descriptionToId{
      {"SINUSOID", SimulatedTrajectoryType::Sinusoid},
      {"TORUS", SimulatedTrajectoryType::Torus},
      {"TORUS2", SimulatedTrajectoryType::Torus2},
      {"BALL", SimulatedTrajectoryType::Ball},
      {"SQUIRCLE", SimulatedTrajectoryType::Squircle},
      {"CIRCLE", SimulatedTrajectoryType::Circle},
      {"DOT", SimulatedTrajectoryType::Dot},
      {"WAVYCIRCLE", SimulatedTrajectoryType::WavyCircle},
      {"LEMNISCATE", SimulatedTrajectoryType::Lemniscate},
      {"LINESEGMENT", SimulatedTrajectoryType::LineSegment},
      {"MOTIONLESS", SimulatedTrajectoryType::Motionless},
      {"FROMTRAJCSV", SimulatedTrajectoryType::FromTrajCsv}};
  auto iter = descriptionToId.find(description);
  if (iter == descriptionToId.end()) {
    LOG(ERROR) << "Unknown trajectory type:" << description << ". Default to WavyCircle.";
    *e = SimulatedTrajectoryType::WavyCircle;
    return false;
  } else {
    *e = iter->second;
  }
  return true;
}

std::string EnumToString(SimulatedTrajectoryType e) {
  const std::string names[] = {
      "Sinusoid", "Torus", "Torus2",     "Ball",       "Squircle",
      "Circle",   "Dot",   "WavyCircle", "Lemniscate", "LineSegment", "Motionless", "FromTrajCsv"};
  return names[static_cast<int>(e)];
}

TrajectoryBase::TrajectoryBase(double samplingFreq, const Eigen::Vector3d &ginw)
    : freq(samplingFreq), interval(1 / freq), gw(ginw) {}

CircularSinusoidalTrajectory::CircularSinusoidalTrajectory(double _maxThetaZ)
    : TrajectoryBase(), wz(17 * M_PI / 41),
      wxy(7 * M_PI / 37),
      rz(1),
      rxy(61.0 / 19),
      maxThetaZ(_maxThetaZ) {}

CircularSinusoidalTrajectory::CircularSinusoidalTrajectory(double imuFreq,
                                                           Eigen::Vector3d ginw,
                                                           double _maxThetaZ)
    : TrajectoryBase(imuFreq, ginw), wz(17 * M_PI / 41),
      wxy(7 * M_PI / 37),
      rz(1),
      rxy(61.0 / 19),
      maxThetaZ(_maxThetaZ) {}

void TrajectoryBase::getTrueInertialMeasurements(
    const okvis::Time tStart, const okvis::Time tEnd,
    okvis::ImuMeasurementDeque& imuMeasurements) const {
  okvis::Time time = tStart;
  ImuMeasurementVector imuMeas;
  imuMeas.reserve((int)((tEnd - tStart).toSec() * freq + 1));
  for (; time < tEnd; time += okvis::Duration(interval)) {
    okvis::ImuMeasurement meas;
    meas.timeStamp = time;
    okvis::kinematics::Transformation T_WB = computeGlobalPose(time);
    meas.measurement.gyroscopes =
        T_WB.C().transpose() * computeGlobalAngularRate(time);
    meas.measurement.accelerometers =
        T_WB.C().transpose() * (computeGlobalLinearAcceleration(time) - gw);
    imuMeas.push_back(meas);
  }
  imuMeasurements = okvis::ImuMeasurementDeque(imuMeas.begin(), imuMeas.end());
}

void TrajectoryBase::getTrueInertialMeasurements(
    okvis::Time time, Eigen::Vector3d *gyroscope,
    Eigen::Vector3d *accelerometer) const {
  okvis::kinematics::Transformation T_WB = computeGlobalPose(time);
  *gyroscope = T_WB.C().transpose() * computeGlobalAngularRate(time);
  *accelerometer = T_WB.C().transpose() * (computeGlobalLinearAcceleration(time) - gw);
}

Eigen::Vector3d TrajectoryBase::computeLocalAngularVelocity(
    const okvis::Time time) const {
  okvis::kinematics::Transformation T_WB = computeGlobalPose(time);
  return T_WB.C().transpose() * computeGlobalAngularRate(time);
}

void TrajectoryBase::getTruePoses(
    const std::vector<okvis::Time> &times,
    Eigen::AlignedVector<okvis::kinematics::Transformation> &vT_WB) {
  vT_WB.clear();
  vT_WB.reserve(times.size());
  for (auto time : times) {
    vT_WB.push_back(computeGlobalPose(time));
  }
}

void TrajectoryBase::getTrueVelocities(
    const std::vector<okvis::Time> &times,
    Eigen::AlignedVector<Eigen::Vector3d> &velocities) {
  velocities.clear();
  velocities.reserve(times.size());
  for (auto time : times) {
    velocities.push_back(computeGlobalLinearVelocity(time));
  }
}

void TrajectoryBase::getSampleTimes(
    const okvis::Time tStart, const okvis::Time tEnd,
    std::vector<okvis::Time>& vTime) {
  okvis::Time time = tStart;
  vTime.clear();
  vTime.reserve((int)((tEnd - tStart).toSec() * freq + 1));
  for (; time < tEnd; time += okvis::Duration(interval)) vTime.push_back(time);
}

Eigen::Vector3d TrajectoryBase::computeGlobalAngularRateNumeric(
    const okvis::Time time) const {
  okvis::Duration h(0, 10000);
  okvis::kinematics::Transformation T_WBdelta =
      computeGlobalPose(time + h);
  okvis::kinematics::Transformation T_WB = computeGlobalPose(time);
  okvis::kinematics::Transformation T_WBmdelta =
      computeGlobalPose(time - h);
  Eigen::Matrix3d Rdelta = (T_WBdelta.C() - T_WBmdelta.C()) / (2 * h.toSec());
  Eigen::Matrix3d OmegaW = Rdelta * T_WB.C().transpose();
  return okvis::kinematics::vee(OmegaW);
}

Eigen::Vector3d
TrajectoryBase::computeGlobalLinearAccelerationNumeric(
    const okvis::Time time) const {
  double h = 1e-6;
  Eigen::Vector3d v_WBdelta =
      computeGlobalLinearVelocity(time + okvis::Duration(h));
  Eigen::Vector3d v_WBmdelta =
      computeGlobalLinearVelocity(time - okvis::Duration(h));
  return (v_WBdelta - v_WBmdelta) / (2 * h);
}

Eigen::Vector3d
TrajectoryBase::computeGlobalLinearVelocityNumeric(
    const okvis::Time time) const {
  double h = 1e-6;
  okvis::kinematics::Transformation T_WBdelta =
      computeGlobalPose(time + okvis::Duration(h));
  okvis::kinematics::Transformation T_WBmdelta =
      computeGlobalPose(time - okvis::Duration(h));
  return (T_WBdelta.r() - T_WBmdelta.r()) / (2 * h);
}

// compute angular rate in the global frame
Eigen::Vector3d CircularSinusoidalTrajectory::computeGlobalAngularRate(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double thetaZDot = maxThetaZ * cos(wz * dTime) * wz;

  double thetaXY = wxy * dTime;
  double wgx = -thetaZDot * sin(thetaXY);
  double wgy = thetaZDot * cos(thetaXY);
  double wgz = wxy;
  return Eigen::Vector3d(wgx, wgy, wgz);
}

Eigen::Vector3d CircularSinusoidalTrajectory::computeGlobalLinearAcceleration(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double cwzt = cos(wz * dTime);
  double thetaZ = maxThetaZ * swzt;
  double thetaZDot = maxThetaZ * cwzt * wz;
  double thetaZDDot = -maxThetaZ * swzt * wz * wz;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double r = rxy - rz + rz * cThetaZ;
  double rDot = -rz * sThetaZ * thetaZDot;
  double rDDot = -rz * (cThetaZ * thetaZDot * thetaZDot + thetaZDDot * sThetaZ);

  double thetaXY = wxy * dTime;
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);

  double xDDot =
      rDDot * cThetaXY - 2 * rDot * sThetaXY * wxy - r * cThetaXY * wxy * wxy;
  double yDDot =
      rDDot * sThetaXY + 2 * rDot * cThetaXY * wxy - r * sThetaXY * wxy * wxy;
  double zDDot = rz * (-sThetaZ * thetaZDot * thetaZDot + cThetaZ * thetaZDDot);
  return Eigen::Vector3d(xDDot, yDDot, zDDot);
}

Eigen::Vector3d CircularSinusoidalTrajectory::computeGlobalLinearVelocity(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double cwzt = cos(wz * dTime);
  double thetaZ = maxThetaZ * swzt;
  double thetaZDot = maxThetaZ * cwzt * wz;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double r = rxy - rz + rz * cThetaZ;
  double rDot = -rz * sThetaZ * thetaZDot;

  double thetaXY = wxy * dTime;
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);

  double xDot = rDot * cThetaXY - r * sThetaXY * wxy;
  double yDot = rDot * sThetaXY + r * cThetaXY * wxy;
  double zDot = rz * cThetaZ * thetaZDot;

  return Eigen::Vector3d(xDot, yDot, zDot);
}

okvis::kinematics::Transformation
CircularSinusoidalTrajectory::computeGlobalPose(const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  //  double cwzt = cos(wz * dTime);
  double thetaZ = maxThetaZ * swzt;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double r = rxy - rz + rz * cThetaZ;

  double thetaXY = wxy * dTime;
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);

  double x = r * cThetaXY;
  double y = r * sThetaXY;
  double z = rz * sThetaZ;
  return okvis::kinematics::Transformation(
      Eigen::Vector3d(x, y, z),
      Eigen::AngleAxisd(thetaXY, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(thetaZ, Eigen::Vector3d::UnitY()));
}

TorusTrajectory::TorusTrajectory(
    double imuFreq, Eigen::Vector3d ginw)
    : CircularSinusoidalTrajectory(imuFreq, ginw),
      wr(19 * M_PI / 137),
      xosc(rxy - rz) {}

Eigen::Vector3d TorusTrajectory::computeGlobalLinearAcceleration(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double cwzt = cos(wz * dTime);
  double thetaZ = maxThetaZ * swzt;
  double thetaZDot = maxThetaZ * cwzt * wz;
  double thetaZDDot = -maxThetaZ * swzt * wz * wz;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double thetar = wr * dTime;
  double sthetar = sin(thetar);
  double cthetar = cos(thetar);
  double r = xosc * cthetar + rz * cThetaZ;
  double rDot = -xosc * sthetar * wr - rz * sThetaZ * thetaZDot;
  double rDDot = -xosc * cthetar * wr * wr -
                 rz * (cThetaZ * thetaZDot * thetaZDot + thetaZDDot * sThetaZ);

  double thetaXY = wxy * dTime;
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);

  double xDDot =
      rDDot * cThetaXY - 2 * rDot * sThetaXY * wxy - r * cThetaXY * wxy * wxy;
  double yDDot =
      rDDot * sThetaXY + 2 * rDot * cThetaXY * wxy - r * sThetaXY * wxy * wxy;
  double zDDot = rz * (-sThetaZ * thetaZDot * thetaZDot + cThetaZ * thetaZDDot);
  return Eigen::Vector3d(xDDot, yDDot, zDDot);
}

Eigen::Vector3d TorusTrajectory::computeGlobalLinearVelocity(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double cwzt = cos(wz * dTime);
  double thetaZ = maxThetaZ * swzt;
  double thetaZDot = maxThetaZ * cwzt * wz;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double thetar = wr * dTime;
  double sthetar = sin(thetar);
  double cthetar = cos(thetar);
  double r = xosc * cthetar + rz * cThetaZ;
  double rDot = -xosc * sthetar * wr - rz * sThetaZ * thetaZDot;

  double thetaXY = wxy * dTime;
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);

  double xDot = rDot * cThetaXY - r * sThetaXY * wxy;
  double yDot = rDot * sThetaXY + r * cThetaXY * wxy;
  double zDot = rz * cThetaZ * thetaZDot;

  return Eigen::Vector3d(xDot, yDot, zDot);
}

okvis::kinematics::Transformation
TorusTrajectory::computeGlobalPose(const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double thetaZ = maxThetaZ * swzt;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double thetar = wr * dTime;
  double r = xosc * cos(thetar) + rz * cThetaZ;

  double thetaXY = wxy * dTime;
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);

  double x = r * cThetaXY;
  double y = r * sThetaXY;
  double z = rz * sThetaZ;
  return okvis::kinematics::Transformation(
      Eigen::Vector3d(x, y, z),
      Eigen::AngleAxisd(thetaXY, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(thetaZ, Eigen::Vector3d::UnitY()));
}

SphereTrajectory::SphereTrajectory(double _rxy, double _maxThetaZ)
    : CircularSinusoidalTrajectory(_maxThetaZ) {
  rxy = _rxy;
}

SphereTrajectory::SphereTrajectory(
    double imuFreq, Eigen::Vector3d ginw, double _rxy, double _maxThetaZ)
    : CircularSinusoidalTrajectory(imuFreq, ginw, _maxThetaZ) {
  rxy = _rxy;
}

Eigen::Vector3d SphereTrajectory::computeGlobalAngularRate(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double thetaZDot = maxThetaZ * cos(wz * dTime) * wz;

  double thetaXY = M_PI * (1.0 - cos(wxy * dTime));
  double wgx = -thetaZDot * sin(thetaXY);
  double wgy = thetaZDot * cos(thetaXY);
  double wgz = M_PI * sin(wxy * dTime) * wxy;
  return Eigen::Vector3d(wgx, wgy, wgz);
}

Eigen::Vector3d SphereTrajectory::computeGlobalLinearAcceleration(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double cwzt = cos(wz * dTime);
  double thetaZ = maxThetaZ * swzt;
  double thetaZDot = maxThetaZ * cwzt * wz;
  double thetaZDDot = -maxThetaZ * swzt * wz * wz;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double r = rxy - rz + rz * cThetaZ;
  double rDot = -rz * sThetaZ * thetaZDot;
  double rDDot = -rz * (cThetaZ * thetaZDot * thetaZDot + thetaZDDot * sThetaZ);

  double thetaXY = M_PI * (1.0 - cos(wxy * dTime));
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);
  double thetaXYDot = M_PI * sin(wxy * dTime) * wxy;
  double thetaXYDDot = M_PI * cos(wxy * dTime) * wxy * wxy;

  double xDDot = rDDot * cThetaXY - 2 * rDot * sThetaXY * thetaXYDot -
                 r * cThetaXY * thetaXYDot * thetaXYDot -
                 r * sThetaXY * thetaXYDDot;
  double yDDot = rDDot * sThetaXY + 2 * rDot * cThetaXY * thetaXYDot -
                 r * sThetaXY * thetaXYDot * thetaXYDot +
                 r * cThetaXY * thetaXYDDot;
  double zDDot = rz * (-sThetaZ * thetaZDot * thetaZDot + cThetaZ * thetaZDDot);
  return Eigen::Vector3d(xDDot, yDDot, zDDot);
}

Eigen::Vector3d SphereTrajectory::computeGlobalLinearVelocity(
    const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double cwzt = cos(wz * dTime);
  double thetaZ = maxThetaZ * swzt;
  double thetaZDot = maxThetaZ * cwzt * wz;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double r = rxy - rz + rz * cThetaZ;
  double rDot = -rz * sThetaZ * thetaZDot;

  double thetaXY = M_PI * (1.0 - cos(wxy * dTime));
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);
  double thetaXYDot = M_PI * sin(wxy * dTime) * wxy;

  double xDot = rDot * cThetaXY - r * sThetaXY * thetaXYDot;
  double yDot = rDot * sThetaXY + r * cThetaXY * thetaXYDot;
  double zDot = rz * cThetaZ * thetaZDot;

  return Eigen::Vector3d(xDot, yDot, zDot);
}

okvis::kinematics::Transformation
SphereTrajectory::computeGlobalPose(const okvis::Time time) const {
  double dTime = time.toSec();
  double swzt = sin(wz * dTime);
  double thetaZ = maxThetaZ * swzt;

  double sThetaZ = sin(thetaZ);
  double cThetaZ = cos(thetaZ);

  double r = rxy - rz + rz * cThetaZ;

  double thetaXY = M_PI * (1.0 - cos(wxy * dTime));
  double sThetaXY = sin(thetaXY);
  double cThetaXY = cos(thetaXY);

  double x = r * cThetaXY;
  double y = r * sThetaXY;
  double z = rz * sThetaZ;
  return okvis::kinematics::Transformation(
      Eigen::Vector3d(x, y, z),
      Eigen::AngleAxisd(thetaXY, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(thetaZ, Eigen::Vector3d::UnitY()));
}

void RoundedSquare::initDataStructures() {
  CHECK_GT(radius_, 1e-8);
  period_.fromSec((2 * M_PI * radius_ + 4 * sideLength_) / velocityNorm_);
  omega_ = velocityNorm_ / radius_;
  // compute endEpochs
  // anticlockwise, start from (sideLength / 2 + r, 0)
  std::vector<double> segments{sideLength_ * 0.5,    radius_ * 0.5 * M_PI,
                               sideLength_,          radius_ * 0.5 * M_PI,
                               sideLength_,          radius_ * 0.5 * M_PI,
                               sideLength_,
                               radius_ * 0.5 * M_PI,  // fourth quadrant
                               sideLength_ * 0.5};

  double distance = 0;
  for (size_t j = 0; j < segments.size(); ++j) {
    distance += segments[j];
    endEpochs_.emplace_back(distance / velocityNorm_);
  }
  CHECK_NEAR(endEpochs_.back(), period_.toSec(), 1e-6);
  endEpochs_[endEpochs_.size() - 1] = period_.toSec();

  double half_d = sideLength_ * 0.5;

  centers_ =
      std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>{
          {half_d, half_d},
          {-half_d, half_d},
          {-half_d, -half_d},
          {half_d, -half_d}};

  beginPoints_ =
      std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>{
          {half_d + radius_, 0},
          {half_d, half_d + radius_},
          {-half_d - radius_, half_d},
          {-half_d, -half_d - radius_},
          {half_d + radius_, -half_d}};
}

RoundedSquare::RoundedSquare(double imuFreq, Eigen::Vector3d ginw,
                             double radius,
                             double sideLength, double velocityNorm)
    : TrajectoryBase(imuFreq, ginw),
      radius_(radius),
      sideLength_(sideLength),
      velocityNorm_(velocityNorm) {
  initDataStructures();
}

okvis::kinematics::Transformation RoundedSquare::computeGlobalPose(
    const okvis::Time time) const {
  double remainder = getPeriodRemainder(time);
  size_t j;
  double delta_t;
  decideTimeSlot(remainder, &j, &delta_t);
  int half_j = j / 2;

  Eigen::Vector2d xny;
  double theta;         // body frame (FLU) in the world frame
  double motion_theta;  // radians elapsed by the circular motion

  Eigen::Matrix2d R_WB;
  Eigen::Vector2d v_B(velocityNorm_, 0);
  Eigen::Vector2d cs_theta;
  switch (j % 2) {
    case 0:
      theta = (half_j + 1) * M_PI * 0.5;
      R_WB = rotMat2d(theta);
      xny = beginPoints_[half_j] + R_WB * v_B * delta_t;
      break;
    case 1:
      motion_theta = M_PI * 0.5 * half_j + delta_t * omega_;
      cs_theta[0] = std::cos(motion_theta);
      cs_theta[1] = std::sin(motion_theta);
      xny = centers_[half_j] + radius_ * cs_theta;

      theta = motion_theta + M_PI * 0.5;
      R_WB = rotMat2d(theta);
      break;
    default:
      break;
  }
  Eigen::Matrix3d R_WB3d = Eigen::Matrix3d::Identity();
  R_WB3d.topLeftCorner<2, 2>() = R_WB;
  Eigen::Vector3d t_WB;
  t_WB << xny, 0;
  Eigen::Quaterniond q_WB(R_WB3d);
  if (q_WB.w() < 0) {
    q_WB.coeffs() *= -1;
  }
  return okvis::kinematics::Transformation(t_WB, q_WB);
}

Eigen::Vector3d RoundedSquare::computeGlobalAngularRate(
    const okvis::Time time) const {
  double remainder = getPeriodRemainder(time);
  size_t j;
  double delta_t;
  decideTimeSlot(remainder, &j, &delta_t);
  Eigen::Vector3d omega_W = Eigen::Vector3d::Zero();
  switch (j % 2) {
    case 1:
      omega_W[2] = omega_;
      break;
    case 0:
    default:
      break;
  }
  return omega_W;
}

Eigen::Vector3d RoundedSquare::computeGlobalLinearAcceleration(
    const okvis::Time time) const {
  double remainder = getPeriodRemainder(time);
  size_t j;
  double delta_t;
  decideTimeSlot(remainder, &j, &delta_t);
  int half_j = j / 2;

  double motion_theta;
  Eigen::Vector3d a_W = Eigen::Vector3d::Zero();
  Eigen::Vector2d cs_theta;

  switch (j % 2) {
    case 0:
      break;
    case 1:
      motion_theta = M_PI * 0.5 * half_j + delta_t * omega_;
      cs_theta[0] = std::cos(motion_theta);
      cs_theta[1] = std::sin(motion_theta);
      a_W.head<2>() = -radius_ * cs_theta * omega_ * omega_;
      break;
    default:
      break;
  }
  return a_W;
}

Eigen::Vector3d RoundedSquare::computeGlobalLinearVelocity(
    const okvis::Time time) const {
  double remainder = getPeriodRemainder(time);
  size_t j;
  double delta_t;
  decideTimeSlot(remainder, &j, &delta_t);
  int half_j = j / 2;

  double theta;
  double motion_theta;
  Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
  Eigen::Vector2d msc_theta;

  Eigen::Matrix2d R_WB;
  Eigen::Vector2d v_B(velocityNorm_, 0);

  switch (j % 2) {
    case 0:
      theta = (half_j + 1) * M_PI * 0.5;
      R_WB = rotMat2d(theta);
      v_W.head<2>() = R_WB * v_B;
      break;
    case 1:
      motion_theta = M_PI * 0.5 * half_j + delta_t * omega_;
      msc_theta[0] = -std::sin(motion_theta);
      msc_theta[1] = std::cos(motion_theta);
      v_W.head<2>() = radius_ * msc_theta * omega_;
      break;
    default:
      break;
  }
  return v_W;
}

Eigen::Matrix2d rotMat2d(double theta) {
  Eigen::Matrix2d mat;
  double ct = std::cos(theta);
  double st = std::sin(theta);
  mat << ct, -st, st, ct;
  return mat;
}

// decide time slot, endEpochs_[j-1] < time_into_period <= endEpochs_[j]
void RoundedSquare::decideTimeSlot(double time_into_period, size_t* j,
                                   double* time_into_slot) const {
  *j = 0;
  for (; *j < endEpochs_.size(); ++(*j)) {
    if (time_into_period < endEpochs_[*j]) break;
  }

  *time_into_slot =
      *j > 0 ? time_into_period - endEpochs_[(*j) - 1] : time_into_period;
}

double RoundedSquare::getPeriodRemainder(const okvis::Time time) const {
  return std::fmod(time.toSec(), period_.toSec());
}

WavyCircle::WavyCircle(double imuFreq, Eigen::Vector3d ginw, double wallRadius,
                       double trajectoryRadius, double wallHeight,
                       double frequencyNumber, double velocityNorm)
    : TrajectoryBase(imuFreq, ginw),
      wallRadius_(wallRadius),
      trajectoryRadius_(trajectoryRadius),
      wallHeight_(wallHeight),
      frequencyNumber_(frequencyNumber),
      waveHeightCoeff_(0.9),
      velocity_(velocityNorm),
      angularRate_(velocityNorm / trajectoryRadius) {
  waveHeight_ = computeWaveHeight();
}

Eigen::Vector3d WavyCircle::computeGlobalAngularRate(const okvis::Time time) const {
  double timeVal = time.toSec();
  Eigen::Matrix3d R_WB = orientation(timeVal * angularRate_);
  Eigen::Matrix<double, 9, 1> residual;
  const double* const parameters[] = {&timeVal};
  Eigen::Matrix<double, 9, 1> j;
  double * jacobians[] = {j.data()};
  ::ceres::internal::AutoDifferentiate<9, ::ceres::internal::StaticParameterDims<1>>(
          *this, parameters, 9, residual.data(), jacobians);
  Eigen::Matrix3d Rprime = Eigen::Map<Eigen::Matrix3d>(j.data());
  Eigen::Matrix3d OmegaW = Rprime * R_WB.transpose();
  return okvis::kinematics::vee(OmegaW);
}

Eigen::Vector3d WavyCircle::computeGlobalLinearAcceleration(
    const okvis::Time time) const {
  double t = time.toSec() * angularRate_;
  Eigen::Vector3d a_WB_W;
  a_WB_W << -trajectoryRadius_ * angularRate_ * angularRate_ * std::cos(t),
      -trajectoryRadius_ * angularRate_ * angularRate_ * std::sin(t),
      -waveHeight_ * frequencyNumber_ * frequencyNumber_ * angularRate_ *
          angularRate_ * std::cos(frequencyNumber_ * t);
  return a_WB_W;
}

Eigen::Vector3d WavyCircle::computeGlobalLinearVelocity(
    const okvis::Time time) const {
  double t = time.toSec() * angularRate_;
  Eigen::Vector3d v_WB_W;
  v_WB_W << - trajectoryRadius_ * angularRate_ * std::sin(t),
      trajectoryRadius_ * angularRate_ * std::cos(t),
      - waveHeight_ * frequencyNumber_ * angularRate_ * std::sin(frequencyNumber_ * t);
  return v_WB_W;
}

okvis::kinematics::Transformation WavyCircle::computeGlobalPose(
    const okvis::Time time) const {
  double t = time.toSec() * angularRate_;
  return okvis::kinematics::Transformation(position(t),
                                           Eigen::Quaterniond(orientation(t)));
}

Eigen::Vector3d WavyCircle::position(double t) const {
  Eigen::Vector3d t_WB_W;
  t_WB_W << trajectoryRadius_ * std::cos(t), trajectoryRadius_ * std::sin(t),
      waveHeight_ * std::cos(frequencyNumber_ * t);
  return t_WB_W;
}

double WavyCircle::nearestDepth() const {
  return std::sqrt(wallRadius_ * wallRadius_ -
                   trajectoryRadius_ * trajectoryRadius_);
}

double WavyCircle::computeWaveHeight() const {
  double halfz = 0.5 * wallHeight_;
  double nd = nearestDepth();
  double tan_vertical_half_Fov = halfz / nd;

  double wh = waveHeightCoeff_ *
              (tan_vertical_half_Fov * trajectoryRadius_ / frequencyNumber_);
  return wh;
}

template<typename T>
Eigen::Matrix<T, 3, 3> WavyCircle::orientation(T t) const {
  Eigen::Matrix<T, 3, 1> F(
      -T(trajectoryRadius_) * sin(t), T(trajectoryRadius_) * cos(t),
      -T(waveHeight_) * T(frequencyNumber_) * sin(T(frequencyNumber_) * t));
  F.normalize();
  Eigen::Matrix<T, 3, 1> L(-cos(t), -sin(t), T(0));
  Eigen::Matrix<T, 3, 1> U = F.cross(L);
  U.normalize();
  Eigen::Matrix<T, 3, 3> R_WB;
  R_WB.col(0) = F;
  R_WB.col(1) = L;
  R_WB.col(2) = U;
  // add rotation about another axis.
  Eigen::Matrix<T, 3, 3> Rx = RotX(T(30 * M_PI / 180) * sin(T(5) * t));
  R_WB = R_WB * Rx;
  return R_WB;
}

Lemniscate::Lemniscate(double imuFreq, Eigen::Vector3d ginw,
                       okvis::Time startTime,
                       double omega, double scale, double slopeAngleRad)
    : TrajectoryBase(imuFreq, ginw),
      omega_(omega), scale_(scale), slopeAngleRad_(slopeAngleRad) {
  period_ = 2 * M_PI / omega_;
  startTime_ = startTime;
  cosSlope_ = std::cos(slopeAngleRad);
  sinSlope_ = std::sin(slopeAngleRad);
  R_WWp_ << cosSlope_, 0, sinSlope_, 0, 1, 0, -sinSlope_, 0, cosSlope_;
  p_WWp_.setZero();
}

double Lemniscate::xWp(okvis::Time time) const {
  okvis::Duration t = time - startTime_;
  double theta = omega_ * t.toSec();
  return scale_ * std::cos(theta) /
         (1 + std::sin(theta) * std::sin(theta));
}

double Lemniscate::yWp(okvis::Time time) const {
  okvis::Duration t = time - startTime_;
  double theta = omega_ * t.toSec();
  return scale_ * std::sin(2 * theta) /
         (2 * (1 + std::sin(theta) * std::sin(theta)));
}

double Lemniscate::zWp(okvis::Time /*time*/) const {
  return 0;
}

double Lemniscate::xW(okvis::Time time) const {
  return cosSlope_ * xWp(time);
}

double Lemniscate::yW(okvis::Time time) const {
  return yWp(time);
}

double Lemniscate::zW(okvis::Time time) const {
  return -sinSlope_ * xWp(time);
}

double Lemniscate::dotxWp(okvis::Time time) const {
  okvis::Duration h(0, 1000);
  double x = xWp(time - h);
  double xp = xWp(time + h);
  return (xp - x) / (2 * h.toSec());
}

double Lemniscate::dotyWp(okvis::Time t) const {
  okvis::Duration h(0, 1000);
  double x = yWp(t - h);
  double xp = yWp(t + h);
  return (xp - x) / (2 * h.toSec());
}

double Lemniscate::dotzWp(okvis::Time /*time*/) const {
  return 0;
}

double Lemniscate::dotxW(okvis::Time time) const {
  return cosSlope_ * dotxWp(time);
}

double Lemniscate::dotyW(okvis::Time t) const {
  return dotyWp(t);
}

double Lemniscate::dotzW(okvis::Time time) const {
  return -sinSlope_ * dotxWp(time);
}

Eigen::Matrix3d Lemniscate::R_WB(okvis::Time t) const {
  double vx = dotxWp(t);
  double vy = dotyWp(t);
  double theta = std::atan2(vy, vx);
  double ct = std::cos(theta);
  double st = std::sin(theta);
  Eigen::Matrix3d R;
  R << ct, -st, 0, st, ct, 0, 0, 0, 1;
  return R_WWp_ * R;
}

double Lemniscate::velocityBody(okvis::Time time) const {
  Eigen::Matrix3d R = R_WB(time);
  Eigen::Vector3d vW(dotxW(time), dotyW(time), dotzW(time));
  Eigen::Vector3d vB = R.transpose() * vW;
  assert(abs(vB[1]) < 1e-6 && abs(vB[2]) < 1e-6);
  return vB[0];
}

Eigen::Vector3d
Lemniscate::computeGlobalAngularRate(const okvis::Time time) const {
  double theta = omega_ * (time - startTime_).toSec();
  double k = 3 * std::sqrt(2) * std::cos(theta) / (scale_ *
             std::sqrt(3 - std::cos(2 * theta)));
  return R_WWp_ * Eigen::Vector3d(0, 0, velocityBody(time) * k);
}

Eigen::Vector3d
Lemniscate::computeGlobalLinearAcceleration(const okvis::Time t) const {
  okvis::Duration h(0, 1000);
  double vx = dotxW(t - h);
  double vy = dotyW(t - h);
  double vz = dotzW(t - h);
  double vxp = dotxW(t + h);
  double vyp = dotyW(t + h);
  double vzp = dotzW(t + h);
  return Eigen::Vector3d(vxp - vx, vyp - vy, vzp - vz) * (1.0 / (2 * h.toSec()));
}

Eigen::Vector3d
Lemniscate::computeGlobalLinearVelocity(const okvis::Time t) const {
  return Eigen::Vector3d(dotxW(t), dotyW(t), dotzW(t));
}

okvis::kinematics::Transformation
Lemniscate::computeGlobalPose(const okvis::Time t) const {
  return okvis::kinematics::Transformation(Eigen::Vector3d(xW(t), yW(t), zW(t)),
                                           Eigen::Quaterniond(R_WB(t)));
}

LineSegment::LineSegment(double imuFreq, Eigen::Vector3d ginw,
                         okvis::Time startTime,
                         double omega,
                         double scale, double basevelocity)
    : TrajectoryBase(imuFreq, ginw),
      startTime_(startTime),
      omega_(omega), scale_(scale),
      basevelocity_(basevelocity) {
  period_ = 2 * M_PI / omega;
}

Eigen::Vector3d
LineSegment::computeGlobalAngularRate(const okvis::Time /*time*/) const {
  return Eigen::Vector3d(0, 0, 0);
}

double LineSegment::xW(okvis::Time time) const {
  okvis::Duration t = time - startTime_;
  okvis::Duration d(0.5 * period_);
  return scale_ * ((t - d).toSec() * (1 + basevelocity_) -
                   std::sin(omega_ * t.toSec()) / omega_);
}

double LineSegment::dotxW(okvis::Time time) const {
  okvis::Duration t = time - startTime_;
  return scale_ * (1 - std::cos(omega_ * t.toSec()) + basevelocity_);
}

Eigen::Matrix3d LineSegment::R_WB(okvis::Time t) const {
  double vx = dotxW(t);
  double vy = 0;
  double theta = std::atan2(vy, vx);
  double ct = std::cos(theta);
  double st = std::sin(theta);
  Eigen::Matrix3d R;
  R << ct, -st, 0, st, ct, 0, 0, 0, 1;
  return R;
}

Eigen::Vector3d
LineSegment::computeGlobalLinearAcceleration(const okvis::Time time) const {
  okvis::Duration h(0, 1000);
  double x = dotxW(time - h);
  double xp = dotxW(time + h);
  double ax = (xp - x) / (2 * h.toSec());
  return Eigen::Vector3d(ax, 0, 0);
}

Eigen::Vector3d
LineSegment::computeGlobalLinearVelocity(const okvis::Time time) const {
  return Eigen::Vector3d(dotxW(time), 0, 0);
}

okvis::kinematics::Transformation
LineSegment::computeGlobalPose(const okvis::Time t) const {
  return okvis::kinematics::Transformation(Eigen::Vector3d(xW(t), 0, 0),
                                           Eigen::Quaterniond(R_WB(t)));
}

std::shared_ptr<TrajectoryBase>
createSimulatedTrajectory(SimulatedTrajectoryType trajectoryType, int rate,
                          const Eigen::Vector3d &gravityInSimWorld,
                          okvis::Time startTime, double slopeAngleRad) {
  switch (trajectoryType) {
  case SimulatedTrajectoryType::Sinusoid:
    return std::shared_ptr<TrajectoryBase>(
        new simul::CircularSinusoidalTrajectory(rate, gravityInSimWorld));

  case SimulatedTrajectoryType::Torus:
    return std::shared_ptr<TrajectoryBase>(
        new simul::TorusTrajectory(rate, gravityInSimWorld));

  case SimulatedTrajectoryType::Squircle:
    return std::shared_ptr<TrajectoryBase>(new SlopedCurve<RoundedSquare>(
        RoundedSquare(rate, gravityInSimWorld), slopeAngleRad));

  case SimulatedTrajectoryType::Circle:
    return std::shared_ptr<TrajectoryBase>(new SlopedCurve<RoundedSquare>(
        RoundedSquare(rate, gravityInSimWorld, 2.0, 0, 2.0), slopeAngleRad));

  case SimulatedTrajectoryType::Dot:
    return std::shared_ptr<TrajectoryBase>(new SlopedCurve<RoundedSquare>(
        RoundedSquare(rate, gravityInSimWorld, 1e-3, 0, 0.8e-3), slopeAngleRad));

  case SimulatedTrajectoryType::WavyCircle:
    return std::shared_ptr<TrajectoryBase>(new SlopedCurve<WavyCircle>(
        WavyCircle(rate, gravityInSimWorld), slopeAngleRad));

  case SimulatedTrajectoryType::Lemniscate:
    return std::shared_ptr<TrajectoryBase>(new SlopedCurve<Lemniscate>(
        Lemniscate(rate, gravityInSimWorld, startTime, 0.3, 6.0), slopeAngleRad));

  case SimulatedTrajectoryType::LineSegment:
    return std::shared_ptr<TrajectoryBase>(new SlopedCurve<LineSegment>(
        LineSegment(rate, gravityInSimWorld, startTime), slopeAngleRad));

  case SimulatedTrajectoryType::Motionless:
    return std::shared_ptr<TrajectoryBase>(
        new simul::Motionless(rate, gravityInSimWorld));

  case SimulatedTrajectoryType::Torus2:
    return std::shared_ptr<TrajectoryBase>(
        new simul::SphereTrajectory(rate, gravityInSimWorld));

  case SimulatedTrajectoryType::Ball:
    return std::shared_ptr<TrajectoryBase>(
        new simul::SphereTrajectory(rate, gravityInSimWorld, 1.0, 0.4 * M_PI));

  default:
    LOG(ERROR) << "Unknown trajectory id " << static_cast<int>(trajectoryType);
    return std::shared_ptr<TrajectoryBase>(new SlopedCurve<WavyCircle>(
        WavyCircle(rate, gravityInSimWorld), slopeAngleRad));
  }
}

}  // namespace simul
