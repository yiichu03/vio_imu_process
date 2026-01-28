#ifndef SIMUL_CURVE_H_
#define SIMUL_CURVE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <fstream>
#include <iostream>
#include <map>
#include <vector>

#include "okvis/ImuMeasurements.hpp"
#include "okvis/kinematics/Transformation.hpp"

#include <swift_vio/memory.h>

namespace simul {
typedef std::vector<okvis::ImuMeasurement,
                    Eigen::aligned_allocator<okvis::ImuMeasurement>>
    ImuMeasurementVector;

enum class SimulatedTrajectoryType {
  Sinusoid = 0,
  Torus,
  Torus2,
  Ball,
  Squircle,
  Circle,
  Dot,
  WavyCircle,
  Lemniscate,
  LineSegment,
  Motionless,
  FromTrajCsv,
};

bool EnumFromString(std::string description, SimulatedTrajectoryType *e);
std::string EnumToString(SimulatedTrajectoryType e);
inline std::ostream &operator<<(std::ostream &s, SimulatedTrajectoryType e) {
  return s << EnumToString(e);
}

class TrajectoryBase {
public:
  TrajectoryBase(double samplingFreq = 100,
                 const Eigen::Vector3d &ginw = Eigen::Vector3d(0, 0, -9.80665));

  virtual ~TrajectoryBase() {}

  void getTrueInertialMeasurements(
      const okvis::Time tStart, const okvis::Time tEnd,
      okvis::ImuMeasurementDeque &imuMeasurements) const;

  void getTrueInertialMeasurements(okvis::Time t, Eigen::Vector3d *gyroscope,
                                   Eigen::Vector3d *accelerometer) const;

  /**
   * @brief computeLocalAngularVelocity  \f$\omega_{WB}^B\f$
   * @warning Not efficient. You may want to use getTrueInertialMeasurements.
   * @param time
   * @return
   */
  Eigen::Vector3d computeLocalAngularVelocity(const okvis::Time time) const;

  void
  getTruePoses(const std::vector<okvis::Time> &times,
               Eigen::AlignedVector<okvis::kinematics::Transformation> &vT_WB);

  void getTrueVelocities(const std::vector<okvis::Time> &times,
                         Eigen::AlignedVector<Eigen::Vector3d> &velocities);

  void getSampleTimes(const okvis::Time tStart, const okvis::Time tEnd,
                      std::vector<okvis::Time> &vTime);

  // compute angular rate in the global frame, $\omega_{WB}^{W}$, with numerical
  // differentiation.
  Eigen::Vector3d computeGlobalAngularRateNumeric(const okvis::Time time) const;

  // compute $\dot{v}_{WB}^W$ with numerical differentiation.
  Eigen::Vector3d
  computeGlobalLinearAccelerationNumeric(const okvis::Time time) const;

  // compute $v_{WB}^W$ with numerical differentiation.
  Eigen::Vector3d
  computeGlobalLinearVelocityNumeric(const okvis::Time time) const;

  Eigen::Vector3d gravityVector() const { return gw; }

  double samplingFrequency() const { return freq; }

  // compute angular rate in the global frame, $\omega_{WB}^{W}$
  virtual Eigen::Vector3d
  computeGlobalAngularRate(const okvis::Time time) const = 0;

  // $\dot{v}_{WB}^W$
  virtual Eigen::Vector3d
  computeGlobalLinearAcceleration(const okvis::Time time) const = 0;
  // $v_{WB}^W$
  virtual Eigen::Vector3d
  computeGlobalLinearVelocity(const okvis::Time time) const = 0;

  // $T_{WB}$
  virtual okvis::kinematics::Transformation
  computeGlobalPose(const okvis::Time time) const = 0;

  virtual std::string name() const { return "TrajectoryBase"; }

protected:
  double freq;        // sampling frequency
  double interval;    // reciprocal of freq
  Eigen::Vector3d gw; // gravity in the global frame
};

// implements the horizontal circular and vertical sinusoidal
// motion of a body frame
// world frame x right, y forward, z up, sit at the circle center
// body frame, at each point on the curve,
// x outward along the radius, y tangent, z up
// imu frame coincides with body frame
class CircularSinusoidalTrajectory : public TrajectoryBase {
 protected:
  double wz;    // parameter determining the angular rate of sinusoidal
                // motion in the vertical direction
  double wxy;   // angular rate in the x-y plane
  double rz;    // the radius of the sinusoidal vertical motion
  double rxy;   // radius of the circular motion
  const double maxThetaZ;  // maximum elevation angle of the camera optical axis

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CircularSinusoidalTrajectory(double _maxThetaZ = 0.2 * M_PI);
  CircularSinusoidalTrajectory(
      double imuFreq, Eigen::Vector3d ginw, double _maxThetaZ = 0.2 * M_PI);
  virtual ~CircularSinusoidalTrajectory() {}

  // compute angular rate in the global frame, $\omega_{WB}^{W}$
  virtual Eigen::Vector3d computeGlobalAngularRate(const okvis::Time time) const;

  // $\dot{v}_{WB}^W$
  virtual Eigen::Vector3d
  computeGlobalLinearAcceleration(const okvis::Time time) const;
  // $v_{WB}^W$
  virtual Eigen::Vector3d computeGlobalLinearVelocity(const okvis::Time time) const;

  // $T_{WB}$
  virtual okvis::kinematics::Transformation computeGlobalPose(
      const okvis::Time time) const;

  virtual std::string name() const {
    return "Sinusoid";
  }
};

// Yarn torus
class TorusTrajectory : public CircularSinusoidalTrajectory {
 protected:
  const double wr;    // angular rate that the radius changes
  const double xosc;  // the oscillation mag in global x direction
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TorusTrajectory(double imuFreq, Eigen::Vector3d ginw);
  virtual ~TorusTrajectory() {}
  Eigen::Vector3d computeGlobalLinearAcceleration(
      const okvis::Time time) const override;

  Eigen::Vector3d computeGlobalLinearVelocity(const okvis::Time time) const override;

  okvis::kinematics::Transformation computeGlobalPose(
      const okvis::Time time) const override;
  std::string name() const override {
    return "Torus";
  }
};

// Yarn ball
class SphereTrajectory : public CircularSinusoidalTrajectory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SphereTrajectory(double _rxy = 37.0/19, double _maxThetaZ = 0.2 * M_PI);
  SphereTrajectory(double imuFreq, Eigen::Vector3d ginw,
                   double _rxy = 37.0/19, double _maxThetaZ = 0.2 * M_PI);
  virtual ~SphereTrajectory() {}
  Eigen::Vector3d computeGlobalAngularRate(const okvis::Time time) const override;

  Eigen::Vector3d computeGlobalLinearAcceleration(
      const okvis::Time time) const override;

  Eigen::Vector3d computeGlobalLinearVelocity(const okvis::Time time) const override;

  okvis::kinematics::Transformation computeGlobalPose(
      const okvis::Time time) const override;
  std::string name() const override {
    return "Sphere";
  }
};

// planar motion with constant velocity magnitude
class RoundedSquare : public TrajectoryBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RoundedSquare(double imuFreq, Eigen::Vector3d ginw, double radius = 1.0,
                double sideLength = 6.0, double velocityNorm = 1.2);

  virtual ~RoundedSquare() {}

  Eigen::Vector3d computeGlobalAngularRate(const okvis::Time time) const override;

  Eigen::Vector3d computeGlobalLinearAcceleration(
      const okvis::Time time) const override;

  Eigen::Vector3d computeGlobalLinearVelocity(const okvis::Time time) const override;

  okvis::kinematics::Transformation computeGlobalPose(
      const okvis::Time time) const override;

  std::vector<double> getEndEpochs() { return endEpochs_; }

  std::string name() const override {
    return "RoundedCircle";
  }

 private:

  // decide time slot, endEpochs_[j-1] < time_into_period <= endEpochs_[j]
  void decideTimeSlot(double time_into_period, size_t* j,
                      double* time_into_slot) const ;

  void initDataStructures();

  double getPeriodRemainder(const okvis::Time time) const;

  const double radius_; // radius of four arcs at corners
  const double sideLength_; // contiguous to the arc of radius
  const double velocityNorm_; // magnitude of velocity, to ensure continuity in velocity

  okvis::Duration period_; // time to travel through the rounded square
  double omega_; // angular rate
  std::vector<double> endEpochs_; // end epochs for each segments
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      centers_;  // centers for corner arcs

  // beginPoints for the 5 line segments on four sides
  // the first side has two halves because the starting point is at its middle
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      beginPoints_;
};

class WavyCircle : public TrajectoryBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  WavyCircle(double imuFreq, Eigen::Vector3d ginw, double wallRadius = 5.0,
             double trajectoryRadius = 4.0, double wallHeight = 3,
             double frequencyNumber = 10, double velocityNorm = 1.2);

  virtual ~WavyCircle() {}

  Eigen::Vector3d computeGlobalAngularRate(const okvis::Time time) const override;

  Eigen::Vector3d computeGlobalLinearAcceleration(
      const okvis::Time time) const override;

  Eigen::Vector3d computeGlobalLinearVelocity(
      const okvis::Time time) const override;

  okvis::kinematics::Transformation computeGlobalPose(
      const okvis::Time time) const override;

  std::string name() const override {
    return "WavyCircle";
  }

  template<typename T>
  Eigen::Matrix<T, 3, 3> orientation(T t) const;

  template <typename T>
  bool operator()(const T* time, T* R_WB_coeffs) const {
    Eigen::Map<Eigen::Matrix<T, 3, 3>> R_WB(R_WB_coeffs);
    T theta = time[0] * T(angularRate_);
    R_WB = orientation(theta);
    return true;
  }

  double waveHeight() const {
    return waveHeight_;
  }

  double angularRate() const {
    return angularRate_;
  }
 private:
  double wallRadius_;
  double trajectoryRadius_;
  double wallHeight_;
  double frequencyNumber_;   // wave frequency
  double waveHeightCoeff_;  // decrease the coefficient to make more point
                             // visible.

  double velocity_;
  double angularRate_;
  double waveHeight_;

  Eigen::Vector3d position(double t) const;

  double nearestDepth() const;

  double computeWaveHeight() const;
};

class Lemniscate : public TrajectoryBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Lemniscate(double imuFreq, Eigen::Vector3d ginw, okvis::Time startTime,
             double omega = 0.3,
             double scale = 6.0, double slopeAngleRad = 0.0);
  virtual ~Lemniscate() {}

  double xWp(okvis::Time t) const;
  double yWp(okvis::Time t) const;
  double zWp(okvis::Time t) const;

  double xW(okvis::Time t) const;
  double yW(okvis::Time t) const;
  double zW(okvis::Time t) const;

  double dotxWp(okvis::Time time) const;
  double dotyWp(okvis::Time time) const;
  double dotzWp(okvis::Time time) const;

  double dotxW(okvis::Time t) const;
  double dotyW(okvis::Time t) const;
  double dotzW(okvis::Time time) const;


  Eigen::Matrix3d R_WB(okvis::Time t) const;
  double velocityBody(okvis::Time time) const;

  Eigen::Vector3d
  computeGlobalAngularRate(const okvis::Time time) const override;

  Eigen::Vector3d
  computeGlobalLinearAcceleration(const okvis::Time time) const override;

  Eigen::Vector3d
  computeGlobalLinearVelocity(const okvis::Time time) const override;

  okvis::kinematics::Transformation
  computeGlobalPose(const okvis::Time time) const override;

  std::string name() const override {
    return "Lemniscate";
  }

private:
  double omega_;
  double scale_;
  double period_;
  double slopeAngleRad_;
  double cosSlope_;
  double sinSlope_;
  Eigen::Matrix3d R_WWp_; // rotation of the Wp frame (z along slope normal) relative to the W frame (z along negative gravity).
  Eigen::Vector3d p_WWp_; // position of the Wp frame (z along slope normal) relative to the W frame (z along negative gravity).
  okvis::Time startTime_;
};

class LineSegment : public TrajectoryBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LineSegment(double imuFreq, Eigen::Vector3d ginw, okvis::Time startTime,
              double omega = 2.0, double scale = 1.0, double basevelocity = 0.2);

  virtual ~LineSegment() {}

  double xW(okvis::Time t) const;

  double dotxW(okvis::Time t) const;

  Eigen::Matrix3d R_WB(okvis::Time t) const;

  Eigen::Vector3d
  computeGlobalAngularRate(const okvis::Time time) const override;

  Eigen::Vector3d
  computeGlobalLinearAcceleration(const okvis::Time time) const override;

  Eigen::Vector3d
  computeGlobalLinearVelocity(const okvis::Time time) const override;

  okvis::kinematics::Transformation
  computeGlobalPose(const okvis::Time time) const override;

  std::string name() const override {
    return "LineSegment";
  }

private:
  okvis::Time startTime_;
  double omega_;
  double scale_;
  double period_;
  double basevelocity_;
};

class Motionless : public TrajectoryBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Motionless(double imuFreq, Eigen::Vector3d ginw)
      : TrajectoryBase(imuFreq, ginw) {}
  virtual ~Motionless() {}

  Eigen::Vector3d computeGlobalAngularRate(
      const okvis::Time /*time*/) const override {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d computeGlobalLinearAcceleration(
      const okvis::Time /*time*/) const override {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d computeGlobalLinearVelocity(
      const okvis::Time /*time*/) const override {
    return Eigen::Vector3d::Zero();
  }

  okvis::kinematics::Transformation computeGlobalPose(
      const okvis::Time /*time*/) const override {
    return okvis::kinematics::Transformation();
  }

  std::string name() const override {
    return "Motionless";
  }
};

template <typename PlanarCurveType>
class SlopedCurve : public TrajectoryBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SlopedCurve(const PlanarCurveType &planarCurve, double slopeAngleRad)
      : TrajectoryBase(planarCurve.template samplingFrequency(),
                       planarCurve.template gravityVector()),
        planarCurve_(planarCurve), slopeAngleRad_(slopeAngleRad) {
    cosSlope_ = std::cos(slopeAngleRad);
    sinSlope_ = std::sin(slopeAngleRad);
    R_WWp_ << cosSlope_, 0, sinSlope_, 0, 1, 0, -sinSlope_, 0, cosSlope_;
    p_WWp_.setZero();
    T_WWp_ = okvis::kinematics::Transformation(p_WWp_, Eigen::Quaterniond(R_WWp_));
  }

  virtual ~SlopedCurve() {}

  Eigen::Vector3d
  computeGlobalAngularRate(const okvis::Time time) const override {
    return R_WWp_ * planarCurve_.computeGlobalAngularRate(time);
  }

  Eigen::Vector3d
  computeGlobalLinearAcceleration(const okvis::Time time) const override {
    return R_WWp_ * planarCurve_.computeGlobalLinearAcceleration(time);
  }

  Eigen::Vector3d
  computeGlobalLinearVelocity(const okvis::Time time) const override {
    return R_WWp_ * planarCurve_.computeGlobalLinearVelocity(time);
  }

  okvis::kinematics::Transformation
  computeGlobalPose(const okvis::Time time) const override {
    return T_WWp_ * planarCurve_.computeGlobalPose(time);
  }

  std::string name() const override {
    return "Sloped" + planarCurve_.template name();
  }

private:
  const PlanarCurveType planarCurve_;
  double slopeAngleRad_;
  double cosSlope_;
  double sinSlope_;
  Eigen::Matrix3d R_WWp_;
  Eigen::Vector3d p_WWp_;
  okvis::kinematics::Transformation T_WWp_;
};

template <typename T>
Eigen::Matrix<T, 3, 3> RotX(T theta) {
  T ct = cos(theta);
  T st = sin(theta);
  Eigen::Matrix<T, 3, 3> Rx;
  Rx << T(1), T(0), T(0),
  T(0), ct, st,
  T(0), -st, ct;
  return Rx;
}

/**
 * @brief rotMat2d This is in effect RotZ(theta + 90).
 * @param theta
 * @return
 */
Eigen::Matrix2d rotMat2d(double theta);

/**
 * @brief createSimulatedTrajectory factory method.
 * @param trajectoryType
 * @param rate
 * @param gravityInWorldFrame should be (0, 0, -gravityMag).
 * @return
 */
std::shared_ptr<TrajectoryBase>
createSimulatedTrajectory(SimulatedTrajectoryType trajectoryType, int rate,
                          const Eigen::Vector3d &gravityInSimWorld, okvis::Time startTime,
                          double slopeAngleRad);

} // namespace simul
#endif // SIMUL_CURVE_H_
