#ifndef STATES_H
#define STATES_H


#include <okvis/Duration.hpp>
#include <okvis/Parameters.hpp>

namespace swift_vio {
/// \brief SensorStates The sensor-internal states enumerated
enum SensorStates
{
  Camera = 0, ///< Camera
  Imu = 1, ///< IMU
  Position = 2, ///< Position, currently unused
  Gps = 3, ///< GPS, currently unused
  Magnetometer = 4, ///< Magnetometer, currently unused
  StaticPressure = 5, ///< Static pressure, currently unused
  DynamicPressure = 6 ///< Dynamic pressure, currently unused
};

/// \brief CameraSensorStates The camera-internal states enumerated
enum CameraSensorStates
{
  T_XCi = 0, ///< Extrinsics as T_SCi or T_C0Ci
  Intrinsics, ///< Intrinsics, eg., for pinhole camera, fx ,fy, cx, cy
  TD,      ///< time delay of the image timestamp with respect to the IMU
               /// timescale, Raw t_Ci + t_d = t_Ci in IMU time,
  TR       ///< t_r is the read out time of a whole frames of a rolling shutter camera
};

/// \brief ImuSensorStates The IMU-internal states enumerated
/// \warning This is slightly inconsistent, since the velocity should be global.
enum ImuSensorStates
{
  Bias = 0, ///< Speed and biases as v in S-frame, then b_g and b_a
  MG,               ///< gyro correction matrix
  TS,               ///< g sensitivity
  MA                ///< accelerometer correction matrix
};

/// \brief StateInfo This configures the state vector ordering
struct StateInfo
{
  /// \brief Constructor
  /// @param[in] id The Id.
  /// @param[in] isRequired Whether or not we require the state.
  /// @param[in] exists Whether or not this exists in the ceres problem.
  StateInfo(uint64_t id = 0, bool isRequired = true,
            bool exists = false, size_t startIndex = 0u)
      : id(id),
        isRequired(isRequired),
        exists(exists),
        startIndexInCov(startIndex)
  {
  }
  uint64_t id; ///< The ID.
  bool isRequired; ///< Whether or not we require the state.
  bool exists; ///< Whether or not this exists in the ceres problem.
  size_t startIndexInCov; ///< start index in the covariance matrix.
};

struct ErrorStateInfo {
  uint64_t id;
  size_t startIndexInCov;
  size_t minimalDim;
  size_t oldStartIndexInCov;

  ErrorStateInfo(uint64_t sid = 0, size_t startIndex = 0, size_t minDim = 0,
                 size_t oldStartIndex = 0)
      : id(sid), startIndexInCov(startIndex), minimalDim(minDim),
        oldStartIndexInCov(oldStartIndex) {}
};

/// \brief GlobalStates The global states enumerated
enum GlobalStates
{
  T_WS = 0, ///< Pose.
  v_WS, ///< Linear velocity.
  GravityDirection,
  MagneticZBias, ///< Magnetometer z-bias, currently unused
  Qff, ///< QFF (pressure at sea level), currently unused
  T_GW, ///< Alignment of global frame, currently unused
};


// the following are just fixed-size containers for related parameterBlockIds:
typedef std::array<StateInfo, 6> GlobalStatesContainer; ///< Container for global states.
typedef std::vector<StateInfo> SpecificSensorStatesContainer;  ///< Container for sensor states. The dimension can vary from sensor to sensor...
typedef std::array<std::vector<SpecificSensorStatesContainer>, 7> AllSensorStatesContainer; ///< Union of all sensor states.

/// \brief States This summarizes all the possible states -- i.e. their ids:
/// \f$ t_j = t_{j_0} - imageDelay + t_{d_j} \f$
/// here \f$ t_{j_0} \f$ is the raw timestamp of image j,
/// \f$ t_{d_j} \f$ is the current estimated time offset between the visual and
/// inertial data, after correcting the initial time offset imageDelay.
/// Therefore, \f$ t_{d_j} \f$ is usually 0 at the beginning of the algorithm.
/// \f$ t_j \f$ is the timestamp of the state, remains constant after initialization.
/// \f$ t_{f_{i,j}} = t_{j_0} - imageDelay + t_d + (v-N/2)t_r/N \f$ here \f$ t_d \f$ and \f$ t_r \f$ are the time
/// offset and image readout time, \f$ t_{f_{i, j}} \f$ is the time feature i is observed in frame j.
struct States {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  States() : id(0), isKeyframe(false) {}
  States(uint64_t id, okvis::Time _timestamp, bool isKeyframe = false)
      : id(id),
        timestamp(_timestamp),
        isKeyframe(isKeyframe) {}

  uint64_t id;
  const okvis::Time timestamp;         // t_j, fixed once initialized
  bool isKeyframe;
  // IMU measurements centering at state timestamp.
  std::shared_ptr<okvis::ImuMeasurementDeque> imuReadingWindow;

  GlobalStatesContainer global;
  AllSensorStatesContainer sensors;
};

typedef
std::map<uint64_t, States, std::less<uint64_t>,
         Eigen::aligned_allocator<std::pair<const uint64_t, States>>> StateMap;

}  // namespace swift_vio

#endif // STATES_H
