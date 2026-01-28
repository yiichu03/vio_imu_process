#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

#include <gtsam/VioBackEndParams.h>

#include <okvis/FrameTypedefs.hpp>

#include <simul/curves.h>
#include <simul/CameraSystemCreator.hpp>
#include <simul/LandmarkGrid.h>

namespace simul {
struct SimImuParameters {
  bool addImuNoise;                ///< add noise to IMU readings?

  SimImuParameters(bool _addImuNoise = true)
      : addImuNoise(_addImuNoise) {}

  std::string toString(const std::string &hint) const {
    std::stringstream ss;
    ss << hint << "addImuNoise " << addImuNoise << ".\n";
    return ss.str();
  }
};

struct SimCameraParameters {
  bool addExtraLandmarks; ///< add extra landmarks to ensure enough observations in frame?
  bool addImageNoise;   ///< add noise to image measurements in pixels?
  bool useImageObservations; ///< use image observations in an estimator?

  simul::SimCameraModelType cameraModelId;  // use the same camera model for all cameras.
  simul::CameraOrientation cameraOrientationId;  // use the same orientation for all cameras.

  LandmarkGridType landmarkDistribution;
  double landmarkCylinderRadius; // radius of the cylinder on whose surface the
                         // landmarks are distributed.
  bool useTrueLandmarkPosition;

  SimCameraParameters(
      bool _addExtraLandmarks = false,
      bool _addImageNoise = true, bool _useImageObservations = true,
      simul::SimCameraModelType _cameraModelId =
          simul::SimCameraModelType::EUROC,
      simul::CameraOrientation _cameraOrientationId =
          simul::CameraOrientation::Forward,
      LandmarkGridType _landmarkDistribution = LandmarkGridType::FourWalls,
      double _landmarkCylinderRadius = 5, bool _useTrueLandmarkPosition = false)
      : addExtraLandmarks(_addExtraLandmarks),
        addImageNoise(_addImageNoise), useImageObservations(_useImageObservations),
        cameraModelId(_cameraModelId),
        cameraOrientationId(_cameraOrientationId),
        landmarkDistribution(_landmarkDistribution),
        landmarkCylinderRadius(_landmarkCylinderRadius),
        useTrueLandmarkPosition(_useTrueLandmarkPosition) {
  }

  std::string toString(const std::string &hint) const {
    std::stringstream ss;
    ss << hint << "addExtraLandmarks " << addExtraLandmarks
       << ", addImageNoise " << addImageNoise << ", useImageObservations? "
       << useImageObservations
       << ".\ncamera geometry type "  << cameraModelId
       << ", camera orientation type "  << cameraOrientationId
       << ".\n";
    ss << "landmark grid type " << landmarkDistribution
       << ", landmark radius " << landmarkCylinderRadius << ".\n";
    return ss.str();
  }
};

struct SimParameters {
  std::string trajLabel;
  SimulatedTrajectoryType trajectoryId;

  SimImuParameters imuParams;
  SimCameraParameters cameraParams;

  double sigma_initial_speed;
  okvis::ImuNoiseParameters imuNoiseParams;
  okvis::CameraNoiseParameters camNoiseParams;

  int numRuns;
  double duration; // Duration of generated data, applies only to CurveData.
  double slopeAngleRad;  // slope of the simulated curve.

  std::string simDataPath; // external input in maplab csv format.
  std::string outputdir;
  bool checkTriangulation;
  bool checkMeasurementJacobian;

  SimParameters() {}

  SimParameters(std::string _trajLabel,
                const SimImuParameters &_imuParams,
                const SimCameraParameters &_cameraParams,
                const double _sigma_initial_speed,
                const okvis::ImuNoiseParameters &_imuNoiseParams,
                const okvis::CameraNoiseParameters &_camNoiseParams,
                int _numRuns,
                double _duration,
                double _slopeAngleRad,
                const std::string _simDataPath = "",
                const std::string _outputdir = "",
                bool _checkTriangulation = false,
                bool _checkMeasurementJacobian = false)
      : imuParams(_imuParams), cameraParams(_cameraParams),
        sigma_initial_speed(_sigma_initial_speed),
        imuNoiseParams(_imuNoiseParams), camNoiseParams(_camNoiseParams),
        numRuns(_numRuns),
        duration(_duration),
        slopeAngleRad(_slopeAngleRad),
        simDataPath(_simDataPath), outputdir(_outputdir),
        checkTriangulation(_checkTriangulation),
        checkMeasurementJacobian(_checkMeasurementJacobian) {
    setTrajLabel(_trajLabel);
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "Trajectory " << trajLabel
       << ", numRuns " << numRuns << ", duration " << duration
       << ", slope angle " << slopeAngleRad * 180 / M_PI << " deg.\n"
       << imuParams.toString("IMU parameters for generating sim data: ")
       << cameraParams.toString("Camera parameters for generating sim data: ")
       << "sigma_initial_speed " << sigma_initial_speed << "\n"
       << imuNoiseParams.toString("IMU noise parameters for generating sim data: ")
       << camNoiseParams.toString("Camera noise parameters for generating sim data: ");
    if (!simDataPath.empty()) {
      ss << "External input dir " << simDataPath << ".\n";
    }
    ss << "Output dir " << outputdir << ".\n";
    ss << "Check triangulation? " << checkTriangulation
       << ", check measurement Jacobian? " << checkMeasurementJacobian << ".\n";
    return ss.str();
  }

  inline void setTrajLabel(const std::string &_trajLabel) {
    trajLabel = _trajLabel;
    EnumFromString(trajLabel, &trajectoryId);
  }
};

struct SimFrontendOptions {
  int maxTrackLength_; ///< Cap feature track length
  int maxMatchKeyframes_;
  double minKeyframeDistance_;
  double minKeyframeAngle_;
  bool useTrueLandmarkPosition_;

  SimFrontendOptions(int maxTrackLength = 10, int maxMatchKeyframes = 3,
                     double minKeyframeDistance = 0.4,
                     double minKeyframeAngle = 10 * M_PI / 180,
                     bool useTrueLandmarkPosition = false)
      : maxTrackLength_(maxTrackLength), maxMatchKeyframes_(maxMatchKeyframes),
        minKeyframeDistance_(minKeyframeDistance),
        minKeyframeAngle_(minKeyframeAngle),
        useTrueLandmarkPosition_(useTrueLandmarkPosition) {}
};

struct LandmarkKeypointMatch {
  okvis::KeypointIdentifier currentKeypoint;
  okvis::KeypointIdentifier previousKeypoint;
  uint64_t landmarkId; // unique identifier
  size_t landmarkIdInVector; // index in the scene grid
};

}  // namespace simul

#endif // SIMPARAMETERS_H
