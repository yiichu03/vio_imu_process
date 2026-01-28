
/**
 * @file SimParametersReader.hpp
 * @brief Source file for the SimParametersReader class.
 * @author
 */

#include <glog/logging.h>
#include <simul/SimParametersReader.hpp>

namespace simul {
// The default constructor.
SimParametersReader::SimParametersReader() : VioParametersReader() {}

// The constructor. This calls readConfigFile().
SimParametersReader::SimParametersReader(const std::string& filename) {
  // cannot call base class constructor because it will not use the overloaded
  // getCameraCalibration()
  readConfigFile(filename);
  readSimParameters(filename);
}

void parseSimGlobalParams(cv::FileNode simGlobalParamNode,
                          SimParameters *simParams) {
  if (simGlobalParamNode["trajectory"].isString()) {
    std::string trajLabel;
    simGlobalParamNode["trajectory"] >> trajLabel;
    simParams->setTrajLabel(trajLabel);
  }
  if (simGlobalParamNode["sigma_v_WB"].isReal()) {
    simGlobalParamNode["sigma_v_WB"] >> simParams->sigma_initial_speed;
  }
  if (simGlobalParamNode["sigma_v_WB"].isSeq()) {
    simGlobalParamNode["sigma_v_WB"][0] >> simParams->sigma_initial_speed;
  }
  if (simGlobalParamNode["numRuns"].isInt()) {
    simGlobalParamNode["numRuns"] >> simParams->numRuns;
  }
  if (simGlobalParamNode["duration"].isReal()) {
    simGlobalParamNode["duration"] >> simParams->duration;
  }
  if (simGlobalParamNode["slopeAngleDeg"].isReal()) {
    simGlobalParamNode["slopeAngleDeg"] >> simParams->slopeAngleRad;
    simParams->slopeAngleRad *= M_PI / 180;
  }
  okvis::parseBoolean(simGlobalParamNode["checkTriangulation"],
                      simParams->checkTriangulation);
  okvis::parseBoolean(simGlobalParamNode["checkMeasurementJacobian"],
                      simParams->checkMeasurementJacobian);
}

void parseSimCameraParams(cv::FileNode simCameraParamNode,
                          SimCameraParameters *simCameraParams) {
  okvis::parseBoolean(simCameraParamNode["addExtraLandmarks"],
                      simCameraParams->addExtraLandmarks);

  okvis::parseBoolean(simCameraParamNode["addImageNoise"],
                      simCameraParams->addImageNoise);
  okvis::parseBoolean(simCameraParamNode["useImageObservations"],
                      simCameraParams->useImageObservations);
  okvis::parseEnum(simCameraParamNode["cameraModelId"], &simCameraParams->cameraModelId);

  okvis::parseEnum(simCameraParamNode["cameraOrientationId"], &simCameraParams->cameraOrientationId);

  okvis::parseEnum(simCameraParamNode["landmarkDistribution"], &simCameraParams->landmarkDistribution);

  if (simCameraParamNode["landmarkCylinderRadius"].isReal()) {
    simCameraParamNode["landmarkCylinderRadius"] >>
        simCameraParams->landmarkCylinderRadius;
  }

  okvis::parseBoolean(simCameraParamNode["useTrueLandmarkPosition"],
                      simCameraParams->useTrueLandmarkPosition);
}

void parseSimImuParams(cv::FileNode simImuParamNode, SimImuParameters *simImuParams) {
  okvis::parseBoolean(simImuParamNode["addImuNoise"], simImuParams->addImuNoise);
}

void SimParametersReader::readSimParameters(const std::string &filename) {
  cv::FileStorage file(filename, cv::FileStorage::READ);
  okvis::parseCameraNoises(file["sim_camera_noise_params"], &simParameters_.camNoiseParams);
  okvis::parseImuParameters(file["sim_imu_noise_params"], &simParameters_.imuNoiseParams);

  parseSimGlobalParams(file["sim_global_params"], &simParameters_);
  parseSimCameraParams(file["sim_camera_params"], &simParameters_.cameraParams);
  parseSimImuParams(file["sim_imu_params"], &simParameters_.imuParams);
}

}  // namespace simul
