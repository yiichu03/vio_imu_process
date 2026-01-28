#include <swift_vio/VioFactoryMethods.hpp>

#include <gtsam/FixedLagSmoother.hpp>
#include <gtsam/RiFixedLagSmoother.hpp>
#include <gtsam/VioBackEndParams.h>

#include <swift_vio/FeatureTrackingFrontend.hpp>
#include <swift_vio/ExternalFramewiseFrontend.hpp>
#include <swift_vio/ImuInitializer.h>
#include <swift_vio/IoUtil.hpp>
#include <swift_vio/SlidingWindowFilter.h>
#include <swift_vio/SlidingWindowSmoother.h>
#include <swift_vio/SwiftParameters.hpp>

#include <io_wrap/CommonGflags.hpp>
#include <io_wrap/StreamHelper.hpp>

#include <okvis/Estimator.hpp>
#include <okvis/Frontend.hpp>
#include <okvis/Parameters.hpp>

namespace swift_vio {
std::shared_ptr<CameraFrontendBase> createFrontend(
    int numCameras, const FrontendOptions& frontendOptions) {
  switch (frontendOptions.featureTrackingMethod) {
    case FeatureTrackingScheme::ExternalFramewiseMatching:
      return std::shared_ptr<CameraFrontendBase>(
            new swift_vio::ExternalFramewiseFrontend(numCameras, frontendOptions));
    default:
      return std::shared_ptr<CameraFrontendBase>(
            new swift_vio::FeatureTrackingFrontend(numCameras, frontendOptions));
  }
}

std::shared_ptr<EstimatorBase> createInitializer(
    const okvis::EstimatorOptions &options) {
  switch (options.initializer) {
    // we do not use make_shared because it may interfere with alignment, see
    // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1049
    case EstimatorAlgorithm::ImuInitializer:
      return std::shared_ptr<EstimatorBase>(new ImuInitializer(options));

    case EstimatorAlgorithm::VioInitializer:
      return std::shared_ptr<EstimatorBase>(new VioInitializer(options));

    default:
      LOG(ERROR) << "Unknown Initializer type!";
      break;
  }
  return std::shared_ptr<EstimatorBase>();
}

std::shared_ptr<EstimatorBase> createBackend(
    const okvis::EstimatorOptions &options) {
  switch (options.algorithm) {
    // we do not use make_shared because it may interfere with alignment, see
    // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1049
    case EstimatorAlgorithm::SlidingWindowSmoother:
      return std::shared_ptr<EstimatorBase>(new SlidingWindowSmoother(options));

    case EstimatorAlgorithm::SlidingWindowFilter:
      return std::shared_ptr<EstimatorBase>(new SlidingWindowFilter(options));

    default:
      LOG(ERROR) << "Unsupported Estimator " << options.algorithm << "!";
      break;
  }
  return std::shared_ptr<EstimatorBase>();
}

void registerCallbacks(
    const std::string& output_dir, const okvis::VioParameters& parameters,
    std::shared_ptr<VioSystemBase> vioSystem, StreamPublisher* publisher,
    KeyframePublisher* keyframePublisher) {
  std::string path = removeTrailingSlash(output_dir);

  vioSystem->setFullStateCallback(
      std::bind(&StreamPublisher::publishFullStateAsCallback, publisher,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4));
  vioSystem->setLandmarksCallback(std::bind(
      &StreamPublisher::publishLandmarksAsCallback, publisher,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  std::string stateFilename = path + "/swift_vio.csv";
  size_t numCameras = parameters.nCameraSystem.numCameras();
  std::vector<std::string> extrinsicParamRepList(numCameras);
  std::vector<std::string> projectionParamRepList(numCameras);
  std::vector<std::string> distortionParamRepList(numCameras);
  for (size_t camIdx = 0; camIdx < numCameras; ++camIdx) {
    extrinsicParamRepList[camIdx] =
        parameters.nCameraSystem.extrinsicRep(camIdx);
    projectionParamRepList[camIdx] = parameters.nCameraSystem.projectionIntrinsicRep(camIdx);
    distortionParamRepList[camIdx] =
        parameters.nCameraSystem.cameraGeometry(camIdx)->distortionType();
  }

  std::string headerLine = vioSystem->headerLine();
  publisher->setCsvFile(stateFilename, headerLine);
  if (FLAGS_dump_output_option == 2) {
    // save estimates of evolving states, and camera extrinsics
    vioSystem->setFullStateCallbackWithExtrinsics(std::bind(
        &StreamPublisher::csvSaveFullStateWithExtrinsicsAsCallback, publisher,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
  } else if (FLAGS_dump_output_option == 3 || FLAGS_dump_output_option == 4) {
    // save estimates of evolving states, camera extrinsics,
    // and all other calibration parameters
    vioSystem->setFullStateCallbackWithAllCalibration(std::bind(
        &StreamPublisher::csvSaveFullStateWithAllCalibrationAsCallback,
        publisher, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5,
        std::placeholders::_6, std::placeholders::_7, std::placeholders::_8,
        std::placeholders::_9));
    if (FLAGS_dump_output_option == 4) {
      vioSystem->setImuCsvFile(path + "/imu0_data.csv");
      const unsigned int numCameras = parameters.nCameraSystem.numCameras();
      for (size_t i = 0; i < numCameras; ++i) {
        std::stringstream num;
        num << i;
        vioSystem->setTracksCsvFile(i,
                                    path + "/cam" + num.str() + "_tracks.csv");
      }
      publisher->setLandmarksCsvFile(path + "/vioSystem_landmarks.csv");
    }
  }
  vioSystem->setStateCallback(
      std::bind(&StreamPublisher::publishStateAsCallback, publisher,
                std::placeholders::_1, std::placeholders::_2));
  if (keyframePublisher) {
    vioSystem->setKeyframeCallback(
        std::bind(&KeyframePublisher::publish, keyframePublisher, std::placeholders::_1));
  }
}
}  // namespace swift_vio

namespace okvis {
std::shared_ptr<okvis::VioFrontendInterface>
createFrontend(int numCameras, const swift_vio::FrontendOptions &frontendOptions) {
  return std::shared_ptr<okvis::VioFrontendInterface>(
      new okvis::Frontend(numCameras, frontendOptions));
}

std::shared_ptr<EstimatorBase> createBackend(
    const EstimatorOptions &options,
    const swift_vio::BackendParams& backendParams) {
  switch (options.algorithm) {
  // we do not use make_shared because it may interfere with alignment, see
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1049
  case swift_vio::EstimatorAlgorithm::OkvisEstimator:
    return std::shared_ptr<okvis::EstimatorBase>(new okvis::Estimator());

  // case swift_vio::EstimatorAlgorithm::FixedLagSmoother:
  //   return std::shared_ptr<okvis::EstimatorBase>(
  //       new swift_vio::FixedLagSmoother(backendParams));

  // case swift_vio::EstimatorAlgorithm::RiFixedLagSmoother:
  //   return std::shared_ptr<okvis::EstimatorBase>(
  //       new swift_vio::RiFixedLagSmoother(backendParams));

  default:
    LOG(ERROR) << "Unsupported Estimator " << options.algorithm << "!";
    break;
  }
  return std::shared_ptr<okvis::Estimator>();
}

}  // namespace okvis
