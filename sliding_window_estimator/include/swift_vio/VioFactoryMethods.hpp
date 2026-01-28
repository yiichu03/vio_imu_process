#ifndef INCLUDE_SWIFT_VIO_VIOFACTORY_METHODS_HPP_
#define INCLUDE_SWIFT_VIO_VIOFACTORY_METHODS_HPP_

#include <io_wrap/Publisher.hpp>

#include <okvis/VioFrontendInterface.hpp>

#include <swift_vio/CameraFrontendBase.h>
#include <swift_vio/EstimatorBase.h>
#include <swift_vio/KeyframePublisher.hpp>
#include <swift_vio/VioSystemBase.h>
#include <swift_vio/SwiftParameters.hpp>

#include <gtsam/VioBackEndParams.h>


namespace swift_vio {
std::shared_ptr<CameraFrontendBase> createFrontend(
    int numCameras, const FrontendOptions& frontendOptions);

std::shared_ptr<EstimatorBase> createInitializer(
    const okvis::EstimatorOptions& options);

std::shared_ptr<EstimatorBase> createBackend(
    const okvis::EstimatorOptions& backendParams);

/**
 * @brief registerCallbacks
 * @param output_dir
 * @param parameters
 * @param vioSystem
 * @param publisher
 * @return
 */
void registerCallbacks(const std::string &output_dir,
                       const okvis::VioParameters &parameters,
                       std::shared_ptr<VioSystemBase> vioSystem,
                       StreamPublisher *publisher,
                       KeyframePublisher *keyframePublisher);

}  // namespace swift_vio

namespace okvis {
class EstimatorBase;
std::shared_ptr<okvis::VioFrontendInterface>
createFrontend(int numCameras,
               const swift_vio::FrontendOptions &frontendOptions);
std::shared_ptr<EstimatorBase>
createBackend(const EstimatorOptions &options,
              const swift_vio::BackendParams &backendParams);
} // namespace okvis

#endif // INCLUDE_SWIFT_VIO_VIOFACTORY_METHODS_HPP_
