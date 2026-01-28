#ifndef INCLUDE_SWIFT_VIO_VIO_TEST_SYSTEM_BUILDER_HPP_
#define INCLUDE_SWIFT_VIO_VIO_TEST_SYSTEM_BUILDER_HPP_

#include <gtsam/VioBackEndParams.h>

#include <io_wrap/Publisher.hpp>
#include <okvis/Parameters.hpp>

#include <simul/CoupledSimulationFrontend.hpp>
#include <simul/SimulationFrontend.hpp>
#include <simul/SimulatorBase.hpp>
#include <simul/SimParameters.h>

#include <swift_vio/EstimatorBase.h>
#include <swift_vio/SlidingWindowSmoother.h>
#include <swift_vio/VioSystemBase.h>

namespace simul {
typedef std::function<void(const Eigen::VectorXd &mse,
                           const Eigen::VectorXd &desiredStdevs,
                           const std::vector<std::string> &dimensionLabels)>
    CheckMseCallback;

typedef std::function<void(const Eigen::Matrix<double, 6, 1> &nees)> CheckNeesCallback;

class VioSimTestSystem {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VioSimTestSystem(const CheckMseCallback &checkMseCallback,
                   const CheckNeesCallback &checkNeesCallback);

  virtual ~VioSimTestSystem();

  void createRefSensorSystem(const SimParameters &simParameters, const okvis::cameras::NCameraSystem &cameraSystem);

  void registerCallbacks(swift_vio::StreamPublisher *publisher);

  /**
   * @brief createInitSensorSystem
   * @param simParameters
   * @param[in, out] vioParameters the VioParameters used in estimation.
   */
  void createInitSensorSystem(const SimParameters &simParameters, okvis::VioParameters *vioParameters);

  void createEstimator(const okvis::VioParameters &vioParameters,
                       const swift_vio::BackendParams *flsParams = NULL);

  void loadSimulatedData(const simul::SimParameters &simParameters);

  /**
   * @brief run the sliding window filter or the sliding window smoother which are based on the swift_vio::EstimatorBase.
   * @param simParameters
   * @param vioParams
   */
  void run(const simul::SimParameters &simParameters,
           okvis::VioParameters *vioParams);

  void runNoiseFree(const simul::SimParameters &simParameters,
                    okvis::VioParameters *vioParams);

  /**
   * @brief run the estimation starting from VioInitializer
   * @param simParameters determines noise added to the initial state variables and observations.
   * @param initial values and std devs for VIO parameters.
   * Their initial values may be reset by predefined values and corrupted by
   * noise in the simulation. Their std dev will remain intact and will be used
   * by the VIO estimator, determining whether a parameter is locked up or not.
   * @param outputPath
   */
  void runFromInitializer(const simul::SimParameters &simParameters,
                          okvis::VioParameters *vioParams);

  /**
   * @brief runOkvisEstimator run okvis estimator or fixed lag smoothers which are based on the okvis::EstimatorBase.
   * @param simParameters
   * @param flsParams fixed lag smoother parameters
   * @param vioParams
   */
  void runOkvisEstimator(const simul::SimParameters &simParameters,
                         const swift_vio::BackendParams &flsParams,
                         okvis::VioParameters *vioParams);

  void checkMeasurementJacobian(int landmarkModelId);

  void checkTriangulation(int landmarkModelId);

  void setCheckNeesCallback(const CheckNeesCallback& checkNeesCallback) {
    checkNeesCallback_ = checkNeesCallback;
  }

  void setCheckMseCallback(const CheckMseCallback& checkMseCallback) {
    checkMseCallback_ = checkMseCallback;
  }

  void printTriangulationSummary() const;

  void printJacobianSummary() const;

  template <typename EstimatorBaseT>
  void publish(std::shared_ptr<const EstimatorBaseT> estimator,
                                 const okvis::VioParameters &parameters) {
    swift_vio::OptimizationResults result;
    swift_vio::getEstimationResult(estimator, estimator->template currentFrameId(),
                        &result, parameters.nCameraSystem.numCameras(),
                        parameters.publishing.publishImuPropagatedState);
    if (fullStateCallback_)
      fullStateCallback_(result.stamp, result.T_WS, result.speedAndBiases,
                         result.omega_S, result.nframeId);
    if (landmarksCallback_ && !result.landmarksVector.empty())
      landmarksCallback_(
          result.stamp, result.landmarksVector,
          result.transferredLandmarks);
  }

  template <typename EstimatorBaseT, typename MultiFrameT>
  void visualize(
      std::shared_ptr<const EstimatorBaseT> estimator,
      const okvis::VioParameters &parameters,
      std::shared_ptr<const MultiFrameT> currentNFrame,
      std::shared_ptr<const MultiFrameT> closestKeyframe) {
    swift_vio::VioVisualizer::VisualizationData::Ptr new_data;
    if (!parameters.visualization.displayImages)
        return;
    new_data = swift_vio::VioVisualizer::VisualizationData::Ptr(
        new swift_vio::VioVisualizer::VisualizationData());
    swift_vio::getVisualizationData(estimator, currentNFrame,
                         closestKeyframe, new_data);

    visualizer_.showDebugImages(new_data);
  }

private:
  std::shared_ptr<SimulatorBase> simData_;

  std::shared_ptr<swift_vio::CameraRig> refCameraSystem_;       // camera system used for projecting landmarks.
  std::shared_ptr<swift_vio::CameraRig> initialCameraSystem_;   // camera system used for initilizing the estimator.
//  std::shared_ptr<swift_vio::CameraRig> estimatedCameraSystem_; // camera system estimated by the estimator.

  okvis::CameraNoiseParameters initialCameraNoiseParameters_;   // camera noise parameters to initialize the estimator.

  okvis::ImuParameters refImuParameters_;                                // IMU parameters for simulation                     

  std::shared_ptr<swift_vio::VioInitializer> initializer_;
  std::shared_ptr<swift_vio::EstimatorBase> estimator_;

  std::shared_ptr<CoupledSimulationFrontend> coupledFrontend_;
  std::shared_ptr<okvis::EstimatorBase> okvisEstimator_;
  std::shared_ptr<okvis::cameras::NCameraSystem> estimatedOkvisCameraSystem_;

  swift_vio::InitialNavState refNavState_;                               // reference nav state at the start.
  swift_vio::InitialNavState initialNavState_;                           // nav state at the start to initialize the estimator.
  std::shared_ptr<SimulationFrontend> frontend_;

  std::shared_ptr<::ceres::EvaluationCallback> evaluationCallback_;
  CheckMseCallback checkMseCallback_;
  CheckNeesCallback checkNeesCallback_;

  size_t checkedLmks_;
  size_t failedTriangulationLmks_;
  size_t poorTriangulationLmks_;
  size_t goodTriangulationLmks_;

  size_t checkedJacobianLmks_;
  size_t diffJacStatusLmks_;
  size_t poorJacobianLmks_;
  size_t goodJacobianLmks_;

  std::string viewerNamePrefix_;
  swift_vio::VioVisualizer visualizer_;

  swift_vio::StreamPublisher* publisher_;
  swift_vio::VioSystemBase::FullStateCallback fullStateCallback_;
  swift_vio::VioSystemBase::LandmarksCallback landmarksCallback_;
};

} // namespace simul
#endif // INCLUDE_SWIFT_VIO_VIO_TEST_SYSTEM_BUILDER_HPP_
