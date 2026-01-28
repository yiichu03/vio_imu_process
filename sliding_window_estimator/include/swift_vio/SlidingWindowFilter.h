#ifndef SLIDINGWINDOWFILTER_H
#define SLIDINGWINDOWFILTER_H

#include <okvis/Parameters.hpp>
#include <okvis/timing/Timers.hpp>
#include <swift_vio/EstimatorBase.h>
#include <swift_vio/PointLandmark.hpp>
#include <swift_vio/PointSharedData.hpp>
#include <swift_vio/memory.h>
#include <swift_vio/SlidingWindowSmoother.h>  // for VioInitializer

namespace swift_vio {
class SlidingWindowFilter : public EstimatorBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SlidingWindowFilter(const okvis::EstimatorOptions &options);

  virtual ~SlidingWindowFilter();

  std::string typeInfo() const final {
    return "SlidingWindowFilter";
  }

  void setPointLandmarkOptions(const swift_vio::PointLandmarkOptions& plOptions) final;

  void initializeLandmarksInFilter();

  void initializeFrom(std::shared_ptr<EstimatorBase> other, ConstMultiFramePtr currentNFrame) override;

  /**
   * @brief initialize from a given navigation state of pose and speed
   * @param initialState Because the estimators use a world frame with z along negative gravity,
   * the initialState needs to respect this.
   * @param currentNFrame
   */
  void initializeFromState(const InitialNavState &initialState, ConstMultiFramePtr currentNFrame) override;

  bool initializeCovByFisherInfo(const VioInitializer &initializer, bool trySvd);

  bool initializeCovByHeuristics();

  void estimate(std::shared_ptr<const VisualMatcherOutput> featureMatches) override;

  void updateWithCameraObservations();

  bool removeLandmarkParameterBlock(uint64_t landmarkId);

  bool hasLandmarkParameterBlock(uint64_t landmarkId) const;

  void decimateCovarianceForLandmarks(const std::vector<uint64_t>& toRemoveLmIds);

  bool applyMarginalizationStrategy(MapPointVector &/*removedLandmarks*/) override;

  void updateStates(const Eigen::Matrix<double, Eigen::Dynamic, 1>& deltaX);

  int marginalizeRedundantFrames(size_t numKeyframes, size_t numImuFrames,
                                 MapPointVector *removedLandmarks);

  size_t getCurrentlyObservedLandmarks(MapPointVector *landmarks) const final {
    *landmarks = slamLandmarks_;
    return slamLandmarks_.size();
  }

  void changeAnchors(const std::vector<uint64_t> &sortedRemovedStateIds);

  TriangulationStatus triangulateMapPoint(
      const MapPoint &mp,
      Eigen::AlignedVector<Eigen::Vector2d> *obsList,
      swift_vio::PointLandmark *pointLandmark,
      Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
      swift_vio::PointSharedData *pointDataPtr,
      std::vector<uint64_t> *orderedCulledFrameIds,
      bool checkDisparity = false) const;

  /**
   * @brief measurementJacobian compute Jacobians for a reprojection residual error.
   * @warning Both poseId and anchorId should be older than the latest frame Id.
   * @param homogeneousPoint, if landmarkModel is AIDP,
   * \f$[\alpha, \beta, 1, \rho] = [X, Y, Z, 1]^C_a / Z^C_a\f$,
   * if landmarkModel is HPP, \f$[X,Y,Z,1]^W\f$.
   * \f $[\alpha, \beta, 1]^T = \rho p_{C{t(i, a)}} \f$ or
   * \f $[\alpha, \beta, 1]^T = \rho p_{C{t(a)}} \f$
   * \f $t(a) \f$ is the epoch of the anchor state/frame.
   * \f $t(i, a) \f$ is the observation epoch of feature i in anchor frame a.
   * @param obs image observation in pixels.
   * @param observationIndex index of the observation inside the point's shared data.
   * @param pointDataPtr shared data of the point.
   * @param J_x Jacobians of the image observation relative to the gravity direction,
   * camera parameters, and cloned states. It ought to be allocated in advance.
   * @param J_pfi Jacobian of the image observation relative to landmark parameters, e.g., [\alpha, \beta, \rho].
   * @param residual
   * @param mixProjectionJac
   * @return true if Jacobians are computed successfully.
   */
  bool measurementJacobian(
      const PointLandmark& homogeneousPoint,
      const Eigen::Vector2d& obs, size_t observationIndex,
      const swift_vio::PointSharedData &pointData,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* J_x,
      Eigen::Matrix<double, 2, 3>* J_pfi,
      Eigen::Vector2d* residual,
      bool mixProjectionJac) const;

  /**
   * @brief measurementJacobianNumeric
   * @param homogeneousPoint
   * @param obs
   * @param observationIndex
   * @param pointData
   * @param J_x
   * @param J_pfi
   * @param residual: the residual is computed with lin points.
   * @return
   */
  bool measurementJacobianNumeric(
      const PointLandmark& homogeneousPoint,
      const Eigen::Vector2d& obs, size_t observationIndex,
      swift_vio::PointSharedData &pointData,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* J_x,
      Eigen::Matrix<double, 2, 3>* J_pfi,
      Eigen::Vector2d* residual);

  /**
   * @brief test measurementJacobian with auto diff on a ceres solver cost function.
   * @param pointLandmark
   * @param obs
   * @param observationIndex
   * @param pointDataPtr
   * @param J_x
   * @param J_pfi
   * @param residual
   * @return
   */
  bool measurementJacobianAutoDiff(
      const PointLandmark& pointLandmark, const Eigen::Vector2d &obs,
      size_t observationIndex,
      const swift_vio::PointSharedData &pointData,
      Eigen::Matrix<double, 2, Eigen::Dynamic> *J_x,
      Eigen::Matrix<double, 2, 3> *J_pfi, Eigen::Vector2d *residual) const;

  /**
   * @brief minimalDimOfAllCameraParams
   * @warning call this no earlier than first call of addStates().
   * @return
   */
  inline size_t minimalDimOfAllCameraParams() const {
    size_t totalCamDim = statesMap_.rbegin()
                             ->second.sensors.at(SensorStates::Camera)
                             .back()
                             .at(CameraSensorStates::TR)
                             .startIndexInCov +
                         (cameraNoiseParametersVec_.back().isReadoutTimeFixed() ? 0u : 1u);
    size_t totalImuDim = statesMap_.rbegin()
                             ->second.sensors.at(SensorStates::Camera)
                             .at(0u)
                             .at(CameraSensorStates::T_XCi)
                             .startIndexInCov;
    return totalCamDim - totalImuDim;
  }


  /// print out the most recent state vector and the stds of its elements.
  /// It can be called in the optimizationLoop, but a better way to save
  /// results is to save in the publisher loop
  bool printStatesAndStdevs(std::ostream &stream, const Eigen::MatrixXd *covariance) const final;

  void printTrackLengthHistogram(std::ostream &stream) const final;

  bool getStateStd(Eigen::Matrix<double, Eigen::Dynamic, 1>* stateStd) const final;

  Eigen::VectorXd getDesiredImuAugmentedParamStdevs() const;

  Eigen::VectorXd getDesiredCameraParamStdevs() const;

  bool getDesiredStdevs(Eigen::VectorXd *desiredStdevs) const override;

  bool computeCovariance(Eigen::MatrixXd* cov) const final;

  bool computeErrors(
      const okvis::kinematics::Transformation &ref_T_WS,
      const Eigen::Vector3d &ref_v_WS, const Eigen::Matrix<double, 6, 1> &biases,
      const okvis::ImuParameters &refImuParams,
      std::shared_ptr<const CameraRig> refCameraSystem,
      Eigen::VectorXd *errors) const override;

  Eigen::VectorXd computeCameraParamsError(
      const States &currentState,
      std::shared_ptr<const CameraRig> refCameraSystem) const;

  bool getOdometryConstraintsForKeyframe(
      std::shared_ptr<LoopQueryKeyframeMessage> queryKeyframe) const final;

  mutable okvis::TimerSwitchable propagateAndCloneTimer;
  mutable okvis::TimerSwitchable triangulateTimer;
  mutable okvis::TimerSwitchable computeHTimer;
  mutable okvis::TimerSwitchable updateLandmarksTimer;
  mutable okvis::TimerSwitchable optimizerCovarianceTimer;
  mutable okvis::TimerSwitchable zeroVelocityUpdateTimer;

  mutable okvis::TimerSwitchable computeKalmanGainTimer;
  mutable okvis::TimerSwitchable updateStatesTimer;
  mutable okvis::TimerSwitchable updateCovarianceTimer;

  std::vector<size_t>
      trackLengthAccumulator;  // histogram of the track lengths, start from
                                // 0,1,2, to a fixed number

  std::shared_ptr<okvis::ceres::LocalParamizationAdditionalInterfaces> landmarkParamizationPtr_;
  okvis::ceres::HomogeneousPointLocalParameterization homogeneousLandmarkParamization_;

  // error state: \delta p, \alpha for q, \delta v
  // state: \pi_{B_i}(=[p_{B_i}^G, q_{B_i}^G, v_{B_i}^G])
  static const int kClonedStateMinimalDim = 9;
  static const int kNavErrorStateDim = 9;

protected:
  /**
   * Order of error state in covariance,
   * \f$ p_{WB}, q_{WB}, v_{WB}, X_{imu,1}, X_{imu,2}, g^W, X_{cam,1}, X_{cam,2}, X_{clone}, L \f$
   * \f$ X_{imu,i} \f$ includes bias and other intrinsic parameters for IMU i.
   * \f$ X_{cam,i} \f$ includes focal length, distortion parameters for camera i.
   * \f$ X_{clone, i} \in X_{clone} \f$ is the cloned (position, quaternion, velocity) for state i.
   * \f$ L_{i} \in L \f$ is the parameters for landamrk i.
   * Index of error state variables except for landmarks are recorded in the state map.
   * The cloned nav state variables are ordered according to the state map which is ordered by time.
   * This ordering supports marginalizing a cloned state in the middle of the cloned state array.
   * Index of landmark error state variables can be computed from the ordered landmark array.
   */

  /**
   * @brief startIndexOfClonedStates
   * @return
   */
  size_t startIndexOfClonedStates() const {
    size_t dim = kClonedStateMinimalDim;
    for (size_t j = 0; j < imuRig_.numImus(); ++j) {
      dim += imuRig_.getImuParamsMinimalDim(j);
    }
    dim += (imuParametersVec_.at(0)->isGravityDirectionVariable() ? 2u : 0u); // gravity direction
    for (size_t j = 0; j < cameraRig_.numCameras(); ++j) {
      dim += cameraRig_.getCameraParamsVariableDim(j, cameraNoiseParametersVec_.at(j));
    }
    return dim;
  }

  size_t startIndexOfGravity() const {
    size_t dim = kClonedStateMinimalDim;
    for (size_t j = 0; j < imuRig_.numImus(); ++j) {
      dim += imuRig_.getImuParamsMinimalDim(j);
    }
    return dim;
  }

  /**
   * @brief startIndexOfClonedStatesFast
   * @warning call this no earlier than first call of addStates().
   * @return
   */
  inline size_t startIndexOfClonedStatesFast() const {
    return statesMap_.rbegin()
               ->second.sensors.at(SensorStates::Camera)
               .back()
               .at(CameraSensorStates::TR)
               .startIndexInCov +
           (cameraNoiseParametersVec_.back().isReadoutTimeFixed() ? 0u : 1u);
  }

  inline size_t startIndexOfImuBias(size_t imuIdx = 0u) const {
    size_t dim = kClonedStateMinimalDim;
    for (size_t j = 0; j < imuIdx; ++j) {
      dim += imuRig_.getImuParamsMinimalDim(j);
    }
    return dim;
  }

  size_t startIndexOfCameraParams(size_t camIdx = 0u) const {
    size_t dim = kClonedStateMinimalDim;
    for (size_t j = 0; j < imuRig_.numImus(); ++j) {
      dim += imuRig_.getImuParamsMinimalDim(j);
    }
    dim += (imuParametersVec_.at(0)->isGravityDirectionVariable() ? 2u : 0u);
    for (size_t j = 0; j < camIdx; ++j) {
      dim += cameraRig_.getCameraParamsVariableDim(j, cameraNoiseParametersVec_.at(j));
    }
    return dim;
  }

  /**
   * @brief startIndexOfCameraParamsFast
   * @warning call this no earlier than first call of addStates().
   * @param camIdx
   * @return
   */
  size_t startIndexOfCameraParamsFast(
      size_t camIdx,
      CameraSensorStates camParamBlockName = CameraSensorStates::T_XCi) const {
    return statesMap_.rbegin()
        ->second.sensors.at(SensorStates::Camera)
        .at(camIdx)
        .at(camParamBlockName)
        .startIndexInCov;
  }

  /**
   * @brief minimal dim of gravity direction, camera parameters, and all cloned states including the last
   * inserted one and all landmarks.
   * Ex: \delta(gW), C_p_B, f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, [k_3], t_d, t_r,
   * C0_p_Ci, C0_q_Ci, f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, [k_3], t_d, t_r,
   * \pi_{B_i}(=[p_{B_i}^G, q_{B_i}^G, v_{B_i}^G]), l_i
   * @warning call this no earlier than first call of addStates().
   * @return
   */
  inline size_t cameraParamPoseAndLandmarkMinimalDim() const {
    return (imuParametersVec_.at(0)->isGravityDirectionVariable() ? 2u : 0u) +
           minimalDimOfAllCameraParams() +
           kClonedStateMinimalDim * statesMap_.size() +
           3 * orderedLandmarks_.size();
  }

  void updateCovarianceIndex();

  // landmark parameters estimated by the filter, ordered according to the covariance matrix.
  std::deque<PointLandmark> orderedLandmarks_;

  Eigen::MatrixXd  covariance_;

  /// members for visualization
  MapPointVector slamLandmarks_;
};
}  // namespace swift_vio

#endif // SLIDINGWINDOWFILTER_H
