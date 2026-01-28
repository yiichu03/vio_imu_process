/**
 * @file EstimatorBase.h EstimatorBase class for swift_vio Estimators
 */

#ifndef ESTIMATORBASE_H
#define ESTIMATORBASE_H

#include <mutex>
#include <vector>

#include "loop_closure/KeyframeForLoopDetection.hpp"

#include <okvis/ceres/Map.hpp>

#include "swift_vio/CameraRig.hpp"
#include "swift_vio/InitialNavState.hpp"

#include "swift_vio/imu/ImuRig.hpp"
#include "swift_vio/States.h"
#include "swift_vio/MapPoint.h"
#include "swift_vio/messages.h"
#include "swift_vio/MultiFrame.hpp"
#include "swift_vio/imu/BoundedImuDeque.hpp"
#include "swift_vio/VioVisualizer.hpp"

namespace swift_vio {

/// @brief Enum to define the status of initialization
enum class InitializationStatus {
  NotStarted = 0,
  Ongoing = 1,
  RunningNonlinEst,
};

class EstimatorBase
{
public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EstimatorBase(const okvis::EstimatorOptions &estimatorOptions);

  /**
   * @brief initialize from another estimator.
   * @param other
   */
  virtual void initializeFrom(std::shared_ptr<EstimatorBase> other, ConstMultiFramePtr currentNFrame);

  /**
   * @brief initialize from a given navigation state of pose and speed
   * @param initialState Because the estimators use a world frame with z along negative gravity,
   * the initialState needs to respect this.
   * @param currentNFrame
   */
  virtual void initializeFromState(const InitialNavState &initialState, ConstMultiFramePtr currentNFrame);

  virtual ~EstimatorBase();

  virtual std::string typeInfo() const {
    return "EstimatorBase";
  }

  /// @name Sensor configuration related
  /// @{
  /**
   * @brief Add a camera to the configuration. Sensors can only be added and never removed.
   * @param cameraNoiseParameters The parameters that tell how to estimate extrinsics.
   * @return Index of new camera.
   */
  int addCameraParameterStds(const okvis::CameraNoiseParameters&
                                     cameraNoiseParameters);
  /**
   * @brief addCameraSystem add the cameraSystem to the estimator.
   * @param cameras
   */
  virtual void addCameraSystem(const okvis::cameras::NCameraSystem& cameras);

  void addCameraSystem(const swift_vio::CameraRig& cameras);

  /**
   * @brief Add an IMU to the configuration.
   * @warning Currently there is only one IMU supported.
   * @param imuParameters The IMU parameters.
   * @return index of IMU.
   */
  virtual int addImu(const okvis::ImuParameters &imuParameters);

  /// @}

  /// @name Setters
  /// @{
  /**
   * @brief Set a time limit for the optimization process.
   * @param[in] timeLimit Time limit in seconds. If timeLimit < 0 the time limit is removed.
   * @param[in] minIterations minimum iterations the optimization process should do
   *            disregarding the time limit.
   * @return True if successful.
   */
  bool setOptimizationTimeLimit(double /*timeLimit*/) {
    return true;
  }

  void setEstimatorOptions(const okvis::EstimatorOptions& estimatorOptions) {
    estimatorOptions_ = estimatorOptions;
  }

  virtual void setPointLandmarkOptions(const PointLandmarkOptions& plOptions) {
    pointLandmarkOptions_ = plOptions;
  }

  void setInitialNavState(const swift_vio::InitialNavState& rhs) {
     initialNavState_ = rhs;
  }

  void setInitializationStatus(InitializationStatus status) {
    initStatus_ = status;
  }

  void setPoseGraphOptions(const swift_vio::PoseGraphOptions& pgp) {
    poseGraphOptions_ = pgp;
  }

  /**
   * @brief Set pose for a given pose ID and the linearization point for position.
   * @warning Currently this function is only used at initialization stage, because
   * it accesses the optimization graph, so not very fast.
   * @param[in] poseId ID of the pose that should be changed.
   * @param[in] T_WS new homogeneous transformation.
   * @return True if successful.
   */
  bool set_T_WS(uint64_t poseId, const okvis::kinematics::Transformation & T_WS);

  /**
   * @brief Set the speeds and IMU biases for a given pose ID and the linearization point for velocity.
   * @warning Currently this function is only used at initialization stage because
   * it accesses the optimization graph, so not very fast.
   * @param[in] poseId ID of the pose to change corresponding speeds and biases for.
   * @param[in] imuIdx index of IMU to get biases for. As only one IMU is supported this is always 0.
   * @param[in] speedAndBias new speeds and biases.
   * @return True if successful.
   */
  bool setSpeed(uint64_t poseId, const Eigen::Vector3d &speed);

  bool setImuBiases(uint64_t poseId, size_t imuIdx, const Eigen::Matrix<double, 6, 1> &bgba);

  /**
   * @brief Add a pose to the state.
   * @param multiFrame Matched multiFrame.
   * @param imuMeasurements IMU measurements from last state to new one.
   * @param asKeyframe Is this new frame a keyframe?
   * @return True if successful.
   */
//  virtual bool addStates(swift_vio::MultiFramePtr /*multiFrame*/,
//                 const okvis::ImuMeasurementDeque &/*imuMeasurements*/,
//                 bool /*asKeyframe*/) {
//    return true;
//  }

  /// @}

  /**
   * @brief add landmarks from the database to the estimator.
   * @param databaseLandmarks with positions and variances.
   */
  virtual void addLandmarksFromDatabase(const Eigen::AlignedMap<uint64_t, PointAndVariance> &/*databaseLandmarks*/) {}

  virtual void estimate(std::shared_ptr<const VisualMatcherOutput> /*featureMatches*/) {}


  /**
   * @brief Applies the dropping/marginalization strategy according to the RSS'13/IJRR'14 paper.
   *        The new number of frames in the window will be numKeyframes+numImuFrames.
   * @param removedLandmarks Get the landmarks that were removed by this operation.
   * @return True if successful.
   */
  virtual bool applyMarginalizationStrategy(MapPointVector &/*removedLandmarks*/) {
    return true;
  }

  virtual bool computeErrors(
      const okvis::kinematics::Transformation &ref_T_WS,
      const Eigen::Vector3d &ref_v_WS, const Eigen::Matrix<double, 6, 1> &biases,
      const okvis::ImuParameters &refImuParameters,
      std::shared_ptr<const swift_vio::CameraRig> refCameraSystem,
      Eigen::VectorXd *errors) const;

  /**
   * @brief computeCovariance compute covariance by okvis marginalization module
   * which handles rank deficiency caused by low-disparity landmarks.
   * @param cov covariance of p_WS, q_WS, v_WS, b_g, b_a.
   * @return true if covariance is computed successfully, false otherwise.
   */
  virtual bool computeCovariance(Eigen::MatrixXd */*cov*/) const {
    return false;
  }

  /// @name Getters for landmarks.
  /// @{
  /**
   * @brief Checks whether the landmark is added to the estimator.
   * @warning Thread Unsafe.
   * @param landmarkId The ID.
   * @return True if added.
   */
  bool isLandmarkAdded(uint64_t landmarkId) const {
    return landmarksMap_.find(landmarkId) != landmarksMap_.end();
  }

  /**
   * @brief Checks whether the landmark is initialized.
   * @warning Thread Unsafe.
   * @param landmarkId The ID.
   * @return True if initialised.
   */
  bool isLandmarkInitialized(uint64_t landmarkId) const {
    return landmarksMap_.at(landmarkId).isInitialized();
  }

  /**
   * @brief Get a specific landmark.
   * @warning Thread Unsafe.
   * @param[in]  landmarkId ID of desired landmark.
   * @param[out] mapPoint Landmark information, such as quality, coordinates etc.
   * @return True if successful.
   */
  bool getLandmark(uint64_t landmarkId, MapPoint& mapPoint) const
  {
    if (landmarksMap_.find(landmarkId) == landmarksMap_.end()) {
      OKVIS_THROW_DBG(Exception,"landmark with id = "<<landmarkId<<" does not exist.")
      return false;
    }
    mapPoint = landmarksMap_.at(landmarkId);
    return true;
  }

  /**
   * @brief getLandmarks
   * @warning Thread Unsafe.
   * @param landmarks
   * @return
   */
  size_t getLandmarks(PointMap & landmarks) const
  {
    landmarks = landmarksMap_;
    return landmarksMap_.size();
  }

  /**
   * @brief getLandmarks
   * @warning Thread Unsafe.
   * @param landmarks
   * @return
   */
  size_t getLandmarks(MapPointVector &landmarks) const
  {
    landmarks.clear();
    landmarks.reserve(landmarksMap_.size());
    for(PointMap::const_iterator it=landmarksMap_.begin(); it!=landmarksMap_.end(); ++it){
      landmarks.emplace_back(it->second);
    }
    return landmarksMap_.size();
  }

  virtual size_t getCurrentlyObservedLandmarks(MapPointVector *landmarks) const {
    return getLandmarks(*landmarks);
  }

  size_t getMarginalizedLandmarks(MapPointVector *landmarks) const {
    *landmarks = marginalizedLandmarks_;
    return marginalizedLandmarks_.size();
  }

  void frameLandmarkIds(uint64_t nframeId, size_t camId, std::vector<uint64_t> *landmarkIds) const {
    *landmarkIds = nframeToLandmarkIds_.at(nframeId).landmarkIds(camId);
  }

  /// @}

  /// @name Getters for estimation status.
  /// @{
  const okvis::EstimatorOptions& estimatorOptions() const {
    return estimatorOptions_;
  }

  bool wellInitialized() const {
    return initStatus_ == InitializationStatus::RunningNonlinEst;
  }

  bool get_T_WS(uint64_t poseId, okvis::kinematics::Transformation &T_WS) const;

  bool getSpeed(uint64_t poseId, Eigen::Vector3d &v_WS) const;

  void getEstimatedCameraSystem(CameraRig *cameraSystem) const;

  void getEstimatedCameraSystem(okvis::cameras::NCameraSystem *cameraSystem) const;

  void getEstimatedImuRig(ImuRig *imuRig) const;

  void getImuBiases(uint64_t poseId, size_t imuIdx, Eigen::Matrix<double, 6, 1> &biases) const;

  /**
   * @brief getCameraSensorExtrinsics get extrinsic parameters T_BC for a camera.
   * @param cameraIdx
   * @param T_BCi
   * @return
   */
  bool getCameraSensorExtrinsics(
      size_t cameraIdx,
      okvis::kinematics::Transformation& T_BCi) const {
    T_BCi = cameraRig_.getCameraExtrinsic(cameraIdx);
    return true;
  }

  /**
   * @brief get variable extrinsic parameters of camIdx.
   * @param[out] extrinsicParams variable extrinsic parameters refined by the estimator.
   * @param camIdx
   * @return
   */
  virtual void getVariableCameraExtrinsics(
      uint64_t poseId, size_t camIdx,
      Eigen::Matrix<double, Eigen::Dynamic, 1> *extrinsicParams) const;

  virtual void getVariableCameraIntrinsics(
      uint64_t poseId, size_t camIdx,
      Eigen::Matrix<double, Eigen::Dynamic, 1> *intrinsicParams) const;

  virtual void getImuAugmentedStatesEstimate(
      uint64_t poseId,
      size_t imuId,
      Eigen::Matrix<double, Eigen::Dynamic, 1>* extraParams) const;

  virtual bool getDesiredStdevs(Eigen::VectorXd *desiredStdevs) const;

  std::vector<std::string> getCameraParamLabels(bool minimal = false) const;

  /**
   * @brief get std. dev. of state for nav state (p,q,v), imu(bg ba), and optionally
   * imu augmented intrinsic parameters, camera extrinsic, intrinsic, td, tr.
   * @param stateStd
   * @return true if std. dev. of states are computed successfully.
   */
  virtual bool getStateStd(Eigen::Matrix<double, Eigen::Dynamic, 1> */*stateStd*/) const {
    return true;
  }

  /**
   * @brief getOdometryConstraintsForKeyframe
   * @pre T_WB in queryKeyframe is set properly.
   * @param queryKeyframe
   * @return
   */
  virtual bool getOdometryConstraintsForKeyframe(
      std::shared_ptr<swift_vio::LoopQueryKeyframeMessage> /*queryKeyframe*/) const;

  /**
   * @brief get the latest keyframe and its info which is used for loop detection.
   */
  bool getLoopQueryKeyframeMessage(
      const std::shared_ptr<const swift_vio::MultiFrame>& multiFrame, 
      const okvis::cameras::NCameraSystem& cameraSystem,
      const std::vector<size_t>& lcdCameras,
      std::shared_ptr<swift_vio::LoopQueryKeyframeMessage>* queryKeyframe) const;
  /// @}

  /// @name Getters for state map.
  /// @{
  uint64_t currentKeyframeId() const {
    for (std::map<uint64_t, States>::const_reverse_iterator rit = statesMap_.rbegin();
        rit != statesMap_.rend(); ++rit) {
      if (rit->second.isKeyframe) {
        return rit->first;
      }
    }
    OKVIS_THROW_DBG(Exception, "no keyframes existing...");
    return 0;
  }

  uint64_t frameIdByAge(size_t age) const {
    std::map<uint64_t, States>::const_reverse_iterator rit = statesMap_.rbegin();
    for(size_t i=0; i<age; ++i){
      ++rit;
      OKVIS_ASSERT_TRUE_DBG(Exception, rit != statesMap_.rend(),
                         "requested age " << age << " out of range.");
    }
    return rit->first;
  }

  size_t numKeyframesInState() const {
    int num = 0;
    for (auto it = statesMap_.begin(); it != statesMap_.end(); ++it) {
      if (it->second.isKeyframe) {
        ++num;
      }
    }
    return num;
  }

  uint64_t currentFrameId() const {
    OKVIS_ASSERT_TRUE_DBG(Exception, statesMap_.size()>0, "no frames added yet.")
    return statesMap_.rbegin()->first;
  }

  okvis::Time currentFrameTimestamp() const {
    OKVIS_ASSERT_TRUE_DBG(Exception, statesMap_.size() > 0,
                          "no frames added yet.")
    return statesMap_.rbegin()->second.timestamp;
  }

  uint64_t oldestFrameId() const {
    OKVIS_ASSERT_TRUE_DBG(Exception, statesMap_.size() > 0,
                          "no frames added yet.")
    return statesMap_.begin()->first;
  }

  okvis::Time oldestFrameTimestamp() const {
    return statesMap_.begin()->second.timestamp;
  }

  size_t statesMapSize() const {
    return statesMap_.size();
  }

  /**
   * @brief Checks if a particular frame is a keyframe.
   * @param[in] frameId ID of frame to check.
   * @return True if the frame is a keyframe.
   */
  bool isKeyframe(uint64_t frameId) const {
    return statesMap_.at(frameId).isKeyframe;
  }

  bool isFrameInStateMap(uint64_t frameId) const {
    return statesMap_.find(frameId) != statesMap_.end();
  }

  /// @}

  /// @name printing methods.
  /// @{
  virtual bool printStatesAndStdevs(std::ostream& stream, const Eigen::MatrixXd *covariance) const;

  void printNavState(std::ostream& stream, uint64_t poseId) const;

  void printImuParameters(std::ostream& stream, uint64_t poseId) const;

  void printGravityParameters(std::ostream &stream) const;

  void printCameraParameters(std::ostream &stream, uint64_t poseId) const;

  virtual void printTrackLengthHistogram(std::ostream& /*stream*/) const {}

  virtual std::vector<std::string> variableLabels() const;

  virtual std::vector<std::string> perturbationLabels() const;

  /**
   * @brief headerLine
   * @param delimiter
   * @return the header line for the state variables
   */
  virtual std::string headerLine(const std::string delimiter=" ") const;

  /**
   * @brief rmseHeaderLine
   * @param delimiter
   * @return header line for the RMSE of the state variables.
   */
  virtual std::string rmseHeaderLine(const std::string delimiter=" ") const;
  /// @}

protected:
  okvis::Time removeState(uint64_t stateId);

  void correctPoseByCameraMotion(
      const RelativePoseMapType &relPoseToPriorFrames,
      okvis::kinematics::Transformation *T_WS, Eigen::Vector3d */*v_WS*/) const;


  // the following keeps track of all the states at different time instances (key=poseId)
  StateMap statesMap_; ///< Buffer for currently considered states.

  std::shared_ptr<okvis::ceres::Map> mapPtr_; ///< For generality, we keep state variables in okvis::Map.

  std::unordered_map<uint64_t, BareMultiFrame> nframeToLandmarkIds_;

  // the following are updated after the optimization
  PointMap landmarksMap_; ///< Contains all the landmarks in the local map and their observations (synched after optimisation).

  // parameters
  okvis::CameraNoiseParametersVec cameraNoiseParametersVec_; ///< Extrinsics parameters.
  std::vector<std::shared_ptr<okvis::ImuNoiseParameters>> imuParametersVec_; ///< IMU parameters.

  // An evolving camera rig to store the optimized camera
  // parameters and interface with the camera models.
  swift_vio::CameraRig cameraRig_;

  // An evolving imu rig to store the optimized imu parameters and
  // interface with the IMU models.
  swift_vio::ImuRig imuRig_;

  // sequential imu measurements covering states in the estimator
  swift_vio::BoundedImuDeque inertialMeasForStates_;

  // initial nav state, (position, orientation, and velocity), and their stds.
  swift_vio::InitialNavState initialNavState_;

  okvis::EstimatorOptions estimatorOptions_;

  PointLandmarkOptions pointLandmarkOptions_; // see PointLandmarkModels.hpp

  PoseGraphOptions poseGraphOptions_;

  InitializationStatus initStatus_;

  MapPointVector marginalizedLandmarks_;

public:
  // Half length of the window centered at the epoch of a pose state variable.
  // It specifies the time window from which the IMU data will be copied for this pose state.
  // There IMU data will be used to predict the camera pose for an observation in the window
  // captured by a rolling shutter camera.
  // half_window_ is recommended to take a value greater than (t_d + t_r) / 2.
  static const okvis::Duration half_window_;


};

/// \brief copy calibration parameters from estimator to optimization result.
/// \warning thread unsafe. Use it only in the estimation thread.
template <typename EstimatorBaseT>
void getCalibrationParameters(
    std::shared_ptr<const EstimatorBaseT> estimator, uint64_t latestNFrameId,
    OptimizationResults *result, size_t numCameras) {
  result->nframeId = latestNFrameId;
  result->isKeyframe = estimator->isKeyframe(latestNFrameId);

  result->vector_of_T_SCi.clear();
  result->variableCameraParams_.resize(numCameras);
  for (size_t i = 0u; i < numCameras; ++i) {
    okvis::kinematics::Transformation T_SC;
    estimator->getCameraSensorExtrinsics(i, T_SC);
    result->vector_of_T_SCi.emplace_back(T_SC);

    Eigen::VectorXd optimizedExtrinsicCoeffs;
    estimator->getVariableCameraExtrinsics(latestNFrameId, i, &optimizedExtrinsicCoeffs);

    Eigen::VectorXd optimizedIntrinsics;
    estimator->getVariableCameraIntrinsics(latestNFrameId, i, &optimizedIntrinsics);

    result->variableCameraParams_.at(i).resize(
        optimizedExtrinsicCoeffs.size() + optimizedIntrinsics.size(), 1);
    result->variableCameraParams_.at(i).head(optimizedExtrinsicCoeffs.size()) =
        optimizedExtrinsicCoeffs;
    result->variableCameraParams_.at(i).tail(optimizedIntrinsics.size()) =
        optimizedIntrinsics;
  }

  estimator->getImuAugmentedStatesEstimate(latestNFrameId, 0u, &result->imuExtraParams_);
  estimator->getStateStd(&result->stateStd_);
}

// saving optimized state and saving it in OptimizationResults struct
template <typename EstimatorBaseT>
void getEstimationResult(
    std::shared_ptr<const EstimatorBaseT> estimator, uint64_t latestNFrameId,
    OptimizationResults *result, size_t numCameras, bool publishImuPropagatedState) {
  okvis::kinematics::Transformation latest_T_WS;
  estimator->get_T_WS(latestNFrameId, latest_T_WS);
  Eigen::Vector3d latest_v_WS;
  estimator->getSpeed(latestNFrameId, latest_v_WS);
  size_t imuIdx = 0u;
  Eigen::Matrix<double, 6, 1> biases;
  estimator->getImuBiases(latestNFrameId, imuIdx, biases);

  okvis::Time latestStateTime = estimator->currentFrameTimestamp();

  // if we publish the state after each IMU propagation we do not need to
  // publish it here.
  if (!publishImuPropagatedState) {
    result->T_WS = latest_T_WS;
    result->speedAndBiases.head<3>() = latest_v_WS;
    result->speedAndBiases.tail<6>() = biases;
    result->stamp = latestStateTime;
    result->onlyPublishLandmarks = false;
  } else {
    result->onlyPublishLandmarks = true;
  }
  estimator->getCurrentlyObservedLandmarks(&(result->landmarksVector));
  estimator->getMarginalizedLandmarks(&(result->transferredLandmarks));
  getCalibrationParameters(estimator, latestNFrameId, result, numCameras);
}

/**
 * Get visualization data for currentNFrame.
 * Note that in the standard MSCKF mode, the landmarks observed in the current
 * NFrame are always not triangulated. For the sake of visualization, we have to
 * triangulate the premature landmarks observed in the current NFrame
 * (see showPrematureLandmarks).
 */
template <typename EstimatorBaseT, typename MultiFrameT>
void getVisualizationData(
    std::shared_ptr<const EstimatorBaseT> estimator,
    std::shared_ptr<const MultiFrameT> currentNFrame,
    std::shared_ptr<const MultiFrameT> closestKeyframe,
    VioVisualizer::VisualizationData::Ptr visualizationDataPtr) {
  visualizationDataPtr->observations.resize(currentNFrame->numKeypoints());
  MapPoint landmark;
  okvis::ObservationVector::iterator it =
      visualizationDataPtr->observations.begin();
  for (size_t camIndex = 0; camIndex < currentNFrame->numFrames(); ++camIndex) {
    std::vector<uint64_t> landmarkIds;
    estimator->frameLandmarkIds(currentNFrame->id(), camIndex, &landmarkIds);
    for (size_t k = 0; k < currentNFrame->numKeypoints(camIndex); ++k) {
      CHECK(it != visualizationDataPtr->observations.end()) <<
                            "Observation-vector not big enough";
      it->keypointIdx = k;
      currentNFrame->getKeypoint(camIndex, k, it->keypointMeasurement);
      currentNFrame->getKeypointSize(camIndex, k, it->keypointSize);
      it->cameraIdx = camIndex;
      it->frameId = currentNFrame->id();
      it->landmarkId = landmarkIds[k];
      if (estimator->isLandmarkAdded(it->landmarkId)) {
        estimator->getLandmark(it->landmarkId, landmark);
        it->landmark_W = landmark.pointHomog;
        it->numObservations = static_cast<int>(landmark.observations.size());
        if (estimator->isLandmarkInitialized(it->landmarkId)) {
          it->isInitialized = true;
        } else {
          it->isInitialized = false;
        }
      } else {
        // set to infinity to tell visualizer that landmark is not added.
        it->landmark_W = Eigen::Vector4d(0, 0, 0, 0);
      }
      ++it;
    }
  }
  visualizationDataPtr->keyFrames = adaptMultiFramePtr(closestKeyframe);
  estimator->get_T_WS(closestKeyframe->id(),
                      visualizationDataPtr->T_WS_keyFrame);

  visualizationDataPtr->currentFrames = adaptMultiFramePtr(currentNFrame);
}
}  // namespace swift_vio

#endif // ESTIMATORBASE_H
