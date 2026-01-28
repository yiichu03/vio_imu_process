/**
 * @file EstimatorBase.cpp EstimatorBase class implementation for swift_vio Estimators
 */

#include "swift_vio/EstimatorBase.h"

#include "swift_vio/EstimationHelper.h"
#include "swift_vio/IoUtil.hpp"
#include "swift_vio/ceres/NormalVectorParameterBlock.hpp"
#include "okvis/ceres/PoseParameterBlock.hpp"
#include "swift_vio/ceres/EuclideanParamBlockSizedLin.hpp"

namespace swift_vio {
EstimatorBase::EstimatorBase(const okvis::EstimatorOptions &estimatorOptions) :
  mapPtr_(new okvis::ceres::Map()), estimatorOptions_(estimatorOptions) {}

void EstimatorBase::initializeFromState(const InitialNavState &/*initialState*/,
                                   ConstMultiFramePtr /*currentNFrame*/) {}

void EstimatorBase::initializeFrom(std::shared_ptr<EstimatorBase> other,
                                   ConstMultiFramePtr /*nframe*/) {
  statesMap_ = other->statesMap_;
  mapPtr_ = other->mapPtr_;
  nframeToLandmarkIds_ = other->nframeToLandmarkIds_;
  landmarksMap_ = other->landmarksMap_;
  cameraNoiseParametersVec_ = other->cameraNoiseParametersVec_; ///< Extrinsics parameters.
  imuParametersVec_ = other->imuParametersVec_;
  cameraRig_ = other->cameraRig_;
  imuRig_ = other->imuRig_;
  inertialMeasForStates_ = other->inertialMeasForStates_;
  initialNavState_ = other->initialNavState_;
  estimatorOptions_ = other->estimatorOptions_;
  pointLandmarkOptions_ = other->pointLandmarkOptions_;
  poseGraphOptions_ = other->poseGraphOptions_;
  initStatus_ = other->initStatus_;
  marginalizedLandmarks_ = other->marginalizedLandmarks_;
}

EstimatorBase::~EstimatorBase() {

}

int EstimatorBase::addCameraParameterStds(
    const okvis::CameraNoiseParameters & cameraNoiseParameters)
{
  cameraNoiseParametersVec_.push_back(cameraNoiseParameters);
  return cameraNoiseParametersVec_.size() - 1;
}

void EstimatorBase::addCameraSystem(const okvis::cameras::NCameraSystem& cameras) {
  cameraRig_ = CameraRig::deepCopy(cameras);
}

void EstimatorBase::addCameraSystem(const swift_vio::CameraRig& cameras) {
  cameraRig_ = cameras.deepCopy();
}

int EstimatorBase::addImu(const okvis::ImuParameters & imuParameters)
{
  if(imuParametersVec_.size()>0u){
    LOG(ERROR) << "only one IMU currently supported";
    return -1;
  }
  imuParametersVec_.emplace_back(new okvis::ImuParameters(imuParameters));
  imuParametersVec_.back()->imuIdx = imuParametersVec_.size() - 1;
  imuRig_.addImu(imuParameters);
  return imuParametersVec_.size() - 1;
}

void EstimatorBase::printNavState(std::ostream& stream, uint64_t poseId) const {
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParamBlockPtr =
      std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
          mapPtr_->parameterBlockPtr(poseId));
  okvis::kinematics::Transformation T_WS = poseParamBlockPtr->estimate();

  const States& stateInQuestion = statesMap_.at(poseId);
  okvis::Time currentTime = stateInQuestion.timestamp;

  Eigen::Quaterniond q_WS = T_WS.q();
  if (q_WS.w() < 0) {
    q_WS.coeffs() *= -1;
  }
  stream << currentTime << " " << poseId
         << " " << T_WS.parameters().transpose().format(swift_vio::kSpaceInitFmt);

  uint64_t speedId = stateInQuestion.global.at(GlobalStates::v_WS).id;
  std::shared_ptr<okvis::ceres::SpeedParameterBlock> sParamBlockPtr =
      std::static_pointer_cast<okvis::ceres::SpeedParameterBlock>(
          mapPtr_->parameterBlockPtr(speedId));
  Eigen::Vector3d speed = sParamBlockPtr->estimate();
  stream << " " << speed.transpose().format(swift_vio::kSpaceInitFmt);
}

void EstimatorBase::printImuParameters(std::ostream &stream,
                                       uint64_t poseId) const {
  for (size_t imuIdx = 0; imuIdx < imuRig_.numImus(); ++imuIdx) {
    Eigen::Matrix<double, 6, 1> biases;
    getImuBiases(poseId, imuIdx, biases);
    stream << " " << biases.transpose().format(kSpaceInitFmt);
    Eigen::Matrix<double, Eigen::Dynamic, 1> extraParams;
    getImuAugmentedStatesEstimate(poseId, imuIdx, &extraParams);
    if (extraParams.size() > 0) {
      stream << " " << extraParams.transpose().format(kSpaceInitFmt);
    }
  }
}

void EstimatorBase::printGravityParameters(std::ostream &stream) const {
  if (imuParametersVec_.at(0)->isGravityDirectionVariable()) {
    uint64_t id = statesMap_.rbegin()
                      ->second.global.at(GlobalStates::GravityDirection)
                      .id;
    Eigen::Vector3d gravity =
        std::static_pointer_cast<okvis::ceres::NormalVectorParameterBlock>(
            mapPtr_->parameterBlockPtr(id))
            ->estimate();
    gravity *= imuParametersVec_.at(0)->g;
    stream << " " << gravity.transpose().format(kSpaceInitFmt);
  }
}

void EstimatorBase::printCameraParameters(std::ostream &stream,
                                       uint64_t poseId) const {
  size_t numCameras = cameraRig_.numCameras();
  for (size_t camIdx = 0u; camIdx < numCameras; ++camIdx) {
    Eigen::VectorXd extrinsicParams;
    getVariableCameraExtrinsics(poseId, camIdx, &extrinsicParams);
    if (extrinsicParams.size())
      stream << " " << extrinsicParams.transpose().format(kSpaceInitFmt);

    Eigen::VectorXd cameraParams;
    EstimationHelper::getVariableCameraIntrinsics(statesMap_.at(poseId), mapPtr_, camIdx,
                                  cameraNoiseParametersVec_.at(camIdx), &cameraParams);
    if (cameraParams.size())
      stream << " " << cameraParams.transpose().format(kSpaceInitFmt);
  }
}

bool EstimatorBase::set_T_WS(uint64_t poseId,
                             const okvis::kinematics::Transformation & T_WS)
{
  auto iter = statesMap_.find(poseId);
  if (iter == statesMap_.end()) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).global.at(GlobalStates::T_WS).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }
  std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlockPtr = mapPtr_
      ->parameterBlockPtr(id);
  std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(parameterBlockPtr)->setEstimate(
      T_WS);
  return true;
}

// Set the speeds and IMU biases for a given pose ID.
bool EstimatorBase::setSpeed(uint64_t poseId, const Eigen::Vector3d &speed)
{
  auto iter = statesMap_.find(poseId);
  if (iter == statesMap_.end()) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).global.at(GlobalStates::v_WS).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }
  std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlockPtr = mapPtr_
      ->parameterBlockPtr(id);
  std::static_pointer_cast<okvis::ceres::SpeedParameterBlock>(parameterBlockPtr)->setEstimate(
      speed);
  return true;
}

bool EstimatorBase::setImuBiases(uint64_t poseId, size_t imuIdx, const Eigen::Matrix<double, 6, 1> &bgba)
{
  const States& stateInQuestion = statesMap_.at(poseId);
  uint64_t biasId = stateInQuestion.sensors.at(SensorStates::Imu)
                      .at(imuIdx)
                      .at(ImuSensorStates::Bias)
                      .id;
  std::shared_ptr<okvis::ceres::BiasParameterBlock> bParamBlockPtr =
      std::static_pointer_cast<okvis::ceres::BiasParameterBlock>(
          mapPtr_->parameterBlockPtr(biasId));
  bParamBlockPtr->setEstimate(bgba);
  return true;
}

bool EstimatorBase::computeErrors(
    const okvis::kinematics::Transformation &ref_T_WS,
    const Eigen::Vector3d &ref_v_WS, const Eigen::Matrix<double, 6, 1> &biasRef,
    const okvis::ImuParameters &/*refImuParameters*/,
    std::shared_ptr<const CameraRig> /*refCameraSystem*/,
    Eigen::VectorXd *errors) const {
  errors->resize(15);
  okvis::kinematics::Transformation est_T_WS;
  uint64_t currFrameId = currentFrameId();
  get_T_WS(currFrameId, est_T_WS);
  errors->head<3>() = ref_T_WS.r() - est_T_WS.r();
  Eigen::Matrix3d dR = ref_T_WS.C() * est_T_WS.C().transpose();
  errors->segment<3>(3) = okvis::kinematics::vee(dR);

  Eigen::Vector3d speed;
  Eigen::Matrix<double, 6, 1> biases;
  getSpeed(currFrameId, speed);
  getImuBiases(currFrameId, 0u, biases);

  errors->segment<3>(6) = speed - ref_v_WS;
  errors->segment<3>(9) = biases.head<3>() - biasRef.head<3>();
  errors->segment<3>(12) = biases.tail<3>() - biasRef.tail<3>();
  return true;
}

bool EstimatorBase::get_T_WS(uint64_t poseId,
                             okvis::kinematics::Transformation &T_WS) const {
  if (statesMap_.find(poseId) == statesMap_.end()) {
    OKVIS_THROW(Exception, "pose with id = " << poseId << " does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).global.at(GlobalStates::T_WS).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW(Exception, "pose with id = " << id << " does not exist.")
    return false;
  }
  std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlockPtr =
      mapPtr_->parameterBlockPtr(id);
  std::shared_ptr<okvis::ceres::PoseParameterBlock> stateParameterBlockPtr =
      std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
          parameterBlockPtr);
  T_WS = stateParameterBlockPtr->estimate();
  return true;
}

bool EstimatorBase::getSpeed(uint64_t poseId, Eigen::Vector3d &v_WS) const {
  if (statesMap_.find(poseId) == statesMap_.end()) {
    OKVIS_THROW(Exception, "pose with id = " << poseId << " does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).global.at(GlobalStates::v_WS).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW(Exception, "pose with id = " << id << " does not exist.")
    return false;
  }
  std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlockPtr =
      mapPtr_->parameterBlockPtr(id);
  std::shared_ptr<okvis::ceres::SpeedParameterBlock> stateParameterBlockPtr =
      std::static_pointer_cast<okvis::ceres::SpeedParameterBlock>(
          parameterBlockPtr);
  v_WS = stateParameterBlockPtr->estimate();
  return true;
}

void EstimatorBase::getEstimatedCameraSystem(CameraRig *cameraSystem) const {
  cameraRig_.assignTo(cameraSystem);
}

void EstimatorBase::getEstimatedCameraSystem(okvis::cameras::NCameraSystem *cameraSystem) const {
  cameraRig_.assignTo(cameraSystem);
}

void EstimatorBase::getEstimatedImuRig(ImuRig *imuRig) const {
  *imuRig = imuRig_;
}

void EstimatorBase::getImuBiases(uint64_t poseId, size_t imuIdx, Eigen::Matrix<double, 6, 1> &biases) const {
  const States& stateInQuestion = statesMap_.at(poseId);
  uint64_t biasId = stateInQuestion.sensors.at(SensorStates::Imu)
                      .at(imuIdx)
                      .at(ImuSensorStates::Bias)
                      .id;
  std::shared_ptr<okvis::ceres::BiasParameterBlock> bParamBlockPtr =
      std::static_pointer_cast<okvis::ceres::BiasParameterBlock>(
          mapPtr_->parameterBlockPtr(biasId));
  biases = bParamBlockPtr->estimate();
}

void EstimatorBase::getImuAugmentedStatesEstimate(uint64_t poseId,
    size_t imuId, Eigen::Matrix<double, Eigen::Dynamic, 1> *extraParams) const {
  std::vector<const double *> imuAugmentedParamPtrs;
  std::vector<size_t> imuAugmentedParamDims;
  EstimationHelper::getImuAugmentedParameterPtrs(statesMap_.at(poseId), mapPtr_, imuId,
      &imuAugmentedParamPtrs, &imuAugmentedParamDims);
  swift_vio::getImuAugmentedStatesEstimate(imuAugmentedParamPtrs, imuAugmentedParamDims,
      extraParams, imuRig_.getModelId(imuId));
}

void EstimatorBase::getVariableCameraIntrinsics(uint64_t poseId,
    size_t camIdx,
    Eigen::Matrix<double, Eigen::Dynamic, 1> *intrinsicParams) const {
  EstimationHelper::getVariableCameraIntrinsics(
      statesMap_.at(poseId), mapPtr_, camIdx,
      cameraNoiseParametersVec_.at(camIdx), intrinsicParams);
}

void EstimatorBase::getVariableCameraExtrinsics(uint64_t poseId,
    size_t camIdx, Eigen::Matrix<double, Eigen::Dynamic, 1> *extrinsicParams) const {
  const States &currentState = statesMap_.at(poseId);
  if (!cameraNoiseParametersVec_.at(camIdx).isExtrinsicsFixed()) {
    uint64_t extrinsicId = currentState.sensors.at(SensorStates::Camera)
                               .at(camIdx)
                               .at(CameraSensorStates::T_XCi)
                               .id;
    std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicParamBlockPtr =
        std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
            mapPtr_->parameterBlockPtr(extrinsicId));
    okvis::kinematics::Transformation T_XC = extrinsicParamBlockPtr->estimate();
    swift_vio::ExtrinsicRepToParamValues(
        cameraRig_.getExtrinsicRepId(camIdx), T_XC, extrinsicParams);
  } else {
    extrinsicParams->resize(0);
  }
}

bool EstimatorBase::getDesiredStdevs(Eigen::VectorXd *desiredStdevs) const {
  desiredStdevs->resize(15, 1);
  (*desiredStdevs) << 0.3, 0.3, 0.3, 0.08, 0.08, 0.08, 0.1, 0.1, 0.1, 0.002,
      0.002, 0.002, 0.02, 0.02, 0.02;
  return true;
}

std::vector<std::string> EstimatorBase::getCameraParamLabels(bool minimal) const {
  std::vector<std::string> cameraParamLabels;
  size_t numCameras = cameraNoiseParametersVec_.size();
  for (size_t j = 0u; j < numCameras; ++j) {
    if (!cameraNoiseParametersVec_.at(j).isExtrinsicsFixed()) {
      std::vector<std::string> camExtrinsicLabels;
      if (minimal) {
        ExtrinsicRepToMinDimensionLabels(cameraRig_.getExtrinsicRepId(j),
                                           &camExtrinsicLabels);
      } else {
        ExtrinsicRepToDimensionLabels(cameraRig_.getExtrinsicRepId(j),
                                        &camExtrinsicLabels);
      }
      cameraParamLabels.insert(cameraParamLabels.end(),
                               camExtrinsicLabels.begin(),
                               camExtrinsicLabels.end());
    }

    if (!cameraNoiseParametersVec_.at(j).isIntrinsicsFixed()) {
      std::vector<std::string> camProjectionIntrinsicLabels;
      ProjIntrinsicRepToDimensionLabels(cameraRig_.getProjectionIntrinsicRepId(j),
                                     &camProjectionIntrinsicLabels);
      std::vector<std::string> camDistortionLabels;
      swift_vio::DistortionTypeToDimensionLabels(
          cameraRig_.distortionType(j), &camDistortionLabels);
      cameraParamLabels.insert(cameraParamLabels.end(),
                               camProjectionIntrinsicLabels.begin(),
                               camProjectionIntrinsicLabels.end());
      cameraParamLabels.insert(cameraParamLabels.end(),
                               camDistortionLabels.begin(),
                               camDistortionLabels.end());
    }
    if (!cameraNoiseParametersVec_.at(j).isTimeDelayFixed()) {
      cameraParamLabels.push_back("td[s]");
    }
    if (!cameraNoiseParametersVec_.at(j).isReadoutTimeFixed()) {
      cameraParamLabels.push_back("tr[s]");
    }
  }
  return cameraParamLabels;
}

std::vector<std::string> EstimatorBase::variableLabels() const {
  std::vector<std::string> variableList{
      "p_WB_W_x(m)",   "p_WB_W_y(m)",   "p_WB_W_z(m)",  "q_WB_x",
      "q_WB_y",        "q_WB_z",        "q_WB_w",       "v_WB_W_x(m/s)",
      "v_WB_W_y(m/s)", "v_WB_W_z(m/s)", "b_g_x(rad/s)", "b_g_y(rad/s)",
      "b_g_z(rad/s)",  "b_a_x(m/s^2)",  "b_a_y(m/s^2)", "b_a_z(m/s^2)"};

  std::vector<std::string> imuParameterNames;
  ImuModelToAugmentedDimensionLabels(
      ImuModelNameToId(imuParametersVec_.at(0)->model_name), &imuParameterNames);
  variableList.insert(variableList.end(), imuParameterNames.begin(),
                      imuParameterNames.end());
  if (imuParametersVec_.at(0)->isGravityDirectionVariable()) {
    variableList.push_back("Gravity_x");
    variableList.push_back("Gravity_y");
    variableList.push_back("Gravity_z");
  }
  std::vector<std::string> camParameterNames = getCameraParamLabels();
  variableList.insert(variableList.end(), camParameterNames.begin(),
                      camParameterNames.end());
  return variableList;
}

std::vector<std::string> EstimatorBase::perturbationLabels() const {
  std::vector<std::string> minVarList{
      "p_WB_W_x(m)",   "p_WB_W_y(m)",
      "p_WB_W_z(m)",    "theta_WB_x",        "theta_WB_y",        "theta_WB_z",
      "v_WB_W_x(m/s)", "v_WB_W_y(m/s)", "v_WB_W_z(m/s)",
      "b_g_x(rad/s)",   "b_g_y(rad/s)",  "b_g_z(rad/s)",  "b_a_x(m/s^2)",
      "b_a_y(m/s^2)",   "b_a_z(m/s^2)"};

  std::vector<std::string> imuParameterNames;
  ImuModelToAugmentedDimensionLabels(
      ImuModelNameToId(imuParametersVec_.at(0)->model_name), &imuParameterNames);
  minVarList.insert(minVarList.end(), imuParameterNames.begin(),
                      imuParameterNames.end());
  if (imuParametersVec_.at(0)->isGravityDirectionVariable()) {
    minVarList.push_back("delta_gx");
    minVarList.push_back("delta_gy");
  }
  std::vector<std::string> camParameterNames = getCameraParamLabels(true);
  minVarList.insert(minVarList.end(), camParameterNames.begin(),
                    camParameterNames.end());
  return minVarList;
}

bool EstimatorBase::printStatesAndStdevs(std::ostream& stream, const Eigen::MatrixXd *cov) const {
  uint64_t poseId = statesMap_.rbegin()->first;
  printNavState(stream, poseId);

  printImuParameters(stream, poseId);

  printGravityParameters(stream);

  printCameraParameters(stream, poseId);

  Eigen::VectorXd stateStd;
  if (cov == nullptr) {
    Eigen::MatrixXd covariance;
    computeCovariance(&covariance);
    stateStd = covariance.diagonal().cwiseSqrt();
  } else {
    stateStd = cov->diagonal().cwiseSqrt();
  }
  stream << " " << stateStd.transpose().format(swift_vio::kSpaceInitFmt) << "\n";
  return true;
}

bool EstimatorBase::getOdometryConstraintsForKeyframe(
    std::shared_ptr<swift_vio::LoopQueryKeyframeMessage> queryKeyframe) const {
  int j = 0;
  auto& odometryConstraintList = queryKeyframe->odometryConstraintList_;
  odometryConstraintList.reserve(poseGraphOptions_.maxOdometryConstraintForAKeyframe);
  okvis::kinematics::Transformation T_WBr = queryKeyframe->T_WB_;
  queryKeyframe->setDefaultCovariance();
  auto riter = statesMap_.rbegin();
  for (++riter;  // skip the last frame which is queryKeyframe.
       riter != statesMap_.rend() && j < poseGraphOptions_.maxOdometryConstraintForAKeyframe;
       ++riter) {
    if (riter->second.isKeyframe) {
      okvis::kinematics::Transformation T_WBn;
      get_T_WS(riter->first, T_WBn);
      okvis::kinematics::Transformation T_BnBr = T_WBn.inverse() * T_WBr;
      std::shared_ptr<swift_vio::NeighborConstraintMessage> odometryConstraint(
          new swift_vio::NeighborConstraintMessage(
              riter->first, riter->second.timestamp, T_BnBr, T_WBn));
      odometryConstraintList.emplace_back(odometryConstraint);
      ++j;
    }
  }
  return true;
}

bool EstimatorBase::getLoopQueryKeyframeMessage(
    const std::shared_ptr<const swift_vio::MultiFrame>& multiFrame, 
    const okvis::cameras::NCameraSystem& cameraSystem,
    const std::vector<size_t>& lcdCameras,
    std::shared_ptr<swift_vio::LoopQueryKeyframeMessage>* queryKeyframe) const {
  auto riter = statesMap_.rbegin();
  if (!riter->second.isKeyframe) {
    return false;
  }

  constexpr size_t minNumKeypointsToPublish = 6;
  size_t maxNumKeypoints = 0;
  for (size_t i = 0u; i < lcdCameras.size(); ++i) {
    int origCamId = lcdCameras.at(i);
    maxNumKeypoints = std::max(maxNumKeypoints, multiFrame->getKeypoints(origCamId).size());
  }
  if (maxNumKeypoints < minNumKeypointsToPublish) {
    LOG(INFO) << "Keyframe at " << riter->second.timestamp 
        << " will not be published for LCD for observing only "
        << maxNumKeypoints << " keypoints.";
    return false;
  }

  okvis::kinematics::Transformation T_WBr;
  get_T_WS(riter->first, T_WBr);

  uint64_t queryKeyframeId = riter->first;
  queryKeyframe->reset(new swift_vio::LoopQueryKeyframeMessage(
      queryKeyframeId, riter->second.timestamp, T_WBr, multiFrame, lcdCameras, cameraSystem));
  getOdometryConstraintsForKeyframe(*queryKeyframe);

  auto& keypointIndexForLandmarkList =
      (*queryKeyframe)->keypointIndexForLandmarkList_;
  keypointIndexForLandmarkList.resize(lcdCameras.size());
  auto& landmarkPositionList = (*queryKeyframe)->landmarkPositionList_;
  landmarkPositionList.resize(lcdCameras.size());
  
  for (size_t i = 0u; i < lcdCameras.size(); ++i) {
    int origCamId = lcdCameras.at(i);
    // add 3d landmarks observed in query keyframe's first frame,
    // and corresponding indices into the 2d keypoint list.
    // The local camera frame will be used as their coordinate frame.
    std::vector<uint64_t> landmarkIdList;
    frameLandmarkIds(queryKeyframeId, origCamId, &landmarkIdList);

    size_t numKeypoints = landmarkIdList.size();
    keypointIndexForLandmarkList.at(i).reserve(numKeypoints / 4);
    landmarkPositionList.at(i).reserve(numKeypoints / 4);
    int keypointIndex = 0;
    okvis::kinematics::Transformation T_BrW = T_WBr.inverse();
    for (const uint64_t landmarkId : landmarkIdList) {
      if (landmarkId != 0) {
        auto result = landmarksMap_.find(landmarkId);
        if (result != landmarksMap_.end() && result->second.quality > 1e-6) {
          keypointIndexForLandmarkList.at(i).push_back(keypointIndex);
          Eigen::Vector4d hp_W = result->second.pointHomog;
          Eigen::Vector4d hp_B = T_BrW * hp_W;
          landmarkPositionList.at(i).push_back(hp_B);
        }
      }
      ++keypointIndex;
    }
    (*queryKeyframe)->nframe_->resetDescriptors(i,
          multiFrame->getDescriptors(origCamId));
    (*queryKeyframe)->nframe_->resetKeypoints(i,
          multiFrame->getKeypoints(origCamId));
  }
  return true;
}

std::string EstimatorBase::headerLine(const std::string delimiter) const {
  std::stringstream stream;
  stream << "timestamp(sec)" << delimiter;
  std::vector<std::string> variableList = variableLabels();
  for (const auto& variable : variableList) {
    stream << variable << delimiter;
  }
  std::vector<std::string> minVarList = perturbationLabels();
  for (const auto &variable : minVarList) {
    stream << "std_" << variable << delimiter;
  }
  return stream.str();
}

std::string EstimatorBase::rmseHeaderLine(const std::string delimiter) const {
  std::stringstream ss;
  std::vector<std::string> minVarList = perturbationLabels();
  ss << "timestamp(sec)" << delimiter;
  for (const auto &variable : minVarList) {
    ss << variable << delimiter;
  }
  return ss.str();
}

okvis::Time EstimatorBase::removeState(uint64_t stateId) {
  StateMap::iterator it = statesMap_.find(stateId);
  okvis::Time removedStateTime = it->second.timestamp;
  mapPtr_->removeParameterBlock(it->second.global[GlobalStates::T_WS].id);
  mapPtr_->removeParameterBlock(it->second.global.at(GlobalStates::v_WS).id);

  nframeToLandmarkIds_.erase(stateId);
  statesMap_.erase(it);
  return removedStateTime;
}

void EstimatorBase::correctPoseByCameraMotion(
    const RelativePoseMapType &relPoseToPriorFrames,
    okvis::kinematics::Transformation *T_WS, Eigen::Vector3d */*v_WS*/) const {
  // from relative pose info, choose that one corresponding to a keyframe
  for (const auto &relPoseInfo : relPoseToPriorFrames) {
    auto res = statesMap_.find(relPoseInfo.first.frameId);
    if (res != statesMap_.end() &&
        relPoseInfo.second.status > RansacStatus::FewInliers) {
      okvis::kinematics::Transformation T_SCA, T_WSA, T_SC0;
      getCameraSensorExtrinsics(relPoseInfo.second.cam1.cameraIndex, T_SCA);
      get_T_WS(relPoseInfo.first.frameId, T_WSA);
      getCameraSensorExtrinsics(relPoseInfo.second.cam2.cameraIndex, T_SC0);
      switch (relPoseInfo.second.status) {
      case RansacStatus::RotationOnlyOk:
        LOG(INFO) << "Correcting IMU pose with camera motion from 2D-2D RANSAC "
                     "orientation only.";

        *T_WS = T_WSA * T_SCA * relPoseInfo.second.T_C1C2 * T_SC0.inverse();
        break;
      case RansacStatus::FundamentalMatOk: {
        LOG(INFO) << "Correcting IMU pose with camera motion from 2D-2D RANSAC.";

        // initialize with projected length according to motion prior.
        okvis::kinematics::Transformation T_C1C2_backend =
            T_SCA.inverse() * T_WSA.inverse() * (*T_WS) * T_SC0;
        okvis::kinematics::Transformation T_C1C2_frontend =
            relPoseInfo.second.T_C1C2;
        T_C1C2_frontend.setTranslation(
            T_C1C2_frontend.r() *
            std::max(0.0, double(T_C1C2_frontend.r().transpose() *
                                 T_C1C2_backend.r())));

        *T_WS = T_WSA * T_SCA * T_C1C2_frontend * T_SC0.inverse();
      } break;
      case RansacStatus::PnPOk:
        break;
      default:
        break;
      }
      break;
    }
  }
}

const okvis::Duration EstimatorBase::half_window_(0.5);

}  // namespace swift_vio
