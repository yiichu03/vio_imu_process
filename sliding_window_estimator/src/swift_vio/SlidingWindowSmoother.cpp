#include "swift_vio/SlidingWindowSmoother.h"

#include "swift_vio/EstimationHelper.h"
#include "swift_vio/InitialNavState.hpp"
#include "swift_vio/TimingUtilities.h"
#include "swift_vio/ceres/EuclideanParamBlockSized.hpp"
#include "swift_vio/ceres/InverseDepthPointBlock.hpp"
#include "swift_vio/ceres/RsReprojectionErrorAidpAdapter.hpp"
#include "swift_vio/FeatureTriangulation.hpp"
#include <swift_vio/PointLandmark.hpp>

#include <okvis/IdProvider.hpp>
#include "okvis/ceres/ImuError.hpp"
#include <okvis/timing/Timers.hpp>

namespace swift_vio {
SlidingWindowSmoother::SlidingWindowSmoother(const okvis::EstimatorOptions &options) :
  EstimatorBase(options), cauchyLossFunctionPtr_(new ::ceres::CauchyLoss(1)),
  huberLossFunctionPtr_(new ::ceres::HuberLoss(1)), marginalizationResidualId_(0), numNFrames_(0u) {
  initStatus_ = InitializationStatus::Ongoing;
}


void SlidingWindowSmoother::initializeFrom(std::shared_ptr<EstimatorBase> other,
                                         ConstMultiFramePtr currentNFrame) {
  EstimatorBase::initializeFrom(other, currentNFrame);
  if (other->typeInfo() == "SlidingWindowSmoother") {
    std::shared_ptr<const VioInitializer> initializer =
        std::static_pointer_cast<VioInitializer>(other);
    landmarkParamizationPtr_ = initializer->landmarkParamizationPtr_;
    cauchyLossFunctionPtr_ = initializer->cauchyLossFunctionPtr_;
    huberLossFunctionPtr_ = initializer->huberLossFunctionPtr_;
    marginalizationErrorPtr_ = initializer->marginalizationErrorPtr_;
    marginalizationResidualId_ = initializer->marginalizationResidualId_;
    numNFrames_ = initializer->numNFrames_;
  } else {
    throw std::invalid_argument("Initializing a SlidingWindowSmoother from " + other->typeInfo() + " is unsupported!");
  }
}

void SlidingWindowSmoother::addLandmarksFromDatabase(
    const Eigen::AlignedMap<uint64_t, PointAndVariance> &/*databaseLandmarks*/) {
  // add these variables to mapPtr_
  // add them to landmarksMap_, and update their status in MapPoint, inState.
}

void SlidingWindowSmoother::setPointLandmarkOptions(const swift_vio::PointLandmarkOptions& plOptions) {
  pointLandmarkOptions_ = plOptions;
  swift_vio::Feature::optimization_config.max_depth =
      plOptions.triangulationMaxDepth;
  landmarkParamizationPtr_ = createLandmarkLocalParameterization(InverseDepthParameterization::kModelId);
}

void SlidingWindowSmoother::estimate(
    std::shared_ptr<const VisualMatcherOutput> syncedData) {
  okvis::Time stateTime = TimingUtilities::computeStateTimestamp(cameraRig_, syncedData->currentNFrame);

  inertialMeasForStates_.push_back(syncedData->imuMeasurements);

  // Only using keyframes seems unstable at initialization.
//  if (syncedData->currentNFrame->isKeyframe()) {
  EstimationHelper::updateFeatureTracks(syncedData->featureTracks,
                                        syncedData->currentNFrame,
                                        &landmarksMap_, &nframeToLandmarkIds_);
  //  } else {
  //    LOG(INFO) << "Skip non-keyframe " << syncedData->currentNFrame->id();
  //    return;
  //  }

  if (numNFrames_ == 0) {
    OKVIS_ASSERT_TRUE(Exception, syncedData->currentNFrame->isKeyframe(), "First NFrame should be a keyframe!");
    okvis::kinematics::Transformation T_WS;
    if (initialNavState_.initializeToCustomPose) {
      T_WS.set(initialNavState_.p_WS, initialNavState_.q_WS);
    } else {
      bool success = swift_vio::initPoseFromImu(syncedData->imuMeasurements, stateTime, T_WS);
      LOG_IF(WARNING, !success) << "Pose could not be initialized from imu measurements.";
      initialNavState_.updatePose(T_WS, stateTime);
    }
    statesMap_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(syncedData->currentNFrame->id()),
        std::forward_as_tuple(syncedData->currentNFrame->id(), stateTime,
                              syncedData->currentNFrame->isKeyframe()));
    States &currentState = statesMap_.rbegin()->second;

    EstimationHelper::setImuWindow(inertialMeasForStates_, &currentState, half_window_);

    EstimationHelper::addNavStateVariables(
        T_WS, initialNavState_.v_WS,
        &currentState, mapPtr_, false);

    EstimationHelper::addGravityVariable(
          imuParametersVec_.at(0)->gravityDirection(),
          &currentState, mapPtr_,
          imuParametersVec_.at(0)->isGravityDirectionVariable());

    EstimationHelper::addImuParameters(imuRig_, imuParametersVec_,
                                       &currentState, mapPtr_);

    EstimationHelper::addCameraParameters(cameraRig_, cameraNoiseParametersVec_,
                                          &currentState,
                                          mapPtr_);

    EstimationHelper::addNavStatePriorFactor(initialNavState_,
                                       &currentState,
                                       mapPtr_);

    EstimationHelper::addImuPriorFactor(imuParametersVec_, &currentState, mapPtr_);

    EstimationHelper::addCameraParamPriorFactor(cameraNoiseParametersVec_, cameraRig_,
                                                &currentState, mapPtr_);

    ++numNFrames_;
    return;
  }

  addLandmarksFromDatabase(syncedData->databaseLandmarks);

  okvis::kinematics::Transformation T_WS;
  Eigen::Vector3d v_WS;
  okvis::ImuMeasurementDeque imuMeas = inertialMeasForStates_.find(
      statesMap_.rbegin()->second.timestamp, stateTime, true);
  EstimationHelper::predictNavState(imuMeas, statesMap_.rbegin()->second,
                                    mapPtr_, *imuParametersVec_.at(0),
                                    stateTime, &T_WS, &v_WS);
  correctPoseByCameraMotion(syncedData->relPoseToPriorFrames, &T_WS, &v_WS);

  EstimationHelper::setImuWindow(inertialMeasForStates_, &statesMap_.rbegin()->second, half_window_);

  statesMap_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(syncedData->currentNFrame->id()),
      std::forward_as_tuple(syncedData->currentNFrame->id(), stateTime,
                            syncedData->currentNFrame->isKeyframe()));

  uint64_t currentStateId = statesMap_.rbegin()->first;
  States &currentState = statesMap_.rbegin()->second;
  const States &previousState = (++statesMap_.rbegin())->second;
  EstimationHelper::setImuWindow(inertialMeasForStates_, &currentState, half_window_);

  EstimationHelper::addNavStateVariables(
      T_WS, v_WS, &currentState, mapPtr_, false);

  EstimationHelper::usePreviousGravity(previousState, &currentState);

  if (estimatorOptions_.constantBias) {
    EstimationHelper::usePreviousImuBias(previousState, &currentState);
  } else {
    EstimationHelper::addImuBiasParameters(imuRig_, &currentState, mapPtr_);
  }

  EstimationHelper::usePreviousImuAugmentedParameters(previousState, &currentState);

  EstimationHelper::usePreviousCameraParameters(previousState, &currentState);

  size_t imuId = 0u;
  if (estimatorOptions_.constantBias) {
    EstimationHelper::addImuObservationsConstBias(
        imuMeas, imuId, imuRig_.getModelId(imuId), *imuParametersVec_.at(imuId),
        previousState, currentState, mapPtr_);
  } else {
    EstimationHelper::addImuObservations(
        imuMeas, imuId, imuRig_.getModelId(imuId), *imuParametersVec_.at(imuId),
        previousState, currentState, mapPtr_);
  }

  EstimationHelper::addCameraParameterConstraints(
      cameraNoiseParametersVec_, previousState,
      currentState, mapPtr_);

  Eigen::AlignedVector<PointLandmark> newLandmarkList;
  okvis::TimerSwitchable triangulateTimer("3.1.1.1 triangulateMapPoint", true);
  triangulateTimer.start();
  for (PointMap::iterator pit = landmarksMap_.begin(); pit != landmarksMap_.end(); ++pit) {
    if (pit->second.inState()) {
      continue;
    }
    Eigen::AlignedVector<Eigen::Vector2d> obsList;
    Eigen::AlignedVector<Eigen::Vector2d> obsStdList;
    swift_vio::PointLandmark landmark(okvis::IdProvider::instance().newId(),
                                      InverseDepthParameterization::kModelId,
                                      landmarkParamizationPtr_.get());
    size_t minCulledFramesPlaceHolder = 2;
    std::shared_ptr<swift_vio::PointSharedData> pointDataPtr(
        new swift_vio::PointSharedData());
    triangulateTimer.start();
    TriangulationStatus status = EstimationHelper::triangulateMapPointRs(
        pit->second, statesMap_, mapPtr_, nframeToLandmarkIds_, cameraRig_,
        imuParametersVec_.at(0), &landmark, &obsList, &obsStdList,
        pointDataPtr.get(), nullptr,
        pointLandmarkOptions_.minTrackLengthForMsckf,
        minCulledFramesPlaceHolder, false, true);
    triangulateTimer.stop();
    if (status.triangulationOk) {
      pit->second.setInitialized(true);
      pit->second.setInState(true);

      Eigen::Vector4d lmkParams =
          Eigen::Map<const Eigen::Vector4d>(landmark.data());
      if (pointDataPtr->anchorIds().size()) {
        pit->second.anchorStateId = pointDataPtr->anchorIds()[0].frameId_;
        pit->second.anchorCameraId = pointDataPtr->anchorIds()[0].cameraIndex_;
        pit->second.pointHomog =
            EstimationHelper::anchoredInverseDepthToWorldCoordinates(
                mapPtr_, cameraRig_, lmkParams,
                pit->second.anchorCamera());
      } else {
        pit->second.pointHomog = lmkParams;
      }

      // add landmark variable to map
      std::shared_ptr<okvis::ceres::InverseDepthPointBlock>
          pointParameterBlock(new okvis::ceres::InverseDepthPointBlock(
              lmkParams, pit->first));
      if (!mapPtr_->addParameterBlock(pointParameterBlock,
                                      okvis::ceres::Map::InverseDepthPoint)) {
        LOG(WARNING) << "Failed to add block for landmark " << pit->first;
      }
    }
  }
  triangulateTimer.stop();

  EstimationHelper::addCameraObservations(statesMap_, nframeToLandmarkIds_, cameraRig_,
                                          &landmarksMap_, mapPtr_, imuParametersVec_.at(0),
                                          cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : NULL);

  EstimationHelper::configureSolver(mapPtr_, estimatorOptions_.max_iterations, 2, false);

  okvis::kinematics::Transformation T_W0B0;
  EstimationHelper::get_T_WB(mapPtr_, currentStateId, T_W0B0);

  mapPtr_->solve();

  EstimationHelper::transferWorldFrame(statesMap_, T_W0B0, currentStateId, mapPtr_, &landmarksMap_);

  EstimationHelper::updateLandmarksFromMap(mapPtr_, cameraRig_, &landmarksMap_);
  EstimationHelper::updateCameraRig(currentState, mapPtr_, cameraNoiseParametersVec_, &cameraRig_);
  EstimationHelper::updateImuRig(currentState, mapPtr_, &imuRig_);

  applyMarginalizationStrategy(marginalizedLandmarks_);
  ++numNFrames_;
  if ((int)numKeyframesInState() >= estimatorOptions_.numKeyframesForInit) {
    initStatus_ = InitializationStatus::RunningNonlinEst;
  }
}

bool SlidingWindowSmoother::applyMarginalizationStrategy(
    MapPointVector &removedLandmarks) {
  // Assumption: Landmarks are anchored in keyframes.
  removedLandmarks.clear();

  size_t numKeyframes = estimatorOptions_.numKeyframes;
  size_t numImuFrames = estimatorOptions_.numImuFrames;
  // the ordinary frames to remove, does not include removeKeyframes.
  std::vector<uint64_t> removeFrames;
  std::vector<uint64_t> removeKeyframes; // the keyframes to remove.
  // frames and keyframes in the IMU window.
  std::vector<uint64_t> nonlinearFrames;
  nonlinearFrames.reserve(numImuFrames);

  size_t countedKeyframes = 0;
  // iterate the statesMap from its back, keep numImuFrames in the IMU window,
  // Those ordinary frames outside the IMU window will be removed.
  // If the number of keyframes outside the IMU window is more than
  // numKeyframes, among them, choose the keyframe most distant from the current
  // frame to remove.
  // Different from okvis::Estimator, the velocity and biases
  // are marginalized together with pose, because velocity and biases are also
  // used in RsReprojectionFactors.

  std::map<uint64_t, States>::reverse_iterator rit = statesMap_.rbegin();
  for (size_t k = 0; k < numImuFrames; k++) {
    nonlinearFrames.push_back(rit->first);
    rit++;
    if (rit == statesMap_.rend()) {
      return true;
    }
  }

  // all frames and keyframes outside the IMU window.
  std::vector<uint64_t> allLinearizedFrames;
  allLinearizedFrames.reserve(statesMap_.size() - numImuFrames);
  while (rit != statesMap_.rend()) {
    if (!rit->second.isKeyframe) {
      removeFrames.push_back(rit->second.id);
    } else {
      countedKeyframes++;
    }
    allLinearizedFrames.push_back(rit->second.id);
    ++rit; // check the next frame
  }

  if (countedKeyframes >= numKeyframes) {
    removeKeyframes.resize(1);
    removeKeyframes[0] = EstimationHelper::findDistantKeyframe(
        statesMap_, mapPtr_, numImuFrames);
  }

  size_t numAllFramesToRemove = removeKeyframes.size() + removeFrames.size();
  if (numAllFramesToRemove == 0) {
    return true;
  }
  CHECK_EQ(numAllFramesToRemove, 1u)
      << "We expect to remove a frame or a keyframe in a marginalization step!";

  // remove linear marginalizationError, if existing.
  // Note that the marg factor info is intact in marginalizationErrorPtr_.
  if (marginalizationErrorPtr_ && marginalizationResidualId_) {
    bool success = mapPtr_->removeResidualBlock(marginalizationResidualId_);
    OKVIS_ASSERT_TRUE_DBG(Exception, success,
                          "could not remove marginalization error");
    marginalizationResidualId_ = 0;
    if (!success)
      return false;
  }

  // these will keep track of what we want to marginalize out.
  std::vector<uint64_t> paremeterBlocksToBeMarginalized;
  std::vector<bool> keepParameterBlocks;

  if (!marginalizationErrorPtr_) {
    marginalizationErrorPtr_.reset(
        new okvis::ceres::MarginalizationError(*mapPtr_.get()));
  }

  // for all removeFrames {
  // (1) mark the pose, velocity; mark the IMU factors;
  // (2) if the IMU biases is not random constant, mark the biases;
  // (3) if the camera extrinsic block is not random constant, mark the camera
  // extrinsic block; mark the extrinsic relative constraint
  // }
  // (4) drop all camera observations in removedFrames.
  for (uint64_t frameId : removeFrames) {
    VLOG(1) << "Marginalize frame " << frameId << ".";
    std::map<uint64_t, States>::iterator it = statesMap_.find(frameId);
    paremeterBlocksToBeMarginalized.push_back(
        it->second.global[GlobalStates::T_WS].id);
    keepParameterBlocks.push_back(false);
    it->second.global[GlobalStates::T_WS].exists = false;

    uint64_t vblockId = it->second.global[GlobalStates::v_WS].id;
    paremeterBlocksToBeMarginalized.push_back(vblockId);
    keepParameterBlocks.push_back(false);
    it->second.global[GlobalStates::v_WS].exists = false;

    okvis::ceres::Map::ResidualBlockCollection residuals =
        mapPtr_->residuals(vblockId);
    for (size_t r = 0; r < residuals.size(); ++r) {
      if (residuals[r].errorInterfacePtr->typeInfo().find("ImuError") !=
          std::string::npos) {
        marginalizationErrorPtr_->addResidualBlock(
            residuals[r].residualBlockId);
      }
    }

    for (size_t j = 0; j < it->second.sensors[SensorStates::Imu].size(); ++j) {
      uint64_t blockId = EstimationHelper::shouldMarginalizeSensorParamBlock(
          SensorStates::Imu, j, ImuSensorStates::Bias, it, mapPtr_);
      if (blockId) {
        paremeterBlocksToBeMarginalized.push_back(blockId);
        keepParameterBlocks.push_back(false);
        it->second.sensors[SensorStates::Imu][j][ImuSensorStates::Bias].exists =
            false;
      }
    }

    for (size_t j = 0; j < it->second.sensors[SensorStates::Camera].size();
         ++j) {
      uint64_t blockId = EstimationHelper::shouldMarginalizeSensorParamBlock(
          SensorStates::Camera, j, CameraSensorStates::T_XCi, it, mapPtr_);
      if (blockId) {
        paremeterBlocksToBeMarginalized.push_back(blockId);
        keepParameterBlocks.push_back(false);
        it->second.sensors[SensorStates::Camera][j][CameraSensorStates::T_XCi]
            .exists = false;
        okvis::ceres::Map::ResidualBlockCollection residuals =
            mapPtr_->residuals(blockId);
        for (size_t r = 0; r < residuals.size(); ++r) {
          if (residuals[r].errorInterfacePtr->typeInfo() ==
              "RelativePoseError") {
            marginalizationErrorPtr_->addResidualBlock(
                residuals[r].residualBlockId);
          }
        }
      }
    }
  } // every removed frame

  for (PointMap::iterator pit = landmarksMap_.begin();
       pit != landmarksMap_.end();) {
    CHECK(!vectorContains(removeFrames, pit->second.anchorStateId))
        << "A landmark is anchored in an ordinary frame!";
    // erase observations from the removed frames of landmarks.
    EstimationHelper::removeObservationInFrames(
        removeFrames, &pit->second.observations, mapPtr_, nullptr);
    // Unmarking observations in landmark Ids is unnecessary because
    // the nframe and landmark Ids will be removed altogether later on.
    if (pit->second.observations.size() == 0u) {
      mapPtr_->removeParameterBlock(pit->first);
      removedLandmarks.emplace_back(pit->second);
      pit = landmarksMap_.erase(pit);
    } else {
      ++pit;
    }
  } // every mappoint

  // for all removedKeyframes {
  // (1) mark the pose, velocity, mark the prior pose factor, prior velocity
  // factor, IMU factor;
  // (2) if the IMU biases is not random constant, mark the
  // biases, mark the bias prior;
  // (3) if the camera extrinsic block is not random constant,
  // mark the camera extrinsic block, mark the extrinsic prior
  // factor and relative constraint.
  // }
  for (auto keyframeId : removeKeyframes) {
    VLOG(1) << "Marginalize keyframe " << keyframeId << ".";
    std::map<uint64_t, States>::iterator it = statesMap_.find(keyframeId);
    uint64_t pblockId = it->second.global[GlobalStates::T_WS].id;
    paremeterBlocksToBeMarginalized.push_back(pblockId);
    keepParameterBlocks.push_back(false);
    it->second.global[GlobalStates::T_WS].exists = false;

    okvis::ceres::Map::ResidualBlockCollection presiduals =
        mapPtr_->residuals(pblockId);
    for (size_t r = 0; r < presiduals.size(); ++r) {
      if (presiduals[r].errorInterfacePtr->typeInfo() == "PoseError") {
        marginalizationErrorPtr_->addResidualBlock(
            presiduals[r].residualBlockId);
      }
    }

    uint64_t vblockId = it->second.global[GlobalStates::v_WS].id;
    paremeterBlocksToBeMarginalized.push_back(vblockId);
    keepParameterBlocks.push_back(false);
    it->second.global[GlobalStates::v_WS].exists = false;
    okvis::ceres::Map::ResidualBlockCollection vresiduals =
        mapPtr_->residuals(vblockId);
    for (size_t r = 0; r < vresiduals.size(); ++r) {
      if (vresiduals[r].errorInterfacePtr->typeInfo().find("ImuError") !=
              std::string::npos ||
          vresiduals[r].errorInterfacePtr->typeInfo() ==
              "EuclideanParamErrorSized") {
        marginalizationErrorPtr_->addResidualBlock(
            vresiduals[r].residualBlockId);
      }
    }

    for (size_t j = 0; j < it->second.sensors[SensorStates::Imu].size(); ++j) {
      uint64_t blockId = EstimationHelper::shouldMarginalizeSensorParamBlock(
          SensorStates::Imu, j, ImuSensorStates::Bias, it, mapPtr_);
      if (blockId) {
        paremeterBlocksToBeMarginalized.push_back(blockId);
        keepParameterBlocks.push_back(false);
        it->second.sensors[SensorStates::Imu][j][ImuSensorStates::Bias].exists =
            false;

        okvis::ceres::Map::ResidualBlockCollection residuals =
            mapPtr_->residuals(blockId);
        for (size_t r = 0; r < residuals.size(); ++r) {
          if (residuals[r].errorInterfacePtr->typeInfo() ==
              "EuclideanParamErrorSized") {
            marginalizationErrorPtr_->addResidualBlock(
                residuals[r].residualBlockId);
          }
        }
      }
    }

    for (size_t j = 0; j < it->second.sensors[SensorStates::Camera].size();
         ++j) {
      uint64_t blockId = EstimationHelper::shouldMarginalizeSensorParamBlock(
          SensorStates::Camera, j, CameraSensorStates::T_XCi, it, mapPtr_);
      if (blockId) {
        paremeterBlocksToBeMarginalized.push_back(blockId);
        keepParameterBlocks.push_back(false);
        it->second.sensors[SensorStates::Camera][j][CameraSensorStates::T_XCi]
            .exists = false;
        okvis::ceres::Map::ResidualBlockCollection residuals =
            mapPtr_->residuals(blockId);
        for (size_t r = 0; r < residuals.size(); ++r) {
          if (residuals[r].errorInterfacePtr->typeInfo() ==
                  "RelativePoseError" ||
              residuals[r].errorInterfacePtr->typeInfo() == "PoseError") {
            marginalizationErrorPtr_->addResidualBlock(
                residuals[r].residualBlockId);
          }
        }
      }
    }
  } // every keyframe

  // (4) mark the landmarks anchored in any removed keyframe,
  // mark observations of those landmarks in the linearized frames,
  // drop their observations in the IMU window,
  // For other landmarks not anchored in the keyframes to remove,
  // drop their observations from those keyframes.
  for (PointMap::iterator pit = landmarksMap_.begin();
       pit != landmarksMap_.end();) {
    if (vectorContains(removeKeyframes, pit->second.anchorStateId)) {
      // Find landmarks hosted in the removed frames,
      // (this implies that they had been triangulated successfully),
      // erase their observation in the latest frames,
      // then remember to marginalize these landmarks and their observations.
      EstimationHelper::removeObservationInFrames(
          nonlinearFrames, &pit->second.observations, mapPtr_,
          &nframeToLandmarkIds_);

      if (pit->second.observations.size() < 2u) {
        // Unmarking observations in landmark Ids is generally unnecessary.
        //            for (auto oit : pit->second.observations) {
        //              auto mfp = nframeToLandmarkIds_.find(oit.first.frameId);
        //              mfp->second.setLandmarkId(oit.first.cameraIndex,
        //                                        oit.first.keypointIndex, 0);
        //            }
        mapPtr_->removeParameterBlock(pit->first);
        removedLandmarks.emplace_back(pit->second);
        pit = landmarksMap_.erase(pit);
        continue;
      }

      for (auto oit = pit->second.observations.begin();
           oit != pit->second.observations.end(); ++oit) {
        CHECK(oit->second.residualId)
            << "An observation has no residual block?";
        CHECK(statesMap_.find(oit->first.frameId) != statesMap_.end())
            << "The nav state for an observation has been removed before?";
        marginalizationErrorPtr_->addResidualBlock(
            reinterpret_cast<::ceres::ResidualBlockId>(oit->second.residualId),
            false);
        // Unmarking observations in landmark Ids is unnecessary because
        // the nframe and landmark Ids will be removed altogether later on.
      }

      paremeterBlocksToBeMarginalized.push_back(pit->first);
      keepParameterBlocks.push_back(false);
      removedLandmarks.emplace_back(pit->second);
      pit = landmarksMap_.erase(pit);
    } else {
      // Otherwise, erase observations from the removed frames of
      // landmarks not hosted in the removed frames.
      EstimationHelper::removeObservationInFrames(
          removeKeyframes, &pit->second.observations, mapPtr_, nullptr);
      // Unmarking observations in landmark Ids is unnecessary because
      // the nframe and landmark Ids will be removed altogether later on.

      if (pit->second.observations.size() == 0u) {
        mapPtr_->removeParameterBlock(pit->first);
        removedLandmarks.emplace_back(pit->second);
        pit = landmarksMap_.erase(pit);
      } else {
        ++pit;
      }
    }
  } // every mappoint

  // remove states
  // slightly abuse removeFrames for indiscriminate state removal.
  removeFrames.insert(removeFrames.end(), removeKeyframes.begin(), removeKeyframes.end());
  for (auto it = removeFrames.begin(); it != removeFrames.end(); ++it) {
    auto sit = statesMap_.find(*it);
    CHECK(sit != statesMap_.end())
        << "The redundant frame has been removed before?";
    nframeToLandmarkIds_.erase(*it);
    statesMap_.erase(sit);
  }

  inertialMeasForStates_.pop_front(statesMap_.begin()->second.timestamp - half_window_);

  // now apply the actual marginalization
  if (paremeterBlocksToBeMarginalized.size() > 0) {
    std::vector<::ceres::ResidualBlockId> addedPriors;
    marginalizationErrorPtr_->marginalizeOut(paremeterBlocksToBeMarginalized,
                                             keepParameterBlocks);
  }

  // update error computation
  if (paremeterBlocksToBeMarginalized.size() > 0) {
    marginalizationErrorPtr_->updateErrorComputation();
  }

  // add the marginalization term again
  if (marginalizationErrorPtr_->num_residuals() == 0) {
    marginalizationErrorPtr_.reset();
  }
  if (marginalizationErrorPtr_) {
    std::vector<std::shared_ptr<okvis::ceres::ParameterBlock>>
        parameterBlockPtrs;
    marginalizationErrorPtr_->getParameterBlockPtrs(parameterBlockPtrs);
    marginalizationResidualId_ = mapPtr_->addResidualBlock(
        marginalizationErrorPtr_, NULL, parameterBlockPtrs);
    OKVIS_ASSERT_TRUE_DBG(Exception, marginalizationResidualId_,
                          "could not add marginalization error");
    if (!marginalizationResidualId_)
      return false;
  }

  return true;
}

std::vector<std::string> SlidingWindowSmoother::perturbationLabels() const {
  std::vector<std::string> minVarList{
      "p_WB_W_x(m)",   "p_WB_W_y(m)",
      "p_WB_W_z(m)",    "theta_WB_x",        "theta_WB_y",        "theta_WB_z",
      "v_WB_W_x(m/s)", "v_WB_W_y(m/s)", "v_WB_W_z(m/s)",
      "b_g_x(rad/s)",   "b_g_y(rad/s)",  "b_g_z(rad/s)",  "b_a_x(m/s^2)",
      "b_a_y(m/s^2)",   "b_a_z(m/s^2)"};
  return minVarList;
}

bool SlidingWindowSmoother::getStateStd(
    Eigen::Matrix<double, Eigen::Dynamic, 1>* stateStd) const {
  // skip computing covariance in processing real world data.
  *stateStd = Eigen::MatrixXd::Constant(15, 1, 1.0);
  return true;
}

bool SlidingWindowSmoother::computeCovariance(Eigen::MatrixXd* cov) const {
//  auto start1 = std::chrono::high_resolution_clock::now();
//  bool resQR = computeCovarianceCeres(cov, ::ceres::CovarianceAlgorithmType::SPARSE_QR);
//  auto stop1 = std::chrono::high_resolution_clock::now();
//  auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1);

//  Eigen::MatrixXd covSVD;
//  auto start2 = std::chrono::high_resolution_clock::now();
  bool resSVD = computeCovarianceCeres(cov, ::ceres::CovarianceAlgorithmType::DENSE_SVD);
//  auto stop2 = std::chrono::high_resolution_clock::now();
//  auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start2);
//  if (resQR && resSVD) {
//    LOG(INFO) << "ceres cov QR takes " << duration1.count() << " us vs SVD "
//              << duration2.count() << " us.QR cov\n"
//              << *cov << "\nSVD cov\n"
//              << covSVD;
//  }
  return resSVD;
}

bool SlidingWindowSmoother::computeCovarianceSchur(Eigen::MatrixXd* cov) const {
  uint64_t poseId = statesMap_.rbegin()->second.id;
  uint64_t speedId = statesMap_.rbegin()->second.global.at(GlobalStates::v_WS).id;
  uint64_t biasId = statesMap_.rbegin()
                                ->second.sensors.at(SensorStates::Imu)
                                .at(0)
                                .at(ImuSensorStates::Bias)
                                .id;
  bool res = mapPtr_->computeNavStateCovariance(poseId, {speedId, biasId},
                                            marginalizationResidualId_, cov);
  return res;
}

bool SlidingWindowSmoother::computeCovarianceCeres(
    Eigen::MatrixXd *cov, ::ceres::CovarianceAlgorithmType covAlgorithm) const {
  uint64_t poseId = statesMap_.rbegin()->second.id;
  uint64_t speedId = statesMap_.rbegin()->second.global.at(GlobalStates::v_WS).id;
  uint64_t biasId = statesMap_.rbegin()
                                ->second.sensors.at(SensorStates::Imu)
                                .at(0)
                                .at(ImuSensorStates::Bias)
                                .id;
  std::vector<
      Eigen::Matrix<double, -1, -1, Eigen::RowMajor>,
      Eigen::aligned_allocator<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>>
      covarianceBlockList;
  bool status = mapPtr_->getParameterBlockMinimalCovariance(
      {poseId, speedId, biasId}, mapPtr_->problemUnsafe(), &covarianceBlockList,
      covAlgorithm);
  if (status) {
    swift_vio::upperTriangularBlocksToSymmMatrix(covarianceBlockList, cov);
  } else {
    LOG(INFO)
        << "The ceres::Covariance with SPARSE_QR often raises rank deficient "
           "Jacobian exception because there are low-disparity landmarks.";
  }
  return status;
}

bool SlidingWindowSmoother::computeFullCovarianceCeres(
    Eigen::MatrixXd *cov, std::vector<uint64_t> *varIdList,
    ::ceres::CovarianceAlgorithmType covAlgorithm) const {
  varIdList->clear();
  varIdList->reserve(20);
  const States &currentState = statesMap_.rbegin()->second;

  for (const SpecificSensorStatesContainer &sensor :
       currentState.sensors.at(SensorStates::Imu)) {
    for (const auto &varinfo : sensor) {
      if (varinfo.exists) {
        varIdList->emplace_back(varinfo.id);
      }
    }
  }

  const auto &varinfo = currentState.global.at(GlobalStates::GravityDirection);
  if (varinfo.exists && imuParametersVec_.at(0)->isGravityDirectionVariable()) {
    varIdList->emplace_back(varinfo.id);
  }

  int i = 0;
  for (const SpecificSensorStatesContainer &sensor :
       currentState.sensors.at(SensorStates::Camera)) {
    const auto &noiseParam = cameraNoiseParametersVec_.at(i);
    if (sensor.at(CameraSensorStates::T_XCi).exists &&
        !noiseParam.isExtrinsicsFixed()) {
      varIdList->emplace_back(sensor.at(CameraSensorStates::T_XCi).id);
    }
    if (sensor.at(CameraSensorStates::Intrinsics).exists &&
        !noiseParam.isIntrinsicsFixed()) {
      varIdList->emplace_back(sensor.at(CameraSensorStates::Intrinsics).id);
    }
    if (sensor.at(CameraSensorStates::TD).exists &&
        !noiseParam.isTimeDelayFixed()) {
      varIdList->emplace_back(sensor.at(CameraSensorStates::TD).id);
    }
    if (sensor.at(CameraSensorStates::TR).exists &&
        !noiseParam.isReadoutTimeFixed()) {
      varIdList->emplace_back(sensor.at(CameraSensorStates::TR).id);
    }
    ++i;
  }

  for (const auto &state : statesMap_) {
    varIdList->emplace_back(state.second.global.at(GlobalStates::T_WS).id);
    varIdList->emplace_back(state.second.global.at(GlobalStates::v_WS).id);
    // The biases in the SlidingWindowSmoother is assumed random constant.
  }

  std::vector<
      Eigen::Matrix<double, -1, -1, Eigen::RowMajor>,
      Eigen::aligned_allocator<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>>
      covarianceBlockList;
  bool status = mapPtr_->getParameterBlockMinimalCovariance(
      *varIdList, mapPtr_->problemUnsafe(), &covarianceBlockList,
      covAlgorithm);
  if (status) {
    swift_vio::upperTriangularBlocksToSymmMatrix(covarianceBlockList, cov);
  }
  return status;
}

}  // namespace swift_vio
