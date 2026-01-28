#include "swift_vio/SlidingWindowFilter.h"
#include "swift_vio/EstimationHelper.h"
#include "swift_vio/FeatureTriangulation.hpp"
#include "swift_vio/FilterHelper.hpp"
#include "swift_vio/filterJacobians.h"
#include "swift_vio/IoUtil.hpp"
#include "swift_vio/TimingUtilities.h"
#include <swift_vio/EkfUpdater.h>

#include <swift_vio/ceres/NormalVectorParameterBlock.hpp>
#include <swift_vio/ceres/EuclideanParamBlock.hpp>
#include <swift_vio/SlidingWindowSmoother.h>

#include <okvis/IdProvider.hpp>

#include "vio/eigen_utils.h"


DEFINE_bool(use_RK4, false,
            "use 4th order runge-kutta or the trapezoidal "
            "rule for integrating IMU data and computing"
            " Jacobians");
DEFINE_bool(use_first_estimate, true,
            "use first estimates to compute covariance?");


namespace swift_vio {
SlidingWindowFilter::SlidingWindowFilter(const okvis::EstimatorOptions &options)
    : EstimatorBase(options),
      propagateAndCloneTimer("3.2.0 propagateAndClone", true),
      triangulateTimer("3.2.1.1 triangulateMapPoint", true),
      computeHTimer("3.2.1 featureJacobian", true),
      updateLandmarksTimer("3.2.5 updateLandmarks", true),
      optimizerCovarianceTimer("1.1 optimizerCovariance", true),
      zeroVelocityUpdateTimer("1.2 zeroVelocityUpdateAtStart", true),
      computeKalmanGainTimer("3.2.2 computeKalmanGain", true),
      updateStatesTimer("3.2.3 updateStates", true),
      updateCovarianceTimer("3.2.4 updateCovariance", true),
      trackLengthAccumulator(100, 0u) {}

SlidingWindowFilter::~SlidingWindowFilter() {
  LOG(INFO) << "Destructing SlidingWindowFilter.";
}

void SlidingWindowFilter::setPointLandmarkOptions(const swift_vio::PointLandmarkOptions& plOptions) {
  pointLandmarkOptions_ = plOptions;
  swift_vio::Feature::optimization_config.max_depth =
      plOptions.triangulationMaxDepth;
  landmarkParamizationPtr_ = createLandmarkLocalParameterization(pointLandmarkOptions_.landmarkModelId);
}

void SlidingWindowFilter::initializeLandmarksInFilter() {
  updateLandmarksTimer.start();
  Eigen::Matrix<double, Eigen::Dynamic, 1> r_i;
  Eigen::MatrixXd H_i;
  Eigen::MatrixXd R_i;
  Eigen::Matrix<double, Eigen::Dynamic, 3> H_fi;
  Eigen::MatrixXd Q2;  // nullspace of H_fi
  Eigen::MatrixXd Q1;  // column space of H_fi

  Eigen::Matrix<double, Eigen::Dynamic, 1> z_o;
  Eigen::MatrixXd H_o;
  Eigen::MatrixXd R_o;

  Eigen::AlignedVector<Eigen::VectorXd> vz_1;
  Eigen::AlignedVector<Eigen::VectorXd> vz_o;
  Eigen::AlignedVector<Eigen::Matrix<double, 3, -1>> vH_1;
  Eigen::AlignedVector<Eigen::Matrix3d> vH_2;
  Eigen::AlignedVector<Eigen::Matrix3d> vR_1;

  Eigen::AlignedVector<Eigen::MatrixXd> vH_o;  // Each entry has a size (2n-3)x(13+9m),
  // where n is the number of observations of a landmark,
  // m is the number of cloned nav states in the sliding window.
  Eigen::AlignedVector<Eigen::MatrixXd> vR_o;  // Each entry has a size (2n-3)x(2n-3).

  size_t totalObsDim = 0u;  // Total dimensions of all features' observations.
  const uint64_t currFrameId = currentFrameId();
  size_t navAndImuParamsDim = startIndexOfGravity();
  size_t numCamPosePointStates = cameraParamPoseAndLandmarkMinimalDim();
  const size_t numCamPoseStates = numCamPosePointStates - 3 * orderedLandmarks_.size();
  Eigen::MatrixXd variableCov = covariance_.block(
      navAndImuParamsDim, navAndImuParamsDim, numCamPoseStates,
      numCamPoseStates); // covariance of camera and pose copy states

  int capacity = pointLandmarkOptions_.maxInStateLandmarks - (int)(orderedLandmarks_.size());
  int numQualified = 0;
  std::vector<PointLandmark> landmarksToAdd;
  landmarksToAdd.reserve(16);
  int totalCand = 0;
  int numLeftover = 0, numJacFailed = 0, numGatingFailed = 0, numNotLast = 0;
  for (PointMap::iterator pit = landmarksMap_.begin();
       pit != landmarksMap_.end(); ++pit) {
    if (pit->second.measurementType() == FeatureTrackStatus::kSlamInitialization) {
      ++totalCand;
      if (capacity <= numQualified) {
        pit->second.setMeasurementFate(FeatureTrackStatus::kLeftOver);
        ++numLeftover;
        continue;
      }
      swift_vio::PointLandmark pointLandmark(
          pit->first, pointLandmarkOptions_.landmarkModelId,
          landmarkParamizationPtr_.get());
      std::shared_ptr<swift_vio::PointSharedData> pointSharedData(new swift_vio::PointSharedData());
      computeHTimer.start();
      FeatureJacobianStatus jacStatus = FilterJacobians::featureJacobian(
          pit->second, statesMap_, mapPtr_, nframeToLandmarkIds_, cameraRig_,
          cameraNoiseParametersVec_,
          imuParametersVec_.at(0), estimatorOptions_, &pointLandmark,
          pointSharedData.get(), H_i, r_i, R_i, &H_fi, nullptr,
          pointLandmarkOptions_.minTrackLengthForMsckf, estimatorOptions_.minMarginalizedFrames);
      computeHTimer.stop();
      if (!jacStatus.successful || jacStatus.triangulationStatus.raysParallel) {
        // We do not initialize points at infinity to the state vector which causes numerical problems.
        pit->second.setMeasurementFate(
            FeatureTrackStatus::kComputingJacobiansFailed);
        ++numJacFailed;
        continue;
      }

      // Since we avoid landmarks of low disparity, from now on we can assume that H_fi
      // has full column rank. That is, the landmark is well observed.
      vio::leftNullspaceAndColumnSpace(H_fi, &Q2, &Q1);
      z_o = Q2.transpose() * r_i;
      H_o = Q2.transpose() * H_i;
      R_o = Q2.transpose() * R_i * Q2;

      if (!FilterHelper::gatingTest(H_o, z_o, R_o, variableCov, estimatorOptions_.useMahalanobisGating)) {
        pit->second.setMeasurementFate(FeatureTrackStatus::kPotentialOutlier);
        ++numGatingFailed;
        continue;
      }

      if (pointLandmark.modelId() ==
          swift_vio::InverseDepthParameterization::kModelId) {
        if ( currFrameId != pointSharedData->anchorIds()[0].frameId_) {
          // Skip the landmark because we assume the Jacobians are computed by
          // using the current frame as the anchor frame.
          // The underlying reason may be that the landmark has failed to unproject in the current NFrame.
          pit->second.setMeasurementFate(FeatureTrackStatus::kLeftOver);
          ++numNotLast;
          continue;
        }
        pit->second.anchorStateId = pointSharedData->anchorIds()[0].frameId_;
        pit->second.anchorCameraId = pointSharedData->anchorIds()[0].cameraIndex_;
        // update pointHomog for visualization
        pit->second.pointHomog = EstimationHelper::anchoredInverseDepthToWorldCoordinates(
              mapPtr_, cameraRig_, pointLandmark.estimate(), pit->second.anchorCamera());
      } else {
        pit->second.pointHomog = pointLandmark.estimate();
      }

      pit->second.setInState(true);
      pit->second.setInitialized(true);
      pit->second.setMeasurementFate(FeatureTrackStatus::kSuccessful);
      ++numQualified;
      landmarksToAdd.emplace_back(pointLandmark);

      slamLandmarks_.emplace_back(pit->second);

      vz_1.push_back(Q1.transpose() * r_i);
      vz_o.push_back(z_o);
      vH_1.push_back(Q1.transpose() * H_i);
      vH_2.push_back(Q1.transpose() * H_fi);
      vH_o.push_back(H_o);
      vR_o.push_back(R_o);
      vR_1.push_back(Q1.transpose() * R_i * Q1);
      totalObsDim += H_i.rows();
    }
  }
  VLOG(1) << "Candidate tracks " << totalCand << ", #left over " << numLeftover
            << ", #jac failed " << numJacFailed << ", #not in last frame " << numNotLast
            << ", #gating failed " << numGatingFailed;
  // augment and update the covariance matrix.
  size_t nNewFeatures = landmarksToAdd.size();
  VLOG(1) << "Initializing " << nNewFeatures << " landmarks into the state vector of "
            << orderedLandmarks_.size() << " landmarks!";
  if (nNewFeatures) {
    Eigen::MatrixXd H_o(totalObsDim - 3 * nNewFeatures, numCamPoseStates);
    Eigen::MatrixXd H_1(3 * nNewFeatures, numCamPoseStates);
    Eigen::MatrixXd invH_2 =
        Eigen::MatrixXd::Zero(3 * nNewFeatures, 3 * nNewFeatures);
    Eigen::MatrixXd R_o = Eigen::MatrixXd::Zero(
        totalObsDim - 3 * nNewFeatures, totalObsDim - 3 * nNewFeatures);
    Eigen::MatrixXd R_1 =
        Eigen::MatrixXd::Zero(3 * nNewFeatures, 3 * nNewFeatures);
    Eigen::Matrix<double, Eigen::Dynamic, 1> z_1(nNewFeatures * 3, 1);
    Eigen::Matrix<double, Eigen::Dynamic, 1> z_o(
        totalObsDim - nNewFeatures * 3, 1);

    size_t startRow = 0u;
    for (size_t featureIndex = 0u; featureIndex < nNewFeatures; ++featureIndex) {
      H_o.block(startRow, 0, vH_o[featureIndex].rows(), numCamPoseStates) =
          vH_o[featureIndex];
      H_1.block(3 * featureIndex, 0, 3, numCamPoseStates) = vH_1[featureIndex];
      invH_2.block<3, 3>(3 * featureIndex, 3 * featureIndex) =
          vH_2[featureIndex].inverse();
      R_o.block(startRow, startRow, vH_o[featureIndex].rows(),
                vH_o[featureIndex].rows()) = vR_o[featureIndex];
      R_1.block<3, 3>(3 * featureIndex, 3 * featureIndex) = vR_1[featureIndex];
      z_1.segment<3>(3 * featureIndex) = vz_1[featureIndex];
      z_o.segment(startRow, vH_o[featureIndex].rows()) = vz_o[featureIndex];
      startRow += vH_o[featureIndex].rows();
    }

    // initialize features into the state vector with z_1, R_1, H_1, H_2.
    // also correct the landmark parameters.
    updateCovarianceTimer.start();
    int covDim = covariance_.rows();
    Eigen::MatrixXd Paug(covDim + nNewFeatures * 3,
                         covDim + nNewFeatures * 3);
    Paug.topLeftCorner(covDim, covDim) = covariance_;
    Eigen::MatrixXd invH2H1 = invH_2 * H_1;
    Paug.block(covDim, 0, 3 * nNewFeatures, covDim) =
        -invH2H1 * Paug.block(navAndImuParamsDim, 0,
                              numCamPoseStates, covDim);
    Paug.block(0, covDim, covDim, 3 * nNewFeatures) =
        Paug.block(covDim, 0, 3 * nNewFeatures, covDim).transpose();
    Paug.bottomRightCorner(3 * nNewFeatures, 3 * nNewFeatures) =
        -Paug.block(covDim, navAndImuParamsDim,
                    3 * nNewFeatures, numCamPoseStates) *
            invH2H1.transpose() +
        invH_2 * R_1 * invH_2.transpose();
    covariance_ = Paug;
    updateCovarianceTimer.stop();

    Eigen::Matrix<double, Eigen::Dynamic, 1> deltaLandmarks = invH_2 * z_1;
    for (size_t j = 0u; j < nNewFeatures; ++j) {
      landmarksToAdd[j].Plus(deltaLandmarks.data() + j * 3);
    }

    orderedLandmarks_.insert(orderedLandmarks_.end(), landmarksToAdd.begin(),
                       landmarksToAdd.end());

    // update the state vector with z_o, R_o, H_o.
    DefaultEkfUpdater updater(covariance_, navAndImuParamsDim, numCamPoseStates);
    computeKalmanGainTimer.start();
    Eigen::Matrix<double, Eigen::Dynamic, 1> deltaX =
        updater.computeCorrection(H_o, z_o, R_o);
    computeKalmanGainTimer.stop();
    updateStatesTimer.start();
    updateStates(deltaX);
    updateStatesTimer.stop();
    updateCovarianceTimer.start();
    updater.updateCovariance(&covariance_);
    updateCovarianceTimer.stop();

    if (pointLandmarkOptions_.landmarkModelId ==
            okvis::ceres::HomogeneousPointLocalParameterization::kModelId &&
        FLAGS_use_first_estimate) {
      for (size_t j = 0u; j < nNewFeatures; ++j) {
        landmarksToAdd[j].fixLinPoint();
      }
    }
  }
  updateLandmarksTimer.stop();
}

void SlidingWindowFilter::initializeFromState(const InitialNavState &initialState, ConstMultiFramePtr currentNFrame) {
  initialNavState_ = initialState;
  CHECK_GE(cameraRig_.numCameras(), 1) << "Add at least one camera before calling initializeFrom!";
  CHECK_GE(imuParametersVec_.size(), 1) << "Add at least one IMU before calling initializeFrom!";

  okvis::Time stateTime = TimingUtilities::computeStateTimestamp(cameraRig_, currentNFrame);

  auto res = nframeToLandmarkIds_.emplace(
        std::piecewise_construct, std::forward_as_tuple(currentNFrame->id()),
        std::forward_as_tuple(currentNFrame->timestamp(), currentNFrame->id(),
                              currentNFrame->numFrames()));
  for (size_t i = 0; i < currentNFrame->numFrames(); ++i) {
    res.first->second.setTimestamp(i, currentNFrame->timestamp(i));
    res.first->second.landmarkIdsMutable(i).resize(currentNFrame->numKeypoints(i), 0u);
  }

  // initialize first state
  statesMap_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(currentNFrame->id()),
      std::forward_as_tuple(currentNFrame->id(), stateTime,
                            currentNFrame->isKeyframe()));
  // add global nav state, gravity
  EstimationHelper::addNavStateVariables(
      initialNavState_.pose(), initialNavState_.v_WS,
      &statesMap_.rbegin()->second, mapPtr_, true);
  EstimationHelper::addGravityVariable(
        imuParametersVec_.at(0)->gravityDirection(),
        &statesMap_.rbegin()->second, mapPtr_,
        imuParametersVec_.at(0)->isGravityDirectionVariable());
  LOG(INFO) << "Gyro bias initialized to " << imuRig_.at(0).getGyroBias().transpose()
            << ", accel bias " << imuRig_.at(0).getAccelBias().transpose();
  EstimationHelper::addImuParameters(imuRig_, imuParametersVec_, &statesMap_.rbegin()->second, mapPtr_);
  EstimationHelper::addCameraParameters(cameraRig_, cameraNoiseParametersVec_, &statesMap_.rbegin()->second, mapPtr_);

  initializeCovByHeuristics();

  // check
  FilterHelper::saveCovariance(covariance_, FLAGS_log_dir);

  initStatus_ = swift_vio::InitializationStatus::RunningNonlinEst;
}

void SlidingWindowFilter::initializeFrom(std::shared_ptr<EstimatorBase> other,
                                         ConstMultiFramePtr currentNFrame) {
  EstimatorBase::initializeFrom(other, currentNFrame);
  if (other->typeInfo() == "ImuInitializer") {
    LOG(INFO) << "Initializing the SlidingWindowFilter from an ImuInitializer.";
    initializeFromState(initialNavState_, currentNFrame); // initial nav state was just copied from the initializer.
  } else if (other->typeInfo() == "SlidingWindowSmoother") {
    LOG(INFO) << "Initializing the SlidingWindowFilter from a VioInitializer.";
    std::shared_ptr<const VioInitializer> initializer =
        std::static_pointer_cast<VioInitializer>(other);
    bool status = initializeCovByFisherInfo(*initializer, true);

    if (!status) {
      LOG(WARNING) << "Resort to heuristics to compute nav state covariances!";
      initializeCovByHeuristics();
    }

    FilterHelper::saveCovariance(covariance_, FLAGS_log_dir); // check

    // remove residual blocks which are not needed by the filter.
    for (PointMap::iterator it = landmarksMap_.begin();
         it != landmarksMap_.end(); ++it) {
      for (auto observationIter = it->second.observations.rbegin();
           observationIter != it->second.observations.rend();
           ++observationIter) {
        if (observationIter->second.residualId) {
          mapPtr_->removeResidualBlock(
              reinterpret_cast<::ceres::ResidualBlockId>(
                  observationIter->second.residualId));
          observationIter->second.residualId = 0;
        }
      }
    }
    mapPtr_->removeAllResidualBlocks();

    // initialize landmarks into the landmarkVector and covariance
    uint64_t currFrameId = currentFrameId();
    for (PointMap::iterator it = landmarksMap_.begin();
         it != landmarksMap_.end(); ++it) {
      if (it->second.inState()) {
        int anchorCameraId = -1;
        for (auto observationIter = it->second.observations.rbegin();
             observationIter != it->second.observations.rend();
             ++observationIter) {
          if (observationIter->first.frameId == currFrameId) {
            anchorCameraId = observationIter->first.cameraIndex;
          } else {
            break;
          }
        }
        if (anchorCameraId >= 0) {
          // By default, the anchor frame for an AIDP point is the current
          // Nframe. So we make sure a landmark is observed in the anchor NFrame
          // before initializing it.
          it->second.setMeasurementType(
              FeatureTrackStatus::MeasurementType::kSlamInitialization);
        } else {
          // This landmark will not be added to the filter,
          // so we discard its initial position.
          mapPtr_->removeParameterBlock(it->second.id);
        }
        it->second.setInState(false);
      }
    }
    initializeLandmarksInFilter();
  }
}

bool SlidingWindowFilter::initializeCovByFisherInfo(const VioInitializer &initializer, bool trySvd) {
  std::vector<uint64_t> oldVarIdList;
  Eigen::MatrixXd oldCov;
  bool covStatus = initializer.computeFullCovarianceCeres(
      &oldCov, &oldVarIdList, ::ceres::CovarianceAlgorithmType::SPARSE_QR);
  if (!covStatus) {
    if (trySvd) {
      LOG(WARNING) << "Resort to dense SVD to compute nav state covariances!";
      covStatus = initializer.computeFullCovarianceCeres(
          &oldCov, &oldVarIdList, ::ceres::CovarianceAlgorithmType::DENSE_SVD);
    } else {
      return false;
    }
  }
  std::unordered_map<uint64_t, ErrorStateInfo> errorStateMap;
  FilterHelper::composeErrorStateMap(statesMap_, imuRig_,
                                     *imuParametersVec_.at(0), cameraRig_,
                                     cameraNoiseParametersVec_, &errorStateMap);

  FilterHelper::setOldErrorStateIndices(oldVarIdList, &errorStateMap,
                                        oldCov.rows());

  int clonedStateIdx = startIndexOfClonedStates();
  int covDim = clonedStateIdx + 9 * statesMap_.size();
  covariance_.resize(covDim, covDim);
  int area = 0;
  for (auto it = errorStateMap.begin(); it != errorStateMap.end(); ++it) {
    for (auto jt = errorStateMap.begin(); jt != errorStateMap.end(); ++jt) {
      const ErrorStateInfo &si = it->second;
      const ErrorStateInfo &sj = jt->second;
      covariance_.block(si.startIndexInCov, sj.startIndexInCov, si.minimalDim,
                        sj.minimalDim) =
          oldCov.block(si.oldStartIndexInCov, sj.oldStartIndexInCov,
                       si.minimalDim, sj.minimalDim);
      area += si.minimalDim * sj.minimalDim;
    }
  }

  // initialize off diagonal blocks for the first nav state
  covariance_.block(0, 0, 9, covDim) =
      covariance_.block(covDim - 9, 0, 9, covDim);
  covariance_.block(0, 0, covDim, 9) =
      covariance_.block(0, covDim - 9, covDim, 9);
  area += covDim * 9 * 2 - 9 * 9;

  CHECK_EQ(area, covDim * covDim)
      << "Inconsistent covariance from two Estimators!";

  FilterHelper::assignCovarianceIndices(errorStateMap, &statesMap_);
  return covStatus;
}

bool SlidingWindowFilter::initializeCovByHeuristics() {
  // initialize covariance for global states and sensor parameters
  int covDim = startIndexOfClonedStates();
  covariance_ = Eigen::MatrixXd::Zero(covDim, covDim);
  size_t gWStartIndex = startIndexOfGravity();
  FilterHelper::initializeNavStateCov(initialNavState_, *(imuParametersVec_.at(0)), &covariance_, gWStartIndex);
  FilterHelper::initializeImuStateCov(imuParametersVec_, imuRig_, &covariance_, &statesMap_.rbegin()->second, startIndexOfImuBias());
  FilterHelper::initializeCameraStateCov(cameraNoiseParametersVec_, cameraRig_, &covariance_, &statesMap_.rbegin()->second, startIndexOfCameraParams());

  // initialize cov for cloned nav state variables.
  FilterHelper::initializeClonedNavStateCov(&statesMap_, &covariance_, covDim, gWStartIndex);
  return true;
}

void SlidingWindowFilter::estimate(std::shared_ptr<const VisualMatcherOutput> syncedData) {
  okvis::Time stateTime = TimingUtilities::computeStateTimestamp(cameraRig_, syncedData->currentNFrame);

  inertialMeasForStates_.push_back(syncedData->imuMeasurements);

  EstimationHelper::updateFeatureTracks(syncedData->featureTracks, syncedData->currentNFrame, &landmarksMap_, &nframeToLandmarkIds_);

  addLandmarksFromDatabase(syncedData->databaseLandmarks);

  propagateAndCloneTimer.start();
  okvis::kinematics::Transformation T_WS;
  Eigen::Vector3d v_WS;

  EstimationHelper::propagateNavStateAndCov(
      syncedData->imuMeasurements, statesMap_.rbegin()->second, mapPtr_,
      *imuParametersVec_.at(0), stateTime, &T_WS, &v_WS, &covariance_);

  EstimationHelper::setImuWindow(inertialMeasForStates_, &statesMap_.rbegin()->second, half_window_);

  statesMap_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(syncedData->currentNFrame->id()),
      std::forward_as_tuple(syncedData->currentNFrame->id(), stateTime,
                            syncedData->currentNFrame->isKeyframe()));

  EstimationHelper::setImuWindow(inertialMeasForStates_, &statesMap_.rbegin()->second, half_window_);

  EstimationHelper::addNavStateVariables(
      T_WS, v_WS, &statesMap_.rbegin()->second, mapPtr_, FLAGS_use_first_estimate);

  EstimationHelper::usePreviousGravity((++statesMap_.rbegin())->second, &statesMap_.rbegin()->second);

  EstimationHelper::usePreviousImuBias((++statesMap_.rbegin())->second, &statesMap_.rbegin()->second);

  EstimationHelper::usePreviousImuAugmentedParameters((++statesMap_.rbegin())->second, &statesMap_.rbegin()->second);

  EstimationHelper::usePreviousCameraParameters((++statesMap_.rbegin())->second, &statesMap_.rbegin()->second);

  const size_t numPointStates = 3 * orderedLandmarks_.size();
  const size_t numOldNavImuCamPoseStates = covariance_.rows() - numPointStates;
  statesMap_.rbegin()->second.global.at(GlobalStates::T_WS).startIndexInCov =
      numOldNavImuCamPoseStates;
  statesMap_.rbegin()->second.global.at(GlobalStates::v_WS).startIndexInCov =
      numOldNavImuCamPoseStates + 6;
  FilterHelper::addCovForClonedNavState(&covariance_, numOldNavImuCamPoseStates);
  propagateAndCloneTimer.stop();

  updateWithCameraObservations();
}

void SlidingWindowFilter::updateWithCameraObservations() {
  // containers of Jacobians of measurements of marginalized features
  Eigen::AlignedVector<Eigen::VectorXd> vr_o;
  Eigen::AlignedVector<Eigen::MatrixXd> vH_o;  // each entry has a size (2n-3)x(13+9m)
  // where n is the number of observations of a landmark,
  // and m is the number of cloned nav states in the sliding window.
  Eigen::AlignedVector<Eigen::MatrixXd> vR_o;  // each entry has a size (2n-3)x(2n-3).

  // containers of Jacobians of measurements of points in the states
  Eigen::AlignedVector<Eigen::VectorXd> vr_i;
  Eigen::AlignedVector<Eigen::MatrixXd> vH_x;  // each entry has a size 2j x(2 + 16 + 9m), 2 for gravity, 16 for camera extrinsics, intrinsics, td, tr.
  Eigen::AlignedVector<Eigen::MatrixXd> vH_f;  // each entry has a size 2j x 3k where j is the number of observations, k is the number of landmarks in state.
  Eigen::AlignedVector<Eigen::MatrixXd> vR_i;

  OKVIS_ASSERT_EQ_DBG(
      Exception, (size_t)covariance_.rows(),
      startIndexOfClonedStatesFast() +
          kClonedStateMinimalDim * statesMap_.size() + 3 * orderedLandmarks_.size(),
      "Inconsistent rows of covariance matrix and number of states");

  int numCamPosePointStates = cameraParamPoseAndLandmarkMinimalDim();
  size_t dimH_o[2] = {0, numCamPosePointStates - 3 * orderedLandmarks_.size() - kClonedStateMinimalDim};
  int numMsckfLandmarks = 0;  // number of landmarks not in state and not tracked in current frame.
  size_t numSlamObservations = 0u;  // number of observations for landmarks in state and tracked now

  const uint64_t currFrameId = currentFrameId();
  size_t navAndImuParamsDim = startIndexOfGravity();
  // covariance block for camera and pose state copies except for the current pose state is used for MSCKF features.
  Eigen::MatrixXd variableCov = covariance_.block(
      navAndImuParamsDim, navAndImuParamsDim,
      dimH_o[1],
      dimH_o[1]);
  // covariance block for camera and pose state copies including the current pose state is used for SLAM features.
  Eigen::MatrixXd variableCov2 = covariance_.block(
      navAndImuParamsDim, navAndImuParamsDim,
      numCamPosePointStates, numCamPosePointStates);

  std::vector<std::pair<uint64_t, int>> msckfLandmarks;
  msckfLandmarks.reserve(pointLandmarkOptions_.maxMarginalizedLandmarks);
  std::vector<std::pair<uint64_t, int>> slamLandmarks;
  slamLandmarks.reserve(pointLandmarkOptions_.maxInStateLandmarks / 2);
  for (PointMap::iterator it = landmarksMap_.begin();
       it != landmarksMap_.end(); ++it) {
    it->second.updateStatus(currFrameId,
                            pointLandmarkOptions_.minTrackLengthForMsckf,
                            pointLandmarkOptions_.minTrackLengthForSlam);
    if (it->second.measurementType() == FeatureTrackStatus::kMsckfTrack) {
      msckfLandmarks.emplace_back(it->first, it->second.observations.size());
    } else if (it->second.measurementType() ==
               FeatureTrackStatus::kSlamObservation) {
      slamLandmarks.emplace_back(it->first, it->second.observations.size());
    }
  }

  std::sort(msckfLandmarks.begin(), msckfLandmarks.end(),
            [](const std::pair<uint64_t, int> &a,
               const std::pair<uint64_t, int> &b) -> bool {
              return a.second > b.second;
            }); // in the descending order of track length.

  std::sort(slamLandmarks.begin(), slamLandmarks.end(),
            [](const std::pair<uint64_t, int> &a,
               const std::pair<uint64_t, int> &b) -> bool {
              return a.second > b.second;
            });

  marginalizedLandmarks_.clear();
  for (auto &idAndLength : msckfLandmarks) {
    MapPoint &mapPoint = landmarksMap_.at(idAndLength.first);
    if (numMsckfLandmarks >= pointLandmarkOptions_.maxMarginalizedLandmarks) {
      mapPoint.setMeasurementFate(FeatureTrackStatus::kLeftOver);
      continue;
    }
    swift_vio::PointLandmark landmark(okvis::IdProvider::instance().newId(),
                                      pointLandmarkOptions_.landmarkModelId,
                                      landmarkParamizationPtr_.get());
    Eigen::MatrixXd H_oi;                          //(2n-3, dimH_o[1])
    Eigen::Matrix<double, Eigen::Dynamic, 1> r_oi; //(2n-3, 1)
    Eigen::MatrixXd R_oi;                          //(2n-3, 2n-3)
    std::shared_ptr<swift_vio::PointSharedData> pointSharedData(new swift_vio::PointSharedData());
    computeHTimer.start();
    FeatureJacobianStatus jacStatus = FilterJacobians::featureJacobian(
        mapPoint, statesMap_, mapPtr_, nframeToLandmarkIds_, cameraRig_,
          cameraNoiseParametersVec_,
        imuParametersVec_.at(0), estimatorOptions_, &landmark, pointSharedData.get(),
        H_oi, r_oi, R_oi, nullptr, nullptr,
        pointLandmarkOptions_.minTrackLengthForMsckf, estimatorOptions_.minMarginalizedFrames);
    computeHTimer.stop();
    if (!jacStatus.successful) {
      mapPoint.setMeasurementFate(FeatureTrackStatus::kComputingJacobiansFailed);
      continue;
    }
    if (!FilterHelper::gatingTest(H_oi, r_oi, R_oi, variableCov, estimatorOptions_.useMahalanobisGating)) {
      mapPoint.setMeasurementFate(FeatureTrackStatus::kPotentialOutlier);
      continue;
    }
    mapPoint.setMeasurementFate(FeatureTrackStatus::kSuccessful);
    // update point coordinates for visualization.
    if (landmark.modelId() ==
        swift_vio::InverseDepthParameterization::kModelId) {
      mapPoint.pointHomog =
          EstimationHelper::anchoredInverseDepthToWorldCoordinates(
              mapPtr_, cameraRig_, landmark.estimate(),
              pointSharedData->anchorCameraId(0));
    } else {
      mapPoint.pointHomog = landmark.estimate();
    }
    mapPoint.setInitialized(true);
    marginalizedLandmarks_.emplace_back(mapPoint);
    vr_o.push_back(r_oi);
    vR_o.push_back(R_oi);
    vH_o.push_back(H_oi);
    dimH_o[0] += r_oi.rows();
    ++numMsckfLandmarks;
  }

  slamLandmarks_.clear();
  for (auto &idAndLength : slamLandmarks) {
    MapPoint &mapPoint = landmarksMap_.at(idAndLength.first);
    // compute residual and Jacobian for an observed point in the state.
    Eigen::VectorXd r_i;
    Eigen::MatrixXd H_x;
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd R_i;

    Eigen::MatrixXd subH_f;

    // get the landmark parameters.
    uint64_t hpbid = mapPoint.id;
    auto idPos = std::find_if(
        orderedLandmarks_.begin(), orderedLandmarks_.end(),
        [hpbid](const PointLandmark &x) {
          return x.id() == hpbid;
        });
    OKVIS_ASSERT_TRUE(Exception, idPos != orderedLandmarks_.end(),
                      "The tracked landmark " << hpbid << " is not in the state vector.");
    size_t covPtId = std::distance(orderedLandmarks_.begin(), idPos);

    computeHTimer.start();
    bool isValidJacobian = FilterJacobians::slamFeatureJacobian(
        mapPoint, statesMap_, mapPtr_, nframeToLandmarkIds_, cameraRig_, cameraNoiseParametersVec_,
        imuParametersVec_.at(0), estimatorOptions_, currentFrameId(), *idPos,
        H_x, r_i, R_i, subH_f);
    computeHTimer.stop();

    if (!isValidJacobian) {
      mapPoint.setMeasurementFate(
          FeatureTrackStatus::kComputingJacobiansFailed);
      continue;
    }

    H_f.resize(subH_f.rows(), 3 * orderedLandmarks_.size());
    H_f.setZero();
    H_f.block(0, covPtId * 3, subH_f.rows(), subH_f.cols()) = subH_f;

    Eigen::MatrixXd H_xf(H_x.rows(), H_x.cols() + H_f.cols());
    H_xf.leftCols(H_x.cols()) = H_x;
    H_xf.rightCols(H_f.cols()) = H_f;
    if (!FilterHelper::gatingTest(H_xf, r_i, R_i, variableCov2, estimatorOptions_.useMahalanobisGating)) {
      mapPoint.setMeasurementFate(FeatureTrackStatus::kPotentialOutlier);
      continue;
    }
    mapPoint.setMeasurementFate(FeatureTrackStatus::kSuccessful);
    slamLandmarks_.emplace_back(mapPoint);
    vr_i.push_back(r_i);
    vH_x.push_back(H_x);
    vH_f.push_back(H_f);
    vR_i.push_back(R_i);
    numSlamObservations += r_i.size();
  }
  VLOG(1) << "Used #MSCKF landmarks " << numMsckfLandmarks << " out of "
          << msckfLandmarks.size() << " candidates. "
          << "Used #observation rows " << numSlamObservations
          << " out of #SLAM landmarks " << slamLandmarks.size();
  // update with MSCKF features
  if (dimH_o[0] > 0) {
    Eigen::MatrixXd H_o(dimH_o[0], dimH_o[1]);
    Eigen::Matrix<double, Eigen::Dynamic, 1> r_o(dimH_o[0], 1);
    Eigen::MatrixXd R_o = Eigen::MatrixXd::Zero(dimH_o[0], dimH_o[0]);
    FilterHelper::stackJacobianAndResidual(vH_o, vr_o, vR_o, &H_o, &r_o, &R_o);
    Eigen::MatrixXd T_H, R_q;  // residual, Jacobian, and noise covariance after projecting to the column space of H_o.
    Eigen::Matrix<double, -1, 1> r_q;
    FilterHelper::shrinkResidual(H_o, r_o, R_o, &T_H, &r_q, &R_q);

    DefaultEkfUpdater updater(covariance_, navAndImuParamsDim, dimH_o[1]);
    computeKalmanGainTimer.start();
    Eigen::Matrix<double, Eigen::Dynamic, 1> deltaX =
        updater.computeCorrection(T_H, r_q, R_q);
    computeKalmanGainTimer.stop();
    updateStatesTimer.start();
    updateStates(deltaX);
    updateStatesTimer.stop();
    updateCovarianceTimer.start();
    updater.updateCovariance(&covariance_);
    updateCovarianceTimer.stop();
  }

  // update with SLAM features
  if (numSlamObservations) {
    const size_t numPointStates = 3 * orderedLandmarks_.size();
    Eigen::MatrixXd H_all(numSlamObservations, numCamPosePointStates);
    H_all.block(0, numCamPosePointStates - numPointStates, numSlamObservations, numPointStates).setZero();
    Eigen::VectorXd r_all(numSlamObservations);
    Eigen::MatrixXd R_all = Eigen::MatrixXd::Zero(numSlamObservations, numSlamObservations);
    size_t startRow = 0u;
    for (size_t jack = 0u; jack < vr_i.size(); ++jack) {
      int blockRows = vr_i[jack].rows();
      H_all.block(startRow, 0, blockRows, numCamPosePointStates - numPointStates) =
          vH_x[jack];
      H_all.block(startRow, numCamPosePointStates - numPointStates, blockRows,
                  numPointStates) = vH_f[jack];
      r_all.segment(startRow, blockRows) = vr_i[jack];
      R_all.block(startRow, startRow, blockRows, blockRows) = vR_i[jack];
      startRow += blockRows;
    }

    DefaultEkfUpdater updater(covariance_, navAndImuParamsDim, numCamPosePointStates);
    computeKalmanGainTimer.start();
    Eigen::Matrix<double, Eigen::Dynamic, 1> deltaX =
        updater.computeCorrection(H_all, r_all, R_all);
    computeKalmanGainTimer.stop();
    updateStatesTimer.start();
    updateStates(deltaX);
    updateStatesTimer.stop();
    updateCovarianceTimer.start();
    updater.updateCovariance(&covariance_);
    updateCovarianceTimer.stop();
  }

  // initialize SLAM features and update state.
  initializeLandmarksInFilter();

  // synchronize landmark positions.
  for (auto it = landmarksMap_.begin(); it != landmarksMap_.end(); ++it) {
    if (it->second.inState()) {
      it->second.quality = 1.0;
      uint64_t toFind = it->first;
      auto landmarkIter = std::find_if(
          orderedLandmarks_.begin(), orderedLandmarks_.end(),
          [toFind](const PointLandmark &x) {
            return x.id() == toFind;
          });
      if (it->second.anchorStateId == 0u) {
        it->second.pointHomog = landmarkIter->estimate();
      } else {
        it->second.pointHomog = EstimationHelper::anchoredInverseDepthToWorldCoordinates(
              mapPtr_, cameraRig_, landmarkIter->estimate(), it->second.anchorCamera());
      }
    } else {  // update landmark positions for visualization and loop closure.
      constexpr bool showPrematureLandmarks = false;
      if (showPrematureLandmarks) {
        if (it->second.observations.size() < pointLandmarkOptions_.minTrackLengthForMsckf) {
          continue;
        }

        Eigen::AlignedVector<Eigen::Vector2d> obsList;
        std::vector<uint64_t> frameIds;
        Eigen::AlignedVector<Eigen::Vector2d> obsStdList;

        swift_vio::PointLandmark pointLandmark(
            it->first,
            okvis::ceres::HomogeneousPointLocalParameterization::kModelId,
            &homogeneousLandmarkParamization_);
        swift_vio::PointSharedData psd;
        triangulateTimer.start();
        swift_vio::TriangulationStatus status =
            EstimationHelper::triangulateMapPoint(it->second, mapPtr_, cameraRig_, &pointLandmark, &obsList,
                                &obsStdList, &psd, pointLandmarkOptions_.minTrackLengthForMsckf, false);
        triangulateTimer.stop();
        if (status.triangulationOk) {
          it->second.quality = 1.0;
          it->second.pointHomog = pointLandmark.estimate();
          it->second.setInitialized(true);
        } else {
          it->second.quality = 0.0;
        }
      }
    }
  }
}

bool SlidingWindowFilter::removeLandmarkParameterBlock(uint64_t landmarkId) {
  auto idPos = std::find_if(
      orderedLandmarks_.begin(), orderedLandmarks_.end(),
      [landmarkId](const PointLandmark &x) {
        return x.id() == landmarkId;
      });
  bool miss = (idPos == orderedLandmarks_.end());
  if (miss) {
    return false;
  } else {
    orderedLandmarks_.erase(idPos);
    return true;
  }
}

bool SlidingWindowFilter::hasLandmarkParameterBlock(uint64_t landmarkId) const {
  auto idPos = std::find_if(
      orderedLandmarks_.begin(), orderedLandmarks_.end(),
      [landmarkId](const PointLandmark &x) {
        return x.id() == landmarkId;
      });
  return idPos != orderedLandmarks_.end();
}

void SlidingWindowFilter::decimateCovarianceForLandmarks(const std::vector<uint64_t>& toRemoveLmIds) {
  if (toRemoveLmIds.size() == 0u)
    return;
  const size_t numNavImuCamStates = startIndexOfClonedStatesFast();
  const size_t numNavImuCamPoseStates = numNavImuCamStates + 9 * statesMap_.size();

  std::vector<size_t> toRemoveIndices;  // start indices of removed landmark parameters,
                                        // each interval of size 3
  toRemoveIndices.reserve(toRemoveLmIds.size());
  int covDim = covariance_.rows();
  for (auto it = toRemoveLmIds.begin(), itEnd = toRemoveLmIds.end();
       it != itEnd; ++it) {
    uint64_t toFind = *it;
    auto idPos = std::find_if(
        orderedLandmarks_.begin(), orderedLandmarks_.end(),
        [toFind](const PointLandmark &x) {
          return x.id() == toFind;
        });
    OKVIS_ASSERT_TRUE(Exception, idPos != orderedLandmarks_.end(),
                      "The landmark in state is not in orderedLandmarks_!");

    int startIndex = numNavImuCamPoseStates + 3 * std::distance(orderedLandmarks_.begin(), idPos);
    toRemoveIndices.push_back(startIndex);
  }
  std::sort(toRemoveIndices.begin(), toRemoveIndices.end());
  std::vector<std::pair<size_t, size_t>> vRowStartInterval;
  vRowStartInterval.reserve(toRemoveLmIds.size() + 1);
  size_t startKeptRow = 0;  // start index of the kept rows.
  for (auto it = toRemoveIndices.begin(), itEnd = toRemoveIndices.end();
       it != itEnd; ++it) {
    vRowStartInterval.push_back(
        std::make_pair(startKeptRow, *it - startKeptRow));
    startKeptRow = *it + 3;
  }
  if (startKeptRow < (size_t)covDim) {
    vRowStartInterval.push_back(
        std::make_pair(startKeptRow, (size_t)covDim - startKeptRow));
  }
  covariance_ = vio::extractBlocks(covariance_, vRowStartInterval, vRowStartInterval);

  for (auto it = toRemoveLmIds.begin(), itEnd = toRemoveLmIds.end();
       it != itEnd; ++it) {
    removeLandmarkParameterBlock(*it);
  }
}

bool SlidingWindowFilter::applyMarginalizationStrategy(MapPointVector &removedLandmarks) {
  marginalizeRedundantFrames(estimatorOptions_.numKeyframes,
                             estimatorOptions_.numImuFrames,
                             &removedLandmarks);

  // remove features no longer tracked.
  std::vector<uint64_t> toRemoveLmIds;
  toRemoveLmIds.reserve(10);
  for (PointMap::iterator pit = landmarksMap_.begin(); pit != landmarksMap_.end();) {
    bool lmkInState = pit->second.inState();
    if (pit->second.shouldRemove(pointLandmarkOptions_.maxHibernationFrames)) {
      size_t numTimesObserved = pit->second.numTimesObserved();
      size_t binIndex = numTimesObserved >= trackLengthAccumulator.size() ? trackLengthAccumulator.size() - 1u : numTimesObserved;
      ++trackLengthAccumulator[binIndex];
      ObservationMap &observationList = pit->second.observations;
      for (auto iter = observationList.begin(); iter != observationList.end(); ++iter) {
        const okvis::KeypointIdentifier &kpi = iter->first;
        auto mfp = nframeToLandmarkIds_.find(kpi.frameId);
        mfp->second.setLandmarkId(kpi.cameraIndex, kpi.keypointIndex, 0);
      }

      if (lmkInState) {
        toRemoveLmIds.push_back(pit->first);
      }

      mapPtr_->removeParameterBlock(pit->first);
      removedLandmarks.emplace_back(pit->second);
      pit = landmarksMap_.erase(pit);
    } else {
      ++pit;
    }
  }

  decimateCovarianceForLandmarks(toRemoveLmIds);
  return true;
}

void SlidingWindowFilter::updateStates(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& deltaX) {
  const size_t numNavImuCamStates = startIndexOfClonedStatesFast();
  const size_t numNavImuCamPoseStates =
      numNavImuCamStates + 9 * statesMap_.size();
  CHECK_EQ(deltaX.size(), (int)(numNavImuCamPoseStates + orderedLandmarks_.size() * 3))
      << "Inconsistent size of the Kalman correction!";
  CHECK_LT((deltaX.head<9>() - deltaX.segment<9>(numNavImuCamPoseStates - 9))
               .lpNorm<Eigen::Infinity>(),
           1e-7)
      << "Correction to the current states from head and tail should be "
         "identical!\nHead pose velocity: "
      << deltaX.head<9>().transpose() << "\nTail pose velocity:"
      << deltaX.segment<9>(numNavImuCamPoseStates - 9).transpose()
      << "\nThis issue is usually caused by numerical imprecision. See more "
         "info by searching keywords 'duplicate rows for the same variable'";

  const States& currentState = statesMap_.rbegin()->second;
  uint64_t stateId = currentState.id;

  // update global states
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParamBlockPtr =
      std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
          mapPtr_->parameterBlockPtr(stateId));
  okvis::kinematics::Transformation T_WS = poseParamBlockPtr->estimate();
  swift_vio::PoseLocalParameterizationSimplified::oplus(
      T_WS.parameterPtr(), deltaX.data(), T_WS.parameterMutablePtr());
  poseParamBlockPtr->setEstimate(T_WS);

  uint64_t vid = currentState.global.at(GlobalStates::v_WS).id;
  std::shared_ptr<okvis::ceres::SpeedParameterBlock> vParamBlockPtr =
      std::static_pointer_cast<okvis::ceres::SpeedParameterBlock>(
          mapPtr_->parameterBlockPtr(vid));
  Eigen::Vector3d v_WB = vParamBlockPtr->estimate();
  vParamBlockPtr->setEstimate(v_WB + deltaX.segment<3>(6));

  // update imu sensor states
  FilterHelper::updateImuStates(currentState, deltaX, imuRig_, mapPtr_);

  if (imuParametersVec_.at(0)->isGravityDirectionVariable()) {
    uint64_t gWId = currentState.global.at(GlobalStates::GravityDirection).id;
    std::shared_ptr<okvis::ceres::NormalVectorParameterBlock>
        paramBlockPtr = std::static_pointer_cast<
            okvis::ceres::NormalVectorParameterBlock>(
            mapPtr_->parameterBlockPtr(gWId));
    Eigen::Vector3d newEstimate = NormalVectorElement::boxPlus(
        paramBlockPtr->estimate(), deltaX.segment<2>(currentState.global.at(GlobalStates::GravityDirection).startIndexInCov));
    paramBlockPtr->setEstimate(newEstimate);
  }

  FilterHelper::updateCameraSensorStates(currentState, deltaX, cameraRig_, cameraNoiseParametersVec_, mapPtr_);

  // Update cloned states except for the last one, which is the current state,
  // corresponding to the global state, has been updated.
  size_t jack = 0;
  auto finalIter = statesMap_.end();
  --finalIter;

  for (auto iter = statesMap_.begin(); iter != finalIter; ++iter, ++jack) {
    stateId = iter->first;
    size_t pOffset = iter->second.global.at(GlobalStates::T_WS).startIndexInCov;

    poseParamBlockPtr = std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
        mapPtr_->parameterBlockPtr(stateId));
    T_WS = poseParamBlockPtr->estimate();
    swift_vio::PoseLocalParameterizationSimplified::oplus(
        T_WS.parameterPtr(), deltaX.data() + pOffset,
        T_WS.parameterMutablePtr());
    poseParamBlockPtr->setEstimate(T_WS);

    uint64_t vid = iter->second.global.at(GlobalStates::v_WS).id;
    vParamBlockPtr =
        std::static_pointer_cast<okvis::ceres::SpeedParameterBlock>(
            mapPtr_->parameterBlockPtr(vid));
    Eigen::Vector3d v_WB = vParamBlockPtr->estimate();
    v_WB += deltaX.segment<3>(pOffset + 6);
    vParamBlockPtr->setEstimate(v_WB);
  }

  // update feature states.
  int numberLandmarks = 0;
  size_t lkStart = startIndexOfClonedStatesFast() + kClonedStateMinimalDim * statesMap_.size();
  size_t aStart = lkStart - 3;  // a dummy initialization.
  for (auto iter = orderedLandmarks_.begin(), iterEnd = orderedLandmarks_.end();
       iter != iterEnd; ++iter, ++numberLandmarks) {
    aStart = lkStart + 3 * numberLandmarks;
    iter->Plus(deltaX.data() + aStart);
  }
  OKVIS_ASSERT_EQ_DBG(Exception, aStart + 3, (size_t)deltaX.rows(),
                      "deltaX size not equal to the expected value.");

  EstimationHelper::updateCameraRig(currentState, mapPtr_, cameraNoiseParametersVec_, &cameraRig_);
  EstimationHelper::updateImuRig(currentState, mapPtr_, &imuRig_);
  EstimationHelper::updateImuParametersVector(currentState, mapPtr_, &imuParametersVec_);
}

int SlidingWindowFilter::marginalizeRedundantFrames(
    size_t numKeyframes, size_t numImuFrames, MapPointVector *removedLandmarks) {
  if (statesMap_.size() < numKeyframes + numImuFrames) {
    return 0;
  }
  std::vector<uint64_t> toRemoveStateIds;
  // jhuai: I think removing one frame or several frames at once does not make much difference.
  EstimationHelper::findRedundantNavStates(statesMap_, mapPtr_, &toRemoveStateIds,
                                       numImuFrames, estimatorOptions_.minMarginalizedFrames);

  size_t numMsckfLandmarks = 0u;
  const size_t gravityDimInCov = imuParametersVec_.at(0)->isGravityDirectionVariable() ? 2u : 0u;
  int featureVariableDim = gravityDimInCov + minimalDimOfAllCameraParams() +
      kClonedStateMinimalDim * (statesMap_.size() - 1);
  int navAndImuParamsDim = startIndexOfGravity();
  const Eigen::MatrixXd featureVariableCov =
      covariance_.block(navAndImuParamsDim, navAndImuParamsDim,
                        featureVariableDim, featureVariableDim);
  int dimH_o[2] = {0, featureVariableDim};
  // containers of Jacobians of measurements
  Eigen::AlignedVector<Eigen::Matrix<double, -1, 1>> vr_o;
  Eigen::AlignedVector<Eigen::MatrixXd> vH_o;
  Eigen::AlignedVector<Eigen::MatrixXd> vR_o;

  // for each map point in the landmarksMap_,
  // see if the landmark is observed in the redundant frames
  for (PointMap::iterator it = landmarksMap_.begin();
       it != landmarksMap_.end(); ++it) {
    if (!it->second.goodForMarginalization(estimatorOptions_.minMarginalizedFrames)) {
      continue;
    }

    std::vector<uint64_t> involved_cam_state_ids;
    auto obsMap = it->second.observations;
    auto obsSearchStart = obsMap.begin();
    for (auto stateId : toRemoveStateIds) {
      auto obsIter = std::find_if(obsSearchStart, obsMap.end(),
                                  IsObservedInNFrame(stateId));
      if (obsIter != obsMap.end()) {
        involved_cam_state_ids.emplace_back(stateId);
        obsSearchStart = obsIter;
        ++obsSearchStart;
      }
    }
    if (involved_cam_state_ids.size() < estimatorOptions_.minMarginalizedFrames) {
      continue;
    }

    swift_vio::PointLandmark landmark(okvis::IdProvider::instance().newId(),
                                      pointLandmarkOptions_.landmarkModelId,
                                      landmarkParamizationPtr_.get());
    Eigen::MatrixXd H_oi;                           //(nObsDim, dimH_o[1])
    Eigen::Matrix<double, Eigen::Dynamic, 1> r_oi;  //(nObsDim, 1)
    Eigen::MatrixXd R_oi;                           //(nObsDim, nObsDim)
    std::shared_ptr<swift_vio::PointSharedData> pointSharedData(new swift_vio::PointSharedData());
    computeHTimer.start();
    FeatureJacobianStatus status =
        FilterJacobians::featureJacobian(it->second, statesMap_, mapPtr_, nframeToLandmarkIds_,
                                         cameraRig_, cameraNoiseParametersVec_, imuParametersVec_.at(0), estimatorOptions_,
                                         &landmark, pointSharedData.get(), H_oi, r_oi, R_oi, nullptr, &involved_cam_state_ids,
                                         pointLandmarkOptions_.minTrackLengthForMsckf, estimatorOptions_.minMarginalizedFrames);
    computeHTimer.stop();
    if (!status.successful) {
      continue;
    }

    if (!FilterHelper::gatingTest(H_oi, r_oi, R_oi, featureVariableCov, estimatorOptions_.useMahalanobisGating)) {
      continue;
    }

    vr_o.push_back(r_oi);
    vR_o.push_back(R_oi);
    vH_o.push_back(H_oi);
    dimH_o[0] += r_oi.rows();
    ++numMsckfLandmarks;
  }

  if (numMsckfLandmarks > 0u) {
    LOG(INFO) << "Used " << numMsckfLandmarks << " MSCKF landmarks for update in marginalization!";
    Eigen::MatrixXd H_o =
        Eigen::MatrixXd::Zero(dimH_o[0], featureVariableDim);
    Eigen::Matrix<double, -1, 1> r_o(dimH_o[0], 1);
    Eigen::MatrixXd R_o = Eigen::MatrixXd::Zero(dimH_o[0], dimH_o[0]);
    FilterHelper::stackJacobianAndResidual(vH_o, vr_o, vR_o, &H_o, &r_o, &R_o);
    Eigen::MatrixXd T_H, R_q;
    Eigen::Matrix<double, Eigen::Dynamic, 1> r_q;
    FilterHelper::shrinkResidual(H_o, r_o, R_o, &T_H, &r_q, &R_q);

    DefaultEkfUpdater updater(covariance_, navAndImuParamsDim, featureVariableDim);
    computeKalmanGainTimer.start();
    Eigen::Matrix<double, Eigen::Dynamic, 1> deltaX =
        updater.computeCorrection(T_H, r_q, R_q);
    computeKalmanGainTimer.stop();
    updateStates(deltaX);

    updateCovarianceTimer.start();
    updater.updateCovariance(&covariance_);
    updateCovarianceTimer.stop();
  }

  // sanity check
  for (const auto &cam_id : toRemoveStateIds) {
    int stateOrder =
        std::distance(statesMap_.begin(), statesMap_.find(cam_id));
    OKVIS_ASSERT_EQ(Exception,
                    stateOrder * kClonedStateMinimalDim +
                        startIndexOfClonedStatesFast(),
                    statesMap_[cam_id].global.at(GlobalStates::T_WS).startIndexInCov,
                    "Inconsistent state order in covariance");
  }

  // remove observations in removed frames
  for (PointMap::iterator it = landmarksMap_.begin();
       it != landmarksMap_.end();) {
    MapPoint& mapPoint = it->second;
    std::map<okvis::KeypointIdentifier, KeypointObservation>::iterator obsIter =
        mapPoint.observations.begin();
    for (uint64_t stateId : toRemoveStateIds) {
      while (obsIter != mapPoint.observations.end() &&
             obsIter->first.frameId < stateId) {
        ++obsIter;
      }
      while (obsIter != mapPoint.observations.end() &&
             obsIter->first.frameId == stateId) {
        // loop in case that there are more than one observations in an NFrame for a landmark.
        const okvis::KeypointIdentifier& kpi = obsIter->first;
        auto mfp = nframeToLandmarkIds_.find(kpi.frameId);
        mfp->second.setLandmarkId(kpi.cameraIndex, kpi.keypointIndex, 0);

        obsIter = mapPoint.observations.erase(obsIter);
      }
    }
    if (mapPoint.observations.size() == 0u) {
      uint64_t landmarkId = it->first;
      mapPtr_->removeParameterBlock(landmarkId);
      if (hasLandmarkParameterBlock(landmarkId)) {
        decimateCovarianceForLandmarks({landmarkId});
      }
      removedLandmarks->emplace_back(it->second);
      it = landmarksMap_.erase(it);
    } else {
      ++it;
    }
  }

  FilterJacobians::changeAnchors(toRemoveStateIds, statesMap_, mapPtr_, cameraRig_,
                                 cameraNoiseParametersVec_,
                                 &orderedLandmarks_, &landmarksMap_, &covariance_);

  for (const auto &cam_id : toRemoveStateIds) {
    auto statesIter = statesMap_.find(cam_id);
    int stateOrder =
        std::distance(statesMap_.begin(), statesIter);
    int stateStartIdInCov =
        startIndexOfClonedStatesFast() + kClonedStateMinimalDim * stateOrder;
    int stateEndIdInCov = stateStartIdInCov + kClonedStateMinimalDim;

    FilterHelper::pruneSquareMatrix(stateStartIdInCov, stateEndIdInCov,
                                    &covariance_);
    removeState(cam_id);
  }
  updateCovarianceIndex();

  inertialMeasForStates_.pop_front(statesMap_.begin()->second.timestamp - half_window_);
  return toRemoveStateIds.size();
}

// Does this function deal with insertion of nav states correspond to database keyframes?
// Yes. These states will be put in the front of the states map,
// but they are not necessarily marginalized first, see findRedundantNavStates.
void SlidingWindowFilter::updateCovarianceIndex() {
  size_t nCovIndex = startIndexOfClonedStatesFast();
  for (std::map<uint64_t, States, std::less<uint64_t>,
                Eigen::aligned_allocator<std::pair<const uint64_t, States>>>::
           iterator iter = statesMap_.begin();
       iter != statesMap_.end(); ++iter) {
    iter->second.global.at(GlobalStates::T_WS).startIndexInCov = nCovIndex;
    iter->second.global.at(GlobalStates::v_WS).startIndexInCov = nCovIndex + 6;
    nCovIndex += kClonedStateMinimalDim;
  }
}

swift_vio::TriangulationStatus SlidingWindowFilter::triangulateMapPoint(
    const MapPoint &mp,
    Eigen::AlignedVector<Eigen::Vector2d> *obsList,
    swift_vio::PointLandmark *pointLandmark,
    Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
    swift_vio::PointSharedData *pointDataPtr,
    std::vector<uint64_t> *orderedCulledFrameIds, bool checkDisparity) const {
  triangulateTimer.start();
  TriangulationStatus status = EstimationHelper::triangulateMapPointRs(
      mp, statesMap_, mapPtr_, nframeToLandmarkIds_, cameraRig_,
      imuParametersVec_.at(0), pointLandmark, obsList, obsStdList,
      pointDataPtr, orderedCulledFrameIds,
      pointLandmarkOptions_.minTrackLengthForMsckf,
      estimatorOptions_.minMarginalizedFrames,
      checkDisparity, false);
  triangulateTimer.stop();
  return status;
}

bool SlidingWindowFilter::measurementJacobian(
    const PointLandmark& homogeneousPoint,
    const Eigen::Vector2d& obs,
    size_t observationIndex,
    const swift_vio::PointSharedData &pointData,
    Eigen::Matrix<double, 2, Eigen::Dynamic>* J_x,
    Eigen::Matrix<double, 2, 3>* J_pfi,
    Eigen::Vector2d* residual,
    bool mixProjectionJac) const {
  bool status = FilterJacobians::measurementJacobian(
      homogeneousPoint, obs, observationIndex, pointData, cameraRig_,
      cameraNoiseParametersVec_,
      statesMap_, estimatorOptions_, imuParametersVec_.at(0), J_x, J_pfi,
      residual, mixProjectionJac);
  return status;
}

bool SlidingWindowFilter::measurementJacobianNumeric(
    const PointLandmark &homogeneousPoint, const Eigen::Vector2d &obs,
    size_t observationIndex, swift_vio::PointSharedData& pointData,
    Eigen::Matrix<double, 2, Eigen::Dynamic> *J_x,
    Eigen::Matrix<double, 2, 3> *J_pfi, Eigen::Vector2d *residual) {
  bool status = FilterJacobians::measurementJacobianNumeric(
      homogeneousPoint, obs, observationIndex, pointData, cameraRig_,
      cameraNoiseParametersVec_, statesMap_, mapPtr_, estimatorOptions_,
      imuParametersVec_.at(0), J_x, J_pfi, residual);
  return status;
}

bool SlidingWindowFilter::measurementJacobianAutoDiff(
    const PointLandmark &pointLandmark, const Eigen::Vector2d &obs,
    size_t observationIndex,
    const swift_vio::PointSharedData &pointData,
    Eigen::Matrix<double, 2, -1> *J_X, Eigen::Matrix<double, 2, 3> *J_pfi,
    Eigen::Vector2d *residual) const {
  bool status = FilterJacobians::measurementJacobianAutoDiff(
          pointLandmark, obs, observationIndex, pointData, cameraRig_,
          cameraNoiseParametersVec_,
          statesMap_, estimatorOptions_, imuParametersVec_.at(0), J_X, J_pfi,
          residual);
  return status;
}

bool SlidingWindowFilter::printStatesAndStdevs(std::ostream& stream, const Eigen::MatrixXd *covariance) const {
  EstimatorBase::printNavState(stream, statesMap_.rbegin()->first);

  EstimatorBase::printImuParameters(stream, statesMap_.rbegin()->first);

  EstimatorBase::printGravityParameters(stream);

  EstimatorBase::printCameraParameters(stream, statesMap_.rbegin()->first);

  const int stateDim = startIndexOfClonedStatesFast();
  Eigen::Matrix<double, Eigen::Dynamic, 1> variances;
  if (covariance) {
    variances = covariance->topLeftCorner(stateDim, stateDim).diagonal();
  } else {
    variances = covariance_.topLeftCorner(stateDim, stateDim).diagonal();
  }
  stream << " " << variances.cwiseSqrt().transpose().format(kSpaceInitFmt) << "\n";
  return true;
}

void SlidingWindowFilter::printTrackLengthHistogram(std::ostream &stream) const {
  stream << "Track length histogram in one test with bins 0, 1, 2...\n";
  for (size_t bin = 0u; bin < trackLengthAccumulator.size(); ++bin) {
    stream << bin << "\t";
  }
  stream << std::endl;
  for (auto it = trackLengthAccumulator.begin();
       it != trackLengthAccumulator.end(); ++it) {
    stream << *it << "\t";
  }
  stream << std::endl;
}

bool SlidingWindowFilter::getStateStd(
    Eigen::Matrix<double, Eigen::Dynamic, 1>* stateStd) const {
  const int dim = startIndexOfClonedStatesFast();
  *stateStd = covariance_.topLeftCorner(dim, dim).diagonal().cwiseSqrt();
  return true;
}

Eigen::VectorXd SlidingWindowFilter::getDesiredImuAugmentedParamStdevs() const {
  Eigen::VectorXd desiredImuIntrinsicStdevs;
  ImuModelToAugmentedDesiredStdevs(
      ImuModelNameToId(imuParametersVec_.at(0)->model_name),
      &desiredImuIntrinsicStdevs);
  return desiredImuIntrinsicStdevs;
}

Eigen::VectorXd SlidingWindowFilter::getDesiredCameraParamStdevs() const {
  Eigen::VectorXd desiredCameraParamStdevs;
  desiredCameraParamStdevs.resize(minimalDimOfAllCameraParams());
  int index = 0;
  size_t numCameras = cameraNoiseParametersVec_.size();
  for (size_t j = 0u; j < numCameras; ++j) {
    if (!cameraNoiseParametersVec_.at(j).isExtrinsicsFixed()) {
      Eigen::VectorXd camExtrinsicStdevs;
      ExtrinsicRepToDesiredStdevs(cameraRig_.getExtrinsicRepId(j),
                                    &camExtrinsicStdevs);
      desiredCameraParamStdevs.segment(index, camExtrinsicStdevs.size()) =
          camExtrinsicStdevs;
      index += camExtrinsicStdevs.size();
    }

    if (!cameraNoiseParametersVec_.at(j).isIntrinsicsFixed()) {
      Eigen::VectorXd camProjectionIntrinsicStdevs;
      ProjIntrinsicRepToDesiredStdevs(cameraRig_.getProjectionIntrinsicRepId(j),
                                   &camProjectionIntrinsicStdevs);
      Eigen::VectorXd camDistortionStdevs;
      swift_vio::DistortionTypeToDesiredStdevs(
          cameraRig_.distortionType(j), &camDistortionStdevs);

      desiredCameraParamStdevs.segment(index,
                                       camProjectionIntrinsicStdevs.size()) =
          camProjectionIntrinsicStdevs;
      index += camProjectionIntrinsicStdevs.size();
      desiredCameraParamStdevs.segment(index, camDistortionStdevs.size()) =
          camDistortionStdevs;
      index += camDistortionStdevs.size();
    }
    if (!cameraNoiseParametersVec_.at(j).isTimeDelayFixed()) {
      desiredCameraParamStdevs[index] = 1e-3;
      index++;
    }
    if (!cameraNoiseParametersVec_.at(j).isReadoutTimeFixed()) {
      desiredCameraParamStdevs[index] = 1e-3;
      index++;
    }
  }
  return desiredCameraParamStdevs;
}

bool SlidingWindowFilter::getDesiredStdevs(Eigen::VectorXd *desiredStdevs) const {
  EstimatorBase::getDesiredStdevs(desiredStdevs);

  desiredStdevs->conservativeResize(startIndexOfClonedStates());
  Eigen::VectorXd desiredImuIntrinsicStdevs =
      getDesiredImuAugmentedParamStdevs();
  Eigen::VectorXd desiredCameraParamStdevs = getDesiredCameraParamStdevs();
  desiredStdevs->segment(15, desiredImuIntrinsicStdevs.size()) =
      desiredImuIntrinsicStdevs;
  if (imuParametersVec_.at(0)->isGravityDirectionVariable()) {
    desiredStdevs->segment<2>(startIndexOfGravity())
        .setConstant(0.01);
  }
  desiredStdevs->segment(startIndexOfCameraParams(),
                         desiredCameraParamStdevs.size()) =
      desiredCameraParamStdevs;
  return true;
}

bool SlidingWindowFilter::computeCovariance(Eigen::MatrixXd* cov) const {
  *cov = covariance_;
  return true;
}

bool SlidingWindowFilter::computeErrors(
    const okvis::kinematics::Transformation &ref_T_WS,
    const Eigen::Vector3d &ref_v_WS, const Eigen::Matrix<double, 6, 1> &biases,
    const okvis::ImuParameters &refImuParams,
    std::shared_ptr<const CameraRig> refCameraSystem,
    Eigen::VectorXd *errors) const {
  EstimatorBase::computeErrors(ref_T_WS, ref_v_WS, biases, refImuParams, refCameraSystem, errors);

  Eigen::VectorXd imuIntrinsicsError = imuRig_.computeImuAugmentedParamsError(0);

  const States& currentState = statesMap_.rbegin()->second;
  Eigen::VectorXd cameraParamsError = computeCameraParamsError(currentState, refCameraSystem);

  errors->conservativeResize(startIndexOfClonedStates(), 1);
  int gravityVarDim = imuParametersVec_.at(0)->isGravityDirectionFixed() ? 0 : 2;
  OKVIS_ASSERT_EQ(Exception, (int)startIndexOfGravity() + gravityVarDim + cameraParamsError.size(),
                  errors->size(), "Wrong error vector dimension!");
  errors->segment(15, imuIntrinsicsError.size()) = imuIntrinsicsError;
  if (imuParametersVec_.at(0)->isGravityDirectionVariable()) {
    uint64_t gWId = statesMap_.rbegin()
                        ->second.global.at(GlobalStates::GravityDirection)
                        .id;
    std::shared_ptr<const okvis::ceres::NormalVectorParameterBlock>
        paramBlockPtr = std::static_pointer_cast<
            const okvis::ceres::NormalVectorParameterBlock>(
            mapPtr_->parameterBlockPtr(gWId));
    errors->segment<2>(startIndexOfGravity()) =
        NormalVectorElement::boxMinus(refImuParams.gravityDirection(),
                                      paramBlockPtr->estimate()) *
        refImuParams.g;
  }

  errors->segment(startIndexOfCameraParams(),
                  cameraParamsError.size()) = cameraParamsError;
  return true;
}

Eigen::VectorXd SlidingWindowFilter::computeCameraParamsError(
    const States &currentState,
    std::shared_ptr<const CameraRig> refCameraSystem) const {
  okvis::kinematics::Transformation ref_T_BC0 = *refCameraSystem->getCameraExtrinsicPtr(0);
  int paramIndex = 0;
  Eigen::Matrix<double, -1, 1> cameraErrors(minimalDimOfAllCameraParams());
  for (size_t camIdx = 0u; camIdx < cameraRig_.numCameras(); ++camIdx) {
    if (!cameraNoiseParametersVec_.at(camIdx).isExtrinsicsFixed()) {
      int extrinsicRepId = cameraRig_.getExtrinsicRepId(camIdx);
      okvis::kinematics::Transformation ref_T_BCi =
          *refCameraSystem->getCameraExtrinsicPtr(camIdx);
      okvis::kinematics::Transformation ref_T_XCi;
      switch (extrinsicRepId) {
      case Extrinsic_p_C0C_q_C0C::kModelId:
        ref_T_XCi = ref_T_BC0.inverse() * ref_T_BCi;
        break;
      default:
        ref_T_XCi = ref_T_BCi;
        break;
      }

      uint64_t extrinsicId =
          currentState.sensors.at(SensorStates::Camera)
              .at(camIdx)
              .at(CameraSensorStates::T_XCi)
              .id;
      std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicParamBlockPtr =
          std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
              mapPtr_->parameterBlockPtr(extrinsicId));

      okvis::kinematics::Transformation T_XCi =
          extrinsicParamBlockPtr->estimate();

      int minExtrinsicDim = cameraRig_.getMinimalExtrinsicDim(camIdx);
      Eigen::VectorXd delta(minExtrinsicDim);
      ExtrinsicRepOminus(extrinsicRepId, T_XCi.parameterPtr(),
                           ref_T_XCi.parameterPtr(), delta.data());
      cameraErrors.segment(paramIndex, minExtrinsicDim) = delta;
      paramIndex += minExtrinsicDim;
    }

    if (!cameraNoiseParametersVec_.at(camIdx).isIntrinsicsFixed()) {
      uint64_t intrinsicId =
          currentState.sensors.at(SensorStates::Camera)
              .at(camIdx)
              .at(CameraSensorStates::Intrinsics)
              .id;
      std::shared_ptr<const okvis::ceres::EuclideanParamBlock>
          intrinsicParamBlockPtr =
              std::static_pointer_cast<const okvis::ceres::EuclideanParamBlock>(
                  mapPtr_->parameterBlockPtr(intrinsicId));
      Eigen::VectorXd allIntrinsics = intrinsicParamBlockPtr->estimate();

      Eigen::VectorXd refIntrinsics;
      refCameraSystem->cameraGeometry(camIdx)->getIntrinsics(refIntrinsics);

      cameraErrors.segment(paramIndex, allIntrinsics.size()) = refIntrinsics - allIntrinsics;
      paramIndex += allIntrinsics.size();
    }

    if (!cameraNoiseParametersVec_.at(camIdx).isTimeDelayFixed()) {
      uint64_t tdId = currentState.sensors.at(SensorStates::Camera)
                          .at(camIdx)
                          .at(CameraSensorStates::TD)
                          .id;
      std::shared_ptr<okvis::ceres::ParameterBlock> tdParamBlockPtr =
          mapPtr_->parameterBlockPtr(tdId);
      cameraErrors[paramIndex] =
          refCameraSystem->cameraGeometry(camIdx)->imageDelay() -
          tdParamBlockPtr->parameters()[0];
      paramIndex += 1;
    }
    if (!cameraNoiseParametersVec_.at(camIdx).isReadoutTimeFixed()) {
      uint64_t trId = currentState.sensors.at(SensorStates::Camera)
                          .at(camIdx)
                          .at(CameraSensorStates::TR)
                          .id;
      std::shared_ptr<okvis::ceres::ParameterBlock> trParamBlockPtr =
          mapPtr_->parameterBlockPtr(trId);
      cameraErrors[paramIndex] =
          refCameraSystem->cameraGeometry(camIdx)->readoutTime() -
          trParamBlockPtr->parameters()[0];
      paramIndex += 1;
    }
  }
  return cameraErrors;
}

bool SlidingWindowFilter::getOdometryConstraintsForKeyframe(
    std::shared_ptr<LoopQueryKeyframeMessage> queryKeyframe) const {
  int j = 0;
  std::vector<std::shared_ptr<NeighborConstraintMessage>>&
      odometryConstraintList = queryKeyframe->odometryConstraintList_;
  odometryConstraintList.reserve(
      poseGraphOptions_.maxOdometryConstraintForAKeyframe);
  okvis::kinematics::Transformation T_WBr = queryKeyframe->T_WB_;
  auto kfCovIndexIter = statesMap_.find(queryKeyframe->id_);
  int cov_T_WBr_start = kfCovIndexIter->second.global.at(GlobalStates::T_WS).startIndexInCov;
  queryKeyframe->setCovariance(covariance_.block<6, 6>(cov_T_WBr_start, cov_T_WBr_start), false);
  auto riter = statesMap_.rbegin();
  for (++riter;  // skip the last frame which is queryKeyframe.
       riter != statesMap_.rend() && j < poseGraphOptions_.maxOdometryConstraintForAKeyframe;
       ++riter) {
    if (riter->second.isKeyframe) {
      okvis::kinematics::Transformation T_WBn;
      get_T_WS(riter->first, T_WBn);
      okvis::kinematics::Transformation T_BnBr = T_WBn.inverse() * T_WBr;
      std::shared_ptr<NeighborConstraintMessage> odometryConstraint(
          new NeighborConstraintMessage(
              riter->first, riter->second.timestamp, T_BnBr, T_WBn));

      auto poseCovIndexIter = statesMap_.find(riter->first);
      int cov_T_WBn_start =
          poseCovIndexIter->second.global.at(GlobalStates::T_WS)
              .startIndexInCov;

      odometryConstraint->cov_T_WB_ = covariance_.block<6, 6>(
            cov_T_WBn_start, cov_T_WBn_start);
      odometryConstraint->cov_T_WBr_T_WB_ = covariance_.block<6, 6>(
            cov_T_WBr_start, cov_T_WBn_start);
      odometryConstraintList.emplace_back(odometryConstraint);
      ++j;
    }
  }
  return true;
}

}  // namespace swift_vio
