#include "swift_vio/ImuInitializer.h"

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
ImuInitializer::ImuInitializer(const okvis::EstimatorOptions &options) :
  EstimatorBase(options), numNFrames_(0u) {
  initStatus_ = InitializationStatus::Ongoing;
}

bool ImuInitializer::getStateStd(
    Eigen::Matrix<double, Eigen::Dynamic, 1>* stateStd) const {
  // return a dummy
  *stateStd = Eigen::MatrixXd::Constant(15, 1, 1.0);
  return true;
}

void ImuInitializer::estimate(
    std::shared_ptr<const VisualMatcherOutput> syncedData) {
  okvis::Time stateTime = TimingUtilities::computeStateTimestamp(
      cameraRig_, syncedData->currentNFrame);

  inertialMeasForStates_.push_back(syncedData->imuMeasurements);

  // track landmarks for optic flow computation.
  EstimationHelper::updateFeatureTracks(syncedData->featureTracks,
                                        syncedData->currentNFrame,
                                        &landmarksMap_, &nframeToLandmarkIds_);

  okvis::Duration radius(2.0 / imuParametersVec_.at(0)->rate);
  if (numNFrames_ == 0) {
    OKVIS_ASSERT_TRUE(Exception, syncedData->currentNFrame->isKeyframe(), "First NFrame should be a keyframe!");
    okvis::kinematics::Transformation T_WS;
    if (initialNavState_.initializeToCustomPose) {
      T_WS.set(initialNavState_.p_WS, initialNavState_.q_WS);
    } else {
      bool success = swift_vio::initPoseFromImu(syncedData->imuMeasurements, stateTime, T_WS, radius);
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

    ++numNFrames_;
    return;
  }

  if ((int)numNFrames_ >= initialNavState_.delayFilterInitByFrames) {
    std::map<uint64_t, std::vector<okvis::Time>> miniNFrameMap;
    EstimationHelper::createMinimalNFrameMap(nframeToLandmarkIds_, &miniNFrameMap);

    std::vector<std::pair<okvis::Duration, float>> meanDisparity;
    EstimationHelper::computeKeypointOpticFlow(landmarksMap_, miniNFrameMap, 0u, &meanDisparity);
    float disparityAtStart = 0.f; // accumulate disparity in the first few frames.
    float accumTime = 0.f;
    int count = 0;
    for (const auto &dtdv : meanDisparity) {
      LOG(INFO) << "dt " << dtdv.first << ", optic flow " << dtdv.second;
      accumTime += dtdv.first.toSec();
      disparityAtStart += dtdv.second;
      ++count;
    }

    if (!initialNavState_.initializeToCustomPose) {
      okvis::kinematics::Transformation T_WS;
      bool success = swift_vio::initPoseFromImu(
          syncedData->imuMeasurements, stateTime, T_WS, radius);

      if (!success) {
        LOG(WARNING) << "Pose could not be initialized from imu measurements!";
        ++numNFrames_;
        return;
      }
      initialNavState_.updatePose(T_WS, stateTime);
    }
    // initialize the biases according to the disparity.
    // If the beginning frames have small disparities, then initialize biases by average IMU data.
    // Otherwise, we keep intact the biases initialized from external input which are usually zeros.
    if (disparityAtStart < initialNavState_.motionDisparityTol) {
      Eigen::Vector3d gravityB = initialNavState_.q_WS.conjugate() *
                                 imuParametersVec_.at(0)->gravity();
      okvis::ImuMeasurementDeque imuMeas = inertialMeasForStates_.find(
          stateTime - radius, stateTime + radius, false);
      okvis::ImuMeasurement biases;
      swift_vio::initBiasesFromStaticImu(imuMeas, gravityB, &biases);
      size_t imuIdx = 0u;
      imuRig_.at(imuIdx).setGyroBias(biases.measurement.gyroscopes);
      imuRig_.at(imuIdx).setAccelBias(biases.measurement.accelerometers);
      LOG(INFO) << "Initialize IMU bias with average IMU data.";
    } else {
      LOG(INFO) << "Initialize IMU bias with default values.";
    }
    initStatus_ = InitializationStatus::RunningNonlinEst;
    // clear variables that will not be used by the subsequent estimator.
    statesMap_.clear();
    mapPtr_.reset(new okvis::ceres::Map());
    landmarksMap_.clear();
    nframeToLandmarkIds_.clear();
  } else {
    okvis::kinematics::Transformation T_WS;
    Eigen::Vector3d v_WS;
    okvis::ImuMeasurementDeque imuMeas = inertialMeasForStates_.find(
        statesMap_.rbegin()->second.timestamp, stateTime, true);
    EstimationHelper::predictNavState(imuMeas, statesMap_.rbegin()->second,
                                      mapPtr_, *imuParametersVec_.at(0),
                                      stateTime, &T_WS, &v_WS);
    correctPoseByCameraMotion(syncedData->relPoseToPriorFrames, &T_WS, &v_WS);

    EstimationHelper::setImuWindow(inertialMeasForStates_,
                                   &statesMap_.rbegin()->second, half_window_);

    statesMap_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(syncedData->currentNFrame->id()),
        std::forward_as_tuple(syncedData->currentNFrame->id(), stateTime,
                              syncedData->currentNFrame->isKeyframe()));

    States &currentState = statesMap_.rbegin()->second;
    const States &previousState = (++statesMap_.rbegin())->second;
    EstimationHelper::setImuWindow(inertialMeasForStates_, &currentState,
                                   half_window_);

    EstimationHelper::addNavStateVariables(T_WS, v_WS, &currentState, mapPtr_,
                                           false);

    EstimationHelper::usePreviousGravity(previousState, &currentState);

    if (estimatorOptions_.constantBias) {
      EstimationHelper::usePreviousImuBias(previousState, &currentState);
    } else {
      EstimationHelper::addImuBiasParameters(imuRig_, &currentState, mapPtr_);
    }

    EstimationHelper::usePreviousImuAugmentedParameters(previousState,
                                                        &currentState);

    EstimationHelper::usePreviousCameraParameters(previousState, &currentState);
  }
  ++numNFrames_;
}

}  // namespace swift_vio
