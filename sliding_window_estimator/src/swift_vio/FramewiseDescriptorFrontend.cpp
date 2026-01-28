#include <swift_vio/FramewiseDescriptorFrontend.hpp>

#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <okvis/VioKeyframeWindowMatchingAlgorithm.hpp>

#include <swift_vio/FrameMatchingStats.hpp>

#include <okvis/CameraModelSwitch.hpp>
#include <okvis/StereoMatchingAlgorithm.hpp>
#include <okvis/ceres/ImuError.hpp>

// cameras and distortions
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/FovDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/timing/Timers.hpp>

// Kneip RANSAC
#include <opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp>

#include <opengv/absolute_pose/FrameNoncentralAbsoluteAdapter.hpp>
#include <opengv/relative_pose/FrameRelativeAdapter.hpp>

#include <opengv/sac/Ransac.hpp>

using namespace okvis;

namespace swift_vio {
FramewiseDescriptorFrontend::FramewiseDescriptorFrontend(size_t numCameras, const FrontendOptions& frontendOptions)
    : okvis::Frontend(numCameras, frontendOptions) {
}

FramewiseDescriptorFrontend::~FramewiseDescriptorFrontend() {
  LOG(INFO) << "Framewise descriptor matching frontend associated " << numNFrames_
            << " NFrames, and selected " << numKeyframes_ << " keyframes.";
}

bool FramewiseDescriptorFrontend::dataAssociationAndInitialization(
    okvis::EstimatorBase& estimator,
    const okvis::VioParameters& params,
    std::shared_ptr<okvis::MultiFrame> nframes,
    bool* asKeyframe) {
  // match new keypoints to existing landmarks/keypoints
  // initialise new landmarks (states)
  // outlier rejection by consistency check
  // RANSAC (2D2D / 3D2D)
  // decide keyframe
  // left-right stereo match & init

  // find distortion type
  okvis::cameras::DistortionType distortionType = params.nCameraSystem
      .distortionType(0);
  for (size_t i = 1; i < params.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception,
                      distortionType == params.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet");
  }
  int num3dMatches = 0;
  // first frame? (did do addStates before, so 1 frame minimum in estimator)
  if (estimator.numFrames() > 1) {
    int requiredMatches = 5;
//    double uncertainMatchFraction = 0;
//    bool rotationOnly = false;

    okvis::TimerSwitchable matchToLastFrameTimer("2.4.2 matchToLastFrame");
    bool framewiseRotationOnly = false;
#define DISTORTION_MODEL_CASE(camera_geometry_t)                               \
  num3dMatches =                                                               \
      matchToLastFrame<okvis::VioKeyframeWindowMatchingAlgorithm<camera_geometry_t>>( \
          estimator, params, nframes->id(), framewiseRotationOnly, false);

    switch (distortionType) { DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES }

#undef DISTORTION_MODEL_CASE

    matchToLastFrameTimer.stop();

      if (!isInitialized_) {
        if (!framewiseRotationOnly) {
          isInitialized_ = true;
          LOG(INFO) << "Initialized frontend from framewise matching.";
        }
      }
      if (num3dMatches <= requiredMatches) {
        LOG(WARNING) << "Tracking last frame failure. Number of 3d2d-matches: " << num3dMatches;
      }
      // At the moment, landmarks that match with the last frame are initialised.
      *asKeyframe = *asKeyframe || doWeNeedANewKeyframePosterior(estimator, nframes);

  } else {
    *asKeyframe = true;  // first frame needs to be keyframe
    LOG(INFO) << "First frame " << nframes->timestamp() << " as keyframe";
  }

  // do stereo match to get new landmarks
  if (frontendOptions_.stereoMatchWithEpipolarCheck) {
    matchStereoWithEpipolarCheckSwitch(distortionType, estimator, nframes);
  } else {
    matchStereoSwitch(distortionType, estimator, nframes);
  }

  ++numNFrames_;
  numKeyframes_ += (*asKeyframe ? 1 : 0);
  return true;
}

bool FramewiseDescriptorFrontend::doWeNeedANewKeyframePosterior(
    const okvis::EstimatorBase& estimator,
    std::shared_ptr<okvis::MultiFrame> currentFrame) {
  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return true;
  }

  if (!isInitialized_)
    return false;

  uint64_t latestKeyframeId = estimator.currentKeyframeId();
  std::shared_ptr<okvis::MultiFrame> latestKeyframePtr =
      estimator.multiFrame(latestKeyframeId);

  double overlap = 0.0;
  double ratio = 0.0;

  // go through all the frames and try to match the initialized keypoints
  for (size_t im = 0; im < currentFrame->numFrames(); ++im) {
    okvis::Matches matches;
    opengv::findMatches(estimator, latestKeyframePtr, im, currentFrame, im,
                        &matches);
    double overlapArea;
    double matchingRatio;
    opengv::computeMatchStats(matches, currentFrame, im, &overlapArea,
                              &matchingRatio);
    overlap = std::max(overlapArea, overlap);
    ratio = std::max(matchingRatio, ratio);
  }

  // take a decision
  if (overlap > frontendOptions_.keyframeInsertionOverlapThreshold
      && ratio > frontendOptions_.keyframeInsertionMatchingRatioThreshold)
    return false;
  else
    return true;
}

// Match a new multiframe to the last frame.
template <class MATCHING_ALGORITHM>
int FramewiseDescriptorFrontend::matchToLastFrame(
    okvis::EstimatorBase& estimator,
    const okvis::VioParameters& params, const uint64_t currentFrameId,
    bool& rotationOnly,
    bool usePoseUncertainty, bool removeOutliers) {
  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return 0;
  }

  uint64_t lastFrameId = estimator.frameIdByAge(1);


  int retCtr = 0;

  for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
    MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                         MATCHING_ALGORITHM::Match3D2D,
                                         briskMatchingThreshold_,
                                         usePoseUncertainty);
    matchingAlgorithm.setFrames(lastFrameId, currentFrameId, im, im);

    // match 3D-2D
    matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
    retCtr += matchingAlgorithm.numMatches();
  }

  runRansac3d2d(estimator, params.nCameraSystem,
                estimator.multiFrame(currentFrameId), removeOutliers);

  for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
    MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                         MATCHING_ALGORITHM::Match2D2D,
                                         briskMatchingThreshold_,
                                         usePoseUncertainty);
    matchingAlgorithm.setFrames(lastFrameId, currentFrameId, im, im);

    // match 2D-2D for initialization of new (mono-)correspondences
    matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
    retCtr += matchingAlgorithm.numMatches();
  }

  // remove outliers
  rotationOnly = false;
  if (!isInitialized_) {
    runRansac2d2d(estimator, params, currentFrameId, lastFrameId, true,
                  removeOutliers, rotationOnly);
  }

  return retCtr;
}

}  // namespace swift_vio
