/**
 * @file FeatureTrackingFrontend.cpp
 * @brief Source file for the FeatureTrackingFrontend class.
 * @author 
 */

#include <swift_vio/FeatureTrackingFrontend.hpp>

#include <brisk/brisk.h>

#include <glog/logging.h>

#include <okvis/ceres/ImuError.hpp>
#include <okvis/IdProvider.hpp>

// cameras and distortions
#include <okvis/CameraModelSwitch.hpp>
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/cameras/FovDistortion.hpp>
#include <okvis/timing/Timers.hpp>

// Kneip RANSAC
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp>
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>

#include "feature_tracker/Feature2d2dMatchingAlgorithm.hpp"
#include "feature_tracker/StereoMatchingAlgorithm.hpp"
#include <feature_tracker/SimpleFrameRelativeAdapter.h>
#include <feature_tracker/SimpleFrameRelativePoseSacProblem.h>
#include <feature_tracker/SimpleFrameRotationOnlySacProblem.h>

DEFINE_bool(noncentral_relative_pose, true, "Use noncentral relative pose ransac for a camera rig?");

namespace swift_vio {
void mergeTwoFeatureTracks(
    FeatureTrackMap *featureTracks,
    std::unordered_map<uint64_t, BareMultiFrame> *multiFrame2LandmarkIdMap,
    uint64_t trackIdSource,
    uint64_t trackIdTarget) {
  // move every obsrevation of source to target
  auto source = featureTracks->find(trackIdSource);
  auto target = featureTracks->find(trackIdTarget);
  if (source != featureTracks->end() && target != featureTracks->end()) {
    for (const auto observation : source->second.observations) {
      target->second.observations.emplace(observation);
      auto framePtr = multiFrame2LandmarkIdMap->find(observation.first.frameId);
      OKVIS_ASSERT_TRUE(
          std::runtime_error, framePtr != multiFrame2LandmarkIdMap->end(),
          "An observation in feature track "
              << trackIdSource << " corresponds to a non-exist NFrame!");
      framePtr->second.setLandmarkId(observation.first.cameraIndex, observation.first.keypointIndex, trackIdTarget);
    }
  }
}

void findMatches(const std::vector<uint64_t> &landmarkIdListA,
                 const std::vector<uint64_t> &landmarkIdListB,
                 okvis::Matches *matches) {
  size_t numKeypointsA = landmarkIdListA.size();
  size_t numKeypointsB = landmarkIdListB.size();
  matches->reserve(std::min(numKeypointsA, numKeypointsB));
  std::unordered_map<uint64_t, size_t> idMap;
  for (size_t k = 0; k < numKeypointsA; ++k) {
    uint64_t lmId = landmarkIdListA[k];
    if (lmId != 0)
      idMap.emplace(lmId, k);
  }

  for (size_t k = 0; k < numKeypointsB; ++k) {
    uint64_t lmId = landmarkIdListB[k];
    if (lmId == 0)
      continue;
    std::unordered_map<uint64_t, size_t>::const_iterator it = idMap.find(lmId);
    if (it != idMap.end()) {
      matches->emplace_back(it->second, k, 0.0);
    }
  }
}

FeatureTrackingFrontend::FeatureTrackingFrontend(
    size_t numCameras, const swift_vio::FrontendOptions &frontendOptions)
    : CameraFrontendBase(numCameras), frontendOptions_(frontendOptions),
      matcher_(std::unique_ptr<okvis::DenseMatcher>(
          new okvis::DenseMatcher(frontendOptions.numThreads))), rotationOnly_(true) {
  // create mutexes for feature detectors and descriptor extractors
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectorMutexes_.push_back(
        std::unique_ptr<std::mutex>(new std::mutex()));
  }
  initialiseFeatureDetectors();
}

FeatureTrackingFrontend::~FeatureTrackingFrontend() {
  LOG(INFO) << "FeatureTrackingFrontend associated " << numNFrames_
            << " NFrames, and selected " << numKeyframes_ << " keyframes.";
}

void FeatureTrackingFrontend::initialiseFeatureDetectors() {
  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->lock();
  }
  featureDetectors_.clear();
  descriptorExtractors_.clear();
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectors_.push_back(
        std::shared_ptr<cv::FeatureDetector>(
#ifdef __ARM_NEON__
            new cv::GridAdaptedFeatureDetector(
            new cv::FastFeatureDetector(frontendOptions_.detectionThreshold),
                frontendOptions_.maxNoKeypoints, 7, 4)));
#else
            new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
                frontendOptions_.detectionThreshold, frontendOptions_.detectionOctaves,
                frontendOptions_.brisk.detectionAbsoluteThreshold,
                frontendOptions_.maxNoKeypoints)));
#endif
    // TODO(jhuai): use FREAK descriptor, see maplab. What's the time cost of feature detection versus feature description?
    descriptorExtractors_.push_back(
        std::shared_ptr<cv::DescriptorExtractor>(
            new brisk::BriskDescriptorExtractor(
                frontendOptions_.brisk.descriptionRotationInvariance,
                frontendOptions_.brisk.descriptionScaleInvariance)));
  }

  distanceCallback_ = std::bind(&aslBriskDistance, std::placeholders::_1, std::placeholders::_2);

  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->unlock();
  }
}

bool FeatureTrackingFrontend::detectAndDescribe(size_t cameraIndex,
                                 std::shared_ptr<MultiFrame> frameOut) {
  OKVIS_ASSERT_TRUE_DBG(Exception, cameraIndex < numCameras_, "Camera index exceeds number of cameras.");
  std::lock_guard<std::mutex> lock(*featureDetectorMutexes_[cameraIndex]);
  featureDetectors_[cameraIndex]->detect(frameOut->image(cameraIndex),
                                         frameOut->keypoints(cameraIndex));
  // Comment the below 3 lines if the detector provides an informative angle or the angle is obtained from say the gravity direction.
  // BRISK will use the angle in computing a descriptor if it is not -1.
  // see comments about this in maplab/algorithms/feature-tracking/src/feature-detection-extraction.cc.
  for (cv::KeyPoint &kp : frameOut->keypoints(cameraIndex)) {
    kp.angle = -1.0f;
  }
  descriptorExtractors_[cameraIndex]->compute(frameOut->image(cameraIndex),
                                              frameOut->keypoints(cameraIndex),
                                              frameOut->descriptors(cameraIndex));
  return true;
}

void FeatureTrackingFrontend::selectKeyframesWith3dPointsForMatching(
    std::shared_ptr<const VisualSubmap> submap,
    std::vector<ConstMultiFramePtr> *keyframes) const {
  if (submap) {
  for (auto it = submap->multiFramePtrMap.begin(); it != submap->multiFramePtrMap.end(); it++) {
    keyframes->push_back(it->second);
    if (keyframes->size() >= frontendOptions_.numOldKeyframesToMatch) {
        break;
    }
  }
  }
}

void FeatureTrackingFrontend::selectKeyframesForMatching(
    std::vector<ConstMultiFramePtr> *keyframes, size_t num) const {
  for (auto rit = multiFramePtrMap_.rbegin(); rit != multiFramePtrMap_.rend(); rit++) {
    if (rit->second->isKeyframe()) {
      keyframes->push_back(rit->second);
    }
    if (keyframes->size() >= num) {
      break;
    }
  }
}

bool FeatureTrackingFrontend::dataAssociation(
    MultiFramePtr currentNFrame,
    std::shared_ptr<const VisualSubmap> submap,
    VisualMatcherOutput *output) {
  // Prepare data structures for this new NFrame.
  multiFramePtrMap_.emplace(currentNFrame->id(), currentNFrame);
  auto nframePtr = multiFrame2LandmarkIdMap_.find(currentNFrame->id());
  CHECK(nframePtr == multiFrame2LandmarkIdMap_.end());

  auto result = multiFrame2LandmarkIdMap_.emplace(
      std::piecewise_construct, std::forward_as_tuple(currentNFrame->id()),
      std::forward_as_tuple(currentNFrame->timestamp(), currentNFrame->id(),
                            numCameras_));
  nframePtr = result.first;
  for (size_t cameraIndex = 0; cameraIndex < currentNFrame->numFrames();
       ++cameraIndex) {
    nframePtr->second.setTimestamp(cameraIndex,
                                   currentNFrame->timestamp(cameraIndex));
    nframePtr->second.landmarkIdsMutable(cameraIndex)
        .resize(currentNFrame->keypoints(cameraIndex).size(), 0u);
  }

  // select keyframes and frames for feature matching
  std::vector<ConstMultiFramePtr> dbKeyframes;
  selectKeyframesWith3dPointsForMatching(submap, &dbKeyframes);

  std::vector<ConstMultiFramePtr> keyframes;
  size_t num = 0;
  if (frontendOptions_.numKeyframesToMatch > dbKeyframes.size()) {
    num = frontendOptions_.numKeyframesToMatch - dbKeyframes.size();
  }
  selectKeyframesForMatching(&keyframes, num);

  output_ = output;

  matchToKeyframesWith3dPoints(submap, dbKeyframes, currentNFrame);

  matchToKeyframes(keyframes, currentNFrame);

  // keyframe decision, at the moment only landmarks that match with keyframe or submap are recorded.
  bool asKeyframe = frontendOptions_.allAreKeyframes || doWeNeedANewKeyframe(currentNFrame);
  currentNFrame->setKeyframe(asKeyframe);
  multiFramePtrMap_.at(currentNFrame->id())->setKeyframe(asKeyframe);

  matchToLastFrame(currentNFrame);

  matchStereoWithEpipolarCheck(currentNFrame);

  memoryManagement(currentNFrame);

  output_->currentNFrame.reset(new MultiFrame(*currentNFrame));

  ++numNFrames_;
  numKeyframes_ += (asKeyframe ? 1 : 0);
  return true;
}

int FeatureTrackingFrontend::matchToKeyframesWith3dPoints(
    std::shared_ptr<const VisualSubmap> /*submap*/,
    const std::vector<ConstMultiFramePtr> &/*dbKeyframes*/,
    ConstMultiFramePtr currentNFrame) {
  okvis::TimerSwitchable matchKeyframesTimer("2.4.1 matchToKeyframesWith3dPoints");
  // for landmarks in the submap,
  // perform 3d-2d matching to the current Nframe
  // run ransacPnP which works on all frame in current NFrame at once to remove outliers, and solve the relative pose.
  runRansacPnP(currentNFrame);
  // save the feature matches into FeatureTracks, also record the landmark ids for keypoints of the current NFrame.

  // for each of say two keyframes in the submap
  // perform 2d-2d matching to the current Nframe with epipolar check.
  // save the feature matches into FeatureTracks, also record the landmark ids for keypoints of the current NFrame.

  matchKeyframesTimer.stop();
  return 0;
}

bool decideOnRotationOnly(const FrameRansacResult &relativePoseResult, const FrameRansacResult &rotationOnlyResult) {
  bool rotationOnly = true;
  if (rotationOnlyResult.inlierRatio > relativePoseResult.inlierRatio || rotationOnlyResult.inlierRatio > 0.8) {
    rotationOnly = true;
  } else {
    if (relativePoseResult.inliers.size() > 10) {
      rotationOnly = false;
    }
  }
  return rotationOnly;
}

int FeatureTrackingFrontend::matchToKeyframes(
    const std::vector<ConstMultiFramePtr> &keyframes,
    ConstMultiFramePtr currentNFrame) {
  if (numNFrames_ == 0) { // This is the first frame.
    output_->closestKeyframe = currentNFrame; // This frame will be a keyframe anyway.
    output_->rotationOnly = true;
    return 0;
  }
  CHECK_GE(keyframes.size(), 1u);
  okvis::TimerSwitchable matchKeyframesTimer("2.4.2 matchToKeyframes");
  int retCtr = 0;
  for (size_t kfcounter = 0; kfcounter < keyframes.size(); ++kfcounter) {
    size_t numCams = nCameraSystem_->numCameras();
    std::vector<std::shared_ptr<okvis::Matches>> multiMatches(numCams);
    std::vector<size_t> multiNumNewMatches(numCams);
    for (size_t im = 0; im < numCams; ++im) {
      Feature2d2dMatchingAlgorithm matchingAlgorithm(
          keyframes[kfcounter], currentNFrame, im, im,
          distanceCallback_,
          frontendOptions_.brisk.matchingThreshold);

      // mask those matches in im of the current NFrame.
      matchingAlgorithm.setSkipBListFrom(multiFrame2LandmarkIdMap_.at(currentNFrame->id()).landmarkIds(im));

      matcher_->match<Feature2d2dMatchingAlgorithm>(matchingAlgorithm);

      const okvis::Matches &matches = matchingAlgorithm.getMatches();

      // collect more matches between the currentNFrame and the ealier keyframe by
      // considering the common feature tracks in both NFrames, which were masked out just now.
      std::shared_ptr<okvis::Matches> existingMatches(new okvis::Matches());
      findMatches(multiFrame2LandmarkIdMap_.at(keyframes[kfcounter]->id()).landmarkIds(im),
                  multiFrame2LandmarkIdMap_.at(currentNFrame->id()).landmarkIds(im),
                  existingMatches.get());
      existingMatches->insert(existingMatches->end(), matches.begin(), matches.end());
      multiMatches.at(im) = existingMatches;
      multiNumNewMatches.at(im) = matches.size();
    }

    // Empirically, RANSAC with NoncentralRelativePoseSacProblem time cost is larger
    // than the combined time cost of ransac for each image pair.
    if (nCameraSystem_->numCameras() >= 2 && FLAGS_noncentral_relative_pose) {
      Eigen::AlignedVector<FrameRansacResult> results;
      runRansacNoncentralRelativePose(keyframes[kfcounter], currentNFrame,
                                      multiMatches, &results);
      for (size_t im = 0; im < nCameraSystem_->numCameras(); ++im) {
        int numNewObservations = updateFeatureTracks(
            *multiMatches.at(im), results[im],
            (int)(multiMatches.at(im)->size() - multiNumNewMatches.at(im)));
        retCtr += numNewObservations;
      }
    } else {
      for (size_t im = 0; im < nCameraSystem_->numCameras(); ++im) {
        FrameRansacResult resultF; // it contains match status
        FrameRansacResult resultH;
        FrameRansacResult *keepResult;
        if (kfcounter == 0 && rotationOnly_) {
          runRansacFundamentalMat(currentNFrame, keyframes[kfcounter], im,
                                  *multiMatches.at(im), &resultF);
          runRansacRotationOnly(currentNFrame, keyframes[kfcounter], im,
                                *multiMatches.at(im), &resultH);
          keepResult = resultF.inliers.size() > resultH.inliers.size()
                           ? &resultF
                           : &resultH;
          // Use relative pose for constraining IMU prediction in the estimator
          // at the beginning.
          output_->addRelativePoseInfo(*keepResult);
          rotationOnly_ = decideOnRotationOnly(resultF, resultH);
        } else {
          runRansacFundamentalMat(currentNFrame, keyframes[kfcounter], im,
                                  *multiMatches.at(im), &resultF);
          keepResult = &resultF;
        }

        int numNewObservations = updateFeatureTracks(
            *multiMatches.at(im), *keepResult,
            (int)(multiMatches.at(im)->size() - multiNumNewMatches.at(im)));
        retCtr += numNewObservations;
      }
    }
  }
  // Set closestKeyframe to keyframes.back() to see the feature matches to the
  // oldest keyframe. As RANSAC is to no avail with few matches, you may
  // see some outlier matches.
  output_->closestKeyframe = keyframes.front();

  output_->rotationOnly = rotationOnly_;
  matchKeyframesTimer.stop();
  return retCtr;
}

int FeatureTrackingFrontend::matchToLastFrame(ConstMultiFramePtr currentNFrame) {
  if (numNFrames_ == 0) {  // This is the first frame.
    return 0;
  }

  int retCtr = 0;
  OKVIS_ASSERT_EQ(Exception, multiFramePtrMap_.rbegin()->second->id(), currentNFrame->id(),
                  "The current NFrame should have been included in the multiframe map!");
  ConstMultiFramePtr lastNFrame = (++multiFramePtrMap_.rbegin())->second;
  if (lastNFrame->isKeyframe()) {
    // already done.
    return 0;
  }

  okvis::TimerSwitchable matchToLastFrameTimer("2.4.3 matchToLastFrame");
  size_t numCams = nCameraSystem_->numCameras();
  std::vector<std::shared_ptr<okvis::Matches>> multiMatches(numCams);
  std::vector<size_t> multiNumNewMatches(numCams);
  for (size_t im = 0u; im < nCameraSystem_->numCameras(); ++im) {
    Feature2d2dMatchingAlgorithm matchingAlgorithm(
        lastNFrame, currentNFrame, im, im, distanceCallback_,
        frontendOptions_.brisk.matchingThreshold);

    // mask those matches in im of the current NFrame.
    matchingAlgorithm.setSkipBListFrom(multiFrame2LandmarkIdMap_.at(currentNFrame->id()).landmarkIds(im));

    matcher_->match<Feature2d2dMatchingAlgorithm>(matchingAlgorithm);
    const okvis::Matches &matches = matchingAlgorithm.getMatches();

    // collect more matches between the currentNFrame and the last NFrame by
    // considering the common feature tracks in both NFrames, which were masked out just now.
    std::shared_ptr<okvis::Matches> existingMatches(new okvis::Matches());
    findMatches(multiFrame2LandmarkIdMap_.at(lastNFrame->id()).landmarkIds(im),
                multiFrame2LandmarkIdMap_.at(currentNFrame->id()).landmarkIds(im),
                existingMatches.get());
    existingMatches->insert(existingMatches->end(), matches.begin(),
                           matches.end());
    multiMatches.at(im) = existingMatches;
    multiNumNewMatches.at(im) = matches.size();
  }

  if (nCameraSystem_->numCameras() >= 2 && FLAGS_noncentral_relative_pose) {
    Eigen::AlignedVector<FrameRansacResult> results;
    runRansacNoncentralRelativePose(lastNFrame, currentNFrame,
                                    multiMatches, &results);
    for (size_t im = 0; im < nCameraSystem_->numCameras(); ++im) {
      int numNewObservations = updateFeatureTracks(
          *multiMatches.at(im), results[im],
          (int)(multiMatches.at(im)->size() - multiNumNewMatches.at(im)));
      retCtr += numNewObservations;
    }
  } else {
    for (size_t im = 0u; im < nCameraSystem_->numCameras(); ++im) {
      FrameRansacResult resultF;
      runRansacFundamentalMat(currentNFrame, lastNFrame, im,
                              *multiMatches.at(im), &resultF);
      int numNewObservations = updateFeatureTracks(
          *multiMatches.at(im), resultF,
          (int)(multiMatches.at(im)->size() - multiNumNewMatches.at(im)));
      retCtr += numNewObservations;
    }
  }
  matchToLastFrameTimer.stop();
  return retCtr;
}

int FeatureTrackingFrontend::matchStereoWithEpipolarCheck(
    ConstMultiFramePtr multiFrame) {
  okvis::TimerSwitchable matchStereoTimer("2.4.4 matchStereo");
  const size_t camNumber = multiFrame->numFrames();
  int retCtr = 0;
  for (size_t im0 = 0; im0 < camNumber; im0++) {
    for (size_t im1 = im0 + 1; im1 < camNumber; im1++) {
      if(!nCameraSystem_->hasOverlap(im0, im1)){
        continue;
      }
      StereoMatchingAlgorithm matchingAlgorithm(
          multiFrame, nCameraSystem_, im0, im1, distanceCallback_,
          frontendOptions_.brisk.matchingThreshold,
          frontendOptions_.epipolarDistanceThreshold);

      matcher_->match<StereoMatchingAlgorithm>(matchingAlgorithm);
      const okvis::Matches &matches = matchingAlgorithm.getMatches();
      int numNewObservations = updateAndMergeFeatureTracks(matches, multiFrame, im0, im1);
      if (rotationOnly_ && numNewObservations > 15) {
        LOG(INFO) << "Motion can be estimated with spatial stereo matches at the start.";
        rotationOnly_ = false;
      }
      retCtr += numNewObservations;
    }
  }
  matchStereoTimer.stop();
  return retCtr;
}

int FeatureTrackingFrontend::updateFeatureTracks(
    const okvis::Matches &matches, const FrameRansacResult &result, int numExistingMatches) {
  // A refers to an earlier Nframe, B always refers to the current Nframe.
  uint64_t frameA = result.relPoseInfo.cam1.frameId;
  uint64_t frameB = result.relPoseInfo.cam2.frameId;
  size_t camIdA = result.relPoseInfo.cam1.cameraIndex;
  size_t camIdB = result.relPoseInfo.cam2.cameraIndex;

  ConstMultiFramePtr frameAPtr = multiFramePtrMap_.at(frameA);
  ConstMultiFramePtr frameBPtr = multiFramePtrMap_.at(frameB);
  std::vector<uint64_t> &frameALandmarks =
      multiFrame2LandmarkIdMap_.at(frameA).landmarkIdsMutable(camIdA);
  std::vector<uint64_t> &frameBLandmarks =
      multiFrame2LandmarkIdMap_.at(frameB).landmarkIdsMutable(camIdB);
  int numNewObservations = 0;
  for (auto i : result.inliers) {
    if (i < numExistingMatches)  // do not consider matches found earlier.
        continue;
    const okvis::Match &match = matches[i];
    uint64_t trackIdA = frameALandmarks[match.idxA];
    uint64_t trackIdB = frameBLandmarks[match.idxB];

    if (trackIdA == 0) {
      if (trackIdB == 0) {
        uint64_t id = okvis::IdProvider::instance().newId();
        auto res = output_->featureTracks.emplace(id, FeatureTrack(id));
        Eigen::Vector2d uv, veluv;
        float size;
        frameAPtr->getKeypoint(camIdA, match.idxA, uv);
        frameAPtr->getKeypointSize(camIdA, match.idxA, size);
        veluv.setZero();
        res.first->second.observations.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(frameA, camIdA, match.idxA),
            std::forward_as_tuple(uv, size, veluv));

        frameBPtr->getKeypoint(camIdB, match.idxB, uv);
        frameBPtr->getKeypointSize(camIdB, match.idxB, size);
        veluv.setZero();
        res.first->second.observations.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(frameB, camIdB, match.idxB),
            std::forward_as_tuple(uv, size, veluv));

        frameALandmarks[match.idxA] = id;
        frameBLandmarks[match.idxB] = id;
        numNewObservations += 2;
      } else { // trackIdB != 0
        auto res = output_->featureTracks.find(trackIdB);
        OKVIS_ASSERT_TRUE(Exception, res != output_->featureTracks.end(),
                          "Landmark in urrent frame should have been added to "
                          "the feature tracks!");

        Eigen::Vector2d uv, veluv;
        float size;
        frameAPtr->getKeypoint(camIdA, match.idxA, uv);
        frameAPtr->getKeypointSize(camIdA, match.idxA, size);
        veluv.setZero();
        res->second.observations.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(frameA, camIdA, match.idxA),
            std::forward_as_tuple(uv, size, veluv));

        frameALandmarks[match.idxA] = trackIdB;
        ++numNewObservations;
      }
    } else { // trackIdA != 0
      if (trackIdB == 0) {
        auto res =
            output_->featureTracks.emplace(trackIdA, FeatureTrack(trackIdA));
        Eigen::AlignedMap<okvis::KeypointIdentifier, FluidObservation>
            &obsList = res.first->second.observations;
        if (res.second == false) {
          auto iter = std::find_if(obsList.begin(), obsList.end(),
                                   IsObservedInFrame(frameB, camIdB));
          if (iter != obsList.end()) {
            VLOG(2) << "Two keypoints in current frame is matched to the "
                            "same landmark!";
            continue;
          }
        }

        Eigen::Vector2d uv, veluv;
        float size;
        frameBPtr->getKeypoint(camIdB, match.idxB, uv);
        frameBPtr->getKeypointSize(camIdB, match.idxB, size);
        veluv.setZero();
        obsList.emplace(std::piecewise_construct,
                        std::forward_as_tuple(frameB, camIdB, match.idxB),
                        std::forward_as_tuple(uv, size, veluv));

        frameBLandmarks[match.idxB] = trackIdA;
        ++numNewObservations;
      } else { // trackIdB != 0
        if (trackIdA != trackIdB) {
          LOG(WARNING) << "A recent match is discarded for linking two feature "
                          "tracks in respect of the earlier matches!";
        } else {
          // This observation has been recorded earlier for the feature track.
        }
      }
    }
  }
  return numNewObservations;
}

int FeatureTrackingFrontend::updateAndMergeFeatureTracks(
    const okvis::Matches &stereoMatches, ConstMultiFramePtr framePtr, size_t camIdA, size_t camIdB) {
  uint64_t frameId = framePtr->id();
  std::vector<uint64_t> &frameALandmarks = multiFrame2LandmarkIdMap_.at(frameId).landmarkIdsMutable(camIdA);
  std::vector<uint64_t> &frameBLandmarks = multiFrame2LandmarkIdMap_.at(frameId).landmarkIdsMutable(camIdB);
  int numNewObservations = 0;
  for (const okvis::Match &match : stereoMatches) {
    uint64_t trackIdA = frameALandmarks[match.idxA];
    uint64_t trackIdB = frameBLandmarks[match.idxB];
    if (trackIdA == 0) {
      if (trackIdB == 0) {
         uint64_t id = okvis::IdProvider::instance().newId();
         auto res = output_->featureTracks.emplace(id, FeatureTrack(id));
         Eigen::Vector2d uv, veluv;
         float size;
         framePtr->getKeypoint(camIdA, match.idxA, uv);
         framePtr->getKeypointSize(camIdA, match.idxA, size);
         veluv.setZero();
         res.first->second.observations.emplace(
             std::piecewise_construct,
             std::forward_as_tuple(frameId, camIdA, match.idxA),
             std::forward_as_tuple(uv, size, veluv));

         framePtr->getKeypoint(camIdB, match.idxB, uv);
         framePtr->getKeypointSize(camIdB, match.idxB, size);
         veluv.setZero();
         res.first->second.observations.emplace(
             std::piecewise_construct,
             std::forward_as_tuple(frameId, camIdB, match.idxB),
             std::forward_as_tuple(uv, size, veluv));

         frameALandmarks[match.idxA] = id;
         frameBLandmarks[match.idxB] = id;
         numNewObservations += 2;
      } else { // trackIdB != 0
        auto res = output_->featureTracks.find(trackIdB);
        OKVIS_ASSERT_TRUE(
            Exception, res != output_->featureTracks.end(),
            "A landmark observed in current NFrame image "
                << camIdB << " should have been added to the feature tracks!");

        Eigen::Vector2d uv, veluv;
        float size;
        framePtr->getKeypoint(camIdA, match.idxA, uv);
        framePtr->getKeypointSize(camIdA, match.idxA, size);
        veluv.setZero();
        res->second.observations.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(frameId, camIdA, match.idxA),
            std::forward_as_tuple(uv, size, veluv));

        frameALandmarks[match.idxA] = trackIdB;
        ++numNewObservations;
      }
    } else { // trackIdA != 0
      if (trackIdB == 0) {
        auto res = output_->featureTracks.find(trackIdA);
          OKVIS_ASSERT_TRUE(
              Exception, res != output_->featureTracks.end(),
              "A landmark observed in current NFrame image "
                  << camIdA << " should have been added to the feature tracks!");
          Eigen::Vector2d uv, veluv;
          float size;
          framePtr->getKeypoint(camIdB, match.idxB, uv);
          framePtr->getKeypointSize(camIdB, match.idxB, size);
          veluv.setZero();
          res->second.observations.emplace(
              std::piecewise_construct,
              std::forward_as_tuple(frameId, camIdB, match.idxB),
              std::forward_as_tuple(uv, size, veluv));

          frameBLandmarks[match.idxB] = trackIdA;
          ++numNewObservations;
      } else { // trackIdB != 0
        if (trackIdA != trackIdB) {
          mergeTwoFeatureTracks(&(output_->featureTracks), &multiFrame2LandmarkIdMap_,
                             std::max(trackIdA, trackIdB),
                             std::min(trackIdA, trackIdB));
        } else {
          // the two observations should have been recorded earlier.
        }
      }
    }
  }
  return numNewObservations;
}

void FeatureTrackingFrontend::memoryManagement(
    ConstMultiFramePtr /*currentNFrame*/) {
  if (multiFramePtrMap_.size() > 5) {
    // remove ordinary frames except for the last one.
    // remove keyframes exceeding frontendOptions_.numKeyframesToMatch
    size_t numKeyframes = 0;
    std::vector<uint64_t> toRemoveNFrameIds;
    toRemoveNFrameIds.reserve(2);
    auto secondToLastIt = ++multiFramePtrMap_.rbegin();
    for (auto rit = secondToLastIt; rit != multiFramePtrMap_.rend(); ++rit) {
      bool toRemove = false;
      if (rit->second->isKeyframe()) {
        ++numKeyframes;
        if (numKeyframes > frontendOptions_.numKeyframesToMatch) {
          toRemove = true;
        }
      } else {
        toRemove = true;
      }
      if (toRemove) {
         toRemoveNFrameIds.push_back(rit->first);
      }
    }
    for (const auto &id : toRemoveNFrameIds) {
      multiFramePtrMap_.erase(id);
      multiFrame2LandmarkIdMap_.erase(id);
    }
  }
}

uint64_t FeatureTrackingFrontend::oldestNFrameId() const {
  return multiFramePtrMap_.begin()->first;
}

bool FeatureTrackingFrontend::doWeNeedANewKeyframe(
    ConstMultiFramePtr currentFrame) const {
  if (numNFrames_ == 0) {
    // The first frame should be a new keyframe.
    return true;
  }

  // check the overlap between matched keypoints and all keypoints of the current NFrame.
  double overlap = 0.0;
  double ratio = 0.0;

  const BareMultiFrame &nframeLandmarkIds = multiFrame2LandmarkIdMap_.at(currentFrame->id());
  for (size_t im = 0; im < currentFrame->numFrames(); ++im) {
    // get the hull of all keypoints in current frame
    std::vector<cv::Point2f> frameBPoints, frameBHull;
    std::vector<cv::Point2f> frameBMatches, frameBMatchesHull;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > frameBLandmarks;
    const std::vector<uint64_t> &frameLandmarkIds = nframeLandmarkIds.landmarkIds(im);
    const size_t numB = currentFrame->numKeypoints(im);
    frameBPoints.reserve(numB);
    frameBLandmarks.reserve(numB);
    Eigen::Vector2d keypoint;
    for (size_t k = 0; k < numB; ++k) {
      currentFrame->getKeypoint(im, k, keypoint);
      frameBPoints.push_back(cv::Point2f(keypoint[0], keypoint[1]));
      if (frameLandmarkIds.at(k) != 0) {
        frameBMatches.push_back(cv::Point2f(keypoint[0], keypoint[1]));
      }
    }

    if (frameBPoints.size() < 3 || frameBMatches.size() < 3)
      continue;
    cv::convexHull(frameBPoints, frameBHull);
    cv::convexHull(frameBMatches, frameBMatchesHull);

    double frameBArea = cv::contourArea(frameBHull);
    double frameBMatchesArea = cv::contourArea(frameBMatchesHull);

    double overlapArea = frameBMatchesArea / frameBArea;
    // matching ratio inside overlap area: count
    int pointsInFrameBMatchesArea = 0;
    if (frameBMatchesHull.size() > 2) {
      for (size_t k = 0; k < frameBPoints.size(); ++k) {
        if (cv::pointPolygonTest(frameBMatchesHull, frameBPoints[k], false)
            > 0) {
          pointsInFrameBMatchesArea++;
        }
      }
    }
    double matchingRatio = double(frameBMatches.size())
        / double(pointsInFrameBMatchesArea);

    overlap = std::max(overlapArea, overlap);
    ratio = std::max(matchingRatio, ratio);
  }

  if (overlap > frontendOptions_.keyframeInsertionOverlapThreshold
      && ratio > frontendOptions_.keyframeInsertionMatchingRatioThreshold)
    return false;
  else
    return true;
}

int FeatureTrackingFrontend::runRansacPnP(ConstMultiFramePtr /*currentFrame*/) {
  int numInliers = 0;
//  bool removeOutliers = true;
  // TODO(jhuai): develop an adapter that does not depend on an estimator.
  // absolute pose adapter for Kneip toolchain
//  opengv::absolute_pose::FrameNoncentralAbsoluteAdapter adapter(estimator,
//                                                                nCameraSystem,
//                                                                currentFrame);

//  size_t numCorrespondences = adapter.getNumberCorrespondences();
//  if (numCorrespondences < 5)
//    return numCorrespondences;

//  // create a RelativePoseSac problem and RANSAC
//  opengv::sac::Ransac<
//      opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> ransac;
//  std::shared_ptr<
//      opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> absposeproblem_ptr(
//      new opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem(
//          adapter,
//          opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem::Algorithm::GP3P));
//  ransac.sac_model_ = absposeproblem_ptr;
//  ransac.threshold_ = 9;
//  ransac.max_iterations_ = 50;
//  // initial guess not needed...
//  // run the ransac
//  ransac.computeModel(0);

//  // assign transformation
//  numInliers = ransac.inliers_.size();
//  if (numInliers >= 10) {

//    // kick out outliers:
//    std::vector<bool> inliers(numCorrespondences, false);
//    for (size_t k = 0; k < ransac.inliers_.size(); ++k) {
//      inliers.at(ransac.inliers_.at(k)) = true;
//    }

//    for (size_t k = 0; k < numCorrespondences; ++k) {
//      if (!inliers[k]) {
//        // get the landmark id:
//        size_t camIdx = adapter.camIndex(k);
//        size_t keypointIdx = adapter.keypointIndex(k);
//        uint64_t lmId = currentFrame->landmarkId(camIdx, keypointIdx);

//        // reset ID:
//        currentFrame->setLandmarkId(camIdx, keypointIdx, 0);

//        // remove observation
//        if (removeOutliers) {
//          estimator.removeObservation(lmId, currentFrame->id(), camIdx,
//                                      keypointIdx);
//        }
//      }
//    }
//  }
  return numInliers;
}

void getMulti2D2DCorrespondences(swift_vio::ConstMultiFramePtr frameAPtr,
                                 swift_vio::ConstMultiFramePtr frameBPtr,
                                 const std::vector<std::shared_ptr<okvis::Matches>> &matches,
                                 const swift_vio::CameraRig &nCameraSystem,
                                 opengv::bearingVectors_t *bearingVectors1,
                                 opengv::bearingVectors_t *bearingVectors2,
                                 std::vector<int> *camCorrespondences1, std::vector<int> *camCorrespondences2) {
  size_t numCams = nCameraSystem.numCameras();
  size_t totalMatch = 0;
  for (size_t im = 0; im < numCams; ++im) {
    totalMatch += matches.at(im)->size();
  }
  bearingVectors1->resize(totalMatch);
  bearingVectors2->resize(totalMatch);
  camCorrespondences1->resize(totalMatch);
  camCorrespondences2->resize(totalMatch);
  size_t mid = 0;
  for (size_t im = 0; im < numCams; ++im) {
    double fu1 = nCameraSystem.cameraGeometry(im)->focalLengthU();
    size_t numMatch = matches.at(im)->size();
    double fu2 = nCameraSystem.cameraGeometry(im)->focalLengthU();

    std::vector<double> sigmaAngles1, sigmaAngles2;
    sigmaAngles1.resize(numMatch);
    sigmaAngles2.resize(numMatch);

    const auto &immatches = *matches.at(im);
    for (size_t k = 0; k < immatches.size(); ++k, ++mid) {
      const size_t idx1 = immatches[k].idxA;
      const size_t idx2 = immatches[k].idxB;
      Eigen::Vector2d keypoint;
      double keypointStdDev;
      frameAPtr->getKeypoint(im, idx1, keypoint);
      frameAPtr->getKeypointSize(im, idx1, keypointStdDev);
      keypointStdDev = 0.8 * keypointStdDev / 12.0;
      sigmaAngles1[k] = sqrt(2) * keypointStdDev * keypointStdDev
          / (fu1 * fu1);

      nCameraSystem.cameraGeometry(im)->backProject(
        keypoint, &bearingVectors1->at(mid));
      bearingVectors1->at(mid).normalize();
      camCorrespondences1->at(mid) = im;

      frameBPtr->getKeypoint(im, idx2, keypoint);
      frameBPtr->getKeypointSize(im, idx2, keypointStdDev);
      keypointStdDev = 0.8 * keypointStdDev / 12.0;
      sigmaAngles2[k] = sqrt(2) * keypointStdDev * keypointStdDev
          / (fu2 * fu2);

      nCameraSystem.cameraGeometry(im)->backProject(
        keypoint, &bearingVectors2->at(mid));
      bearingVectors2->at(mid).normalize();
      camCorrespondences2->at(mid) = im;
    }
  }
}

void getCameraRelativePoses(const swift_vio::CameraRig &nCameraSystem,
                           opengv::translations_t *camOffsets,
                           opengv::rotations_t *camRotations) {
  size_t numCams = nCameraSystem.numCameras();
  camOffsets->resize(numCams);
  camRotations->resize(numCams);
  okvis::kinematics::Transformation T_C0B = nCameraSystem.T_SC(0)->inverse();
  camOffsets->at(0).setZero();
  camRotations->at(0).setIdentity();
  for (size_t im = 1; im < numCams; ++im) {
    okvis::kinematics::Transformation T_C0Ci = T_C0B * *(nCameraSystem.T_SC(im));
    camOffsets->at(im) = T_C0Ci.r();
    camRotations->at(im) = T_C0Ci.C();
  }
}

int FeatureTrackingFrontend::runRansacNoncentralRelativePose(
    ConstMultiFramePtr olderFrame,
    ConstMultiFramePtr currentFrame,
    const std::vector<std::shared_ptr<okvis::Matches>> &matches,
    Eigen::AlignedVector<FrameRansacResult> *results) {
  // https://laurentkneip.github.io/opengv/page_how_to_use.html#sec_ransac.
  // for better robustness, I prefer using generailized eigensolver for noncentralrelativeposesacproblem.
  // We give up MultiNoncentralRelativePoseSacProblem because it requires a minimum number of matches in each frame pair.
  opengv::bearingVectors_t bearingVectors1; // unit ray directions in camera i at pose 1, i = 0, N-1.
  opengv::bearingVectors_t bearingVectors2; // unit ray directions in camera i at pose 2
  std::vector<int> camCorrespondences1;
  std::vector<int> camCorrespondences2;
  getMulti2D2DCorrespondences(olderFrame, currentFrame, matches, *nCameraSystem_,
                              &bearingVectors1, &bearingVectors2,
                              &camCorrespondences1, &camCorrespondences2);

  opengv::translations_t camOffsets; // B_p_Ci
  opengv::rotations_t camRotations; // B_R_Ci
  getCameraRelativePoses(*nCameraSystem_, &camOffsets, &camRotations);

  opengv::relative_pose::NoncentralRelativeAdapter adapter(
      bearingVectors1,
      bearingVectors2, camCorrespondences1, camCorrespondences2,
      camOffsets,
      camRotations);

  results->resize(numCameras_);
  size_t numCorrespondences = adapter.getNumberCorrespondences();
  if (numCorrespondences < 10) {
    // won't generate meaningful results. let's hope the few
    // correspondences we have are all inliers!!
    // Note we have Mahalanobis gating or robust loss as the last defense layer.
    for (size_t camId = 0; camId < numCameras_; ++camId) {
      FrameRansacResult &result = results->at(camId);
      result.relPoseInfo.cam1.frameId = olderFrame->id();
      result.relPoseInfo.cam1.cameraIndex = camId;
      result.relPoseInfo.cam2.frameId = currentFrame->id();
      result.relPoseInfo.cam2.cameraIndex = camId;
      size_t numMatch = matches[camId]->size();
      result.numCorrespondences = numMatch;
      for (size_t i = 0; i < numMatch; ++i)
        result.inliers.push_back(i);
      result.inlierRatio = 1.0;
      result.relPoseInfo.status = RansacStatus::FewMatches;
    }
    return 0;
  }

  opengv::sac::Ransac<
      opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem> ransac;
  // The ransac result in model_coefficients is B1_T_B2.
  std::shared_ptr<
      opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem> relposeproblem_ptr(
      new opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem(
      adapter,
      opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem::GE));
  ransac.sac_model_ = relposeproblem_ptr;
  float deviation = 2.5; // in pixels along one axis
  float fu = nCameraSystem_->cameraGeometry(0)->focalLengthU();
  ransac.threshold_ = 1.0 - cos(atan2(sqrt(2.0f)*deviation, fu));
  ransac.max_iterations_ = 50;

  ransac.computeModel();

  std::vector<size_t> accumMatches(numCameras_ + 1);
  accumMatches[0] = 0;
  for (size_t i = 0; i < numCameras_; ++i) {
    accumMatches[i + 1] = accumMatches[i] + matches.at(i)->size();
  }
  CHECK_EQ(accumMatches.back(), camCorrespondences1.size());
  for (size_t i = 0; i < ransac.inliers_.size(); ++i) {
     size_t mid = ransac.inliers_[i];  // index of an inlier within bearingvectors
     size_t camId = camCorrespondences1[mid];
     results->at(camId).inliers.push_back(mid - accumMatches[camId]);
  }

  for (size_t camId = 0; camId < numCameras_; ++camId) {
    FrameRansacResult &result = results->at(camId);
    result.relPoseInfo.cam1.frameId = olderFrame->id();
    result.relPoseInfo.cam1.cameraIndex = camId;
    result.relPoseInfo.cam2.frameId = currentFrame->id();
    result.relPoseInfo.cam2.cameraIndex = camId;
    size_t numMatch = matches[camId]->size();
    result.numCorrespondences = numMatch;
    if (numMatch < 1) {
      result.relPoseInfo.T_C1C2.setIdentity();
      result.inlierRatio = 0;
      result.relPoseInfo.status = RansacStatus::FewMatches;
    } else {
      Eigen::Matrix4d T_C1C2_mat = Eigen::Matrix4d::Identity();
      T_C1C2_mat.topLeftCorner<3, 4>() = ransac.model_coefficients_;
      result.relPoseInfo.T_C1C2.set(T_C1C2_mat);
      result.inlierRatio = float(result.inliers.size()) / float(numMatch);
      result.relPoseInfo.status = RansacStatus::NoncentralOk;
    }
  }
  return ransac.inliers_.size();
}

int FeatureTrackingFrontend::runRansacFundamentalMat(
    ConstMultiFramePtr currentFrame, ConstMultiFramePtr olderFrame,
    size_t camIdx, const okvis::Matches &matches, FrameRansacResult *result) {
  // relative pose adapter for Kneip toolchain
  opengv::relative_pose::SimpleFrameRelativeAdapter adapter(
      matches, *nCameraSystem_, olderFrame, camIdx, currentFrame, camIdx);

  size_t numCorrespondences = adapter.getNumberCorrespondences();
  result->relPoseInfo.cam1.frameId = olderFrame->id();
  result->relPoseInfo.cam1.cameraIndex = camIdx;
  result->relPoseInfo.cam2.frameId = currentFrame->id();
  result->relPoseInfo.cam2.cameraIndex = camIdx;

  result->numCorrespondences = numCorrespondences;

  if (numCorrespondences < 10) {
    // won't generate meaningful results. let's hope the few
    // correspondences we have are all inliers!!
    // Note we have Mahalanobis gating or robust loss as the last defense layer.
    result->relPoseInfo.status = RansacStatus::FewMatches;
    result->inliers.resize(numCorrespondences);
    for (size_t i = 0; i < numCorrespondences; ++i) {
      result->inliers[i] = i;
    }
    result->relPoseInfo.T_C1C2.setIdentity();
    result->inlierRatio = 1.0;
    return 0;
  }

  typedef opengv::sac_problems::relative_pose::SimpleFrameRelativePoseSacProblem
      FrameRelativePoseSacProblem;
  opengv::sac::Ransac<FrameRelativePoseSacProblem> rel_pose_ransac;
  std::shared_ptr<FrameRelativePoseSacProblem> rel_pose_problem_ptr(
      new FrameRelativePoseSacProblem(adapter,
                                      FrameRelativePoseSacProblem::STEWENIUS));
  rel_pose_ransac.sac_model_ = rel_pose_problem_ptr;
  rel_pose_ransac.threshold_ = 9; // Threshold is roughly \f$ |r - r_0|^2 * f^2 / 2(\sigma^2) \f$
  rel_pose_ransac.max_iterations_ = 50;

  rel_pose_ransac.computeModel(0);

  result->inliers = rel_pose_ransac.inliers_;
  Eigen::Matrix4d T_C1C2_mat = Eigen::Matrix4d::Identity();
  T_C1C2_mat.topLeftCorner<3, 4>() =
      rel_pose_ransac.model_coefficients_;
  result->relPoseInfo.T_C1C2.set(T_C1C2_mat);
  result->inlierRatio =
      float(rel_pose_ransac.inliers_.size()) / float(numCorrespondences);
  result->relPoseInfo.status = RansacStatus::FundamentalMatOk;
  return result->inliers.size();
}

int FeatureTrackingFrontend::runRansacRotationOnly(
    ConstMultiFramePtr currentFrame, ConstMultiFramePtr olderFrame,
    size_t camIdx, const okvis::Matches &matches, FrameRansacResult *result) {
  // jhuai: We do not compensate the rolling shutter effect in the opengv 5
  // pt-algorithm, because for keypoints matched by descriptors with outliers,
  // their optic flow and the relative pose form a chicken-egg problems.

  // relative pose adapter for Kneip toolchain
  opengv::relative_pose::SimpleFrameRelativeAdapter adapter(
      matches, *nCameraSystem_, olderFrame, camIdx, currentFrame, camIdx);

  result->relPoseInfo.cam1.frameId = olderFrame->id();
  result->relPoseInfo.cam1.cameraIndex = camIdx;
  result->relPoseInfo.cam2.frameId = currentFrame->id();
  result->relPoseInfo.cam2.cameraIndex = camIdx;

  size_t numCorrespondences = adapter.getNumberCorrespondences();
  result->numCorrespondences = numCorrespondences;
  if (numCorrespondences < 10) {
    result->relPoseInfo.status = RansacStatus::FewMatches;
    return 0; // won't generate meaningful results. let's hope the few
              // correspondences we have are all inliers!!
  }

  typedef opengv::sac_problems::relative_pose::SimpleFrameRotationOnlySacProblem
      FrameRotationOnlySacProblem;
  opengv::sac::Ransac<FrameRotationOnlySacProblem> rotation_only_ransac;
  std::shared_ptr<FrameRotationOnlySacProblem> rotation_only_problem_ptr(
      new FrameRotationOnlySacProblem(adapter));
  rotation_only_ransac.sac_model_ = rotation_only_problem_ptr;
  rotation_only_ransac.threshold_ = 9;
  rotation_only_ransac.max_iterations_ = 50;

  rotation_only_ransac.computeModel(0);

  result->inliers = rotation_only_ransac.inliers_;
  Eigen::Matrix4d T_C1C2_mat = Eigen::Matrix4d::Identity();
  T_C1C2_mat.topLeftCorner<3, 3>() =
      rotation_only_ransac.model_coefficients_;
  result->relPoseInfo.T_C1C2.set(T_C1C2_mat);

  result->numCorrespondences = numCorrespondences;
  result->inlierRatio =
      float(rotation_only_ransac.inliers_.size()) / float(numCorrespondences);
  result->relPoseInfo.status = RansacStatus::RotationOnlyOk;
  return result->inliers.size();
}

}  // namespace swift_vio
