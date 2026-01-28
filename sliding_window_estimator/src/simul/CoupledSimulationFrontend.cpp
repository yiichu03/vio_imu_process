#include <simul/CoupledSimulationFrontend.hpp>

#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <vio/Sample.h>
#include <okvis/IdProvider.hpp>

// cameras and distortions
#include <okvis/CameraModelSwitch.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/FovDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <simul/PointLandmarkSimulationRS.hpp>

namespace simul {

const double CoupledSimulationFrontend::fourthRoot2_ = 1.1892071150;

CoupledSimulationFrontend::CoupledSimulationFrontend(
    const std::vector<Eigen::Vector4d,
                      Eigen::aligned_allocator<Eigen::Vector4d>>
        &homogeneousPoints,
    const std::vector<uint64_t> &lmIds, size_t numCameras, const SimFrontendOptions& options)
    : VioFrontendInterface(numCameras),
      options_(options), homogeneousPoints_(homogeneousPoints),
      lmIds_(lmIds) {
  isInitialized_ = true;
}

int CoupledSimulationFrontend::dataAssociationAndInitialization(
    okvis::EstimatorBase& estimator, const std::vector<std::unordered_map<size_t, size_t>>& keypointIndices,
    std::shared_ptr<okvis::MultiFrame> nframes, bool* asKeyframe) {
  okvis::cameras::DistortionType distortionType =
      nframes->cameraSystem().distortionType(0);
  int requiredMatches = 5;
  int trackedFeatures = 0;
  if (estimator.numFrames() > 1) {
    // Find matches between a previous keyframe and current frame.
    std::vector<LandmarkKeypointMatch> landmarkKeyframeMatches;
    int numKeyframes = 0;
    for (size_t age = 1; age < estimator.numFrames(); ++age) {
      uint64_t olderFrameId = estimator.frameIdByAge(age);
      if (!estimator.isKeyframe(olderFrameId))
        continue;

      auto rit = nframeList_.rbegin();
      for (; rit != nframeList_.rend(); ++rit) {
        if (rit->nframe_->id() == olderFrameId) {
          OKVIS_ASSERT_TRUE(Exception, rit->isKeyframe_,
                            "Inconsistent frontend and backend frame status!");
          break;
        }
      }
      matchToFrame2(rit->keypointIndices_, keypointIndices, rit->nframe_->id(),
                   nframes->id(), &landmarkKeyframeMatches);

#define DISTORTION_MODEL_CASE(camera_geometry_t)                               \
  trackedFeatures += addMatchToEstimator<camera_geometry_t>(                   \
      estimator, rit->nframe_, nframes, landmarkKeyframeMatches);

      switch (distortionType) { DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES }

#undef DISTORTION_MODEL_CASE

      ++numKeyframes;

      if (numKeyframes >= options_.maxMatchKeyframes_) {
        break;
      }
    }
    // find matches between the previous frame and current frame.
    uint64_t lastFrameId = estimator.frameIdByAge(1);
    if (!estimator.isKeyframe(lastFrameId)) {
      auto lastNFrame = nframeList_.rbegin();
      OKVIS_ASSERT_EQ(Exception, lastNFrame->nframe_->id(), lastFrameId,
                      "Inconsistent frontend and backend frame status!");

      std::vector<LandmarkKeypointMatch> landmarkFrameMatches;
      matchToFrame2(lastNFrame->keypointIndices_, keypointIndices,
                   lastNFrame->nframe_->id(), nframes->id(),
                   &landmarkFrameMatches);

#define DISTORTION_MODEL_CASE(camera_geometry_t)                               \
  trackedFeatures += addMatchToEstimator<camera_geometry_t>(                   \
      estimator, lastNFrame->nframe_, nframes, landmarkFrameMatches);

      switch (distortionType) { DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES }

#undef DISTORTION_MODEL_CASE
    }
    if (trackedFeatures <= requiredMatches) {
      VLOG(0) << "Tracking landmarks failure. Number of matches: "
              << trackedFeatures;
    }
  }

  okvis::kinematics::Transformation current_T_WS;
  estimator.get_T_WS(nframes->id(), current_T_WS);
  *asKeyframe = *asKeyframe || doWeNeedANewKeyframe(estimator, nframes);
  ++numNFrames_;
  if (*asKeyframe) {
    previousKeyframePose_ = current_T_WS;
    ++numKeyframes_;
  }
  nframeList_.emplace_back(nframes, current_T_WS, keypointIndices, *asKeyframe);
  uint64_t oldestFrameId = estimator.oldestFrameId();
  while (nframeList_.front().nframe_->id() < oldestFrameId) {
    nframeList_.pop_front();
  }

  return trackedFeatures;
}

bool CoupledSimulationFrontend::doWeNeedANewKeyframe(
    const okvis::EstimatorBase& estimator,
    std::shared_ptr<okvis::MultiFrame> currentFrame) const {
  if (estimator.numFrames() < 2) {
    return true;
  }

  okvis::kinematics::Transformation current_T_WS;
  estimator.get_T_WS(currentFrame->id(), current_T_WS);
  okvis::kinematics::Transformation T_SpSc = previousKeyframePose_.inverse() * current_T_WS;
  double distance = T_SpSc.r().norm();
  double rotAngle = std::acos(T_SpSc.q().w()) * 2;
  if (distance > options_.minKeyframeDistance_ || rotAngle > options_.minKeyframeAngle_) {
    return true;
  } else {
    return false;
  }
}


template <class CAMERA_GEOMETRY_T>
int CoupledSimulationFrontend::addMatchToEstimator(
    okvis::EstimatorBase& estimator, std::shared_ptr<okvis::MultiFrame> prevFrames,
    std::shared_ptr<okvis::MultiFrame> currFrames,
    const std::vector<LandmarkKeypointMatch>& landmarkMatches) const {
  int trackedFeatures = 0;
  for (auto landmarkMatch : landmarkMatches) {
    uint64_t lmIdPrevious = prevFrames->landmarkId(
        landmarkMatch.previousKeypoint.cameraIndex,
        landmarkMatch.previousKeypoint.keypointIndex);
    uint64_t lmIdCurrent =
        currFrames->landmarkId(landmarkMatch.currentKeypoint.cameraIndex,
                               landmarkMatch.currentKeypoint.keypointIndex);
    if (lmIdPrevious != 0 && estimator.isLandmarkAdded(lmIdPrevious)) {
      if (lmIdCurrent != 0) { // avoid duplicates.
        if (lmIdPrevious != lmIdCurrent) {
          LOG(WARNING) << "Different landmarks " << lmIdPrevious << " and "
                    << lmIdCurrent << " are involved in a feature match!";
        }
        continue;
      }
      CHECK_EQ(lmIdPrevious, landmarkMatch.landmarkId);
      currFrames->setLandmarkId(landmarkMatch.currentKeypoint.cameraIndex,
                                landmarkMatch.currentKeypoint.keypointIndex,
                                lmIdPrevious);
      estimator.addObservation<CAMERA_GEOMETRY_T>(
          lmIdPrevious, currFrames->id(),
          landmarkMatch.currentKeypoint.cameraIndex,
          landmarkMatch.currentKeypoint.keypointIndex);
      ++trackedFeatures;
    } else {
      // This happens when either both observations are not associated to any landmark yet, or
      // a landmark added earlier has been marginalized from the estimator but
      // its observations in the multiframe is not nullified yet.
      okvis::KeypointIdentifier IdA = landmarkMatch.previousKeypoint;
      okvis::KeypointIdentifier IdB = landmarkMatch.currentKeypoint;

      okvis::kinematics::Transformation T_WSa;
      okvis::kinematics::Transformation T_WSb;
      // Use estimated values rather than reference ones to triangulate the landmark.
      estimator.get_T_WS(IdA.frameId, T_WSa);
      estimator.get_T_WS(IdB.frameId, T_WSb);

      okvis::kinematics::Transformation T_SaCa;
      estimator.getCameraSensorExtrinsics(IdA.cameraIndex, T_SaCa);

      okvis::kinematics::Transformation T_SbCb;
      estimator.getCameraSensorExtrinsics(IdB.cameraIndex, T_SbCb);
      okvis::kinematics::Transformation T_WCa = T_WSa * T_SaCa;
      okvis::kinematics::Transformation T_CaCb =
          T_WCa.inverse() * (T_WSb * T_SbCb);

      okvis::MultiFramePtr frameA = prevFrames;
      okvis::MultiFramePtr frameB = currFrames;
      okvis::triangulation::ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>
          probabilisticStereoTriangulator(frameA, frameB, IdA.cameraIndex,
                                          IdB.cameraIndex, T_CaCb);

      Eigen::Vector4d hP_Ca;
      bool canBeInitialized;  // It is essentially if two rays are NOT parallel.

      double fA = frameA->geometryAs<CAMERA_GEOMETRY_T>(IdA.cameraIndex)
                      ->focalLengthU();
      double keypointAStdDev;
      frameA->getKeypointSize(IdA.cameraIndex, IdA.keypointIndex,
                              keypointAStdDev);
      keypointAStdDev = 0.8 * keypointAStdDev / 12.0;
      double raySigma = fourthRoot2_ * keypointAStdDev / fA;

      // valid tells if all involved Chi2's are small enough.
      bool valid = probabilisticStereoTriangulator.stereoTriangulate(
          IdA.keypointIndex, IdB.keypointIndex, hP_Ca, canBeInitialized,
          raySigma, true);

      if (valid) {
        prevFrames->setLandmarkId(landmarkMatch.previousKeypoint.cameraIndex,
                                  landmarkMatch.previousKeypoint.keypointIndex,
                                  landmarkMatch.landmarkId);
        currFrames->setLandmarkId(landmarkMatch.currentKeypoint.cameraIndex,
                                  landmarkMatch.currentKeypoint.keypointIndex,
                                  landmarkMatch.landmarkId);

        Eigen::Vector4d hP_W = homogeneousPoints_[landmarkMatch.landmarkIdInVector];
        if (!options_.useTrueLandmarkPosition_) {
          // Use estimated landmark position as in real scenarios.
          hP_W = T_WCa * hP_Ca;
        }
        estimator.addLandmark(landmarkMatch.landmarkId, hP_W);
        estimator.setLandmarkInitialized(landmarkMatch.landmarkId,
                                         canBeInitialized);
        estimator.addObservation<CAMERA_GEOMETRY_T>(
            landmarkMatch.landmarkId, IdA.frameId, IdA.cameraIndex,
            IdA.keypointIndex);
        estimator.addObservation<CAMERA_GEOMETRY_T>(
            landmarkMatch.landmarkId, IdB.frameId, IdB.cameraIndex,
            IdB.keypointIndex);
        trackedFeatures += 2;
      } // else pass
    }
  }
  return trackedFeatures;
}

int CoupledSimulationFrontend::matchToFrame2(
    const std::vector<std::unordered_map<size_t, size_t>>& previousKeypointIndices,
    const std::vector<std::unordered_map<size_t, size_t>>& currentKeypointIndices,
    const uint64_t prevFrameId, const uint64_t currFrameId,
    std::vector<LandmarkKeypointMatch>* landmarkMatches) const {
  landmarkMatches->clear();
  int numMatches = 0;
  for (size_t im = 0; im < previousKeypointIndices.size(); ++im) {
    const std::unordered_map<size_t, size_t>& previousKeypointIds = previousKeypointIndices[im];
    const std::unordered_map<size_t, size_t>& currentKeypointIds = currentKeypointIndices[im];

    for (const auto & l2k : currentKeypointIds) {
      auto res = previousKeypointIds.find(l2k.first);
      if (res != previousKeypointIds.end()) {
        LandmarkKeypointMatch lmKpMatch;
        lmKpMatch.previousKeypoint =
            okvis::KeypointIdentifier(prevFrameId, im, res->second);
        lmKpMatch.currentKeypoint =
            okvis::KeypointIdentifier(currFrameId, im, l2k.second);
        lmKpMatch.landmarkId = lmIds_[l2k.first];
        lmKpMatch.landmarkIdInVector = l2k.first;
        landmarkMatches->push_back(lmKpMatch);
        ++numMatches;
      }
    }
  }
  return numMatches;
}

}  // namespace simul
