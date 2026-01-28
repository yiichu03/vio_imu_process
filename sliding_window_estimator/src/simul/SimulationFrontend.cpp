#include <simul/SimulationFrontend.hpp>

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

const double SimulationFrontend::fourthRoot2_ = 1.1892071150;

SimulationFrontend::SimulationFrontend(
    const std::vector<Eigen::Vector4d,
                      Eigen::aligned_allocator<Eigen::Vector4d>>
        &homogeneousPoints,
    const std::vector<uint64_t> &lmIds, size_t numCameras, const SimFrontendOptions& options)
    : options_(options), homogeneousPoints_(homogeneousPoints),
      lmIds_(lmIds), numNFrames_(0), numKeyframes_(0), numCameras_(numCameras) {
}

int SimulationFrontend::dataAssociation(
    std::shared_ptr<swift_vio::MultiFrame> nframes, const okvis::kinematics::Transformation & T_WB,
    const std::vector<std::unordered_map<size_t, size_t>>& keypointIndices,
    swift_vio::VisualMatcherOutput *output) {
  int requiredMatches = 5;
  int trackedFeatures = 0;
  auto res = multiFrame2LandmarkIdMap_.emplace(
      std::piecewise_construct, std::forward_as_tuple(nframes->id()),
      std::forward_as_tuple(nframes->timestamp(), nframes->id(),
                            numCameras_));
  for (size_t j = 0; j < numCameras_; ++j) {
    res.first->second.setTimestamp(j,
        nframes->timestamp(j));
    // We initialize the feature tracks and landmarks from feature matches
    // to mimic a real feature tracking frontend.
    res.first->second.landmarkIdsMutable(j)
        .resize(keypointIndices.at(j).size(), 0u);
  }

  if (numNFrames_) {
    // Find matches between a previous keyframe and current frame.
    std::vector<LandmarkKeypointMatch> landmarkKeyframeMatches;
    int numKeyframes = 0;
    for (int age = (int)nframeList_.size() - 1; age >= 0; --age) {
      if (!nframeList_[age].isKeyframe_)
        continue;

      matchToFrame(nframeList_[age].keypointIndices_, keypointIndices,
                    nframeList_[age].nframe_->id(), nframes->id(),
                    &landmarkKeyframeMatches);

      trackedFeatures += updateFeatureTracks(
          nframeList_[age].nframe_, nframes, landmarkKeyframeMatches,
          &output->featureTracks);

      ++numKeyframes;
      if (numKeyframes >= options_.maxMatchKeyframes_) {
        break;
      }
    }
    // find matches between the previous frame and current frame.
    if (!nframeList_.back().isKeyframe_) {
      auto lastNFrame = nframeList_.rbegin();
      std::vector<LandmarkKeypointMatch> landmarkFrameMatches;
      matchToFrame(lastNFrame->keypointIndices_, keypointIndices,
                   lastNFrame->nframe_->id(), nframes->id(),
                   &landmarkFrameMatches);
      trackedFeatures += updateFeatureTracks(
          lastNFrame->nframe_, nframes, landmarkFrameMatches,
          &output->featureTracks);
    }
    if (trackedFeatures <= requiredMatches) {
      VLOG(0) << "Tracking landmarks failure. Number of matches: "
              << trackedFeatures;
    }
  }

  bool asKeyframe = nframes->isKeyframe() || doWeNeedANewKeyframe(T_WB);
  nframes->setKeyframe(asKeyframe);
  ++numNFrames_;
  if (asKeyframe) {
    previousKeyframePose_ = T_WB;
    ++numKeyframes_;
  }
  nframeList_.emplace_back(nframes, T_WB, keypointIndices, asKeyframe);

  for (auto iter = nframeList_.rbegin(); iter != nframeList_.rend(); ++iter) {
    if (iter->isKeyframe_) {
      output->closestKeyframe = iter->nframe_;
      break;
    }
  }
  output->rotationOnly = false;

  // keep bounded size of records.
  size_t numKeyframes = 0;
  uint64_t oldestKeyframeId = 0;
  for (auto iter = nframeList_.rbegin(); iter != nframeList_.rend(); ++iter) {
    numKeyframes += iter->isKeyframe_ ?  1 : 0;
    if (numKeyframes > 10) {
      oldestKeyframeId = iter->nframe_->id();
      break;
    }
  }
  while (nframeList_.front().nframe_->id() < oldestKeyframeId) {
    nframeList_.pop_front();
  }
  for (auto iter = multiFrame2LandmarkIdMap_.begin(); iter != multiFrame2LandmarkIdMap_.end();) {
    if (iter->first < oldestKeyframeId) {
      iter = multiFrame2LandmarkIdMap_.erase(iter);
    } else {
      ++iter;
    }
  }
  return trackedFeatures;
}

bool SimulationFrontend::doWeNeedANewKeyframe(
    const okvis::kinematics::Transformation &current_T_WB) const {
  if (numNFrames_ == 0) {
    return true;
  }

  okvis::kinematics::Transformation T_SpSc = previousKeyframePose_.inverse() * current_T_WB;
  double distance = T_SpSc.r().norm();
  double rotAngle = std::acos(T_SpSc.q().w()) * 2;
  if (distance > options_.minKeyframeDistance_ || rotAngle > options_.minKeyframeAngle_) {
    return true;
  } else {
    return false;
  }
}

int SimulationFrontend::updateFeatureTracks(
    std::shared_ptr<swift_vio::MultiFrame> prevFrames,
    std::shared_ptr<swift_vio::MultiFrame> currFrames,
    const std::vector<LandmarkKeypointMatch> &landmarkMatches,
    swift_vio::FeatureTrackMap *featureTracks) {
  int trackedFeatures = 0;
  for (auto landmarkMatch : landmarkMatches) {
    swift_vio::BareMultiFrame &prevNFrameLmks =
        multiFrame2LandmarkIdMap_.at(prevFrames->id());
    swift_vio::BareMultiFrame &currNFrameLmks =
        multiFrame2LandmarkIdMap_.at(currFrames->id());
    uint64_t lmIdPrevious =
        prevNFrameLmks.landmarkId(landmarkMatch.previousKeypoint.cameraIndex,
                                  landmarkMatch.previousKeypoint.keypointIndex);
    uint64_t lmIdCurrent =
        currNFrameLmks.landmarkId(landmarkMatch.currentKeypoint.cameraIndex,
                                  landmarkMatch.currentKeypoint.keypointIndex);
    if (lmIdPrevious != 0) {
      if (lmIdCurrent != 0) {
        if (lmIdPrevious != lmIdCurrent) {
          LOG(WARNING) << "Different landmarks " << lmIdPrevious << " and "
                       << lmIdCurrent << " are involved in a feature match!";
        }
        continue;
      }
      CHECK_EQ(lmIdPrevious, landmarkMatch.landmarkId);
      // The observation in the previous frame has been added before, so we only
      // add the observation in the current frame.
      swift_vio::FeatureTrackMap::iterator iter =
          featureTracks->find(lmIdPrevious);
      if (iter == featureTracks->end()) {
        auto res = featureTracks->emplace(
            landmarkMatch.landmarkId,
            swift_vio::FeatureTrack(landmarkMatch.landmarkId));
        iter = res.first;
      }
      Eigen::Vector2d uv;
      currFrames->getKeypoint(landmarkMatch.currentKeypoint.cameraIndex,
                              landmarkMatch.currentKeypoint.keypointIndex, uv);
      double size;
      currFrames->getKeypointSize(landmarkMatch.currentKeypoint.cameraIndex,
                                  landmarkMatch.currentKeypoint.keypointIndex,
                                  size);
      Eigen::Vector2f veluv;
      iter->second.observations.emplace(
          std::piecewise_construct,
          std::forward_as_tuple(currFrames->id(),
                                landmarkMatch.currentKeypoint.cameraIndex,
                                landmarkMatch.currentKeypoint.keypointIndex),
          std::forward_as_tuple(uv.cast<float>(), size, veluv));

      currNFrameLmks.setLandmarkId(landmarkMatch.currentKeypoint.cameraIndex,
                                   landmarkMatch.currentKeypoint.keypointIndex,
                                   lmIdPrevious);
      ++trackedFeatures;
    } else {
      if (lmIdCurrent != 0) {
       // The observation in the current frame has been added before,
       // but the observation in the previous frame is not added yet.
        swift_vio::FeatureTrackMap::iterator iter =
            featureTracks->find(lmIdCurrent);
        CHECK(iter != featureTracks->end());
        Eigen::Vector2d uv;
        prevFrames->getKeypoint(landmarkMatch.previousKeypoint.cameraIndex,
                                landmarkMatch.previousKeypoint.keypointIndex,
                                uv);
        double size;
        prevFrames->getKeypointSize(
            landmarkMatch.previousKeypoint.cameraIndex,
            landmarkMatch.previousKeypoint.keypointIndex, size);
        Eigen::Vector2f veluv;
        iter->second.observations.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(prevFrames->id(),
                                  landmarkMatch.previousKeypoint.cameraIndex,
                                  landmarkMatch.previousKeypoint.keypointIndex),
            std::forward_as_tuple(uv.cast<float>(), size, veluv));
        prevNFrameLmks.setLandmarkId(
            landmarkMatch.previousKeypoint.cameraIndex,
            landmarkMatch.previousKeypoint.keypointIndex, lmIdCurrent);
        ++trackedFeatures;
      } else {
        // The observations in the current frame and the previous frame
        // have not been added yet.
        swift_vio::FeatureTrackMap::iterator iter =
            featureTracks->find(landmarkMatch.landmarkId);
        if (iter == featureTracks->end()) {
          auto res = featureTracks->emplace(
              landmarkMatch.landmarkId,
              swift_vio::FeatureTrack(landmarkMatch.landmarkId));
          iter = res.first;
        } else {
          // This happens when the landmark has been added for camera 0,
          // and now we are working on camera 1.
        }
        Eigen::Vector2d uv;
        prevFrames->getKeypoint(landmarkMatch.previousKeypoint.cameraIndex,
                                landmarkMatch.previousKeypoint.keypointIndex,
                                uv);
        double size;
        prevFrames->getKeypointSize(
            landmarkMatch.previousKeypoint.cameraIndex,
            landmarkMatch.previousKeypoint.keypointIndex, size);
        Eigen::Vector2f veluv;
        iter->second.observations.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(prevFrames->id(),
                                  landmarkMatch.previousKeypoint.cameraIndex,
                                  landmarkMatch.previousKeypoint.keypointIndex),
            std::forward_as_tuple(uv.cast<float>(), size, veluv));
        prevNFrameLmks.setLandmarkId(
            landmarkMatch.previousKeypoint.cameraIndex,
            landmarkMatch.previousKeypoint.keypointIndex,
            landmarkMatch.landmarkId);
        ++trackedFeatures;

        currFrames->getKeypoint(landmarkMatch.currentKeypoint.cameraIndex,
                                landmarkMatch.currentKeypoint.keypointIndex,
                                uv);
        currFrames->getKeypointSize(landmarkMatch.currentKeypoint.cameraIndex,
                                    landmarkMatch.currentKeypoint.keypointIndex,
                                    size);
        iter->second.observations.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(currFrames->id(),
                                  landmarkMatch.currentKeypoint.cameraIndex,
                                  landmarkMatch.currentKeypoint.keypointIndex),
            std::forward_as_tuple(uv.cast<float>(), size, veluv));

        currNFrameLmks.setLandmarkId(
            landmarkMatch.currentKeypoint.cameraIndex,
            landmarkMatch.currentKeypoint.keypointIndex,
            landmarkMatch.landmarkId);
        ++trackedFeatures;
      }
    }
  }
  return trackedFeatures;
}

int SimulationFrontend::matchToFrame(
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
