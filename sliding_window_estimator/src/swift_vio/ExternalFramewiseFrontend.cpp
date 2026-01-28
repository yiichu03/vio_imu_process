#include <swift_vio/ExternalFramewiseFrontend.hpp>

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
ExternalFramewiseFrontend::ExternalFramewiseFrontend(size_t numCameras, const FrontendOptions& frontendOptions)
    : CameraFrontendBase(numCameras), frontendOptions_(frontendOptions) {
}

ExternalFramewiseFrontend::~ExternalFramewiseFrontend() {
  LOG(INFO) << "Framewise descriptor matching frontend associated " << numNFrames_
            << " NFrames, and selected " << numKeyframes_ << " keyframes.";
}

bool ExternalFramewiseFrontend::detectAndDescribe(size_t cameraIndex,
                                                  std::shared_ptr<MultiFrame> frameOut)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, cameraIndex < numCameras_, "Camera index exceeds number of cameras.");
  // std::lock_guard<std::mutex> lock(*featureDetectorMutexes_[cameraIndex]);
  // featureDetectors_[cameraIndex]->detect(frameOut->image(cameraIndex),
  //                                        frameOut->keypoints(cameraIndex));
  // descriptorExtractors_[cameraIndex]->compute(frameOut->image(cameraIndex),
  //                                             frameOut->keypoints(cameraIndex),
  //                                             frameOut->descriptors(cameraIndex));
  return true;
}

uint64_t ExternalFramewiseFrontend::oldestNFrameId() const {
  return 0;
  // return multiFramePtrMap_.begin()->first;
}

bool ExternalFramewiseFrontend::dataAssociation(
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
       ++cameraIndex)
  {
    nframePtr->second.setTimestamp(cameraIndex,
                                   currentNFrame->timestamp(cameraIndex));
    nframePtr->second.landmarkIdsMutable(cameraIndex)
        .resize(currentNFrame->keypoints(cameraIndex).size(), 0u);
  }

  // select keyframes and frames for feature matching
  // std::vector<ConstMultiFramePtr> dbKeyframes;
  // selectKeyframesWith3dPointsForMatching(submap, &dbKeyframes);

  // std::vector<ConstMultiFramePtr> keyframes;
  // size_t num = 0;
  // if (frontendOptions_.numKeyframesToMatch > dbKeyframes.size())
  // {
  //   num = frontendOptions_.numKeyframesToMatch - dbKeyframes.size();
  // }
  // selectKeyframesForMatching(&keyframes, num);

  // output_ = output;

  // matchToKeyframesWith3dPoints(submap, dbKeyframes, currentNFrame);

  // matchToKeyframes(keyframes, currentNFrame);

  // // keyframe decision, at the moment only landmarks that match with keyframe or submap are recorded.
  bool asKeyframe = frontendOptions_.allAreKeyframes || doWeNeedANewKeyframe(currentNFrame);
  currentNFrame->setKeyframe(asKeyframe);
  multiFramePtrMap_.at(currentNFrame->id())->setKeyframe(asKeyframe);

  // matchToLastFrame(currentNFrame);

  // matchStereoWithEpipolarCheck(currentNFrame);

  // memoryManagement(currentNFrame);

  output_->currentNFrame.reset(new MultiFrame(*currentNFrame));

  ++numNFrames_;
  numKeyframes_ += (asKeyframe ? 1 : 0);
  return true;
}

bool ExternalFramewiseFrontend::doWeNeedANewKeyframe(
    ConstMultiFramePtr currentFrame) const
{
  if (numNFrames_ == 0)
  {
    // The first frame should be a new keyframe.
    return true;
  }

  // check the overlap between matched keypoints and all keypoints of the current NFrame.
  double overlap = 0.0;
  double ratio = 0.0;

  const BareMultiFrame &nframeLandmarkIds = multiFrame2LandmarkIdMap_.at(currentFrame->id());
  for (size_t im = 0; im < currentFrame->numFrames(); ++im)
  {
    // get the hull of all keypoints in current frame
    std::vector<cv::Point2f> frameBPoints, frameBHull;
    std::vector<cv::Point2f> frameBMatches, frameBMatchesHull;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> frameBLandmarks;
    const std::vector<uint64_t> &frameLandmarkIds = nframeLandmarkIds.landmarkIds(im);
    const size_t numB = currentFrame->numKeypoints(im);
    frameBPoints.reserve(numB);
    frameBLandmarks.reserve(numB);
    Eigen::Vector2d keypoint;
    for (size_t k = 0; k < numB; ++k)
    {
      currentFrame->getKeypoint(im, k, keypoint);
      frameBPoints.push_back(cv::Point2f(keypoint[0], keypoint[1]));
      if (frameLandmarkIds.at(k) != 0)
      {
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
    if (frameBMatchesHull.size() > 2)
    {
      for (size_t k = 0; k < frameBPoints.size(); ++k)
      {
        if (cv::pointPolygonTest(frameBMatchesHull, frameBPoints[k], false) > 0)
        {
          pointsInFrameBMatchesArea++;
        }
      }
    }
    double matchingRatio = double(frameBMatches.size()) / double(pointsInFrameBMatchesArea);

    overlap = std::max(overlapArea, overlap);
    ratio = std::max(matchingRatio, ratio);
  }

  if (overlap > frontendOptions_.keyframeInsertionOverlapThreshold && ratio > frontendOptions_.keyframeInsertionMatchingRatioThreshold)
    return false;
  else
    return true;
}
}  // namespace swift_vio
