/**
 * @file FeatureTrackingFrontend.hpp
 * @brief Header file for the FeatureTrackingFrontend class.
 */

#ifndef INCLUDE_SWIFT_VIO_FEATURE_TRACKING_FRONTEND_HPP_
#define INCLUDE_SWIFT_VIO_FEATURE_TRACKING_FRONTEND_HPP_

#include <feature_tracker/distance.h>

#include <okvis/assert_macros.hpp>
#include <okvis/DenseMatcher.hpp>
#include <okvis/timing/Timer.hpp>

#include <swift_vio/CameraFrontendBase.h>
#include <swift_vio/messages.h>

namespace swift_vio {
/**
* @brief merge feature track source to feature track target.
* @param featureTracks
* @param multiFrame2LandmarkIdMap
* @param trackIdTarget
* @param trackIdSource
*/
void mergeTwoFeatureTracks(
    FeatureTrackMap *featureTracks,
    std::unordered_map<uint64_t, BareMultiFrame> *multiFrame2LandmarkIdMap,
    uint64_t trackIdSource,
    uint64_t trackIdTarget);

/**
 * @brief findMatches
 * @param landmarkIdListA
 * @param landmarkIdListB
 * @param[in, out] matches the found matches of the same landmark id will be appended to the existing matches.
 */
void findMatches(const std::vector<uint64_t> &landmarkIdListA,
                 const std::vector<uint64_t> &landmarkIdListB,
                 okvis::Matches *matches);

/**
 * @brief Feature tracking frontend.
 * This frontend matches features of the current frame and last frame,
 * features of keyframes and current frames,
 * filter outliers by using RANSAC, and select keyframes.
 * Feature matching for 2 N-frames includes 2d-2d matching and 3d-2d matching.
 * The 3d-2d matching requires the submap of motion and structure from the
 * Estimator. Compared to other Frontend, this frontend does not depend on
 * Estimator.
 *
 * TODO(jhuai): implement a frontend based on KLT,
 * use KLT to track between the last frame and current frame and compute the
 * optic flow for rolling shutter compensation, then extract keypoints for
 * tracked points and use descriptor-based matching between keyframes and current
 * frame to handle large motion.
 *
 */
class FeatureTrackingFrontend : public CameraFrontendBase {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor.
   * @param numCameras Number of cameras in the sensor configuration.
   */
  FeatureTrackingFrontend(size_t numCameras, const swift_vio::FrontendOptions& frontendOptions);

  virtual ~FeatureTrackingFrontend();

  /// @name override methods.
  /// @{
  bool detectAndDescribe(
      size_t cameraIndex, std::shared_ptr<MultiFrame> frameOut) override;

  bool dataAssociation(
      std::shared_ptr<MultiFrame> currentNFrame,
      std::shared_ptr<const VisualSubmap> submap,
      VisualMatcherOutput *output) override;

  uint64_t oldestNFrameId() const override;
  /// @}

 protected:
  /**
   * @brief   feature detectors with the current settings.
   *          The vector contains one for each camera to ensure that there are no problems with parallel detection.
   * @warning Lock with featureDetectorMutexes_[cameraIndex] when using the detector.
   */
  std::vector<std::shared_ptr<cv::FeatureDetector> > featureDetectors_;
  /**
   * @brief   feature descriptors with the current settings.
   *          The vector contains one for each camera to ensure that there are no problems with parallel detection.
   * @warning Lock with featureDetectorMutexes_[cameraIndex] when using the descriptor.
   */
  std::vector<std::shared_ptr<cv::DescriptorExtractor> > descriptorExtractors_;
  /// Mutexes for feature detectors and descriptors.
  std::vector<std::unique_ptr<std::mutex> > featureDetectorMutexes_;

  ///@}

  swift_vio::FrontendOptions frontendOptions_;
  std::unique_ptr<okvis::DenseMatcher> matcher_; ///< Matcher object.

  BinaryDescriptorDistanceCallback distanceCallback_;

  std::map<uint64_t, MultiFramePtr> multiFramePtrMap_;  ///< The processed multiframes by the frontend in chronical order, including the current NFrame.
  ///< It does not include those keyframes retrieved from the database.
  std::unordered_map<uint64_t, BareMultiFrame> multiFrame2LandmarkIdMap_;

  bool rotationOnly_;  ///< is the camera going through rotationOnly motion at the beginning?

  VisualMatcherOutput *output_; ///< intermediate result.

  /// (re)instantiates feature detectors and descriptor extractors. Used after settings changed or at startup.
  void initialiseFeatureDetectors();

  void selectKeyframesWith3dPointsForMatching(
      std::shared_ptr<const VisualSubmap> submap,
      std::vector<ConstMultiFramePtr> *keyframes) const;

  void selectKeyframesForMatching(
      std::vector<ConstMultiFramePtr> *keyframes, size_t num) const;

  int matchToKeyframesWith3dPoints(
      std::shared_ptr<const VisualSubmap> submap,
      const std::vector<ConstMultiFramePtr> &dbKeyframes,
      ConstMultiFramePtr currentNFrame);

  /**
   * @brief Match a new multiframe to existing keyframes
   * @param[in]  currentFrameId         ID of the current frame that should be matched against keyframes.
   * @return The number of matches in total.
   */
  int matchToKeyframes(const std::vector<ConstMultiFramePtr> &keyframes,
                       ConstMultiFramePtr currentFrame);  // for wide-baseline matches (good initial guess)

  /**
   * @brief Match a new multiframe to the last frame.
   * @param currentFrameId      ID of the current frame that should be matched against the last one.
   * @return The number of matches in total.
   */
  int matchToLastFrame(ConstMultiFramePtr currentFrame);

  /**
   * @brief Match the frames inside the multiframe to each other.
   * @param multiFrame  Multiframe containing the frames to match.
   */
  int matchStereoWithEpipolarCheck(ConstMultiFramePtr multiFrame);

  /**
   * @brief update feature tracks with temporal matches
   * @param matches
   * @param result inliers of matches
   * @return number of new observations
   */
  int updateFeatureTracks(const okvis::Matches &matches, const FrameRansacResult &result, int numExistingMatches);

  /**
   * @brief update feature tracks with stereo matches with known relative frame pose, and merge tracks if needed.
   * @param stereoMatches
   * @return number of new observations
   */
  int updateAndMergeFeatureTracks(const okvis::Matches &stereoMatches,
                                   ConstMultiFramePtr nframe, size_t camIdA,
                                   size_t camIdB);

  /**
   * @brief Decision whether a new frame should be keyframe or not.
   * Determine keyframes by checking the ratio of the area covered by feature matches and that by all features,
   * and checking the number of feature matches and the number of features in the area of matches.
   * @param currentFrame  Keyframe candidate.
   * @return True if it should be a new keyframe.
   */
  bool doWeNeedANewKeyframe(ConstMultiFramePtr currentFrame) const;

  int runRansacFundamentalMat(ConstMultiFramePtr currentFrame,
                              ConstMultiFramePtr olderFrame, size_t camIdx,
                              const okvis::Matches &matches,
                              FrameRansacResult *result);

  int runRansacNoncentralRelativePose(
      ConstMultiFramePtr olderFrame,
      ConstMultiFramePtr currentFrame,
      const std::vector<std::shared_ptr<okvis::Matches>> & matches,
      Eigen::AlignedVector<FrameRansacResult> *results);

  int runRansacRotationOnly(ConstMultiFramePtr currentFrame,
                            ConstMultiFramePtr olderFrame, size_t camIdx,
                            const okvis::Matches &matches,
                            FrameRansacResult *result);

  int runRansacPnP(ConstMultiFramePtr currentFrame);

  /**
   * @brief remove old or redundant NFrames
   */
  void memoryManagement(ConstMultiFramePtr currentNFrame);
};

}  // namespace swift_vio

#endif // INCLUDE_SWIFT_VIO_FEATURE_TRACKING_FRONTEND_HPP_
