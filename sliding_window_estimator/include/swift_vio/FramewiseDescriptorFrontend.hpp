
#ifndef INCLUDE_OKVIS_FRAMEWISE_DESCRIPTOR_FRONTEND_HPP_
#define INCLUDE_OKVIS_FRAMEWISE_DESCRIPTOR_FRONTEND_HPP_

#include <mutex>
#include <okvis/DenseMatcher.hpp>
#include <okvis/Frontend.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/timing/Timer.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

namespace swift_vio {
/**
 * @brief The frontend that supports framewise matching using BRISK descriptors.
 *
 * As with okvis::Frontend, the FramewiseDescriptorFrontend also initializes landmarks with
 * the (perhaps imprecisely calibrated) camera system.
 * These initial landmark positions will seed the landmarks in the smoothers,
 * but will not be used by filters which will triangulate landmarks based on feature tracks.
 *
 */
class FramewiseDescriptorFrontend : public okvis::Frontend {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor.
   * @param numCameras Number of cameras in the sensor configuration.
   */
  FramewiseDescriptorFrontend(size_t numCameras, const FrontendOptions& frontendOptions);
  ~FramewiseDescriptorFrontend() final;

  ///@{
 

  /**
   * @brief Matching as well as initialization of landmarks and state.
   * @warning This method is not threadsafe.
   * @warning This method uses the estimator. Make sure to not access it in
   * another thread.
   * @param estimator       EstimatorBase.
   * @param params          Configuration parameters.
   * @param[in, out] nframes     Multiframe including the descriptors of all the
   * keypoints.
   * @param[out] asKeyframe Should the frame be a keyframe?
   * @return True if successful.
   */
  bool dataAssociationAndInitialization(
      okvis::EstimatorBase& estimator,
      const okvis::VioParameters& params,
      std::shared_ptr<okvis::MultiFrame> nframes, bool* asKeyframe) final;
 
  ///@}

 private:
  /**
   * @brief Decision whether a new frame should be keyframe or not after the
   * currentFrame has matched to its previous frame, by examining
   * matches between currentFrame and latest keyframe.
   * The keypoints for a match have the same landmark Id.
   * @param estimator     const reference to the estimator.
   * @param currentFrame  Keyframe candidate.
   * @return True if it should be a new keyframe.
   */
  bool doWeNeedANewKeyframePosterior(
      const okvis::EstimatorBase& estimator,
      std::shared_ptr<okvis::MultiFrame> currentFrame);

  /**
   * @brief Match a new multiframe to the last frame. This method differs from
   * Frontend::matchToLastFrame by passing out the argument rotationOnly.
   * @tparam MATCHING_ALGORITHM Algorithm to match new keypoints to existing
   * landmarks
   * @warning As this function uses the estimator it is not threadsafe.
   * @param estimator           SlidingWindowFilter.
   * @param params              Parameter struct.
   * @param currentFrameId      ID of the current frame that should be matched
   * against the last one.
   * @param usePoseUncertainty  Use the pose uncertainty for the matching.
   * @param removeOutliers      Remove outliers during RANSAC.
   * @return The number of matches in total.
   */
  template <class MATCHING_ALGORITHM>
  int matchToLastFrame(
      okvis::EstimatorBase& estimator,
      const okvis::VioParameters& params, const uint64_t currentFrameId,
      bool& rotationOnly,
      bool usePoseUncertainty = true, bool removeOutliers = true);
};

}  // namespace swift_vio
#endif  // INCLUDE_OKVIS_FRAMEWISE_DESCRIPTOR_FRONTEND_HPP_
