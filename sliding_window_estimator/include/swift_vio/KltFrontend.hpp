
#ifndef INCLUDE_SWIFT_VIO_KLT_FRONTEND_HPP_
#define INCLUDE_SWIFT_VIO_KLT_FRONTEND_HPP_

#include <okvis/VioFrontendInterface.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/timing/Timer.hpp>

#include <feature_tracker/FeatureTracker.h>


namespace swift_vio {
/**
 * @brief The frontend that supports framewise matching using optic flow.
 *
 * As with okvis::Frontend, the KltFrontend also initializes landmarks with
 * the (perhaps imprecisely calibrated) camera system.
 * These initial landmark positions will seed the landmarks in the smoothers,
 * but will not be used by filters which will triangulate landmarks based on feature tracks.
 *
 */
class KltFrontend : public okvis::VioFrontendInterface {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor.
   * @param numCameras Number of cameras in the sensor configuration.
   */
  KltFrontend(size_t numCameras, const FrontendOptions& frontendOptions);
  ~KltFrontend() final;

  ///@{

  bool detectAndDescribe(size_t /*cameraIndex*/,
                         std::shared_ptr<okvis::MultiFrame> /*frameOut*/,
                         const okvis::kinematics::Transformation &/*T_WC*/,
                         const std::vector<cv::KeyPoint> */*keypoints*/) final {
    return true;
  }

  bool dataAssociationAndInitialization(
      okvis::EstimatorBase& estimator,
      const okvis::VioParameters& params,
      std::shared_ptr<okvis::MultiFrame> nframes, bool* asKeyframe) final;
 
  ///@}

 private:
  swift_vio::FrontendOptions frontendOptions_;

  feature_tracker::FeatureTracker featureTracker_;

  template <class CAMERA_GEOMETRY_T>
  int matchToLastFrameKLT(okvis::EstimatorBase& estimator,
                          const okvis::VioParameters& params,
                          std::shared_ptr<okvis::MultiFrame> nframes,
                          bool usePoseUncertainty = true,
                          bool removeOutliers = true);

  /**
   * @brief Check the relative motion between current frame and an older frame,
   * and record the relative motion in the estimator. If overlap area or
   * matching ratio in the overlap area between the two frames does not meet
   * thresholds, the current frame will be chosen as a keyframe, i.e.,
   * asKeyframe is set true.
   * @param estimator
   * @param params
   * @param currentFrameId
   * @param olderFrameId
   * @param[out] rotationOnly whether the relative motion between the two frames is only rotation?
   * @param removeOutliers
   * @param[out] asKeyframe
   * @return
   */
  int checkMotionByRansac2d2d(okvis::EstimatorBase& estimator,
                              const okvis::VioParameters& params,
                              uint64_t currentFrameId, uint64_t olderFrameId,
                              bool *rotationOnly,
                              bool removeOutliers, bool* asKeyframe);
};

}  // namespace swift_vio
#endif  // INCLUDE_SWIFT_VIO_KLT_FRONTEND_HPP_
