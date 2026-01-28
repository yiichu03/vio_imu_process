
#pragma once

#include <mutex>

#include <swift_vio/CameraFrontendBase.h>

#include <okvis/assert_macros.hpp>
#include <okvis/timing/Timer.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

namespace swift_vio {
/**
 * @brief The frontend that supports framewise matching using BRISK descriptors.
 *
 * As with okvis::Frontend, the ExternalFramewiseFrontend also initializes landmarks with
 * the (perhaps imprecisely calibrated) camera system.
 * These initial landmark positions will seed the landmarks in the smoothers,
 * but will not be used by filters which will triangulate landmarks based on feature tracks.
 *
 */
class ExternalFramewiseFrontend : public CameraFrontendBase {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor.
   * @param numCameras Number of cameras in the sensor configuration.
   */
  ExternalFramewiseFrontend(size_t numCameras, const FrontendOptions& frontendOptions);
  ~ExternalFramewiseFrontend() final;


  /// @name override methods.
  /// @{
  bool detectAndDescribe(size_t cameraIndex, std::shared_ptr<MultiFrame> frameOut) override;

  bool dataAssociation(
        std::shared_ptr<MultiFrame> currentNFrame,
        std::shared_ptr<const VisualSubmap> submap,
        VisualMatcherOutput *output) override;

  uint64_t oldestNFrameId() const override;
  /// @}

 protected:

   /**
   * @brief Decision whether a new frame should be keyframe or not.
   * Determine keyframes by checking the ratio of the area covered by feature matches and that by all features,
   * and checking the number of feature matches and the number of features in the area of matches.
   * @param currentFrame  Keyframe candidate.
   * @return True if it should be a new keyframe.
   */
  bool doWeNeedANewKeyframe(ConstMultiFramePtr currentFrame) const;

  swift_vio::FrontendOptions frontendOptions_;
  std::map<uint64_t, MultiFramePtr> multiFramePtrMap_;  ///< The processed multiframes by the frontend in chronical order, including the current NFrame.
  ///< It does not include those keyframes retrieved from the database.
  std::unordered_map<uint64_t, BareMultiFrame> multiFrame2LandmarkIdMap_;

  bool rotationOnly_;  ///< is the camera going through rotationOnly motion at the beginning?
  VisualMatcherOutput *output_; ///< intermediate result.
};

}  // namespace swift_vio
