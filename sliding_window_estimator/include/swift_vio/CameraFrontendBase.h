#ifndef CAMERAFRONTENDBASE_H
#define CAMERAFRONTENDBASE_H

#include <swift_vio/messages.h>
#include <atomic>
#include <okvis/VioParametersReader.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <swift_vio/VisualMap.h>

namespace swift_vio {
class CameraFrontendBase {
 public:
  CameraFrontendBase(size_t numCameras) :
    numNFrames_(0), numKeyframes_(0),
    numCameras_(numCameras) {
  }

  virtual ~CameraFrontendBase() {
  }
  /// @name
  /// In the derived class, the following methods (and nothing else) have to be implemented:
  ///@{
  /**
   * @brief Detection and descriptor extraction on a per image basis.
   * @param cameraIndex   Index of camera to do detection and description.
   * @param frameOut      Multiframe containing the frames.
   *                      Resulting keypoints and descriptors are saved in here.
   * @return True if successful.
   */
  virtual bool detectAndDescribe(
      size_t cameraIndex, std::shared_ptr<MultiFrame> frameOut) = 0;

  void setCameraSystem(const swift_vio::CameraRig cameraRig) {
    nCameraSystem_ = cameraRig.deepCopyPtr();
  }

  void setCameraSystem(const okvis::cameras::NCameraSystem cameraRig) {
    nCameraSystem_ = swift_vio::CameraRig::deepCopyPtr(cameraRig);
  }

  void updateCameraSystem(std::shared_ptr<const swift_vio::CameraRig> cameraRig) {
    cameraRig->assignTo(nCameraSystem_.get());
  }

  std::shared_ptr<const swift_vio::CameraRig> cameraSystem() const {
    return nCameraSystem_;
  }

  /**
   * @brief oldestNFrameId
   * @warning This function assumes sequential processing of NFrames.
   * It will break when NFrames from database are involved.
   * @return
   */
  virtual uint64_t oldestNFrameId() const { return 0; }

  /**
   * @brief Matching
   * @param estimator       EstimatorBase.
   * @param params          Configuration parameters.
   * @param[in] currentNFrame     Multiframe including the descriptors of all the keypoints.
   * @param[in] submap      The visual submap from the database including previous keyframes and landmarks.
   * @param[in, out] output     Feature matches, asKeyframe etc.
   * @return True if successful.
   */
  virtual bool dataAssociation(
      std::shared_ptr<MultiFrame> currentNFrame,
      std::shared_ptr<const VisualSubmap> submap,
      VisualMatcherOutput *output) = 0;

  ///@}

  int numNFrames() const {
    return numNFrames_;
  }

  int numKeyframes() const {
    return numKeyframes_;
  }

  std::atomic_int numNFrames_;    ///< Number processed NFrames.
  std::atomic_int numKeyframes_;  ///< Number processed keyframes

  const size_t numCameras_;   ///< Number of cameras in the configuration.
  std::shared_ptr<CameraRig> nCameraSystem_;
};
}  // namespace swift_vio
#endif // CAMERAFRONTENDBASE_H
