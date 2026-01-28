
/**
 * @file VioVisualizer.hpp
 * @brief Header file for the VioVisualizer class.
 */

#ifndef INCLUDE_SWIFT_VIO_VIOVISUALIZER_HPP_
#define INCLUDE_SWIFT_VIO_VIOVISUALIZER_HPP_

#include <opencv2/highgui.hpp>
#include <okvis/assert_macros.hpp>

#include <okvis/Parameters.hpp>
#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <okvis/FrameTypedefs.hpp>

namespace swift_vio {

/**
 * @brief This class is responsible to visualize the matching results
 */
class VioVisualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// @brief This struct contains the relevant data for visualizing
  struct VisualizationData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<VisualizationData> Ptr;
    okvis::ObservationVector observations;    ///< Vector containing all the keypoint observations.
    std::shared_ptr<const MultiFrame> currentFrames; ///< Current multiframe.
    std::shared_ptr<const MultiFrame> keyFrames;     ///< Current keyframe. Its images will be copied, other members intact.
    okvis::kinematics::Transformation T_WS_keyFrame;  ///< Pose of the current keyframe
  };

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  VioVisualizer();

  /**
   * @brief Constructor.
   * @param parameters Parameters and settings.
   */
  VioVisualizer(const okvis::VioParameters& parameters,
                const std::string viewerNamePrefix="Reprojected landmarks for camera");
  virtual ~VioVisualizer();

  /**
   * @brief Initialise parameters. Called in constructor.
   * @param parameters Parameters and settings.
   */
  void updateParameters(const okvis::VioParameters& parameters);

  const okvis::VioParameters &parameters() const {
    return parameters_;
  }

  okvis::VioParameters &parametersMutable() {
    return parameters_;
  }

  void setCameraSystem(const swift_vio::CameraRig &cameraRig);

  void setupDisplays();

  /**
   * @brief Show the current frames with the current keyframe and all its matches.
   * @param data Visualization data containing all the info.
   */
  void showDebugImages(VisualizationData::Ptr& data);

  /**
   * @brief Circles all keypoints in the current frame, links the matching ones to
   *        the current keyframe and returns the result.
   * @param data Visualization data.
   * @param image_number Index of the frame to display.
   * @return OpenCV matrix with the resulting image.
   */
  cv::Mat drawMatches(VisualizationData::Ptr& data, size_t image_number);
  
  /**
   * @brief Circle keypoints with main direction and color with track length.
   * @param data
   * @param image_number
   * @return
   */
  cv::Mat drawColoredKeypoints(VisualizationData::Ptr& data, size_t image_number) const;

  inline std::string viewerNamePrefix() const {
    return viewerNamePrefix_;
  }

 private:
  /**
   * @brief Circles all keypoints in the current frame and returns the result.
   * @param data Visualization data.
   * @param cameraIndex Index of the frame to display.
   * @return OpenCV matrix with the resulting image.
   */
  cv::Mat drawKeypoints(VisualizationData::Ptr& data, size_t cameraIndex);

  /// Parameters and settings.
  okvis::VioParameters parameters_;
  std::string viewerNamePrefix_;
};

/**
 * @brief downscaleImage
 * @param in
 * @param downscaleFactor has to be power of 2.
 * @return downscaled image.
 */
cv::Mat downscaleImage(cv::Mat in, int downscaleFactor);

} /* namespace swift_vio */

#endif /* INCLUDE_SWIFT_VIO_VIOVISUALIZER_HPP_ */
