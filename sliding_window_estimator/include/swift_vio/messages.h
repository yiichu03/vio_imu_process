/**
 * @file   messages.h
 * @author Jianzhu Huai
 *
 * @brief  Messages between FeatureAssciation frontend, estimation backend, and loop closure module.
 */

#ifndef INCLUDE_SWIFT_VIO_MESSAGES_H_
#define INCLUDE_SWIFT_VIO_MESSAGES_H_

#include <okvis/FrameTypedefs.hpp>
#include <okvis/ImuMeasurements.hpp>

#include <swift_vio/Frame.hpp>
#include <swift_vio/MapPoint.h>
#include <swift_vio/MultiFrame.hpp>
#include <swift_vio/memory.h>
#include <swift_vio/CameraIdentifier.h>

namespace swift_vio {
enum class RansacStatus {FewMatches = 0, FewInliers, PnPOk, FundamentalMatOk, RotationOnlyOk, NoncentralOk};

struct RelativePoseInfo {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraIdentifier cam1;  // usually a camera in an eariler nframe.
  CameraIdentifier cam2;  // usually a camera in the current nframe.
  okvis::kinematics::Transformation T_C1C2;
  RansacStatus status;

  RelativePoseInfo() {}

  RelativePoseInfo(const CameraIdentifier &_cam1,
                   const CameraIdentifier &_cam2,
                   const okvis::kinematics::Transformation &_T_C1C2,
                   RansacStatus _status)
      : cam1(_cam1), cam2(_cam2), T_C1C2(_T_C1C2), status(_status) {}
};

typedef Eigen::AlignedUnorderedMap<CameraIdentifier, RelativePoseInfo> RelativePoseMapType;

struct FrameRansacResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int numCorrespondences;
  std::vector<int> inliers; ///< inlier indices within matches
  float inlierRatio;

  RelativePoseInfo relPoseInfo;

  FrameRansacResult() {}
};

struct VisualMatcherOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name visualization members
  /// @{
  std::shared_ptr<const MultiFrame> currentNFrame; // Multiframe including keypoints, and keyframe status.
  // Make sure to shallow copy the original NFrame when assigning to this member for threadsafety.
  // The visualization will use the keypoints and images in the shallowed-copied NFrame.
  // Note the keypoints have been deeply copied.
  // The frontend will use the keypoints and descriptors of the original NFrame.

  // TODO(jhuai): remove this member and modify the visualization accordingly because images in a keyframe should be released in long term VIO.
  std::shared_ptr<const MultiFrame> closestKeyframe; // Cloest keyframe used in frontend feature association whose images are copied and used for visualization.
  /// @}

  /// @name localization members
  /// @{
  // homogeneous coordinates in the world frame of landmarks from the database which are observed in the current NFrame.
  Eigen::AlignedMap<uint64_t, PointAndVariance> databaseLandmarks;
  /// @}

  /// @name sequential camera and IMU data
  /// @{
  FeatureTrackMap featureTracks; // observations of landmarks due to the frontend feature association for the current NFrame.
  // Previous observations for earlier NFrames will have been moved to estimator where
  // they are merged into its landmarkMap.

  // the relative pose between the current NFrame and prior frames, estimated by 5-pt RANSAC.
  RelativePoseMapType relPoseToPriorFrames;

  okvis::ImuMeasurementDeque imuMeasurements; // measurements since the last NFrame to the currentNFrame

  bool rotationOnly;
  /// @}

//  VisualMatcherOutput() {}

  VisualMatcherOutput(std::shared_ptr<const MultiFrame> nframe,
                      const okvis::ImuMeasurementDeque &imuMeas)
      : currentNFrame(nframe), imuMeasurements(imuMeas) {}

  void addRelativePoseInfo(const FrameRansacResult &ransacResult) {
    relPoseToPriorFrames.emplace(ransacResult.relPoseInfo.cam1, ransacResult.relPoseInfo);
  }
};

void showMatchImages(const PointMap &pointMap,
    std::shared_ptr<const MultiFrame> frame,
    std::shared_ptr<const MultiFrame> keyframe);


/// @brief This struct contains the results of the optimization for ease of publication.
///        It is also used for publishing poses that have been propagated with the IMU
///        measurements.
struct OptimizationResults {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  okvis::Time stamp;                          ///< Timestamp of the optimized/propagated pose.
  okvis::kinematics::Transformation T_WS;     ///< The pose.
  okvis::SpeedAndBias speedAndBiases;         ///< The speeds and biases.
  Eigen::Matrix<double, 3, 1> omega_S;        ///< The rotational speed of the sensor.
  /// The relative transformation of the cameras to the sensor (IMU) frame
  std::vector<okvis::kinematics::Transformation,
      Eigen::aligned_allocator<okvis::kinematics::Transformation> > vector_of_T_SCi;

  MapPointVector landmarksVector;      ///< Vector containing the current landmarks.
  MapPointVector transferredLandmarks; ///< Vector of the landmarks that have been marginalized out.
  bool onlyPublishLandmarks;                  ///< Boolean to signalise the publisherLoop() that only the landmarks should be published
  uint64_t nframeId;
  bool isKeyframe;
  Eigen::Matrix<double, Eigen::Dynamic, 1> imuExtraParams_;
  ///< optimized extrinsics, projection intrinsic parameters, distortion, time delay, readout time of cameras.
  std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> variableCameraParams_;

  Eigen::Matrix<double, Eigen::Dynamic, 1> stateStd_;  ///< std. dev. of nav, imu, [cam extrinsic, intrinsic, td tr] parameters
};

};  // namespace swift_vio

#endif
