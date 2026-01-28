
#ifndef INCLUDE_COUPLED_SIMULATION_FRONTEND_HPP_
#define INCLUDE_COUPLED_SIMULATION_FRONTEND_HPP_

#include <mutex>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <okvis/DenseMatcher.hpp>
#include <okvis/EstimatorBase.hpp>
#include <okvis/VioFrontendInterface.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/timing/Timer.hpp>
#include <okvis/triangulation/ProbabilisticStereoTriangulator.hpp>

#include <swift_vio/memory.h>
#include <simul/SimParameters.h>

namespace simul {

/**
 * @brief A frontend for simulation with predefined landmarks.
 * It couples frontend and backend estimator, presumably to take advantage of
 * triangulated landmarks from the estimator.
 */
class CoupledSimulationFrontend : public okvis::VioFrontendInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  struct AssociatedFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::shared_ptr<okvis::MultiFrame> nframe_;
    okvis::kinematics::Transformation pose_; // pose of the body frame at the nframe epoch.
    // mapping from the landmark index in the landmark grid to the keypoint index in a frame of the nframe.
    std::vector<std::unordered_map<size_t, size_t>> keypointIndices_;
    bool isKeyframe_;

    AssociatedFrame(std::shared_ptr<okvis::MultiFrame> nframe, const okvis::kinematics::Transformation& pose,
                    const std::vector<std::unordered_map<size_t, size_t>>& keypointIndices, bool isKeyframe) :
      nframe_(nframe), pose_(pose), keypointIndices_(keypointIndices), isKeyframe_(isKeyframe) {

    }
  };

  /**
   * @brief Constructor.
   * @param numCameras Number of cameras in the sensor configuration.
   */
  CoupledSimulationFrontend(
      const std::vector<Eigen::Vector4d,
                        Eigen::aligned_allocator<Eigen::Vector4d>>
          &homogeneousPoints,
      const std::vector<uint64_t> &lmIds, size_t numCameras,
      const SimFrontendOptions &options);

  virtual ~CoupledSimulationFrontend() {}

  ///@{

  /**
   * @brief given keypoints associated with landmarks, add features to the estimator.
   * @param estimator
   * @param[in] keypointIndices indices of keypoints for landmarks observed by frames.
   * @param[in, out] nframes
   * @param[out] asKeyframe
   * @return number of tracked features.
   */
  int dataAssociationAndInitialization(
      okvis::EstimatorBase& estimator, const std::vector<std::unordered_map<size_t, size_t>>& keypointIndices,
      std::shared_ptr<okvis::MultiFrame> nframes, bool* asKeyframe);

  bool
  detectAndDescribe(size_t /*cameraIndex*/,
                    std::shared_ptr<okvis::MultiFrame> /*frameOut*/,
                    const okvis::kinematics::Transformation & /*T_WC*/,
                    const std::vector<cv::KeyPoint> * /*keypoints*/) final {
    return true;
  }

  bool dataAssociationAndInitialization(
      okvis::EstimatorBase & /*estimator*/,
      const okvis::VioParameters & /*params*/,
      std::shared_ptr<okvis::MultiFrame> /*nframes*/,
      bool * /*asKeyframe*/) final {
    return true;
  }
  ///@}

  static const double fourthRoot2_; // sqrt(sqrt(2))

 private:
  SimFrontendOptions options_;
  Eigen::AlignedDeque<AssociatedFrame> nframeList_;

  okvis::kinematics::Transformation previousKeyframePose_;

  // scene landmarks
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      homogeneousPoints_;
  std::vector<uint64_t> lmIds_;

  /**
   * @brief Decision whether a new frame should be keyframe or not.
   * @param estimator     const reference to the estimator.
   * @param currentFrame  Keyframe candidate.
   * @return True if it should be a new keyframe.
   */
  bool doWeNeedANewKeyframe(
      const okvis::EstimatorBase& estimator,
      std::shared_ptr<okvis::MultiFrame> currentFrame) const;

  /**
   * @brief find keypoint matches between two multiframes.
   * @param previousKeypointIndices mapping from landmark index in the landmark grid to keypoint index in the previous multiframe.
   * @param currentKeypointIndices
   * @param prevFrameId
   * @param currFrameId
   * @param landmarkMatches
   * @return
   */
  int matchToFrame2(
      const std::vector<std::unordered_map<size_t, size_t>>& previousKeypointIndices,
      const std::vector<std::unordered_map<size_t, size_t>>& currentKeypointIndices,
      const uint64_t prevFrameId, const uint64_t currFrameId,
      std::vector<LandmarkKeypointMatch>* landmarkMatches) const;

  /**
   * @brief given landmark matches between two frames, add proper constraints to the estimator
   * @param estimator
   * @param prevFrames
   * @param currFrames
   * @param landmarkMatches the list of keypoint match between the two frames of one landmark
   */
  template <class CAMERA_GEOMETRY_T>
  int addMatchToEstimator(
      okvis::EstimatorBase& estimator,
      std::shared_ptr<okvis::MultiFrame> prevFrames,
      std::shared_ptr<okvis::MultiFrame> currFrames,
      const std::vector<LandmarkKeypointMatch>& landmarkMatches) const;
};

}  // namespace simul

#endif  // INCLUDE_COUPLED_SIMULATION_FRONTEND_HPP_
