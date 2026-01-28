
#ifndef INCLUDE_OKVIS_SIMULATION_FRONTEND_HPP_
#define INCLUDE_OKVIS_SIMULATION_FRONTEND_HPP_

#include <mutex>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <okvis/DenseMatcher.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/timing/Timer.hpp>
#include <okvis/triangulation/ProbabilisticStereoTriangulator.hpp>

#include <simul/SimParameters.h>
#include <swift_vio/CameraFrontendBase.h>
#include <swift_vio/memory.h>

/// \brief okvis Main namespace of this package.
namespace simul {

/**
 * @brief A frontend for simulation with predefined landmarks.
 */
class SimulationFrontend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  struct AssociatedFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::shared_ptr<swift_vio::MultiFrame> nframe_;
    okvis::kinematics::Transformation pose_; // pose of the body frame at the nframe epoch.
    // mapping from the landmark index in the landmark grid to the keypoint index in a frame of the nframe.
    std::vector<std::unordered_map<size_t, size_t>> keypointIndices_;
    bool isKeyframe_;

    AssociatedFrame(std::shared_ptr<swift_vio::MultiFrame> nframe, const okvis::kinematics::Transformation& pose,
                    const std::vector<std::unordered_map<size_t, size_t>>& keypointIndices, bool isKeyframe) :
      nframe_(nframe), pose_(pose), keypointIndices_(keypointIndices), isKeyframe_(isKeyframe) {

    }
  };

  /**
   * @brief Constructor.
   * @param numCameras Number of cameras in the sensor configuration.
   */
  SimulationFrontend(
      const std::vector<Eigen::Vector4d,
                        Eigen::aligned_allocator<Eigen::Vector4d>>
          &homogeneousPoints,
      const std::vector<uint64_t> &lmIds, size_t numCameras,
      const SimFrontendOptions &options);

  virtual ~SimulationFrontend() {}

  ///@{
  int dataAssociation(
      std::shared_ptr<swift_vio::MultiFrame> nframes, const okvis::kinematics::Transformation & T_WB,
      const std::vector<std::unordered_map<size_t, size_t>>& keypointIndices,
      swift_vio::VisualMatcherOutput *output);

  int numNFrames() const {
    return numNFrames_;
  }

  int numKeyframes() const {
    return numKeyframes_;
  }
  ///@}

  static const double fourthRoot2_; // sqrt(sqrt(2))

 private:

  SimFrontendOptions options_;
  Eigen::AlignedDeque<AssociatedFrame> nframeList_;
  std::unordered_map<uint64_t, swift_vio::BareMultiFrame> multiFrame2LandmarkIdMap_;

  okvis::kinematics::Transformation previousKeyframePose_;

  // scene landmarks
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      homogeneousPoints_;
  std::vector<uint64_t> lmIds_;

  std::atomic_int numNFrames_;    ///< Number processed NFrames.
  std::atomic_int numKeyframes_;  ///< Number processed keyframes

  const size_t numCameras_;   ///< Number of cameras in the configuration.

  /**
   * @brief Decision whether a new frame should be keyframe or not.
   * @param current_T_WB
   * @return True if it should be a new keyframe.
   */
  bool doWeNeedANewKeyframe(
      const okvis::kinematics::Transformation &current_T_WB) const;

  /**
   * @brief find keypoint matches between two multiframes.
   * @param previousKeypointIndices mapping from landmark index in the landmark grid to keypoint index in the previous multiframe.
   * @param currentKeypointIndices
   * @param prevFrameId
   * @param currFrameId
   * @param landmarkMatches
   * @return
   */
  int matchToFrame(
      const std::vector<std::unordered_map<size_t, size_t>>& previousKeypointIndices,
      const std::vector<std::unordered_map<size_t, size_t>>& currentKeypointIndices,
      const uint64_t prevFrameId, const uint64_t currFrameId,
      std::vector<LandmarkKeypointMatch>* landmarkMatches) const;

  /**
   * @brief given landmark matches between two frames, update feature tracks and
   * landmark Ids stored in multiFrame2LandmarkIdMap_
   * @param prevFrames
   * @param currFrames
   * @param landmarkMatches the list of keypoint match between the two frames of one landmark
   * @param[out] featureTracks
   */
  int updateFeatureTracks(
      std::shared_ptr<swift_vio::MultiFrame> prevFrames,
      std::shared_ptr<swift_vio::MultiFrame> currFrames,
      const std::vector<LandmarkKeypointMatch>& landmarkMatches,
      swift_vio::FeatureTrackMap *featureTracks);
};

}  // namespace simul

#endif  // INCLUDE_OKVIS_SIMULATION_FRONTEND_HPP_
