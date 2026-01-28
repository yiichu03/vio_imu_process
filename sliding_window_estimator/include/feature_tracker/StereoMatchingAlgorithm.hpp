/**
 * @file StereoMatchingAlgorithm.hpp
 * @brief Header file for the StereoMatchingAlgorithm class.
 * @author 
 */

#ifndef INCLUDE_SWIFT_VIO_STEREOMATCHINGALGORITHM_HPP_
#define INCLUDE_SWIFT_VIO_STEREOMATCHINGALGORITHM_HPP_

#include <memory>

#include <okvis/DenseMatcher.hpp>
#include <okvis/MatchingAlgorithm.hpp>

#include <okvis/FrameTypedefs.hpp>
#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <feature_tracker/distance.h>

namespace swift_vio {

/**
 * \brief matching algorithm for frames of accurate relative poses,
 * e.g., frames from N cameras of a camera rig.
 */
class StereoMatchingAlgorithm : public okvis::MatchingAlgorithm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)


  enum MatchingTypes {
    Match3D2D = 1,  ///< Match 3D position of established landmarks to 2D keypoint position
    Match2D2D = 2   ///< Match 2D position of established landmarks to 2D keypoint position
  };

  /**
   * @brief Constructor.
   * @param estimator           EstimatorBase.
   * @param matchingType        Matching type. See MatchingTypes enum.
   * @param distanceThreshold   Descriptor distance threshold.
   * @param usePoseUncertainty  Use the pose uncertainty for matching.
   */
  StereoMatchingAlgorithm(ConstMultiFramePtr currentNFrame,
                          std::shared_ptr<const CameraRig> cameraRig,
                          size_t camIdA,
                          size_t camIdB,
                          BinaryDescriptorDistanceCallback &callback,
                          float descriptorDistanceThreshold,
                          float epipolarDistanceThreshold);

  virtual ~StereoMatchingAlgorithm();

  /// \brief This will be called exactly once for each call to DenseMatcher::match().
  virtual void doSetup();

  /// \brief What is the size of list A?
  virtual size_t sizeA() const;
  /// \brief What is the size of list B?
  virtual size_t sizeB() const;

  /// \brief Get the distance threshold for which matches exceeding it will not be returned as matches.
  virtual float distanceThreshold() const;
  /// \brief Set the distance threshold for which matches exceeding it will not be returned as matches.
  void setDistanceThreshold(float distanceThreshold);

  void setEpipolarDistanceThreshold(float distanceThreshold);

  /// \brief Should we skip the item in list A? This will be called once for each item in the list
  virtual bool skipA(size_t indexA) const {
    return skipA_[indexA];
  }

  /// \brief Should we skip the item in list B? This will be called many times.
  virtual bool skipB(size_t indexB) const {
    return skipB_[indexB];
  }

  void setSkipAList(const std::vector<bool> &skipA) final {
    OKVIS_ASSERT_EQ_DBG(Exception, frameA_->numKeypoints(camIdA_), skipA.size(),
                    "Incompatible skipA size " << skipA.size() << "!");
    skipA_ = skipA;
  }

  void setSkipBList(const std::vector<bool> &skipB) final {
    OKVIS_ASSERT_EQ_DBG(Exception, frameA_->numKeypoints(camIdB_), skipB.size(),
                    "Incompatible skipB size " << skipB.size() << "!");
    skipB_ = skipB;
  }

  /**
   * @brief Calculate the distance between two keypoints.
   * @param indexA Index of the first keypoint.
   * @param indexB Index of the other keypoint.
   * @return Distance between the two keypoint descriptors.
   * @remark Points that absolutely don't match will return float::max.
   */
  virtual float distance(size_t indexA, size_t indexB) const {
    OKVIS_ASSERT_LT_DBG(MatchingAlgorithm::Exception, indexA, sizeA(), "index A out of bounds");
    OKVIS_ASSERT_LT_DBG(MatchingAlgorithm::Exception, indexB, sizeB(), "index B out of bounds");
    const float dist = static_cast<float>(distanceCallback_(
        frameA_->keypointDescriptor(camIdA_, indexA),
        frameA_->keypointDescriptor(camIdB_, indexB)));

    if (dist < distanceThreshold_) {
      if (verifyMatch(indexA, indexB))
        return dist;
    }
    return std::numeric_limits<float>::max();
  }

  /// \brief Geometric verification of a match.
  bool verifyMatch(size_t indexA, size_t indexB) const;

  /// \brief A function that tells you how many times at maximum setBestMatch() will be called.
  virtual void reserveMatches(size_t numMatches);

  /// \brief At the end of the matching step, this function is called once
  ///        for each pair of matches discovered.
  virtual void setBestMatch(size_t indexA, size_t indexB, double distance);

  /// \brief Get the number of matches.
  size_t numMatches();
  /// \brief Get the number of uncertain matches.
//  size_t numUncertainMatches();

  /// \brief access the matching result.
  const okvis::Matches &getMatches() const {
    return matches_;
  }

  /// \brief assess the validity of the relative uncertainty computation.
//  bool isRelativeUncertaintyValid() {
//    return validRelativeUncertainty_;
//  }

  void setDescriptorDistanceCallback(BinaryDescriptorDistanceCallback &callback) {
    distanceCallback_ = callback;
  }

 private:
  /// \name Which frames to take
  /// \{
  ConstMultiFramePtr frameA_;
  std::shared_ptr<const CameraRig> nCameraSystem_;
  size_t camIdA_ = 0;
  size_t camIdB_ = 0;
  /// Focal length of camera used in frame A.
  double fA_ = 0;
  /// Focal length of camera used in frame B.
  double fB_ = 0;
  /// \}

  /// \brief Calculates the distance between two descriptors.
  BinaryDescriptorDistanceCallback distanceCallback_;
  /// Distances above this threshold will not be returned as matches.
  float distanceThreshold_;

  float epipolarDistanceThreshold_;

  float epipolarDistanceThresholdSquared_;

  /// \name Store some transformations that are often used
  /// \{
  /// use a fully relative formulation
  okvis::kinematics::Transformation T_CaCb_;
//  okvis::kinematics::Transformation T_CbCa_;
//  okvis::kinematics::Transformation T_SaCa_;
//  okvis::kinematics::Transformation T_SbCb_;
//  okvis::kinematics::Transformation T_WSa_;
//  okvis::kinematics::Transformation T_WSb_;
//  okvis::kinematics::Transformation T_SaW_;
//  okvis::kinematics::Transformation T_SbW_;
//  okvis::kinematics::Transformation T_WCa_;
//  okvis::kinematics::Transformation T_WCb_;
//  okvis::kinematics::Transformation T_CaW_;
//  okvis::kinematics::Transformation T_CbW_;
  /// \}

  okvis::Matches matches_;

  /// The number of matches.
  size_t numMatches_ = 0;
  /// The number of uncertain matches.
//  size_t numUncertainMatches_ = 0;

  Eigen::Matrix<double, 3, -1> backProjectionsA_;
  Eigen::Matrix<double, 3, -1> backProjectionsB_;

  /// temporarily store all projections
//  Eigen::Matrix<double, Eigen::Dynamic, 2> projectionsIntoB_;
  /// temporarily store all projection uncertainties
//  Eigen::Matrix<double, Eigen::Dynamic, 2> projectionsIntoBUncertainties_;

  /// Should keypoint[index] in frame A be skipped
  std::vector<bool> skipA_;
  /// Should keypoint[index] in frame B be skipped
  std::vector<bool> skipB_;

  /// Camera center of frame A.
//  Eigen::Vector3d pA_W_;
  /// Camera center of frame B.
//  Eigen::Vector3d pB_W_;

  /// Temporarily store ray sigmas of frame A.
  std::vector<double> raySigmasA_;
  /// Temporarily store ray sigmas of frame B.
  std::vector<double> raySigmasB_;

//  bool validRelativeUncertainty_ = false;
//  bool usePoseUncertainty_ = false;

};

}  // namespace swift_vio

#endif /* INCLUDE_SWIFT_VIO_STEREOMATCHINGALGORITHM_HPP_ */
