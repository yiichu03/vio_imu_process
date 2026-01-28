
/**
 * @file SimpleFrameRelativeAdapter.hpp
 * @brief Header file for the SimpleFrameRelativeAdapter class.
 * @author 
 */

#ifndef INCLUDE_SWIFT_VIO_BAREFRAMERELATIVEADAPTER_H_
#define INCLUDE_SWIFT_VIO_BAREFRAMERELATIVEADAPTER_H_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/FrameTypedefs.hpp>

/**
 * \brief Namespace for classes extending the OpenGV library.
 */
namespace opengv {
/**
 * \brief The namespace for the relative pose methods.
 */
namespace relative_pose {

/// \brief Adapter for relative pose RANSAC (2D2D)
class SimpleFrameRelativeAdapter : public RelativeAdapterBase {
 private:
  using RelativeAdapterBase::_t12;
  using RelativeAdapterBase::_R12;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor
   * @param estimator     The estimator.
   * @param nCameraSystem Camera configuration and parameters.
   * @param multiFrameIdA The first multiframe.
   * @param camIdA        The camera index for the first multiframe
   *                      in order to access the relevant frame.
   * @param multiFrameIdB The second multiframe.
   * @param camIdB        The camera index for the second multiframe.
   *                      in order to access the relevant frame.
   */
  SimpleFrameRelativeAdapter(const okvis::Matches &matches,
                             const swift_vio::CameraRig &nCameraSystem,
                             swift_vio::ConstMultiFramePtr frameAPtr,
                             size_t camIdA,
                             swift_vio::ConstMultiFramePtr frameBPtr,
                             size_t camIdB);

  virtual ~SimpleFrameRelativeAdapter() {
  }

  /// @name Algorithm input
  /// @{

  /**
   * \brief Retrieve the bearing vector of a correspondence in viewpoint 1.
   * \param[in] index The serialized index of the correspondence.
   * \return The corresponding bearing vector.
   */
  virtual opengv::bearingVector_t getBearingVector1(size_t index) const;
  /**
   * \brief Retrieve the bearing vector of a correspondence in viewpoint 2.
   * \param[in] index The serialized index of the correspondence.
   * \return The corresponding bearing vector.
   */
  virtual opengv::bearingVector_t getBearingVector2(size_t index) const;
  /**
   * \brief Retrieve the position of a camera of a correspondence in viewpoint
   *        1 seen from the origin of the viewpoint.
   * \param[in] index The serialized index of the correspondence.
   * \return The position of the corresponding camera seen from the viewpoint
   *         origin.
   */
  virtual opengv::translation_t getCamOffset1(size_t index) const;
  /**
   * \brief Retrieve the rotation from a camera of a correspondence in
   *        viewpoint 1 to the viewpoint origin.
   * \param[in] index The serialized index of the correspondence.
   * \return The rotation from the corresponding camera back to the viewpoint
   *         origin.
   */
  virtual opengv::rotation_t getCamRotation1(size_t index) const;
  /**
   * \brief Retrieve the position of a camera of a correspondence in viewpoint
   *        2 seen from the origin of the viewpoint.
   * \param[in] index The serialized index of the correspondence.
   * \return The position of the corresponding camera seen from the viewpoint
   *         origin.
   */
  virtual opengv::translation_t getCamOffset2(size_t index) const;
  /**
   * \brief Retrieve the rotation from a camera of a correspondence in
   *        viewpoint 2 to the viewpoint origin.
   * \param[in] index The serialized index of the correspondence.
   * \return The rotation from the corresponding camera back to the viewpoint
   *         origin.
   */
  virtual opengv::rotation_t getCamRotation2(size_t index) const;
  /**
   * \brief Retrieve the number of correspondences.
   * \return The number of correspondences.
   */
  virtual size_t getNumberCorrespondences() const;

  /// @}

  // custom:
  /**
   * @brief Obtain the angular standard deviation of the correspondence in frame 1 in [rad].
   * @param index The index of the correspondence.
   * @return The standard deviation in [rad].
   */
  double getSigmaAngle1(size_t index);
  /**
   * @brief Obtain the angular standard deviation of the correspondence in frame 2 in [rad].
   * @param index The index of the correspondence.
   * @return The standard deviation in [rad].
   */
  double getSigmaAngle2(size_t index);
  /**
   * @brief Get the keypoint index in frame 1 of a correspondence.
   * @param index The serialized index of the correspondence.
   * @return The keypoint index of the correspondence in frame 1.
   */
  size_t getMatchKeypointIdxA(size_t index) {
    return matches_.at(index).idxA;
  }
  /**
   * @brief Get the keypoint index in frame 2 of a correspondence.
   * @param index The serialized index of the correspondence.
   * @return The keypoint index of the correspondence in frame 2.
   */
  size_t getMatchKeypointIdxB(size_t index) {
    return matches_.at(index).idxB;
  }
  /**
   * \brief Retrieve the weight of a correspondence. The weight is supposed to
   *        reflect the quality of a correspondence, and typically is between
   *        0 and 1.
   * \warning This is not implemented and always returns 1.0.
   */
  virtual double getWeight(size_t) const {
    return 1.0;
  }  // TODO : figure out, if this is needed

 private:
  /// The bearing vectors of the correspondences in frame 1.
  opengv::bearingVectors_t bearingVectors1_;
  /// The bearing vectors of the correspondences in frame 2.
  opengv::bearingVectors_t bearingVectors2_;
  /// The matching keypoints of both frames.
  const okvis::Matches &matches_;

  // also store individual uncertainties
  /// The standard deviations of the bearing vectors of frame 1 in [rad].
  std::vector<double> sigmaAngles1_;
  /// The standard deviations of the bearing vectors of frame 2' in [rad].
  std::vector<double> sigmaAngles2_;

};

}
}

#endif /* INCLUDE_SWIFT_VIO_BAREFRAMERELATIVEADAPTER_H_ */
