
/**
 * @file SimpleFrameRelativeAdapter.cpp
 * @brief Source file for the SimpleFrameRelativeAdapter class.
 * @author
 */

#include <feature_tracker/SimpleFrameRelativeAdapter.h>
#include <swift_vio/FrameMatchingStats.hpp>
#include <okvis/FrameTypedefs.hpp>

// cameras and distortions
#include <okvis/CameraModelSwitch.hpp>
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/cameras/FovDistortion.hpp>

// Constructor.
opengv::relative_pose::SimpleFrameRelativeAdapter::SimpleFrameRelativeAdapter(
    const okvis::Matches &matches, const swift_vio::CameraRig &nCameraSystem,
    swift_vio::ConstMultiFramePtr frameAPtr, size_t camIdA,
    swift_vio::ConstMultiFramePtr frameBPtr, size_t camIdB)
    : matches_(matches) {
  double fu1 = nCameraSystem.cameraGeometry(camIdA)->focalLengthU();
  size_t numKeypointsA = frameAPtr->numKeypoints(camIdA);

  double fu2 = nCameraSystem.cameraGeometry(camIdB)->focalLengthU();
  size_t numKeypointsB = frameBPtr->numKeypoints(camIdB);

  // resize members
  bearingVectors1_.resize(numKeypointsA);
  bearingVectors2_.resize(numKeypointsB);
  sigmaAngles1_.resize(numKeypointsA);
  sigmaAngles2_.resize(numKeypointsB);

  // precompute
  for (size_t k = 0; k < matches_.size(); ++k) {
    const size_t idx1 = matches_[k].idxA;
    const size_t idx2 = matches_[k].idxB;
    Eigen::Vector2d keypoint;
    double keypointStdDev;
    frameAPtr->getKeypoint(camIdA, idx1, keypoint);
    frameAPtr->getKeypointSize(camIdA, idx1, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;
    sigmaAngles1_[idx1] = sqrt(2) * keypointStdDev * keypointStdDev
        / (fu1 * fu1);

    nCameraSystem.cameraGeometry(camIdA)->backProject(
      keypoint, &bearingVectors1_[idx1]);

    bearingVectors1_[idx1].normalize();

    frameBPtr->getKeypoint(camIdB, idx2, keypoint);
    frameBPtr->getKeypointSize(camIdB, idx2, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;
    sigmaAngles2_[idx2] = sqrt(2) * keypointStdDev * keypointStdDev
        / (fu2 * fu2);

    nCameraSystem.cameraGeometry(camIdB)->backProject(
      keypoint, &bearingVectors2_[idx2]);

    bearingVectors2_[idx2].normalize();
  }
}

// Retrieve the bearing vector of a correspondence in viewpoint 1.
opengv::bearingVector_t opengv::relative_pose::SimpleFrameRelativeAdapter::getBearingVector1(
    size_t index) const {
  return bearingVectors1_[matches_[index].idxA];
}

// Retrieve the bearing vector of a correspondence in viewpoint 2.
opengv::bearingVector_t opengv::relative_pose::SimpleFrameRelativeAdapter::getBearingVector2(
    size_t index) const {
  return bearingVectors2_[matches_[index].idxB];
}

// Retrieve the position of a camera of a correspondence in viewpoint 1 seen from the origin of the viewpoint.
opengv::translation_t opengv::relative_pose::SimpleFrameRelativeAdapter::getCamOffset1(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Vector3d::Zero();
}

// Retrieve the rotation from a camera of a correspondence in viewpoint 1 to the viewpoint origin.
opengv::rotation_t opengv::relative_pose::SimpleFrameRelativeAdapter::getCamRotation1(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Matrix3d::Identity();
}

// Retrieve the position of a camera of a correspondence in viewpoint 2 seen from the origin of the viewpoint.
opengv::translation_t opengv::relative_pose::SimpleFrameRelativeAdapter::getCamOffset2(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Vector3d::Zero();
}

// Retrieve the rotation from a camera of a correspondence in viewpoint 2 to the viewpoint origin.
opengv::rotation_t opengv::relative_pose::SimpleFrameRelativeAdapter::getCamRotation2(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Matrix3d::Identity();
}

// Retrieve the number of correspondences.
size_t opengv::relative_pose::SimpleFrameRelativeAdapter::getNumberCorrespondences() const {
  return matches_.size();
}

// Obtain the angular standard deviation of the correspondence in frame 1 in [rad].
double opengv::relative_pose::SimpleFrameRelativeAdapter::getSigmaAngle1(
    size_t index) {
  return sigmaAngles1_[matches_[index].idxA];
}

// Obtain the angular standard deviation of the correspondence in frame 2 in [rad].
double opengv::relative_pose::SimpleFrameRelativeAdapter::getSigmaAngle2(
    size_t index) {
  return sigmaAngles2_[matches_[index].idxB];
}
