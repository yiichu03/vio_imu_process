
/**
 * @file StereoMatchingAlgorithm.cpp
 * @brief Source file for the StereoMatchingAlgorithm class.
 * @author
 */

#include <feature_tracker/StereoMatchingAlgorithm.hpp>

#include <okvis/IdProvider.hpp>
#include <okvis/cameras/CameraBase.hpp>

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <swift_vio/TwoViewGeometry.hpp>

// cameras and distortions
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/cameras/FovDistortion.hpp>

#include <opencv2/features2d/features2d.hpp> // for cv::KeyPoint

namespace swift_vio {
StereoMatchingAlgorithm::StereoMatchingAlgorithm(
    ConstMultiFramePtr nframe,
    std::shared_ptr<const CameraRig> cameraRig,
    size_t camIdA, size_t camIdB,
    BinaryDescriptorDistanceCallback &callback,
    float descriptorDistanceThreshold,
    float epipolarDistanceThreshold) :
  frameA_(nframe), nCameraSystem_(cameraRig),
  camIdA_(camIdA), camIdB_(camIdB), fA_(cameraRig->cameraGeometry(camIdA)->focalLengthU()),
  fB_(cameraRig->cameraGeometry(camIdB)->focalLengthU()),
  distanceCallback_(callback),
  distanceThreshold_(descriptorDistanceThreshold),
  epipolarDistanceThreshold_(epipolarDistanceThreshold),
  epipolarDistanceThresholdSquared_(epipolarDistanceThreshold * epipolarDistanceThreshold) {
}

StereoMatchingAlgorithm::~StereoMatchingAlgorithm() {

}

// This will be called exactly once for each call to DenseMatcher::match().
void StereoMatchingAlgorithm::doSetup() {
  // reset the match counter
  numMatches_ = 0;
//  numUncertainMatches_ = 0;

  T_CaCb_ = nCameraSystem_->getCameraExtrinsicPtr(camIdA_)->inverse() *
            *(nCameraSystem_->getCameraExtrinsicPtr(camIdB_));
  const size_t numA = frameA_->numKeypoints(camIdA_);
  if (skipA_.size() != numA) {
    skipA_.clear();
    skipA_.resize(numA, false);
  }
  raySigmasA_.resize(numA);
  backProjectionsA_.resize(3, numA);
  for (size_t k = 0; k < numA; ++k) {
    Eigen::Vector2d keypoint;
    frameA_->getKeypoint(camIdA_, k, keypoint);
    Eigen::Vector3d xy1;
    nCameraSystem_->cameraGeometry(camIdA_)->backProject(keypoint, &xy1);
    backProjectionsA_.col(k) = xy1;
    double keypointAStdDev;
    frameA_->getKeypointSize(camIdA_, k, keypointAStdDev);
    keypointAStdDev = 0.8 * keypointAStdDev / 12.0;
    raySigmasA_[k] = sqrt(sqrt(2)) * keypointAStdDev / fA_;
  }

  const size_t numB = frameA_->numKeypoints(camIdB_);
  if (skipB_.size() != numB) {
    skipB_.clear();
    skipB_.resize(numB, false);
  }

  raySigmasB_.resize(numB);
  backProjectionsB_.resize(3, numB);
  for (size_t k = 0; k < numB; ++k) {
    Eigen::Vector2d keypoint;
    frameA_->getKeypoint(camIdB_, k, keypoint);
    Eigen::Vector3d xy1;
    nCameraSystem_->cameraGeometry(camIdB_)->backProject(keypoint, &xy1);
    backProjectionsB_.col(k) = xy1;
    double keypointBStdDev;
    frameA_->getKeypointSize(camIdB_, k, keypointBStdDev);
    keypointBStdDev = 0.8 * keypointBStdDev / 12.0;
    raySigmasB_[k] = sqrt(sqrt(2)) * keypointBStdDev / fB_;
  }
}

size_t StereoMatchingAlgorithm::sizeA() const {
  return frameA_->numKeypoints(camIdA_);
}

size_t StereoMatchingAlgorithm::sizeB() const {
  return frameA_->numKeypoints(camIdB_);
}

// Set the distance threshold for which matches exceeding it will not be returned as matches.

void StereoMatchingAlgorithm::setDistanceThreshold(
    float distanceThreshold) {
  distanceThreshold_ = distanceThreshold;
}

// Get the distance threshold for which matches exceeding it will not be returned as matches.

float StereoMatchingAlgorithm::distanceThreshold() const {
  return distanceThreshold_;
}


void StereoMatchingAlgorithm::setEpipolarDistanceThreshold(
    float distanceThreshold) {
  epipolarDistanceThreshold_ = distanceThreshold;
  epipolarDistanceThresholdSquared_ = distanceThreshold * distanceThreshold;
}

bool StereoMatchingAlgorithm::verifyMatch(
    size_t indexA, size_t indexB) const {
  float chi2 = okvis::TwoViewGeometry::computeErrorEssentialMat(
      T_CaCb_, backProjectionsB_.col(indexB), backProjectionsA_.col(indexA), 1.0, 1.0,
      raySigmasB_[indexB], raySigmasA_[indexA]);
  if (chi2 > epipolarDistanceThresholdSquared_) {
    return false;
  } else {
//    // check if we can triangulate.
//    Eigen::Vector4d hP;
//    bool canBeInitialized;
//    bool valid = probabilisticStereoTriangulator_.stereoTriangulate(
//        indexA, indexB, hP, canBeInitialized,
//        std::max(raySigmasA_[indexA], raySigmasB_[indexB]));
//    if (valid && canBeInitialized) {
//      return true;
//    } else {
//      return false;
//    }
    return true;
  }
}

void StereoMatchingAlgorithm::reserveMatches(
    size_t numMatches) {
  matches_.reserve(numMatches);
}

size_t StereoMatchingAlgorithm::numMatches() {
  return numMatches_;
}

//size_t StereoMatchingAlgorithm::numUncertainMatches() {
//  return numUncertainMatches_;
//}

// This implementation is adapted from the Match2D2D branch of
// VioKeyframeWindowMatchingAlgorithm::setBestMatch().
void StereoMatchingAlgorithm::setBestMatch(
    size_t indexA, size_t indexB, double distance) {
  matches_.emplace_back(indexA, indexB, distance);

  // assign correspondences
//  uint64_t lmIdA = frameA_->landmarkId(camIdA_, indexA);
//  uint64_t lmIdB = frameA_->landmarkId(camIdB_, indexB);

  // check that not both are set
//  if (lmIdA != 0 && lmIdB != 0) {
//    if (lmIdA == lmIdB) {
//      return;
//    } else {
//      estimator_->mergeTwoLandmarks(lmIdA, lmIdB);
//      return;
//    }
//  }

  // re-triangulate...
  // potential 2d2d match - verify by triangulation
//  Eigen::Vector4d hP_Ca;
//  bool canBeInitialized;
//  bool valid = probabilisticStereoTriangulator_.stereoTriangulate(
//      indexA, indexB, hP_Ca, canBeInitialized,
//      std::max(raySigmasA_[indexA], raySigmasB_[indexB]));
//  if (!valid) {
//    return;
//  }

  // get the uncertainty
//  if (canBeInitialized) {  // know more exactly
//    Eigen::Matrix3d pointUOplus_A;
//    probabilisticStereoTriangulator_.getUncertainty(indexA, indexB, hP_Ca,
//                                                    pointUOplus_A,
//                                                    canBeInitialized);
//  }

  // check and adapt landmark status
//  bool insertA = lmIdA == 0;
//  bool insertB = lmIdB == 0;
//  bool insertHomogeneousPointParameterBlock = false;
//  uint64_t lmId = 0;  // 0 just to avoid warning
//  if (insertA && insertB) {
    // ok, we need to assign a new Id...
//    lmId = okvis::IdProvider::instance().newId();
//    frameA_->setLandmarkId(camIdA_, indexA, lmId);
//    frameB_->setLandmarkId(camIdB_, indexB, lmId);
//    lmIdA = lmId;
//    lmIdB = lmId;
    // and add it to the graph
//    insertHomogeneousPointParameterBlock = true;
//  } else {
//    if (!insertA) {
//      lmId = lmIdA;
//      if (!estimator_->isLandmarkAdded(lmId)) {
//        // add landmark and observation to the graph
//        insertHomogeneousPointParameterBlock = true;
//        insertA = true;
//      }
//    }
//    if (!insertB) {
//      lmId = lmIdB;
//      if (!estimator_->isLandmarkAdded(lmId)) {
//        // add landmark and observation to the graph
//        insertHomogeneousPointParameterBlock = true;
//        insertB = true;
//      }
//    }
//  }
  // add landmark to graph if necessary
//  if (insertHomogeneousPointParameterBlock) {
//    estimator_->addLandmark(lmId, T_WCa_ * hP_Ca);
//    OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmId),
//                      lmId<<" not added, bug");
//    estimator_->setLandmarkInitialized(lmId, canBeInitialized);
//  } else {
//    // update initialization status, set better estimate, if possible
//    // jhuai: check if the landmark has been initialized before initializing again
//    // so as to avoid reverting better estimates obtained in optimization.
//    if (!estimator_->isLandmarkInitialized(lmId) && canBeInitialized) {
//      estimator_->setLandmarkInitialized(lmId, true);
//      estimator_->setLandmark(lmId, T_WCa_ * hP_Ca);
//    }
//  }

  // in image A
//  const okvis::MapPoint &landmark = estimator_->getLandmarkUnsafe(lmId);
//  bool observedEarlierA = landmark.hasObservationInImage(mfIdA_, camIdA_);
//  if (insertA && !observedEarlierA) {  // ensure no double observations...
//    frameA_->setLandmarkId(camIdA_, indexA, lmId);
//    lmIdA = lmId;
//    // initialize in graph
//    OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmId),
//                      "landmark id=" << lmId<<" not added");
//    estimator_->addObservation<camera_geometry_t>(lmId, mfIdA_, camIdA_,
//                                                  indexA);
//  }

  // in image B
//  bool observedEarlierB = landmark.hasObservationInImage(mfIdB_, camIdB_);
//  if (insertB && !observedEarlierB) {  // ensure no double observations...
//    frameB_->setLandmarkId(camIdB_, indexB, lmId);
//    lmIdB = lmId;
//    // initialize in graph
//    OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmId),
//                      "landmark " << lmId << " not added");
//    estimator_->addObservation<camera_geometry_t>(lmId, mfIdB_, camIdB_,
//                                                  indexB);
//  }

  numMatches_++;
}
}  // namespace swift_vio
