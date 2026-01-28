#include <swift_vio/KltFrontend.hpp>
#include <swift_vio/implementation/KltFrontend.hpp>

#include <swift_vio/FrameTranslationOnlySacProblem.hpp>

// cameras and distortions
#include <okvis/CameraModelSwitch.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/FovDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/timing/Timers.hpp>
// Kneip RANSAC
#include <opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp>

#include <opengv/relative_pose/FrameRelativeAdapter.hpp>

#include <opengv/sac/Ransac.hpp>

using namespace okvis;

namespace swift_vio {
KltFrontend::KltFrontend(size_t numCameras, const FrontendOptions& frontendOptions)
    : VioFrontendInterface(numCameras), frontendOptions_(frontendOptions) {
}

KltFrontend::~KltFrontend() {
  LOG(INFO) << "KltFrontend associated " << numNFrames_
            << " NFrames, and selected " << numKeyframes_ << " keyframes.";
}

bool KltFrontend::dataAssociationAndInitialization(
    okvis::EstimatorBase& estimator,
    const okvis::VioParameters& params,
    std::shared_ptr<okvis::MultiFrame> nframes,
    bool* asKeyframe) {
  // match new keypoints to existing landmarks/keypoints
  // initialise new landmarks (states)
  // outlier rejection by consistency check
  // RANSAC (2D2D / 3D2D)
  // decide keyframe
  // left-right stereo match & init

  // find distortion type
  okvis::cameras::DistortionType distortionType = params.nCameraSystem
      .distortionType(0);
  for (size_t i = 1; i < params.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception,
                      distortionType == params.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet");
  }
  int num3dMatches = 0;
    int requiredMatches = 5;
    // match to last frame
    okvis::TimerSwitchable matchToLastFrameTimer("2.4.2 matchToLastFrameKLT");

#define DISTORTION_MODEL_CASE(camera_geometry_t)                               \
  num3dMatches = matchToLastFrameKLT<camera_geometry_t>(                       \
      estimator, params, nframes, false);

    switch (distortionType) { DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES }

#undef DISTORTION_MODEL_CASE

    matchToLastFrameTimer.stop();

    if (num3dMatches <= requiredMatches) {
      LOG(WARNING) << "Tracking last frame failure. Number of 3d2d-matches: " << num3dMatches;
    }
    if (estimator.numFrames() > 1) {
      uint64_t currentFrameId = nframes->id();
      uint64_t lastKeyframeId = estimator.currentKeyframeId();
      bool removeOutliers = false;
      bool rotationOnly = false;
      checkMotionByRansac2d2d(estimator, params, currentFrameId, lastKeyframeId, &rotationOnly,
                              removeOutliers, asKeyframe);
      if (!isInitialized_) {
        if (!rotationOnly) {
          isInitialized_ = true;
          LOG(INFO) << "Initialized frontend from KLT matching.";
        }
      }
    } else {
      *asKeyframe = true;
    }
    ++numNFrames_;
    numKeyframes_ += (*asKeyframe ? 1 : 0);
    return true;
}

// Match a new multiframe to the last frame.
template <class CAMERA_GEOMETRY_T>
int KltFrontend::matchToLastFrameKLT(
    okvis::EstimatorBase& estimator,
    const okvis::VioParameters& /*params*/,
    std::shared_ptr<okvis::MultiFrame> nframes,
    bool /*usePoseUncertainty*/, bool /*removeOutliers*/) {
  int retCtr = 0;
  if (estimator.numFrames() == 1) {
    configureFeatureTracker<CAMERA_GEOMETRY_T>(nframes, estimator, &featureTracker_);
    featureTracker_.initialize();
  } else {
    uint64_t fIdB = nframes->id();
    okvis::kinematics::Transformation T_WSb;
    estimator.get_T_WS(fIdB, T_WSb);

    uint64_t fIdA = estimator.frameIdByAge(1);
    okvis::kinematics::Transformation T_WSa;
    estimator.get_T_WS(fIdA, T_WSa);

    std::vector<cv::Matx33f> R_CkCkm1_list;
    for (size_t camId = 0; camId < nframes->numFrames(); ++camId) {
      okvis::kinematics::Transformation T_SC;
      estimator.getCameraSensorExtrinsics(camId, T_SC);
      Eigen::Quaterniond q_CkCkm1 =
          (T_WSb.q() * T_SC.q()).conjugate() * (T_WSa.q() * T_SC.q());
      Eigen::Matrix3f R_CkCkm1 = q_CkCkm1.toRotationMatrix().cast<float>();
      cv::Matx33f mat_R_CkCkm1;
      cv::eigen2cv(R_CkCkm1, mat_R_CkCkm1);
      R_CkCkm1_list.emplace_back(mat_R_CkCkm1);
    }
    featureTracker_.setRelativeOrientation(R_CkCkm1_list);
  }

  featureTracker_.stereoCallback(
      nframes->image(0),
      nframes->numFrames() > 1 ?
          nframes->image(1) :
          cv::Mat(),
      feature_tracker::MessageHeader{nframes->timestamp()});

//  featureTracker_.drawFeaturesMono();

  std::vector<feature_tracker::FeatureIDType> curr_ids(0);
  featureTracker_.getCurrentFeatureIds(&curr_ids);
  retCtr += curr_ids.size();

  std::vector<std::vector<cv::KeyPoint>> curr_keypoints(2);
  featureTracker_.getCurrentKeypoints(&curr_keypoints[0], &curr_keypoints[1]);

  featureTracker_.prepareForNextFrame(); // clear many things for next frame

  for (size_t im = 0; im < nframes->numFrames(); ++im) {
    nframes->resetKeypoints(im, curr_keypoints[im]);
  }

  addConstraintToEstimator<CAMERA_GEOMETRY_T>(
      curr_ids, nframes, estimator);
  return retCtr;
}

int KltFrontend::checkMotionByRansac2d2d(okvis::EstimatorBase &estimator,
                                            const okvis::VioParameters &params,
                                            uint64_t currentFrameId,
                                            uint64_t olderFrameId,
                                            bool *rotationOnly,
                                            bool removeOutliers,
                                            bool *asKeyframe) {
  const size_t numCameras = params.nCameraSystem.numCameras();
  RelativeMotionType rmt = UNCERTAIN_MOTION;
  size_t totalInlierNumber = 0;
  bool rotation_only_success = false;
  bool translation_only_success = false;
  double maxOverlap = 0.0;
  double maxMatchRatio = 0.0;
  std::shared_ptr<okvis::MultiFrame> frameBPtr =
      estimator.multiFrame(currentFrameId);
  // run relative RANSAC
  for (size_t im = 0; im < numCameras; ++im) {
    // relative pose adapter for Kneip toolchain
    opengv::relative_pose::FrameRelativeAdapter adapter(
        estimator, params.nCameraSystem, olderFrameId, im, currentFrameId, im);
    double overlap;
    double matchRatio;

    adapter.computeMatchStats(frameBPtr, im, &overlap, &matchRatio);
    maxOverlap = std::max(overlap, maxOverlap);
    maxMatchRatio = std::max(matchRatio, maxMatchRatio);
    size_t numCorrespondences = adapter.getNumberCorrespondences();

    if (numCorrespondences < 10) { // won't generate meaningful results.
      continue;
    }
    // try both the rotation-only RANSAC and the relative one:

    // create a RelativePoseSac problem and RANSAC
    typedef opengv::sac_problems::relative_pose::
        FrameRotationOnlySacProblem FrameRotationOnlySacProblem;
    opengv::sac::Ransac<FrameRotationOnlySacProblem> rotation_only_ransac;
    std::shared_ptr<FrameRotationOnlySacProblem>
        rotation_only_problem_ptr(
            new FrameRotationOnlySacProblem(adapter));
    rotation_only_ransac.sac_model_ = rotation_only_problem_ptr;
    // This is about 3 pixel in image.
    rotation_only_ransac.threshold_ = 9;
    rotation_only_ransac.max_iterations_ = 50;

    // run the ransac
    rotation_only_ransac.computeModel(0);

    // get quality
    int rotation_only_inliers = rotation_only_ransac.inliers_.size();
    float rotation_only_ratio = static_cast<float>(rotation_only_inliers) /
                                static_cast<float>(numCorrespondences);

    // now the translation only one:
    okvis::kinematics::Transformation T_SaCa, T_SbCb;
    estimator.getCameraSensorExtrinsics(im, T_SaCa);
    estimator.getCameraSensorExtrinsics(im, T_SbCb);
    okvis::kinematics::Transformation T_WSa, T_WSb;
    estimator.get_T_WS(olderFrameId, T_WSa);
    estimator.get_T_WS(currentFrameId, T_WSb);
    adapter.setR12((T_WSa.C() * T_SaCa.C()).transpose() * T_WSb.C() *
                   T_SbCb.C());

    typedef opengv::sac_problems::relative_pose::
        FrameTranslationOnlySacProblem FrameTranslationOnlySacProblem;
    opengv::sac::Ransac<FrameTranslationOnlySacProblem> translation_only_ransac;
    std::shared_ptr<FrameTranslationOnlySacProblem> translation_only_problem_ptr(
        new FrameTranslationOnlySacProblem(adapter));

    translation_only_ransac.sac_model_ = translation_only_problem_ptr;
    // This is about 3 pixels in image. More info at getSelectedDistancesToModel().
    translation_only_ransac.threshold_ = 9;
    translation_only_ransac.max_iterations_ = 50;

    // run the ransac
    translation_only_ransac.computeModel(0);

    // assess success
    int translation_only_inliers = translation_only_ransac.inliers_.size();
    float translation_only_ratio = static_cast<float>(translation_only_inliers) /
                           static_cast<float>(numCorrespondences);

    // decide on success and fill inliers
    std::vector<bool> inliers(numCorrespondences, false);
    if (rotation_only_ratio > translation_only_ratio || rotation_only_ratio > 0.8) {
      if (rotation_only_inliers > 10) {
        rotation_only_success = true;
      }
      rmt = okvis::ROTATION_ONLY;
      totalInlierNumber += rotation_only_inliers;
      for (size_t k = 0; k < rotation_only_ransac.inliers_.size(); ++k) {
        inliers.at(rotation_only_ransac.inliers_.at(k)) = true;
      }
    } else {
      if (translation_only_inliers > 10) {
        translation_only_success = true;
      }
      rmt = okvis::RELATIVE_POSE;
      totalInlierNumber += translation_only_inliers;
      for (size_t k = 0; k < translation_only_ransac.inliers_.size(); ++k) {
        inliers.at(translation_only_ransac.inliers_.at(k)) = true;
      }
    }

    // failure?
    if (!rotation_only_success && !translation_only_success) {
      continue;
    }

    // otherwise: kick out outliers!
    std::shared_ptr<okvis::MultiFrame> multiFrame = estimator.multiFrame(
        currentFrameId);

    for (size_t k = 0; k < numCorrespondences; ++k) {
      size_t idxB = adapter.getMatchKeypointIdxB(k);
      if (!inliers[k]) {
        uint64_t lmId = multiFrame->landmarkId(im, idxB);
        // reset ID:
        multiFrame->setLandmarkId(im, idxB, 0);
        // remove observation
        if (removeOutliers) {
          if (lmId != 0 && estimator.isLandmarkAdded(lmId)){
            estimator.removeObservation(lmId, currentFrameId, im, idxB);
          }
        }
      }
    }
  }

  *rotationOnly = rmt == okvis::ROTATION_ONLY;

  if (totalInlierNumber <= 15 || rmt == okvis::UNCERTAIN_MOTION) {
    *asKeyframe = true;
  }
  if (isInitialized_) {
    if (maxOverlap > frontendOptions_.keyframeInsertionOverlapThreshold &&
        maxMatchRatio > frontendOptions_.keyframeInsertionMatchingRatioThreshold) {
//      *asKeyframe = *asKeyframe;
    } else {
      *asKeyframe = true;
    }
  } else {
//    *asKeyframe = *asKeyframe;
  }

  if (translation_only_success || rotation_only_success) {
    return totalInlierNumber;
  } else {
    return -1;
  }
}
}  // namespace swift_vio
