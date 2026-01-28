#ifndef INCLUDE_SWIFT_VIO_POINT_LANDMARK_SIMULATION_RS_HPP
#define INCLUDE_SWIFT_VIO_POINT_LANDMARK_SIMULATION_RS_HPP

#ifdef HAVE_BSPLINES
#include <bsplines/SO3BSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>
#endif

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/kinematics/Transformation.hpp>

#include <random>

#include <simul/curves.h>
#include <swift_vio/MultipleTransformPointJacobian.hpp>
#include <swift_vio/TransformMultiplyJacobian.hpp>

namespace simul {
class PointLandmarkSimulationRS
{
 public:
  /**
   * @brief project a landmark in the rolling shutter camera.
   * @param simulatedTrajectory
   * @param hpW
   * @param T_SCi
   * @param cameraGeometry
   * @param centralRowTime
   * @param projection
   * @return status
   */
  static okvis::cameras::CameraBase::ProjectionStatus projectLandmark(
      std::shared_ptr<const simul::TrajectoryBase> simulatedTrajectory,
      const Eigen::Vector4d& hpW,
      const okvis::kinematics::Transformation& T_SCi,
      std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry,
      okvis::Time centralRowTime, Eigen::Vector2d* projection, Eigen::Vector4d *point_C);

  /**
   * @brief use a grid to remove redundant keypoints.
   * In each grid cell, keep the keypoint of the shortest distance.
   * @param imageWidth
   * @param imageHeight
   * @param keypoints
   * @param landmarkIndices
   * @param sparseKeypoints
   * @param sparseLandmarkIndices
   * @return number of sparse keypoints
   */
  static size_t
  sparsifyKeypointsByDistance(size_t imageWidth, size_t imageHeight,
                              const std::vector<cv::KeyPoint> &keypoints,
                              const std::vector<size_t> &landmarkIndices,
                              std::vector<cv::KeyPoint> *sparseKeypoints,
                              std::vector<size_t> *sparseLandmarkIndices);
  /**
   * @brief projectLandmarksToNFrame
   * @param[in] homogeneousPoints
   * @param[in] simulatedTrajectory
   * @param[in] trueCentralRowEpoch: We assume all cameras capture the central row at the same time,
   * but they may have different time offset relative to say the IMU.
   * @param[in] cameraSystemRef
   * @param[out] nframes will be assigned keypoints for individual frames.
   * Only keypoints for successfully projected landmarks are kept.
   * @param[out] frameLandmarkIndices landmark indices of keypoints in frames.
   * @param[out] keypointIndices mapping from landmark index in the landmark grid to its keypoint index in a frame of nframes.
   * @param[in] imageNoiseMag
   */

  template <typename MultiFrameT>
  static void projectLandmarksToNFrame(
      const std::vector<Eigen::Vector4d,
                        Eigen::aligned_allocator<Eigen::Vector4d>>&
          homogeneousPoints,
      std::shared_ptr<const simul::TrajectoryBase> simulatedTrajectory,
      okvis::Time trueCentralRowEpoch,
      const swift_vio::CameraRig &cameraSystemRef,
      std::shared_ptr<MultiFrameT> nframes,
      std::vector<std::vector<size_t>>* frameLandmarkIndices,
      std::vector<std::unordered_map<size_t, size_t>>* keypointIndices,
      const double* imageNoiseMag) {
    size_t numFrames = nframes->numFrames();

    okvis::kinematics::Transformation T_WS_ref =
        simulatedTrajectory->computeGlobalPose(trueCentralRowEpoch);

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, 1.0};
    // project landmarks onto frames of nframes
    for (size_t i = 0; i < numFrames; ++i) {
      std::vector<size_t> landmarkIndices;
      landmarkIndices.reserve(homogeneousPoints.size() / 2);

      std::vector<cv::KeyPoint> keypoints;
      keypoints.reserve(homogeneousPoints.size() / 2);

      size_t w = cameraSystemRef.cameraGeometry(i)->imageWidth();
      size_t h = cameraSystemRef.cameraGeometry(i)->imageHeight();
      for (size_t j = 0; j < homogeneousPoints.size(); ++j) {
        Eigen::Vector4d point_C;
        Eigen::Vector2d projection;
        double tr = cameraSystemRef.cameraGeometry(i)->readoutTime();
        okvis::cameras::CameraBase::ProjectionStatus status;
        if (tr > 1e-6) {
          status = projectLandmark(
                simulatedTrajectory, homogeneousPoints[j], cameraSystemRef.getCameraExtrinsic(i),
                cameraSystemRef.cameraGeometry(i), trueCentralRowEpoch, &projection, &point_C);
        } else {
          point_C = cameraSystemRef.getCameraExtrinsic(i).inverse() *
                                    T_WS_ref.inverse() * homogeneousPoints[j];

          status = cameraSystemRef.cameraGeometry(i)->projectHomogeneous(
              point_C, &projection);
        }
        float pZ = (float)(point_C[2] / point_C[3]);
        if (pZ <= 0.1f) {  // remove points too close.
          continue;
        }
        if (status == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
          Eigen::Vector2d measurement(projection);
          if (imageNoiseMag) {
            measurement[0] += d(gen) * *imageNoiseMag;
            measurement[1] += d(gen) * *imageNoiseMag;
          }
          // some points may be out of bounds after adding noise
          if (measurement[0] < 0 || measurement[1] < 0 ||
              measurement[0] > w - 1 || measurement[1] > h - 1)
            continue;
          keypoints.emplace_back(measurement[0], measurement[1], 8.0, -1, pZ);
          landmarkIndices.emplace_back(j);
        }
      }

      std::vector<size_t> sparseLandmarkIndices;
      std::vector<cv::KeyPoint> sparseKeypoints;
      sparsifyKeypointsByDistance(w, h, keypoints,
          landmarkIndices, &sparseKeypoints, &sparseLandmarkIndices);

      std::unordered_map<size_t, size_t> frameKeypointIndices;
      for (size_t ii = 0; ii < sparseLandmarkIndices.size(); ++ii) {
        frameKeypointIndices[sparseLandmarkIndices[ii]] = ii;
      }

      nframes->resetKeypoints(i, sparseKeypoints);

      if (frameLandmarkIndices)
        frameLandmarkIndices->emplace_back(sparseLandmarkIndices);
      if (keypointIndices)
        keypointIndices->emplace_back(frameKeypointIndices);
    }
  }

#ifdef HAVE_BSPLINES
  template <int N>
  static okvis::cameras::CameraBase::ProjectionStatus projectLandmark(
      std::shared_ptr<const bsplines::EuclideanBSpline<N, 3, bsplines::NsecTimePolicy>> transTraj,
      std::shared_ptr<const bsplines::SO3BSpline<N, bsplines::NsecTimePolicy>> rotTraj,
      const Eigen::Vector4d& hpW,
      const okvis::kinematics::Transformation& T_SCi,
      std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry,
      okvis::Time centralRowTime, Eigen::Vector2d* projection, Eigen::Vector4d *point_C) {
    double dt = 1e6; // f(t) / f'(t)
    double relativeFeatureTime = 0; // feature epoch relative to central row, i.e., frame timestamp.
    const double tol = 1e-5;
    int numIter = 0;
    const int maxIter = 5;

    double tr = cameraGeometry->readoutTime();
    double height = cameraGeometry->imageHeight();
    okvis::cameras::CameraBase::ProjectionStatus status;
    while (numIter < maxIter && std::fabs(dt) > tol) {
        okvis::Time featureTime = centralRowTime + okvis::Duration(relativeFeatureTime);
        auto rotEval = rotTraj->template getEvaluatorAt<1>(featureTime.toNSec());
        Eigen::Vector4d q_WBt_raw = rotEval.eval();
        auto transEval = transTraj->template getEvaluatorAt<1>(featureTime.toNSec());
        Eigen::Vector3d t_WBt = transEval.eval();
        Eigen::Quaterniond q_WBt(q_WBt_raw);
        okvis::kinematics::Transformation T_WBt(t_WBt, q_WBt);
        *point_C = T_SCi.inverse() * T_WBt.inverse() * hpW;
        Eigen::Matrix<double, 2, 4> pointJacobian;
        status = cameraGeometry->projectHomogeneous(
            *point_C, projection, &pointJacobian);
        if (status != okvis::cameras::CameraBase::ProjectionStatus::Successful) {
            break;
        }
        double f_of_t = (*projection)[1] - (relativeFeatureTime / tr + 0.5) * height;

        // compute Jacobians required by Newton Raphson method.
        Eigen::AlignedVector<okvis::kinematics::Transformation> transformList{T_SCi, T_WBt};
        std::vector<int> exponentList{-1, -1};
        swift_vio::MultipleTransformPointJacobian mtpj(transformList, exponentList, hpW);
        mtpj.computeJacobians();
        Eigen::Matrix<double, 4, 6> dpC_dT_WB = mtpj.dp_dT(0);
        okvis::kinematics::Transformation T_identity;
        Eigen::Vector3d v_WB = transEval.evalD(1);
        Eigen::Vector3d omega_B = rotEval.evalAngularVelocityLocal();
        swift_vio::TransformMultiplyJacobian tmj(T_WBt, T_identity, v_WB, omega_B);
        Eigen::Matrix<double, 6, 1> dT_WB_dt;
        dT_WB_dt.head<3>() = tmj.dp_dt();
        dT_WB_dt.tail<3>() = tmj.dtheta_dt();

        double fprime_of_t = pointJacobian.row(1) * dpC_dT_WB * dT_WB_dt - height / tr;
        dt = f_of_t / fprime_of_t;
  //            LOG(INFO) << "Rs projection iter " << numIter << " dt " << dt;
        relativeFeatureTime -= dt;
        ++numIter;
    }
    return status;
  }

  template <typename MultiFrameT, int N>
  static void projectLandmarksToNFrame(
      const std::vector<Eigen::Vector4d,
                        Eigen::aligned_allocator<Eigen::Vector4d>>
          &homogeneousPoints,
      std::shared_ptr<typename bsplines::EuclideanBSpline<N, 3, bsplines::NsecTimePolicy>> transTraj,
      std::shared_ptr<typename bsplines::SO3BSpline<N, bsplines::NsecTimePolicy>> rotTraj,
      okvis::Time trueCentralRowEpoch,
      const swift_vio::CameraRig &cameraSystemRef,
      std::shared_ptr<MultiFrameT> nframes,
      std::vector<std::vector<size_t>> *frameLandmarkIndices,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndices,
      const double *imageNoiseMag) {
    size_t numFrames = nframes->numFrames();

    typename bsplines::SO3BSpline<N, bsplines::NsecTimePolicy>::template Evaluator<0> rotEval =
        rotTraj->template getEvaluatorAt<0>(trueCentralRowEpoch.toNSec());
    Eigen::Vector4d q_WS_raw = rotEval.eval();
    typename bsplines::EuclideanBSpline<N, 3, bsplines::NsecTimePolicy>::template Evaluator<0> transEval =
        transTraj->template getEvaluatorAt<0>(trueCentralRowEpoch.toNSec());
    Eigen::Vector3d t_WS_ref = transEval.eval();
    Eigen::Quaterniond q_WS_ref(q_WS_raw);
    okvis::kinematics::Transformation T_WS_ref(t_WS_ref, q_WS_ref);

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, 1.0};
    // project landmarks onto frames of nframes
    for (size_t i = 0; i < numFrames; ++i) {
      std::vector<size_t> landmarkIndices;
      landmarkIndices.reserve(homogeneousPoints.size() / 2);

      std::vector<cv::KeyPoint> keypoints;
      keypoints.reserve(homogeneousPoints.size() / 2);
      size_t w = cameraSystemRef.cameraGeometry(i)->imageWidth();
      size_t h = cameraSystemRef.cameraGeometry(i)->imageHeight();
      for (size_t j = 0; j < homogeneousPoints.size(); ++j) {
        Eigen::Vector4d point_C;
        Eigen::Vector2d projection;
        double tr = cameraSystemRef.cameraGeometry(i)->readoutTime();
        okvis::cameras::CameraBase::ProjectionStatus status;
        if (tr > 1e-6) {
          status = projectLandmark<N>(transTraj, rotTraj, homogeneousPoints[j],
                                   cameraSystemRef.getCameraExtrinsic(i),
                                   cameraSystemRef.cameraGeometry(i),
                                   trueCentralRowEpoch, &projection, &point_C);
        } else {
          point_C = cameraSystemRef.getCameraExtrinsic(i).inverse() *
                    T_WS_ref.inverse() * homogeneousPoints[j];
          status = cameraSystemRef.cameraGeometry(i)->projectHomogeneous(
              point_C, &projection);
        }
        float pZ = (float)(point_C[2] / point_C[3]);
        if (pZ <= 0.1f) {  // remove points too close.
          continue;
        }
        if (status ==
            okvis::cameras::CameraBase::ProjectionStatus::Successful) {
          Eigen::Vector2d measurement(projection);
          if (imageNoiseMag) {
            measurement[0] += d(gen) * *imageNoiseMag;
            measurement[1] += d(gen) * *imageNoiseMag;
          }
          // some points may be out of bounds after adding noise
          if (measurement[0] < 0 || measurement[1] < 0 ||
              measurement[0] > w - 1 || measurement[1] > h - 1)
            continue;
          keypoints.emplace_back(measurement[0], measurement[1], 8.0, -1, pZ);
          landmarkIndices.emplace_back(j);
        }
      }

      std::vector<size_t> sparseLandmarkIndices;
      std::vector<cv::KeyPoint> sparseKeypoints;
      sparsifyKeypointsByDistance(w, h, keypoints,
          landmarkIndices, &sparseKeypoints, &sparseLandmarkIndices);

      std::unordered_map<size_t, size_t> frameKeypointIndices;
      for (size_t ii = 0; ii < sparseLandmarkIndices.size(); ++ii) {
        frameKeypointIndices[sparseLandmarkIndices[ii]] = ii;
      }

      nframes->resetKeypoints(i, sparseKeypoints);

      if (frameLandmarkIndices)
        frameLandmarkIndices->emplace_back(sparseLandmarkIndices);
      if (keypointIndices)
        keypointIndices->emplace_back(frameKeypointIndices);
    }
  }
#endif
};
}  // namespace simul
#endif // INCLUDE_SWIFT_VIO_POINT_LANDMARK_SIMULATION_RS_HPP
