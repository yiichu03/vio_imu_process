/**
 * @file filterJacobians.h
 *
 */

#include <okvis/CameraModelSwitch.hpp>
#include <okvis/ceres/Map.hpp>

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/ceres/RsReprojectionErrorAidp.hpp>
#include <swift_vio/ceres/RsReprojectionError.hpp>

#include <swift_vio/imu/SimpleImuPropagationJacobian.hpp>
#include <swift_vio/MapPoint.h>
#include <swift_vio/MultiFrame.hpp>

#include <swift_vio/PointLandmark.hpp>
#include <swift_vio/PointSharedData.hpp>
#include <swift_vio/States.h>

#include <glog/logging.h>

namespace swift_vio {
class FilterJacobians {
public:
  /**
   * @brief startIndexOfCameraParamsFast
   * @warning call this no earlier than first call of addStates().
   * @param camIdx
   * @return
   */
  static size_t startIndexOfCameraParamsFast(
      const StateMap &statesMap,
      size_t camIdx,
      CameraSensorStates camParamBlockName = CameraSensorStates::T_XCi) {
    return statesMap.rbegin()
        ->second.sensors.at(SensorStates::Camera)
        .at(camIdx)
        .at(camParamBlockName)
        .startIndexInCov;
  }

  static size_t startIndexOfClonedStatesFast(const StateMap &statesMap,
                                             const okvis::CameraNoiseParameters &camNoiseParams) {
    return statesMap.rbegin()
               ->second.sensors.at(SensorStates::Camera)
               .back()
               .at(CameraSensorStates::TR)
               .startIndexInCov +
           (camNoiseParams.isReadoutTimeFixed() ? 0u : 1u);
  }

  static size_t minimalDimOfAllCameraParams(const StateMap &statesMap,
                                            const okvis::CameraNoiseParameters &camNoiseParams) {
    size_t totalCamDim = statesMap.rbegin()
                             ->second.sensors.at(SensorStates::Camera)
                             .back()
                             .at(CameraSensorStates::TR)
                             .startIndexInCov +
                         (camNoiseParams.isReadoutTimeFixed() ? 0u : 1u);
    size_t startIndex = statesMap.rbegin()
                             ->second.sensors.at(SensorStates::Camera)
                             .at(0u)
                             .at(CameraSensorStates::T_XCi)
                             .startIndexInCov;
    return totalCamDim - startIndex;
  }

  /**
   * @brief measurementJacobian compute Jacobians for a reprojection residual error.
   * @warning Both poseId and anchorId should be older than the latest frame Id.
   * @param homogeneousPoint, if landmarkModel is AIDP,
   * \f$[\alpha, \beta, 1, \rho] = [X, Y, Z, 1]^C_a / Z^C_a\f$,
   * if landmarkModel is HPP, \f$[X,Y,Z,1]^W\f$.
   * \f $[\alpha, \beta, 1]^T = \rho p_{C{t(i, a)}} \f$ or
   * \f $[\alpha, \beta, 1]^T = \rho p_{C{t(a)}} \f$
   * \f $t(a) \f$ is the epoch of the anchor state/frame.
   * \f $t(i, a) \f$ is the observation epoch of feature i in anchor frame a.
   * @param obs image observation in pixels.
   * @param observationIndex index of the observation inside the point's shared data.
   * @param pointDataPtr shared data of the point.
   * @param J_x Jacobians of the image observation relative to the gravity direction,
   * camera parameters, and cloned states. It ought to be allocated in advance.
   * @param J_pfi Jacobian of the image observation relative to landmark parameters, e.g., [\alpha, \beta, \rho].
   * @param residual
   * @param mixProjectionJac do we multiply projection Jacobians with other Jacobians computed at different lin points?
   * @return true if Jacobians are computed successfully.
   */
  static bool measurementJacobian(
      const PointLandmark& homogeneousPoint,
      const Eigen::Vector2d& obs,
      size_t observationIndex,
      const swift_vio::PointSharedData &pointData,
      const CameraRig &cameraRig,
      const okvis::CameraNoiseParametersVec &camNoiseVec,
      const StateMap &statesMap,
      const okvis::EstimatorOptions &estimatorOptions,
      std::shared_ptr<const okvis::ImuParameters> imuParams,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* J_x,
      Eigen::Matrix<double, 2, 3>* J_pfi, Eigen::Vector2d* residual,
      bool mixProjectionJac);

  static bool measurementResidual(
      const PointLandmark &homogeneousPoint, const Eigen::Vector2d &obs,
      size_t observationIndex, const swift_vio::PointSharedData &pointData,
      const CameraRig &cameraRig,
      const okvis::EstimatorOptions &estimatorOptions,
      Eigen::Vector2d *residual);

  static bool measurementJacobianNumeric(
      const PointLandmark& homogeneousPoint,
      const Eigen::Vector2d& obs,
      size_t observationIndex,
      swift_vio::PointSharedData& pointData,
      CameraRig& cameraRig,
      const okvis::CameraNoiseParametersVec &camNoiseVec,
      const StateMap &statesMap,
      std::shared_ptr<okvis::ceres::Map> mapPtr,
      const okvis::EstimatorOptions &estimatorOptions,
      std::shared_ptr<okvis::ImuParameters> imuParams,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* J_x,
      Eigen::Matrix<double, 2, 3>* J_pfi, Eigen::Vector2d* residual);

  /**
   * @brief test measurementJacobian with auto diff on a ceres solver cost function.
   * @param pointLandmark
   * @param obs
   * @param observationIndex
   * @param pointDataPtr
   * @param J_x
   * @param J_pfi
   * @param residual
   * @return
   */
  static bool measurementJacobianAutoDiff(
      const PointLandmark &pointLandmark, const Eigen::Vector2d &obs,
      size_t observationIndex,
      const swift_vio::PointSharedData &pointData,
      const CameraRig &cameraRig,
      const okvis::CameraNoiseParametersVec &camNoiseVec,
      const StateMap &statesMap,
      const okvis::EstimatorOptions &estimatorOptions,
      std::shared_ptr<const okvis::ImuParameters> imuParams,
      Eigen::Matrix<double, 2, -1> *J_X, Eigen::Matrix<double, 2, 3> *J_pfi,
      Eigen::Vector2d *residual);

  /**
   * @brief slamFeatureJacobian, compute the residual and Jacobians for a SLAM
   * feature i observed in the current NFrame.
   * We assume that the landmark is well observed including its depth.
   * @param mp mappoint
   * @param H_x Jacobian w.r.t variables related to gravity direction, camera extrinsics,
   * intrinsics, cloned states.
   * @param r_i residual of the observation of the map point in the latest
   * frame.
   * @param R_i covariance matrix of this observation.
   * @param H_f Jacobian w.r.t variables of features, of columns 3k
   * where k is the number of features in the state vector.
   * @return true if succeeded in computing the residual and Jacobians.
   */
  static bool slamFeatureJacobian(const MapPoint &mp, const StateMap &statesMap,
                           std::shared_ptr<const okvis::ceres::Map> mapPtr,
                           const BareMultiFrameMap &frameTimeMap,
                           const CameraRig &cameraRig,
                           const okvis::CameraNoiseParametersVec &camNoiseVec,
                           std::shared_ptr<const okvis::ImuParameters> imuParams,
                           const okvis::EstimatorOptions &estimatorOptions,
                           uint64_t currentFrameId,
                           const PointLandmark &homoPointRep,
                           Eigen::MatrixXd &H_x,
                           Eigen::Matrix<double, -1, 1> &r_i,
                           Eigen::MatrixXd &R_i, Eigen::MatrixXd &subH_f);

  /**
   * @brief compute the marginalized Jacobian for a feature i's track.
   * A landmark is first triangulated with the feature track, then Jacobians for
   * the list of observations are computed. The landmark Jacobian is optionally marginalized.
   * @warning The number of observations of the map points is at least two.
   * @param mp mappoint
   * @param H_oi Jacobians of feature observations w.r.t variables related to
   * gravity direction, camera extrinsics, intrinsics, and cloned nav states.
   * @param r_oi residuals
   * @param R_oi covariance matrix of these observations.
   * @param pH_fi pointer to the Jacobian of feature observations w.r.t the
   * feature parameterization, e.g., [\alpha, \beta, \rho].
   * if pH_fi is NULL, r_oi H_oi and R_oi are values after marginalizing H_fi,
   * H_oi is of size e.g., (2n-3)x(13+9(m-1)-3);
   * otherwise, H_oi is of size 2nx(13+9(m-1)-3).
   * @param involved_frame_ids frames for which to compute Jacobians.
   * @return Jacobian status for the feature track.
   */
  static FeatureJacobianStatus featureJacobian(
      const MapPoint &mp,
      const StateMap &statesMap,
      std::shared_ptr<const okvis::ceres::Map> mapPtr,
      const BareMultiFrameMap &frameTimeMap,
      const CameraRig &cameraRig,
      const okvis::CameraNoiseParametersVec &camNoiseVec,
      std::shared_ptr<const okvis::ImuParameters> imuParams,
      const okvis::EstimatorOptions &estimatorOptions,
      swift_vio::PointLandmark *pointLandmark,
      swift_vio::PointSharedData *pointDataPtr,
      Eigen::MatrixXd &H_oi, Eigen::Matrix<double, Eigen::Dynamic, 1> &r_oi,
      Eigen::MatrixXd &R_oi, Eigen::Matrix<double, Eigen::Dynamic, 3> *pH_fi,
      std::vector<uint64_t> *framesToUse, size_t minTrackLength,
      size_t minCulledFrames);

  static void changeAnchors(
      const std::vector<uint64_t> &sortedRemovedStateIds,
      const StateMap &statesMap, std::shared_ptr<const okvis::ceres::Map> mapPtr,
      const CameraRig &cameraRig,
      const okvis::CameraNoiseParametersVec &camNoiseVec,
      std::deque<PointLandmark> *orderedLandmarks,
      PointMap *landmarksMap,
      Eigen::MatrixXd *covariance);
/**
 * @brief compute the Jacobians of T_BCi relative to extrinsic parameters.
 * Perturbation in T_BC is defined by kinematics::oplus.
 * Perturbation in extrinsic parameters are defined by extrinsic models.
 * @param T_BCi Transform from i camera frame to body frame.
 * @param T_BC0 Transform from main camera frame to body frame.
 * @param camiExtrinsicRepId
 * @param cam0ExtrinsicRepId
 * @param camiIndex
 * @param cam0Index
 * @param dT_BCi_dExtrinsics list of Jacobians for T_BCi.
 * @param camIndices Indices of cameras in the camera rig. For these cameras,
 * the Jacobians relative to their extrinsics are computed.
 */
static void computeExtrinsicJacobians(
    const okvis::kinematics::Transformation& T_BCi,
    const okvis::kinematics::Transformation& T_BC0,
    int camiExtrinsicRepId, int cam0ExtrinsicRepId,
    size_t camiIndex, size_t cam0Index,
    Eigen::AlignedVector<Eigen::MatrixXd>* dT_BCi_dExtrinsics,
    std::vector<size_t>* camIndices);
};
}  // namespace swift_vio
