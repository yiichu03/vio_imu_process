#include "swift_vio/filterJacobians.h"

#include <okvis/CameraModelSwitch.hpp>
#include <okvis/IdProvider.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>

#include <swift_vio/ceres/NormalVectorParameterBlock.hpp>
#include <swift_vio/ceres/ChordalDistance.hpp>

#include <swift_vio/ceres/RsReprojectionErrorPap.hpp>
#include <swift_vio/ceres/RsReprojectionErrorAidp.hpp>
#include <swift_vio/ceres/RsReprojectionError.hpp>

#include <swift_vio/EstimationHelper.h>
#include <swift_vio/FilterHelper.hpp>
#include <swift_vio/imu/SimpleImuPropagationJacobian.hpp>
#include <swift_vio/MultipleTransformPointJacobian.hpp>

#include <swift_vio/PointLandmark.hpp>
#include <swift_vio/PointSharedData.hpp>

#include <glog/logging.h>

DEFINE_bool(mix_projection_jac, true, 
            "Do we multiply projection Jacobians with other Jacobians computed at different lin points?");

namespace swift_vio {
static const int kClonedStateMinimalDim = 9;

bool FilterJacobians::measurementJacobian(
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
    bool mixProjectionJac) {
  // compute Jacobians for a measurement in current image j of feature i \f$f_i\f$.
  // C_{t(i,j)} is the camera frame at the observation epoch t(i,j).
  // B_{t(i,j)} is the body frame at the observation epoch t(i,j).
  // B_j is the body frame at the state epoch t_j associated with the NFrame of image j.
  // B_a is the body frame at the state epoch t_a associated with the anchor NFrame.

  Eigen::Matrix2Xd
      intrinsicsJacobian;  // \f$\frac{\partial [z_u, z_v]^T}{\partial(intrinsics)}\f$
  Eigen::Matrix<double, 2, 3>
      dz_drhoxpCtj;  // \f$\frac{\partial [z_u, z_v]^T}{\partial
                       // \rho p_i^{C_{t(i,j)}}\f$

  size_t camIdx = pointData.cameraIndex(observationIndex);
  const okvis::kinematics::Transformation T_BCj = cameraRig.getCameraExtrinsic(camIdx);
  okvis::kinematics::Transformation T_WBtij = pointData.T_WBtij(observationIndex);
  okvis::kinematics::Transformation T_BC0 =
      cameraRig.getCameraExtrinsic(kMainCameraIndex);

  // Entities for computing Jacobians.
  // transformation chain applied to the homogeneous point from left to right.
  // These poses may have linearization points fixed earlier.
  Eigen::AlignedVector<okvis::kinematics::Transformation> transformList;
  std::vector<int> exponentList;
  transformList.reserve(4);
  exponentList.reserve(4);
  transformList.push_back(T_BCj);
  exponentList.push_back(-1);

  okvis::kinematics::Transformation T_WBtij_lin =
      pointData.T_WBtij_ForJacobian(observationIndex);
  transformList.push_back(T_WBtij_lin);
  exponentList.push_back(-1);

  std::vector<size_t> camIndices{camIdx};  // indices of cameras in the transformation chain.
  std::vector<size_t> mtpjExtrinsicIndices{0u};  // indices of extrinsics in the transformation chain.
  std::vector<size_t> mtpjPoseIndices{1u};  // indices of poses in the transformation chain.
  // extrinsics in the transformation chain from left to right.
  Eigen::AlignedVector<okvis::kinematics::Transformation> T_BC_list{T_BCj};
  int extrinsicRepId = cameraRig.getExtrinsicRepId(camIdx);
  std::vector<int> extrinsicRepIdList{extrinsicRepId};
  std::vector<size_t> observationIndices{observationIndex};

  uint64_t poseId = pointData.frameId(observationIndex);
  // frames involved in the transformation chain from left to right.
  std::vector<uint64_t> frameIndices{poseId};
  // state poses involved in the transformation chain from left to right as used in the transformList.
  Eigen::AlignedVector<okvis::kinematics::Transformation> T_WBt_list{T_WBtij_lin};

  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dhp_dparams;
  homogeneousPoint.ComputeJacobian(dhp_dparams.data());

  okvis::kinematics::Transformation T_CtjX; // X is W for HPP or \f$C_{t(a)}\f$ for AIDP.

  if (homogeneousPoint.modelId() ==
      swift_vio::InverseDepthParameterization::kModelId) {
    size_t anchorCamIdx = pointData.anchorIds()[0].cameraIndex_;
    const okvis::kinematics::Transformation T_BCa =
        cameraRig.getCameraExtrinsic(anchorCamIdx);

    okvis::kinematics::Transformation T_WBta;
    size_t anchorObservationIndex = pointData.anchorIds()[0].observationIndex_;
    okvis::kinematics::Transformation T_WBta_lin;

    T_WBta = pointData.T_WB_mainAnchorStateEpoch();
    T_WBta_lin = pointData.T_WB_mainAnchorStateEpochForJacobian();

    okvis::kinematics::Transformation T_WCta = T_WBta * T_BCa;
    T_CtjX = (T_WBtij * T_BCj).inverse() * T_WCta;

    transformList.push_back(T_WBta_lin);
    exponentList.push_back(1);

    transformList.push_back(T_BCa);
    exponentList.push_back(1);

    camIndices.push_back(anchorCamIdx);
    mtpjExtrinsicIndices.push_back(3u);
    mtpjPoseIndices.push_back(2u);
    T_BC_list.push_back(T_BCa);

    int anchorExtrinsicRepId = cameraRig.getExtrinsicRepId(anchorCamIdx);
    extrinsicRepIdList.push_back(anchorExtrinsicRepId);
    observationIndices.push_back(anchorObservationIndex);
    frameIndices.push_back(pointData.anchorIds()[0].frameId_);
    T_WBt_list.push_back(T_WBta_lin);
  } else {
    T_CtjX = (T_WBtij * T_BCj).inverse();
  }

  std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry =
      cameraRig.cameraGeometry(camIdx);
  // \f$\rho \times p^{C(t_{i,j})}\f$
  Eigen::Vector3d rhoxpCtj = (T_CtjX * homogeneousPoint.estimate()).head<3>();

  Eigen::Vector4d homogeneousPointLin =
      homogeneousPoint.getLinearizationPoint();
  swift_vio::MultipleTransformPointJacobian mtpj(transformList, exponentList,
                                                 homogeneousPointLin);
  Eigen::Vector3d rhoxpCtjLin = mtpj.evaluate().head<3>();
  okvis::cameras::CameraBase::ProjectionStatus residualStatus;
  okvis::cameras::CameraBase::ProjectionStatus projectionJacStatus;

  Eigen::Vector2d imagePoint;
  if (mixProjectionJac) {
    residualStatus = cameraGeometry->project(
        rhoxpCtj, &imagePoint, &dz_drhoxpCtj, &intrinsicsJacobian);
    *residual = obs - imagePoint;
    projectionJacStatus = residualStatus;
  } else {
    residualStatus = cameraGeometry->project(rhoxpCtj, &imagePoint);
    *residual = obs - imagePoint;
    projectionJacStatus = cameraGeometry->project(
        rhoxpCtjLin, &imagePoint, &dz_drhoxpCtj, &intrinsicsJacobian);
  }
  if (residualStatus != okvis::cameras::CameraBase::ProjectionStatus::Successful ||
      projectionJacStatus != okvis::cameras::CameraBase::ProjectionStatus::Successful) {
    return false;
  }

  if (!estimatorOptions.useMahalanobisGating) {
    if (std::fabs((*residual)[0]) > estimatorOptions.maxProjectionErrorTol ||
        std::fabs((*residual)[1]) > estimatorOptions.maxProjectionErrorTol) {
      return false;
    }
  }
  // start index of subsets of J_x relative to the covariance matrix and the
  // minimal dim of a subset's corresponding variable.
  std::vector<std::pair<size_t, size_t>> startIndexToMinDim;
  Eigen::AlignedVector<Eigen::MatrixXd> dpoint_dX; // drhoxpCtj_dParameters
  // compute dpoint_dX
  size_t gravityDimInCov = imuParams->isGravityDirectionVariable() ? 2u : 0u;
  size_t cameraParamStartIndex =
      startIndexOfCameraParamsFast(statesMap, kMainCameraIndex);
  size_t intactDim = cameraParamStartIndex - gravityDimInCov;
  mtpj.computeJacobians();
  for (size_t ja = 0; ja < camIndices.size(); ++ja) { // observing camera and/or anchor camera.
    // Extrinsic Jacobians.
    int mainExtrinsicRepId =
        cameraRig.getExtrinsicRepId(kMainCameraIndex);
    if (!camNoiseVec.at(camIndices[ja]).isExtrinsicsFixed()) {
      Eigen::Matrix<double, 4, 6> dpoint_dT_BC = mtpj.dp_dT(mtpjExtrinsicIndices[ja]);
      std::vector<size_t> involvedCameraIndices;
      Eigen::AlignedVector<Eigen::MatrixXd> dT_BC_dExtrinsics;
      computeExtrinsicJacobians(T_BC_list[ja], T_BC0, extrinsicRepIdList[ja],
                                mainExtrinsicRepId, camIndices[ja], kMainCameraIndex,
                                &dT_BC_dExtrinsics, &involvedCameraIndices);
      size_t camParamIdx = 0u;
      for (auto idx : involvedCameraIndices) {
        size_t extrinsicStartIndex = startIndexOfCameraParamsFast(statesMap, idx);
        size_t extrinsicDim = cameraRig.getMinimalExtrinsicDim(idx);
        startIndexToMinDim.emplace_back(extrinsicStartIndex, extrinsicDim);
        dpoint_dX.emplace_back(dpoint_dT_BC * dT_BC_dExtrinsics[camParamIdx]);
        ++camParamIdx;
      }
    }

    // Jacobians relative to nav states
    Eigen::Matrix<double, 4, 6> dpoint_dT_WBtij = mtpj.dp_dT(mtpjPoseIndices[ja]);
    auto stateIter = statesMap.find(frameIndices[ja]);
    int navStateIndex = stateIter->second.global.at(GlobalStates::T_WS).startIndexInCov;
    startIndexToMinDim.emplace_back(navStateIndex, 6u);

    // Jacobians relative to time parameters, velocity, and gravity.
    if (ja == 1u) {
      // Because the anchor frame is at the known state epoch, the Jacobians of its
      // pose relative to time and velocity are zero.
      dpoint_dX.emplace_back(dpoint_dT_WBtij);
    } else {  // ja == 0
      Eigen::Matrix<double, 4, 6> dpoint_dT_WBtj = dpoint_dT_WBtij;
      Eigen::Matrix3d Phi_pq_tij_tj = pointData.Phi_pq_feature(observationIndices[ja]);
      dpoint_dT_WBtj.rightCols(3) += dpoint_dT_WBtij.leftCols(3) * Phi_pq_tij_tj;
      dpoint_dX.emplace_back(dpoint_dT_WBtj);
      Eigen::Vector3d v_WBt =
          pointData.v_WBtij_ForJacobian(observationIndices[ja]);
      Eigen::Matrix<double, 6, 1> dT_WBtij_dt;
      dT_WBtij_dt.head<3>() =
          swift_vio::SimpleImuPropagationJacobian::dp_dt(v_WBt);
      Eigen::Vector3d omega_Btij =
          pointData.omega_Btij(observationIndices[ja]);
      dT_WBtij_dt.tail<3>() = swift_vio::SimpleImuPropagationJacobian::dtheta_dt(
          omega_Btij, T_WBt_list[ja].q());

      // Jacobian relative to time offset and readout time
      Eigen::Vector2d dt_dtdtr(1, 1);
      dt_dtdtr[1] = pointData.normalizedRow(observationIndices[ja]);
      if (!camNoiseVec.at(camIndices[ja]).isTimeDelayFixed()) {
        size_t cameraDelayIndex = startIndexOfCameraParamsFast(
            statesMap, camIndices[ja], CameraSensorStates::TD);
        startIndexToMinDim.emplace_back(cameraDelayIndex, 1u);
        dpoint_dX.emplace_back(dpoint_dT_WBtij * dT_WBtij_dt * dt_dtdtr[0]);
      }
      if (!camNoiseVec.at(camIndices[ja]).isReadoutTimeFixed()) {
        size_t readoutTimeIndex = startIndexOfCameraParamsFast(
            statesMap, camIndices[ja], CameraSensorStates::TR);
        startIndexToMinDim.emplace_back(readoutTimeIndex, 1u);
        dpoint_dX.emplace_back(dpoint_dT_WBtij * dT_WBtij_dt * dt_dtdtr[1]);
      }

      // Jacobian relative to velocity.
      double featureDelay =
          pointData.normalizedFeatureTime(observationIndices[ja]);
      startIndexToMinDim.emplace_back(navStateIndex + 6u, 3u);
      dpoint_dX.emplace_back(dpoint_dT_WBtij.leftCols(3) * featureDelay);

      if (gravityDimInCov > 0u) {
        Eigen::Matrix<double, 6, 2> dT_WBt_dunitgW;
        dT_WBt_dunitgW.topRows<3>() =
            swift_vio::SimpleImuPropagationJacobian::dp_dunitgW(
                featureDelay, imuParams->gravityDirection(),
                imuParams->g);
        startIndexToMinDim.emplace_back(intactDim, gravityDimInCov);
        dpoint_dX.emplace_back(dpoint_dT_WBtij.leftCols(3) *
                               dT_WBt_dunitgW.topRows<3>());
      }
    }
  }
  // Accumulate Jacobians relative to state variables including nav states, camera extrinsics, td and tr.
  J_x->setZero();
  size_t iterIndex = 0u;
  for (auto& startAndLen : startIndexToMinDim) {
    J_x->block(0, startAndLen.first - intactDim, 2, startAndLen.second) +=
        dz_drhoxpCtj * dpoint_dX[iterIndex].topRows<3>();
    ++iterIndex;
  }
  // Jacobian relative to camera intrinsic parameters.
  if (!camNoiseVec.at(camIdx).isIntrinsicsFixed()) {
    int projIntrinsicRepId = cameraRig.getProjectionIntrinsicRepId(camIdx);
    ProjIntrinsicRepMinimalIntrinsicJacobian(projIntrinsicRepId, &intrinsicsJacobian);
    size_t startIndex =
        startIndexOfCameraParamsFast(statesMap,
            camIdx, CameraSensorStates::Intrinsics) -
        intactDim;
    J_x->block(0, startIndex, 2, intrinsicsJacobian.cols()) =
        intrinsicsJacobian;
  }
  // Jacobian relative to landmark parameters.
  (*J_pfi) = dz_drhoxpCtj * mtpj.dp_dpoint().topRows<3>() * dhp_dparams;
  return true;
}

bool FilterJacobians::measurementResidual(
    const PointLandmark &homogeneousPoint, const Eigen::Vector2d &obs,
    size_t observationIndex, const swift_vio::PointSharedData &pointData,
    const CameraRig &cameraRig,
    const okvis::EstimatorOptions &estimatorOptions,
    Eigen::Vector2d *residual) {
  size_t camIdx = pointData.cameraIndex(observationIndex);
  const okvis::kinematics::Transformation T_BCj = cameraRig.getCameraExtrinsic(camIdx);
  okvis::kinematics::Transformation T_WBtij = pointData.T_WBtij(observationIndex);
  okvis::kinematics::Transformation T_BC0 =
      cameraRig.getCameraExtrinsic(kMainCameraIndex);

  // Entities for computing Jacobians.
  // transformation chain applied to the homogeneous point from left to right.
  // These poses may have linearization points fixed earlier.
  Eigen::AlignedVector<okvis::kinematics::Transformation> transformList;
  std::vector<int> exponentList;
  transformList.reserve(4);
  exponentList.reserve(4);
  transformList.push_back(T_BCj);
  exponentList.push_back(-1);

  okvis::kinematics::Transformation T_WBtij_lin =
      pointData.T_WBtij_ForJacobian(observationIndex);
  transformList.push_back(T_WBtij_lin);
  exponentList.push_back(-1);

  std::vector<size_t> camIndices{camIdx};  // indices of cameras in the transformation chain.
  std::vector<size_t> mtpjExtrinsicIndices{0u};  // indices of extrinsics in the transformation chain.
  std::vector<size_t> mtpjPoseIndices{1u};  // indices of poses in the transformation chain.
  // extrinsics in the transformation chain from left to right.
  Eigen::AlignedVector<okvis::kinematics::Transformation> T_BC_list{T_BCj};
  int extrinsicRepId = cameraRig.getExtrinsicRepId(camIdx);
  std::vector<int> extrinsicRepIdList{extrinsicRepId};
  std::vector<size_t> observationIndices{observationIndex};

  uint64_t poseId = pointData.frameId(observationIndex);
  // frames involved in the transformation chain from left to right.
  std::vector<uint64_t> frameIndices{poseId};
  // state poses involved in the transformation chain from left to right as used in the transformList.
  Eigen::AlignedVector<okvis::kinematics::Transformation> T_WBt_list{T_WBtij_lin};

  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dhp_dparams;
  homogeneousPoint.ComputeJacobian(dhp_dparams.data());

  okvis::kinematics::Transformation T_CtjX; // X is W for HPP or \f$C_{t(a)}\f$ for AIDP.

  if (homogeneousPoint.modelId() ==
      swift_vio::InverseDepthParameterization::kModelId) {
    size_t anchorCamIdx = pointData.anchorIds()[0].cameraIndex_;
    const okvis::kinematics::Transformation T_BCa =
        cameraRig.getCameraExtrinsic(anchorCamIdx);

    okvis::kinematics::Transformation T_WBta;
    size_t anchorObservationIndex = pointData.anchorIds()[0].observationIndex_;
    okvis::kinematics::Transformation T_WBta_lin;

    T_WBta = pointData.T_WB_mainAnchorStateEpoch();
    T_WBta_lin = pointData.T_WB_mainAnchorStateEpochForJacobian();

    okvis::kinematics::Transformation T_WCta = T_WBta * T_BCa;
    T_CtjX = (T_WBtij * T_BCj).inverse() * T_WCta;

    transformList.push_back(T_WBta_lin);
    exponentList.push_back(1);

    transformList.push_back(T_BCa);
    exponentList.push_back(1);

    camIndices.push_back(anchorCamIdx);
    mtpjExtrinsicIndices.push_back(3u);
    mtpjPoseIndices.push_back(2u);
    T_BC_list.push_back(T_BCa);

    int anchorExtrinsicRepId = cameraRig.getExtrinsicRepId(anchorCamIdx);
    extrinsicRepIdList.push_back(anchorExtrinsicRepId);
    observationIndices.push_back(anchorObservationIndex);
    frameIndices.push_back(pointData.anchorIds()[0].frameId_);
    T_WBt_list.push_back(T_WBta_lin);
  } else {
    T_CtjX = (T_WBtij * T_BCj).inverse();
  }

  std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry =
      cameraRig.cameraGeometry(camIdx);

  Eigen::Vector4d homogeneousPointLin =
      homogeneousPoint.getLinearizationPoint();
  swift_vio::MultipleTransformPointJacobian mtpj(transformList, exponentList,
                                                 homogeneousPointLin);
  Eigen::Vector3d rhoxpCtjLin = mtpj.evaluate().head<3>();
  okvis::cameras::CameraBase::ProjectionStatus projectionJacStatus;

  Eigen::Vector2d imagePoint;
  projectionJacStatus = cameraGeometry->project(
      rhoxpCtjLin, &imagePoint);
  *residual = obs - imagePoint;

  if (projectionJacStatus != okvis::cameras::CameraBase::ProjectionStatus::Successful) {
    return false;
  }

  if (!estimatorOptions.useMahalanobisGating) {
    if (std::fabs((*residual)[0]) > estimatorOptions.maxProjectionErrorTol ||
        std::fabs((*residual)[1]) > estimatorOptions.maxProjectionErrorTol) {
      return false;
    }
  }
  return true;
}

bool FilterJacobians::measurementJacobianNumeric(
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
    Eigen::Matrix<double, 2, 3>* J_pfi, Eigen::Vector2d* residual) {
  bool status = measurementResidual(homogeneousPoint, obs, observationIndex, pointData,
                                    cameraRig, estimatorOptions, residual);
  if (!status) {
    return false;
  }

  // Lambda function:
  // 1. disturb a state var or a parameter,
  // 2. pointData.computePoseAndVelocityAtObservation();
  // 3. pointData.computePoseAndVelocityForJacobians();
  // 4. measurementResidual();
  // 5. difference to compute the Jacobian
  // 6. reset the state var or parameter.
  // Note that the ImuParameters used in pointData is the same as imuParams.

  // compute J_pfi
  const double h = 1e-5;
  const double invh = 1.0 / h;
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta[i] = h;
    PointLandmark hpPlus = homogeneousPoint;
    hpPlus.PlusForJacobian(delta.data());
    Eigen::Vector2d residualPlus;
    measurementResidual(hpPlus, obs, observationIndex, pointData, cameraRig,
                        estimatorOptions, &residualPlus);
    J_pfi->col(i) = - (residualPlus - *residual) * invh;
  }

  J_x->setZero();
  size_t gravityDimInCov = imuParams->isGravityDirectionVariable() ? 2u : 0u;
  size_t cameraParamStartIndex =
      startIndexOfCameraParamsFast(statesMap, kMainCameraIndex);
  size_t intactDim = cameraParamStartIndex - gravityDimInCov;

  // compute J_T_WBj
  auto frameId = pointData.frameId(observationIndex);
  auto stateIter = statesMap.find(frameId);
  int navStateIndex =
      stateIter->second.global.at(GlobalStates::T_WS).startIndexInCov;
  std::shared_ptr<okvis::ceres::PoseParameterBlock> T_WBj_ptr =
      std::dynamic_pointer_cast<okvis::ceres::PoseParameterBlock>(
          mapPtr->parameterBlockPtr(frameId));
  okvis::kinematics::Transformation T_WBj_lin_orig = T_WBj_ptr->linPoint();

  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta[i] = h;

    okvis::kinematics::Transformation T_WBj;
    swift_vio::PoseLocalParameterizationSimplified::oplus(
        T_WBj_lin_orig.parameterPtr(), delta.data(), T_WBj.parameterMutablePtr());
    T_WBj_ptr->fixLinPoint(T_WBj);

    pointData.computePoseAndVelocityAtObservation();
    pointData.computePoseAndVelocityForJacobians();
    Eigen::Vector2d residualPlus;
    measurementResidual(homogeneousPoint, obs, observationIndex, pointData,
                        cameraRig, estimatorOptions, &residualPlus);
    J_x->col(navStateIndex - intactDim + i) = (residualPlus - *residual) * invh;
  }

  T_WBj_ptr->fixLinPoint(T_WBj_lin_orig);
  pointData.computePoseAndVelocityAtObservation();
  pointData.computePoseAndVelocityForJacobians();

  // compute J_T_WBa
  if (homogeneousPoint.modelId() ==
      swift_vio::InverseDepthParameterization::kModelId) {
      auto frameId = pointData.anchorIds()[0].frameId_;
      auto stateIter = statesMap.find(frameId);
      int navStateIndex =
          stateIter->second.global.at(GlobalStates::T_WS).startIndexInCov;
      std::shared_ptr<okvis::ceres::PoseParameterBlock> T_WBj_ptr =
          std::dynamic_pointer_cast<okvis::ceres::PoseParameterBlock>(
              mapPtr->parameterBlockPtr(frameId));
      okvis::kinematics::Transformation T_WBj_lin_orig = T_WBj_ptr->linPoint();

      for (int i = 0; i < 6; ++i) {
        Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
        delta[i] = h;

        okvis::kinematics::Transformation T_WBj;
        swift_vio::PoseLocalParameterizationSimplified::oplus(
            T_WBj_lin_orig.parameterPtr(), delta.data(), T_WBj.parameterMutablePtr());
        T_WBj_ptr->fixLinPoint(T_WBj);

        pointData.computePoseAndVelocityAtObservation();
        pointData.computePoseAndVelocityForJacobians();
        Eigen::Vector2d residualPlus;
        measurementResidual(homogeneousPoint, obs, observationIndex, pointData,
                            cameraRig, estimatorOptions, &residualPlus);
        J_x->col(navStateIndex - intactDim + i) = (residualPlus - *residual) * invh;
      }
      T_WBj_ptr->fixLinPoint(T_WBj_lin_orig);
      pointData.computePoseAndVelocityAtObservation();
      pointData.computePoseAndVelocityForJacobians();
  }

  // TODO(jhuai): compute other Jacobians by numeric diff.

  (*J_x) *= -1;

  return true;
}


bool FilterJacobians::measurementJacobianAutoDiff(
    const PointLandmark &pointLandmark, const Eigen::Vector2d &obs,
    size_t observationIndex,
    const swift_vio::PointSharedData &pointData,
    const CameraRig &cameraRig,
    const okvis::CameraNoiseParametersVec &camNoiseVec,
    const StateMap &statesMap,
    const okvis::EstimatorOptions &estimatorOptions,
    std::shared_ptr<const okvis::ImuParameters> imuParams,
    Eigen::Matrix<double, 2, -1> *J_X, Eigen::Matrix<double, 2, 3> *J_pfi,
    Eigen::Vector2d *residual) {
  // Refer to http://ceres-solver.org/interfacing_with_autodiff.html
  // http://ceres-solver.org/nnls_modeling.html?highlight=costfunctiontofunctor#costfunctiontofunctor

  // subscript t for target, 0 for main camera, h for host.
  uint64_t poseId = pointData.frameId(observationIndex);
  size_t camIdx = pointData.cameraIndex(observationIndex);
  okvis::cameras::DistortionType distortionType =
      cameraRig.distortionType(camIdx);
  int projIntrinsicRepId = cameraRig.getProjectionIntrinsicRepId(camIdx);
  int extrinsicRepId = cameraRig.getExtrinsicRepId(camIdx);
  int mainCameraExtrinsicRepId =
      cameraRig.getExtrinsicRepId(kMainCameraIndex);

  swift_vio::MeasurementJacobianStatus status =
      swift_vio::MeasurementJacobianStatus::GeneralProjectionFailed;
  Eigen::Matrix2d obsCov =
      Eigen::Matrix2d::Identity(); // identity weight to get raw Jacobians.
  std::shared_ptr<const okvis::cameras::CameraBase> baseCameraGeometry =
      cameraRig.cameraGeometry(camIdx);

  std::shared_ptr<okvis::ceres::ErrorInterface> observationError;
  std::shared_ptr<const okvis::ceres::ParameterBlock> poseParamBlockPtr =
      pointData.poseParameterBlockPtr(observationIndex);
  std::shared_ptr<const okvis::ceres::ParameterBlock> speedParamBlockPtr =
      pointData.speedParameterBlockPtr(observationIndex);
  std::shared_ptr<const okvis::ceres::ParameterBlock> biasParamBlockPtr =
      pointData.biasParameterBlockPtr(observationIndex);

  Eigen::Matrix<double, 9, 1> speedAndBias;
  speedAndBias.head<3>() = Eigen::Map<const Eigen::Vector3d>(
      speedParamBlockPtr->parameters());
  speedAndBias.tail<6>() = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
      biasParamBlockPtr->parameters());
  uint64_t sbid = okvis::IdProvider::instance().newId();
  std::shared_ptr<okvis::ceres::SpeedAndBiasParameterBlock> sbParamBlockPtr(
      new okvis::ceres::SpeedAndBiasParameterBlock(speedAndBias, sbid,
                                                   okvis::Time()));

  auto statesIter = statesMap.find(poseId);
  const States &targetState = statesIter->second;

  okvis::kinematics::Transformation T_BCt =
      cameraRig.getCameraExtrinsic(camIdx);
  okvis::kinematics::Transformation T_BC0 =
      cameraRig.getCameraExtrinsic(kMainCameraIndex);

  Eigen::VectorXd intrinsicParams;
  cameraRig.cameraGeometry(camIdx)->getIntrinsics(intrinsicParams);
  int intrinsicDim = intrinsicParams.size();

  const double *tdParamBlockPtr =
      pointData.cameraTimeDelayParameterBlockPtr(camIdx);
  const double *trParamBlockPtr =
      pointData.frameReadoutTimeParameterBlockPtr(camIdx);

  switch (estimatorOptions.cameraObservationModelId) {
  case okvis::ceres::RsReprojectionErrorBase::kModelId: {
    // create cost function.
    std::shared_ptr<const okvis::ImuMeasurementDeque> imuMeasDequePtr =
        targetState.imuReadingWindow;
    std::shared_ptr<Eigen::Matrix<double, 6, 1>> posVelLin(new Eigen::Matrix<double, 6, 1>());
    *posVelLin = pointData.posVelLinPoint(observationIndex);

    okvis::Time stateEpoch = targetState.timestamp;
    okvis::Time imageTime = pointData.imageTime(observationIndex);
    double gravityNorm = pointData.gravityNorm();
    switch (distortionType) {
#define DISTORTION_MODEL_CASE(CameraGeometry)                                  \
  {                                                                            \
    std::shared_ptr<const CameraGeometry> targetCamera =                       \
        std::static_pointer_cast<const CameraGeometry>(baseCameraGeometry);    \
    observationError.reset(                                                    \
        new okvis::ceres::RsReprojectionError<CameraGeometry>(                 \
            targetCamera, obs, obsCov, imuMeasDequePtr, posVelLin, stateEpoch, \
            imageTime, gravityNorm));                                          \
  }

      DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES

#undef DISTORTION_MODEL_CASE
    }

    uint64_t gid = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::NormalVectorParameterBlock>
        unitgWBlockPtr(
            new okvis::ceres::NormalVectorParameterBlock(imuParams->gravityDirection(), gid));

    // compute residual with the latest estimate.
    double const *parameters[] = {poseParamBlockPtr->parameters(),
                                  pointLandmark.data(),
                                  T_BCt.parameterPtr(),
                                  intrinsicParams.data(),
                                  trParamBlockPtr,
                                  tdParamBlockPtr,
                                  sbParamBlockPtr->parameters(),
                                  unitgWBlockPtr->parameters()};
    bool resEvaluateOk = observationError->EvaluateWithMinimalJacobians(
        parameters, residual->data(), nullptr, nullptr);

    // compute Jacobians with the specified linearization points.
    Eigen::Map<const Eigen::Quaterniond> q_WBt(poseParamBlockPtr->parameters() +
                                               3);
    okvis::kinematics::Transformation T_WBt_lin(posVelLin->head<3>(), q_WBt);
    parameters[0] = T_WBt_lin.parameterPtr();
    Eigen::Map<const Eigen::Matrix<double, 9, 1>> latestSpeedAndBias(
        sbParamBlockPtr->parameters());
    okvis::SpeedAndBiases speedAndBiasLin = latestSpeedAndBias;
    speedAndBiasLin.head<3>() = posVelLin->tail<3>();
    parameters[6] = speedAndBiasLin.data();

    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> duv_dT_WBt;
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> duv_dT_WBt_minimal;
    Eigen::Matrix<double, 2, 4, Eigen::RowMajor> duv_dhpW;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> duv_dhpW_minimal;
    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> duv_dT_BCt;
    Eigen::Matrix<double, 2, Extrinsic_p_BC_q_BC::kNumParams, Eigen::RowMajor>
        duv_dT_BCt_minimal;
    Eigen::Matrix<double, 2, -1, Eigen::RowMajor> duv_dIntrinsic(2, intrinsicDim);
    Eigen::Matrix<double, 2, 1> duv_dtr;
    Eigen::Matrix<double, 2, 1> duv_dtd;
    Eigen::Matrix<double, 2, 9, Eigen::RowMajor> duv_dsb;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> duv_dunitgW;
    Eigen::Matrix<double, 2, 2, Eigen::RowMajor> duv_dunitgW_minimal;

    double *jacobians[] = {duv_dT_WBt.data(), duv_dhpW.data(),
                           duv_dT_BCt.data(), duv_dIntrinsic.data(),
                           duv_dtr.data(),    duv_dtd.data(),
                           duv_dsb.data(),    duv_dunitgW.data()};
    double *jacobiansMinimal[] = {duv_dT_WBt_minimal.data(),
                                  duv_dhpW_minimal.data(),
                                  duv_dT_BCt_minimal.data(),
                                  nullptr,
                                  nullptr,
                                  nullptr,
                                  nullptr,
                                  duv_dunitgW_minimal.data()};
    Eigen::Vector2d residualUnused;
    bool jacEvaluateOk = observationError->EvaluateWithMinimalJacobians(
        parameters, residualUnused.data(), jacobians, jacobiansMinimal);
    if (resEvaluateOk && jacEvaluateOk) {
      status = swift_vio::MeasurementJacobianStatus::Successful;
    }

    // project Jacobians to minimal extrinsic and intrinsic parameterization.
    Eigen::AlignedVector<Eigen::MatrixXd> dT_BCi_dExtrinsics;
    std::vector<size_t> involvedCameraIndices;
    computeExtrinsicJacobians(
        T_BCt, T_BC0, extrinsicRepId, mainCameraExtrinsicRepId, camIdx,
        kMainCameraIndex, &dT_BCi_dExtrinsics, &involvedCameraIndices);

    // find indices in the covariance matrix for these Jacobian blocks.
    std::vector<std::pair<int, size_t>> startIndexToMinDim;
    startIndexToMinDim.reserve(10);
    Eigen::AlignedVector<Eigen::Matrix<double, 2, -1>> duv_dX_list;

    size_t gravityDimInCov = imuParams->isGravityDirectionVariable() ? 2u : 0u;
    size_t cameraParamStartIndex =
        startIndexOfCameraParamsFast(statesMap, kMainCameraIndex);
    size_t intactDim = cameraParamStartIndex - gravityDimInCov;
    // gravity.
    if (gravityDimInCov) {
      startIndexToMinDim.emplace_back(intactDim, gravityDimInCov);
      double featureDelay =
          pointData.normalizedFeatureTime(observationIndex);
      Eigen::Matrix<double, 6, 2> dT_WBt_dunitgW;
      dT_WBt_dunitgW.topRows<3>() =
          swift_vio::SimpleImuPropagationJacobian::dp_dunitgW(
              featureDelay, imuParams->gravityDirection(),
              imuParams->g);
      duv_dX_list.emplace_back(duv_dT_WBt_minimal.leftCols(3) *
                               dT_WBt_dunitgW.topRows<3>());
    }

    // camera extrinsics
    if (involvedCameraIndices.size() > 1u &&
        !camNoiseVec.at(kMainCameraIndex).isExtrinsicsFixed()) {
      Eigen::Matrix<double, 2, -1> duv_dExtrinsic0_minimal =
          duv_dT_BCt_minimal * dT_BCi_dExtrinsics[1];
      startIndexToMinDim.emplace_back(
          startIndexOfCameraParamsFast(statesMap, kMainCameraIndex),
          duv_dExtrinsic0_minimal.cols());
      duv_dX_list.emplace_back(duv_dExtrinsic0_minimal);
    }

    if (!camNoiseVec.at(camIdx).isExtrinsicsFixed()) {
      Eigen::Matrix<double, 2, -1> duv_dExtrinsici_minimal =
          duv_dT_BCt_minimal * dT_BCi_dExtrinsics[0];
      startIndexToMinDim.emplace_back(startIndexOfCameraParamsFast(statesMap, camIdx),
                                      duv_dExtrinsici_minimal.cols());
      duv_dX_list.emplace_back(duv_dExtrinsici_minimal);
    }
    // camera intrinsics including distortion for the target camera
    if (!camNoiseVec.at(camIdx).isIntrinsicsFixed()) {
      startIndexToMinDim.emplace_back(
          startIndexOfCameraParamsFast(statesMap,
              camIdx, CameraSensorStates::Intrinsics),
          duv_dIntrinsic.cols());
      duv_dX_list.emplace_back(duv_dIntrinsic);
    }
    // camera time offset and readout time
    if (!camNoiseVec.at(camIdx).isReadoutTimeFixed()) {
      startIndexToMinDim.emplace_back(
          startIndexOfCameraParamsFast(statesMap, camIdx,
                                       CameraSensorStates::TR),
          duv_dtr.cols());
      duv_dX_list.emplace_back(duv_dtr);
    }
    if (!camNoiseVec.at(camIdx).isReadoutTimeFixed()) {
      startIndexToMinDim.emplace_back(
          startIndexOfCameraParamsFast(statesMap, camIdx,
                                       CameraSensorStates::TD),
          duv_dtd.cols());
      duv_dX_list.emplace_back(duv_dtd);
    }

    // nav state for the target camera
    int orderInCov = targetState.global.at(GlobalStates::T_WS).startIndexInCov;
    startIndexToMinDim.emplace_back(orderInCov, duv_dT_WBt_minimal.cols());
    duv_dX_list.emplace_back(duv_dT_WBt_minimal);

    startIndexToMinDim.emplace_back(orderInCov + 6, 3);
    duv_dX_list.emplace_back(duv_dsb.leftCols<3>());

    // We discard Jacobians relative to biases and IMU intrinsic parameters
    // as the current filter design does not support them.

    // assemble the whole Jacobian with blocks.
    J_X->setZero();
    int j = 0;
    for (const auto &indexToDim : startIndexToMinDim) {
      J_X->block(0, indexToDim.first - intactDim, 2, indexToDim.second) +=
          duv_dX_list.at(j);
      ++j;
    }
    *J_pfi = duv_dhpW.topLeftCorner<2, 3>();
    *residual = -(*residual);
    break;
  }

  case okvis::ceres::ReprojectionError2dBase::kModelId:
  case okvis::ceres::RsReprojectionErrorAidpBase::kModelId: {
    // create cost function.
    std::shared_ptr<const okvis::ImuMeasurementDeque> imuMeasDequePtr =
        targetState.imuReadingWindow;

    std::shared_ptr<Eigen::Matrix<double, 6, 1>> posVelLin(new Eigen::Matrix<double, 6, 1>());
    *posVelLin = pointData.posVelLinPoint(observationIndex);

    okvis::Time stateEpoch = targetState.timestamp;
    okvis::Time imageTime = pointData.imageTime(observationIndex);
    switch (distortionType) {
#define DISTORTION_MODEL_CASE(CameraGeometry)                                  \
  {                                                                            \
    std::shared_ptr<const CameraGeometry> targetCamera =                       \
        std::static_pointer_cast<const CameraGeometry>(baseCameraGeometry);    \
    observationError.reset(                                                    \
        new okvis::ceres::RsReprojectionErrorAidp<CameraGeometry>(           \
            obs, obsCov, targetCamera, imuMeasDequePtr,                        \
            imuParams, stateEpoch, imageTime));                  \
  }

      DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES

#undef DISTORTION_MODEL_CASE
    }

    uint64_t anchorFrameId = pointData.anchorIds().at(0).frameId_;
    size_t anchorCameraId = pointData.anchorIds().at(0).cameraIndex_;
    size_t anchorObservationId =
        pointData.anchorIds().at(0).observationIndex_;
    std::shared_ptr<const okvis::ceres::ParameterBlock> hostPoseParamBlockPtr =
        pointData.poseParameterBlockPtr(anchorObservationId);
    okvis::kinematics::Transformation T_BCh =
        cameraRig.getCameraExtrinsic(anchorCameraId);
    std::vector<const double *> imuAugmentedParameterPtrs = pointData.imuAugmentedParameterPtrs();
    Eigen::Matrix<double, 9, 1> shapeMatrices[3];
    if (imuAugmentedParameterPtrs.size() == 0) {
      for (int j = 0; j < 3; ++j) {
        shapeMatrices[j] = Eigen::Matrix<double, 9, 1>::Zero();
        imuAugmentedParameterPtrs.emplace_back(shapeMatrices[j].data());
      }
    }
    // compute residual with the latest estimate.
    double const *parameters[] = {poseParamBlockPtr->parameters(),
                                  pointLandmark.data(),
                                  hostPoseParamBlockPtr->parameters(),
                                  T_BCt.parameterPtr(),
                                  T_BCh.parameterPtr(),
                                  intrinsicParams.data(),
                                  trParamBlockPtr,
                                  tdParamBlockPtr,
                                  speedParamBlockPtr->parameters(),
                                  biasParamBlockPtr->parameters(),
                                  imuAugmentedParameterPtrs[0],
                                  imuAugmentedParameterPtrs[1],
                                  imuAugmentedParameterPtrs[2]};

    bool resEvaluateOk = observationError->EvaluateWithMinimalJacobians(
        parameters, residual->data(), nullptr, nullptr);

    // compute Jacobians with the specified linearization points.
    okvis::kinematics::Transformation T_WBt_lin = pointData.poseLinPoint(observationIndex);
    parameters[0] = T_WBt_lin.parameterPtr();

    okvis::kinematics::Transformation T_WBh_lin = pointData.poseLinPoint(anchorObservationId);
    parameters[2] = T_WBh_lin.parameterPtr();

    Eigen::Vector3d velLin = pointData.velLinPoint(observationIndex);
    parameters[8] = velLin.data();

    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> duv_dT_WBt;
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> duv_dT_WBt_minimal;
    Eigen::Matrix<double, 2, 4, Eigen::RowMajor> duv_dhpW;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> duv_dhpW_minimal;
    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> duv_dT_WBh;
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> duv_dT_WBh_minimal;
    Eigen::Matrix<double, 2, 7, Eigen::RowMajor>
        duv_dT_BC[2]; // for target and host respectively.
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> duv_dT_BC_minimal[2];

    Eigen::Matrix<double, 2, -1, Eigen::RowMajor> duv_dIntrinsic(2,
                                                                 intrinsicDim);
    Eigen::Matrix<double, 2, -1, Eigen::RowMajor> duv_dIntrinsic_minimal(
        2, intrinsicDim);

    Eigen::Matrix<double, 2, 1> duv_dtr;
    Eigen::Matrix<double, 2, 1> duv_dtr_minimal;
    Eigen::Matrix<double, 2, 1> duv_dtd;
    Eigen::Matrix<double, 2, 1> duv_dtd_minimal;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> duv_dspeed;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> duv_dspeed_minimal;
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> duv_dbias;
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> duv_dbias_minimal;

    double *jacobians[] = {duv_dT_WBt.data(),
                           duv_dhpW.data(),
                           duv_dT_WBh.data(),
                           duv_dT_BC[0].data(),
                           duv_dT_BC[1].data(),
                           duv_dIntrinsic.data(),
                           duv_dtr.data(),
                           duv_dtd.data(),
                           duv_dspeed.data(),
                           duv_dbias.data(),
                           nullptr,
                           nullptr,
                           nullptr};
    double *jacobiansMinimal[] = {duv_dT_WBt_minimal.data(),
                                  duv_dhpW_minimal.data(),
                                  duv_dT_WBh_minimal.data(),
                                  duv_dT_BC_minimal[0].data(),
                                  duv_dT_BC_minimal[1].data(),
                                  duv_dIntrinsic_minimal.data(),
                                  duv_dtr_minimal.data(),
                                  duv_dtd_minimal.data(),
                                  duv_dspeed_minimal.data(),
                                  duv_dbias_minimal.data(),
                                  nullptr,
                                  nullptr,
                                  nullptr};
    Eigen::Vector2d residualUnused;
    bool jacEvaluateOk = observationError->EvaluateWithMinimalJacobians(
        parameters, residualUnused.data(), jacobians, jacobiansMinimal);
    if (resEvaluateOk && jacEvaluateOk) {
      status = swift_vio::MeasurementJacobianStatus::Successful;
    }

    // project Jacobians to minimal extrinsic and intrinsic parameterization.
    Eigen::AlignedVector<Eigen::MatrixXd>
        dT_BC_dExtrinsics[2]; // two components correspond to target and host
                              // extrinsics.
    std::vector<size_t> involvedCameraIndices[2];
    computeExtrinsicJacobians(
        T_BCt, T_BC0, extrinsicRepId, mainCameraExtrinsicRepId, camIdx,
        kMainCameraIndex, &dT_BC_dExtrinsics[0], &involvedCameraIndices[0]);
    int hostExtrinsicRepId = cameraRig.getExtrinsicRepId(anchorCameraId);
    computeExtrinsicJacobians(T_BCh, T_BC0, hostExtrinsicRepId,
                              mainCameraExtrinsicRepId, anchorCameraId,
                              kMainCameraIndex, &dT_BC_dExtrinsics[1],
                              &involvedCameraIndices[1]);

    Eigen::Matrix<double, 2, -1> duv_dIntrinsic_reduced =
        duv_dIntrinsic_minimal;
    ProjIntrinsicRepMinimalIntrinsicJacobian(projIntrinsicRepId,
                                          &duv_dIntrinsic_reduced);

    // find indices in the covariance matrix for these Jacobian blocks.
    std::vector<std::pair<int, size_t>> startIndexToMinDim;
    startIndexToMinDim.reserve(10);
    Eigen::AlignedVector<Eigen::Matrix<double, 2, -1>> duv_dX_list;

    size_t gravityDimInCov = imuParams->isGravityDirectionVariable() ? 2u : 0u;
    size_t cameraParamStartIndex =
        startIndexOfCameraParamsFast(statesMap, kMainCameraIndex);
    size_t intactDim = cameraParamStartIndex - gravityDimInCov;
    // gravity.
    if (gravityDimInCov) {
      startIndexToMinDim.emplace_back(intactDim, gravityDimInCov);
      double featureDelay =
          pointData.normalizedFeatureTime(observationIndex);
      Eigen::Matrix<double, 6, 2> dT_WBt_dunitgW;
      dT_WBt_dunitgW.topRows<3>() =
          swift_vio::SimpleImuPropagationJacobian::dp_dunitgW(
              featureDelay, imuParams->gravityDirection(),
              imuParams->g);
      duv_dX_list.emplace_back(duv_dT_WBt_minimal.leftCols(3) *
                               dT_WBt_dunitgW.topRows<3>());
    }

    // camera extrinsics
    std::vector<size_t> targetHostCameraIds{camIdx, anchorCameraId};
    for (int e = 0; e < 2; ++e) {
      if (involvedCameraIndices[e].size() > 1u &&
          !camNoiseVec.at(kMainCameraIndex).isExtrinsicsFixed()) {
        Eigen::Matrix<double, 2, -1> duv_dExtrinsic0_minimal =
            duv_dT_BC_minimal[e] * dT_BC_dExtrinsics[e][1];
        startIndexToMinDim.emplace_back(
            startIndexOfCameraParamsFast(statesMap, kMainCameraIndex),
            duv_dExtrinsic0_minimal.cols());
        duv_dX_list.emplace_back(duv_dExtrinsic0_minimal);
      }

      if (!camNoiseVec.at(targetHostCameraIds[e]).isExtrinsicsFixed()) {
        Eigen::Matrix<double, 2, -1> duv_dExtrinsici_minimal =
            duv_dT_BC_minimal[e] * dT_BC_dExtrinsics[e][0];
        startIndexToMinDim.emplace_back(
            startIndexOfCameraParamsFast(statesMap, targetHostCameraIds[e]),
            duv_dExtrinsici_minimal.cols());
        duv_dX_list.emplace_back(duv_dExtrinsici_minimal);
      }
    }
    // camera intrinsics including distortion for the target camera
    if (!camNoiseVec.at(camIdx).isIntrinsicsFixed()) {
      startIndexToMinDim.emplace_back(
          startIndexOfCameraParamsFast(statesMap,
              camIdx, CameraSensorStates::Intrinsics),
          duv_dIntrinsic_reduced.cols());
      duv_dX_list.emplace_back(duv_dIntrinsic_reduced);
    }
    // camera time offsets and readout times
    if (!camNoiseVec.at(camIdx).isReadoutTimeFixed()) {
      startIndexToMinDim.emplace_back(
          startIndexOfCameraParamsFast(statesMap, camIdx,
                                       CameraSensorStates::TR),
          duv_dtr_minimal.cols());
      duv_dX_list.emplace_back(duv_dtr_minimal);
    }
    if (!camNoiseVec.at(camIdx).isTimeDelayFixed()) {
      startIndexToMinDim.emplace_back(
          startIndexOfCameraParamsFast(statesMap, camIdx,
                                       CameraSensorStates::TD),
          duv_dtd_minimal.cols());
      duv_dX_list.emplace_back(duv_dtd_minimal);
    }
    // nav state for the target camera
    int orderInCov = targetState.global.at(GlobalStates::T_WS).startIndexInCov;
    startIndexToMinDim.emplace_back(orderInCov, duv_dT_WBt_minimal.cols());
    duv_dX_list.emplace_back(duv_dT_WBt_minimal);

    startIndexToMinDim.emplace_back(orderInCov + 6, 3);
    duv_dX_list.emplace_back(duv_dspeed_minimal);

    // We discard Jacobians relative to biases and IMU intrinsic parameters
    // as the filter design does not support them.

    // nav state for the host camera.
    const auto &hostStateIter = statesMap.find(anchorFrameId);
    int hostOrderInCov =
        hostStateIter->second.global.at(GlobalStates::T_WS).startIndexInCov;
    startIndexToMinDim.emplace_back(hostOrderInCov, duv_dT_WBh_minimal.cols());
    duv_dX_list.emplace_back(duv_dT_WBh_minimal);

    // assemble the whole Jacobian with blocks.
    J_X->setZero();
    int j = 0;
    for (const auto &indexToDim : startIndexToMinDim) {
      J_X->block(0, indexToDim.first - intactDim, 2, indexToDim.second) +=
          duv_dX_list.at(j);
      ++j;
    }
    Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dhp_dparams;
    InverseDepthParameterization::plusJacobian(nullptr, dhp_dparams.data());

    *J_pfi = duv_dhpW * dhp_dparams;
    *residual = -(*residual);
    break;
  }

  case okvis::ceres::RsReprojectionErrorPapBase::kModelId: {
    switch (distortionType) {
#define DISTORTION_MODEL_CASE(CameraGeometry)                                  \
  {                                                                            \
    std::shared_ptr<const CameraGeometry> targetCamera =                       \
        std::static_pointer_cast<const CameraGeometry>(baseCameraGeometry);    \
    observationError.reset(                                                    \
        new okvis::ceres::RsReprojectionErrorPap<CameraGeometry>(              \
            targetCamera, obs, obsCov, observationIndex, &pointData));         \
  }

      DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES

#undef DISTORTION_MODEL_CASE
    }
    OKVIS_ASSERT_TRUE(
        std::runtime_error, false,
        "RsReprojectionErrorPap Jacobian computation is not maintained!");

    std::vector<size_t> anchorObservationIndices =
        pointData.anchorObservationIds();
    std::vector<std::shared_ptr<const okvis::ceres::ParameterBlock>>
        anchorPoseBlockPtrs;
    anchorPoseBlockPtrs.reserve(2);
    std::vector<std::shared_ptr<const okvis::ceres::ParameterBlock>>
        anchorSpeedAndBiasBlockPtrs;
    anchorSpeedAndBiasBlockPtrs.reserve(2);
    for (auto anchorObsId : anchorObservationIndices) {
      anchorPoseBlockPtrs.push_back(
          pointData.poseParameterBlockPtr(anchorObsId));
      anchorSpeedAndBiasBlockPtrs.push_back(
          pointData.speedAndBiasParameterBlockPtr(anchorObsId));
    }

    double const *const parameters[] = {
        poseParamBlockPtr->parameters(),
        anchorPoseBlockPtrs[0]->parameters(),
        anchorPoseBlockPtrs[1]->parameters(),
        pointLandmark.data(),
        T_BCt.parameterPtr(),
        intrinsicParams.data(),
        trParamBlockPtr,
        tdParamBlockPtr,
        sbParamBlockPtr->parameters(),
        anchorSpeedAndBiasBlockPtrs[0]->parameters(),
        anchorSpeedAndBiasBlockPtrs[1]->parameters()};

    const int krd = okvis::ceres::RsReprojectionErrorPapBase::kNumResiduals;
    const int kPoseNumber = 3;
    // The elements are de_dTWBj, de_dTWBmi, de_dTWBai.
    std::vector<Eigen::Matrix<double, krd, 7, Eigen::RowMajor>,
                Eigen::aligned_allocator<
                    Eigen::Matrix<double, krd, 7, Eigen::RowMajor>>>
        de_dTWB(kPoseNumber);
    std::vector<Eigen::Matrix<double, krd, 6, Eigen::RowMajor>,
                Eigen::aligned_allocator<
                    Eigen::Matrix<double, krd, 6, Eigen::RowMajor>>>
        de_dTWB_minimal(kPoseNumber);
    std::vector<Eigen::Matrix<double, krd, 9, Eigen::RowMajor>,
                Eigen::aligned_allocator<
                    Eigen::Matrix<double, krd, 9, Eigen::RowMajor>>>
        de_dSpeedAndBias(kPoseNumber);
    std::vector<Eigen::Matrix<double, krd, 9, Eigen::RowMajor>,
                Eigen::aligned_allocator<
                    Eigen::Matrix<double, krd, 9, Eigen::RowMajor>>>
        de_dSpeedAndBias_minimal(kPoseNumber);

    Eigen::Matrix<double, krd, 4, Eigen::RowMajor> de_dPoint;
    Eigen::Matrix<double, krd, 3, Eigen::RowMajor> de_dPoint_minimal;
    Eigen::Matrix<double, krd, Extrinsic_p_BC_q_BC::kGlobalDim, Eigen::RowMajor>
        de_dExtrinsic;
    Eigen::Matrix<double, krd, Extrinsic_p_BC_q_BC::kNumParams, Eigen::RowMajor>
        de_dExtrinsic_minimal;

    Eigen::Matrix<double, krd, -1, Eigen::RowMajor> de_dintrinsic(krd,
                                                                  intrinsicDim);
    Eigen::Matrix<double, krd, -1, Eigen::RowMajor> de_dintrinsic_minimal(
        krd, intrinsicDim);
    Eigen::Matrix<double, krd, 1> de_dtr;
    Eigen::Matrix<double, krd, 1> de_dtr_minimal;
    Eigen::Matrix<double, krd, 1> de_dtd;
    Eigen::Matrix<double, krd, 1> de_dtd_minimal;

    double *jacobians[] = {de_dTWB[0].data(),
                           de_dTWB[1].data(),
                           de_dTWB[2].data(),
                           de_dPoint.data(),
                           de_dExtrinsic.data(),
                           de_dintrinsic.data(),
                           de_dtr.data(),
                           de_dtd.data(),
                           de_dSpeedAndBias[0].data(),
                           de_dSpeedAndBias[1].data(),
                           de_dSpeedAndBias[2].data()};
    double *jacobiansMinimal[] = {de_dTWB_minimal[0].data(),
                                  de_dTWB_minimal[1].data(),
                                  de_dTWB_minimal[2].data(),
                                  de_dPoint_minimal.data(),
                                  de_dExtrinsic_minimal.data(),
                                  de_dintrinsic_minimal.data(),
                                  de_dtr_minimal.data(),
                                  de_dtd_minimal.data(),
                                  de_dSpeedAndBias_minimal[0].data(),
                                  de_dSpeedAndBias_minimal[1].data(),
                                  de_dSpeedAndBias_minimal[2].data()};
    bool evaluateOk = observationError->EvaluateWithMinimalJacobians(
        parameters, residual->data(), jacobians, jacobiansMinimal);
    const std::vector<AnchorFrameIdentifier> &anchorIds =
        pointData.anchorIds();
    if (evaluateOk) {
      status = swift_vio::MeasurementJacobianStatus::Successful;
    } else {
      if (anchorIds[0].frameId_ == poseId) {
        status =
            swift_vio::MeasurementJacobianStatus::MainAnchorProjectionFailed;
      } else if (anchorIds[1].frameId_ == poseId) {
        status = swift_vio::MeasurementJacobianStatus::
            AssociateAnchorProjectionFailed;
      }
    }

    J_X->setZero();
    int jacColIndex = 0;
    if (!camNoiseVec.at(camIdx).isExtrinsicsFixed()) {
      J_X->block(0, jacColIndex, krd, de_dExtrinsic_minimal.cols()) = de_dExtrinsic_minimal;
      jacColIndex += de_dExtrinsic_minimal.cols();
    }

    if (!camNoiseVec.at(camIdx).isIntrinsicsFixed()) {
      J_X->block(0, jacColIndex, krd, de_dintrinsic_minimal.cols()) = de_dintrinsic_minimal;
      jacColIndex += de_dintrinsic_minimal.cols();
    }

    if (!camNoiseVec.at(camIdx).isTimeDelayFixed()) {
      J_X->block(0, jacColIndex, krd, 1) = de_dtd_minimal;
      ++jacColIndex;
    }

    if (!camNoiseVec.at(camIdx).isReadoutTimeFixed()) {
      J_X->block(0, jacColIndex, krd, 1) = de_dtr_minimal;
      ++jacColIndex;
    }

    size_t cameraParamStartIndex =
        startIndexOfCameraParamsFast(statesMap, kMainCameraIndex);
    std::vector<uint64_t> jmaFrameIds{poseId, anchorIds[0].frameId_,
                                      anchorIds[1].frameId_};
    for (int f = 0; f < 3; ++f) {
      uint64_t frameId = jmaFrameIds[f];
      auto smIter = statesMap.find(frameId);
      size_t orderInCov =
          smIter->second.global.at(GlobalStates::T_WS).startIndexInCov;
      J_X->block<krd, kClonedStateMinimalDim>(0, orderInCov -
                                                       cameraParamStartIndex)
          << de_dTWB_minimal[f],
          de_dSpeedAndBias_minimal[f].template topLeftCorner<krd, 3>();
    }

    *J_pfi = de_dPoint_minimal;
    *residual = -(*residual);
    break;
  }
  default:
    MODEL_DOES_NOT_EXIST_EXCEPTION
    break;
  }
  return status == MeasurementJacobianStatus::Successful;
}

FeatureJacobianStatus FilterJacobians::featureJacobian(
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
    std::vector<uint64_t> *orderedCulledFrameIds, size_t minTrackLength,
    size_t minCulledFrames) {
  // all observations for this feature point
  Eigen::AlignedVector<Eigen::Vector2d> obsList;
  Eigen::AlignedVector<Eigen::Vector2d> obsStdList; // std in pixels
  FeatureJacobianStatus jacobianStatus;

  jacobianStatus.triangulationStatus = EstimationHelper::triangulateMapPointRs(
      mp, statesMap, mapPtr, frameTimeMap, cameraRig, imuParams, pointLandmark,
      &obsList, &obsStdList, pointDataPtr, orderedCulledFrameIds,
      minTrackLength, minCulledFrames, false, false);

  if (!jacobianStatus.triangulationStatus.triangulationOk) {
    return jacobianStatus;
  }

  const size_t gravityDimInCov = imuParams->isGravityDirectionVariable() ? 2u : 0u;
  // dimension of variables used in computing feature Jacobians, including
  // gravity direction, camera intrinsics and cloned states except the most recent one
  // in which an observation should never occur for a MSCKF feature.
  int featureVariableDim =
      gravityDimInCov +
      minimalDimOfAllCameraParams(statesMap, camNoiseVec.back()) +
      kClonedStateMinimalDim * (statesMap.size() - 1);
  if (pH_fi == NULL) {
    CHECK_NE(statesMap.rbegin()->first, pointDataPtr->lastFrameId())
        << "The landmark should not be observed by the latest frame for an "
           "MSCKF feature.";
  } else {
    featureVariableDim += 9;
  }
  pointDataPtr->computePoseAndVelocityForJacobians();

  size_t numObservations = pointDataPtr->numObservations();
  Eigen::AlignedVector<Eigen::Matrix<double, 2, 3>> vJ_pfi;
  Eigen::AlignedVector<Eigen::Matrix<double, 2, 1>> vri;  // residuals for feature i
  vJ_pfi.reserve(numObservations);
  vri.reserve(numObservations);

  // containers of the above Jacobians for all observations of a mappoint
  Eigen::AlignedVector<Eigen::Matrix<double, 2, Eigen::Dynamic>> vJ_X;
  vJ_X.reserve(numObservations);

  size_t numValidObs = 0u;
  auto observationIter = pointDataPtr->begin();
  Eigen::AlignedVector<Eigen::Vector2d> obsStdKeep;
  obsStdKeep.reserve(numObservations);
  // compute Jacobians for a measurement in image j of the current feature i
  for (size_t observationIndex = 0; observationIndex < numObservations; ++observationIndex) {
    Eigen::Matrix<double, 2, Eigen::Dynamic> J_x(2, featureVariableDim);
    Eigen::Matrix<double, 2, 3> J_pfi;
    Eigen::Vector2d residual;

    bool validJacobian = measurementJacobian(
        *pointLandmark, obsList[observationIndex],
        observationIndex, *pointDataPtr, cameraRig, camNoiseVec, statesMap,
        estimatorOptions, imuParams, &J_x, &J_pfi, &residual, FLAGS_mix_projection_jac);
    if (!validJacobian) {
        ++observationIter;
        continue;
    }

    vri.push_back(residual);
    vJ_X.push_back(J_x);
    vJ_pfi.push_back(J_pfi);
    obsStdKeep.emplace_back(obsStdList[observationIndex]);
    ++numValidObs;
    ++observationIter;
  }
  if (numValidObs < minTrackLength) {
    jacobianStatus.triangulationStatus.lackObservations = true;
    return jacobianStatus;
  }

  // Now we stack the Jacobians and marginalize the point position related
  // dimensions by projecting \f$H_{x_i}\f$ onto the nullspace of $H_{f^i}$.
  Eigen::MatrixXd H_xi(2 * numValidObs, featureVariableDim);
  Eigen::MatrixXd H_fi(2 * numValidObs, 3);
  Eigen::Matrix<double, Eigen::Dynamic, 1> ri(2 * numValidObs, 1);
  Eigen::MatrixXd Ri =
      Eigen::MatrixXd::Identity(2 * numValidObs, 2 * numValidObs);
  for (size_t saga = 0; saga < numValidObs; ++saga) {
    size_t saga2 = saga * 2;
    H_xi.block(saga2, 0, 2, featureVariableDim) = vJ_X[saga];
    H_fi.block<2, 3>(saga2, 0) = vJ_pfi[saga];
    ri.segment<2>(saga2) = vri[saga];
    Ri.block<2, 2>(saga2, saga2) = obsStdKeep[saga].cwiseAbs2().asDiagonal();
  }

  if (pH_fi) { // this point is to be included in the state vector.
    r_oi = ri;
    H_oi = H_xi;
    R_oi = Ri;
    *pH_fi = H_fi;
  } else {
    int columnRankHf = jacobianStatus.triangulationStatus.raysParallel ? 2 : 3;
    // 2nx(2n-ColumnRank), n==numValidObs
    Eigen::MatrixXd nullQ = FilterHelper::leftNullspaceWithRankCheck(H_fi, columnRankHf);

    r_oi.noalias() = nullQ.transpose() * ri;
    H_oi.noalias() = nullQ.transpose() * H_xi;
    R_oi = nullQ.transpose() * (Ri * nullQ).eval();
  }

  vri.clear();
  vJ_pfi.clear();
  vJ_X.clear();
  jacobianStatus.successful = true;
  return jacobianStatus;
}

bool FilterJacobians::slamFeatureJacobian(const MapPoint &mp, const StateMap &statesMap,
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
                         Eigen::MatrixXd &R_i, Eigen::MatrixXd &subH_f) {
  std::shared_ptr<swift_vio::PointSharedData> pointDataPtr(new swift_vio::PointSharedData());

  Eigen::AlignedVector<Eigen::Vector2d> obsList;
  obsList.reserve(2);
  Eigen::AlignedVector<Eigen::Vector2d> obsStdList;
  obsStdList.reserve(2);

  if (homoPointRep.modelId() ==
      swift_vio::InverseDepthParameterization::kModelId) {
    // add the observation for anchor camera frame to simplify computing Jacobians.
    okvis::KeypointIdentifier anchorFrameId(mp.anchorStateId, mp.anchorCameraId, 0u);
    auto itObs = std::find_if(
        mp.observations.begin(), mp.observations.end(),
        [anchorFrameId](const std::pair<okvis::KeypointIdentifier, KeypointObservation> &v) {
          return v.first.frameId == anchorFrameId.frameId &&
                 v.first.cameraIndex == anchorFrameId.cameraIndex;
        });
    OKVIS_ASSERT_FALSE(std::runtime_error, itObs == mp.observations.end(),
                       "Anchor observation not found!");

    uint64_t poseId = mp.anchorStateId;
    Eigen::Vector2d measurement = itObs->second.uv.cast<double>();

    okvis::Time imageTimestamp = frameTimeMap.at(poseId).timestamp(itObs->first.cameraIndex);
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry =
        cameraRig.cameraGeometry(itObs->first.cameraIndex);
    uint32_t imageHeight = cameraGeometry->imageHeight();
    obsList.push_back(measurement);

    double kpSize = itObs->second.size;
    double sigma = kpSize / 8;
    obsStdList.emplace_back(Eigen::Vector2d::Constant(sigma));

    std::shared_ptr<const okvis::ceres::ParameterBlock> parameterBlockPtr = mapPtr->parameterBlockPtr(poseId);

    double kpN = measurement[1] / imageHeight - 0.5;
    pointDataPtr->addKeypointObservation(
        itObs->first,
        std::dynamic_pointer_cast<
            const okvis::ceres::PoseParameterBlock>(
            parameterBlockPtr),
        kpN, imageTimestamp);

    AnchorFrameIdentifier anchorId{mp.anchorStateId, mp.anchorCameraId, 0u};
    pointDataPtr->setAnchors({anchorId});
  }

  // add observations in images of the current frame.
  int cameraId = -1;
  size_t numNewObservations = 0u;
  for (auto itObs = mp.observations.rbegin(), iteObs = mp.observations.rend();
       itObs != iteObs; ++itObs) {
    if (itObs->first.frameId == currentFrameId) {
      if (static_cast<int>(itObs->first.cameraIndex) == cameraId) { // avoid double observations.
        VLOG(2) << "The frontend may associate one landmark with two observations in the same image!";
        continue;
      }
      uint64_t poseId = itObs->first.frameId;
      cameraId = itObs->first.cameraIndex;
      Eigen::Vector2d measurement = itObs->second.uv.cast<double>();

      okvis::Time imageTimestamp = frameTimeMap.at(poseId).timestamp(itObs->first.cameraIndex);
      obsList.push_back(measurement);

      double kpSize = itObs->second.size;
      double sigma = kpSize / 8;
      obsStdList.emplace_back(Eigen::Vector2d::Constant(sigma));

      std::shared_ptr<const okvis::ceres::ParameterBlock> parameterBlockPtr = mapPtr->parameterBlockPtr(poseId);
      std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry =
          cameraRig.cameraGeometry(itObs->first.cameraIndex);
      uint32_t imageHeight = cameraGeometry->imageHeight();
      double kpN = measurement[1] / imageHeight - 0.5;
      pointDataPtr->addKeypointObservation(
          itObs->first,
          std::dynamic_pointer_cast<
              const okvis::ceres::PoseParameterBlock>(
              parameterBlockPtr),
          kpN, imageTimestamp);
      ++numNewObservations;
    } else {
      break;
    }
  }

  OKVIS_ASSERT_TRUE_DBG(std::runtime_error,
                        1u <= numNewObservations &&
                            numNewObservations <= cameraRig.numCameras(),
                        "Landmark observations " << numNewObservations
                                                 << " unexpected!");

  EstimationHelper::propagatePoseAndVelocityForMapPoint(statesMap, mapPtr, imuParams, pointDataPtr.get());

  const size_t gravityDimInCov = imuParams->isGravityDirectionVariable() ? 2u : 0u;
  // dimension of variables used in computing feature Jacobians, including
  // gravity direction, camera intrinsics, and all cloned states.
  int featureVariableDim =
      gravityDimInCov +
      minimalDimOfAllCameraParams(statesMap, camNoiseVec.back()) +
      kClonedStateMinimalDim * statesMap.size();

  Eigen::AlignedVector<Eigen::Vector2d> obsStdKeep;
  obsStdKeep.reserve(2);

  pointDataPtr->computePoseAndVelocityForJacobians();
  auto observationIter = pointDataPtr->begin();
  size_t observationIndex = 0u;

  if (homoPointRep.modelId() ==
      swift_vio::InverseDepthParameterization::kModelId) {
    // skip the anchor frame observation.
    ++observationIter;
    ++observationIndex;
  } // else pass
  size_t numObservations = pointDataPtr->numObservations();
  // compute Jacobians for all observations in the current frame.
  Eigen::AlignedVector<Eigen::Matrix<double, 2, Eigen::Dynamic>> vJ_X;
  vJ_X.reserve(numObservations);
  Eigen::AlignedVector<Eigen::Matrix<double, 2, 3>> vJ_pfi;
  Eigen::AlignedVector<Eigen::Matrix<double, 2, 1>> vri;
  vJ_pfi.reserve(numObservations);
  vri.reserve(numObservations);

  for (; observationIndex < numObservations; ++observationIndex) {
    Eigen::Matrix<double, 2, Eigen::Dynamic> J_x(2, featureVariableDim);
    Eigen::Matrix<double, 2, 3> J_pfi;
    Eigen::Vector2d residual;
    // TODO(jhuai): A point in the state vector and expressed in the world frame should
    // use its first estimate in computing Jacobians for consistency.
    // An earlier linearization point is not needed for anchored inverse depth parameterization though.
    bool validJacobian = measurementJacobian(
        homoPointRep, obsList[observationIndex],
        observationIndex, *pointDataPtr, cameraRig, camNoiseVec,
          statesMap, estimatorOptions, imuParams, &J_x, &J_pfi, &residual, FLAGS_mix_projection_jac);
    if (!validJacobian) {
        ++observationIter;
        continue;
    }
    vri.push_back(residual);
    vJ_X.push_back(J_x);
    vJ_pfi.push_back(J_pfi);

    ++observationIter;
    obsStdKeep.emplace_back(obsStdList[observationIndex]);
  }

  size_t numValidObs = obsStdKeep.size();
  H_x.resize(2 * numValidObs, featureVariableDim);
  subH_f.resize(2 * numValidObs, 3);
  r_i.resize(2 * numValidObs, 1);
  R_i = Eigen::MatrixXd::Identity(2 * numValidObs, 2 * numValidObs);
  for (size_t saga = 0; saga < numValidObs; ++saga) {
    size_t saga2 = saga * 2;
    H_x.block(saga2, 0, 2, featureVariableDim) = vJ_X[saga];
    subH_f.block<2, 3>(saga2, 0) = vJ_pfi[saga];
    r_i.segment<2>(saga2) = vri[saga];
    R_i.block<2, 2>(saga2, saga2) = obsStdKeep[saga].cwiseAbs2().asDiagonal();
  }

  vri.clear();
  vJ_pfi.clear();
  vJ_X.clear();
  return numValidObs > 0;
}

void FilterJacobians::computeExtrinsicJacobians(
    const okvis::kinematics::Transformation& T_BCi,
    const okvis::kinematics::Transformation& T_BC0,
    int cameraExtrinsicRepId,
    int mainCameraExtrinsicRepId,
    size_t camiIndex, size_t cam0Index,
    Eigen::AlignedVector<Eigen::MatrixXd>* dT_BCi_dExtrinsics,
    std::vector<size_t>* involvedCameraIndices) {
  dT_BCi_dExtrinsics->reserve(2);
  involvedCameraIndices->clear();
  involvedCameraIndices->reserve(2);
  involvedCameraIndices->push_back(camiIndex);
  switch (cameraExtrinsicRepId) {
    case Extrinsic_p_CB::kModelId: {
      Eigen::Matrix<double, 6, Extrinsic_p_CB::kNumParams> dT_BC_dExtrinsic;
      Extrinsic_p_CB::dT_BC_dExtrinsic(T_BCi, nullptr, &dT_BC_dExtrinsic);
      dT_BCi_dExtrinsics->push_back(dT_BC_dExtrinsic);
    } break;
    case Extrinsic_p_BC_q_BC::kModelId: {
      Eigen::Matrix<double, 6, Extrinsic_p_BC_q_BC::kNumParams>
          dT_BC_dExtrinsic;
      Extrinsic_p_BC_q_BC::dT_BC_dExtrinsic(T_BCi, nullptr, &dT_BC_dExtrinsic);
      dT_BCi_dExtrinsics->push_back(dT_BC_dExtrinsic);
    } break;
    case Extrinsic_p_C0C_q_C0C::kModelId: {
      involvedCameraIndices->push_back(cam0Index);
      Eigen::Matrix<double, 6, Extrinsic_p_C0C_q_C0C::kNumParams> dT_BC_dT_C0Ci;
      Eigen::Matrix<double, 6, Extrinsic_p_C0C_q_C0C::kNumParams> dT_BC_dT_BC0;
      Extrinsic_p_C0C_q_C0C::dT_BC_dExtrinsic(T_BCi, &T_BC0, &dT_BC_dT_C0Ci);
      Extrinsic_p_C0C_q_C0C::dT_BC_dT_BC0(T_BCi, &T_BC0, &dT_BC_dT_BC0);
      dT_BCi_dExtrinsics->push_back(dT_BC_dT_C0Ci);

      switch (mainCameraExtrinsicRepId) {
        case Extrinsic_p_CB::kModelId: {
          Eigen::Matrix<double, 6, Extrinsic_p_CB::kNumParams>
              dT_BC0_dExtrinsic;
          Extrinsic_p_CB::dT_BC_dExtrinsic(T_BC0, nullptr, &dT_BC0_dExtrinsic);
          dT_BCi_dExtrinsics->push_back(dT_BC_dT_BC0 * dT_BC0_dExtrinsic);
        } break;
        case Extrinsic_p_BC_q_BC::kModelId: {
          Eigen::Matrix<double, 6, Extrinsic_p_BC_q_BC::kNumParams>
              dT_BC0_dExtrinsic;
          Extrinsic_p_BC_q_BC::dT_BC_dExtrinsic(T_BC0, nullptr, &dT_BC0_dExtrinsic);
          dT_BCi_dExtrinsics->push_back(dT_BC_dT_BC0 * dT_BC0_dExtrinsic);
        } break;
        default:
          throw std::runtime_error(
              "Unknown extrinsic model type for main camera!");
      }
    } break;
    default:
      throw std::runtime_error("Unknown extrinsic model type for a camera!");
  }
}

void FilterJacobians::changeAnchors(
    const std::vector<uint64_t> &sortedRemovedStateIds,
    const StateMap &statesMap, std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const CameraRig &cameraRig,
    const okvis::CameraNoiseParametersVec &camNoiseVec,
    std::deque<PointLandmark> *orderedLandmarks,
    PointMap *landmarksMap,
    Eigen::MatrixXd *covariance) {
  int covDim = covariance->rows();
  const size_t numNavImuCamStates = startIndexOfClonedStatesFast(statesMap, camNoiseVec.back());
  const size_t numNavImuCamPoseStates =
      numNavImuCamStates + 9 * statesMap.size();
  Eigen::Matrix<double, 3, -1> reparamJacobian(3, covDim);
  Eigen::AlignedVector<Eigen::Matrix<double, 3, -1>>
      reparamJacobianList; // container of these reparameterizing Jacobians.
  reparamJacobianList.reserve(10);
  std::vector<size_t> vCovPtId; // id in covariance of point features to be
                                // reparameterized, 0 for the first landmark.
  vCovPtId.reserve(10);

  int mainExtrinsicRepId =
      cameraRig.getExtrinsicRepId(kMainCameraIndex);
  okvis::kinematics::Transformation T_BC0 =
      cameraRig.getCameraExtrinsic(kMainCameraIndex);

  size_t landmarkIdx = 0;
  for (std::deque<PointLandmark>::iterator landmarkIter = orderedLandmarks->begin();
       landmarkIter != orderedLandmarks->end(); ++landmarkIter, ++landmarkIdx) {
    MapPoint &mapPoint = landmarksMap->at(landmarkIter->id());
    uint64_t newAnchorFrameId =
        mapPoint.shouldChangeAnchor(sortedRemovedStateIds);
    if (newAnchorFrameId) {
      // transform from the body frame at the anchor frame epoch to the world
      // frame.
      okvis::kinematics::Transformation T_WBa;

      EstimationHelper::get_T_WB(mapPtr, mapPoint.anchorStateId, T_WBa);
      okvis::kinematics::Transformation T_BCa =
        cameraRig.getCameraExtrinsic(mapPoint.anchorCameraId);
      okvis::kinematics::Transformation T_WCa = T_WBa * T_BCa;

      // use the camera with the minimum index as the anchor camera.
      int newAnchorCameraId = -1;
      for (auto observationIter = mapPoint.observations.rbegin();
           observationIter != mapPoint.observations.rend(); ++observationIter) {
        if (observationIter->first.frameId == newAnchorFrameId) {
          newAnchorCameraId = observationIter->first.cameraIndex;
        } else {
          break;
        }
      }
      OKVIS_ASSERT_NE(std::runtime_error, newAnchorCameraId, -1,
                      "Anchor image not found!");
      okvis::kinematics::Transformation T_WBj;
      EstimationHelper::get_T_WB(mapPtr, newAnchorFrameId, T_WBj);
      okvis::kinematics::Transformation T_BCj =
        cameraRig.getCameraExtrinsic(newAnchorCameraId);
      okvis::kinematics::Transformation T_WCj = T_WBj * T_BCj;

      OKVIS_ASSERT_EQ(std::runtime_error, landmarkIter->modelId(),
                      swift_vio::InverseDepthParameterization::kModelId,
                      "Only inverse depth parameterization is supposed to be "
                      "reparameterized!");
      Eigen::Vector4d ab1rho = landmarkIter->estimate();
      Eigen::Vector4d rhoxpCj = T_WCj.inverse() * T_WCa * ab1rho;
      landmarkIter->setEstimate(rhoxpCj / rhoxpCj[2]);

      // compute Jacobians.
      Eigen::Vector4d hPointLin = landmarkIter->getLinearizationPoint();
      swift_vio::MultipleTransformPointJacobian mtpj;
      std::vector<int> exponentList{-1, -1, 1, 1};
      Eigen::AlignedVector<okvis::kinematics::Transformation> transformLinList;
      transformLinList.reserve(4);
      transformLinList.push_back(T_BCj);

      okvis::kinematics::Transformation T_WBa_lin;
      EstimationHelper::get_T_WB_lin(mapPtr, mapPoint.anchorStateId, T_WBa_lin);

      okvis::kinematics::Transformation T_WBj_lin;
      EstimationHelper::get_T_WB_lin(mapPtr, newAnchorFrameId, T_WBj_lin);

      transformLinList.emplace_back(T_WBj_lin);
      transformLinList.emplace_back(T_WBa_lin);

      // Empirically this hallucination improves pose NEES.
      hPointLin = (T_WBa_lin * T_BCa).inverse() * (T_WBa * T_BCa) * ab1rho;
      hPointLin = hPointLin / hPointLin[2];

      transformLinList.push_back(T_BCa);

      mtpj.initialize(transformLinList, exponentList, hPointLin);
      Eigen::Vector4d rhoxpCjLin = mtpj.evaluate();

      Eigen::Matrix<double, 3, 4> dnewParams_drhoxpCj;
      double inverseZ = 1.0 / rhoxpCjLin[2];
      double inverseZ2 = inverseZ * inverseZ;
      dnewParams_drhoxpCj << inverseZ, 0, -rhoxpCjLin[0] * inverseZ2, 0,
          0, inverseZ, -rhoxpCjLin[1] * inverseZ2, 0,
          0, 0, -rhoxpCjLin[3] * inverseZ2, inverseZ;

      size_t startRowC = statesMap.at(newAnchorFrameId)
                             .global.at(GlobalStates::T_WS)
                             .startIndexInCov;
      size_t startRowA = statesMap.at(mapPoint.anchorStateId)
                             .global.at(GlobalStates::T_WS)
                             .startIndexInCov;
      std::vector<size_t> camIndices{static_cast<size_t>(newAnchorCameraId),
                                     mapPoint.anchorCameraId};
      std::vector<size_t> mtpjExtrinsicIndices{0u, 3u};
      Eigen::AlignedVector<okvis::kinematics::Transformation> T_BC_list{T_BCj,
                                                                        T_BCa};
      std::vector<size_t> mtpjPoseIndices{1u, 2u};
      std::vector<size_t> startIndices{startRowC, startRowA};

      std::vector<std::pair<size_t, size_t>> startIndexToMinDim;
      Eigen::AlignedVector<Eigen::MatrixXd> dpoint_dX; // drhoxpCtj_dParameters
      mtpj.computeJacobians();
      for (size_t ja = 0; ja < camIndices.size(); ++ja) {
        // Jacobians relative to camera extrinsics.
        if (!camNoiseVec.at(camIndices[ja]).isExtrinsicsFixed()) {
          Eigen::Matrix<double, 4, 6> dpoint_dT_BC =
              mtpj.dp_dT(mtpjExtrinsicIndices[ja]);
          std::vector<size_t> involvedCameraIndices;
          Eigen::AlignedVector<Eigen::MatrixXd> dT_BC_dExtrinsics;
          int extrinsicRepId =
              cameraRig.getExtrinsicRepId(camIndices[ja]);
          computeExtrinsicJacobians(T_BC_list[ja], T_BC0, extrinsicRepId,
                                    mainExtrinsicRepId,
                                    camIndices[ja], kMainCameraIndex,
                                    &dT_BC_dExtrinsics,
                                    &involvedCameraIndices);
          size_t camParamIdx = 0u;
          for (auto idx : involvedCameraIndices) {
            size_t extrinsicStartIndex = startIndexOfCameraParamsFast(statesMap, idx);
            size_t extrinsicDim = cameraRig.getMinimalExtrinsicDim(idx);
            startIndexToMinDim.emplace_back(extrinsicStartIndex, extrinsicDim);
            dpoint_dX.emplace_back(dpoint_dT_BC *
                                   dT_BC_dExtrinsics[camParamIdx]);
            ++camParamIdx;
          }
        }

        // Jacobians relative to nav states
        Eigen::Matrix<double, 4, 6> dpoint_dT_WBt =
            mtpj.dp_dT(mtpjPoseIndices[ja]);
        startIndexToMinDim.emplace_back(startIndices[ja], 6u);
        dpoint_dX.emplace_back(dpoint_dT_WBt);
      }

      // Accumulate Jacobians relative to nav states and camera extrinsics.
      reparamJacobian.setZero();
      size_t iterIndex = 0u;
      for (auto &startAndLen : startIndexToMinDim) {
        reparamJacobian.block(0, startAndLen.first, 3, startAndLen.second) +=
            dnewParams_drhoxpCj.leftCols<3>() *
            dpoint_dX[iterIndex].topRows<3>();
        ++iterIndex;
      }

      // Jacobian relative to landmark parameters.
      size_t covPtId = landmarkIdx;
      vCovPtId.push_back(covPtId);

      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dhp_dparams;
      InverseDepthParameterization::plusJacobian(nullptr, dhp_dparams.data());

      reparamJacobian.block<3, 3>(0, numNavImuCamPoseStates + 3 * covPtId) =
          dnewParams_drhoxpCj * mtpj.dp_dpoint() * dhp_dparams;
      reparamJacobianList.push_back(reparamJacobian);

      mapPoint.anchorStateId = newAnchorFrameId;
      mapPoint.anchorCameraId = newAnchorCameraId;
    }
  }

  // update covariance for reparameterized landmarks.
  if (reparamJacobianList.size()) {
    int landmarkIndex = 0;
    Eigen::MatrixXd featureJacMat = Eigen::MatrixXd::Identity(
        covDim, covDim); // Jacobian of all the new states w.r.t the old states
    for (auto it = reparamJacobianList.begin(); it != reparamJacobianList.end();
         ++it, ++landmarkIndex) {
      featureJacMat.block(numNavImuCamPoseStates + vCovPtId[landmarkIndex] * 3,
                          0, 3, covDim) = reparamJacobianList[landmarkIndex];
    }
    *covariance =
        (featureJacMat * *covariance).eval() * featureJacMat.transpose();
    VLOG(1) << "Reanchored " << reparamJacobianList.size() << " landmarks!";
  }
}
}  // namespace swift_vio
