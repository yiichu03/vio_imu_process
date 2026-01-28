/**
 * @file   EstimationHelper.cpp
 * @brief  EstimationHelper source file.
 * @author Jianzhu Huai
 */

#include "swift_vio/EstimationHelper.h"

#include <swift_vio/ceres/CameraTimeParamBlock.hpp>
#include <swift_vio/ceres/DynamicImuError.hpp>
#include <swift_vio/ceres/EuclideanParamBlock.hpp>
#include <swift_vio/ceres/EuclideanParamBlockSizedLin.hpp>
#include <swift_vio/ceres/EuclideanParamErrorSized.hpp>
#include <swift_vio/ceres/EuclideanParamError.hpp>
#include <swift_vio/ceres/ImuErrorConstBias.hpp>
#include <swift_vio/ceres/NormalVectorParameterBlock.hpp>
#include <swift_vio/ceres/ScalarError.hpp>

#include <swift_vio/FeatureTriangulation.hpp>
#include <swift_vio/imu/ImuOdometry.h>
#include <swift_vio/IoUtil.hpp>
#include <swift_vio/PointSharedData.hpp>

#include <okvis/CameraModelSwitch.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/RelativePoseError.hpp>
#include <okvis/IdProvider.hpp>

namespace swift_vio {
EstimationHelper::EstimationHelper() {}

void EstimationHelper::addNavStateVariables(
    const okvis::kinematics::Transformation &T_WS, const Eigen::Vector3d &v_WS,
    States *latestState, std::shared_ptr<okvis::ceres::Map> mapPtr,
    bool fixLinPoint) {
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock(
      new okvis::ceres::PoseParameterBlock(T_WS, latestState->id,
                                        latestState->timestamp));
  latestState->global.at(GlobalStates::T_WS).exists = true;
  latestState->global.at(GlobalStates::T_WS).id = latestState->id;

  uint64_t vid = okvis::IdProvider::instance().newId();
  std::shared_ptr<okvis::ceres::SpeedParameterBlock>
      velParameterBlock(
          new okvis::ceres::SpeedParameterBlock(v_WS, vid));
  latestState->global.at(GlobalStates::v_WS).exists = true;
  latestState->global.at(GlobalStates::v_WS).id = vid;
  if (fixLinPoint) {
    poseParameterBlock->fixLinPoint(T_WS);
    velParameterBlock->fixLinPoint(v_WS);
  }
  mapPtr->addParameterBlock(poseParameterBlock,
                            okvis::ceres::Map::Parameterization::Pose6dSimple);
  mapPtr->addParameterBlock(velParameterBlock,
                            okvis::ceres::Map::Parameterization::Trivial);
}

void EstimationHelper::addGravityVariable(
    const Eigen::Vector3d &gW, States *latestState,
    std::shared_ptr<okvis::ceres::Map> mapPtr, bool estimate_gravity_direction) {
  uint64_t gid = okvis::IdProvider::instance().newId();
  std::shared_ptr<okvis::ceres::NormalVectorParameterBlock>
      gDirectionParamBlock(
          new okvis::ceres::NormalVectorParameterBlock(gW.normalized(), gid));
  mapPtr->addParameterBlock(gDirectionParamBlock,
                            okvis::ceres::Map::Parameterization::UnitVector);

  latestState->global.at(GlobalStates::GravityDirection).id = gid;
  latestState->global.at(GlobalStates::GravityDirection).exists = true;
  if (estimate_gravity_direction) {
    mapPtr->setParameterBlockVariable(gid);
  } else {
    mapPtr->setParameterBlockConstant(gid);
  }
}

void EstimationHelper::addImuBiasParameters(
    const ImuRig &imuRig, States *latestState,
    std::shared_ptr<okvis::ceres::Map> mapPtr) {
  for (size_t i = 0; i < imuRig.numImus(); ++i) {
    SpecificSensorStatesContainer imuInfo(4);
    imuInfo.at(ImuSensorStates::Bias).exists = true;

    uint64_t id = okvis::IdProvider::instance().newId();
    Eigen::Matrix<double, 6, 1> bgba;
    bgba << imuRig.at(i).getGyroBias(), imuRig.at(i).getAccelBias();
    std::shared_ptr<okvis::ceres::BiasParameterBlock> biasParameterBlock(
        new okvis::ceres::BiasParameterBlock(bgba, id));

    mapPtr->addParameterBlock(biasParameterBlock);
    imuInfo.at(ImuSensorStates::Bias).id = id;
    latestState->sensors.at(SensorStates::Imu).push_back(imuInfo);
  }
}

void EstimationHelper::addImuParameters(
    const ImuRig &imuRig, std::vector<std::shared_ptr<okvis::ImuNoiseParameters>> &noiseVec,
    States *latestState, std::shared_ptr<okvis::ceres::Map> mapPtr) {
  addImuBiasParameters(imuRig, latestState, mapPtr);

  for (size_t i = 0; i < imuRig.numImus(); ++i) {
    SpecificSensorStatesContainer &imuInfo =
        latestState->sensors.at(SensorStates::Imu).at(i);
    int imuModelId = imuRig.getModelId(i);
    Eigen::VectorXd imuAugmentedParams = imuRig.getImuAugmentedParams();
    switch (imuModelId) {
    case Imu_BG_BA_MG_TS_MA::kModelId: {
      Eigen::Matrix<double, 9, 1> MG = imuAugmentedParams.head<9>();
      uint64_t id = okvis::IdProvider::instance().newId();
      std::shared_ptr<okvis::ceres::ShapeMatrixParamBlock> MgBlockPtr(
          new okvis::ceres::ShapeMatrixParamBlock(MG, id));
      mapPtr->addParameterBlock(MgBlockPtr, okvis::ceres::Map::Trivial);
      imuInfo.at(ImuSensorStates::MG).id = id;
      if (noiseVec.at(i)->sigma_Mg_element == 0.0) {
        mapPtr->setParameterBlockConstant(id);
      }

      const Eigen::Matrix<double, 9, 1> TS = imuAugmentedParams.segment<9>(9);
      id = okvis::IdProvider::instance().newId();
      std::shared_ptr<okvis::ceres::ShapeMatrixParamBlock> TsBlockPtr(
          new okvis::ceres::ShapeMatrixParamBlock(TS, id));
      mapPtr->addParameterBlock(TsBlockPtr, okvis::ceres::Map::Trivial);
      imuInfo.at(ImuSensorStates::TS).id = id;
      if (noiseVec.at(i)->sigma_Ts_element == 0.0) {
        mapPtr->setParameterBlockConstant(id);
      }

      Eigen::Matrix<double, 6, 1> MA = imuAugmentedParams.tail<6>();
      id = okvis::IdProvider::instance().newId();
      std::shared_ptr<okvis::ceres::BiasParameterBlock> MaBlockPtr(
          new okvis::ceres::BiasParameterBlock(MA, id));
      mapPtr->addParameterBlock(MaBlockPtr, okvis::ceres::Map::Trivial);
      imuInfo.at(ImuSensorStates::MA).id = id;
      if (noiseVec.at(i)->sigma_Ma_element == 0.0) {
        mapPtr->setParameterBlockConstant(id);
      }

      imuInfo.at(ImuSensorStates::MG).exists = true;
      imuInfo.at(ImuSensorStates::TS).exists = true;
      imuInfo.at(ImuSensorStates::MA).exists = true;
    } break;
    case Imu_BG_BA::kModelId:
      break;
    default:
      LOG(WARNING) << "Adding parameter block not implemented for IMU model "
                   << imuModelId;
      break;
    }
  }
}

void EstimationHelper::addCameraParameters(
    const CameraRig &cameraRig, const okvis::CameraNoiseParametersVec &noiseVec,
    States *latestState, std::shared_ptr<okvis::ceres::Map> mapPtr) {
  for (size_t i = 0; i < cameraRig.numCameras(); ++i) {
    SpecificSensorStatesContainer cameraInfos(5);
    cameraInfos.at(CameraSensorStates::T_XCi).exists = true;
    cameraInfos.at(CameraSensorStates::Intrinsics).exists = true;
    cameraInfos.at(CameraSensorStates::TD).exists = true;
    cameraInfos.at(CameraSensorStates::TR).exists = true;

    const okvis::kinematics::Transformation T_BC =
        cameraRig.getCameraExtrinsic(i);
    const okvis::kinematics::Transformation T_BC0 =
        cameraRig.getCameraExtrinsic(kMainCameraIndex);
    uint64_t id = okvis::IdProvider::instance().newId();
    okvis::Time stateEpoch = latestState->timestamp;
    std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicsParameterBlockPtr;
    switch (cameraRig.getExtrinsicRepId(i)) {
    case Extrinsic_p_CB::kModelId:
    case Extrinsic_p_BC_q_BC::kModelId:
      extrinsicsParameterBlockPtr.reset(
          new okvis::ceres::PoseParameterBlock(T_BC, id, stateEpoch));
      break;
    case Extrinsic_p_C0C_q_C0C::kModelId:
      extrinsicsParameterBlockPtr.reset(
          new okvis::ceres::PoseParameterBlock(T_BC0.inverse() * T_BC, id, stateEpoch));
      break;
    }

    mapPtr->addParameterBlock(extrinsicsParameterBlockPtr,
                              okvis::ceres::Map::Pose6dSimple);
    cameraInfos.at(CameraSensorStates::T_XCi).id = id;
    if (noiseVec.at(i).isExtrinsicsFixed()) {
      mapPtr->setParameterBlockConstant(id);
    }

    Eigen::VectorXd allIntrinsics;
    cameraRig.cameraGeometry(i)->getIntrinsics(allIntrinsics);
    id = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::EuclideanParamBlock>
        intrinsicParamBlockPtr(new okvis::ceres::EuclideanParamBlock(
            allIntrinsics, id, stateEpoch, allIntrinsics.size()));
    mapPtr->addParameterBlock(intrinsicParamBlockPtr,
                              okvis::ceres::Map::Parameterization::Trivial);
    cameraInfos.at(CameraSensorStates::Intrinsics).id = id;
    if (noiseVec.at(i).isIntrinsicsFixed()) {
      mapPtr->setParameterBlockConstant(id);
    }

    id = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::CameraTimeParamBlock> tdParamBlockPtr(
        new okvis::ceres::CameraTimeParamBlock(cameraRig.getImageDelay(i), id,
                                               stateEpoch));
    mapPtr->addParameterBlock(tdParamBlockPtr,
                              okvis::ceres::Map::Parameterization::Trivial);
    cameraInfos.at(CameraSensorStates::TD).id = id;
    if (noiseVec.at(i).sigma_td == 0) {
      mapPtr->setParameterBlockConstant(id);
    }

    id = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::CameraTimeParamBlock> trParamBlockPtr(
        new okvis::ceres::CameraTimeParamBlock(cameraRig.getReadoutTime(i), id,
                                               stateEpoch));
    mapPtr->addParameterBlock(trParamBlockPtr,
                              okvis::ceres::Map::Parameterization::Trivial);
    cameraInfos.at(CameraSensorStates::TR).id = id;
    if (noiseVec.at(i).sigma_tr == 0) {
      mapPtr->setParameterBlockConstant(id);
    }

    latestState->sensors.at(SensorStates::Camera).push_back(cameraInfos);
  }
}

void EstimationHelper::addNavStatePriorFactor(
    const InitialNavState &initialNavState, States *latestState,
    std::shared_ptr<okvis::ceres::Map> mapPtr) {
  uint64_t id = latestState->global.at(GlobalStates::T_WS).id;
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock =
      std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
          mapPtr->parameterBlockPtr(id));
  okvis::kinematics::Transformation T_WS = poseParameterBlock->estimate();

  uint64_t vid = latestState->global.at(GlobalStates::v_WS).id;
  std::shared_ptr<okvis::ceres::SpeedParameterBlock> speedParameterBlock =
      std::static_pointer_cast<okvis::ceres::SpeedParameterBlock>(
          mapPtr->parameterBlockPtr(vid));
  Eigen::Vector3d v_WS = speedParameterBlock->estimate();

  Eigen::Matrix<double, 6, 6> information;
  initialNavState.toInformation(&information);
  std::shared_ptr<okvis::ceres::PoseError> poseError(new okvis::ceres::PoseError(T_WS, information));
  mapPtr->addResidualBlock(poseError, NULL, poseParameterBlock);

  Eigen::Vector3d speedvariance = initialNavState.sigma_v_WS.cwiseAbs2();
  std::shared_ptr<okvis::ceres::SpeedParameterError> speedError(
      new okvis::ceres::SpeedParameterError(v_WS, speedvariance));

  mapPtr->addResidualBlock(speedError, NULL, speedParameterBlock);
}

void EstimationHelper::addCameraParamPriorFactor(
    const okvis::CameraNoiseParametersVec &cameraNoiseParametersList,
    const CameraRig &cameraRig, States *latestState,
    std::shared_ptr<okvis::ceres::Map> mapPtr) {
  for (size_t i = 0; i < cameraNoiseParametersList.size(); ++i) {
    const auto &noiseParams = cameraNoiseParametersList.at(i);
    const auto &camInfo = latestState->sensors.at(SensorStates::Camera).at(i);
    double translationStdev =
        noiseParams.sigma_absolute_translation;
    double translationVariance = translationStdev * translationStdev;
    double rotationStdev =
        noiseParams.sigma_absolute_orientation;
    double rotationVariance = rotationStdev * rotationStdev;

    if (translationVariance > 1.0e-16 && rotationVariance > 1.0e-16) {
      const okvis::kinematics::Transformation T_SC =
          cameraRig.getCameraExtrinsic(i);
      std::shared_ptr<okvis::ceres::PoseError> cameraPoseError(
          new okvis::ceres::PoseError(T_SC, translationVariance, rotationVariance));
      mapPtr->addResidualBlock(cameraPoseError, NULL,
                               mapPtr->parameterBlockPtr(
                                   camInfo.at(CameraSensorStates::T_XCi)
                                       .id));

    } else {
      mapPtr->setParameterBlockConstant(
          latestState->sensors.at(SensorStates::Camera)
              .at(i)
              .at(CameraSensorStates::T_XCi)
              .id);
    }

    if (!noiseParams.isIntrinsicsFixed()) {
      Eigen::VectorXd intrinsics;
      cameraRig.cameraGeometry(i)->getIntrinsics(intrinsics);
      Eigen::MatrixXd covProjIntrinsics = ProjectionIntrinsicRepGetInitCov(
          cameraRig.getProjectionIntrinsicRepId(i),
          noiseParams.sigma_focal_length, noiseParams.sigma_principal_point);

      int distortionDim = cameraRig.getDistortionDim(i);
      Eigen::VectorXd varDistortion(distortionDim);
      for (int jack = 0; jack < distortionDim; ++jack)
        varDistortion[jack] = std::pow(noiseParams.sigma_distortion[jack], 2);
      CHECK_EQ(distortionDim + covProjIntrinsics.rows(), intrinsics.size());
      Eigen::VectorXd intrinsicVariance(intrinsics.size());
      intrinsicVariance.head(covProjIntrinsics.rows()) =
          covProjIntrinsics.diagonal();
      intrinsicVariance.tail(distortionDim) = varDistortion;
      std::shared_ptr<okvis::ceres::EuclideanParamError> intrinsicError(
          new okvis::ceres::EuclideanParamError(intrinsics, intrinsicVariance));
      intrinsicError->setParameterBlockAndResidualSizes();
      mapPtr->addResidualBlock(
          intrinsicError, NULL,
          mapPtr->parameterBlockPtr(
              camInfo.at(CameraSensorStates::Intrinsics).id));
    }
    if (!noiseParams.isTimeDelayFixed()) {
      std::shared_ptr<okvis::ceres::ScalarError> timeError(
          new okvis::ceres::ScalarError(cameraRig.getImageDelay(i),
                                        noiseParams.sigma_td *
                                            noiseParams.sigma_td));
      mapPtr->addResidualBlock(
          timeError, NULL,
          mapPtr->parameterBlockPtr(camInfo.at(CameraSensorStates::TD).id));
    }
    if (!noiseParams.isReadoutTimeFixed()) {
      std::shared_ptr<okvis::ceres::ScalarError> timeError(
          new okvis::ceres::ScalarError(cameraRig.getReadoutTime(i),
                                        noiseParams.sigma_tr *
                                            noiseParams.sigma_tr));
      mapPtr->addResidualBlock(
          timeError, NULL,
          mapPtr->parameterBlockPtr(camInfo.at(CameraSensorStates::TR).id));
    }
  }
}

void EstimationHelper::addImuPriorFactor(
    const std::vector<std::shared_ptr<okvis::ImuNoiseParameters>>
        &imuParametersVec,
    States *latestState, std::shared_ptr<okvis::ceres::Map> mapPtr) {
  for (size_t i = 0; i < imuParametersVec.size(); ++i) {
    const auto &noiseParams = imuParametersVec.at(i);
    const auto &imuInfo = latestState->sensors.at(SensorStates::Imu).at(i);
    Eigen::Matrix<double, 6, 1> variances;
    const double sigma_bg = noiseParams->sigma_bg;
    const double sigma_ba = noiseParams->sigma_ba;

    uint64_t bid = imuInfo.at(ImuSensorStates::Bias).id;
    std::shared_ptr<okvis::ceres::BiasParameterBlock> biasParameterBlock =
        std::static_pointer_cast<okvis::ceres::BiasParameterBlock>(
            mapPtr->parameterBlockPtr(bid));
    Eigen::Matrix<double, 6, 1> bias = biasParameterBlock->estimate();
    Eigen::Matrix<double, 6, 1> biasVariance;
    double bgVar = sigma_bg * sigma_bg;
    double baVar = sigma_ba * sigma_ba;
    biasVariance.head<3>() = Eigen::Vector3d(bgVar, bgVar, bgVar);
    biasVariance.tail<3>() = Eigen::Vector3d(baVar, baVar, baVar);

    std::shared_ptr<okvis::ceres::BiasParameterError> biasError(
        new okvis::ceres::BiasParameterError(bias, biasVariance));
    mapPtr->addResidualBlock(biasError, NULL, biasParameterBlock);

    if (noiseParams->sigma_Mg_element > 0.0) {
      auto blockPtr =
          mapPtr->parameterBlockPtr(imuInfo.at(ImuSensorStates::MG).id);
      std::shared_ptr<okvis::ceres::ShapeMatrixParamBlock> MgPtr =
          std::static_pointer_cast<okvis::ceres::ShapeMatrixParamBlock>(
              blockPtr);
      std::shared_ptr<okvis::ceres::EuclideanParamErrorSized<9>> MgError(
          new okvis::ceres::EuclideanParamErrorSized<9>(
              MgPtr->estimate(),
              noiseParams->sigma_Mg_element * noiseParams->sigma_Mg_element));
      mapPtr->addResidualBlock(MgError, NULL, blockPtr);
    }
    if (noiseParams->sigma_Ts_element > 0.0) {
      auto blockPtr =
          mapPtr->parameterBlockPtr(imuInfo.at(ImuSensorStates::TS).id);
      std::shared_ptr<okvis::ceres::ShapeMatrixParamBlock> TsPtr =
          std::static_pointer_cast<okvis::ceres::ShapeMatrixParamBlock>(
              blockPtr);
      std::shared_ptr<okvis::ceres::EuclideanParamErrorSized<9>> TsError(
          new okvis::ceres::EuclideanParamErrorSized<9>(
              TsPtr->estimate(),
              noiseParams->sigma_Ts_element * noiseParams->sigma_Ts_element));
      mapPtr->addResidualBlock(TsError, NULL, blockPtr);
    }
    if (noiseParams->sigma_Ma_element > 0.0) {
      auto blockPtr =
          mapPtr->parameterBlockPtr(imuInfo.at(ImuSensorStates::MA).id);
      std::shared_ptr<okvis::ceres::EuclideanParamBlockSized<6>> MaPtr =
          std::static_pointer_cast<okvis::ceres::EuclideanParamBlockSized<6>>(
              blockPtr);
      std::shared_ptr<okvis::ceres::EuclideanParamErrorSized<6>> MaError(
          new okvis::ceres::EuclideanParamErrorSized<6>(
              MaPtr->estimate(),
              noiseParams->sigma_Ma_element * noiseParams->sigma_Ma_element));
      mapPtr->addResidualBlock(MaError, NULL, blockPtr);
    }
  }
}

void EstimationHelper::usePreviousGravity(const States &previousState,
                                          States *currentState) {
  currentState->global.at(GlobalStates::GravityDirection) =
      previousState.global.at(GlobalStates::GravityDirection);
}

void EstimationHelper::usePreviousImuBias(const States &previousState,
                                          States *currentState) {
  for (size_t i = 0; i < previousState.sensors.at(SensorStates::Imu).size();
       ++i) {
    SpecificSensorStatesContainer imuInfo(4);
    imuInfo.at(ImuSensorStates::Bias) =
        previousState.sensors.at(SensorStates::Imu)
            .at(i)
            .at(ImuSensorStates::Bias);
    currentState->sensors.at(SensorStates::Imu).push_back(imuInfo);
  }
}

void EstimationHelper::usePreviousImuAugmentedParameters(
    const States &previousState, States *currentState) {
  for (size_t i = 0; i < previousState.sensors.at(SensorStates::Imu).size();
       ++i) {
    const SpecificSensorStatesContainer &prevImuInfo =
        previousState.sensors.at(SensorStates::Imu).at(i);
    SpecificSensorStatesContainer &imuInfo =
        currentState->sensors.at(SensorStates::Imu).at(i);
    for (size_t j = ImuSensorStates::Bias + 1; j < prevImuInfo.size(); ++j) {
      imuInfo.at(j) = prevImuInfo.at(j);
    }
  }
}

void EstimationHelper::usePreviousCameraParameters(const States &previousState,
                                                   States *currentState) {
  currentState->sensors.at(SensorStates::Camera) =
      previousState.sensors.at(SensorStates::Camera);
}

void EstimationHelper::setImuWindow(
    const swift_vio::BoundedImuDeque &inertialMeasForStates, States *oneState,
    okvis::Duration halfBuffer) {
  auto imuMeasCover =
      inertialMeasForStates.findWindow(oneState->timestamp, halfBuffer);
  oneState->imuReadingWindow.reset(
      new okvis::ImuMeasurementDeque(imuMeasCover));
}

int EstimationHelper::predictNavState(
    const okvis::ImuMeasurementDeque &imuMeasurements,
    const States &previousState,
    std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const okvis::ImuParameters &imuParameters, okvis::Time currentStateTime,
    okvis::kinematics::Transformation *T_WS, Eigen::Vector3d *v_WS) {
  // imu propagation to predict the current frame pose from the last frame.

  // get the previous states
  okvis::Time lastStateTime = previousState.timestamp;
  uint64_t T_WS_id = previousState.id;
  uint64_t speedId = previousState.global.at(GlobalStates::v_WS).id;
  uint64_t biasId = previousState.sensors.at(SensorStates::Imu)
                        .at(0)
                        .at(ImuSensorStates::Bias)
                        .id;

  OKVIS_ASSERT_TRUE_DBG(std::runtime_error,
                        mapPtr->parameterBlockExists(T_WS_id),
                        "this is an okvis bug. previous pose does not exist.");

  *T_WS = std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
              mapPtr->parameterBlockPtr(T_WS_id))
              ->estimate();

  *v_WS =
      std::static_pointer_cast<const okvis::ceres::SpeedParameterBlock>(
          mapPtr->parameterBlockPtr(speedId))
          ->estimate();

  Eigen::Matrix<double, 6, 1> bgba =
      std::static_pointer_cast<const okvis::ceres::BiasParameterBlock>(
          mapPtr->parameterBlockPtr(biasId))
          ->estimate();

  // propagate pose and speedAndBias
  Eigen::Matrix<double, 9, 1> speedAndBias;
  speedAndBias.head<3>() = *v_WS;
  speedAndBias.tail<6>() = bgba;

  int numUsedImuMeasurements = okvis::ceres::ImuError::propagation(
      imuMeasurements, imuParameters, *T_WS, speedAndBias, lastStateTime,
      currentStateTime);
  *v_WS = speedAndBias.head<3>();
  if (numUsedImuMeasurements < 1) {
    LOG(INFO) << "predictNavState used IMU measurements " << numUsedImuMeasurements << ".";
  }
  return numUsedImuMeasurements;
}

int EstimationHelper::propagateNavStateAndCov(
    const okvis::ImuMeasurementDeque &imuMeasurements,
    const States &previousState,
    std::shared_ptr<const okvis::ceres::Map> mapPtr,
    okvis::ImuParameters imuParameters, okvis::Time currentStateTime,
    okvis::kinematics::Transformation *T_WS, Eigen::Vector3d *v_WS,
    Eigen::MatrixXd *covariance) {
  okvis::Time startTime = previousState.timestamp;
  size_t navAndImuParamsDim = previousState.sensors.at(SensorStates::Camera).at(0).at(CameraSensorStates::T_XCi).startIndexInCov;
  Eigen::MatrixXd Pkm1 =
      covariance->topLeftCorner(navAndImuParamsDim, navAndImuParamsDim);
  Eigen::MatrixXd F_tot = Eigen::MatrixXd::Identity(navAndImuParamsDim, navAndImuParamsDim);
  okvis::Time latestImuEpoch = imuMeasurements.back().timeStamp;
  okvis::Time propagationTargetTime = currentStateTime;
  if (latestImuEpoch < currentStateTime) {
    propagationTargetTime = latestImuEpoch;
    LOG(WARNING) << "Latest IMU readings does not extend to corrected state "
                    "time. Is temporal_imu_data_overlap too small?";
  }

  std::vector<const double *> imuAugmentedParamPtrs;
  std::vector<size_t> imuAugmentedParamDims;
  getImuAugmentedParameterPtrs(previousState, mapPtr, imuParameters.imuIdx, &imuAugmentedParamPtrs, &imuAugmentedParamDims);
  int imuModelId = ImuModelNameToId(imuParameters.model_name);
  Eigen::VectorXd imuAugmentedParams;
  swift_vio::getImuAugmentedStatesEstimate(
      imuAugmentedParamPtrs, imuAugmentedParamDims, &imuAugmentedParams, imuModelId);

  uint64_t bid = previousState.sensors.at(SensorStates::Imu).at(imuParameters.imuIdx).at(ImuSensorStates::Bias).id;

  uint64_t T_WS_id = previousState.id;
  uint64_t speedId = previousState.global.at(GlobalStates::v_WS).id;
  std::shared_ptr<const okvis::ceres::PoseParameterBlock> posePtr = std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
        mapPtr->parameterBlockPtr(T_WS_id));
  *T_WS = posePtr->estimate();
  std::shared_ptr<const okvis::ceres::SpeedParameterBlock> speedPtr =
      std::static_pointer_cast<const okvis::ceres::SpeedParameterBlock>(
          mapPtr->parameterBlockPtr(speedId));
  *v_WS = speedPtr->estimate();
  Eigen::Matrix<double, 6, 1> posVelLinPoint;
  posVelLinPoint.head<3>() = posePtr->positionLinPoint();
  posVelLinPoint.tail<3>() = speedPtr->linPoint();

  int numUsedImuMeasurements = -1;
  if (imuModelId == Imu_BG_BA_MG_TS_MA::kModelId) {
    Imu_BG_BA_MG_TS_MA iem;
    iem.updateParameters(mapPtr->parameterBlockPtr(bid)->parameters(),
                         imuAugmentedParams.data());
    numUsedImuMeasurements = ImuOdometry::propagation(
        imuMeasurements, imuParameters, *T_WS, *v_WS, iem, startTime,
        propagationTargetTime, &Pkm1, &F_tot,
        &posVelLinPoint);
  } else {
    Imu_BG_BA_TG_TS_TA iem;
    iem.updateParameters(mapPtr->parameterBlockPtr(bid)->parameters());
    numUsedImuMeasurements = ImuOdometry::propagation(
        imuMeasurements, imuParameters, *T_WS, *v_WS, iem, startTime,
        propagationTargetTime, &Pkm1, &F_tot,
        &posVelLinPoint);
  }

  if (numUsedImuMeasurements < 2) {
    LOG(WARNING) << "numUsedImuMeasurements " << numUsedImuMeasurements
                 << " is too small. currentStateTime " << currentStateTime
                 << " lastFrameTimestamp " << startTime << std::endl;
  }

  int covDim = covariance->rows();
  covariance->topLeftCorner(navAndImuParamsDim, navAndImuParamsDim) = Pkm1;
  covariance->block(0, navAndImuParamsDim, navAndImuParamsDim,
                    covDim - navAndImuParamsDim) =
      F_tot * covariance->block(0, navAndImuParamsDim, navAndImuParamsDim,
                                covDim - navAndImuParamsDim);
  covariance->
      block(navAndImuParamsDim, 0, covDim - navAndImuParamsDim,
             navAndImuParamsDim)
      .noalias() = covariance->
                       block(0, navAndImuParamsDim, navAndImuParamsDim,
                              covDim - navAndImuParamsDim)
                       .transpose();
  return numUsedImuMeasurements;
}

void EstimationHelper::addImuObservations(
    const okvis::ImuMeasurementDeque &imuMeasurements, size_t imuId,
    int imuModelId, const okvis::ImuParameters &imuParameters,
    const States &lastState, const States &currentState,
    std::shared_ptr<okvis::ceres::Map> mapPtr) {
  switch (imuModelId) {
  case Imu_BG_BA_MG_TS_MA::kModelId: {
    typedef okvis::ceres::DynamicImuError<Imu_BG_BA_MG_TS_MA> DynamicImuErrorT;
    std::shared_ptr<DynamicImuErrorT> imuError(
        new DynamicImuErrorT(imuMeasurements, imuParameters,
                             lastState.timestamp, currentState.timestamp));
    imuError->setParameterBlockAndResidualSizes();

    mapPtr->addResidualBlock(
        imuError, NULL, mapPtr->parameterBlockPtr(lastState.id),
        mapPtr->parameterBlockPtr(lastState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(lastState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::Bias)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.id),
        mapPtr->parameterBlockPtr(
            currentState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                        .at(imuId)
                                        .at(ImuSensorStates::Bias)
                                        .id),
        mapPtr->parameterBlockPtr(
            lastState.global.at(GlobalStates::GravityDirection).id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::MG)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::TS)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::MA)
                                      .id));
    break;
  }

  case Imu_BG_BA::kModelId:
  default: {
    typedef okvis::ceres::DynamicImuError<Imu_BG_BA> DynamicImuErrorT;
    std::shared_ptr<DynamicImuErrorT> imuError(
        new DynamicImuErrorT(imuMeasurements, imuParameters,
                             lastState.timestamp, currentState.timestamp));
    imuError->setParameterBlockAndResidualSizes();

    mapPtr->addResidualBlock(
        imuError, NULL, mapPtr->parameterBlockPtr(lastState.id),
        mapPtr->parameterBlockPtr(lastState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(lastState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::Bias)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.id),
        mapPtr->parameterBlockPtr(
            currentState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                        .at(imuId)
                                        .at(ImuSensorStates::Bias)
                                        .id),
        mapPtr->parameterBlockPtr(
            lastState.global.at(GlobalStates::GravityDirection).id));
    break;
  }
  }
}

void EstimationHelper::addImuObservationsConstBias(
    const okvis::ImuMeasurementDeque &imuMeasurements, size_t imuId,
    int imuModelId, const okvis::ImuParameters &imuParameters,
    const States &lastState, const States &currentState,
    std::shared_ptr<okvis::ceres::Map> mapPtr) {
  switch (imuModelId) {
  case Imu_BG_BA_MG_TS_MA::kModelId: {
    typedef okvis::ceres::ImuErrorConstBias<Imu_BG_BA_MG_TS_MA> DynamicImuErrorT;
    std::shared_ptr<DynamicImuErrorT> imuError(
        new DynamicImuErrorT(imuMeasurements, imuParameters,
                             lastState.timestamp, currentState.timestamp));
    imuError->setParameterBlockAndResidualSizes();

    mapPtr->addResidualBlock(
        imuError, NULL, mapPtr->parameterBlockPtr(lastState.id),
        mapPtr->parameterBlockPtr(lastState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(lastState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::Bias)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.id),
        mapPtr->parameterBlockPtr(
            currentState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(
            lastState.global.at(GlobalStates::GravityDirection).id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::MG)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::TS)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::MA)
                                      .id));
    break;
  }

  case Imu_BG_BA::kModelId:
  default: {
    typedef okvis::ceres::ImuErrorConstBias<Imu_BG_BA> DynamicImuErrorT;
    std::shared_ptr<DynamicImuErrorT> imuError(
        new DynamicImuErrorT(imuMeasurements, imuParameters,
                             lastState.timestamp, currentState.timestamp));
    imuError->setParameterBlockAndResidualSizes();

    mapPtr->addResidualBlock(
        imuError, NULL, mapPtr->parameterBlockPtr(lastState.id),
        mapPtr->parameterBlockPtr(lastState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(lastState.sensors.at(SensorStates::Imu)
                                      .at(imuId)
                                      .at(ImuSensorStates::Bias)
                                      .id),
        mapPtr->parameterBlockPtr(currentState.id),
        mapPtr->parameterBlockPtr(
            currentState.global.at(GlobalStates::v_WS).id),
        mapPtr->parameterBlockPtr(
            lastState.global.at(GlobalStates::GravityDirection).id));
    break;
  }
  }
}

void EstimationHelper::addCameraParameterConstraints(
    const okvis::CameraNoiseParametersVec &cameraNoiseParametersList,
    const States &lastState, const States &currentState,
    std::shared_ptr<okvis::ceres::Map> mapPtr) {
  for (size_t i = 0; i < cameraNoiseParametersList.size(); ++i) {
    const auto & noiseParams = cameraNoiseParametersList.at(i);
    if (lastState.sensors.at(SensorStates::Camera)
            .at(i)
            .at(CameraSensorStates::T_XCi)
            .id != currentState.sensors.at(SensorStates::Camera)
                       .at(i)
                       .at(CameraSensorStates::T_XCi)
                       .id) {
      double dt = (currentState.timestamp - lastState.timestamp).toSec();
      double translationSigmaC =
          noiseParams.sigma_c_relative_translation;
      double translationVariance = translationSigmaC * translationSigmaC * dt;
      double rotationSigmaC =
          noiseParams.sigma_c_relative_orientation;
      double rotationVariance = rotationSigmaC * rotationSigmaC * dt;
      std::shared_ptr<okvis::ceres::RelativePoseError> relativeExtrinsicsError(
          new okvis::ceres::RelativePoseError(translationVariance, rotationVariance));
      mapPtr->addResidualBlock(
          relativeExtrinsicsError, NULL,
          mapPtr->parameterBlockPtr(lastState.sensors.at(SensorStates::Camera)
                                        .at(i)
                                        .at(CameraSensorStates::T_XCi)
                                        .id),
          mapPtr->parameterBlockPtr(
              currentState.sensors.at(SensorStates::Camera)
                  .at(i)
                  .at(CameraSensorStates::T_XCi)
                  .id));
    }
  }
}

void EstimationHelper::updateFeatureTracks(const FeatureTrackMap &newObservations,
                        std::shared_ptr<const MultiFrame> currentNFrame,
                        PointMap *trackMap,
                        std::unordered_map<uint64_t, BareMultiFrame> *nframe2LandmarkIds) {
  auto res = nframe2LandmarkIds->emplace(
        std::piecewise_construct, std::forward_as_tuple(currentNFrame->id()),
        std::forward_as_tuple(currentNFrame->timestamp(), currentNFrame->id(),
                              currentNFrame->numFrames()));
  for (size_t i = 0; i < currentNFrame->numFrames(); ++i) {
    res.first->second.setTimestamp(i, currentNFrame->timestamp(i));
    res.first->second.landmarkIdsMutable(i).resize(currentNFrame->numKeypoints(i), 0u);
  }
  for (const auto &track : newObservations) {
    PointMap::iterator iter = trackMap->find(track.first);
    if (iter == trackMap->end()) {
      auto res = trackMap->emplace(std::piecewise_construct,
                                   std::forward_as_tuple(track.first),
                                   std::forward_as_tuple(track.first));
      iter = res.first;
    }

    for (const auto &obs : track.second.observations) {
      auto nframe = nframe2LandmarkIds->find(obs.first.frameId);
      if (nframe != nframe2LandmarkIds->end()) {
        nframe->second.setLandmarkId(obs.first.cameraIndex, obs.first.keypointIndex, track.first);
        iter->second.addObservation(obs.first, obs.second);
      } else {
        // There are two causes:
        // 1. The estimator only uses observations from a subset of frames as
        // recorded in nframe2LandmarkIds.
        // 2. The frontend matches the current frame to many earlier keyframes.
        VLOG(1) << "Discard a keypoint in frame " << obs.first.frameId << ".";
      }
    }
  }
}

void EstimationHelper::computeKeypointOpticFlow(
    const PointMap &landmarksMap,
    const std::map<uint64_t, std::vector<okvis::Time>> &nframeMap, size_t camId,
    std::vector<std::pair<okvis::Duration, float>> *framewiseDisparity) {
  Eigen::AlignedVector<Eigen::Vector2f> accumUV(nframeMap.size() - 1,
                                                Eigen::Vector2f(0.f, 0.f));
  std::vector<int> accumCount(nframeMap.size() - 1, 0);

  for (PointMap::const_iterator it = landmarksMap.begin();
       it != landmarksMap.end(); ++it) {
    const Eigen::AlignedMap<okvis::KeypointIdentifier, KeypointObservation> &matchList =
        it->second.observations;

    Eigen::AlignedMap<okvis::KeypointIdentifier, KeypointObservation>::const_iterator lastMatchIt =
        matchList.end();
    Eigen::Vector2f lastObs;
    size_t lastStateOrder = 0u;

    for (Eigen::AlignedMap<okvis::KeypointIdentifier, KeypointObservation>::const_iterator matchIt =
             matchList.begin(); matchIt != matchList.end(); ++matchIt) {
      if (matchIt->first.cameraIndex != camId) {
        continue;
      }
      if (lastMatchIt == matchList.end()) {
        lastMatchIt = matchIt;
        lastObs = lastMatchIt->second.uv;
        lastStateOrder = std::distance(
            nframeMap.begin(), nframeMap.find(lastMatchIt->first.frameId));
      } else {
        Eigen::Vector2f currentObs = matchIt->second.uv;
        Eigen::Vector2f dist = (currentObs - lastObs).cwiseAbs();
        size_t stateOrder = std::distance(
            nframeMap.begin(), nframeMap.find(matchIt->first.frameId));
        Eigen::Vector2f ration = dist / (stateOrder - lastStateOrder);
        for (size_t sid = lastStateOrder; sid < stateOrder; ++sid) {
          accumUV[sid] += ration;
          ++accumCount[sid];
        }
        lastObs = currentObs;
        lastStateOrder = stateOrder;
      }
    }
  }

  framewiseDisparity->resize(nframeMap.size() - 1);
  auto it = ++nframeMap.begin();
  okvis::Time lastStateTime = nframeMap.begin()->second.at(camId);
  for (size_t i = 0u; i < framewiseDisparity->size(); ++i, ++it) {
    Eigen::Vector2f mean(10000.f, 10000.f);
    if (accumCount[i])
      mean = accumUV[i] / accumCount[i];

    framewiseDisparity->at(i).first = it->second.at(camId) - lastStateTime;
    framewiseDisparity->at(i).second = mean[0] + mean[1];
    lastStateTime = it->second.at(camId);
  }
}

void EstimationHelper::createMinimalNFrameMap(
    const std::unordered_map<uint64_t, BareMultiFrame> &nframe2LandmarkIds,
    std::map<uint64_t, std::vector<okvis::Time>> *miniNFrameMap) {
  miniNFrameMap->clear();
  for (auto it = nframe2LandmarkIds.begin(); it != nframe2LandmarkIds.end();
       ++it) {
    std::vector<okvis::Time> frameTimes;
    frameTimes.reserve(it->second.numFrames());
    for (size_t i = 0; i < it->second.numFrames(); ++i) {
      frameTimes.emplace_back(it->second.timestamp(i));
    }
    miniNFrameMap->emplace(std::piecewise_construct,
                           std::forward_as_tuple(it->first),
                           std::forward_as_tuple(frameTimes));
  }
}

size_t EstimationHelper::removeOldFrames(
    uint64_t oldestNFrameId,
    std::unordered_map<uint64_t, BareMultiFrame> *nframeToLandmarkIds) {
  size_t removedFrames = 0;
  for (auto it = nframeToLandmarkIds->begin(); it != nframeToLandmarkIds->end();) {
    if (it->first < oldestNFrameId) {
      it = nframeToLandmarkIds->erase(it);
      ++removedFrames;
    } else {
      ++it;
    }
  }
  return removedFrames;
}

size_t EstimationHelper::removeOldObservations(uint64_t oldestNFrameId,
                                               PointMap *landmarksMap) {
  size_t removedObservations = 0;
  for (auto it = landmarksMap->begin(); it != landmarksMap->end();) {
    for (auto oit = it->second.observations.begin();
         oit != it->second.observations.end();) {
      if (oit->first.frameId < oldestNFrameId) {
        oit = it->second.observations.erase(oit);
        ++removedObservations;
      } else {
        ++oit;
      }
    }
    if (it->second.observations.size() == 0) {
      it = landmarksMap->erase(it);
    } else {
      ++it;
    }
  }
  return removedObservations;
}

size_t EstimationHelper::gatherMapPointObservationsRs(
    const MapPoint &mp,
    std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const BareMultiFrameMap &frameTimeMap,
    const CameraRig &cameraRig,
    swift_vio::PointSharedData *pointDataPtr,
    Eigen::AlignedVector<Eigen::Vector3d> *obsDirections,
    Eigen::AlignedVector<Eigen::Vector2d> *obsList,
    Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
    std::vector<std::pair<uint64_t, int>> *orderedBadFrameIds) {
  obsDirections->clear();
  obsList->clear();
  obsStdList->clear();
  const ObservationMap &observations = mp.observations;
  obsDirections->reserve(observations.size());
  obsList->reserve(observations.size());
  obsStdList->reserve(observations.size());

  for (auto itObs = observations.begin(), iteObs = observations.end();
       itObs != iteObs; ++itObs) {
    uint64_t poseId = itObs->first.frameId;
    Eigen::Vector2d measurement = itObs->second.uv.cast<double>();
    auto multiFrameIter = frameTimeMap.find(poseId);
    //    OKVIS_ASSERT_TRUE(Exception, multiFrameIter !=
    //    multiFramePtrMap_.end(), "multiframe not found");
    const BareMultiFrame &multiframe = multiFrameIter->second;

    okvis::Time imageTimestamp = multiframe.timestamp(itObs->first.cameraIndex);
    Eigen::Vector3d backProjectionDirection;
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry =
        cameraRig.cameraGeometry(itObs->first.cameraIndex);
    bool validDirection =
        cameraGeometry->backProject(measurement, &backProjectionDirection);
    if (!validDirection) {
      orderedBadFrameIds->emplace_back(poseId, itObs->first.cameraIndex);
      continue;
    }
    obsDirections->push_back(backProjectionDirection);
    obsList->push_back(measurement);

    double kpSize = itObs->second.size;
    double sigma = kpSize / 8;
    obsStdList->emplace_back(sigma, sigma);

    std::shared_ptr<const okvis::ceres::PoseParameterBlock> parameterBlockPtr =
        std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(mapPtr->parameterBlockPtr(poseId));

    uint32_t imageHeight = cameraGeometry->imageHeight();
    double kpN = measurement[1] / imageHeight - 0.5;
    pointDataPtr->addKeypointObservation(itObs->first, parameterBlockPtr, kpN,
                                         imageTimestamp);
  }
  return pointDataPtr->numObservations();
}

size_t EstimationHelper::gatherMapPointObservations(
    const MapPoint &mp,
    std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const CameraRig &cameraRig,
    swift_vio::PointSharedData *pointDataPtr,
    Eigen::AlignedVector<Eigen::Vector3d> *obsDirections,
    Eigen::AlignedVector<Eigen::Vector2d> *obsList,
    Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
    std::vector<std::pair<uint64_t, int>> *orderedBadFrameIds) {
  obsDirections->clear();
  obsList->clear();
  obsStdList->clear();
  const ObservationMap &observations = mp.observations;
  obsDirections->reserve(observations.size());
  obsList->reserve(observations.size());
  obsStdList->reserve(observations.size());

  for (auto itObs = observations.begin(), iteObs = observations.end();
       itObs != iteObs; ++itObs) {
    uint64_t poseId = itObs->first.frameId;
    Eigen::Vector2d measurement = itObs->second.uv.cast<double>();

    // use the latest estimates for camera intrinsic parameters
    Eigen::Vector3d backProjectionDirection;
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry =
        cameraRig.cameraGeometry(itObs->first.cameraIndex);
    bool validDirection =
        cameraGeometry->backProject(measurement, &backProjectionDirection);
    if (!validDirection) {
      orderedBadFrameIds->emplace_back(poseId, itObs->first.cameraIndex);
      continue;
    }
    obsDirections->push_back(backProjectionDirection);
    obsList->push_back(measurement);

    double kpSize = itObs->second.size;
    double sigma = kpSize / 8;
    obsStdList->emplace_back(sigma, sigma);

    std::shared_ptr<const okvis::ceres::PoseParameterBlock> parameterBlockPtr =
        std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(mapPtr->parameterBlockPtr(poseId));

    uint32_t imageHeight = cameraGeometry->imageHeight();
    double kpN = measurement[1] / imageHeight - 0.5;
    pointDataPtr->addKeypointObservation(itObs->first, parameterBlockPtr, kpN,
                                         okvis::Time());
  }
  return pointDataPtr->numObservations();
}

void EstimationHelper::getCameraTimeParameterPtrs(
    const States &currentState,
    std::shared_ptr<const okvis::ceres::Map> mapPtr,
    std::vector<const double *> *cameraDelayParameterPtrs,
    std::vector<const double *> *cameraReadoutTimeParameterPtrs) {
  size_t numCameras = currentState.sensors.at(SensorStates::Camera).size();
  cameraDelayParameterPtrs->reserve(numCameras);
  cameraReadoutTimeParameterPtrs->reserve(numCameras);

  for (size_t camIdx = 0u; camIdx < numCameras; ++camIdx) {
    const SpecificSensorStatesContainer & camInfo = currentState.sensors.at(SensorStates::Camera)
        .at(camIdx);
    uint64_t tdId = camInfo.at(CameraSensorStates::TD).id;
    std::shared_ptr<const okvis::ceres::ParameterBlock> tdParamBlockPtr =
        mapPtr->parameterBlockPtr(tdId);
    uint64_t trId = camInfo.at(CameraSensorStates::TR).id;
    std::shared_ptr<const okvis::ceres::ParameterBlock> trParamBlockPtr =
        mapPtr->parameterBlockPtr(trId);
    cameraDelayParameterPtrs->push_back(tdParamBlockPtr->parameters());
    cameraReadoutTimeParameterPtrs->push_back(trParamBlockPtr->parameters());
  }
}

void EstimationHelper::getImuAugmentedParameterPtrs(
    const States &currentState, std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const size_t imuIdx, std::vector<const double *> *imuAugmentedParamPtrs,
    std::vector<size_t> *imuAugmentedParamDims) {
  const SpecificSensorStatesContainer &oneImu =
      currentState.sensors.at(SensorStates::Imu).at(imuIdx);
  imuAugmentedParamPtrs->reserve(3);
  imuAugmentedParamDims->reserve(3);
  for (size_t i = ImuSensorStates::Bias + 1; i < oneImu.size(); ++i) {
    if (oneImu.at(i).exists) {
      const auto paramBlock = mapPtr->parameterBlockPtr(oneImu.at(i).id);
      imuAugmentedParamPtrs->push_back(paramBlock->parameters());
      imuAugmentedParamDims->push_back(paramBlock->dimension());
    }
  }
}

void EstimationHelper::propagatePoseAndVelocityForMapPoint(
    const StateMap &statesMap, std::shared_ptr<const okvis::ceres::Map> mapPtr,
    std::shared_ptr<const okvis::ImuParameters> &imuParams,
    swift_vio::PointSharedData* pointDataPtr) {
  std::vector<std::pair<uint64_t, size_t>> frameIds = pointDataPtr->frameIds();
  int observationIndex = 0;
  for (const std::pair<uint64_t, size_t>& frameAndCameraIndex : frameIds) {
    uint64_t frameId = frameAndCameraIndex.first;
    auto statesIter = statesMap.find(frameId);
    pointDataPtr->setMotionInfo(observationIndex, statesIter->second.timestamp,
                                statesIter->second.imuReadingWindow,
                                statesIter->second.isKeyframe);

    const size_t imuIdx = imuParams->imuIdx;
    uint64_t vid = statesMap.at(frameId).global.at(GlobalStates::v_WS).id;
    uint64_t bid = statesMap.at(frameId).sensors.at(SensorStates::Imu).at(imuIdx).at(ImuSensorStates::Bias).id;
    std::shared_ptr<const okvis::ceres::SpeedParameterBlock> speedParameterBlockPtr =
        std::static_pointer_cast<const okvis::ceres::SpeedParameterBlock>(mapPtr->parameterBlockPtr(vid));
    std::shared_ptr<const okvis::ceres::BiasParameterBlock> biasParameterBlockPtr =
        std::static_pointer_cast<const okvis::ceres::BiasParameterBlock>(mapPtr->parameterBlockPtr(bid));

    pointDataPtr->setVelocityAndBiasParameterBlockPtr(observationIndex,
                                               speedParameterBlockPtr, biasParameterBlockPtr);
    ++observationIndex;
  }

  std::vector<const double *> cameraDelayParameterPtrs;
  std::vector<const double *> cameraReadoutTimeParameterPtrs;
  getCameraTimeParameterPtrs(statesMap.rbegin()->second, mapPtr, &cameraDelayParameterPtrs,
                             &cameraReadoutTimeParameterPtrs);
  pointDataPtr->setCameraTimeParameterPtrs(cameraDelayParameterPtrs,
                                           cameraReadoutTimeParameterPtrs);

  std::vector<const double *> imuAugmentedParamPtrs;
  std::vector<size_t> imuAugmentedParamDims;
  getImuAugmentedParameterPtrs(statesMap.rbegin()->second, mapPtr, imuParams->imuIdx,
      &imuAugmentedParamPtrs, &imuAugmentedParamDims);
  pointDataPtr->setImuAugmentedParameterPtrs(imuAugmentedParamPtrs, imuAugmentedParamDims,
                                             imuParams);
  pointDataPtr->computePoseAndVelocityAtObservation();
}

TriangulationStatus EstimationHelper::triangulateMapPoint(
    const MapPoint &mp,
    std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const CameraRig &cameraRig,
    swift_vio::PointLandmark *landmark,
    Eigen::AlignedVector<Eigen::Vector2d> *obsList,
    Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
    swift_vio::PointSharedData *pointDataPtr,
    size_t minTrackLength, bool checkDisparity) {
  TriangulationStatus status;
  // each entry is undistorted coordinates in image plane at
  // z=1 in the specific camera frame, [\bar{x},\bar{y},1]
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      obsDirections;
  std::vector<std::pair<uint64_t, int>> badObservationIdentifiers;
  size_t numObs = gatherMapPointObservations(
      mp, mapPtr, cameraRig, pointDataPtr, &obsDirections,
      obsList, obsStdList, &badObservationIdentifiers);

  if (numObs < minTrackLength) {
    status.lackObservations = true;
    return status;
  }

  std::vector<okvis::kinematics::Transformation,
              Eigen::aligned_allocator<okvis::kinematics::Transformation>>
      T_WSs = pointDataPtr->poseAtFrameList();

  std::vector<size_t> camIndices = pointDataPtr->cameraIndexList();
  size_t numCameras = cameraRig.numCameras();
  Eigen::AlignedVector<okvis::kinematics::Transformation> T_BCs;
  T_BCs.reserve(numCameras);
  for (size_t j = 0u; j < numCameras; ++j) {
    T_BCs.push_back(cameraRig.getCameraExtrinsic(j));
  }

  if (checkDisparity) {
    double focalLength = cameraRig.cameraGeometry(0)->focalLengthU();
    if (hasLowDisparity(obsDirections, T_WSs, T_BCs, focalLength, camIndices,
                        6.0)) {
      status.raysParallel = true;
      return status;
    }
  }

  pointDataPtr->decideAnchors(landmark->modelId(), false);
  std::vector<size_t> anchorSeqIds = pointDataPtr->anchorObservationIds();

  std::vector<okvis::kinematics::Transformation,
              Eigen::aligned_allocator<okvis::kinematics::Transformation>>
      T_WCa_list;
  T_WCa_list.reserve(anchorSeqIds.size());

  // anchor body frame is at the state epoch, so its pose does not depend on td
  // and tr any more.
  for (auto anchorObsIndex : anchorSeqIds) {
    okvis::kinematics::Transformation T_WB =
        pointDataPtr->poseParameterBlockPtr(anchorObsIndex)->estimate();
    T_WCa_list.push_back(T_WB * T_BCs[camIndices[anchorObsIndex]]);
  }

  status = landmark->initialize(T_WSs, obsDirections, T_BCs, T_WCa_list,
                                camIndices, anchorSeqIds);
  return status;
}

TriangulationStatus EstimationHelper::triangulateMapPointRs(
    const MapPoint &mp, const StateMap &statesMap,
    std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const BareMultiFrameMap &frameTimeMap, const CameraRig &cameraRig,
    std::shared_ptr<const okvis::ImuParameters> imuParams,
    swift_vio::PointLandmark *landmark,
    Eigen::AlignedVector<Eigen::Vector2d> *obsList,
    Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
    swift_vio::PointSharedData *pointDataPtr,
    std::vector<uint64_t> *orderedFrameIdsToUse, size_t minTrackLength,
    size_t minCulledFrames, bool checkDisparity, bool anchorInKeyframe) {
  TriangulationStatus status;
  // each entry is undistorted coordinates in image plane at
  // z=1 in the specific camera frame, \f$ [\bar{x},\bar{y},1] \f$.
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      obsDirections;
  std::vector<std::pair<uint64_t, int>> badObservationIdentifiers;
  size_t numObs = gatherMapPointObservationsRs(
      mp, mapPtr, frameTimeMap, cameraRig, pointDataPtr, &obsDirections,
      obsList, obsStdList, &badObservationIdentifiers);

  if (numObs < minTrackLength) {
    status.lackObservations = true;
    return status;
  }

  propagatePoseAndVelocityForMapPoint(statesMap, mapPtr, imuParams,
                                      pointDataPtr);

  std::vector<okvis::kinematics::Transformation,
              Eigen::aligned_allocator<okvis::kinematics::Transformation>>
      T_WSs = pointDataPtr->poseAtObservationList();

  std::vector<size_t> camIndices = pointDataPtr->cameraIndexList();
  size_t numCameras = cameraRig.numCameras();
  Eigen::AlignedVector<okvis::kinematics::Transformation> T_BCs;
  T_BCs.reserve(numCameras);
  for (size_t j = 0u; j < numCameras; ++j) {
    T_BCs.push_back(cameraRig.getCameraExtrinsic(j));
  }

  if (checkDisparity) {
    double focalLength = cameraRig.cameraGeometry(0)->focalLengthU();
    if (hasLowDisparity(obsDirections, T_WSs, T_BCs, focalLength, camIndices,
                        6.0)) {
      status.raysParallel = true;
      return status;
    }
  }

  bool anchorFound = false;
  if (orderedFrameIdsToUse) {
    swift_vio::eraseBadObservations(badObservationIdentifiers,
                                    orderedFrameIdsToUse);
    if (orderedFrameIdsToUse->size() < minCulledFrames) {
      status.lackObservations = true;
      return status;
    }
    anchorFound = pointDataPtr->decideAnchors(*orderedFrameIdsToUse,
                             landmark->modelId(), anchorInKeyframe);
  } else {
    anchorFound = pointDataPtr->decideAnchors(landmark->modelId(), anchorInKeyframe);
  }
  if (!anchorFound) {
    status.noKeyframeHost = true;
    return status;
  }
  std::vector<size_t> anchorSeqIds = pointDataPtr->anchorObservationIds();

  std::vector<okvis::kinematics::Transformation,
              Eigen::aligned_allocator<okvis::kinematics::Transformation>>
      T_WCa_list;
  T_WCa_list.reserve(anchorSeqIds.size());

  // anchor body frame is at the state epoch, so its pose does not depend on td
  // and tr any more.
  for (auto anchorObsIndex : anchorSeqIds) {
    okvis::kinematics::Transformation T_WB =
        pointDataPtr->poseParameterBlockPtr(anchorObsIndex)->estimate();
    T_WCa_list.push_back(T_WB * T_BCs[camIndices[anchorObsIndex]]);
  }

  status = landmark->initialize(T_WSs, obsDirections, T_BCs, T_WCa_list,
                                camIndices, anchorSeqIds);

  if (status.triangulationOk && orderedFrameIdsToUse) {
    pointDataPtr->removeExtraObservations(*orderedFrameIdsToUse);
  }
  return status;
}

Eigen::Vector4d EstimationHelper::anchoredInverseDepthToWorldCoordinates(
    std::shared_ptr<const okvis::ceres::Map> mapPtr, const CameraRig &cameraRig,
    const Eigen::Vector4d &ab1rho, const CameraIdentifier &anchorCamera) {
  okvis::kinematics::Transformation T_WB =
      std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
          mapPtr->parameterBlockPtr(anchorCamera.frameId))
          ->estimate();
  const okvis::kinematics::Transformation T_BCa =
      cameraRig.getCameraExtrinsic(anchorCamera.cameraIndex);
  Eigen::Vector4d hpW = T_WB * T_BCa * ab1rho;
  double inverseW = 1.0 / hpW[3];
  return hpW * inverseW;
}

void EstimationHelper::addCameraObservations(
    const StateMap &statesMap,
    const std::unordered_map<uint64_t, BareMultiFrame> &frameTimeMap,
    const CameraRig &cameraRig, PointMap *landmarksMap,
    std::shared_ptr<okvis::ceres::Map> mapPtr,
    std::shared_ptr<const okvis::ImuParameters> imuParameters,
    ::ceres::LossFunction * lossFuncPtr) {
  // TODO: add observations for database landmarks which have Euclidean coordinates.

  // add opportunistic landmark observations.
  for (PointMap::iterator pit = landmarksMap->begin();
       pit != landmarksMap->end(); ++pit) {
    if (pit->second.inState()) {
      MapPoint &mp = pit->second;
      for (ObservationMap::reverse_iterator riter =
               mp.observations.rbegin();
           riter != mp.observations.rend(); ++riter) {
        ::ceres::ResidualBlockId retVal = 0u;
        if (riter->second.residualId == 0u) {
          okvis::cameras::DistortionType distortionType =
              cameraRig.distortionType(riter->first.cameraIndex);

#define DISTORTION_MODEL_CASE(camera_geometry_t)                               \
  {                                                                            \
    std::shared_ptr<const camera_geometry_t> cameraGeometry =                  \
        cameraRig.template geometryAs<camera_geometry_t>(                      \
            riter->first.cameraIndex);                                         \
    okvis::Time targetImageTime = frameTimeMap.at(riter->first.frameId)        \
                                      .timestamp(riter->first.cameraIndex);    \
    retVal = addPointFrameAidpResidual(                                        \
        statesMap.at(riter->first.frameId), statesMap.at(mp.anchorStateId),    \
        cameraGeometry, pit->first, riter->first, riter->second,               \
        mp.anchorCamera(), mapPtr, imuParameters, targetImageTime,             \
        lossFuncPtr);                                                          \
    riter->second.residualId = reinterpret_cast<uint64_t>(retVal);             \
  }

          switch (distortionType) { DISTORTION_MODEL_SWITCH_CASES }

#undef DISTORTION_MODEL_CASE
        }
      }
    }
  }
}

void EstimationHelper::configureSolver(
    std::shared_ptr<okvis::ceres::Map> mapPtr, size_t numIter,
    size_t numThreads, bool verbose) {
  // assemble options
  mapPtr->options.linear_solver_type = ::ceres::SPARSE_SCHUR;
  // mapPtr->options.initial_trust_region_radius = 1.0e4;
  // mapPtr->options.initial_trust_region_radius = 2.0e6;
  // mapPtr->options.preconditioner_type = ::ceres::IDENTITY;
  mapPtr->options.trust_region_strategy_type = ::ceres::DOGLEG;
  // mapPtr->options.trust_region_strategy_type =
  // ::ceres::LEVENBERG_MARQUARDT; mapPtr->options.use_nonmonotonic_steps =
  // true; mapPtr->options.max_consecutive_nonmonotonic_steps = 10;
  // mapPtr->options.function_tolerance = 1e-12;
  // mapPtr->options.gradient_tolerance = 1e-12;
  // mapPtr->options.jacobi_scaling = false;
#ifdef USE_OPENMP
  mapPtr->options.num_threads = numThreads;
#endif
  mapPtr->options.max_num_iterations = numIter;

  if (verbose) {
    mapPtr->options.minimizer_progress_to_stdout = true;
  } else {
    mapPtr->options.minimizer_progress_to_stdout = false;
  }
}

void EstimationHelper::transferWorldFrame(
    const StateMap &statesMap, const okvis::kinematics::Transformation &T_W0B0, uint64_t poseId,
    std::shared_ptr<okvis::ceres::Map> mapPtr,
    PointMap *landmarks) {
  okvis::kinematics::Transformation T_W1B0;
  get_T_WB(mapPtr, poseId, T_W1B0);

//  okvis::kinematics::Transformation T_W0W1 = T_W0B0 * T_W1B0.inverse();  // Gauge freedom 6
  okvis::kinematics::Transformation T_W0W1;  // Gauge freedom 4
  Eigen::Quaterniond q_W0W1 = T_W0B0.q() * T_W1B0.q().inverse();
  Eigen::Vector3d eul_W0W1 = okvis::kinematics::R2ypr(q_W0W1.toRotationMatrix());
  Eigen::Matrix3d R_W0W1 = okvis::kinematics::ypr2R(Eigen::Vector3d(eul_W0W1[0], 0, 0));
  T_W0W1.setRotation(Eigen::Quaterniond(R_W0W1));
  T_W0W1.setTranslation(T_W0B0.r() - R_W0W1 * T_W1B0.r());

  for (const auto & state : statesMap) {
      uint64_t sid = state.first;
      std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlockPtr = mapPtr
          ->parameterBlockPtr(sid);
      std::shared_ptr<okvis::ceres::PoseParameterBlock> poseBlockPtr =
      std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(parameterBlockPtr);
      poseBlockPtr->setEstimate(T_W0W1 * poseBlockPtr->estimate());

      uint64_t vid = state.second.global.at(GlobalStates::v_WS).id;
      std::shared_ptr<okvis::ceres::ParameterBlock> blockPtr = mapPtr
          ->parameterBlockPtr(vid);
      std::shared_ptr<okvis::ceres::SpeedParameterBlock> speedBlockPtr =
      std::static_pointer_cast<okvis::ceres::SpeedParameterBlock>(blockPtr);
      speedBlockPtr->setEstimate(T_W0W1.C() * speedBlockPtr->estimate());
  }
  for (auto &point : *landmarks) {
    if (point.second.inState() && point.second.anchorStateId == 0) {
      std::shared_ptr<okvis::ceres::ParameterBlock> blockPtr =
          mapPtr->parameterBlockPtr(point.first);
      std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock> pBlockPtr =
          std::static_pointer_cast<
              okvis::ceres::HomogeneousPointParameterBlock>(blockPtr);
      pBlockPtr->setEstimate(T_W0W1 * pBlockPtr->estimate());
    }
  }
}

void EstimationHelper::updateLandmarksFromMap(
    std::shared_ptr<const okvis::ceres::Map> mapPtr, const CameraRig &cameraRig,
    PointMap *landmarksMap) {
  okvis::ceres::HomogeneousPointLocalParameterization
      homogeneousLandmarkParamization;
  for (auto it = landmarksMap->begin(); it != landmarksMap->end(); ++it) {
    if (it->second.inState()) {
      it->second.quality = 1.0;
      Eigen::Vector4d lmkParams = Eigen::Map<const Eigen::Vector4d>(
          mapPtr->parameterBlockPtr(it->first)->parameters());
      if (it->second.anchorStateId == 0u) {
        it->second.pointHomog = lmkParams;
      } else {
        it->second.pointHomog =
            EstimationHelper::anchoredInverseDepthToWorldCoordinates(
                mapPtr, cameraRig, lmkParams, it->second.anchorCamera());
      }
    } else { // update landmark positions for visualization and loop closure.
      constexpr bool showPrematureLandmarks = false;
      if (showPrematureLandmarks) {
        if (it->second.observations.size() < 3u) {
          continue;
        }
        swift_vio::PointLandmark pointLandmark(
            it->first,
            okvis::ceres::HomogeneousPointLocalParameterization::kModelId,
            &homogeneousLandmarkParamization);
        Eigen::AlignedVector<Eigen::Vector2d> obsList;
        Eigen::AlignedVector<Eigen::Vector2d> obsStdList;
        std::shared_ptr<swift_vio::PointSharedData> pointDataPtr(
            new swift_vio::PointSharedData());
        swift_vio::TriangulationStatus status = triangulateMapPoint(
            it->second, mapPtr, cameraRig, &pointLandmark, &obsList,
            &obsStdList, pointDataPtr.get(), 2, true);
        if (status.triangulationOk) {
          it->second.quality = 1.0;
          it->second.pointHomog = pointLandmark.estimate();
        } else {
          it->second.quality = 0.0;
        }
      }
    }
  }
}

void EstimationHelper::updateCameraRig(
    const States &lastState, std::shared_ptr<const okvis::ceres::Map> mapPtr,
    const okvis::CameraNoiseParametersVec &noiseVec,
    CameraRig *cameraRig) {
  size_t numCameras = cameraRig->numCameras();

  uint64_t id = lastState.sensors.at(SensorStates::Camera)
                    .at(kMainCameraIndex)
                    .at(CameraSensorStates::T_XCi)
                    .id;
  std::shared_ptr<const okvis::ceres::PoseParameterBlock> parameterBlockPtr =
      std::dynamic_pointer_cast<const okvis::ceres::PoseParameterBlock>(
          mapPtr->parameterBlockPtr(id));
  okvis::kinematics::Transformation T_BC0 = parameterBlockPtr->estimate();

  for (size_t camIdx = 0u; camIdx < numCameras; ++camIdx) {
    const SpecificSensorStatesContainer &camInfo = lastState.sensors.at(SensorStates::Camera)
          .at(camIdx);
    if (!noiseVec.at(camIdx).isExtrinsicsFixed()) {
      int extrinsicRepId = cameraRig->getExtrinsicRepId(camIdx);
      okvis::kinematics::Transformation T_XCi;
      uint64_t eid = camInfo.at(CameraSensorStates::T_XCi).id;
      std::shared_ptr<const okvis::ceres::PoseParameterBlock> parameterBlockPtr =
          std::dynamic_pointer_cast<const okvis::ceres::PoseParameterBlock>(
              mapPtr->parameterBlockPtr(eid));
      switch (extrinsicRepId) {
      case Extrinsic_p_CB::kModelId:
      case Extrinsic_p_BC_q_BC::kModelId: {
        cameraRig->setCameraExtrinsic(camIdx, parameterBlockPtr->estimate());
      } break;
      case Extrinsic_p_C0C_q_C0C::kModelId: {
        T_XCi = parameterBlockPtr->estimate();
        cameraRig->setCameraExtrinsic(camIdx, T_BC0 * T_XCi);
      } break;
      }
    }
    if (!noiseVec.at(camIdx).isIntrinsicsFixed()) {
      uint64_t inid = camInfo.at(CameraSensorStates::Intrinsics).id;
      std::shared_ptr<const okvis::ceres::EuclideanParamBlock> intrinsicPtr =
          std::dynamic_pointer_cast<const okvis::ceres::EuclideanParamBlock>(
              mapPtr->parameterBlockPtr(inid));
      cameraRig->setCameraIntrinsics(camIdx, intrinsicPtr->estimate());
    }
    if (!noiseVec.at(camIdx).isTimeDelayFixed()) {
      uint64_t tdid = camInfo.at(CameraSensorStates::TD).id;
      std::shared_ptr<const okvis::ceres::CameraTimeParamBlock> tdBlockPtr =
          std::dynamic_pointer_cast<const okvis::ceres::CameraTimeParamBlock>(
              mapPtr->parameterBlockPtr(tdid));
      double tdEstimate = tdBlockPtr->estimate();
      cameraRig->setImageDelay(camIdx, tdEstimate);
    }
    if (!noiseVec.at(camIdx).isReadoutTimeFixed()) {
      uint64_t trid = camInfo.at(CameraSensorStates::TR).id;
      std::shared_ptr<const okvis::ceres::CameraTimeParamBlock> trBlockPtr =
          std::dynamic_pointer_cast<const okvis::ceres::CameraTimeParamBlock>(
              mapPtr->parameterBlockPtr(trid));
      double trEstimate = trBlockPtr->estimate();
      cameraRig->setReadoutTime(camIdx, trEstimate);
    }
  } // every camera
}

void EstimationHelper::updateImuRig(
    const States &lastState, std::shared_ptr<const okvis::ceres::Map> mapPtr,
    ImuRig *imuRig) {
  for (size_t i = 0; i < imuRig->numImus(); ++i) {
    std::shared_ptr<const okvis::ceres::BiasParameterBlock> bPtr = std::static_pointer_cast<const okvis::ceres::BiasParameterBlock>(
          mapPtr->parameterBlockPtr(lastState.sensors.at(SensorStates::Imu).at(i).at(ImuSensorStates::Bias).id));
    imuRig->at(i).setGyroBias(bPtr->estimate().head<3>());
    imuRig->at(i).setAccelBias(bPtr->estimate().tail<3>());
    std::vector<const double *> imuAugmentedParamPtrs;
    std::vector<size_t> imuAugmentedParamDims;
    getImuAugmentedParameterPtrs(lastState, mapPtr, i, &imuAugmentedParamPtrs,
                                 &imuAugmentedParamDims);
    Eigen::Matrix<double, Eigen::Dynamic, 1> extraParams;
    getImuAugmentedStatesEstimate(imuAugmentedParamPtrs, imuAugmentedParamDims, &extraParams, imuRig->getModelId(i));
    imuRig->setImuAugmentedParams(i, extraParams);
  }
}

void EstimationHelper::updateImuParametersVector(
    const States &lastState, std::shared_ptr<const okvis::ceres::Map> mapPtr,
    std::vector<std::shared_ptr<okvis::ImuNoiseParameters>> *imuParamsVec) {
  if (imuParamsVec->at(0)->isGravityDirectionVariable()) {
    uint64_t gWId = lastState.global.at(GlobalStates::GravityDirection).id;
    std::shared_ptr<const okvis::ceres::NormalVectorParameterBlock>
        paramBlockPtr = std::static_pointer_cast<
            const okvis::ceres::NormalVectorParameterBlock>(
            mapPtr->parameterBlockPtr(gWId));
    Eigen::Vector3d unitgW = paramBlockPtr->estimate();

    for (size_t i = 0; i < imuParamsVec->size(); ++i) {
      imuParamsVec->at(i)->setGravityDirection(unitgW);
    }
  }
}

void EstimationHelper::get_T_WB(std::shared_ptr<const okvis::ceres::Map> mapPtr,
                                uint64_t poseId,
                                okvis::kinematics::Transformation &T_WB) {
  std::shared_ptr<const okvis::ceres::ParameterBlock> parameterBlockPtr =
      mapPtr->parameterBlockPtr(poseId);
  std::shared_ptr<const okvis::ceres::PoseParameterBlock> stateParameterBlockPtr =
      std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
          parameterBlockPtr);
  T_WB = stateParameterBlockPtr->estimate();
}

void EstimationHelper::get_T_WB_lin(
    std::shared_ptr<const okvis::ceres::Map> mapPtr, uint64_t poseId,
    okvis::kinematics::Transformation &T_WB) {
  std::shared_ptr<const okvis::ceres::ParameterBlock> parameterBlockPtr =
      mapPtr->parameterBlockPtr(poseId);
  std::shared_ptr<const okvis::ceres::PoseParameterBlock> stateParameterBlockPtr =
      std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
          parameterBlockPtr);
  T_WB = stateParameterBlockPtr->linPoint();
}

void EstimationHelper::getVariableCameraIntrinsics(
    const States &currentState, std::shared_ptr<const okvis::ceres::Map> mapPtr,
    size_t camIdx, const okvis::CameraNoiseParameters &camNoiseParams,
    Eigen::Matrix<double, Eigen::Dynamic, 1> *cameraParams) {
  Eigen::VectorXd intrinsics;
  const SpecificSensorStatesContainer &camInfo =
      currentState.sensors.at(SensorStates::Camera).at(camIdx);
  if (!camNoiseParams.isIntrinsicsFixed()) {
    uint64_t inid = camInfo.at(CameraSensorStates::Intrinsics).id;
    std::shared_ptr<const okvis::ceres::EuclideanParamBlock> intrinsicPtr =
        std::dynamic_pointer_cast<const okvis::ceres::EuclideanParamBlock>(
            mapPtr->parameterBlockPtr(inid));
    intrinsics = intrinsicPtr->estimate();
  }

  cameraParams->resize(intrinsics.size() + 2, 1);
  cameraParams->head(intrinsics.size()) = intrinsics;

  int numCols = intrinsics.size();
  double tdEstimate(0), trEstimate(0);
  if (!camNoiseParams.isTimeDelayFixed()) {
    uint64_t tdid = camInfo.at(CameraSensorStates::TD).id;
    std::shared_ptr<const okvis::ceres::CameraTimeParamBlock> tdPtr =
        std::dynamic_pointer_cast<const okvis::ceres::CameraTimeParamBlock>(
            mapPtr->parameterBlockPtr(tdid));
    tdEstimate = tdPtr->estimate();
    (*cameraParams)[numCols] = tdEstimate;
    ++numCols;
  }

  if (!camNoiseParams.isReadoutTimeFixed()) {
    uint64_t trid = camInfo.at(CameraSensorStates::TR).id;
    std::shared_ptr<const okvis::ceres::CameraTimeParamBlock> trPtr =
        std::dynamic_pointer_cast<const okvis::ceres::CameraTimeParamBlock>(
            mapPtr->parameterBlockPtr(trid));
    trEstimate = trPtr->estimate();
    (*cameraParams)[numCols] = trEstimate;
    ++numCols;
  }

  cameraParams->conservativeResize(numCols, 1);
}

bool EstimationHelper::removeObservationAndResidual(
    ::ceres::ResidualBlockId residualBlockId,
    std::shared_ptr<okvis::ceres::Map> mapPtr, PointMap *landmarksMap,
    std::unordered_map<uint64_t, BareMultiFrame> *nframeToLandmarkIds) {
  const okvis::ceres::Map::ParameterBlockCollection parameters =
      mapPtr->parameters(residualBlockId);
  const uint64_t landmarkId = parameters.at(1).first;
  // remove in landmarksMap
  MapPoint &mapPoint = landmarksMap->at(landmarkId);
  for (std::map<okvis::KeypointIdentifier, KeypointObservation>::iterator it =
           mapPoint.observations.begin();
       it != mapPoint.observations.end();) {
    if (it->second.residualId == uint64_t(residualBlockId)) {
      auto mfp = nframeToLandmarkIds->find(it->first.frameId);
      mfp->second.setLandmarkId(it->first.cameraIndex, it->first.keypointIndex,
                                0);
      it = mapPoint.observations.erase(it);
      break;
    } else {
      it++;
    }
  }
  // remove residual block
  mapPtr->removeResidualBlock(residualBlockId);
  return true;
}

void EstimationHelper::removeObservationInFrames(
    const std::vector<uint64_t> &orderedRemoveFrames,
    Eigen::AlignedMap<okvis::KeypointIdentifier, KeypointObservation>
        *observations,
    std::shared_ptr<okvis::ceres::Map> mapPtr,
    std::unordered_map<uint64_t, BareMultiFrame> *nframeToLandmarkIds) {
  ObservationMap::iterator obsIter = observations->begin();
  for (uint64_t stateId : orderedRemoveFrames) {
    while (obsIter != observations->end() && obsIter->first.frameId < stateId) {
      ++obsIter;
    }
    while (obsIter != observations->end() &&
           obsIter->first.frameId == stateId) {
      // loop in case that there are more than one observations in an NFrame for
      // a landmark.
      const okvis::KeypointIdentifier &kpi = obsIter->first;
      if (nframeToLandmarkIds) {
        auto mfp = nframeToLandmarkIds->find(kpi.frameId);
        mfp->second.setLandmarkId(kpi.cameraIndex, kpi.keypointIndex, 0);
      }
      if (obsIter->second.residualId) {
        mapPtr->removeResidualBlock(reinterpret_cast<::ceres::ResidualBlockId>(
            obsIter->second.residualId));
      } else {
        // This happens when the landmark of the feature track has not been
        // triangulated and thus none of the observations is added as a residual.
        VLOG(2) << "No residual is associated to frame "
                << obsIter->first.frameId << ", cam "
                << obsIter->first.cameraIndex << ", keypoint "
                << obsIter->first.keypointIndex << "!";
      }
      obsIter = observations->erase(obsIter);
    }
  }
}

uint64_t EstimationHelper::shouldMarginalizeSensorParamBlock(
    int sensorType, int sensorId, int sensorParamType,
    std::map<uint64_t, States>::const_iterator it,
    std::shared_ptr<const okvis::ceres::Map> mapPtr) {
  const StateInfo &sinfo =
      it->second.sensors[sensorType][sensorId][sensorParamType];
  if (!sinfo.exists) {
    return 0;
  }
  uint64_t blockId = sinfo.id;
  if (mapPtr->parameterBlockPtr(blockId)->fixed()) {
    return 0; // we never eliminate fixed blocks.
  }
  std::map<uint64_t, States>::const_iterator checkit = it;
  checkit++;
  // only get rid of it, if it's different
  if (checkit->second.sensors[sensorType][sensorId][sensorParamType].exists &&
      checkit->second.sensors[sensorType][sensorId][sensorParamType].id ==
          blockId) {
    return 0;
  }
  return blockId;
}

} // namespace swift_vio
