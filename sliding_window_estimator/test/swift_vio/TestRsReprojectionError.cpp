#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ceres/ceres.h>

#include <swift_vio/CameraIdentifier.h>
#include <swift_vio/ceres/CameraTimeParamBlock.hpp>
#include <swift_vio/ceres/EuclideanParamBlock.hpp>
#include <swift_vio/ceres/EuclideanParamBlockSized.hpp>
#include <swift_vio/ceres/RsReprojectionErrorAidp.hpp>
#include <swift_vio/ceres/RsReprojectionError.hpp>
#include <swift_vio/ExtrinsicReps.hpp>
#include <swift_vio/ProjectionIntrinsicReps.h>

#include <okvis/FrameTypedefs.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/Time.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>

#include "SmootherJacobianUtilities.h"
#include <simul/PointLandmarkSimulationRS.hpp>
#include <simul/curves.h>
#include <simul/numeric_ceres_residual_Jacobian.hpp>

// When readout time, tr is non zero, analytic, numeric and automatic Jacobians
// of the rolling shutter reprojection factor are roughly the same.
// Surprisingly, if tr is zero, automatic Jacobians of the rolling shutter
// reprojection factor relative to the time offset and readout time are zeros
// and disagree with the values supported by both numeric and analytic
// approaches. Other than that, the rest Jacobians by the three method when tr
// is zero are roughly the same.

void setupPoseOptProblem(bool perturbPose, bool rollingShutter,
                         bool noisyKeypoint, int cameraObservationModelId) {
  // srand((unsigned int) time(0));
  swift_vio::CameraObservationOptions coo(cameraObservationModelId, perturbPose,
                                          rollingShutter, noisyKeypoint);

  swift_vio::CameraObservationJacobianTest jacTest(coo);

  std::shared_ptr<okvis::ImuParameters> imuParameters(
      new okvis::ImuParameters());
  double imuFreq = imuParameters->rate;
  Eigen::Vector3d ginw = imuParameters->gravity();

  jacTest.setImuParameters(imuParameters);

  std::shared_ptr<simul::TrajectoryBase> cameraMotion(
      new simul::WavyCircle(imuFreq, ginw));
  okvis::ImuMeasurementDeque imuMeasurements;

  okvis::Time startEpoch(2.0);
  okvis::Time endEpoch(5.0);
  cameraMotion->getTrueInertialMeasurements(startEpoch - okvis::Duration(1),
                                            endEpoch + okvis::Duration(1),
                                            imuMeasurements);
  jacTest.addNavStatesAndExtrinsic(cameraMotion, startEpoch, 0.3);

  double initialCameraTimeOffset(0.0); // camera time offset's initial estimate.
  double cameraTimeOffset(0.0);        // true camera time offset.
  jacTest.setImageTimes(cameraTimeOffset);
  jacTest.addImuAugmentedParameterBlocks();
  jacTest.addImuData(imuMeasurements);

  std::shared_ptr<swift_vio::DistortedPinholeCameraGeometry> cameraGeometry =
      std::static_pointer_cast<swift_vio::DistortedPinholeCameraGeometry>(
          swift_vio::DistortedPinholeCameraGeometry::createTestObject());

  Eigen::VectorXd intrinsicParams;
  cameraGeometry->getIntrinsics(intrinsicParams);
  double tr = 0;
  if (jacTest.coo_.rollingShutter) {
    tr = 0.03;
  }
  cameraGeometry->setReadoutTime(tr);
  cameraGeometry->setImageDelay(cameraTimeOffset);
  jacTest.addCameraParameterBlocks(intrinsicParams, startEpoch, tr,
                                   initialCameraTimeOffset);

  // get some random points
  const size_t numberTrials = 200;
  std::vector<std::shared_ptr<swift_vio::PointLandmark>> visibleLandmarks;
  Eigen::AlignedVector<Eigen::AlignedVector<Eigen::Vector2d>>
      pointObservationList;
  jacTest.createLandmarksAndObservations(cameraGeometry, &visibleLandmarks,
                                         &pointObservationList, numberTrials);

  LOG(INFO) << "created " << visibleLandmarks.size()
            << " visible points and add respective reprojection error terms... ";
  int verifyTimes = 0;
  int verifyHostTimes = 0;
  for (size_t i = 0u; i < visibleLandmarks.size(); ++i) {
    jacTest.addLandmark(visibleLandmarks[i]);
    for (size_t j = 0; j < pointObservationList[i].size(); ++j) {
      std::shared_ptr<okvis::ImuMeasurementDeque> imuMeasDequePtr(
          new okvis::ImuMeasurementDeque(imuMeasurements));

      if (coo.cameraObservationModelId ==
          okvis::ceres::RsReprojectionErrorBase::kModelId) {
        std::shared_ptr<::ceres::CostFunction> costFunctionPtr;
        std::shared_ptr<okvis::ceres::ErrorInterface> errorInterface;
        std::shared_ptr<okvis::ceres::RsReprojectionError<
            swift_vio::DistortedPinholeCameraGeometry>>
            localCostFunctionPtr(new okvis::ceres::RsReprojectionError<
                                 swift_vio::DistortedPinholeCameraGeometry>(
                cameraGeometry, pointObservationList[i][j],
                swift_vio::kCovariance, imuMeasDequePtr,
                std::shared_ptr<const Eigen::Matrix<double, 6, 1>>(),
                jacTest.stateEpoch(j), jacTest.getImageTimestamp(j, 0),
                imuParameters->g));
        costFunctionPtr = std::static_pointer_cast<::ceres::CostFunction>(
            localCostFunctionPtr);
        errorInterface = std::static_pointer_cast<okvis::ceres::ErrorInterface>(
            localCostFunctionPtr);
        jacTest.addResidual(costFunctionPtr, j, i);
        std::shared_ptr<swift_vio::PointSharedData> pointDataPtr;
        if (i % 20 == 0 && j == 2) {
          jacTest.verifyJacobians(errorInterface, j, i, pointDataPtr,
                                  cameraGeometry, pointObservationList[i][j]);
          ++verifyTimes;
        }
      } else if (coo.cameraObservationModelId ==
                 okvis::ceres::RsReprojectionErrorAidpBase::kModelId) {
        std::shared_ptr<::ceres::CostFunction> costFunctionPtr;
        std::shared_ptr<okvis::ceres::ErrorInterface> errorInterface;
        swift_vio::CameraIdentifier targetCameraId(j, jacTest.obsCamId_);
        swift_vio::CameraIdentifier hostCameraId(jacTest.hostFrameId_,
                                                 jacTest.hostCamId_);
        std::shared_ptr<okvis::ceres::RsReprojectionErrorAidpAdapter<
            swift_vio::DistortedPinholeCameraGeometry>>
            localCostFunctionPtr(
                new okvis::ceres::RsReprojectionErrorAidpAdapter<
                    swift_vio::DistortedPinholeCameraGeometry>(
                    targetCameraId, hostCameraId, pointObservationList[i][j],
                    swift_vio::kCovariance, cameraGeometry, imuMeasDequePtr,
                    imuParameters, jacTest.stateEpoch(j),
                    jacTest.getImageTimestamp(j, 0)));
        localCostFunctionPtr->setParameterBlockAndResidualSizes();
        costFunctionPtr = std::static_pointer_cast<::ceres::CostFunction>(
            localCostFunctionPtr);
        errorInterface = std::static_pointer_cast<okvis::ceres::ErrorInterface>(
            localCostFunctionPtr);
        jacTest.addResidual(costFunctionPtr, j, i);
        std::shared_ptr<swift_vio::PointSharedData> pointDataPtr;
        if (i % 20 == 0) {
          LOG(INFO) << "#" << verifyTimes << " Jacobian check for ReprojectionErrorAidp: frame "
                    << j << ", host " << jacTest.hostFrameId_ << ".";
          jacTest.verifyJacobians(errorInterface, j, i, pointDataPtr,
                                  cameraGeometry, pointObservationList[i][j]);
          if ((int)j == jacTest.hostFrameId_)
            ++verifyHostTimes;
          ++verifyTimes;
        }
      }
    }
  }
  LOG(INFO) << "Successfully constructed ceres solver pose optimization "
               "problem and verified "
            << verifyTimes << " Jacobians and " << verifyHostTimes
            << " host Jacobians.";
  jacTest.solveAndCheck();
}

TEST(CeresErrorTerms, RsReprojectionErrorNoiseFree) {
  setupPoseOptProblem(false, false, false,
                      okvis::ceres::RsReprojectionErrorBase::kModelId);
}

TEST(CeresErrorTerms, RsReprojectionErrorNoisy) {
  setupPoseOptProblem(true, true, true,
                      okvis::ceres::RsReprojectionErrorBase::kModelId);
}

TEST(CeresErrorTerms, RsReprojectionErrorAidpNoiseFree) {
  setupPoseOptProblem(false, false, false,
                      okvis::ceres::RsReprojectionErrorAidpBase::kModelId);
}

TEST(CeresErrorTerms, RsReprojectionErrorAidpNoisy) {
  setupPoseOptProblem(true, true, true,
                      okvis::ceres::RsReprojectionErrorAidpBase::kModelId);
}
