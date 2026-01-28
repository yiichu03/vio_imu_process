#include <gtest/gtest.h>

#include <simul/CameraSystemCreator.hpp>
#include <simul/VioSimTestSystem.hpp>
#include <simul/simGflags.hpp>
#include <swift_vio/ceres/RsReprojectionError.hpp>
#include <swift_vio/ceres/RsReprojectionErrorAidp.hpp>

DECLARE_string(log_dir); // FLAGS_log_dir can be passed in commandline as
                         // --log_dir=/some/log/dir

namespace {
void checkMSE(const Eigen::VectorXd &mse, const Eigen::VectorXd &desiredStdevs,
              const std::vector<std::string> &dimensionLabels) {
  for (int i = 0; i < mse.size(); ++i) {
    EXPECT_LT(mse[i], std::pow(desiredStdevs[i], 2))
        << dimensionLabels[i] + " MSE";
  }
}

void checkNEES(const Eigen::Matrix<double, 6, 1> &nees) {
  EXPECT_LT(nees[0], 8) << "Position NEES";
  EXPECT_LT(nees[1], 5) << "Orientation NEES";
  EXPECT_LT(nees[2], 10) << "Pose NEES";
}
} // namespace

okvis::VioParameters createVioParameters(const std::vector<std::string> &extrinsicRepNames) {
  okvis::VioParameters vioparameters;
  vioparameters.imu.model_name = "BG_BA_MG_TS_MA";

  for (size_t i = 0; i < extrinsicRepNames.size(); ++i) {
    std::shared_ptr<okvis::kinematics::Transformation> dummy_T_BC(
        new okvis::kinematics::Transformation());
    dummy_T_BC->setRandom();

    okvis::cameras::DistortionType distortionType =
        swift_vio::DistortionNameToTypeId(FLAGS_sim_distortion_type);
    std::shared_ptr<okvis::cameras::CameraBase> dummyCameraGeometry =
        simul::CameraSystemCreator::createTestCameraGeometry(distortionType);

    dummyCameraGeometry->setImageDelay(0.0);
    dummyCameraGeometry->setReadoutTime(0.0);
    vioparameters.nCameraSystem.addCamera(dummy_T_BC, dummyCameraGeometry,
                                          distortionType, "FXY_CXY",
                                          extrinsicRepNames[i], false);
  }
  return vioparameters;
}

void testEstimator(const std::vector<std::string> &extrinsicRepNames) {
  int cameraObservationModelId = 0;
  int landmarkModelId = FLAGS_sim_landmark_model;

  okvis::VioParameters vioparameters = createVioParameters(extrinsicRepNames);
  // parameters for generating simulation data.
  okvis::ImuNoiseParameters simImuNoiseParams;
  simul::initImuNoiseParams(vioparameters.imu.model_name, FLAGS_sim_sigma_g_c,
                            FLAGS_sim_sigma_a_c, FLAGS_sim_sigma_gw_c,
                            FLAGS_sim_sigma_aw_c, &simImuNoiseParams);
  simul::SimImuParameters imuParams(true);

  okvis::CameraNoiseParameters simCamNoiseParams;
  simul::initCameraNoiseParams(2e-2, 1e-2, &simCamNoiseParams);

  double landmarkCylinderRadius = 5;
  simul::SimCameraParameters cameraParams(
      false, true, true, simul::SimCameraModelType::EUROC,
      simul::CameraOrientation::Forward,
      simul::LandmarkGridType::FourWalls,
      landmarkCylinderRadius);

  simul::SimParameters simParameters(
      FLAGS_sim_trajectory_label, imuParams, cameraParams,
      FLAGS_sim_sigma_speed, simImuNoiseParams, simCamNoiseParams,
      FLAGS_sim_num_runs, 300.0, M_PI / 6, FLAGS_sim_data_path,
      FLAGS_log_dir);

  okvis::EstimatorOptions estimatorOptions;
  estimatorOptions.algorithm =
      swift_vio::EstimatorAlgorithm::SlidingWindowFilter;
  estimatorOptions.cameraObservationModelId = cameraObservationModelId;
  estimatorOptions.computeOkvisNees = false;
  estimatorOptions.numKeyframes = 5;
  estimatorOptions.numImuFrames = 3;
  estimatorOptions.useMahalanobisGating = false;

  swift_vio::PointLandmarkOptions plOptions;
  plOptions.minTrackLengthForSlam = 6;
  plOptions.maxHibernationFrames = 3;
  plOptions.maxInStateLandmarks = 80;
  plOptions.landmarkModelId = landmarkModelId;

  vioparameters.optimization = estimatorOptions;
  vioparameters.pointLandmarkOptions = plOptions;
  vioparameters.imu = simImuNoiseParams;
  vioparameters.camera_noise = simCamNoiseParams;

  simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
  simSystem.run(simParameters, &vioparameters);
}

void testMeasurementJacobian(int cameraObservationModelId, int landmarkModelId,
                             bool checkTriangulation, bool checkJacobian) {
  std::vector<std::string> extrinsicRepNames = {"P_BC_Q_BC", "P_BC_Q_BC"};
  okvis::VioParameters vioparameters = createVioParameters(extrinsicRepNames);

  // parameters for generating simulation data.
  okvis::ImuNoiseParameters simImuNoiseParams;
  simul::initImuNoiseParams(vioparameters.imu.model_name, FLAGS_sim_sigma_g_c,
                            FLAGS_sim_sigma_a_c, FLAGS_sim_sigma_gw_c,
                            FLAGS_sim_sigma_aw_c, &simImuNoiseParams);
  // disable perturbing IMU intrinsic parameters because
  // RsReprojectionFactor for comparison does not consider the effects of Mg Ts Ma.
  simImuNoiseParams.sigma_Mg_element = 0.0;
  simImuNoiseParams.sigma_Ts_element = 0.0;
  simImuNoiseParams.sigma_Ma_element = 0.0;
  simul::SimImuParameters imuParams;

  okvis::CameraNoiseParameters simCamNoiseParams;
  simul::initCameraNoiseParams(2e-2, 1e-2, &simCamNoiseParams);

  double landmarkCylinderRadius = 5;
  simul::SimCameraParameters cameraParams(
      false, false, true, simul::SimCameraModelType::EUROC,
      simul::CameraOrientation::Forward, simul::LandmarkGridType::FourWalls,
      landmarkCylinderRadius);

  simul::SimParameters simParameters(
      FLAGS_sim_trajectory_label, imuParams, cameraParams,
      0.0, simImuNoiseParams, simCamNoiseParams,
      1, 300.0, 0.0, "", FLAGS_log_dir, checkTriangulation,
      checkJacobian);

  okvis::EstimatorOptions estimatorOptions;
  estimatorOptions.algorithm =
      swift_vio::EstimatorAlgorithm::SlidingWindowFilter;
  estimatorOptions.cameraObservationModelId = cameraObservationModelId;
  estimatorOptions.computeOkvisNees = false;
  estimatorOptions.numKeyframes = 5;
  estimatorOptions.numImuFrames = 3;
  estimatorOptions.useMahalanobisGating = false;

  swift_vio::PointLandmarkOptions plOptions;
  plOptions.minTrackLengthForSlam = 6;
  plOptions.maxHibernationFrames = 3;
  plOptions.maxInStateLandmarks = 80;
  plOptions.landmarkModelId = landmarkModelId;

  vioparameters.optimization = estimatorOptions;
  vioparameters.pointLandmarkOptions = plOptions;
  vioparameters.imu = simImuNoiseParams;
  vioparameters.camera_noise = simCamNoiseParams;

  simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
  simSystem.runNoiseFree(simParameters, &vioparameters);
}

TEST(SlidingWindowFilterMono, TrajectoryLabel) { testEstimator({"P_BC_Q_BC"}); }

TEST(SlidingWindowFilter, TrajectoryLabel) {
  testEstimator({"P_BC_Q_BC", "P_BC_Q_BC"});
}

TEST(SlidingWindowFilter, triangulateMapPointAIDP) {
  int cameraObservationModelId = okvis::ceres::RsReprojectionErrorAidpBase::kModelId;
  testMeasurementJacobian(cameraObservationModelId, 1, true, false);
}

TEST(SlidingWindowFilter, triangulateMapPointHPP) {
  int cameraObservationModelId = okvis::ceres::RsReprojectionErrorBase::kModelId;
  testMeasurementJacobian(cameraObservationModelId, 0, true, false);
}

TEST(SlidingWindowFilter, measurementJacobianAIDP) {
  int cameraObservationModelId = okvis::ceres::RsReprojectionErrorAidpBase::kModelId;
  testMeasurementJacobian(cameraObservationModelId, 1, false, true);
}

TEST(SlidingWindowFilter, measurementJacobianHPP) {
  int cameraObservationModelId = okvis::ceres::RsReprojectionErrorBase::kModelId;
  testMeasurementJacobian(cameraObservationModelId, 0, false, true);
}
