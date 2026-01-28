/**
 * @file vi_bundle_adjustment.cpp
 * @brief Use ceres solver to bundle-adjust the visual inertial SLAM with
 * calibration parameters given data in maplab csv format.
 * @author Jianzhu Huai
 */

#include <gtest/gtest.h>

#include <simul/SimulatorBase.hpp>

#include <swift_vio/ViBundleAdjustment.h>

DEFINE_string(camchain_yaml, "", "Path of camchain.yaml conforming to opencv yaml 1.0.");

DEFINE_string(imu_yaml, "", "Path of imu.yaml conforming to opencv yaml 1.0.");

DEFINE_string(initial_camchain_imu_yaml, "", "Path of initial_camchain_imu.yaml conforming to opencv yaml 1.0.");

DEFINE_bool(estimate_readout_time, true, "Estimate readout time in the optimization?");

DEFINE_double(
    max_projection_error_tol, 7.0,
    "If an observation's reprojection error is greater than this tolerance, it "
    "will be discounted in computing residual statistics.");

DECLARE_double(robust_loss_width);

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  FLAGS_logtostderr = 1;
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  if (argc < 2) {
    std::cout << "Usage:" << argv[0]
              << " maplab_csv_dir outptu_dir\nThe folder maplab_csv_dir is where landmark "
                 "observations, IMU data, landmarks, and initial trajectory "
                 "are stored in maplab csv format.\nThese data will be used to "
                 "construct the bundle adjustment problem which optimizes for "
                 "trajectory, landmarks, and calibration parameters.\n";
    return 1;
  }
  std::string csvdir = argv[1];
  std::string outputdir = argv[2];

  // TODO(jhuai): load the IMU parameters from a config yaml.
  okvis::ImuParameters imuParameters;
  std::string imuYaml;
  simul::loadImuYaml(imuYaml, &imuParameters);
  std::shared_ptr<simul::SimulatorBase> data(new simul::SimFromRealData(
      csvdir, imuParameters, false, false, false));

  data->initializeLandmarkGrid(simul::LandmarkGridType(), 5.0);

  std::string cameraYaml = FLAGS_camchain_yaml;
  std::shared_ptr<swift_vio::CameraRig> cameraSystem =
      simul::loadCameraSystemYaml(cameraYaml);

  std::string initialParamsYaml = FLAGS_initial_camchain_imu_yaml;
  Eigen::Vector3d initialGravity;
  Eigen::AlignedVector<simul::CameraSpatioTemporalParameters> initialRelativeParams;
  simul::loadInitialCameraImuParams(initialParamsYaml, &initialRelativeParams, &initialGravity);
  for (size_t i = 0u; i < cameraSystem->numCameras(); ++i) {
    cameraSystem->setCameraExtrinsic(i, okvis::kinematics::Transformation(
                                   initialRelativeParams.at(i).T_imu_cam));
    cameraSystem->setImageDelay(i,
                                initialRelativeParams.at(i).timeshift_cam_imu);
  }

  imuParameters.setGravityDirection(initialGravity.normalized());

  okvis::CameraNoiseParameters cameraNoiseParams;
  cameraNoiseParams.sigma_absolute_orientation = 0;
  cameraNoiseParams.sigma_absolute_translation = 0;
  cameraNoiseParams.sigma_observation = 1.0;

  swift_vio::ViBundleAdjustment viba;

  viba.addImu(imuParameters);
  viba.addCameraSystem(cameraSystem);
  viba.addCameraParameterStds(cameraNoiseParams);

  swift_vio::VisualInertialBAOptions options;
  options.estimateReadoutTime = FLAGS_estimate_readout_time;
  viba.buildOptimizationProblem(data, options);

  viba.solve();

  viba.computeStatistics(FLAGS_max_projection_error_tol);

  viba.saveResults(outputdir);

  return 0;
}


