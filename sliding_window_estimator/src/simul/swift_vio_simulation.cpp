/**
 * @file swift_vio_simulation.cpp
 * @brief This file includes the simulation for swift_vio.
 * @author Jianzhu Huai
 */
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <io_wrap/CommonGflags.hpp>
#include <io_wrap/Publisher.hpp>

#include <simul/simGflags.hpp>
#include <simul/SimParametersReader.hpp>

#include <simul/SimParameters.h>
#include <simul/VioSimTestSystem.hpp>

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
}  // namespace

namespace bfs = boost::filesystem;

void configureIO(
   simul::SimParameters *simParameters,
    const std::string &methodcodename) {
  simParameters->simDataPath = FLAGS_sim_data_path;
  bfs::path dir(FLAGS_log_dir);
  bfs::path basename(methodcodename);
  bfs::path fullpath = dir / basename;
  if (!boost::filesystem::exists(fullpath))
    bfs::create_directory(fullpath);
  simParameters->outputdir = fullpath.string();
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true); // true to strip gflags
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  const std::string nodeName = "swift_vio_sim";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh(nodeName);

  std::string configFilename;
  if (argc >= 2) {
    configFilename = argv[1];
  } else {
    LOG(ERROR) << "Usage:" << argv[0] << " </swift_vio_ws/src/swift_vio/config/simulation.yaml> [extra gflags]";
    return 1;
  }

  // load initial values and std devs for VIO parameters.
  simul::SimParametersReader simParamsReader(configFilename);
  okvis::VioParameters vioParameters;
  simParamsReader.getVioParameters(&vioParameters);
  simul::SimParameters simParameters;
  simParamsReader.getSimParameters(&simParameters);
  if (!FLAGS_sim_trajectory_csv.empty()) {
    simParameters.setTrajLabel(vio::basename(FLAGS_sim_trajectory_csv));
  }
  swift_vio::BackendParams flsParams;
  flsParams.parseYAML(configFilename);

  swift_vio::Publisher publisher(nh);
  publisher.setParameters(vioParameters);

  { // monocular vio initializer
    okvis::VioParameters parameters = vioParameters;
    parameters.imu.model_name = "BG_BA_MG_TS_MA";
    parameters.nCameraSystem.removeCamera(1);
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 1);
    parameters.pointLandmarkOptions.landmarkModelId = 1;
    configureIO(&simParameters, "mono_MG_TS_MA_initializer");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.runFromInitializer(simParameters, &parameters);
  }

  { // stereo vio initializer
    okvis::VioParameters parameters = vioParameters;
    parameters.imu.model_name = "BG_BA_MG_TS_MA";
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 2);
    parameters.pointLandmarkOptions.landmarkModelId = 1;
    configureIO(&simParameters, "stereo_MG_TS_MA_initializer");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.runFromInitializer(simParameters, &parameters);
  }

  { // monocular SWF
    okvis::VioParameters parameters = vioParameters;
    parameters.imu.model_name = "BG_BA_MG_TS_MA";
    parameters.nCameraSystem.removeCamera(1);
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 1);
    configureIO(&simParameters, "mono_MG_TS_MA");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.run(simParameters, &parameters);
  }

  { // stereo SWF
    okvis::VioParameters parameters = vioParameters;
    parameters.imu.model_name = "BG_BA_MG_TS_MA";
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 2);
    configureIO(&simParameters, "stereo_MG_TS_MA");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.run(simParameters, &parameters);
  }

  { // monocular SWF
    okvis::VioParameters parameters = vioParameters;
    parameters.imu.model_name = "BG_BA";
    parameters.nCameraSystem.removeCamera(1);
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 1);
    configureIO(&simParameters, "mono_BG_BA");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.run(simParameters, &parameters);
  }

  { // stereo SWF
    okvis::VioParameters parameters = vioParameters;
    parameters.imu.model_name = "BG_BA";
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 2);
    configureIO(&simParameters, "stereo_BG_BA");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.run(simParameters, &parameters);
  }

  { // mono okvis
    okvis::VioParameters parameters = vioParameters;
    parameters.optimization.algorithm = swift_vio::EstimatorAlgorithm::OkvisEstimator;
    parameters.imu.model_name = "BG_BA";
    parameters.nCameraSystem.removeCamera(1);
    simParameters.imuNoiseParams.model_name = "BG_BA";
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 1);
    configureIO(&simParameters, "mono_OKVIS_SWF");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.runOkvisEstimator(simParameters, flsParams, &parameters);
  }

  { // stereo okvis
    okvis::VioParameters parameters = vioParameters;
    parameters.optimization.algorithm = swift_vio::EstimatorAlgorithm::OkvisEstimator;
    parameters.imu.model_name = "BG_BA";
    simParameters.imuNoiseParams.model_name = "BG_BA";
    CHECK_EQ(parameters.nCameraSystem.numCameras(), 2);
    configureIO(&simParameters, "stereo_OKVIS_SWF");
    simul::VioSimTestSystem simSystem(checkMSE, checkNEES);
    simSystem.registerCallbacks(&publisher);
    simSystem.runOkvisEstimator(simParameters, flsParams, &parameters);
  }

}
