/**
 * @file swift_vio_node.cpp
 * @brief This file includes the ROS node implementation.
 * @author Jianzhu Huai
 */

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include "sensor_msgs/Imu.h"
#include <image_transport/image_transport.h>

#include <glog/logging.h>

#include <io_wrap/CommonGflags.hpp>
#include <io_wrap/Player.hpp>
#include <io_wrap/RosParametersReader.hpp>
#include <io_wrap/Subscriber.hpp>
#include <swift_vio/IoUtil.hpp>
#include <swift_vio/VioFactoryMethods.hpp>
#include <swift_vio/VioSystem.hpp>

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true); // true to strip gflags
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  const std::string nodeName = "swift_vio_node";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh(nodeName);

  std::string configFilename;
  if (argc >= 2) {
    configFilename = argv[1];
  } else {
    LOG(ERROR) << "Usage:" << argv[0] << " <config yml> [extra gflags]";
    return 1;
  }

  swift_vio::RosParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);
  swift_vio::setInputParameters(&parameters.input);

  swift_vio::Publisher publisher(nh);
  publisher.setParameters(parameters);

  std::shared_ptr<swift_vio::EstimatorBase> initializer =
      swift_vio::createInitializer(parameters.optimization);

  std::shared_ptr<swift_vio::EstimatorBase> estimator = swift_vio::createBackend(
      parameters.optimization);
  std::shared_ptr<swift_vio::CameraFrontendBase> frontend = swift_vio::createFrontend(
      parameters.nCameraSystem.numCameras(), parameters.frontendOptions);

  swift_vio::KeyframePublisher keyframePublisher(nh);
  keyframePublisher.waitForSubscriber(FLAGS_loopclosure);

  std::shared_ptr<swift_vio::VioSystemBase> vioSystem(new swift_vio::VioSystem(
      parameters, initializer, estimator, frontend));

  swift_vio::registerCallbacks(
      FLAGS_output_dir, parameters, vioSystem, &publisher, &keyframePublisher);

  // player to grab messages directly from files on a hard drive.
  std::shared_ptr<swift_vio::Player> pPlayer;
  std::shared_ptr<std::thread> ptPlayer;
  if (FLAGS_load_input_option == 0) {
    swift_vio::Subscriber subscriber(nh, vioSystem, vio_parameters_reader);
    ros::Rate rate(parameters.sensors_information.cameraRate);
    while (ros::ok()) {
      ros::spinOnce();
      vioSystem->display();
      rate.sleep();
    }
  } else {
    pPlayer.reset(new swift_vio::Player(vioSystem, parameters));
    ptPlayer.reset(
        new std::thread(&swift_vio::Player::Run, std::ref(*pPlayer)));

    ros::Rate rate(parameters.sensors_information.cameraRate);
    while (!pPlayer->mbFinished) {
      vioSystem->display();
      rate.sleep();
    }
    ptPlayer->join();
    std::this_thread::sleep_for(
        std::chrono::seconds(5)); // in case the optimizer lags
  }

  std::string filename = swift_vio::removeTrailingSlash(FLAGS_output_dir) +
                         "/feature_statistics.txt";
  vioSystem->saveStatistics(filename);
  return 0;
}
