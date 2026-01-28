
#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include <okvis/Parameters.hpp>
#include "../test/swift_vio/MockVioSystem.hpp"
#include "test/test_config.h"

#include "io_wrap/Player.hpp"

TEST(Player, VioDatasetPlayer) {
  std::shared_ptr<swift_vio::VioSystemBase> mvi(new swift_vio::MockVioSystem());
  okvis::VioParameters parameters;

  swift_vio::InputData input;
  swift_vio::InitialNavState initialState;
  swift_vio::FrontendOptions frontendOptions;
  okvis::SensorsInformation sensors_information;

  input.videoFile = std::string(DATASET_PATH) + "/honorv10/movie.mp4";
  input.timeFile = std::string(DATASET_PATH) + "/honorv10/frame_timestamps.txt";
  input.imageFolder = "";
  input.imuFile = std::string(DATASET_PATH) + "/honorv10/gyro_accel.csv";

  frontendOptions.useMedianFilter = false;

  sensors_information.cameraRate = 20;

  parameters.input = input;
  parameters.initialState = initialState;
  parameters.frontendOptions = frontendOptions;
  parameters.sensors_information = sensors_information;

  ros::Time::init();
  swift_vio::Player player(mvi, parameters);
  std::thread playerThread(&swift_vio::Player::Run, std::ref(player));
  playerThread.join();

  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "Finished processing the dataset at " << input.videoFile << "." << std::endl;
}

TEST(Player, parseCommaSeparatedTopics) {
  {
    std::string topics = "/cam0/image_raw,/cam1/image_raw";
    std::vector<std::string> expected_list{"/cam0/image_raw",
                                           "/cam1/image_raw"};
    std::vector<std::string> topic_list =
        swift_vio::parseCommaSeparatedTopics(topics);
    for (size_t i = 0; i < expected_list.size(); ++i) {
      EXPECT_EQ(expected_list[i], topic_list[i]);
    }
  }
  {
    std::string topics = "/cam0/image_raw,";
    std::vector<std::string> expected_list{"/cam0/image_raw"};
    std::vector<std::string> topic_list =
        swift_vio::parseCommaSeparatedTopics(topics);
    for (size_t i = 0; i < expected_list.size(); ++i) {
      EXPECT_EQ(expected_list[i], topic_list[i]);
    }
  }
}
