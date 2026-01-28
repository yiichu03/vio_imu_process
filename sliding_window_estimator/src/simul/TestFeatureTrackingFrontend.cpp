/**
 * @file TestFeatureTrackingFrontend.cpp
 * @brief This file tests FeatureTrackingFrontend.
 * @author Jianzhu Huai
 */

#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <okvis/Parameters.hpp>
#include <okvis/timing/Timer.hpp>

#include <swift_vio/EstimationHelper.h>
#include <swift_vio/FrameSynchronizer.hpp>
#include <swift_vio/VioFactoryMethods.hpp>
#include <swift_vio/VisualMap.h>

int loadEurocImageNames(const std::string &path,
                        std::vector<std::vector<std::string>> *image_names) {
  size_t numCameras = image_names->size();
  okvis::Time latest(0);
  int num_camera_images = 0;
  for (size_t i = 0; i < numCameras; ++i) {
    num_camera_images = 0;
    std::string folder(path + "/cam" + std::to_string(i) + "/data");

    for (auto it = boost::filesystem::directory_iterator(folder);
         it != boost::filesystem::directory_iterator(); it++) {
      if (!boost::filesystem::is_directory(
              it->path())) { // we eliminate directories
        num_camera_images++;
        image_names->at(i).push_back(it->path().filename().string());
      } else {
        continue;
      }
    }

    if (num_camera_images == 0) {
      LOG(ERROR) << "no images at " << folder;
      return 0;
    }

    LOG(INFO) << "No. cam " << i << " images: " << num_camera_images;
    // the filenames are not going to be sorted. So do this here
    std::sort(image_names->at(i).begin(), image_names->at(i).end());
  }
  return num_camera_images;
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true); // true to strip gflags
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  const std::string nodeName = "feature_tracking_frontend";

  std::string configFilename;
  std::string path;
  if (argc >= 3) {
    configFilename = argv[1];
    path = argv[2];
  } else {
    LOG(ERROR) << "Usage:" << argv[0]
               << " <config yml> "
                  "</ijrr_euroc_mav_dataset/machine_hall/MH_05_difficult/"
                  "MH_05_difficult/mav0/> [max-frames]";
    return 1;
  }
  int maxFrames = std::numeric_limits<int>::max();
  if (argc >= 4) {
    maxFrames = std::atoi(argv[3]);
  }

  okvis::VioParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);

  size_t numCameras = parameters.nCameraSystem.numCameras();
  // create feature tracking frontend
  std::shared_ptr<swift_vio::CameraFrontendBase> frontend =
      swift_vio::createFrontend(numCameras, parameters.frontendOptions);
  frontend->setCameraSystem(parameters.nCameraSystem);

  // load all images and timestamps
  std::vector<std::vector<std::string>> image_names(numCameras);
  int num_camera_images = loadEurocImageNames(path, &image_names);
  if (num_camera_images == 0) {
    return 2;
  }
  double fps = 20;
  swift_vio::FrameSynchronizer frameSynchronizer(numCameras, 0.2 / fps);

  okvis::timing::Timer extractionTimer("extraction timer", true);
  okvis::timing::Timer matchingTimer("matching timer", true);

  std::unordered_map<uint64_t, swift_vio::BareMultiFrame> nframeToLandmarkIds;
  swift_vio::PointMap landmarksMap;

  std::vector<std::vector<std::string>::iterator> cam_iterators(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    cam_iterators.at(i) = image_names.at(i).begin();
  }
  int counter = 0;
  okvis::Time t;
  bool finished = false;
  while (counter < maxFrames) {
    // check if at the end
    for (size_t i = 0; i < numCameras; ++i) {
      if (cam_iterators[i] == image_names[i].end()) {
        std::cout << std::endl
                  << "Finished images. " << std::endl;
        finished = true;
        break;
      }
    }
    if (finished) {
      break;
    }
    for (size_t i = 0; i < numCameras; ++i) {
      cv::Mat filtered = cv::imread(path + "/cam" + std::to_string(i) +
                                        "/data/" + *cam_iterators.at(i),
                                    cv::IMREAD_GRAYSCALE);
      std::string nanoseconds =
          cam_iterators.at(i)->substr(cam_iterators.at(i)->size() - 13, 9);
      std::string seconds =
          cam_iterators.at(i)->substr(0, cam_iterators.at(i)->size() - 13);
      t = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));

      std::shared_ptr<okvis::CameraMeasurement> frame =
          std::make_shared<okvis::CameraMeasurement>();

      frame->measurement.image = filtered;
      frame->timeStamp = t;
      frame->sensorId = i;

      std::shared_ptr<swift_vio::MultiFrame> multiFrame;
      multiFrame = frameSynchronizer.addNewFrame(frame);
      extractionTimer.start();
      frontend->detectAndDescribe(frame->sensorId, multiFrame);
      extractionTimer.stop();
      frameSynchronizer.detectionEndedForMultiFrame(multiFrame->id());

      if (frameSynchronizer.detectionCompletedForAllCameras(multiFrame->id())) {
        VLOG(1) << "Feature association for frame " << counter;
        okvis::ImuMeasurementDeque imuData;
        std::shared_ptr<swift_vio::VisualSubmap> vsubmap(
            new swift_vio::VisualSubmap());
        std::shared_ptr<swift_vio::VisualMatcherOutput> matcherOutput(
            new swift_vio::VisualMatcherOutput(multiFrame, imuData));
        matchingTimer.start();
        frontend->dataAssociation(multiFrame, vsubmap, matcherOutput.get());
        matchingTimer.stop();

        swift_vio::EstimationHelper::updateFeatureTracks(
            matcherOutput->featureTracks, matcherOutput->currentNFrame,
            &landmarksMap, &nframeToLandmarkIds);

        swift_vio::showMatchImages(landmarksMap, matcherOutput->currentNFrame, matcherOutput->closestKeyframe);

        // remove the old frames and their observations
        uint64_t oldestNFrameId = frontend->oldestNFrameId();
        swift_vio::EstimationHelper::removeOldFrames(oldestNFrameId, &nframeToLandmarkIds);
        swift_vio::EstimationHelper::removeOldObservations(oldestNFrameId, &landmarksMap);
      }
      cam_iterators[i]++;
    }
    ++counter;

    // display progress
    if (counter % 20 == 0) {
      std::cout << "\rProgress: "
                << int(double(counter) / double(num_camera_images) * 100)
                << "%  " << std::flush;
    }
  }
  // print timing statistics
  LOG(INFO) << okvis::timing::Timing::print();
  return 0;
}
