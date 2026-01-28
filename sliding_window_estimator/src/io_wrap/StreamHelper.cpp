#include "io_wrap/StreamHelper.hpp"
#include "swift_vio/CameraRig.hpp"
#include "swift_vio/ExtrinsicReps.hpp"
#include "swift_vio/imu/ImuModels.hpp"
#include "swift_vio/ProjectionIntrinsicReps.h"

DEFINE_string(datafile_separator, ",",
              "the separator used for a ASCII output file");

namespace swift_vio {
std::vector<std::string> parseCommaSeparatedTopics(const std::string& topic_list) {
  std::vector<std::string> topics;
  std::stringstream ss(topic_list);
  std::string topic;
  while (getline(ss, topic, ',')) {
      topics.push_back(topic);
  }
  return topics;
}

std::vector<bool> isCompressed(const std::vector<std::string> &image_topics) {
  std::vector<bool> compressedStatus;
  for (auto topic : image_topics) {
    compressedStatus.push_back(topic.find("compressed") != std::string::npos);
  }
  return compressedStatus;
}
}  // namespace swift_vio
