#include "swift_vio/TimingUtilities.h"

namespace swift_vio {
TimingUtilities::TimingUtilities() {}

okvis::Time
TimingUtilities::computeImuDataWaitUntilTime(std::shared_ptr<const CameraRig> cameraSystem,
                            ConstMultiFramePtr frame,
                            okvis::Duration temporal_overlap) {
  okvis::Time latestFrameTime(0);
  for (size_t i = 0u; i < cameraSystem->numCameras(); ++i) {
    latestFrameTime =
        std::max(latestFrameTime,
                 frame->timestamp(i) +
                     okvis::Duration(
                         cameraSystem->getImageDelay(i) +
                         cameraSystem->getReadoutTime(i) * 0.5));
  }
  return latestFrameTime + temporal_overlap;
}

okvis::Time TimingUtilities::computeStateTimestamp(const CameraRig &cameraSystem,
                                  ConstMultiFramePtr frame) {
  std::vector<okvis::Duration> predictedMidRowTimes(cameraSystem.numCameras());
  for (size_t i = 0u; i < cameraSystem.numCameras(); ++i) {
    predictedMidRowTimes[i] = frame->timestamp(i) - frame->timestamp(0) +
                              okvis::Duration(cameraSystem.getImageDelay(i));
  }

  okvis::Duration cumDuration = predictedMidRowTimes[0];
  for (size_t i = 1u; i < cameraSystem.numCameras(); ++i) {
    cumDuration += predictedMidRowTimes[i];
  }
  okvis::Duration delta;
  delta.fromNSec(cumDuration.toNSec() / static_cast<int64_t>(cameraSystem.numCameras()));
  return frame->timestamp(0) + delta;
}
}  // namespace swift_vio
