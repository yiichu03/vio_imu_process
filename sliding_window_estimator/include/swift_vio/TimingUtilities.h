#ifndef TIMINGUTILITIES_H
#define TIMINGUTILITIES_H

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>

namespace swift_vio {
class TimingUtilities
{
public:
  TimingUtilities();

  static okvis::Time
  computeImuDataWaitUntilTime(std::shared_ptr<const CameraRig> cameraSystem,
                              ConstMultiFramePtr frame,
                              okvis::Duration temporal_overlap);
  static okvis::Time
  computeStateTimestamp(const CameraRig &cameraSystem,
                        ConstMultiFramePtr frame);
};
}  // namespace swift_vio
#endif // TIMINGUTILITIES_H
