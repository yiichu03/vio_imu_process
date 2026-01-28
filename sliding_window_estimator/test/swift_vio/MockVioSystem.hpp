
#ifndef MOCK_VIO_SYSTEM_HPP_
#define MOCK_VIO_SYSTEM_HPP_

#define GTEST_USE_OWN_TR1_TUPLE 0
#include "gtest/gtest.h"

#include <swift_vio/VioSystemBase.h>

namespace swift_vio {
class MockVioSystem : public VioSystemBase {
 public:
  bool addImage(const okvis::Time& /*stamp*/, size_t /*cameraIndex*/,
                const cv::Mat& /*image*/,
                const std::vector<cv::KeyPoint>* /*keypoints*/,
                bool* /*asKeyframe*/) {
    return true;
  }
  bool addKeypoints(const okvis::Time& /*stamp*/, size_t /*cameraIndex*/,
                    const std::vector<cv::KeyPoint>& /*keypoints*/,
                    const std::vector<uint64_t>& /*landmarkIds*/,
                    const cv::Mat& /*descriptors*/, bool* /*asKeyframe*/) {
    return true;
  }
  bool addImuMeasurement(const okvis::Time& /*stamp*/, const Eigen::Vector3d& /*alpha*/,
                         const Eigen::Vector3d& /*omega*/) {
    return true;
  }
};

}  // namespace okvis

#endif /* MOCK_VIO_SYSTEM_HPP_ */
