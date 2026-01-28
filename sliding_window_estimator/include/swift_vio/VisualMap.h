#ifndef VISUALMAP_H
#define VISUALMAP_H

#include <okvis/ceres/Map.hpp>
#include "swift_vio/States.h"
#include "swift_vio/imu/ImuRig.hpp"
#include "swift_vio/CameraRig.hpp"
#include "swift_vio/MapPoint.h"
#include "swift_vio/MultiFrame.hpp"
#include "swift_vio/memory.h"

namespace swift_vio {
struct VisualSubmap {
    std::unordered_map<uint64_t, ConstMultiFramePtr> multiFramePtrMap; ///< All keyframes
    PointMap landmarksMap; ///< All landmarks and observations in keyframes
    Eigen::AlignedVector<okvis::kinematics::Transformation> T_WS_list;
    std::unordered_map<uint64_t, std::vector<std::vector<uint64_t>>> nframeToLandmarkIds;
};

class VisualMap
{
public:
  VisualMap();

  /**
   * @brief query the visual map to get a submap of keyframes, landmarks based on the query pose
   * @param T_WB
   * @param[out] submap
   */
  void getSubmap(okvis::kinematics::Transformation T_WB, VisualSubmap *submap);

  /**
   * @brief addKeyframeAndLandmarks
   * @param[in] submap
   */
  void addKeyframeAndLandmarks(std::shared_ptr<const VisualSubmap> submap);

  void setCameraRig(std::shared_ptr<const CameraRig> cameraRig);

  void setImuRig(std::shared_ptr<const ImuRig> imuRig);

  // std::unordered_map<uint64_t, DBoW2::BowVector> featureVectorMap_; ///< Feature vectors for keyframes

  std::map<uint64_t, ConstMultiFramePtr> multiFramePtrMap_; ///< All keyframes

  PointMap landmarksMap_; ///< All landmarks and observations in keyframes

  Eigen::AlignedUnorderedMap<uint64_t, okvis::kinematics::Transformation> T_WB_map_;

  std::unordered_map<uint64_t, std::vector<std::vector<uint64_t>>> nframeToLandmarkIds;

  swift_vio::CameraRig cameraRig_;

  swift_vio::ImuRig imuRig_;
};

}  // namespace swift_vio

#endif // VISUALMAP_H
