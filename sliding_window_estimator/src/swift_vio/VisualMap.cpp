#include "swift_vio/VisualMap.h"

namespace swift_vio {
VisualMap::VisualMap() {

}

void VisualMap::getSubmap(okvis::kinematics::Transformation /*queryPose*/,
                          VisualSubmap */*submap*/) {}

void VisualMap::addKeyframeAndLandmarks(std::shared_ptr<const VisualSubmap> /*submap*/) {}

}  // namespace swift_vio
