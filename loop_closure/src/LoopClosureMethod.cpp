#include <loop_closure/LoopClosureMethod.hpp>
namespace swift_vio {
LoopClosureMethod::LoopClosureMethod() : foundLoops_(0u) {}

LoopClosureMethod::~LoopClosureMethod() {}

bool LoopClosureMethod::detectLoop(
    std::shared_ptr<const swift_vio::LoopQueryKeyframeMessage> queryKeyframe,
    std::shared_ptr<KeyframeInDatabase>& /*queryKeyframeInDB*/,
    std::shared_ptr<LoopFrameAndMatches>& /*loopFrameAndMatches*/) {
  queryKeyframe.reset();
  return false;
}

bool LoopClosureMethod::addConstraintsAndOptimize(
    const KeyframeInDatabase& /*queryKeyframe*/,
    std::shared_ptr<const LoopFrameAndMatches> loopKeyframe,
    PgoResult& /*pgoResult*/) {
  if (loopKeyframe) {
    return true;
  } else {
    return false;
  }
}

std::shared_ptr<KeyframeInDatabase>
LoopClosureMethod::initializeKeyframeInDatabase(
    const swift_vio::LoopQueryKeyframeMessage& queryKeyframe) const {
  return queryKeyframe.toKeyframeInDatabase();
}
}  // namespace swift_vio
