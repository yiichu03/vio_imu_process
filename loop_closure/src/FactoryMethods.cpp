#include <loop_closure/FactoryMethods.hpp>
#include <loop_closure/LoopClosureDetector.h>

namespace swift_vio {

std::shared_ptr<LoopClosureMethod> createLoopClosureMethod(
    std::shared_ptr<LoopClosureDetectorParams> lcParams) {
  LOG(INFO) << "Creating loop closure method " << lcParams->loop_closure_method_ << ".";
  switch (lcParams->loop_closure_method_) {
    case LoopClosureMethodType::OrbBoW:
      return std::shared_ptr<LoopClosureMethod>(
          new LoopClosureDetector(lcParams));
    default:
      return std::shared_ptr<LoopClosureMethod>(
          new LoopClosureMethod());
  }
}
}  // namespace swift_vio
