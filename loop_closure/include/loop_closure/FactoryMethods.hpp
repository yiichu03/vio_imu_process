#ifndef INCLUDE_SWIFT_VIO_FACTORY_METHODS_HPP_
#define INCLUDE_SWIFT_VIO_FACTORY_METHODS_HPP_

#include <loop_closure/LoopClosureDetectorParams.h>
#include <loop_closure/LoopClosureMethod.hpp>

namespace swift_vio {
/**
 * @brief createLoopClosureMethod
 * @param lcParams
 * @return
 */
std::shared_ptr<LoopClosureMethod> createLoopClosureMethod(
    std::shared_ptr<LoopClosureDetectorParams> lcParams);

}  // namespace swift_vio

#endif // INCLUDE_SWIFT_VIO_FACTORY_METHODS_HPP_
