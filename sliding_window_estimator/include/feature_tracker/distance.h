#ifndef DISTANCE_H
#define DISTANCE_H

#include <brisk/internal/hamming.h>
#include <functional>

namespace swift_vio {

typedef std::function<uint32_t(const unsigned char *descriptorA,
                               const unsigned char *descriptorB)>
    BinaryDescriptorDistanceCallback;

/// The ethz asl BRISK has a size 48 Bytes in contrast to OpenCV BRISK with a size of 64 Bytes.
/// The FREAK has a size of 64 Bytes.
uint32_t aslBriskDistance(const unsigned char *descriptorA,
                          const unsigned char *descriptorB);

uint32_t ocvBriskDistance(const unsigned char *descriptorA,
                          const unsigned char *descriptorB);

uint32_t ocvFreakDistance(const unsigned char *descriptorA,
                          const unsigned char *descriptorB);
} // namespace swift_vio
#endif // DISTANCE_H
