
#include "feature_tracker/distance.h"
namespace swift_vio {
uint32_t aslBriskDistance(const unsigned char *descriptorA,
                          const unsigned char *descriptorB) {
  return brisk::Hamming::PopcntofXORed(descriptorA, descriptorB, 3);
}

uint32_t ocvBriskDistance(const unsigned char *descriptorA,
                          const unsigned char *descriptorB) {
  return brisk::Hamming::PopcntofXORed(descriptorA, descriptorB, 4);
}

uint32_t ocvFreakDistance(const unsigned char *descriptorA,
                          const unsigned char *descriptorB) {
  return brisk::Hamming::PopcntofXORed(descriptorA, descriptorB, 4);
}
} // namespace swift_vio
