#ifndef IMUINITIALIZER_H
#define IMUINITIALIZER_H

#include <swift_vio/EstimatorBase.h>

namespace swift_vio {
/**
 * @brief ImuInitializer initialize a VIO system with only IMU data,
 * assuming the VIO system starts from stationary status.
 */
class ImuInitializer : public EstimatorBase
{
public:
  ImuInitializer(const okvis::EstimatorOptions &options);

  std::string typeInfo() const final {
    return "ImuInitializer";
  }

  bool getStateStd(Eigen::Matrix<double, Eigen::Dynamic, 1> *stateStd) const final;

  void estimate(std::shared_ptr<const VisualMatcherOutput> featureMatches) final;

private:

  size_t numNFrames_;
};

}  // namespace swift_vio
#endif // IMUINITIALIZER_H
