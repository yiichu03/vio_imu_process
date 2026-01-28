#ifndef FILTER_BASE_H
#define FILTER_BASE_H

#include <memory>

#include <Eigen/Core>

#include <okvis/ceres/ParameterBlock.hpp>
#include <okvis/timing/Timers.hpp>

namespace swift_vio {
struct StatePointerAndEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StatePointerAndEstimate(
      std::shared_ptr<const okvis::ceres::ParameterBlock> _parameterBlockPtr,
      const Eigen::VectorXd &_parameterEstimate)
      : parameterBlockPtr(_parameterBlockPtr),
        parameterEstimate(_parameterEstimate) {}
  std::shared_ptr<const okvis::ceres::ParameterBlock> parameterBlockPtr;
  Eigen::VectorXd parameterEstimate; // This can record an earlier estimate.
};

typedef std::vector<StatePointerAndEstimate,
                    Eigen::aligned_allocator<StatePointerAndEstimate>>
    StatePointerAndEstimateList;

class FilterBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FilterBase();

  virtual ~FilterBase();

  virtual void
  cloneFilterStates(StatePointerAndEstimateList *currentStates) const = 0;

  /**
   * @brief computeStackedJacobianAndResidual
   * @param T_H of size n x variableDim
   * @param r_q
   * @param R_q
   * @return number of residuals.
   */
  virtual int computeStackedJacobianAndResidual(
      Eigen::MatrixXd *T_H, Eigen::Matrix<double, Eigen::Dynamic, 1> *r_q,
      Eigen::MatrixXd *R_q);

  /**
   * @brief boxminusFromInput
   * @param refState
   * @param deltaX = refState - currentState
   */
  virtual void
  boxminusFromInput(const StatePointerAndEstimateList &refState,
                    Eigen::Matrix<double, Eigen::Dynamic, 1> *deltaX) const = 0;

  /**
   * @brief updateStates
   * @param deltaX currentState <-- currentState \boxplus deltaX.
   */
  virtual void
  updateStates(const Eigen::Matrix<double, Eigen::Dynamic, 1> &deltaX) = 0;

  void updateEkf(int variableStartIndex, int variableDim);

  void updateIekf(int variableStartIndex, int variableDim,
                  int maxNumIteration = 6, double updateVecNormTermination = 1e-4);

protected:
  Eigen::MatrixXd
      covariance_; ///< covariance of the error vector of all states, error is
                   ///< defined as \tilde{x} = x - \hat{x} except for rotations
  int numResiduals_;

  okvis::TimerSwitchable computeKalmanGainTimer;
  okvis::TimerSwitchable updateStatesTimer;
  okvis::TimerSwitchable updateCovarianceTimer;
};
} // namespace swift_vio

#endif // FILTER_BASE_H
