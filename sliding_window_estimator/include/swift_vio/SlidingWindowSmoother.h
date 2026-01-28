/**
 * @file SlidingWindowSmoother.h
 */

#ifndef SLIDING_WINDOW_SMOOTHER_H
#define SLIDING_WINDOW_SMOOTHER_H

#include <swift_vio/EstimatorBase.h>
#include <okvis/ceres/MarginalizationError.hpp>

namespace swift_vio {
/**
 Sliding window smoother supporting full self-calibration.
 */
class SlidingWindowSmoother : public EstimatorBase
{
public:
  SlidingWindowSmoother(const okvis::EstimatorOptions &options);

  std::string typeInfo() const final {
    return "SlidingWindowSmoother";
  }

  void setPointLandmarkOptions(const swift_vio::PointLandmarkOptions& plOptions) final;

  void estimate(std::shared_ptr<const VisualMatcherOutput> featureMatches) final;

  /**
   * @brief add to the state vector landmarks from the database
   * @param databaseLandmarks
   */
  void addLandmarksFromDatabase(const Eigen::AlignedMap<uint64_t, PointAndVariance> &databaseLandmarks) final;

  bool applyMarginalizationStrategy(MapPointVector &removedPoints) final;

  std::vector<std::string> perturbationLabels() const final;

  /**
   * @brief get std. dev. of state for nav state (p,q,v), imu(bg ba), and optionally
   * imu augmented intrinsic parameters, camera extrinsic, intrinsic, td, tr.
   * @param stateStd
   * @return true if std. dev. of states are computed successfully.
   */
  bool getStateStd(Eigen::Matrix<double, Eigen::Dynamic, 1>* stateStd) const final;

  /**
   * @brief computeCovariance compute covariance by okvis marginalization module
   * which handles rank deficiency caused by low-disparity landmarks.
   * @param cov covariance of p_WS, q_WS, v_WS, b_g, b_a.
   * @return true if covariance is computed successfully, false otherwise.
   */
  bool computeCovariance(Eigen::MatrixXd* cov) const override;

  bool computeCovarianceSchur(Eigen::MatrixXd *cov) const;

  /**
   * @brief computeCovarianceCeres compute covariance by ceres::Covariance which
   * can handle rank deficiency if DENSE_SVD is used.
   * @param[out] cov covariance of p_WS, q_WS, v_WS, b_g, b_a.
   * @param[in] covAlgorithm SPARSE_QR or DENSE_SVD. DENSE_SVD is slow but
   * handles rank deficiency.
   * @return true if covariance is computed successfully, false otherwise.
   */
  bool
  computeCovarianceCeres(Eigen::MatrixXd *cov,
                         ::ceres::CovarianceAlgorithmType covAlgorithm) const;


  bool computeFullCovarianceCeres(
      Eigen::MatrixXd *cov, std::vector<uint64_t> *varIdList,
      ::ceres::CovarianceAlgorithmType covAlgorithm) const;

  void initializeFrom(std::shared_ptr<EstimatorBase> other, ConstMultiFramePtr currentNFrame) final;

private:
  std::shared_ptr<okvis::ceres::LocalParamizationAdditionalInterfaces> landmarkParamizationPtr_;
  std::shared_ptr< ::ceres::LossFunction> cauchyLossFunctionPtr_; ///< Cauchy loss.
  std::shared_ptr< ::ceres::LossFunction> huberLossFunctionPtr_; ///< Huber loss.

  std::shared_ptr<okvis::ceres::MarginalizationError> marginalizationErrorPtr_; ///< The marginalisation class
  ::ceres::ResidualBlockId marginalizationResidualId_; ///< Remembers the marginalisation object's Id

  size_t numNFrames_;
};

typedef SlidingWindowSmoother VioInitializer;

}  // namespace swift_vio
#endif // SLIDING_WINDOW_SMOOTHER_H
