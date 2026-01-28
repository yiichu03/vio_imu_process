/**
 * @file   RiFixedLagSmoother.hpp
 * @brief  Declaration of RiFixedLagSmoother which wraps
 * gtsam::FixedLagSmoother for VIO with right invariant error formulation.
 * @author Jianzhu Huai
 */

#ifndef INCLUDE_GTSAM_RI_SLIDING_WINDOW_SMOOTHER_HPP_
#define INCLUDE_GTSAM_RI_SLIDING_WINDOW_SMOOTHER_HPP_

#include <gtsam/FixedLagSmoother.hpp>

#ifdef HAVE_GTSAM

namespace swift_vio {
/**
 * RiFixedLagSmoother builds upon gtsam FixedLagSmoother with right invariant errors.
 */
class RiFixedLagSmoother : public FixedLagSmoother {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor if a ceres map is already available.
   * @param mapPtr Shared pointer to ceres map.
   */
  RiFixedLagSmoother(const BackendParams& backendParams);

  virtual ~RiFixedLagSmoother();

  void addInitialPriorFactors() override;

  /**
   * @brief addImuValues add values for the last navigation state variable.
   */
  void addImuValues() override;

  /**
   * @brief addImuFactor add the IMU factor for the last navigation state variable.
   */
  void addImuFactor() override;

 protected:
  /**
   * @brief addLandmarkToGraph add a new landmark to the graph.
   * @param landmarkId
   */
  bool addLandmarkToGraph(uint64_t landmarkId, const Eigen::Vector4d& hpW) override;

  /**
   * @brief updateLandmarkInGraph add observations for an existing landmark.
   * @param landmarkId
   */
  void updateLandmarkInGraph(uint64_t landmarkId) override;


  /**
   * @brief addLandmarkSmartFactorToGraph Adds the smart factor of a landmark to
   * the graph for the first time.
   * @param lm_id
   */
  void addLandmarkSmartFactorToGraph(const LandmarkId& lm_id) final;

  /**
   * @brief updateLandmarkSmartFactorInGraph: Update the smart factor for
   * a landmark already in the graph.
   * @param lmk_id
   */
  void updateLandmarkSmartFactorInGraph(const LandmarkId& lmk_id) final;

  /**
   * @brief update state variables with FixedLagSmoother estimates.
   */
  void updateStates() override;


  /**
   * @brief gtsamMarginalCovariance compute marginal covariance blocks for the
   * lastest nav state and biases with gtsam::Marginals. It works with
   * Incremental and BatchFixedLagSmoother.
   * @param[in, out] cov
   * @return
   */
  bool gtsamMarginalCovariance(Eigen::MatrixXd* cov) const final;

};
}  // namespace swift_vio
#else
namespace swift_vio {
  typedef FixedLagSmoother RiFixedLagSmoother;
}  // namespace swift_vio
#endif  // #ifdef HAVE_GTSAM
#endif /* INCLUDE_GTSAM_RI_SLIDING_WINDOW_SMOOTHER_HPP_ */
