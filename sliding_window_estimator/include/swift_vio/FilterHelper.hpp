#ifndef INCLUDE_SWIFT_VIO_FILTER_HELPER_HPP_
#define INCLUDE_SWIFT_VIO_FILTER_HELPER_HPP_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

#include <okvis/ceres/Map.hpp>

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/imu/ImuRig.hpp>
#include <swift_vio/States.h>

namespace swift_vio {
class FilterHelper {
public:
  /**
   * @brief stackJacobianAndResidual fill every entry of H_o, r_o, and R_o by
   * components in vH_o, vr_o, vR_o.
   * @param vH_o
   * @param vr_o
   * @param vR_o
   * @param H_o allocated in advance.
   * @param r_o allocated in advance.
   * @param R_o allocated and initialized to zero in advance.
   */
  static void stackJacobianAndResidual(
      const std::vector<Eigen::MatrixXd,
                        Eigen::aligned_allocator<Eigen::MatrixXd>> &vH_o,
      const std::vector<Eigen::Matrix<double, -1, 1>,
                        Eigen::aligned_allocator<Eigen::Matrix<double, -1, 1>>>
          &vr_o,
      const std::vector<Eigen::MatrixXd,
                        Eigen::aligned_allocator<Eigen::MatrixXd>> &vR_o,
      Eigen::MatrixXd *H_o, Eigen::Matrix<double, -1, 1> *r_o,
      Eigen::MatrixXd *R_o);
  static void shrinkResidual(const Eigen::MatrixXd &H_o,
                             const Eigen::MatrixXd &r_o,
                             const Eigen::MatrixXd &R_o, Eigen::MatrixXd *T_H,
                             Eigen::Matrix<double, Eigen::Dynamic, 1> *r_q,
                             Eigen::MatrixXd *R_q);
  static int pruneSquareMatrix(int rm_state_start, int rm_state_end,
                               Eigen::MatrixXd *state_cov);

  static bool gatingTest(const Eigen::MatrixXd &H, const Eigen::VectorXd &r,
                         const Eigen::MatrixXd &R, const Eigen::MatrixXd &cov,
                         bool useMahalanobis);

  static Eigen::MatrixXd leftNullspaceWithRankCheck(const Eigen::MatrixXd &A,
                                                    int columnRankHint);

  static bool multiplyLeftNullspaceWithGivens(
      Eigen::MatrixXd *Hf, Eigen::MatrixXd *Hx,
      Eigen::Matrix<double, Eigen::Dynamic, 1> *residual, Eigen::MatrixXd *R,
      int columnRankHint);

  static void addCovForClonedNavState(Eigen::MatrixXd *covariance, size_t insertionIndex);

  static void initializeClonedNavStateCov(StateMap *stateMap, Eigen::MatrixXd *covariance, size_t clonedStateStartIdx, size_t gravityIdx);

  static void saveCovariance(const Eigen::MatrixXd &cov, const std::string &outputdir);

  static void initializeNavStateCov(const InitialNavState &initialNavState,
                                    const okvis::ImuParameters &gParams,
                                    Eigen::MatrixXd *covariance, size_t gWStartIndex);

  static void initializeImuStateCov(
      const std::vector<std::shared_ptr<okvis::ImuParameters>> &imuParametersVec,
      const ImuRig &imuRig,
      Eigen::MatrixXd *covariance, States *currentState, size_t imuParamStartIdx);

  static void initializeCameraStateCov(
      const okvis::CameraNoiseParametersVec &cameraNoiseParametersVec,
      const CameraRig &cameraRig, Eigen::MatrixXd *covariance,
      States *currentState, size_t camParamStartIdx);

  static void updateImuStates(const States &currentState,
                              const Eigen::VectorXd &deltaX,
                              const ImuRig &imuRig,
                              std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void updateCameraSensorStates(
      const States &currentState, const Eigen::VectorXd &deltaX,
      const CameraRig &cameraRig,
      const okvis::CameraNoiseParametersVec &cameraNoiseParametersVec,
      std::shared_ptr<okvis::ceres::Map> mapPtr);

  /**
   * @brief fill the error state map given states map, mainly set the indices of
   * variables in the covariance of the filters.
   * @param statesMap
   * @param errorStateMap
   */
  static void composeErrorStateMap(
      const StateMap &statesMap, const ImuRig &imuRig,
      const okvis::ImuParameters &imuParams, const CameraRig &cameraRig,
      const okvis::CameraNoiseParametersVec &cameraNoiseParametersVec,
      std::unordered_map<uint64_t, ErrorStateInfo> *errorStateMap);

  /**
   * @brief setOldErrorStateIndices
   * @param oldVarIdList the list of variables whose covariance has been computed by a smoother.
   * These variables are: IMU parameters, gravity parameters, camera parameters, nav states.
   * @param errorStateMap: the oldStartIndexInCov member in each element will be updated.
   * @param totalDim: total minimal dimension of variables in the list, for sanity check
   */
  static void setOldErrorStateIndices(
      const std::vector<uint64_t> &oldVarIdList,
      std::unordered_map<uint64_t, ErrorStateInfo> *errorStateMap,
      size_t totalDim);

  /**
   * @brief assign indices of variables in the covariance matrix to stateMap.
   * Note that for the sensor variables, we only update sensor info in the latest state for efficiency.
   * @param errorStateMap contains indices of variables in the cov matrix.
   * @param stateMap
   */
  static void assignCovarianceIndices(
      const std::unordered_map<uint64_t, ErrorStateInfo> &errorStateMap,
      StateMap *stateMap);
  /**
   * Chi-square thresholds based on the DOF of state (chi2(0.95,DOF))
   * degrees from 0, 1, 2, ...
   */
  static const double chi2_95percentile[];
};
}  // namespace swift_vio
#endif // INCLUDE_SWIFT_VIO_FILTER_HELPER_HPP_
