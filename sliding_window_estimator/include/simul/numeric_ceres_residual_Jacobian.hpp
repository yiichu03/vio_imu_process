#ifndef INCLUDE_SWIFT_VIO_NUMERIC_CERES_RESIDUAL_JACOBIAN_HPP_
#define INCLUDE_SWIFT_VIO_NUMERIC_CERES_RESIDUAL_JACOBIAN_HPP_

#include <sstream>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <okvis/ceres/ErrorInterface.hpp>
#include <okvis/ceres/ParameterBlock.hpp>

#include <swift_vio/PointLandmark.hpp>
#include <swift_vio/ceres/NormalVectorParameterBlock.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>

namespace simul {
#define ARE_MATRICES_CLOSE(ref, computed, tol)                                 \
  do {                                                                         \
    double diffRatio = (ref - computed)                                        \
                           .cwiseAbs()                                         \
                           .cwiseQuotient(Eigen::MatrixXd(                     \
                               (ref.cwiseAbs().array() > tol)                  \
                                   .select(ref.cwiseAbs().array(), 1)))        \
                           .maxCoeff();                                        \
    std::stringstream ss;                                                      \
    ss << "Two matrices " << #ref << " and " << #computed                      \
       << " largest elementwise difference is " + std::to_string(diffRatio)    \
       << "\n";                                                                \
    ss << #ref << "\n";                                                        \
    if (ref.cols() == 1)                                                       \
      ss << ref.transpose() << "\n";                                           \
    else                                                                       \
      ss << ref << "\n";                                                       \
    ss << #computed << "\n";                                                   \
    if (computed.cols() == 1)                                                  \
      ss << computed.transpose() << "\n";                                      \
    else                                                                       \
      ss << computed << "\n";                                                  \
    ss << '(' << #ref << '-' << #computed << ").cwiseAbs"                      \
       << "\n";                                                                \
    if (ref.cols() == 1)                                                       \
      ss << (ref - computed).cwiseAbs().transpose() << "\n";                   \
    else                                                                       \
      ss << (ref - computed).cwiseAbs() << "\n";                               \
    ss << "cwiseQuotient"                                                      \
       << "\n";                                                                \
    if (ref.cols() == 1)                                                       \
      ss << (ref - computed)                                                   \
                .cwiseAbs()                                                    \
                .cwiseQuotient(                                                \
                    Eigen::MatrixXd((ref.cwiseAbs().array() > tol)             \
                                        .select(ref.cwiseAbs().array(), 1)))   \
                .transpose();                                                  \
    else                                                                       \
      ss << (ref - computed)                                                   \
                .cwiseAbs()                                                    \
                .cwiseQuotient(                                                \
                    Eigen::MatrixXd((ref.cwiseAbs().array() > tol)             \
                                        .select(ref.cwiseAbs().array(), 1)));  \
    EXPECT_LT(diffRatio, tol) << ss.str();                                     \
  } while (0)

#define ARE_MATRICES_CLOSE_IN_VALUE(ref, computed, tol, res)                   \
  do {                                                                         \
    double diffAbs = (ref - computed).cwiseAbs().maxCoeff();                   \
    std::stringstream ss;                                                           \
    ss << "Two matrices " << #ref << " and " << #computed                      \
       << " largest elementwise absolute difference is " +                     \
              std::to_string(diffAbs)                                          \
       << "\n";                                                                \
    ss << #ref << "\n";                                                        \
    if (ref.cols() == 1)                                                       \
      ss << ref.transpose() << "\n";                                           \
    else                                                                       \
      ss << ref << "\n";                                                       \
    ss << #computed << "\n";                                                   \
    if (computed.cols() == 1)                                                  \
      ss << computed.transpose() << "\n";                                      \
    else                                                                       \
      ss << computed << "\n";                                                  \
    ss << '(' << #ref << '-' << #computed << ").cwiseAbs"                      \
       << "\n";                                                                \
    if (ref.cols() == 1)                                                       \
      ss << (ref - computed).cwiseAbs().transpose() << "\n";                   \
    else                                                                       \
      ss << (ref - computed).cwiseAbs() << "\n";                               \
    if (diffAbs > tol)                                                         \
      res = false;                                                             \
    else                                                                       \
      res = true;                                                              \
    EXPECT_LT(diffAbs, tol) << ss.str();                                       \
  } while (0)

#define ARE_MATRICES_CLOSE_IN_RATIO(ref, computed, tol)                        \
  do {                                                                         \
    double diffRatio = (ref - computed)                                        \
                           .cwiseAbs()                                         \
                           .cwiseQuotient(Eigen::MatrixXd(                     \
                               (ref.cwiseAbs().array() > tol)                  \
                                   .select(ref.cwiseAbs().array(), 1)))        \
                           .maxCoeff();                                        \
    std::stringstream ss;                                                           \
    ss << "Two matrices " << #ref << " and " << #computed                      \
       << " largest elementwise normalized difference is " +                   \
              std::to_string(diffRatio)                                        \
       << "\n";                                                                \
    ss << #ref << "\n";                                                        \
    if (ref.cols() == 1)                                                       \
      ss << ref.transpose() << "\n";                                           \
    else                                                                       \
      ss << ref << "\n";                                                       \
    ss << #computed << "\n";                                                   \
    if (computed.cols() == 1)                                                  \
      ss << computed.transpose() << "\n";                                      \
    else                                                                       \
      ss << computed << "\n";                                                  \
    ss << "cwiseQuotient"                                                      \
       << "\n";                                                                \
    if (ref.cols() == 1)                                                       \
      ss << (ref - computed)                                                   \
                .cwiseAbs()                                                    \
                .cwiseQuotient(                                                \
                    Eigen::MatrixXd((ref.cwiseAbs().array() > tol)             \
                                        .select(ref.cwiseAbs().array(), 1)))   \
                .transpose();                                                  \
    else                                                                       \
      ss << (ref - computed)                                                   \
                .cwiseAbs()                                                    \
                .cwiseQuotient(                                                \
                    Eigen::MatrixXd((ref.cwiseAbs().array() > tol)             \
                                        .select(ref.cwiseAbs().array(), 1)));  \
    EXPECT_LT(diffRatio, tol) << ss.str();                                     \
  } while (0)

/**
 * paramBlock->parameters() is a component of parameters.
 */
template <class JacEigenType = Eigen::Matrix<double, Eigen::Dynamic,
                                             Eigen::Dynamic, Eigen::RowMajor>>
void computeNumericJac(okvis::ceres::ParameterBlock &paramBlock,
                       const okvis::ceres::ErrorInterface *costFuncPtr,
                       double const *const *parameters,
                       const Eigen::VectorXd &residuals,
                       JacEigenType *jacNumeric) {
  const double epsilon = 1e-6;
  size_t paramDim = paramBlock.dimension();
  Eigen::VectorXd purturbedResiduals = residuals;
  for (size_t jack = 0; jack < paramDim; ++jack) {
    Eigen::VectorXd deltaVec = Eigen::VectorXd::Zero(paramDim);
    deltaVec[jack] = epsilon;
    Eigen::VectorXd currEst(paramDim);
    Eigen::VectorXd currAndDelta(paramDim);
    double *paramPtr = paramBlock.parameters();
    for (size_t k = 0; k < paramDim; ++k) {
      currEst[k] = paramPtr[k];
    }
    currAndDelta = currEst + deltaVec;
    paramBlock.setParameters(currAndDelta.data());
    costFuncPtr->EvaluateWithMinimalJacobians(
        parameters, purturbedResiduals.data(), NULL, NULL);
    jacNumeric->col(jack) = (purturbedResiduals - residuals) / epsilon;
    paramBlock.setParameters(currEst.data());
  }
}

void computeNumericJacPose(okvis::ceres::PoseParameterBlock &paramBlock,
                           const okvis::ceres::ErrorInterface *costFuncPtr,
                           double const *const *parameters,
                           const Eigen::VectorXd &residuals,
                           Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                         Eigen::RowMajor> *jacNumeric,
                           bool minimal);

void computeNumericJacPoint(
    swift_vio::PointLandmark &paramBlock,
    const ::ceres::LocalParameterization &localParameterization,
    const okvis::ceres::ErrorInterface *costFuncPtr,
    double const *const *parameters, const Eigen::VectorXd &residuals,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        *jacNumeric,
    bool minimal);

void computeNumericJacUnitVector(
    okvis::ceres::NormalVectorParameterBlock &paramBlock,
    const ::ceres::LocalParameterization *localParameterizationPtr,
    const okvis::ceres::ErrorInterface *costFuncPtr,
    double const *const *parameters, const Eigen::VectorXd &residuals,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        *jacNumeric);
} // namespace simul
#endif // INCLUDE_SWIFT_VIO_NUMERIC_CERES_RESIDUAL_JACOBIAN_HPP_
