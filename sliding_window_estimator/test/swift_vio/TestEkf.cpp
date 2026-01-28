
#include <gtest/gtest.h>

#include <random>

#include "swift_vio/memory.h"
#include <swift_vio/FilterBase.h>
#include <swift_vio/MultipleTransformPointJacobian.hpp>

#include <okvis/cameras/CameraBase.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/sophus_operators.hpp>

namespace swift_vio {
class SimPoseFilter : public FilterBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimPoseFilter()
      : poseParameterBlock(new okvis::ceres::PoseParameterBlock(
            okvis::kinematics::Transformation(), 1u, okvis::Time(0.0))) {}

  virtual ~SimPoseFilter() {}

  void
  cloneFilterStates(StatePointerAndEstimateList *currentStates) const override {
    currentStates->clear();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> T_WC(
        poseParameterBlock->parameters());
    currentStates->emplace_back(poseParameterBlock, T_WC);
  }

  int computeStackedJacobianAndResidual(
      Eigen::MatrixXd *T_H, Eigen::Matrix<double, Eigen::Dynamic, 1> *r_q,
      Eigen::MatrixXd *R_q) override {
    T_H->resize(observations.size() * 2, 6);
    r_q->resize(observations.size() * 2);
    R_q->resize(observations.size() * 2, observations.size() * 2);
    R_q->setIdentity();
    int validObservations = 0;
    okvis::kinematics::Transformation T_WC_est = poseParameterBlock->estimate();
    Eigen::AlignedVector<okvis::kinematics::Transformation> transformList{T_WC_est};

    for (size_t j = 0u; j < observations.size(); ++j) {
      Eigen::Vector2d imagePoint;
      Eigen::Matrix<double, 2, 4> dz_dpCtj;
      Eigen::Vector4d pCtj = T_WC_est.inverse() * observedCorners[j];
      okvis::cameras::CameraBase::ProjectionStatus status =
          cameraSystem->cameraGeometry(0)->projectHomogeneous(pCtj, &imagePoint,
                                                              &dz_dpCtj);
      if (status != okvis::cameras::CameraBase::ProjectionStatus::Successful) {
        continue;
      }
      r_q->segment<2>(2 * validObservations) = observations[j] - imagePoint;
      swift_vio::MultipleTransformPointJacobian mtpj(transformList, {-1},
                                                 observedCorners[j]);
      mtpj.computeJacobians();
      T_H->block<2, 6>(2 * validObservations, 0) = dz_dpCtj * mtpj.dp_dT(0u);
      (*R_q)(validObservations * 2, validObservations * 2) =
          std::pow(observationStddev[j][0], 2);
      (*R_q)(validObservations * 2 + 1, validObservations * 2 + 1) =
          std::pow(observationStddev[j][1], 2);
      validObservations++;
    }
    LOG(INFO) << "#Linearized observations " << validObservations;
    return validObservations;
  }

  void boxminusFromInput(
      const StatePointerAndEstimateList &refState,
      Eigen::Matrix<double, Eigen::Dynamic, 1> *deltaX) const override {
    int covDim = covariance_.rows();
    deltaX->resize(covDim, 1);
    Eigen::Matrix<double, 6, 1> delta_T_WC;
    swift_vio::PoseLocalParameterizationSimplified::ominus(
        refState.at(0).parameterBlockPtr->parameters(),
        refState.at(0).parameterEstimate.data(), delta_T_WC.data());
    deltaX->head<6>() = delta_T_WC;
  }

  void updateStates(
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &deltaX) override {
    okvis::kinematics::Transformation T_WC = poseParameterBlock->estimate();
    // In effect this amounts to PoseParameterBlock::plus().
    Eigen::Vector3d deltaAlpha = deltaX.segment<3>(3);
    Eigen::Quaterniond deltaq = okvis::kinematics::expAndTheta(deltaAlpha);
    T_WC = okvis::kinematics::Transformation(T_WC.r() + deltaX.head<3>(),
                                      deltaq * T_WC.q());
    poseParameterBlock->setEstimate(T_WC);
  }

  void setPoseEstimate(const okvis::kinematics::Transformation &T_WC) {
    poseParameterBlock->setEstimate(T_WC);
  }

  void setCovariance(const Eigen::Matrix<double, 6, 6> &covPose) {
    covariance_ = covPose;
  }

  const Eigen::MatrixXd &covariance() const { return covariance_; }

  okvis::kinematics::Transformation estimate() const {
    return poseParameterBlock->estimate();
  }

  void
  setCameraSystem(std::shared_ptr<okvis::cameras::NCameraSystem> cameraSystem) {
    this->cameraSystem = cameraSystem;
  }

  void
  setObservations(const Eigen::AlignedVector<Eigen::Vector4d> &observedCorners,
                  const Eigen::AlignedVector<Eigen::Vector2d> &observations,
                  const Eigen::AlignedVector<Eigen::Vector2d> &observationStddev) {
    this->observedCorners = observedCorners;
    this->observations = observations;
    this->observationStddev = observationStddev;
  }

  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock;
  std::shared_ptr<okvis::cameras::NCameraSystem> cameraSystem;

  Eigen::AlignedVector<Eigen::Vector4d> observedCorners;
  Eigen::AlignedVector<Eigen::Vector2d> observations;
  Eigen::AlignedVector<Eigen::Vector2d> observationStddev;
};
} // namespace swift_vio

class FilterBaseTest : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  FilterBaseTest() {

    corners.resize(cols * rows);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        corners[i * cols + j] = Eigen::Vector4d(i * spacing, j * spacing, 0, 1);
      }
    }

    std::shared_ptr<okvis::kinematics::Transformation> T_SC_0(
        new okvis::kinematics::Transformation());
    std::shared_ptr<okvis::cameras::CameraBase> cameraGeometry0(
        okvis::cameras::PinholeCamera<
            okvis::cameras::EquidistantDistortion>::createTestObject());
    cameraSystem.reset(new okvis::cameras::NCameraSystem);
    cameraSystem->addCamera(
        T_SC_0, cameraGeometry0,
        okvis::cameras::DistortionType::Equidistant);
    filter.setCameraSystem(cameraSystem);
  }

  void SetUp() override {}

  void simulateObservations(const okvis::kinematics::Transformation &T_WS) {
    T_WS_ref = T_WS;
    observations.clear();
    observations.reserve(corners.size());
    observedCorners.clear();
    observedCorners.reserve(corners.size());

    size_t camId = 0u;
    for (size_t j = 0u; j < corners.size(); ++j) {
      Eigen::Vector2d projection;
      Eigen::Vector4d point_C = cameraSystem->T_SC(camId)->inverse() *
                                T_WS_ref.inverse() * corners[j];
      okvis::cameras::CameraBase::ProjectionStatus status =
          cameraSystem->cameraGeometry(camId)->projectHomogeneous(point_C,
                                                                  &projection);
      if (status == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
        observations.emplace_back(projection);
        observedCorners.emplace_back(corners[j]);
      }
    }
  }

  void setupFilter(bool addNoise) {
    T_WS_init = T_WS_ref;
    T_WS_init.oplus(Eigen::Matrix<double, 6, 1>::Random() * 0.2);

    filter.setPoseEstimate(T_WS_init);
    double sigmaTranslation = 0.2;
    double sigmaRotation = 0.2;
    Eigen::Matrix<double, 6, 1> covDiagonal;
    covDiagonal.head<3>().setConstant(sigmaTranslation);
    covDiagonal.tail<3>().setConstant(sigmaRotation);
    covInitial = covDiagonal.asDiagonal();
    filter.setCovariance(covInitial);
    if (addNoise) {
      observationStddev.clear();
      observationStddev.reserve(corners.size());
      for (size_t j = 0u; j < observations.size(); ++j) {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> d{0, imageNoiseStd};
        observations[j][0] += d(gen);
        observations[j][1] += d(gen);
        observationStddev.emplace_back(
            Eigen::Vector2d(imageNoiseStd, imageNoiseStd));
      }
    } else {
      for (size_t j = 0u; j < observations.size(); ++j) {
        observationStddev.emplace_back(
            Eigen::Vector2d(imageNoiseStd * 0.1, imageNoiseStd * 0.1));
      }
    }
    filter.setObservations(observedCorners, observations, observationStddev);
  }

  swift_vio::SimPoseFilter filter;

  okvis::kinematics::Transformation T_WS_ref;
  int cols = 6;
  int rows = 7;
  double spacing = 0.06;
  double imageNoiseStd = 1.0;
  Eigen::AlignedVector<Eigen::Vector4d> corners;
  std::shared_ptr<okvis::cameras::NCameraSystem> cameraSystem;

  Eigen::AlignedVector<Eigen::Vector2d> observations;
  Eigen::AlignedVector<Eigen::Vector2d> observationStddev;
  Eigen::AlignedVector<Eigen::Vector4d> observedCorners;

  okvis::kinematics::Transformation T_WS_init;
  Eigen::Matrix<double, 6, 6> covInitial;
};

inline void isRotationClose(const okvis::kinematics::Transformation &T_expected,
                            const okvis::kinematics::Transformation &T_actual,
                            double tol) {

}

inline void
isTranslationClose(const okvis::kinematics::Transformation &T_expected,
                   const okvis::kinematics::Transformation &T_actual,
                   double tol) {

}

TEST_F(FilterBaseTest, EkfUpdate) {
  okvis::kinematics::Transformation T_shift(
      Eigen::Vector3d(0.3, 0.4, 2),
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d R_WC_canon;
  R_WC_canon << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  okvis::kinematics::Transformation T_WC_canon(
      Eigen::Vector3d(spacing * rows * 0.5, spacing * cols * 0.5, 0),
      Eigen::Quaterniond(R_WC_canon));
  simulateObservations(T_shift * T_WC_canon);
  setupFilter(false);
  filter.updateEkf(0, 6);

  EXPECT_LT((T_WS_ref.q().coeffs() - filter.estimate().q().coeffs())
                .lpNorm<Eigen::Infinity>(),
            1e-2)
      << "T_expected q:" << T_WS_ref.q().coeffs().transpose()
      << "\nT_estimated q:" << filter.estimate().q().coeffs().transpose() 
      << "\nInitial q:" << T_WS_init.q().coeffs().transpose();

  EXPECT_LT((T_WS_ref.r() - filter.estimate().r()).lpNorm<Eigen::Infinity>(), 1e-1)
      << "T_expected r:" << T_WS_ref.r().transpose()
      << "\nT_estimated r:" << filter.estimate().r().transpose()
      << "\nInitial r:" << T_WS_init.r().transpose()
      << "\nInitial cov\n"
      << covInitial << "\nFiltered cov\n"
      << filter.covariance();
}

TEST_F(FilterBaseTest, IekfUpdate) {
  okvis::kinematics::Transformation T_shift(
      Eigen::Vector3d(0.3, 0.4, 2),
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d R_WC_canon;
  R_WC_canon << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  okvis::kinematics::Transformation T_WC_canon(
      Eigen::Vector3d(spacing * rows * 0.5, spacing * cols * 0.5, 0),
      Eigen::Quaterniond(R_WC_canon));
  simulateObservations(T_shift * T_WC_canon);
  setupFilter(false);
  filter.updateIekf(0, 6);

  EXPECT_LT((T_WS_ref.q().coeffs() - filter.estimate().q().coeffs())
                .lpNorm<Eigen::Infinity>(),
            1e-2)
      << "T_expected q:" << T_WS_ref.q().coeffs().transpose()
      << "\nT_estimated q:" << filter.estimate().q().coeffs().transpose()
      << "\nInitial q:" << T_WS_init.q().coeffs().transpose();

  EXPECT_LT((T_WS_ref.r() - filter.estimate().r()).lpNorm<Eigen::Infinity>(), 1e-2)
      << "T_expected r:" << T_WS_ref.r().transpose()
      << "\nT_estimated r:" << filter.estimate().r().transpose()
      << "\nInitial cov\n" << covInitial << "\nFiltered cov\n"
      << filter.covariance();
}
