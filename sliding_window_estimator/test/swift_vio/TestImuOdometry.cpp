/**
 * @file TestImuOdometry.cpp
 * @brief source file for testing ImuOdometry methods.
 * @author Jianzhu Huai
 */

#include <gtest/gtest.h>

#include "gtsam/ImuFactorTestHelpers.h"

#include <swift_vio/imu/ImuOdometry.h>
#include <swift_vio/ImuOdometryLegacy.hpp>

#include <okvis/ceres/ImuError.hpp>
#include <okvis/timing/Timer.hpp>

#include "vio/Sample.h"
#include "vio/eigen_utils.h"

#include "sophus/se3.hpp"

#include "swift_vio/imu/CovPropConfig.hpp"
#include "eigen/gtest.hpp"

// compare RungeKutta and Euler forward and backward integration
class ImuOdometryBackwardTest : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuOdometryBackwardTest() :
      cpc(true, false),
      p_WS_W0(cpc.get_p_WS_W0()),
      q_WS0(cpc.get_q_WS0()),
      sb0(cpc.get_sb0()),
      T_WS0(cpc.get_p_WS_W0(), cpc.get_q_WS0()),
      imuParams(cpc.get_imu_params()) {
    std::cout << "States before forward integration:" << std::endl;
    print_p_q_sb(p_WS_W0, q_WS0, sb0);
    v_WS0 = sb0.head<3>();
    bias0 = sb0.tail<6>();
  }

  void SetUp() override {};

  typedef swift_vio::Imu_BG_BA_TG_TS_TA ImuModelT;
  CovPropConfig<ImuModelT> cpc;
  Eigen::Vector3d p_WS_W0;
  Eigen::Quaterniond q_WS0;
  okvis::SpeedAndBiases sb0;
  Eigen::Vector3d v_WS0;
  Eigen::Matrix<double, 6, 1> bias0;
  okvis::kinematics::Transformation T_WS0;
  okvis::ImuParameters imuParams;
};

TEST_F(ImuOdometryBackwardTest, BackwardRK) {
  ImuModelT imuModel;
  imuModel.updateParameters(bias0.data(), cpc.getImuExtraParamPtr());
  Eigen::Vector3d v_WS = v_WS0;
  Eigen::Matrix<double, 6, 1> bias = bias0;
  okvis::kinematics::Transformation T_WS = T_WS0;
  swift_vio::ImuOdometry::propagation_RungeKutta(
      cpc.get_imu_measurements(), cpc.get_imu_params(), T_WS, v_WS,
      imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time());
  swift_vio::ImuOdometry::propagationBackward_RungeKutta(
      cpc.get_imu_measurements(), imuParams, T_WS, v_WS,
      imuModel, cpc.get_meas_end_time(), cpc.get_meas_begin_time());

  Eigen::Vector3d p_WS_RoundTrip = T_WS.r();
  Eigen::Quaterniond q_WS_RoundTrip = T_WS.q();
  Eigen::Vector3d speedRoundTrip = v_WS;
  Eigen::Matrix<double, 6, 1> biasRoundTrip = bias;
  std::cout << "States after forward-backward RK integration:" << std::endl;
  print_p_q_v(p_WS_RoundTrip, q_WS_RoundTrip, speedRoundTrip);

  // Runge Kutta return to the starting position ?
  cpc.check_q_near(q_WS_RoundTrip, 1e-8);
  cpc.check_v_near(speedRoundTrip, 2e-2);
  cpc.check_p_near(p_WS_RoundTrip, 1.5);
}

TEST_F(ImuOdometryBackwardTest, StepwiseBackwardRK) {
  // forward
  okvis::ImuMeasurementDeque imuMeasurements = cpc.get_imu_measurements();
  auto iterLast = imuMeasurements.begin();
  Eigen::Vector3d v_WS = v_WS0;
  Eigen::Matrix<double, 6, 1> bias = bias0;
  Eigen::Vector3d p_WS_W = p_WS_W0;
  Eigen::Quaterniond q_WS = q_WS0;
  for (auto iter = imuMeasurements.begin(); iter != imuMeasurements.end();
       ++iter) {
    if (iter == imuMeasurements.begin()) continue;
    ImuModelT imuModel;
    imuModel.updateParameters(bias0.data(), cpc.getImuExtraParamPtr());
    swift_vio::ode::integrateOneStep_RungeKutta(
        iterLast->measurement.gyroscopes, iterLast->measurement.accelerometers,
        iter->measurement.gyroscopes, iter->measurement.accelerometers,
        cpc.get_g(), cpc.get_sigma_g_c(), cpc.get_sigma_a_c(),
        cpc.get_sigma_gw_c(), cpc.get_sigma_aw_c(), cpc.get_dt(), p_WS_W, q_WS,
        v_WS, imuModel);
    iterLast = iter;
  }

  std::cout << "States after forward RK integration:" << std::endl;
  print_p_q_v(p_WS_W, q_WS, v_WS);

  // backward
  auto iterRLast = imuMeasurements.rbegin();
  for (auto iterR = imuMeasurements.rbegin(); iterR != imuMeasurements.rend();
       ++iterR) {
    if (iterR == imuMeasurements.rbegin()) continue;
    ImuModelT imuModel;
    imuModel.updateParameters(bias0.data(), cpc.getImuExtraParamPtr());
    swift_vio::ode::integrateOneStepBackward_RungeKutta(
        iterR->measurement.gyroscopes, iterR->measurement.accelerometers,
        iterRLast->measurement.gyroscopes,
        iterRLast->measurement.accelerometers, cpc.get_g(), cpc.get_sigma_g_c(),
        cpc.get_sigma_a_c(), cpc.get_sigma_gw_c(), cpc.get_sigma_aw_c(),
        cpc.get_dt(), p_WS_W, q_WS, v_WS, imuModel);
    iterRLast = iterR;
  }

  std::cout << "States after backward RK integration:" << std::endl;
  print_p_q_v(p_WS_W, q_WS, v_WS);

  Eigen::Vector3d p_WS_RoundTrip = p_WS_W;
  Eigen::Quaterniond q_WS_RoundTrip = q_WS;

  cpc.check_q_near(q_WS_RoundTrip, 1e-8);
  cpc.check_v_near(v_WS, 2e-2);
  cpc.check_p_near(p_WS_RoundTrip, 1.5);
}

TEST_F(ImuOdometryBackwardTest, BackwardEuler) {
  Eigen::Vector3d v_WS = v_WS0;
  okvis::kinematics::Transformation T_WS = T_WS0;
  swift_vio::Imu_BG_BA_TG_TS_TA iem;
  iem.updateParameters(bias0.data(), cpc.getImuExtraParamPtr());
  swift_vio::ImuOdometry::propagation(cpc.get_imu_measurements(), cpc.get_imu_params(),
                           T_WS, v_WS, iem, cpc.get_meas_begin_time(),
                           cpc.get_meas_end_time());

  std::cout << "States after forward Euler integration:" << std::endl;
  print_p_q_v(T_WS.r(), T_WS.q(), v_WS);

  swift_vio::ImuOdometry::propagationBackward(
      cpc.get_imu_measurements(), imuParams, T_WS, v_WS, iem,
      cpc.get_meas_end_time(), cpc.get_meas_begin_time());
  Eigen::Vector3d p_WS_W = T_WS.r();
  Eigen::Quaterniond q_WS = T_WS.q();

  std::cout << "States after backward Euler integration:" << std::endl;
  print_p_q_v(T_WS.r(), T_WS.q(), v_WS);

  // Euler return to the starting position ?
  cpc.check_q_near(q_WS, 1e-6);
  cpc.check_v_near(v_WS, 0.1);
  cpc.check_p_near(p_WS_W, 5.0);
}

template<typename ImuModelT>
void IMUOdometryTrapezoidRule(
    const CovPropConfig<ImuModelT>& cpc,
    Eigen::Vector3d* p_WS_W1,
    Eigen::Quaterniond* q_WS1,
    Eigen::Vector3d *speed1,
    Eigen::Matrix<double, 6, 1> *bias1,
    Eigen::MatrixXd *covariance,
    Eigen::MatrixXd *jacobian,
    bool zeroVarianceForImuExtraParams = true,
    bool usePositionVelocityLin = true,
    bool verbose = false) {
  const Eigen::Vector3d p_WS_W0 = cpc.get_p_WS_W0();
  const Eigen::Quaterniond q_WS0 = cpc.get_q_WS0();

  const Eigen::Matrix<double, 6, 1> bias0 = cpc.get_bias0();
  const Eigen::Matrix<double, ImuModelT::kAugmentedMinDim, 1>& vTgTsTa = cpc.getImuExtraParams();
  const okvis::ImuMeasurementDeque& imuMeasurements = cpc.get_imu_measurements();
  const okvis::ImuParameters& imuParams = cpc.get_imu_params();

  okvis::kinematics::Transformation T_WS(p_WS_W0, q_WS0);
  Eigen::Vector3d v_WS = cpc.get_v_WS0();
  Eigen::Matrix<double, 6, 1> bias = cpc.get_bias0();
  okvis::timing::Timer okvisTimer("okvis", false);

  *covariance = Eigen::Matrix<
      double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim,
      ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim>::Identity();
  if (zeroVarianceForImuExtraParams) {
    covariance
        ->template bottomRightCorner<ImuModelT::kAugmentedMinDim,
                                     ImuModelT::kAugmentedMinDim>()
        .setZero();
  }
  *jacobian = Eigen::Matrix<
      double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim,
      ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim>::Identity();

  int numUsedImuMeasurements = 0;
  Eigen::Matrix<double, 6, 1> positionVelocityLin;
  positionVelocityLin << T_WS.r(), v_WS;
  switch (ImuModelT::kModelId) {
  case swift_vio::Imu_BG_BA_MG_TS_MA::kModelId: {
      swift_vio::Imu_BG_BA_MG_TS_MA imuModel;
      imuModel.updateParameters(bias.data(), vTgTsTa.data());
      numUsedImuMeasurements = swift_vio::ImuOdometry::propagation(
          imuMeasurements, imuParams, T_WS, v_WS, imuModel,
          imuMeasurements.begin()->timeStamp, imuMeasurements.rbegin()->timeStamp,
          covariance, jacobian,
          usePositionVelocityLin ? &positionVelocityLin : nullptr);
      }
      break;
  case swift_vio::Imu_BG_BA_TG_TS_TA::kModelId:
  default: {
    swift_vio::Imu_BG_BA_TG_TS_TA imuModel;
    imuModel.updateParameters(bias.data(), vTgTsTa.data());
    numUsedImuMeasurements = swift_vio::ImuOdometry::propagation(
        imuMeasurements, imuParams, T_WS, v_WS, imuModel,
        imuMeasurements.begin()->timeStamp, imuMeasurements.rbegin()->timeStamp,
        covariance, jacobian,
        usePositionVelocityLin ? &positionVelocityLin : nullptr);
    }
    break;
  }

  double timeElapsed = okvisTimer.stop();

  *p_WS_W1 = T_WS.r();
  *q_WS1 = T_WS.q();
  *speed1 = v_WS;
  *bias1 = bias0;
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> covDiagonal = covariance->diagonal();
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> sqrtCovDiagonal = covDiagonal.cwiseSqrt();
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> jacobianFirstRow = jacobian->row(0);

  std::cout
      << "\n>>> Time used by huai trapezoid rule forward propagtion with covariance "
      << timeElapsed << std::endl;
  if (verbose) {
    std::cout << "numUsedMeas " << numUsedImuMeasurements << " totalMeas "
              << (int)imuMeasurements.size() << std::endl;
    std::cout << "q_WS " << q_WS1->w() << " " << q_WS1->x() << " " << q_WS1->y()
              << " " << q_WS1->z() << std::endl;
    std::cout << "p_WS_W " << p_WS_W1->transpose() << std::endl;
    std::cout << "speed " << speed1->transpose() << std::endl;
    std::cout << "cov diagonal sqrt " << std::endl;
    std::cout << sqrtCovDiagonal.transpose() << std::endl;
    std::cout << "Jacobian diagonal " << std::endl;
    std::cout << vio::superdiagonal(*jacobian).transpose() << std::endl;
    std::cout << "Jacobian first row " << std::endl;
    std::cout << jacobianFirstRow.transpose() << std::endl;
  }
}

TEST(ImuOdometry, positionVelocityLinMGTSMA) {
  srand((unsigned int)time(0));
  typedef swift_vio::Imu_BG_BA_MG_TS_MA ImuModelT;
  CovPropConfig<ImuModelT> cpc(false, true);
  bool zeroVarForImuExtraParams = true;
  bool verbose = false;

  bool usePositionVelocityLin = false;
  Eigen::Vector3d p_WS;
  Eigen::Quaterniond q_WS;
  Eigen::Vector3d v_WS;
  Eigen::Matrix<double, 6, 1> bias;

  Eigen::MatrixXd cov;
  Eigen::MatrixXd jacobian;
  IMUOdometryTrapezoidRule(cpc, &p_WS, &q_WS, &v_WS, &bias, &cov, &jacobian,
                           zeroVarForImuExtraParams, usePositionVelocityLin,
                           verbose);

  usePositionVelocityLin = true;
  Eigen::Vector3d p_WS_lin;
  Eigen::Quaterniond q_WS_lin;
  okvis::SpeedAndBiases speedAndBiasLin;
  Eigen::Vector3d v_WS_lin;
  Eigen::Matrix<double, 6, 1> biasLin;

  Eigen::MatrixXd covLin;
  Eigen::MatrixXd jacobianLin;
  IMUOdometryTrapezoidRule(cpc, &p_WS_lin, &q_WS_lin, &v_WS_lin, &biasLin, &covLin,
                           &jacobianLin, zeroVarForImuExtraParams,
                           usePositionVelocityLin, verbose);

  expectNearAbsRel(p_WS, p_WS_lin, 1e-4, 1e-4);
  check_q_near(q_WS, q_WS_lin, 1e-6);
  expectNearAbsRel(v_WS, v_WS_lin, 1e-4, 1e-4);
  expectNearAbsRel(cov, covLin, 1e-4, 1e-4);
  expectNearAbsRel(jacobian, jacobianLin, 1e-4, 1e-4);
}

TEST(ImuOdometry, positionVelocityLin) {
  srand((unsigned int)time(0));
  typedef swift_vio::Imu_BG_BA_TG_TS_TA ImuModelT;
  CovPropConfig<ImuModelT> cpc(false, true);
  bool zeroVarForImuExtraParams = true;
  bool verbose = false;

  bool usePositionVelocityLin = false;
  Eigen::Vector3d p_WS;
  Eigen::Quaterniond q_WS;
  Eigen::Vector3d v_WS;
  Eigen::Matrix<double, 6, 1> bias;

  Eigen::MatrixXd cov;
  Eigen::MatrixXd jacobian;
  IMUOdometryTrapezoidRule(cpc, &p_WS, &q_WS, &v_WS, &bias, &cov, &jacobian,
                           zeroVarForImuExtraParams, usePositionVelocityLin,
                           verbose);

  usePositionVelocityLin = true;
  Eigen::Vector3d p_WS_lin;
  Eigen::Quaterniond q_WS_lin;

  Eigen::Vector3d v_WS_lin;
  Eigen::Matrix<double, 6, 1> biasLin;

  Eigen::MatrixXd covLin;
  Eigen::MatrixXd jacobianLin;
  IMUOdometryTrapezoidRule(cpc, &p_WS_lin, &q_WS_lin, &v_WS_lin, &biasLin, &covLin,
                           &jacobianLin, zeroVarForImuExtraParams,
                           usePositionVelocityLin, verbose);

  expectNearAbsRel(p_WS, p_WS_lin, 1e-4, 1e-4);
  check_q_near(q_WS, q_WS_lin, 1e-6);
  expectNearAbsRel(v_WS, v_WS_lin, 1e-4, 1e-4);
  expectNearAbsRel(cov, covLin, 1e-4, 1e-4);
  expectNearAbsRel(jacobian, jacobianLin, 1e-4, 1e-4);
}

TEST(ImuOdometry, StatePropagationMGTSMA) {
  bool verbose = true;  // print the covariance and jacobian results

  srand((unsigned int)time(0));
  typedef swift_vio::Imu_BG_BA_MG_TS_MA ImuModelT;
  CovPropConfig<ImuModelT> cpc(false, false);

  /// method 1: RK4
  Eigen::Vector3d p_WS_W = cpc.get_p_WS_W0();
  Eigen::Quaterniond q_WS = cpc.get_q_WS0();
  Eigen::Matrix<double, 3, 1> v_WS = cpc.get_v_WS0();
  Eigen::Matrix<double, 6, 1> bias = cpc.get_bias0();

  okvis::timing::Timer RK4Timer("RK4", false);
  swift_vio::Imu_BG_BA_MG_TS_MA imuModel;
  imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
  okvis::kinematics::Transformation T_WS(p_WS_W, q_WS);
  int numUsedImuMeasurements = swift_vio::ImuOdometry::propagation_RungeKutta(
      cpc.get_imu_measurements(), cpc.get_imu_params(), T_WS, v_WS,
      imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time());
  double timeElapsed = RK4Timer.stop();

  Eigen::Vector3d p_WS_W_RK4 = T_WS.r();
  Eigen::Quaterniond q_WS_RK4 = T_WS.q();
  Eigen::Vector3d speedRK4 = v_WS;
  Eigen::Matrix<double, 6, 1> biasRK4 = bias;

  std::cout << "\n>>> Time used by RK4 forward state propagtion "
            << timeElapsed << std::endl;

  if (verbose) {
    std::cout << "numUsedMeas " << numUsedImuMeasurements << " totalMeas "
                << (int)cpc.get_imu_measurements().size() << std::endl;
    std::cout << "q_WS " << q_WS_RK4.w() << " " << q_WS_RK4.x() << " " << q_WS_RK4.y()
              << " " << q_WS_RK4.z() << std::endl;
    std::cout << "p_WS_W " << p_WS_W_RK4.transpose() << std::endl;
    std::cout << "speed " << speedRK4.transpose() << " " << biasRK4.transpose() << std::endl;
  }

  /// method 2 : propagation by using trapezoid rules.
  Eigen::Vector3d p_WS_Trapezoid;
  Eigen::Quaterniond q_WS_Trapezoid;
  Eigen::Vector3d speedTrapezoid = cpc.get_v_WS0();

  T_WS.set(cpc.get_p_WS_W0(), cpc.get_q_WS0());
  v_WS = cpc.get_v_WS0();
  okvis::timing::Timer timer("trapezoid", false);
  numUsedImuMeasurements = swift_vio::ImuOdometry::propagation(
      cpc.get_imu_measurements(), cpc.get_imu_params(), T_WS, v_WS, imuModel,
      cpc.get_meas_begin_time(), cpc.get_meas_end_time());
  p_WS_Trapezoid = T_WS.r();
  q_WS_Trapezoid = T_WS.q();
  speedTrapezoid = v_WS;
  timeElapsed = timer.stop();

  std::cout
      << "\n>>> Time used by trapezoid rule forward state propagtion "
      << timeElapsed << std::endl;
  if (verbose) {
    std::cout << "numUsedMeas " << numUsedImuMeasurements << " totalMeas "
              << (int)cpc.get_imu_measurements().size() << std::endl;
    std::cout << "q_WS " << q_WS_Trapezoid.w() << " " << q_WS_Trapezoid.x() << " " << q_WS_Trapezoid.y()
              << " " << q_WS_Trapezoid.z() << std::endl;
    std::cout << "p_WS_W " << p_WS_Trapezoid.transpose() << std::endl;
    std::cout << "speed " << speedTrapezoid.transpose() << std::endl;
  }

  // RK4 vs Trapezoid state
  EXPECT_TRUE(std::fabs(q_WS_Trapezoid.w() - q_WS_RK4.w()) < 1e-2 &&
              std::fabs(q_WS_Trapezoid.x() - q_WS_RK4.x()) < 1e-2 &&
              std::fabs(q_WS_Trapezoid.y() - q_WS_RK4.y()) < 1e-2 &&
              std::fabs(q_WS_Trapezoid.z() - q_WS_RK4.z()) < 1e-2);
  EXPECT_LT((p_WS_W_RK4 - p_WS_Trapezoid).norm(), 50);
  EXPECT_LT((speedRK4 - speedTrapezoid).norm(), 5);
}

TEST(ImuOdometry, CovariancePropagationMGTSMA) {
  bool zeroVarForImuExtraParams = true;
  bool verbose = true;  // print the covariance and jacobian results

  srand((unsigned int)time(0));
  typedef swift_vio::Imu_BG_BA_MG_TS_MA ImuModelT;
  CovPropConfig<ImuModelT> cpc(false, true);

  /// method 1: RK4
  Eigen::Vector3d p_WS_W = cpc.get_p_WS_W0();
  Eigen::Quaterniond q_WS = cpc.get_q_WS0();
  Eigen::Vector3d v_WS = cpc.get_v_WS0();
  Eigen::Matrix<double, 6, 1> bias = cpc.get_bias0();
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim,
      ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim> covRK4;
  covRK4.setIdentity();
  if (zeroVarForImuExtraParams) {
    covRK4.bottomRightCorner<ImuModelT::kAugmentedMinDim, ImuModelT::kAugmentedMinDim>().setZero();
  }
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim,
      ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim> jacobianRK4;
  jacobianRK4.setIdentity();

  okvis::timing::Timer RK4Timer("RK4", false);
  const okvis::ImuMeasurementDeque& imuMeasurements = cpc.get_imu_measurements();
  auto iterLast = imuMeasurements.begin();
  for (auto iter = imuMeasurements.begin(); iter != imuMeasurements.end();
       ++iter) {
    if (iter == imuMeasurements.begin()) continue;
    ImuModelT imuModel;
    imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
    swift_vio::ode::integrateOneStep_RungeKutta(
        iterLast->measurement.gyroscopes, iterLast->measurement.accelerometers,
        iter->measurement.gyroscopes, iter->measurement.accelerometers,
        cpc.get_g(), cpc.get_sigma_g_c(), cpc.get_sigma_a_c(),
        cpc.get_sigma_gw_c(), cpc.get_sigma_aw_c(), cpc.get_dt(), p_WS_W, q_WS,
        v_WS, imuModel, &covRK4, &jacobianRK4);
    iterLast = iter;
  }
  double timeElapsed = RK4Timer.stop();

  Eigen::Vector3d p_WS_W_RK4 = p_WS_W;
  Eigen::Quaterniond q_WS_RK4 = q_WS;
  Eigen::Vector3d speedRK4 = v_WS;
  Eigen::Matrix<double,
                ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1>
      covDiagonal = covRK4.diagonal();
  Eigen::Matrix<double,
                ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1>
      sqrtCovDiagonalRK4 = covDiagonal.cwiseSqrt();
  Eigen::Matrix<double,
                ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1>
      jacobianFirstRowRK4 = jacobianRK4.row(0).transpose();
  std::cout << "\n>>> Time used by RK4 forward propagtion with covariance "
            << timeElapsed << std::endl;

  if (verbose) {
    std::cout << "q_WS " << q_WS_RK4.w() << " " << q_WS_RK4.x() << " " << q_WS_RK4.y()
              << " " << q_WS_RK4.z() << std::endl;
    std::cout << "p_WS_W " << p_WS_W_RK4.transpose() << std::endl;
    std::cout << "speed " << speedRK4.transpose() << std::endl;

    std::cout << "cov(r_s^w, \\phi^w, v_s^w, b_g, b_a, T_g, T_s, T_a), its "
                 "diagonal sqrt "
              << std::endl;
    std::cout << sqrtCovDiagonalRK4.transpose() << std::endl;
    std::cout << "Jacobian superdiagonal " << std::endl;
    std::cout << vio::superdiagonal(jacobianRK4).transpose() << std::endl;
    std::cout << "Jacobian first row " << std::endl;
    std::cout << jacobianFirstRowRK4.transpose() << std::endl;
  }

  /// method 2 : propagation by using trapezoid rules implemented with okvis error convention.
  Eigen::Vector3d p_WS_Trapezoid;
  Eigen::Quaterniond q_WS_Trapezoid;
  Eigen::Vector3d speedTrapezoid;
  Eigen::Matrix<double, 6, 1> biasTrapezoid;
  bool usePositionVelocityLin = true;
  Eigen::MatrixXd covTrapezoid;
  Eigen::MatrixXd jacobianTrapezoid;
  IMUOdometryTrapezoidRule(cpc, &p_WS_Trapezoid, &q_WS_Trapezoid,
                           &speedTrapezoid, &biasTrapezoid, &covTrapezoid,
                           &jacobianTrapezoid, zeroVarForImuExtraParams,
                           usePositionVelocityLin, verbose);
  Eigen::Matrix<double,
                ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1>
      sqrtCovDiagonalTrapezoid = covTrapezoid.diagonal().cwiseSqrt();
  Eigen::Matrix<double,
                ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1>
      jacobianFirstRowTrapezoid = jacobianTrapezoid.row(0);

  // RK4 vs Trapezoid state
  EXPECT_TRUE(std::fabs(q_WS_Trapezoid.w() - q_WS_RK4.w()) < 1e-5 &&
              std::fabs(q_WS_Trapezoid.x() - q_WS_RK4.x()) < 1e-5 &&
              std::fabs(q_WS_Trapezoid.y() - q_WS_RK4.y()) < 1e-5 &&
              std::fabs(q_WS_Trapezoid.z() - q_WS_RK4.z()) < 1e-5);
  EXPECT_LT((p_WS_W_RK4 - p_WS_Trapezoid).norm(), 50);
  EXPECT_LT((speedRK4 - speedTrapezoid).norm(), 2);

  if (verbose) {
    eigen::printTwoMatrices(jacobianRK4, jacobianTrapezoid,
                            {"jacobian RK4", "Jacobian Trapezoid"}, 0, 0, 9, 9);
    eigen::printTwoMatrices(jacobianRK4, jacobianTrapezoid,
                            {"jacobian RK4", "Jacobian Trapezoid"}, 0, 9, 9, 6);
    eigen::printTwoMatrices(jacobianRK4, jacobianTrapezoid,
                            {"jacobian RK4", "Jacobian Trapezoid"}, 0, 15, 9,
                            9);
    eigen::printTwoMatrices(jacobianRK4, jacobianTrapezoid,
                            {"jacobian RK4", "Jacobian Trapezoid"}, 0, 24, 9,
                            9);
    eigen::printTwoMatrices(jacobianRK4, jacobianTrapezoid,
                            {"jacobian RK4", "Jacobian Trapezoid"}, 0, 33, 9,
                            6);
    eigen::printTwoMatrices(covRK4, covTrapezoid, {"cov RK4", "cov Trapezoid"},
                            0, 0, 9, 9);
  }

  // RK4 vs Trapezoid covariance
  EXPECT_LT((sqrtCovDiagonalTrapezoid.head<3>() - sqrtCovDiagonalRK4.head<3>()).norm(), 300);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(3) - sqrtCovDiagonalRK4.segment<3>(3)).norm(),
            2e-2);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(6) - sqrtCovDiagonalRK4.segment<3>(6)).norm(),
            10);

  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(9) - sqrtCovDiagonalRK4.segment<3>(9)).norm(),
            1e-6);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(12) - sqrtCovDiagonalRK4.segment<3>(12)).norm(),
            1e-6);

  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<9>(15) - sqrtCovDiagonalRK4.segment<9>(15)).norm(),
            1e-6);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<9>(24) - sqrtCovDiagonalRK4.segment<9>(24)).norm(),
            1e-6);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<6>(33) - sqrtCovDiagonalRK4.segment<6>(33)).norm(),
            1e-6);

  // RK4 vs Trapezoid jacobian
  EXPECT_LT((jacobianFirstRowRK4.head<3>() - jacobianFirstRowTrapezoid.head<3>()).norm() /
                jacobianFirstRowRK4.head<3>().norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<6>(3) - jacobianFirstRowTrapezoid.segment<6>(3)).norm() /
                jacobianFirstRowRK4.segment<6>(3).norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<6>(9) - jacobianFirstRowTrapezoid.segment<6>(9)).norm() /
                jacobianFirstRowRK4.segment<6>(9).norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<9>(15) - jacobianFirstRowTrapezoid.segment<9>(15)).norm() /
                jacobianFirstRowRK4.segment<9>(15).norm(),
            5e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<9>(24) - jacobianFirstRowTrapezoid.segment<9>(24)).norm() /
                jacobianFirstRowRK4.segment<9>(24).norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<6>(33) - jacobianFirstRowTrapezoid.segment<6>(33)).norm() /
                jacobianFirstRowRK4.segment<6>(33).norm(),
            1e-3);
}

/// test and compare the propagation for both states and covariance by both the
/// classic RK4 and okvis's state transition method
TEST(ImuOdometry, CovariancePropagation) {
  // use positive variance for elements in Tg Ts Ta?
  bool zeroVarForImuExtraParams = true;
  bool verbose = true;  // print the covariance and jacobian results

  srand((unsigned int)time(0));
  typedef swift_vio::Imu_BG_BA_TG_TS_TA ImuModelT;
  CovPropConfig<ImuModelT> cpc(false, true);

  /// method 1: RK4
  Eigen::Vector3d p_WS_W = cpc.get_p_WS_W0();
  Eigen::Quaterniond q_WS = cpc.get_q_WS0();
  Eigen::Vector3d v_WS = cpc.get_v_WS0();
  Eigen::Matrix<double, 6, 1> bias = cpc.get_bias0();

  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim,
      ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim> covRK4;
  covRK4.setIdentity();
  if (zeroVarForImuExtraParams) {
    covRK4.bottomRightCorner<ImuModelT::kAugmentedMinDim, ImuModelT::kAugmentedMinDim>().setZero();
  }
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim,
      ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim> jacobianRK4;
  jacobianRK4.setIdentity();

  okvis::timing::Timer RK4Timer("RK4", false);
  const okvis::ImuMeasurementDeque& imuMeasurements = cpc.get_imu_measurements();
  auto iterLast = imuMeasurements.begin();
  for (auto iter = imuMeasurements.begin(); iter != imuMeasurements.end();
       ++iter) {
    if (iter == imuMeasurements.begin()) continue;
    ImuModelT imuModel;
    imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
    swift_vio::ode::integrateOneStep_RungeKutta(
        iterLast->measurement.gyroscopes, iterLast->measurement.accelerometers,
        iter->measurement.gyroscopes, iter->measurement.accelerometers,
        cpc.get_g(), cpc.get_sigma_g_c(), cpc.get_sigma_a_c(),
        cpc.get_sigma_gw_c(), cpc.get_sigma_aw_c(), cpc.get_dt(), p_WS_W, q_WS,
        v_WS, imuModel, &covRK4, &jacobianRK4);
    iterLast = iter;
  }
  double timeElapsed = RK4Timer.stop();

  Eigen::Vector3d p_WS_W_RK4 = p_WS_W;
  Eigen::Quaterniond q_WS_RK4 = q_WS;
  Eigen::Vector3d speedRK4 = v_WS;
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> covDiagonal = covRK4.diagonal();
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> sqrtCovDiagonalRK4 = covDiagonal.cwiseSqrt();
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> jacobianFirstRowRK4 = jacobianRK4.row(0).transpose();
  std::cout << "\n>>> Time used by RK4 forward propagtion with covariance "
            << timeElapsed << std::endl;

  if (verbose) {
    std::cout << "q_WS " << q_WS_RK4.w() << " " << q_WS_RK4.x() << " " << q_WS_RK4.y()
              << " " << q_WS_RK4.z() << std::endl;
    std::cout << "p_WS_W " << p_WS_W_RK4.transpose() << std::endl;
    std::cout << "speed " << speedRK4.transpose() << std::endl;

    std::cout << "cov(r_s^w, \\phi^w, v_s^w, b_g, b_a, T_g, T_s, T_a), its "
                 "diagonal sqrt "
              << std::endl;
    std::cout << sqrtCovDiagonalRK4.transpose() << std::endl;
    std::cout << "Jacobian superdiagonal " << std::endl;
    std::cout << vio::superdiagonal(jacobianRK4).transpose() << std::endl;
    std::cout << "Jacobian first row " << std::endl;
    std::cout << jacobianFirstRowRK4.transpose() << std::endl;
  }

  /// method 2 : propagation by using trapezoid rules implemented with okvis error convention.
  Eigen::Vector3d p_WS_Trapezoid;
  Eigen::Quaterniond q_WS_Trapezoid;
  Eigen::Vector3d speedTrapezoid;
  Eigen::Matrix<double, 6, 1> biasTrapezoid;
  bool usePositionVelocityLin = true;
  Eigen::MatrixXd covTrapezoid;
  Eigen::MatrixXd jacobianTrapezoid;
  IMUOdometryTrapezoidRule(cpc, &p_WS_Trapezoid, &q_WS_Trapezoid,
                           &speedTrapezoid, &biasTrapezoid, &covTrapezoid,
                           &jacobianTrapezoid, zeroVarForImuExtraParams,
                           usePositionVelocityLin, verbose);
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> sqrtCovDiagonalTrapezoid = covTrapezoid.diagonal().cwiseSqrt();
  Eigen::Matrix<double, ImuModelT::kAugmentedMinDim + swift_vio::kNavStateBiasMinDim, 1> jacobianFirstRowTrapezoid = jacobianTrapezoid.row(0);

  /// method 3 : okvis propagation leutenegger's implementation
  okvis::kinematics::Transformation T_WS =
      okvis::kinematics::Transformation(cpc.get_p_WS_W0(), cpc.get_q_WS0());
  Eigen::Matrix<double, 9, 1> sb = cpc.get_sb0();
  okvis::timing::Timer leutenTimer("leutenegger", false);

  Eigen::Matrix<double, 15, 15> covOkvis;
  covOkvis.setZero();
  Eigen::Matrix<double, 15, 15> jacobianOkvis;
  jacobianOkvis.setIdentity();

  // The Leutenegger's ImuError propagation function starts propagation with an
  // zero covariance. The original implementation has some issue in covariance
  // propagation, its Jacobian is correct though.
  int numUsedImuMeasurements = okvis::ceres::ImuError::propagation(
      cpc.get_imu_measurements(), cpc.get_imu_params(), T_WS, sb,
      cpc.get_meas_begin_time(), cpc.get_meas_end_time(), &covOkvis, &jacobianOkvis);
  timeElapsed = leutenTimer.stop();

  Eigen::Vector3d p_WS_Okvis = T_WS.r();
  Eigen::Quaterniond q_WS_Okvis = T_WS.q();
  okvis::SpeedAndBiases speedAndBiasOkvis = sb;
  Eigen::Matrix<double, 15, 1> covDiagonalOkvis = covOkvis.diagonal();
  Eigen::Matrix<double, 15, 1> sqrtCovDiagonalOkvis = covDiagonalOkvis.cwiseSqrt();
  Eigen::Matrix<double, 15, 1> jacobianFirstRowOkvis = jacobianOkvis.row(0).transpose();

  std::cout << "\n>>> Time used by OKVIS Leutenegger forward propagtion with covariance "
            << timeElapsed << std::endl;
  if (verbose) {
    std::cout << "numUsedMeas " << numUsedImuMeasurements << " totalMeas "
              << (int)cpc.get_meas_size() << std::endl;
    std::cout << "q_WS " << q_WS_Okvis.w() << " " << q_WS_Okvis.x() << " "
              << q_WS_Okvis.y() << " " << q_WS_Okvis.z() << std::endl;
    std::cout << "p_WS_W " << p_WS_Okvis.transpose() << std::endl;
    std::cout << "speed and bias " << speedAndBiasOkvis.transpose() << std::endl;
    std::cout << "starting with 0s, cov diagonal sqrt " << std::endl;
    std::cout << sqrtCovDiagonalOkvis.transpose() << std::endl;
    std::cout << "Jacobian diagonal " << std::endl;
    std::cout << vio::superdiagonal(jacobianOkvis).transpose() << std::endl;
    std::cout << "Jacobian first row " << std::endl;
    std::cout << jacobianFirstRowOkvis.transpose() << std::endl;
  }

  /// method 4: simple Euler integration
  Eigen::Matrix<double, 15, 15> covEuler;
  covEuler.setIdentity();

  Sophus::SE3d T_WS0_se3(cpc.get_q_WS0(), cpc.get_p_WS_W0());
  sb = cpc.get_sb0();
  double time_pair[2] = {cpc.get_meas_begin_time().toSec(),
                         cpc.get_meas_end_time().toSec()};
  std::vector<Eigen::Matrix<double, 7, 1>,
              Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>>
      measurements = cpc.get_imu_measurement_vector();

  Eigen::Matrix<double, 6, 1> gwomegaw;
  gwomegaw.setZero();
  gwomegaw.head<3>() = Eigen::Vector3d(0, 0, -cpc.get_g());

  Eigen::Matrix<double, 12, 1> q_n_aw_babw = cpc.get_q_n_aw_babw();

  Sophus::SE3d T_WS1_se3;
  Eigen::Matrix<double, 3, 1> v_WS1_Euler;
  okvis::timing::Timer simpleTimer("simple", false);
  swift_vio::ode::predictStates(T_WS0_se3, sb, time_pair, measurements,
                                   gwomegaw, q_n_aw_babw, &T_WS1_se3,
                                   &v_WS1_Euler, &covEuler, cpc.getImuExtraParams());
  timeElapsed = simpleTimer.stop();

  std::cout << "\n>>> Time used by 1st order propagtion with covariance "
            << timeElapsed << std::endl;

  Eigen::Vector3d p_WS_Euler = T_WS1_se3.translation();
  Eigen::Quaterniond q_WS_Euler = T_WS1_se3.unit_quaternion();
  okvis::SpeedAndBiases speedAndBiasEuler = sb;
  speedAndBiasEuler.head<3>() = v_WS1_Euler;
  Eigen::Matrix<double, 15, 1> covDiagonalEuler = covEuler.diagonal();
  Eigen::Matrix<double, 15, 1> sqrtCovDiagonalEuler = covDiagonalEuler.cwiseSqrt();
  Eigen::Matrix<double, 15, 1> sqrtCovDiagonalPermutated = sqrtCovDiagonalEuler;
  sqrtCovDiagonalPermutated.segment<3>(3) = sqrtCovDiagonalEuler.segment<3>(6);
  sqrtCovDiagonalPermutated.segment<3>(6) = sqrtCovDiagonalEuler.segment<3>(3);
  sqrtCovDiagonalPermutated.segment<3>(12) = sqrtCovDiagonalEuler.segment<3>(9);
  sqrtCovDiagonalPermutated.segment<3>(9) = sqrtCovDiagonalEuler.segment<3>(12);
  sqrtCovDiagonalEuler = sqrtCovDiagonalPermutated;
  if (verbose) {
    std::cout << "q_WS " << q_WS_Euler.w() << " " << q_WS_Euler.x() << " " << q_WS_Euler.y()
              << " " << q_WS_Euler.z() << std::endl;
    std::cout << "p_WS_W " << p_WS_Euler.transpose() << std::endl;
    std::cout << "speed and bias " << speedAndBiasEuler.transpose() << std::endl;
    std::cout << "cov diagonal sqrt " << std::endl;
    std::cout << sqrtCovDiagonalEuler.transpose() << std::endl;
  }

  // RK4 vs Trapezoid state
  EXPECT_TRUE(std::fabs(q_WS_Trapezoid.w() - q_WS_RK4.w()) < 1e-5 &&
              std::fabs(q_WS_Trapezoid.x() - q_WS_RK4.x()) < 1e-5 &&
              std::fabs(q_WS_Trapezoid.y() - q_WS_RK4.y()) < 1e-5 &&
              std::fabs(q_WS_Trapezoid.z() - q_WS_RK4.z()) < 1e-5);
  EXPECT_LT((p_WS_W_RK4 - p_WS_Trapezoid).norm(), 50);
  EXPECT_LT((speedRK4 - speedTrapezoid).norm(), 2);

  // RK4 vs Euler state
  EXPECT_TRUE(std::fabs(q_WS_Euler.w() - q_WS_RK4.w()) < 5e-2 &&
              std::fabs(q_WS_Euler.x() - q_WS_RK4.x()) < 5e-2 &&
              std::fabs(q_WS_Euler.y() - q_WS_RK4.y()) < 5e-2 &&
              std::fabs(q_WS_Euler.z() - q_WS_RK4.z()) < 5e-2);
  EXPECT_LT((p_WS_W_RK4 - p_WS_Euler).norm(), 2000);
  EXPECT_LT((speedRK4 - speedAndBiasEuler.head<3>()).norm(), 50);

  // RK4 vs OKVIS state
  EXPECT_TRUE(std::fabs(q_WS_Okvis.w() - q_WS_RK4.w()) < 1e-5 &&
              std::fabs(q_WS_Okvis.x() - q_WS_RK4.x()) < 1e-5 &&
              std::fabs(q_WS_Okvis.y() - q_WS_RK4.y()) < 1e-5 &&
              std::fabs(q_WS_Okvis.z() - q_WS_RK4.z()) < 1e-5);
  EXPECT_LT((p_WS_W_RK4 - p_WS_Okvis).norm(), 50);
  EXPECT_LT((speedRK4 - speedAndBiasOkvis.head<3>()).norm(), 2);

  // RK4 vs Trapezoid covariance
  EXPECT_LT((sqrtCovDiagonalTrapezoid.head<3>() - sqrtCovDiagonalRK4.head<3>()).norm(), 300);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(3) - sqrtCovDiagonalRK4.segment<3>(3)).norm(),
            2e-2);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(6) - sqrtCovDiagonalRK4.segment<3>(6)).norm(),
            10);

  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(9) - sqrtCovDiagonalRK4.segment<3>(9)).norm(),
            1e-6);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<3>(12) - sqrtCovDiagonalRK4.segment<3>(12)).norm(),
            1e-6);

  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<9>(15) - sqrtCovDiagonalRK4.segment<9>(15)).norm(),
            1e-6);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<9>(24) - sqrtCovDiagonalRK4.segment<9>(24)).norm(),
            1e-6);
  EXPECT_LT((sqrtCovDiagonalTrapezoid.segment<9>(33) - sqrtCovDiagonalRK4.segment<9>(33)).norm(),
            1e-6);

  // RK4 vs Trapezoid jacobian
  EXPECT_LT((jacobianFirstRowRK4.head<3>() - jacobianFirstRowTrapezoid.head<3>()).norm() /
                jacobianFirstRowRK4.head<3>().norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<6>(3) - jacobianFirstRowTrapezoid.segment<6>(3)).norm() /
                jacobianFirstRowRK4.segment<6>(3).norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<6>(9) - jacobianFirstRowTrapezoid.segment<6>(9)).norm() /
                jacobianFirstRowRK4.segment<6>(9).norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<9>(15) - jacobianFirstRowTrapezoid.segment<9>(15)).norm() /
                jacobianFirstRowRK4.segment<9>(15).norm(),
            5e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<9>(24) - jacobianFirstRowTrapezoid.segment<9>(24)).norm() /
                jacobianFirstRowRK4.segment<9>(24).norm(),
            1e-3);
  EXPECT_LT((jacobianFirstRowRK4.segment<9>(33) - jacobianFirstRowTrapezoid.segment<9>(33)).norm() /
                jacobianFirstRowRK4.segment<9>(33).norm(),
            1e-3);

  // RK4 vs OKVIS jacobian
  EXPECT_LT(
      (jacobianFirstRowRK4.head<3>() - jacobianFirstRowOkvis.head<3>()).norm() /
          jacobianFirstRowRK4.head<3>().norm(),
      1e-3);
  EXPECT_LT(
      (jacobianFirstRowRK4.segment<6>(3) - jacobianFirstRowOkvis.segment<6>(3))
              .norm() /
          jacobianFirstRowRK4.segment<6>(3).norm(),
      1e-3);
  EXPECT_LT(
      (jacobianFirstRowRK4.segment<6>(9) - jacobianFirstRowOkvis.segment<6>(9))
              .norm() /
          jacobianFirstRowRK4.segment<6>(9).norm(),
      1e-3);

  // RK4 vs Euler covariance
  EXPECT_LT((sqrtCovDiagonalEuler.head<3>() - sqrtCovDiagonalRK4.head<3>()).norm() /
                sqrtCovDiagonalEuler.head<3>().norm(),
            8e-2);
  EXPECT_LT((sqrtCovDiagonalEuler.segment<3>(3) - sqrtCovDiagonalRK4.segment<3>(3)).norm() /
                sqrtCovDiagonalEuler.segment<3>(3).norm(),
            7e-2);
  EXPECT_LT((sqrtCovDiagonalEuler.segment<3>(6) - sqrtCovDiagonalRK4.segment<3>(6)).norm() /
                sqrtCovDiagonalEuler.segment<3>(6).norm(),
            0.1);

  EXPECT_LT((sqrtCovDiagonalEuler.segment<3>(9) - sqrtCovDiagonalRK4.segment<3>(9)).norm(),
            1e-6);
  EXPECT_LT((sqrtCovDiagonalEuler.segment<3>(12) - sqrtCovDiagonalRK4.segment<3>(12)).norm(),
            1e-6);
}

TEST(ImuOdometry, dokvis_dri) {
  okvis::kinematics::Transformation T_WB;
  T_WB.setRandom();
  Eigen::Vector3d v_WB;
  v_WB.setRandom();
  Eigen::Matrix<double, 15, 15> permutator = swift_vio::dokvis_drightinvariant(T_WB, v_WB);
  Eigen::Matrix<double, 15, 15> permutator_inv =
      swift_vio::drightinvariant_dokvis(T_WB, v_WB);
  EXPECT_LT(((permutator * permutator_inv) -
             Eigen::Matrix<double, 15, 15>::Identity())
                .lpNorm<Eigen::Infinity>(),
            1e-8);
}

TEST(ImuOdometry, dokvis_dforster) {
  okvis::kinematics::Transformation T_WB;
  T_WB.setRandom();
  Eigen::Matrix<double, 15, 15> permutator = swift_vio::dokvis_dforster(T_WB.C());
  Eigen::Matrix<double, 15, 15> permutator_inv =
      swift_vio::dforster_dokvis(T_WB.C());
  EXPECT_LT(((permutator * permutator_inv) -
             Eigen::Matrix<double, 15, 15>::Identity())
                .lpNorm<Eigen::Infinity>(),
            1e-8);
}

void propagateRiCovarianceFromZero(
    std::shared_ptr<const Eigen::Vector3d> p_WS0_ptr,
    std::shared_ptr<const Eigen::Vector3d> v_WS0_ptr, bool verbose) {
  srand((unsigned int)time(0));
  CovPropConfig<swift_vio::Imu_BG_BA> cpc(false, true);
  Eigen::Vector3d initialPosition = cpc.get_p_WS_W0();
  if (p_WS0_ptr) {
    initialPosition = *p_WS0_ptr;
  }
  Eigen::Vector3d initialVelocity = cpc.get_sb0().head<3>();
  if (v_WS0_ptr) {
    initialVelocity = *v_WS0_ptr;
  }
  if (verbose) {
    std::cout << "Initial position " << initialPosition.transpose()
              << " velocity " << initialVelocity.transpose() << "\n";
  }
  okvis::kinematics::Transformation T_WS =
      okvis::kinematics::Transformation(initialPosition, cpc.get_q_WS0());
  Eigen::Vector3d v_WS = initialVelocity;
  Eigen::Matrix<double, 6, 1> bias = cpc.get_bias0();

  okvis::timing::Timer timer("RK4", false);
  Eigen::Matrix<double, 15, 15> covRK4, jacobianRK4;
  covRK4.setZero();
  jacobianRK4.setIdentity();

  swift_vio::Imu_BG_BA imuModel;
  imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
  int numUsedImuMeasurements = swift_vio::ImuOdometry::propagation_RungeKutta(
      cpc.get_imu_measurements(), cpc.get_imu_params(), T_WS, v_WS,
      imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time(), &covRK4, &jacobianRK4);

  double timeElapsed = timer.stop();

  Eigen::Vector3d p_WS_RK4 = T_WS.r();
  Eigen::Quaterniond q_WS_RK4 = T_WS.q();
  Eigen::Vector3d speedRK4 = v_WS;
  Eigen::Matrix<double, 15, 1> covDiagonalRK4 = covRK4.diagonal();
  Eigen::Matrix<double, 15, 1> sqrtCovDiagonalRK4 = covDiagonalRK4.cwiseSqrt();

  if (verbose) {
    std::cout
        << "\n>>> Time used by OKVIS Leutenegger corrected forward propagtion with "
           "0 initial covariance: "
        << timeElapsed << "\n";
    std::cout << "numUsedMeas " << numUsedImuMeasurements << " totalMeas "
              << (int)cpc.get_meas_size() << std::endl;
    std::cout << "q_WS " << q_WS_RK4.w() << " " << q_WS_RK4.x() << " "
              << q_WS_RK4.y() << " " << q_WS_RK4.z() << std::endl;
    std::cout << "p_WS_W " << p_WS_RK4.transpose() << std::endl;
    std::cout << "speed " << speedRK4.transpose() << std::endl;
    std::cout << "cov diagonal sqrt " << std::endl;
    std::cout << sqrtCovDiagonalRK4.transpose() << std::endl;
    std::cout << "Jacobian super diagonal " << std::endl;
    std::cout << vio::superdiagonal(jacobianRK4).transpose() << std::endl;
    std::cout << "cov\n" << covRK4 << "\njac\n" << jacobianRK4 << "\n";
  }

  // Right invariant error method.
  Eigen::Matrix<double, 15, 15> covariance_ri = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 15> jacobian_ri;

  okvis::kinematics::Transformation T_WS_ri(initialPosition, cpc.get_q_WS0());
  Eigen::Matrix<double, 6, 1> bias_ri = cpc.get_bias0();

  Eigen::Vector3d v_WS_ri = initialVelocity;
  swift_vio::Imu_BG_BA iem;
  iem.updateParameters(bias_ri.data());

  okvis::timing::Timer riTimer("right invariant error", false);
  numUsedImuMeasurements = swift_vio::ImuOdometry::propagationRightInvariantError(
      cpc.get_imu_measurements(), cpc.get_imu_params(), T_WS_ri, v_WS_ri, iem,
      cpc.get_meas_begin_time(), cpc.get_meas_end_time(), &covariance_ri,
      &jacobian_ri);
  double riElapsed = riTimer.stop();

  EXPECT_LT((p_WS_RK4 - T_WS_ri.r()).norm() / p_WS_RK4.norm(), 5e-3);
  check_q_near(q_WS_RK4, T_WS_ri.q(), 1e-5);
  EXPECT_LT((speedRK4 - v_WS_ri).head<3>().norm() /
                speedRK4.norm(),
             1e-2);
  EXPECT_LT((bias - bias_ri).norm(), 1e-6);

  Eigen::Matrix<double, 15, 15> permutator_j =
      swift_vio::dokvis_drightinvariant(T_WS_ri, v_WS_ri);

  Eigen::Matrix<double, 15, 15> permutator_i_inv = swift_vio::drightinvariant_dokvis(
      okvis::kinematics::Transformation(initialPosition, cpc.get_q_WS0()),
      initialVelocity);

  jacobian_ri = permutator_j * jacobian_ri * permutator_i_inv;
  covariance_ri = permutator_j * covariance_ri * permutator_j.transpose();

  if (verbose) {
    std::cout << "\n>>> Time used by right invariant error forward propagtion with "
                 "0 initial covariance: "
              << riElapsed << "\n";
    std::cout << "cov diag sqrt\n"
              << covariance_ri.diagonal().cwiseSqrt().transpose() << "\n";
    std::cout << "Jacobian super diagonal\n"
              << vio::superdiagonal(jacobian_ri).transpose() << "\n";
    std::cout << "cov\n" << covariance_ri << "\njac\n" << jacobian_ri << "\n";
  }

  std::cout << "Check jacobians of okvis and right invariant error\n";
  expectNearAbsRel(jacobianRK4.topLeftCorner<15, 9>(),
                      jacobian_ri.topLeftCorner<15, 9>(), 1e-3, 0.09);
  expectNearAbsRel(jacobianRK4.topRightCorner<15, 6>(),
                      jacobian_ri.topRightCorner<15, 6>(), 1e-3, 0.1);

  std::cout << "Check P, Q, V covariance of okvis and right invariant error\n";
  expectNearAbsRel(covRK4.topLeftCorner<15, 9>(),
                      covariance_ri.topLeftCorner<15, 9>(), 3, 0.35);
  std::cout << "Check Bg Ba covariance of okvis and right invariant error\n";
  expectNearAbsRel(covRK4.topRightCorner<15, 6>(),
                      covariance_ri.topRightCorner<15, 6>(), 10, 2.5e-1);
}

TEST(ImuOdometry, RiCovariancePropagationFromZero) {
  propagateRiCovarianceFromZero(nullptr, nullptr, true);
}

TEST(ImuOdometry, RiCovariancePropagationWithLargeP) {
  // Propagation from a large position does not seem to worsen covariance or
  // jacobian.
  std::shared_ptr<const Eigen::Vector3d> positionPtr(
      new Eigen::Vector3d(Eigen::Vector3d::Random() * 1000));
  propagateRiCovarianceFromZero(positionPtr, nullptr, true);
}

TEST(ImuOdometry, RiCovariancePropagationWithLargeV) {
  // Propagation from a large velocity does not seem to worsen covariance or
  // jacobian.
  std::shared_ptr<const Eigen::Vector3d> velocityPtr(
      new Eigen::Vector3d(Eigen::Vector3d::Random() * 1000));
  propagateRiCovarianceFromZero(nullptr, velocityPtr, true);
}

TEST(ImuOdometry, initPoseFromImu) {
  Eigen::Vector3d acc_B = Eigen::Vector3d::Random();
  okvis::ImuMeasurementDeque imuMeasurements;
  imuMeasurements.emplace_back(okvis::Time(1.0), okvis::ImuSensorReadings(Eigen::Vector3d::Random(), acc_B));
  okvis::kinematics::Transformation T_WB;
  swift_vio::initPoseFromImu(imuMeasurements, okvis::Time(1.0), T_WB);
  Eigen::Vector3d transVec = T_WB.q()._transformVector(acc_B);
  transVec.normalize();
  ASSERT_LT((transVec - Eigen::Vector3d(0, 0, 1)).norm(), 1e-8);
}

TEST(ImuOdometry, alignZ) {
  Eigen::Vector3d acc_B = Eigen::Vector3d::Random();
  Eigen::Quaterniond q_WB;
  swift_vio::alignZ(acc_B, &q_WB);
  Eigen::Vector3d acc_W = q_WB._transformVector(acc_B);
  acc_W.normalize();
  ASSERT_LT((acc_W - Eigen::Vector3d(0, 0, 1)).norm(), 1e-8);
}

