/**
 * @file    TestGtsamImuFactor.cpp
 * @brief   Unit test for Gtsam Imu propagation with the preintegration
 * measurement against OKVIS corrected propagation
 * @author  J. Huai
 */
#include <gtest/gtest.h>
#include <fstream>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include "gtsam/ImuFrontEnd.h"
#include "gtsam/ImuFactorTestHelpers.h"

#include <okvis/timing/Timer.hpp>

#include "swift_vio/imu/ImuModels.hpp"
#include "swift_vio/imu/ImuOdometry.h"
#include "swift_vio/imu/CovPropConfig.hpp"

#include "vio/Sample.h"
#include "vio/eigen_utils.h"

Eigen::IOFormat commaInitFmt(Eigen::StreamPrecision, 0, ", ", "\n", "", "",
                             "", "");

/**
 * @brief ImuOdometryGtsam
 * @param p_WS_W0
 * @param q_WS0
 * @param sb0
 * @param imuMeasurements
 * @param imuParams
 * @param p_WS_W1
 * @param q_WS1
 * @param sb1
 * @param[out] covariance cov(\xi_j)
 * @param jacobian
 * @param verbose
 */
void ImuOdometryGtsam(const Eigen::Vector3d& p_WS_W0,
                      const Eigen::Quaterniond& q_WS0,
                      const okvis::SpeedAndBiases& sb0,
                      const okvis::ImuMeasurementDeque& imuMeasurements,
                      const okvis::ImuParameters& imuParams,
                      Eigen::Vector3d* p_WS_W1, Eigen::Quaterniond* q_WS1,
                      okvis::SpeedAndBiases* sb1,
                      Eigen::Matrix<double, 15, 15>* covariance,
                      Eigen::Matrix<double, 15, 15>* jacobian,
                      bool verbose = false) {
  /// gtsam error definition:
  /// R_{wb} = \hat{R}_{wb} Exp(\theta_i),
  /// p_{wb} = \hat{p}_{wb} + R_{wb}\delta p_b,
  /// v_{wb} = \hat{v}_{wb} + \delta v_b^w
  /// and definitions of \delta b_g and \delta b_a follows \delta v_b^w.
  /// gtsam error vector [\theta, \delta p, \delta v_s^w, \delta b_a, \delta b_g]
  /// gtsam imu factor error definition
  /// e_{gtsam-imu}= \begin{bmatrix} log((\hat{R}_j^i(z_{imu}))^{-1}R_j^i)^{V}
  /// \\ R^j_g(\hat{p}_j^g - p_j^g) \\ R^j_g(\hat{v}_j^g - v_j^g) \\ {b}_{a,i} -
  /// b_{a,j}\\ {b}_{g,i} - b_{g,j}\end{bmatrix}

  okvis::Time startEpoch = imuMeasurements.front().timeStamp;
  okvis::Time finishEpoch = imuMeasurements.back().timeStamp;
  gtsam::Vector3 n_gravity = Eigen::Vector3d(
      0, 0, -imuParams.g);  ///< Gravity vector in nav frame (namely, the global
                            ///< frame, as used in the client's VINS)

  swift_vio::ImuFrontEnd::PimPtr combinedPim;

  okvis::timing::Timer gtsamTimer("gtsam", false);
  swift_vio::ImuParams imuParamsKimera;
  imuParamsKimera.set(imuParams);
  imuParamsKimera.imu_preintegration_type_ =
      swift_vio::ImuPreintegrationType::kPreintegratedCombinedMeasurements;

  swift_vio::ImuFrontEnd imuIntegrator(imuParamsKimera);
  imuIntegrator.preintegrateImuMeasurements(imuMeasurements, sb0, startEpoch,
                                            finishEpoch, combinedPim);

  double timeElapsed = gtsamTimer.stop();
  std::cout << "Time used by gtsam preintegration " << timeElapsed << std::endl;

  Eigen::Matrix<double, 9, 6> D_r_pose_i, D_r_pose_j, D_r_bias_i;
  Eigen::Matrix<double, 9, 3> D_r_vel_i, D_r_vel_j;
  Eigen::Vector3d p_ij = combinedPim->deltaPij();
  Eigen::Vector3d v_ij = combinedPim->deltaVij();
  gtsam::Rot3 gtR_ij = combinedPim->deltaRij();

  *q_WS1 = q_WS0 * gtR_ij.toQuaternion();
  *p_WS_W1 = p_WS_W0 + sb0.head<3>() * combinedPim->deltaTij() +
      0.5 * n_gravity * combinedPim->deltaTij() * combinedPim->deltaTij() +
      q_WS0 * p_ij;
  Eigen::Vector3d v_WS1 =
      sb0.head<3>() + n_gravity * combinedPim->deltaTij() + q_WS0 * v_ij;

  gtsam::NavState state_i(gtsam::Rot3(q_WS0), p_WS_W0, sb0.head<3>());
  gtsam::imuBias::ConstantBias bias_i(sb0.tail<3>(), sb0.segment<3>(3));
  gtsam::NavState predictedState_j = combinedPim->predict(state_i, bias_i);
  EXPECT_TRUE(gtsam::assert_equal(predictedState_j.pose().rotation(), gtsam::Rot3(*q_WS1)));
  EXPECT_TRUE(gtsam::assert_equal(predictedState_j.velocity(), v_WS1));
  Eigen::Vector3d predictedPosition_j = predictedState_j.position();
  EXPECT_TRUE(gtsam::assert_equal(predictedPosition_j, *p_WS_W1));

  Eigen::Matrix<double, 15, 15> cov_r =
      dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*combinedPim)
          .preintMeasCov();

  // jacobian of the OKVIS error \xi relative to gtsam error state at t_j.
  Eigen::Matrix<double, 15, 15> dxi_deta_j = swift_vio::dokvis_dforster(*q_WS1);

  gtsam::Pose3 pose_i(gtsam::Rot3(q_WS0), p_WS_W0);
  Eigen::Vector3d vel_i = sb0.head<3>();
  gtsam::Pose3 pose_j(gtsam::Rot3(*q_WS1), *p_WS_W1);
  Eigen::Vector3d vel_j = v_WS1;

  /*Eigen::Matrix<double, 9, 1> r_Rpv =*/combinedPim->computeErrorAndJacobians(
      pose_i, vel_i, pose_j, vel_j, bias_i, &D_r_pose_i, &D_r_vel_i,
      &D_r_pose_j, &D_r_vel_j, &D_r_bias_i);
  Eigen::Matrix<double, 15, 15> dr_deta_i;
  // Jacobian of imu factor error relative to the error in the state at t_i,
  // $ \frac{\partial e(X_{j}, X_{i}, z_{imu})}{\partial (\theta_i,\delta
  // p_i,\delta v_i, \delta b_{a,i},\delta b_{g,i})} $
  // see Forster et al. eq (70) and (39) for their definitions.
  dr_deta_i << D_r_pose_i, D_r_vel_i, D_r_bias_i, Eigen::Matrix<double, 6, 9>::Zero(),
      Eigen::Matrix<double, 6, 6>::Identity();


  // Jacobian of imu factor error relative to the error in the state at t_j.
  Eigen::Matrix<double, 15, 15> dr_deta_j;
  dr_deta_j << D_r_pose_j, D_r_vel_j, Eigen::Matrix<double, 9, 6>::Zero(),
      Eigen::Matrix<double, 6, 9>::Zero(),
      -Eigen::Matrix<double, 6, 6>::Identity(); // -1 because CombinedImuFactor computeError in bias as e_b = b_i - b_j.

  Eigen::Matrix<double, 15, 15> dxi_deta_i = swift_vio::dokvis_dforster(q_WS0);

  sb1->head<3>() = v_WS1;
  sb1->tail<6>() = sb0.tail<6>();

  // assumes zero cov and identity Jac at the start.
  // Jacobian of the okvis error \xi_{j|i} at t_j relative to the reordered gtsam error \eta at t_i.
  // d\xi_{j|i} / d\xi_i = d\xi_{j|i} / d\eta_{j|i} * d\eta_{j|i} / d\eta_{i} * (d\xi_i / d\eta_i)^{-1}.
  // where d\eta_{j|i} / d\eta_{i}  = (de / d\eta_{j|i})^{-1} * de / d\eta_i.
  // we use transpose because it has the same effect as inverse.
  *jacobian = dxi_deta_j * (-dr_deta_j.transpose()) * dr_deta_i * dxi_deta_i.transpose();
  Eigen::Matrix<double, 15, 15> dr_deta_j_inv = dr_deta_j.transpose();
  *covariance = dxi_deta_j * dr_deta_j_inv * cov_r * dr_deta_j * dxi_deta_j.transpose();

  Eigen::Matrix<double, 15, 1> sqrtDiagCov1 = covariance->diagonal().cwiseSqrt();

  if (verbose) {
    std::cout << "q_WS " << q_WS1->w() << " " << q_WS1->x() << " " << q_WS1->y()
              << " " << q_WS1->z() << std::endl;
    std::cout << "p_WS_W " << p_WS_W1->transpose() << std::endl;
    std::cout << "speed and bias " << v_WS1.transpose() << " "
              << combinedPim->biasHat().gyroscope().transpose() << " "
              << combinedPim->biasHat().accelerometer().transpose() << std::endl;
    std::cout << "cov diagonal sqrt\n" << sqrtDiagCov1.transpose() << "\n";
    std::cout << "Jacobian super diagonal " << std::endl;
    std::cout << vio::superdiagonal(*jacobian).transpose() << std::endl;
//    std::cout << "Covariance\n" << covariance->format(commaInitFmt) << "\n";
//    std::cout << "Jacobian\n" << jacobian->format(commaInitFmt) << "\n";
  }
}

TEST(ImuOdometry, PropagationByPreintegrationZeroCov) {
  // This test validates that propagation from a start state and zero covariance can be
  // implemented in terms of preintegration.
  bool verbose = true;
  const std::string outputdir = "./";
  const std::string baseOutputPath =
    outputdir.empty() ? "./"
                      : (outputdir.back() == '/' ? outputdir : outputdir + "/");
  for (int seed = 0; seed < 1; ++seed) {
    CovPropConfig<swift_vio::Imu_BG_BA> cpc(false, true, seed);
    okvis::kinematics::Transformation T_WS =
        okvis::kinematics::Transformation(cpc.get_p_WS_W0(), cpc.get_q_WS0());
    Eigen::Vector3d speed = cpc.get_v_WS0();
    Eigen::Matrix<double, 6, 1> bias = cpc.get_bias0();
    okvis::timing::Timer timer("RK4", false);
    Eigen::Matrix<double, 15, 15> covRK4, jacobianRK4;
    covRK4.setZero();
    jacobianRK4.setIdentity();

    swift_vio::Imu_BG_BA imuModel;
    imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
    int numUsedImuMeasurements = swift_vio::ImuOdometry::propagation_RungeKutta(
        cpc.get_imu_measurements(), cpc.get_imu_params(), T_WS, speed,
        imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time(), &covRK4, &jacobianRK4);
    double timeElapsed = timer.stop();

    Eigen::Vector3d p_WS_RK4 = T_WS.r();
    Eigen::Quaterniond q_WS_RK4 = T_WS.q();
    Eigen::Vector3d speedRK4 = speed;
    Eigen::Matrix<double, 6, 1> biasRK4 = bias;
    Eigen::Matrix<double, 15, 1> covDiagonalRK4 = covRK4.diagonal();
    Eigen::Matrix<double, 15, 1> sqrtCovDiagonalRK4 = covDiagonalRK4.cwiseSqrt();

    std::cout << "\n>>> Time used by RK4 forward propagtion with "
                "0 initial covariance " << timeElapsed << std::endl;
    if (verbose) {
        std::cout << "numUsedMeas " << numUsedImuMeasurements << " totalMeas "
                << (int)cpc.get_meas_size() << std::endl;
        std::cout << "q_WS " << q_WS_RK4.w() << " " << q_WS_RK4.x() << " "
                << q_WS_RK4.y() << " " << q_WS_RK4.z() << std::endl;
        std::cout << "p_WS_W " << p_WS_RK4.transpose() << std::endl;
        std::cout << "speed and bias " << speedRK4.transpose() << " " << biasRK4.transpose() << std::endl;
        std::cout << "cov diagonal sqrt " << std::endl;
        std::cout << sqrtCovDiagonalRK4.transpose() << std::endl;
        std::cout << "Jacobian super diagonal " << std::endl;
        std::cout << vio::superdiagonal(jacobianRK4).transpose() << std::endl;
    //    std::cout << "Covariance\n" << covRK4.format(commaInitFmt) << "\n";
    //    std::cout << "Jacobian\n" << jacobianRK4.format(commaInitFmt) << std::endl;
    }

    Eigen::Vector3d p_WS_gtsam;
    Eigen::Quaterniond q_WS_gtsam;
    okvis::SpeedAndBiases speedAndBiasGtsam;
    Eigen::Matrix<double, 15, 15> covGtsam;
    Eigen::Matrix<double, 15, 15> jacobianGtsam;
    ImuOdometryGtsam(cpc.get_p_WS_W0(), cpc.get_q_WS0(), cpc.get_sb0(),
                    cpc.get_imu_measurements(), cpc.get_imu_params(),
                    &p_WS_gtsam, &q_WS_gtsam, &speedAndBiasGtsam, &covGtsam,
                    &jacobianGtsam, verbose);

    EXPECT_LT((p_WS_RK4 - p_WS_gtsam).norm() / p_WS_RK4.norm(),  5e-3);
    check_q_near(q_WS_RK4, q_WS_gtsam, 1e-5);
    EXPECT_LT((speedRK4 - speedAndBiasGtsam.head<3>()).norm() / speedRK4.norm(), 3e-3);
    EXPECT_LT((biasRK4 - speedAndBiasGtsam.tail<6>()).norm(), 1e-6);

    std::cout << "Check jacobians of RK4 and gtsam\n";
    expectNearAbsRel(jacobianRK4.topLeftCorner<15, 9>(),
                        jacobianGtsam.topLeftCorner<15, 9>(), 1e-3, 0.09);
    expectNearAbsRel(jacobianRK4.topRightCorner<15, 6>(),
                        jacobianGtsam.topRightCorner<15, 6>(), 1e-3, 0.2);

    std::cout << "Check P, V, Q covariance of RK4 and gtsam\n";
    expectNearAbsRel(covRK4.topLeftCorner<15, 9>(),
                        covGtsam.topLeftCorner<15, 9>(), 2, 0.35);
    std::cout << "Check Bg, Ba covariance of RK4 and gtsam\n";
    expectNearAbsRel(covRK4.topRightCorner<15, 6>(),
                        covGtsam.topRightCorner<15, 6>(), 10, 2.5e-1);
    auto saveMatrix = [&](const std::string& name,
                          const Eigen::Matrix<double, 15, 15>& mat) {
      const std::string filename =
          (baseOutputPath.empty() ? "" : baseOutputPath) + name + "_" +
          std::to_string(seed) + ".txt";
      std::ofstream ofs(filename, std::ios::trunc);
      if (!ofs.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
      }
      ofs.setf(std::ios::fixed);
      ofs.precision(18);
      for (int r = 0; r < 15; ++r) {
        for (int c = 0; c < 15; ++c) {
          ofs << mat(r, c);
          if (c < 14) {
            ofs << ' ';
          }
        }
        ofs << '\n';
      }
    };
    saveMatrix("jacobianRK4", jacobianRK4);
    saveMatrix("jacobianGtsam", jacobianGtsam);
    saveMatrix("covRK4", covRK4);
    saveMatrix("covGtsam", covGtsam);
  }
}

