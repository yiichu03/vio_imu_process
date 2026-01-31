#include <Eigen/Dense>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavStateImuEKF.h>

#include <gtsam/geometry/Rot3.h>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <iomanip>

#include <gtest/gtest.h>
#include <fstream>

#include <okvis/timing/Timer.hpp>

#include "gtsam/ImuFrontEndParams.h"
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

// ---------- build JincBias from del* blocks ----------
static inline Eigen::Matrix<double,9,6> BuildJincBias_ba_bg(
    const Eigen::Matrix3d& delRdelBg,
    const Eigen::Matrix3d& delPdelBa,
    const Eigen::Matrix3d& delPdelBg,
    const Eigen::Matrix3d& delVdelBa,
    const Eigen::Matrix3d& delVdelBg)
{
  Eigen::Matrix<double,9,6> J = Eigen::Matrix<double,9,6>::Zero();

  // row blocks: [dtheta, dp, dv]
  // col blocks: [dba, dbg]  (i.e., [ba, bg])
  J.block<3,3>(0,3) = delRdelBg;  // dtheta / dbg
  J.block<3,3>(3,0) = delPdelBa;  // dp     / dba
  J.block<3,3>(3,3) = delPdelBg;  // dp     / dbg
  J.block<3,3>(6,0) = delVdelBa;  // dv     / dba
  J.block<3,3>(6,3) = delVdelBg;  // dv     / dbg
  return J;
}

static inline Eigen::Matrix<double,9,6> swapBias(
    const Eigen::Matrix<double,9,6>& JincBias_ba_bg)
{
  Eigen::Matrix<double,6,6> P = Eigen::Matrix<double,6,6>::Zero();
  // [bg,ba] -> [ba,bg]
  P.block<3,3>(0,3) = Eigen::Matrix3d::Identity(); // ba <- ba part of okvis vector (which is at +3)
  P.block<3,3>(3,0) = Eigen::Matrix3d::Identity(); // bg <- bg part of okvis vector (which is at +0)
  // Now columns become [bg,ba] as needed by OKVIS state layout
  Eigen::Matrix<double,9,6> JincBias_bg_ba = JincBias_ba_bg * P;
  return JincBias_bg_ba;
}

// Jacobians for X_{e|s} = X_s \boxplus \Delta X = f(x_s, Dx), Dx can be viewed as z.
struct Maps15 {
  Eigen::Matrix<double,15,15> F;    // df_e(x_s, Dx)/dx_s, both error states are in OKVIS error state conventions, x=[p,q,v,bg,ba]
  Eigen::Matrix<double,15,15> G;    // df_e/dz for transition matrix (z=[dtheta,dp,dv,dba,dbg] of gtsam)
  Eigen::Matrix<double,15,15> covG; // df_e/dz for covariance (z=[dtheta,dp,dv,dba,dbg] of gtsam)
  Eigen::Matrix<double,15,15> G_inv;
  Eigen::Matrix<double,15,15> covG_inv;
  Eigen::Matrix<double,15,15> Phi;  // dx_e/dx_s (OKVIS)
};

// We assume the combined preint covariance is in GTSAM order z=[dtheta,dp,dv,dba,dbg].
// We map it into OKVIS state error ordering [dp,dtheta,dv,dbg,dba] at the end.
static inline Maps15 BuildMaps15_Manifold(
    const Eigen::Matrix3d& Rws,
    const Eigen::Matrix3d& dR,
    const Eigen::Vector3d& dP,
    const Eigen::Vector3d& dV,
    double dt)
{
  Maps15 m;
  m.F.setZero();
  m.G.setZero();
  m.covG.setZero();

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  m.F.block<3,3>(0,0) = I; // dp_e / dp_s
  m.F.block<3,3>(0,6) = dt * I; // dp_e / dv_s
  m.F.block<3,3>(0,3) = - okvis::kinematics::crossMx(Rws *dP); // dp_e / dtheta_s

  m.F.block<3,3>(3,3) = I;
  m.F.block<3,3>(6,6) = I;
  m.F.block<3,3>(6,3) = - okvis::kinematics::crossMx(Rws * dV);

  m.F.block<3,3>(9,9)   = I;
  m.F.block<3,3>(12,12) = I;

  // Now G maps combined preint residual-space error z to end-state error.
  // z layout GTSAM [dtheta, dp, dv, dba, dbg] defined in right perturbation sense.
  // End-state OKVIS [dp, dtheta, dv, dbg, dba] defined in left perturbation sense, R3xSO3xR3xR6
  Eigen::Matrix3d Rwe_from_s = Rws * dR;
  // G for computing covariance because the cov error state is defined like
  // \Delta X_{se} = \Delta\hat{X}_{se} \boxplus [\delta \theta_{se}, \delta p_{se}, \delta v_{se}]
  // where \boxplus is defined as a NavState boxplus.
  // With this, we note that for gtsam 4.2, the cov error state is different from the residual error state, 
  // thus the CombinedImuFactor weighting is wrong/inconsistent in gtsam 4.2.
  // dp_e <- Rs * dp
  m.covG.block<3,3>(0,3) = Rwe_from_s;
  // dtheta_e <- dtheta
  m.covG.block<3,3>(3,0) = Rwe_from_s;
  // dv_e <- Rs * dv
  m.covG.block<3,3>(6,6) = Rwe_from_s;
  // dbg_e <- dbg
  m.covG.block<3,3>(9,12) = -I;
  // dba_e <- dba
  m.covG.block<3,3>(12,9) = -I;
  m.covG_inv = m.covG.transpose();

  // G for computing transition matrix because the residual error state is defined like
  // \Delta R_{se} = \Delta\hat{R}_{se} \boxplus \delta \theta_{se}
  // \Delta p_{se} = \Delta\hat{p}_{se} + \delta p_{se}
  // \Delta v_{se} = \Delta\hat{v}_{se} + \delta v_{se}
  // dp_e <- Rs * dp
  m.G.block<3,3>(0,3) = Rws;
  // dtheta_e <- dtheta
  m.G.block<3,3>(3,0) = Rwe_from_s;
  // dv_e <- Rs * dv
  m.G.block<3,3>(6,6) = Rws;
  // dbg_e <- dbg
  m.G.block<3,3>(9,12) = -I;
  // dba_e <- dba
  m.G.block<3,3>(12,9) = -I;
  m.G_inv = m.G.transpose();
  return m;
}

static inline Maps15 BuildMaps15_Tangent(
    const Eigen::Matrix3d& Rws,
    const Eigen::Matrix3d& dR,
    const Eigen::Vector3d& dP,
    const Eigen::Vector3d& dV,
    double dt)
{
  Maps15 m;
  m.F.setZero();
  m.G.setZero();
  m.covG.setZero();

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  m.F.block<3,3>(0,0) = I; // dp_e / dp_s
  m.F.block<3,3>(0,6) = dt * I; // dp_e / dv_s
  m.F.block<3,3>(0,3) = - okvis::kinematics::crossMx(Rws *dP); // dp_e / dtheta_s

  m.F.block<3,3>(3,3) = I;
  m.F.block<3,3>(6,6) = I;
  m.F.block<3,3>(6,3) = - okvis::kinematics::crossMx(Rws * dV);

  m.F.block<3,3>(9,9)   = I;
  m.F.block<3,3>(12,12) = I;

  // Now G maps combined preint residual-space error z to end-state error.
  // z layout GTSAM [dphi, dp, dv, dba, dbg] defined in right perturbation sense.
  // End-state OKVIS [dp, dtheta, dv, dbg, dba] defined in left perturbation sense.
  Eigen::Matrix3d Rwe_from_s = Rws * dR;
  gtsam::Vector3 phi = gtsam::Rot3::Logmap(gtsam::Rot3(dR));
  Eigen::Matrix3d Jr = gtsam::so3::DexpFunctor(phi).rightJacobian();
  Eigen::Matrix3d Jr_inv = gtsam::so3::DexpFunctor(phi).rightJacobianInverse();

  // G for computing transition matrix because the residual error state is defined like
  // \Delta \phi_{se} = \Delta (\hat{\phi}_{se} + \delta \phi_{se})
  // \Delta p_{se} = \Delta\hat{p}_{se} + \delta p_{se}
  // \Delta v_{se} = \Delta\hat{v}_{se} + \delta v_{se}
  // dp_e <- Rs * dp
  m.G.block<3,3>(0,3) = Rws;
  // dtheta_e <- dtheta
  m.G.block<3,3>(3,0) = Rwe_from_s * Jr;
  // dv_e <- Rs * dv
  m.G.block<3,3>(6,6) = Rws;
  // dbg_e <- dbg
  m.G.block<3,3>(9,12) = -I;
  // dba_e <- dba
  m.G.block<3,3>(12,9) = -I;
  m.G_inv = m.G.transpose();
  m.G_inv.block<3,3>(0,3) = Jr_inv * Rwe_from_s.transpose();

  m.covG = m.G;
  m.covG_inv = m.G_inv;

  return m;
}

// Cov propagation using combined preint covariance (already includes bias RW + cross terms).
static inline Eigen::Matrix<double,15,15> PropCov15_Combined(
    const Maps15& m,
    const Eigen::Matrix<double,15,15>& Sigma_s,
    const Eigen::Matrix<double,15,15>& Phi,
    const Eigen::Matrix<double,15,15>& Sigma_preint_combined)
{
  return Phi * Sigma_s * Phi.transpose()
       + m.covG * Sigma_preint_combined * m.covG.transpose();
}

static inline Eigen::Matrix<double, 15, 15> PropJac15_Combined(
    const Maps15& m, const Eigen::Matrix<double, 9, 6>& JincBias_bg_ba)
{
  Eigen::Matrix<double, 15, 15> J;
  J = m.F;
  J.topRightCorner<9, 6>() += m.G.topLeftCorner<9, 9>() * JincBias_bg_ba;
  return J;
}

template <typename PimT>
static int PropagationByPreintegrationImpl(bool use_manifold, bool zero_cov, bool zero_bias) {
  std::cout << "\n=== TEST: PropagationByPreintegration ===\n";
  std::string tag = use_manifold ? "Manifold" : "Tangent";
  std::cout << "Preintegration: " << (use_manifold ? "Manifold" : "Tangent") << "\n";

  int seed = 0;
  bool to_9d = false; // if you'd like to compare the jac and cov of 15D prop and those of 9D prop.
  CovPropConfig<swift_vio::Imu_BG_BA> cpc(false, true, seed);
  if (zero_bias) {
    cpc.zeroBias0();
  }
  okvis::ImuParameters imu_params = cpc.get_imu_params();
  if (to_9d) {
    imu_params.sigma_gw_c = 0.0;
    imu_params.sigma_aw_c = 0.0;
  }
  okvis::kinematics::Transformation T_WS(cpc.get_p_WS_W0(), cpc.get_q_WS0());
  Eigen::Vector3d speed = cpc.get_v_WS0();
  Eigen::Matrix<double,6,1> bias = cpc.get_bias0();
  std::cout << "bias_s " << bias.transpose() << std::endl;

  // --- RK4 ---
  okvis::timing::Timer timer("RK4", false);
  Eigen::Matrix<double,15,15> covRK4, jacobianRK4;
  if (zero_cov) {
    covRK4.setZero();
  } else {
    covRK4.setIdentity();
    covRK4.diagonal().head<3>().setConstant(100);
    covRK4.diagonal().segment<3>(3).setConstant(1);
    covRK4.diagonal().segment<3>(6).setConstant(10);
    covRK4.diagonal().segment<3>(9).setConstant(0.01);
    covRK4.diagonal().tail<3>().setConstant(0.1);
    if (to_9d) {
      covRK4.diagonal().segment<3>(9).setConstant(0.0);
      covRK4.diagonal().tail<3>().setConstant(0.0);
    }
  }
  jacobianRK4.setIdentity();

  Eigen::Matrix<double,15,15> Sigma_s = covRK4;

  std::cout << "IMU meas_begin_time " << cpc.get_meas_begin_time()
            << " meas_end_time " << cpc.get_meas_end_time() << std::endl;
  {
    const std::string baseOutputPath = "./";
    const std::string imuFilename =
        baseOutputPath + "imu_data_" + tag + "_" + std::to_string(seed) + ".txt";
    const std::string configFilename =
        baseOutputPath + "cpc_config_" + tag + "_" + std::to_string(seed) + ".yaml";

    std::ofstream imuOfs(imuFilename, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!imuOfs.is_open()) {
      std::cerr << "Failed to open " << imuFilename << " for writing\n";
    } else {
      char line[256];
      for (const auto& m : cpc.get_imu_measurements()) {
        const auto& ts = m.timeStamp;
        const auto& gyr = m.measurement.gyroscopes;
        const auto& acc = m.measurement.accelerometers;
        const int len = std::snprintf(
            line, sizeof(line), "%u.%09u %.15g %.15g %.15g %.15g %.15g %.15g\n",
            ts.sec, ts.nsec, gyr.x(), gyr.y(), gyr.z(), acc.x(), acc.y(), acc.z());
        imuOfs.write(line, len);
      }
    }

    std::ofstream cfgOfs(configFilename, std::ios::out | std::ios::trunc);
    if (!cfgOfs.is_open()) {
      std::cerr << "Failed to open " << configFilename << " for writing\n";
    } else {
      cfgOfs.setf(std::ios::fixed);
      cfgOfs << std::setprecision(18);
      cfgOfs << "biases:\n";
      cfgOfs << "  gyro: [" << bias.head<3>().x() << ", " << bias.head<3>().y()
             << ", " << bias.head<3>().z() << "]\n";
      cfgOfs << "  accel: [" << bias.tail<3>().x() << ", " << bias.tail<3>().y()
             << ", " << bias.tail<3>().z() << "]\n";
      cfgOfs << "imu_params:\n";
      cfgOfs << "  g_max: " << imu_params.g_max << "\n";
      cfgOfs << "  a_max: " << imu_params.a_max << "\n";
      cfgOfs << "  sigma_g_c: " << imu_params.sigma_g_c << "\n";
      cfgOfs << "  sigma_a_c: " << imu_params.sigma_a_c << "\n";
      cfgOfs << "  sigma_gw_c: " << imu_params.sigma_gw_c << "\n";
      cfgOfs << "  sigma_aw_c: " << imu_params.sigma_aw_c << "\n";
      cfgOfs << "  g: " << imu_params.g << "\n";
      cfgOfs << "  rate: " << imu_params.rate << "\n";
      cfgOfs << "  tau: " << imu_params.tau << "\n";
      cfgOfs << "  dt: " << cpc.get_dt() << "\n";
      cfgOfs << "gravity: [0, 0, " << -cpc.get_g() << "]\n";
    }
  }
  swift_vio::Imu_BG_BA imuModel;
  imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
  (void)swift_vio::ImuOdometry::propagation_RungeKutta(
      cpc.get_imu_measurements(), imu_params, T_WS, speed,
      imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time(),
      &covRK4, &jacobianRK4);
  timer.stop();

  // ---- initial mean state ----
  gtsam::Rot3 Rws(cpc.get_q_WS0());
  Eigen::Vector3d pws = cpc.get_p_WS_W0();
  Eigen::Vector3d vws = cpc.get_v_WS0();

  // ---- params ----
  gtsam::Vector3 n_gravity(0, 0, -cpc.get_g());
  auto pimParams = std::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(n_gravity);
  pimParams->gyroscopeCovariance      = std::pow(imu_params.sigma_g_c, 2.0) * Eigen::Matrix3d::Identity();
  pimParams->accelerometerCovariance  = std::pow(imu_params.sigma_a_c, 2.0) * Eigen::Matrix3d::Identity();
  pimParams->integrationCovariance    = 1e-16 * Eigen::Matrix3d::Identity();
  pimParams->biasOmegaCovariance      = (imu_params.sigma_gw_c * imu_params.sigma_gw_c) * gtsam::I_3x3;
  pimParams->biasAccCovariance        = (imu_params.sigma_aw_c * imu_params.sigma_aw_c) * gtsam::I_3x3;
  pimParams->use2ndOrderCoriolis      = false;
  pimParams->biasAccOmegaInt.setZero();

  gtsam::imuBias::ConstantBias bias_gtsam(bias.tail<3>(), bias.head<3>());

  // ---- construct pim (type depends on template argument) ----
  PimT pim(pimParams, bias_gtsam);

  // ---- integrate measurements ----
  const auto imuMeasurements = cpc.get_imu_measurements();
  okvis::Time t_start = imuMeasurements.front().timeStamp;
  okvis::Time t_end   = imuMeasurements.back().timeStamp;

  okvis::Time time = t_start;
  bool hasStarted = false;
  auto it = imuMeasurements.begin();

  for (size_t k = 0; k + 1 < imuMeasurements.size(); ++k, ++it) {
    const auto& m0 = imuMeasurements[k];
    const auto& m1 = imuMeasurements[k + 1];

    Eigen::Vector3d omega0 = m0.measurement.gyroscopes;
    Eigen::Vector3d acc0   = m0.measurement.accelerometers;
    Eigen::Vector3d omega1 = m1.measurement.gyroscopes;
    Eigen::Vector3d acc1   = m1.measurement.accelerometers;

    okvis::Time nexttime = ((it + 1) == imuMeasurements.end()) ? t_end : (it + 1)->timeStamp;
    double dt = (nexttime - time).toSec();

    if (t_end < nexttime) {
      double interval = (nexttime - it->timeStamp).toSec();
      nexttime = t_end;
      dt = (nexttime - time).toSec();
      const double r = dt / interval;
      omega1 = ((1.0 - r) * omega0 + r * omega1).eval();
      acc1   = ((1.0 - r) * acc0   + r * acc1  ).eval();
    }
    if (dt <= 0.0) continue;

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / (nexttime - it->timeStamp).toSec();
      omega0 = (r * omega0 + (1.0 - r) * omega1).eval();
      acc0   = (r * acc0   + (1.0 - r) * acc1  ).eval();
    }

    const Eigen::Vector3d omega = 0.5 * (omega0 + omega1);
    const Eigen::Vector3d acc   = 0.5 * (acc0   + acc1);

    assert(std::fabs(dt) > 1e-9);
    pim.integrateMeasurement(acc, omega, dt);

    time = nexttime;
    if (nexttime == t_end) break;
  }

  // ---- predict and extract increments ----
  gtsam::NavState s0(Rws, pws, vws);
  gtsam::NavState s1 = pim.predict(s0, bias_gtsam);

  gtsam::Rot3 dR_gs = pim.deltaRij();
  Eigen::Matrix3d dR = dR_gs.matrix();
  Eigen::Vector3d dP = pim.deltaPij();
  Eigen::Vector3d dV = pim.deltaVij();
  double DT = pim.deltaTij();

  // ---- sanity: mean reconstruction check ----
  // These recon quantities assume biases are zero.
  Eigen::Vector3d p_recon = pws + vws * DT + 0.5 * n_gravity * DT * DT + Rws * dP;
  gtsam::Rot3 R_recon = Rws * dR_gs;
  Eigen::Vector3d v_recon = vws + n_gravity * DT + Rws * dV;
  std::cout << "P_recon " << p_recon.transpose() << ", s1 position " << s1.position().transpose() 
      << ", rk4 " << T_WS.r().transpose() << std::endl;
  std::cout << "v_recon " << v_recon.transpose() << ", s1 velocity " << s1.velocity().transpose() 
      << ", rk4 " << speed.head<3>().transpose() << std::endl;

  std::cout << "R_recon " << R_recon.toQuaternion().coeffs().transpose()
          << ", s1 quat " << s1.attitude().toQuaternion().coeffs().transpose()
          << ", rk4 " << T_WS.q().coeffs().transpose() << std::endl;

  expectNearAbsRel(p_recon, T_WS.r(), 1e-4, 1e-2);
  ::check_q_near(R_recon.toQuaternion(), T_WS.q(), 1e-5);
  expectNearAbsRel(v_recon, speed.head<3>(), 1e-4, 1e-2);

  // ---- bias jacobian: Manifold vs Tangent differ ----
  Eigen::Matrix<double,9,6> JincBias_bg_ba;
  if constexpr (std::is_same_v<PimT, gtsam::PreintegratedCombinedMeasurementsT<gtsam::ManifoldPreintegration>>) {
    JincBias_bg_ba = swapBias(BuildJincBias_ba_bg(
        pim.delRdelBiasOmega(),
        pim.delPdelBiasAcc(),
        pim.delPdelBiasOmega(),
        pim.delVdelBiasAcc(),
        pim.delVdelBiasOmega()));
  } else {
    JincBias_bg_ba << pim.preintegrated_H_biasOmega(), pim.preintegrated_H_biasAcc();
  }

  Eigen::Matrix<double,15,15> Sigma_z = pim.preintMeasCov();

  Maps15 maps;
  if (use_manifold)
    maps = BuildMaps15_Manifold(Rws.matrix(), dR, dP, dV, DT);
  else
    maps = BuildMaps15_Tangent(Rws.matrix(), dR, dP, dV, DT);

  Eigen::Matrix<double,15,15> Jac_e_preint = PropJac15_Combined(maps, JincBias_bg_ba);

  Eigen::Matrix<double,15,15> Sigma_e_preint = PropCov15_Combined(maps, Sigma_s, Jac_e_preint, Sigma_z);

  std::cout << "Comparing jacobians\n";
  std::cout << "jacRK4\n" << jacobianRK4 << "\njacPreint\n" << Jac_e_preint << std::endl;
  expectNearAbsRel(jacobianRK4, Jac_e_preint, 1e-4, 2e-2);

  std::cout << "Comparing covariance\n";
  std::cout << "covRK4\n" << covRK4 << "\ncovGtsam\n" << Sigma_e_preint << std::endl;
  expectNearAbsRel(covRK4, Sigma_e_preint, 1e-4, 2e-2);
  std::string baseOutputPath = "./";
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
  saveMatrix("jacobianGtsam" + tag, Jac_e_preint);
  saveMatrix("covRK4", covRK4);
  saveMatrix("covGtsam" + tag, Sigma_e_preint);

  return 0;
}

int PropagationByPreintegration(bool use_manifold, bool zero_cov, bool zero_bias) {
  if (use_manifold) {
    using PimT = gtsam::PreintegratedCombinedMeasurementsT<gtsam::ManifoldPreintegration>;
    return PropagationByPreintegrationImpl<PimT>(true, zero_cov, zero_bias);
  } else {
    using PimT = gtsam::PreintegratedCombinedMeasurementsT<gtsam::TangentPreintegration>;
    return PropagationByPreintegrationImpl<PimT>(false, zero_cov, zero_bias);
  }
}

template <typename PimT>
static int PreintegrationByPropagationImpl(bool use_manifold) {
  std::cout << "\n=== TEST: PreintegrationByPropagation ===\n";
  std::cout << "Preintegration: " << (use_manifold ? "Manifold" : "Tangent") << "\n";

  int seed = 0;
  CovPropConfig<swift_vio::Imu_BG_BA> cpc(false, true, seed);

  okvis::kinematics::Transformation T_WS(cpc.get_p_WS_W0(), cpc.get_q_WS0());
  Eigen::Vector3d speed = cpc.get_v_WS0();
  Eigen::Matrix<double,6,1> bias = cpc.get_bias0();
  std::cout << "bias_s " << bias.transpose() << std::endl;

  // --- RK4 with zero start cov ---
  Eigen::Matrix<double,15,15> covRK4 = Eigen::Matrix<double,15,15>::Zero();
  Eigen::Matrix<double,15,15> jacobianRK4 = Eigen::Matrix<double,15,15>::Identity();

  swift_vio::Imu_BG_BA imuModel;
  okvis::ImuParameters imu_params = cpc.get_imu_params();
  imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
  (void)swift_vio::ImuOdometry::propagation_RungeKutta(
      cpc.get_imu_measurements(), imu_params, T_WS, speed,
      imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time(),
      &covRK4, &jacobianRK4);

  // ---- initial mean state ----
  gtsam::Rot3 Rws(cpc.get_q_WS0());
  Eigen::Vector3d pws = cpc.get_p_WS_W0();
  Eigen::Vector3d vws = cpc.get_v_WS0();

  // ---- params ----
  gtsam::Vector3 n_gravity(0, 0, -cpc.get_g());
  auto pimParams = std::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(n_gravity);
  pimParams->gyroscopeCovariance      = std::pow(imu_params.sigma_g_c, 2.0) * Eigen::Matrix3d::Identity();
  pimParams->accelerometerCovariance  = std::pow(imu_params.sigma_a_c, 2.0) * Eigen::Matrix3d::Identity();
  pimParams->integrationCovariance    = 1e-16 * Eigen::Matrix3d::Identity();
  pimParams->biasOmegaCovariance      = (imu_params.sigma_gw_c * imu_params.sigma_gw_c) * gtsam::I_3x3;
  pimParams->biasAccCovariance        = (imu_params.sigma_aw_c * imu_params.sigma_aw_c) * gtsam::I_3x3;
  pimParams->use2ndOrderCoriolis      = false;
  pimParams->biasAccOmegaInt.setZero();

  gtsam::imuBias::ConstantBias bias_gtsam(bias.tail<3>(), bias.head<3>());

  PimT pim(pimParams, bias_gtsam);

  const auto imuMeasurements = cpc.get_imu_measurements();
  okvis::Time t_start = imuMeasurements.front().timeStamp;
  okvis::Time t_end   = imuMeasurements.back().timeStamp;

  okvis::Time time = t_start;
  bool hasStarted = false;
  auto it = imuMeasurements.begin();

  for (size_t k = 0; k + 1 < imuMeasurements.size(); ++k, ++it) {
    const auto& m0 = imuMeasurements[k];
    const auto& m1 = imuMeasurements[k + 1];

    Eigen::Vector3d omega0 = m0.measurement.gyroscopes;
    Eigen::Vector3d acc0   = m0.measurement.accelerometers;
    Eigen::Vector3d omega1 = m1.measurement.gyroscopes;
    Eigen::Vector3d acc1   = m1.measurement.accelerometers;

    okvis::Time nexttime = ((it + 1) == imuMeasurements.end()) ? t_end : (it + 1)->timeStamp;
    double dt = (nexttime - time).toSec();

    if (t_end < nexttime) {
      double interval = (nexttime - it->timeStamp).toSec();
      nexttime = t_end;
      dt = (nexttime - time).toSec();
      const double r = dt / interval;
      omega1 = ((1.0 - r) * omega0 + r * omega1).eval();
      acc1   = ((1.0 - r) * acc0   + r * acc1  ).eval();
    }
    if (dt <= 0.0) continue;

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / (nexttime - it->timeStamp).toSec();
      omega0 = (r * omega0 + (1.0 - r) * omega1).eval();
      acc0   = (r * acc0   + (1.0 - r) * acc1  ).eval();
    }

    const Eigen::Vector3d omega = 0.5 * (omega0 + omega1);
    const Eigen::Vector3d acc   = 0.5 * (acc0   + acc1);

    assert(std::fabs(dt) > 1e-9);
    pim.integrateMeasurement(acc, omega, dt);

    time = nexttime;
    if (nexttime == t_end) break;
  }

  gtsam::NavState s0(Rws, pws, vws);
  (void)pim.predict(s0, bias_gtsam);

  gtsam::Rot3 dR_gs = pim.deltaRij();
  Eigen::Matrix3d dR = dR_gs.matrix();
  Eigen::Vector3d dP = pim.deltaPij();
  Eigen::Vector3d dV = pim.deltaVij();
  double DT = pim.deltaTij();
  Maps15 maps;
  if (use_manifold)
    maps = BuildMaps15_Manifold(Rws.matrix(), dR, dP, dV, DT);
  else
    maps = BuildMaps15_Tangent(Rws.matrix(), dR, dP, dV, DT);

  const Eigen::Matrix<double,9,9> G9_inv = maps.G_inv.topLeftCorner<9, 9>();
  Eigen::Matrix<double,9,6> JincBias_bg_ba_rk4 =
      G9_inv * jacobianRK4.topRightCorner<9, 6>();

  Eigen::Matrix<double,6,6> P = Eigen::Matrix<double,6,6>::Zero();
  P.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  P.block<3,3>(3,0) = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double,9,6> JincBias_ba_bg_rk4 =
      JincBias_bg_ba_rk4 * P.transpose();

  Eigen::Matrix<double,15,15> Sigma_z_rk4 =
      maps.covG_inv * covRK4 * maps.covG_inv.transpose();

  Eigen::Matrix<double,9,6> JincBias_ba_bg_gtsam;
  if constexpr (std::is_same_v<PimT, gtsam::PreintegratedCombinedMeasurementsT<gtsam::ManifoldPreintegration>>) {
    JincBias_ba_bg_gtsam = BuildJincBias_ba_bg(
        pim.delRdelBiasOmega(),
        pim.delPdelBiasAcc(),
        pim.delPdelBiasOmega(),
        pim.delVdelBiasAcc(),
        pim.delVdelBiasOmega());
  } else {
    JincBias_ba_bg_gtsam << pim.preintegrated_H_biasAcc(), pim.preintegrated_H_biasOmega();
  }

  Eigen::Matrix<double,15,15> Sigma_z_gtsam = pim.preintMeasCov();

  std::cout << "Comparing bias jacobians" << std::endl;
  std::cout << "JincBias_ba_bg_rk4:\n" << JincBias_ba_bg_rk4 << std::endl;
  std::cout << "JincBias_ba_bg_gtsam:\n" << JincBias_ba_bg_gtsam << std::endl;

  expectNearAbsRel(JincBias_ba_bg_rk4, JincBias_ba_bg_gtsam, 1e-4, 1e-2);

  std::cout << "Comparing preintegration covariance" << std::endl;
  std::cout << "Sigma_z_rk4:\n" << Sigma_z_rk4 << std::endl;
  std::cout << "Sigma_z_gtsam:\n" << Sigma_z_gtsam << std::endl;
  expectNearAbsRel(Sigma_z_rk4, Sigma_z_gtsam, 1e-4, 1e-2);

  return 0;
}

int PreintegrationByPropagation(bool use_manifold) {
  if (use_manifold) {
    using PimT = gtsam::PreintegratedCombinedMeasurementsT<gtsam::ManifoldPreintegration>;
    return PreintegrationByPropagationImpl<PimT>(true);
  } else {
    using PimT = gtsam::PreintegratedCombinedMeasurementsT<gtsam::TangentPreintegration>;
    return PreintegrationByPropagationImpl<PimT>(false);
  }
}

static inline void computePhiAndCovTangent(
    const gtsam::PreintegratedImuMeasurementsT<gtsam::TangentPreintegration>& pim,
    const gtsam::Rot3& Rws,
    Eigen::Matrix<double,9,9>* jac,
    Eigen::Matrix<double,9,9>* cov) {
  gtsam::Rot3 dR_gs = pim.deltaRij();
  Eigen::Matrix3d dR = dR_gs.matrix();
  Eigen::Vector3d dP = pim.deltaPij();
  Eigen::Vector3d dV = pim.deltaVij();
  double DT = pim.deltaTij();

  Maps15 maps = BuildMaps15_Tangent(Rws.matrix(), dR, dP, dV, DT);

  *jac = maps.F.topLeftCorner<9,9>();
  const Eigen::Matrix<double,9,9> G9 = maps.covG.topLeftCorner<9,9>();
  const auto Sigma_z = pim.preintMeasCov();
  const Eigen::Matrix<double,9,9> Sigma_s = *cov;
  cov->noalias() = (*jac) * Sigma_s * jac->transpose();
  cov->noalias() += G9 * Sigma_z * G9.transpose();
}

static inline void computePhiAndCovManifold(
    const gtsam::PreintegratedImuMeasurementsT<gtsam::ManifoldPreintegration>& pim,
    const gtsam::Rot3& Rws,
    Eigen::Matrix<double,9,9>* jac,
    Eigen::Matrix<double,9,9>* cov) {
  gtsam::Rot3 dR_gs = pim.deltaRij();
  Eigen::Matrix3d dR = dR_gs.matrix();
  Eigen::Vector3d dP = pim.deltaPij();
  Eigen::Vector3d dV = pim.deltaVij();
  double DT = pim.deltaTij();

  Maps15 maps = BuildMaps15_Manifold(Rws.matrix(), dR, dP, dV, DT);

  *jac = maps.F.topLeftCorner<9,9>();
  const Eigen::Matrix<double,9,9> G9 = maps.covG.topLeftCorner<9,9>();
  const auto Sigma_z = pim.preintMeasCov();
  const Eigen::Matrix<double,9,9> Sigma_s = *cov;
  cov->noalias() = (*jac) * Sigma_s * jac->transpose();
  cov->noalias() += G9 * Sigma_z * G9.transpose();
}

static inline Eigen::Matrix<double,9,9> JacNavstateToOkvis(
    const Eigen::Matrix3d& Rws) {
  Eigen::Matrix<double,9,9> J = Eigen::Matrix<double,9,9>::Zero();
  // navstate error: [dtheta, dp, dv] in body frame (right perturbation SE_2(3))
  // okvis error:    [dp, dtheta, dv] in world frame (left perturbation R3xSO3xR3)
  J.block<3,3>(0,3) = Rws;
  J.block<3,3>(3,0) = Rws;
  J.block<3,3>(6,6) = Rws;
  return J;
}

static inline Eigen::Matrix<double,9,9> JacOkvisToNavstate(
    const Eigen::Matrix3d& Rws) {
  Eigen::Matrix<double,9,9> J = Eigen::Matrix<double,9,9>::Zero();
  const Eigen::Matrix3d Rsw = Rws.transpose();
  J.block<3,3>(0,3) = Rsw;
  J.block<3,3>(3,0) = Rsw;
  J.block<3,3>(6,6) = Rsw;
  return J;
}

static inline void computePhiAndCovEkf(
    const Eigen::Matrix<double,9,9>& jac_nav,
    const Eigen::Matrix<double,9,9>& cov_nav,
    const gtsam::Rot3& Rws_start,
    const gtsam::Rot3& Rws_end,
    Eigen::Matrix<double,9,9>* jac_okvis,
    Eigen::Matrix<double,9,9>* cov_okvis) {
  const Eigen::Matrix<double,9,9> J_s_inv = JacOkvisToNavstate(Rws_start.matrix());
  const Eigen::Matrix<double,9,9> J_e     = JacNavstateToOkvis(Rws_end.matrix());
  jac_okvis->noalias() = J_e * jac_nav * J_s_inv;
  cov_okvis->noalias() = J_e * cov_nav * J_e.transpose();
}

int Gtsam9DPropagations(bool zero_cov) {
  std::cout << "\n=== TEST: Gtsam9DPropagations ===\n";

  int seed = 0;
  CovPropConfig<swift_vio::Imu_BG_BA> cpc(false, true, seed);
  cpc.zeroBias0();

  okvis::kinematics::Transformation T_WS(cpc.get_p_WS_W0(), cpc.get_q_WS0());
  Eigen::Vector3d speed = cpc.get_v_WS0();
  Eigen::Matrix<double,6,1> bias = cpc.get_bias0();

  Eigen::Matrix<double,9,9> cov0 = Eigen::Matrix<double,9,9>::Zero();
  if (!zero_cov) {
    cov0.diagonal().head<3>().setConstant(100);
    cov0.diagonal().segment<3>(3).setConstant(1);
    cov0.diagonal().tail<3>().setConstant(10);
  }

  okvis::ImuParameters imu_params = cpc.get_imu_params();
  imu_params.sigma_gw_c = 0.0;
  imu_params.sigma_aw_c = 0.0;

  Eigen::Matrix<double,15,15> covRK4 = Eigen::Matrix<double,15,15>::Zero();
  Eigen::Matrix<double,15,15> jacobianRK4 = Eigen::Matrix<double,15,15>::Zero();
  covRK4.topLeftCorner<9,9>() = cov0;
  jacobianRK4.topLeftCorner<9,9>().setIdentity();

  swift_vio::Imu_BG_BA imuModel;
  imuModel.updateParameters(bias.data(), cpc.getImuExtraParamPtr());
  (void)swift_vio::ImuOdometry::propagation_RungeKutta(
      cpc.get_imu_measurements(), imu_params, T_WS, speed,
      imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time(),
      &covRK4, &jacobianRK4);

  Eigen::Matrix<double,9,9> covRK4_9 = covRK4.topLeftCorner<9,9>();
  Eigen::Matrix<double,9,9> jacRK4_9 = jacobianRK4.topLeftCorner<9,9>();

  gtsam::Rot3 Rws(cpc.get_q_WS0());
  Eigen::Vector3d pws = cpc.get_p_WS_W0();
  Eigen::Vector3d vws = cpc.get_v_WS0();
  gtsam::NavState s0(Rws, pws, vws);
  gtsam::imuBias::ConstantBias bias_gtsam(bias.tail<3>(), bias.head<3>());

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  gtsam::Vector3 n_gravity(0, 0, -cpc.get_g());
  auto pimParams =
      std::make_shared<gtsam::PreintegratedImuMeasurements::Params>(n_gravity);
  pimParams->gyroscopeCovariance     = std::pow(imu_params.sigma_g_c, 2.0) * I;
  pimParams->accelerometerCovariance = std::pow(imu_params.sigma_a_c, 2.0) * I;
  pimParams->integrationCovariance   = 1e-16 * I;
  pimParams->use2ndOrderCoriolis     = false;

  typedef gtsam::PreintegratedImuMeasurementsT<gtsam::TangentPreintegration> PreintegratedImuMeasurementsTangent;
  typedef gtsam::PreintegratedImuMeasurementsT<gtsam::ManifoldPreintegration> PreintegratedImuMeasurementsManifold;

  PreintegratedImuMeasurementsTangent pim_tangent(pimParams, bias_gtsam);
  PreintegratedImuMeasurementsManifold pim_manifold(pimParams, bias_gtsam);

  const Eigen::Matrix<double,9,9> J_okvis_to_nav = JacOkvisToNavstate(Rws.matrix());
  Eigen::Matrix<double,9,9> cov0_nav;
  cov0_nav.noalias() = J_okvis_to_nav * cov0 * J_okvis_to_nav.transpose();
  gtsam::NavStateImuEKF ekf(s0, cov0_nav, pimParams);
  Eigen::Matrix<double,9,9> jac_ekf = Eigen::Matrix<double,9,9>::Identity();

  const auto imuMeasurements = cpc.get_imu_measurements();
  okvis::Time t_start = imuMeasurements.front().timeStamp;
  okvis::Time t_end   = imuMeasurements.back().timeStamp;

  okvis::Time time = t_start;
  bool hasStarted = false;
  auto it = imuMeasurements.begin();

  for (size_t k = 0; k + 1 < imuMeasurements.size(); ++k, ++it) {
    const auto& m0 = imuMeasurements[k];
    const auto& m1 = imuMeasurements[k + 1];

    Eigen::Vector3d omega0 = m0.measurement.gyroscopes;
    Eigen::Vector3d acc0   = m0.measurement.accelerometers;
    Eigen::Vector3d omega1 = m1.measurement.gyroscopes;
    Eigen::Vector3d acc1   = m1.measurement.accelerometers;

    okvis::Time nexttime = ((it + 1) == imuMeasurements.end()) ? t_end : (it + 1)->timeStamp;
    double dt = (nexttime - time).toSec();

    if (t_end < nexttime) {
      double interval = (nexttime - it->timeStamp).toSec();
      nexttime = t_end;
      dt = (nexttime - time).toSec();
      const double r = dt / interval;
      omega1 = ((1.0 - r) * omega0 + r * omega1).eval();
      acc1   = ((1.0 - r) * acc0   + r * acc1  ).eval();
    }
    if (dt <= 0.0) continue;

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / (nexttime - it->timeStamp).toSec();
      omega0 = (r * omega0 + (1.0 - r) * omega1).eval();
      acc0   = (r * acc0   + (1.0 - r) * acc1  ).eval();
    }

    const Eigen::Vector3d omega = 0.5 * (omega0 + omega1);
    const Eigen::Vector3d acc   = 0.5 * (acc0   + acc1);

    pim_tangent.integrateMeasurement(acc, omega, dt);
    pim_manifold.integrateMeasurement(acc, omega, dt);

    gtsam::NavStateImuEKF::Jacobian A;
    ekf.Dynamics(pimParams->n_gravity, ekf.state(), omega, acc, dt, A);
    jac_ekf = A * jac_ekf;
    ekf.predict(omega, acc, dt);

    time = nexttime;
    if (nexttime == t_end) break;
  }

  Eigen::Matrix<double,9,9> jac_tangent, cov_tangent = cov0;
  Eigen::Matrix<double,9,9> jac_manifold, cov_manifold = cov0;
  computePhiAndCovTangent(pim_tangent, Rws, &jac_tangent, &cov_tangent);
  computePhiAndCovManifold(pim_manifold, Rws, &jac_manifold, &cov_manifold);
  const gtsam::Rot3 Rws_end = ekf.state().attitude();
  Eigen::Matrix<double,9,9> jac_ekf_okvis, cov_ekf_okvis;
  computePhiAndCovEkf(jac_ekf, ekf.covariance(), Rws, Rws_end,
                      &jac_ekf_okvis, &cov_ekf_okvis);

  std::cout << "\nComparing 9D Jacobians (RK4 vs GTSAM tangent)\n";
  std::cout << "jacRK4_9\n" << jacRK4_9
            << "\njacGtsamTangent_9\n" << jac_tangent << '\n';
  expectNearAbsRel(jacRK4_9, jac_tangent, 1e-4, 2e-2);
  std::cout << "\nComparing 9D Covariances (RK4 vs GTSAM tangent)\n";
  std::cout << "covRK4_9\n" << covRK4_9
            << "\ncovGtsamTangent_9\n" << cov_tangent << '\n';
  expectNearAbsRel(covRK4_9, cov_tangent, 1e-4, 2e-2);

  std::cout << "\nComparing 9D Jacobians (RK4 vs GTSAM manifold)\n";
  std::cout << "jacRK4_9\n" << jacRK4_9
            << "\njacGtsamManifold_9\n" << jac_manifold << '\n';
  expectNearAbsRel(jacRK4_9, jac_manifold, 1e-4, 2e-2);
  std::cout << "\nComparing 9D Covariances (RK4 vs GTSAM manifold)\n";
  std::cout << "covRK4_9\n" << covRK4_9
            << "\ncovGtsamManifold_9\n" << cov_manifold << '\n';
  expectNearAbsRel(covRK4_9, cov_manifold, 1e-4, 2e-2);

  std::cout << "\nComparing 9D Jacobians (RK4 vs GTSAM EKF)\n";
  std::cout << "jacRK4_9\n" << jacRK4_9
            << "\njacGtsamEkf_9\n" << jac_ekf_okvis << '\n';
  expectNearAbsRel(jacRK4_9, jac_ekf_okvis, 1e-4, 2e-2);
  std::cout << "\nComparing 9D Covariances (RK4 vs GTSAM EKF)\n";
  std::cout << "covRK4_9\n" << covRK4_9
            << "\ncovGtsamEkf_9\n" << cov_ekf_okvis << '\n';
  expectNearAbsRel(covRK4_9, cov_ekf_okvis, 1e-4, 2e-2);

  return 0;
}

int main(int argc, char** argv) {
  bool use_manifold = true;
  bool run_prop_by_preint = true;
  bool run_preint_by_prop = true;
  bool run_gtsam9d = false;
  bool mode_specified = false;
  bool zero_cov = false;
  bool zero_bias = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "manifold") {
      use_manifold = true;
    } else if (arg == "tangent") {
      use_manifold = false;
    } else if (arg == "prop_by_preint" || arg == "prop-by-preint" || arg == "prop") {
      if (!mode_specified) {
        run_prop_by_preint = false;
        run_preint_by_prop = false;
        mode_specified = true;
      }
      run_prop_by_preint = true;
    } else if (arg == "preint_by_prop" || arg == "preint-by-prop" || arg == "preint") {
      if (!mode_specified) {
        run_prop_by_preint = false;
        run_preint_by_prop = false;
        run_gtsam9d = false;
        mode_specified = true;
      }
      run_preint_by_prop = true;
    } else if (arg == "all") {
      run_prop_by_preint = true;
      run_preint_by_prop = true;
      run_gtsam9d = true;
      mode_specified = true;
    } else if (arg == "gtsam9d" || arg == "gtsam-9d" || arg == "gtsam_9d") {
      if (!mode_specified) {
        run_prop_by_preint = false;
        run_preint_by_prop = false;
        run_gtsam9d = false;
        mode_specified = true;
      }
      run_gtsam9d = true;
    } else if (arg == "zero_cov" || arg == "zero-cov") {
      zero_cov = true;
    } else if (arg == "zero_bias" || arg == "zero-bias") {
      zero_bias = true;
    } else {
      std::cout << "Usage: " << argv[0]
                << " [manifold|tangent] [all|prop_by_preint|preint_by_prop|gtsam9d] [zero_cov] [zero_bias]\n";
      return 1;
    }
  }

  if (run_prop_by_preint) {
    PropagationByPreintegration(use_manifold, zero_cov, zero_bias);
  }
  if (run_preint_by_prop) {
    PreintegrationByPropagation(use_manifold);
  }
  if (run_gtsam9d) {
    Gtsam9DPropagations(zero_cov);
  }
  return 0;
}