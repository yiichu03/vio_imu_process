#include <Eigen/Dense>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
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

struct Maps15 {
  Eigen::Matrix<double,15,15> F;    // df_e(x_s, Dx)/dx_s (OKVIS error state, x=[p,q,v,bg,ba])
  Eigen::Matrix<double,15,15> G;    // dx_e/dz for transition matrix (z=[dtheta,dp,dv,dba,dbg] of gtsam)
  Eigen::Matrix<double,15,15> covG; // dx_e/dz for covariance (z=[dtheta,dp,dv,dba,dbg] of gtsam)
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
  // End-state OKVIS [dp, dtheta, dv, dbg, dba] defined in left perturbation sense.
  Eigen::Matrix3d Rwe_from_s = Rws * dR;
  // G for computing covariance because the cov error state is defined like
  // \Delta X_{se} = \Delta\hat{X}_{se} \boxplus [\delta \theta_{se}, \delta p_{se}, \delta v_{se}]
  // where \boxplus is defined as a NavState boxplus.
  // With this, we note that for gtsam 4.1.1, the cov error state is different from the residual error state, 
  // thus the CombinedImuFactor weighting is wrong. 
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
static int PropagationByPreintegrationImpl(bool use_manifold, bool zero_cov) {
  std::cout << "\n=== TEST: PropagationByPreintegration ===\n";
  std::string tag = use_manifold ? "Manifold" : "Tangent";
  std::cout << "Preintegration: " << (use_manifold ? "Manifold" : "Tangent") << "\n";

  int seed = 0;
  CovPropConfig<swift_vio::Imu_BG_BA> cpc(false, true, seed);

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
      cfgOfs << "  g_max: " << cpc.get_imu_params().g_max << "\n";
      cfgOfs << "  a_max: " << cpc.get_imu_params().a_max << "\n";
      cfgOfs << "  sigma_g_c: " << cpc.get_imu_params().sigma_g_c << "\n";
      cfgOfs << "  sigma_a_c: " << cpc.get_imu_params().sigma_a_c << "\n";
      cfgOfs << "  sigma_gw_c: " << cpc.get_imu_params().sigma_gw_c << "\n";
      cfgOfs << "  sigma_aw_c: " << cpc.get_imu_params().sigma_aw_c << "\n";
      cfgOfs << "  g: " << cpc.get_imu_params().g << "\n";
      cfgOfs << "  rate: " << cpc.get_imu_params().rate << "\n";
      cfgOfs << "  tau: " << cpc.get_imu_params().tau << "\n";
      cfgOfs << "  dt: " << cpc.get_dt() << "\n";
      cfgOfs << "gravity: [0, 0, " << -cpc.get_g() << "]\n";
    }
  }
  swift_vio::Imu_BG_BA imuModel;
  okvis::ImuParameters imu_params = cpc.get_imu_params();
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

int PropagationByPreintegration(bool use_manifold, bool zero_cov) {
  if (use_manifold) {
    using PimT = gtsam::PreintegratedCombinedMeasurementsT<gtsam::ManifoldPreintegration>;
    return PropagationByPreintegrationImpl<PimT>(true, zero_cov);
  } else {
    using PimT = gtsam::PreintegratedCombinedMeasurementsT<gtsam::TangentPreintegration>;
    return PropagationByPreintegrationImpl<PimT>(false, zero_cov);
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
  // -=======================输出了两个矩阵：covRK4和jacobianRK4
  (void)swift_vio::ImuOdometry::propagation_RungeKutta(
      cpc.get_imu_measurements(), imu_params, T_WS, speed,
      imuModel, cpc.get_meas_begin_time(), cpc.get_meas_end_time(),
      &covRK4, &jacobianRK4);

  // ---- initial mean state ----
  gtsam::Rot3 Rws(cpc.get_q_WS0());
  // 这里重新拿一遍初始 R,p,v，但用 GTSAM 的类型承载：
  Eigen::Vector3d pws = cpc.get_p_WS_W0();
  Eigen::Vector3d vws = cpc.get_v_WS0();

  // ---- params ----
  // n_gravity：重力向量（世界系下），这里是 [0,0,-g]。
  gtsam::Vector3 n_gravity(0, 0, -cpc.get_g());
  auto pimParams = std::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(n_gravity);
  pimParams->gyroscopeCovariance      = std::pow(imu_params.sigma_g_c, 2.0) * Eigen::Matrix3d::Identity();
  pimParams->accelerometerCovariance  = std::pow(imu_params.sigma_a_c, 2.0) * Eigen::Matrix3d::Identity();
  pimParams->integrationCovariance    = 1e-16 * Eigen::Matrix3d::Identity();
  pimParams->biasOmegaCovariance      = (imu_params.sigma_gw_c * imu_params.sigma_gw_c) * gtsam::I_3x3;
  pimParams->biasAccCovariance        = (imu_params.sigma_aw_c * imu_params.sigma_aw_c) * gtsam::I_3x3;
  pimParams->use2ndOrderCoriolis      = false;
  pimParams->biasAccOmegaInt.setZero();

  // bias 是 [bg, ba]（头3是gyro，尾3是acc） bias.tail<3>() 作为 accel bias，bias.head<3>() 作为 gyro bias
  gtsam::imuBias::ConstantBias bias_gtsam(bias.tail<3>(), bias.head<3>());

  PimT pim(pimParams, bias_gtsam);

  // imuMeasurements：一串离散 IMU 测量（每个带时间戳、gyro、acc）。
  const auto imuMeasurements = cpc.get_imu_measurements();
  // t_start/t_end：用序列首尾时间戳当积分区间。
  okvis::Time t_start = imuMeasurements.front().timeStamp;
  okvis::Time t_end   = imuMeasurements.back().timeStamp;

  okvis::Time time = t_start;
  bool hasStarted = false;
  auto it = imuMeasurements.begin();

  // 积分主循环：对每一段做梯形法（中值）积分，并处理起末端插值
  for (size_t k = 0; k + 1 < imuMeasurements.size(); ++k, ++it) {
    // 每次取相邻两帧 IMU m0,m1，对区间做一次积分。
    const auto& m0 = imuMeasurements[k];
    const auto& m1 = imuMeasurements[k + 1];

    // 提取两端的陀螺与加计读数。
    Eigen::Vector3d omega0 = m0.measurement.gyroscopes;
    Eigen::Vector3d acc0   = m0.measurement.accelerometers;
    Eigen::Vector3d omega1 = m1.measurement.gyroscopes;
    Eigen::Vector3d acc1   = m1.measurement.accelerometers;

    // nexttime：默认用下一条 measurement 的时间戳；如果已经到末尾则用 t_end。
    // dt：当前要积分的时间长度 = nexttime - time（注意不是 m1.time - m0.time，而是按 time 游标推进）。
    okvis::Time nexttime = ((it + 1) == imuMeasurements.end()) ? t_end : (it + 1)->timeStamp;
    double dt = (nexttime - time).toSec();

    if (t_end < nexttime) {
      // 如果按正常下一帧时间戳会超过积分终点 t_end，就把末端截断到 t_end。
      double interval = (nexttime - it->timeStamp).toSec();
      nexttime = t_end;
      dt = (nexttime - time).toSec();
      const double r = dt / interval;
      omega1 = ((1.0 - r) * omega0 + r * omega1).eval();
      acc1   = ((1.0 - r) * acc0   + r * acc1  ).eval();
    }
    if (dt <= 0.0) continue;

    if (!hasStarted) {
      // 第一次积分时，time 可能不是严格等于 it->timeStamp（或者为了让梯形法更稳），这里通过一次插值让 omega0/acc0 与当前积分起点更匹配。
      hasStarted = true;
      const double r = dt / (nexttime - it->timeStamp).toSec();
      omega0 = (r * omega0 + (1.0 - r) * omega1).eval();
      acc0   = (r * acc0   + (1.0 - r) * acc1  ).eval();
    }

    // 取中值（两端平均）来近似区间内常值角速度/加速度。
    const Eigen::Vector3d omega = 0.5 * (omega0 + omega1);
    const Eigen::Vector3d acc   = 0.5 * (acc0   + acc1);

    assert(std::fabs(dt) > 1e-9);
    // 把该段（acc, omega, dt）喂给 GTSAM 预积分器。
    pim.integrateMeasurement(acc, omega, dt);

    time = nexttime; // 推进时间游标到下一个时刻。
    if (nexttime == t_end) break;
  }

  gtsam::NavState s0(Rws, pws, vws);
  // 触发 predict（主要是让预积分内部完成一致的更新），然后取出预积分增量
  // 调 predict：对 combined 预积分而言，这一步会使用预积分量预测末端状态。
  // 这里不关心返回的末端状态，所以 (void) 丢弃，但它确保内部量（有些实现会在 predict 时更新一些缓存）保持一致。
  (void)pim.predict(s0, bias_gtsam);

  gtsam::Rot3 dR_gs = pim.deltaRij();
  Eigen::Matrix3d dR = dR_gs.matrix();
  Eigen::Vector3d dP = pim.deltaPij();
  Eigen::Vector3d dV = pim.deltaVij();
  double DT = pim.deltaTij();
  Maps15 maps;
  if (use_manifold)
    maps = BuildMaps15_Manifold(Rws.matrix(), dR, dP, dV, DT);
  else  // 用这个
    maps = BuildMaps15_Tangent(Rws.matrix(), dR, dP, dV, DT);

  // topRightCorner<9,6>()：取出右上角 9×6 块
  const Eigen::Matrix<double,9,9> G9_inv = maps.G_inv.topLeftCorner<9, 9>();
  Eigen::Matrix<double,9,6> JincBias_bg_ba_rk4 =
      G9_inv * jacobianRK4.topRightCorner<9, 6>();

  // 把列顺序换成 GTSAM 常用拼接顺序 [ba, bg]
  Eigen::Matrix<double,6,6> P = Eigen::Matrix<double,6,6>::Zero();
  P.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  P.block<3,3>(3,0) = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double,9,6> JincBias_ba_bg_rk4 =
      JincBias_bg_ba_rk4 * P.transpose();
  
  // 从 RK4 的协方差反推 “预积分协方差” Sigma_z
  Eigen::Matrix<double,15,15> Sigma_z_rk4 =
      maps.covG_inv * covRK4 * maps.covG_inv.transpose();

  // 计算 GTSAM 自己输出的 bias Jacobian，并与反推的对比
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

  // 取出 GTSAM 的预积分协方差，并与反推的对比 1. ba_bg_gtsam    2. Sigma_z_gtsam 2%
  Eigen::Matrix<double,15,15> Sigma_z_gtsam = pim.preintMeasCov();

  // 打印两个 9×6 bias Jacobian：一个来自 RK4 反推（已经换成 [ba,bg]）；一个来自 GTSAM 输出
  std::cout << "Comparing bias jacobians" << std::endl;
  std::cout << "JincBias_ba_bg_rk4:\n" << JincBias_ba_bg_rk4 << std::endl;
  std::cout << "JincBias_ba_bg_gtsam:\n" << JincBias_ba_bg_gtsam << std::endl;

  expectNearAbsRel(JincBias_ba_bg_rk4, JincBias_ba_bg_gtsam, 1e-4, 1e-2);

  std::cout << "Comparing preintegration covariance" << std::endl;
  std::cout << "Sigma_z_rk4:\n" << Sigma_z_rk4 << std::endl;
  std::cout << "Sigma_z_gtsam:\n" << Sigma_z_gtsam << std::endl;
  expectNearAbsRel(Sigma_z_rk4, Sigma_z_gtsam, 1e-4, 1e-2); // compare

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


int main(int argc, char** argv) {
  bool use_manifold = true;
  bool run_prop_by_preint = true;
  bool run_preint_by_prop = true;
  bool mode_specified = false;
  bool zero_cov = false;

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
        mode_specified = true;
      }
      run_preint_by_prop = true;
    } else if (arg == "both") {
      run_prop_by_preint = true;
      run_preint_by_prop = true;
      mode_specified = true;
    } else if (arg == "zero_cov" || arg == "zero-cov") {
      zero_cov = true;
    } else {
      std::cout << "Usage: " << argv[0]
                << " [manifold|tangent] [both|prop_by_preint|preint_by_prop] [zero_cov]\n";
      return 1;
    }
  }

  if (run_prop_by_preint) {
    PropagationByPreintegration(use_manifold, zero_cov);
  }
  if (run_preint_by_prop) {
    PreintegrationByPropagation(use_manifold);
  }
  return 0;
}
