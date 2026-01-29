#include <Eigen/Dense>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

static inline Eigen::Matrix3d crossMx(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

// Copied (minimally) from src/apps/propag_by_preint.cpp to keep the mapping identical.
struct Maps15 {
  Eigen::Matrix<double, 15, 15> F;    // df_e(x_s, Dx)/dx_s (OKVIS error state, x=[p,q,v,bg,ba])
  Eigen::Matrix<double, 15, 15> G;    // dx_e/dz for transition matrix (z=[dtheta,dp,dv,dba,dbg] of gtsam)
  Eigen::Matrix<double, 15, 15> covG; // dx_e/dz for covariance (z=[dtheta,dp,dv,dba,dbg] of gtsam)
  Eigen::Matrix<double, 15, 15> G_inv;
  Eigen::Matrix<double, 15, 15> covG_inv;
  Eigen::Matrix<double, 15, 15> Phi; // dx_e/dx_s (OKVIS)
};

static inline Eigen::Matrix<double, 9, 6> swapBias(const Eigen::Matrix<double, 9, 6> &JincBias_ba_bg) {
  Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
  // [bg,ba] -> [ba,bg]
  P.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); // ba <- ba part of okvis vector (which is at +3)
  P.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity(); // bg <- bg part of okvis vector (which is at +0)
  // Now columns become [bg,ba] as needed by OKVIS state layout
  Eigen::Matrix<double, 9, 6> JincBias_bg_ba = JincBias_ba_bg * P;
  return JincBias_bg_ba;
}

static inline Maps15 BuildMaps15_Tangent(const Eigen::Matrix3d &Rws, const Eigen::Matrix3d &dR, const Eigen::Vector3d &dP,
                                         const Eigen::Vector3d &dV, double dt) {
  Maps15 m;
  m.F.setZero();
  m.G.setZero();
  m.covG.setZero();

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  m.F.block<3, 3>(0, 0) = I;     // dp_e / dp_s
  m.F.block<3, 3>(0, 6) = dt * I; // dp_e / dv_s
  m.F.block<3, 3>(0, 3) = -crossMx(Rws * dP); // dp_e / dtheta_s

  m.F.block<3, 3>(3, 3) = I;
  m.F.block<3, 3>(6, 6) = I;
  m.F.block<3, 3>(6, 3) = -crossMx(Rws * dV);

  m.F.block<3, 3>(9, 9) = I;
  m.F.block<3, 3>(12, 12) = I;

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
  m.G.block<3, 3>(0, 3) = Rws;
  // dtheta_e <- dtheta
  m.G.block<3, 3>(3, 0) = Rwe_from_s * Jr;
  // dv_e <- Rs * dv
  m.G.block<3, 3>(6, 6) = Rws;
  // dbg_e <- dbg
  m.G.block<3, 3>(9, 12) = -I;
  // dba_e <- dba
  m.G.block<3, 3>(12, 9) = -I;
  m.G_inv = m.G.transpose();
  m.G_inv.block<3, 3>(0, 3) = Jr_inv * Rwe_from_s.transpose();

  m.covG = m.G;
  m.covG_inv = m.G_inv;

  return m;
}

namespace {

struct OvNominal {
  Eigen::Matrix3d R_GtoI = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
};

struct OvPack {
  OvNominal xs;
  OvNominal xe;
  double dt = 0.0;
  Eigen::Vector3d gravity_G = Eigen::Vector3d(0, 0, -9.81);
  Eigen::Matrix<double, 15, 15> Phi = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> Sigma = Eigen::Matrix<double, 15, 15>::Zero();
};

static inline std::string trim(const std::string &s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])))
    b++;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
    e--;
  return s.substr(b, e - b);
}

static std::vector<std::string> read_lines(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("unable to open pack yaml: " + path);
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    lines.push_back(line);
  }
  return lines;
}

static bool parse_scalar_double_top(const std::vector<std::string> &lines, const std::string &key, double &out) {
  const std::string needle = key + ":";
  for (const auto &line : lines) {
    const std::string t = trim(line);
    if (t.rfind(needle, 0) != 0)
      continue;
    std::string rest = trim(t.substr(needle.size()));
    if (rest.empty())
      return false;
    out = std::stod(rest);
    return true;
  }
  return false;
}

static bool parse_inline_list_in_section(const std::vector<std::string> &lines, const std::string &section, const std::string &key,
                                         std::vector<double> &out) {
  const std::string section_hdr = section + ":";
  bool in_section = false;
  for (size_t i = 0; i < lines.size(); i++) {
    const std::string raw = lines[i];
    const std::string t = trim(raw);
    if (!in_section) {
      if (t == section_hdr && (raw.empty() || raw[0] != ' ')) {
        in_section = true;
      }
      continue;
    }

    // stop at next top-level key
    if (!raw.empty() && raw[0] != ' ')
      break;

    const std::string key_hdr = key + ":";
    if (t.rfind(key_hdr, 0) != 0)
      continue;
    const size_t lb = raw.find('[');
    const size_t rb = raw.find(']', lb == std::string::npos ? 0 : lb + 1);
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb)
      return false;
    const std::string inside = raw.substr(lb + 1, rb - (lb + 1));
    std::stringstream ss(inside);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
      const std::string tt = trim(tok);
      if (tt.empty())
        continue;
      vals.push_back(std::stod(tt));
    }
    out = std::move(vals);
    return !out.empty();
  }
  return false;
}

static Eigen::Matrix<double, 15, 15> parse_matrix15_top(const std::vector<std::string> &lines, const std::string &key) {
  const std::string hdr = key + ":";
  size_t start = std::string::npos;
  for (size_t i = 0; i < lines.size(); i++) {
    const std::string raw = lines[i];
    if (!raw.empty() && raw[0] == ' ')
      continue;
    if (trim(raw) == hdr) {
      start = i + 1;
      break;
    }
  }
  if (start == std::string::npos) {
    throw std::runtime_error("missing matrix key: " + key);
  }

  Eigen::Matrix<double, 15, 15> M;
  M.setZero();
  size_t r = 0;
  for (size_t i = start; i < lines.size() && r < 15; i++) {
    const std::string raw = trim(lines[i]);
    if (raw.empty())
      continue;
    if (raw.rfind("-", 0) != 0)
      continue;
    const size_t lb = raw.find('[');
    const size_t rb = raw.find(']', lb == std::string::npos ? 0 : lb + 1);
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
      throw std::runtime_error("invalid matrix row under key: " + key);
    }
    const std::string inside = raw.substr(lb + 1, rb - (lb + 1));
    std::stringstream ss(inside);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
      const std::string tt = trim(tok);
      if (tt.empty())
        continue;
      vals.push_back(std::stod(tt));
    }
    if (vals.size() != 15) {
      throw std::runtime_error("expected 15 columns under key: " + key);
    }
    for (size_t c = 0; c < 15; c++) {
      M(static_cast<int>(r), static_cast<int>(c)) = vals[c];
    }
    r++;
  }
  if (r != 15) {
    throw std::runtime_error("expected 15 rows under key: " + key);
  }
  return M;
}

static Eigen::Vector3d to_vec3(const std::vector<double> &v, const std::string &what) {
  if (v.size() != 3) {
    throw std::runtime_error("expected 3 elements for " + what);
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

static Eigen::Matrix3d quat_xyzw_to_R(const std::vector<double> &q, const std::string &what) {
  if (q.size() != 4) {
    throw std::runtime_error("expected 4 elements for " + what);
  }
  Eigen::Quaterniond qq(q[3], q[0], q[1], q[2]); // Eigen is w,x,y,z
  qq.normalize();
  return qq.toRotationMatrix();
}

static OvPack load_pack(const std::string &path) {
  const auto lines = read_lines(path);
  OvPack pack;

  if (!parse_scalar_double_top(lines, "dt", pack.dt) || !(pack.dt > 0.0)) {
    throw std::runtime_error("missing/invalid dt");
  }

  // xs
  {
    std::vector<double> q, p, v;
    if (!parse_inline_list_in_section(lines, "xs_nominal", "q_GtoI_xyzw", q))
      throw std::runtime_error("missing xs_nominal.q_GtoI_xyzw");
    if (!parse_inline_list_in_section(lines, "xs_nominal", "p_IinG", p))
      throw std::runtime_error("missing xs_nominal.p_IinG");
    if (!parse_inline_list_in_section(lines, "xs_nominal", "v_IinG", v))
      throw std::runtime_error("missing xs_nominal.v_IinG");
    pack.xs.R_GtoI = quat_xyzw_to_R(q, "xs_nominal.q_GtoI_xyzw");
    pack.xs.p_IinG = to_vec3(p, "xs_nominal.p_IinG");
    pack.xs.v_IinG = to_vec3(v, "xs_nominal.v_IinG");
  }
  // xe
  {
    std::vector<double> q, p, v;
    if (!parse_inline_list_in_section(lines, "xe_nominal", "q_GtoI_xyzw", q))
      throw std::runtime_error("missing xe_nominal.q_GtoI_xyzw");
    if (!parse_inline_list_in_section(lines, "xe_nominal", "p_IinG", p))
      throw std::runtime_error("missing xe_nominal.p_IinG");
    if (!parse_inline_list_in_section(lines, "xe_nominal", "v_IinG", v))
      throw std::runtime_error("missing xe_nominal.v_IinG");
    pack.xe.R_GtoI = quat_xyzw_to_R(q, "xe_nominal.q_GtoI_xyzw");
    pack.xe.p_IinG = to_vec3(p, "xe_nominal.p_IinG");
    pack.xe.v_IinG = to_vec3(v, "xe_nominal.v_IinG");
  }

  // gravity vector in global/world frame
  {
    std::vector<double> g;
    if (!parse_inline_list_in_section(lines, "gravity_g", "config_gravity_input", g)) {
      throw std::runtime_error("missing gravity_g.config_gravity_input");
    }
    pack.gravity_G = to_vec3(g, "gravity_g.config_gravity_input");
  }

  pack.Phi = parse_matrix15_top(lines, "Phi_15x15");
  pack.Sigma = parse_matrix15_top(lines, "Sigma_15x15_from_zero");

  return pack;
}

template <typename Derived>
static void saveMatrixTxt(const std::string &path, const Eigen::MatrixBase<Derived> &mat) {
  std::ofstream ofs(path, std::ios::trunc);
  if (!ofs.is_open()) {
    throw std::runtime_error("failed to open for writing: " + path);
  }
  ofs.setf(std::ios::fixed);
  ofs << std::setprecision(18);
  for (int r = 0; r < mat.rows(); ++r) {
    for (int c = 0; c < mat.cols(); ++c) {
      ofs << mat(r, c);
      if (c < mat.cols() - 1) {
        ofs << ' ';
      }
    }
    ofs << '\n';
  }
}

static Eigen::Matrix<double, 15, 15> build_perm_OV_to_OKVIS() {
  // OV:    [dtheta, dp, dv, dbg, dba]
  // OKVIS: [dp, dtheta, dv, dbg, dba]
  Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Zero();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  P.block<3, 3>(0, 3) = I;   // dp <- dp
  P.block<3, 3>(3, 0) = I;   // dtheta <- dtheta
  P.block<3, 3>(6, 6) = I;   // dv <- dv
  P.block<3, 3>(9, 9) = I;   // dbg <- dbg
  P.block<3, 3>(12, 12) = I; // dba <- dba
  return P;
}

static double rot_angle_rad(const Eigen::Matrix3d &R) {
  // robust enough for small angles
  const double tr = std::max(-1.0, std::min(3.0, R.trace()));
  const double c = std::max(-1.0, std::min(1.0, 0.5 * (tr - 1.0)));
  return std::acos(c);
}

} // namespace

int main(int argc, char **argv) {
  try {
    if (argc < 2) {
      std::cerr << "usage: " << argv[0] << " <imu_prop_pack.yaml> [out_dir]\n";
      return EXIT_FAILURE;
    }
    const std::string pack_path = argv[1];
    const std::string out_dir = (argc >= 3) ? std::string(argv[2]) : std::string(".");

    const OvPack pack = load_pack(pack_path);

    // 1) Convert OV nominal R_GtoI -> Rws = R_GtoI^T (body->world)
    const Eigen::Matrix3d Rws_s = pack.xs.R_GtoI.transpose();
    const Eigen::Matrix3d Rws_e = pack.xe.R_GtoI.transpose();
    const Eigen::Vector3d pws_s = pack.xs.p_IinG;
    const Eigen::Vector3d vws_s = pack.xs.v_IinG;
    const Eigen::Vector3d pws_e = pack.xe.p_IinG;
    const Eigen::Vector3d vws_e = pack.xe.v_IinG;

    // 2) Solve for dR/dP/dV in start body (S) frame.
    const Eigen::Matrix3d dR = Rws_s.transpose() * Rws_e;
    const Eigen::Vector3d dP = Rws_s.transpose() * (pws_e - pws_s - vws_s * pack.dt - 0.5 * pack.gravity_G * pack.dt * pack.dt);
    const Eigen::Vector3d dV = Rws_s.transpose() * (vws_e - vws_s - pack.gravity_G * pack.dt);

    // Recon check.
    const Eigen::Vector3d p_recon = pws_s + vws_s * pack.dt + 0.5 * pack.gravity_G * pack.dt * pack.dt + Rws_s * dP;
    const Eigen::Vector3d v_recon = vws_s + pack.gravity_G * pack.dt + Rws_s * dV;
    const Eigen::Matrix3d R_recon = Rws_s * dR;
    const double p_err = (p_recon - pws_e).norm();
    const double v_err = (v_recon - vws_e).norm();
    const double R_err = rot_angle_rad(R_recon.transpose() * Rws_e);
    std::cout << std::setprecision(6) << "recon errors: |p|=" << p_err << " |v|=" << v_err << " angle=" << R_err << " rad\n";

    // 3) Permute OV -> OKVIS 15D ordering.
    const Eigen::Matrix<double, 15, 15> P = build_perm_OV_to_OKVIS();
    const Eigen::Matrix<double, 15, 15> covRK4 = P * pack.Sigma * P.transpose();
    const Eigen::Matrix<double, 15, 15> jacRK4 = P * pack.Phi * P.transpose();

    // 4) Build maps (Tangent).
    const Maps15 maps = BuildMaps15_Tangent(Rws_s, dR, dP, dV, pack.dt);

    // 5) Compute Sigma_z and JincBias from RK4-equivalent cov/jac.
    const Eigen::Matrix<double, 15, 15> Sigma_z = maps.covG_inv * covRK4 * maps.covG_inv.transpose();
    const Eigen::Matrix<double, 9, 6> JincBias_bg_ba_rk4 =
        maps.G_inv.topLeftCorner<9, 9>() * jacRK4.topRightCorner<9, 6>();

    // Also output swapped [ba,bg] column order.
    // swapBias maps [ba,bg] -> [bg,ba], so applying it twice returns to original.
    const Eigen::Matrix<double, 9, 6> JincBias_ba_bg_rk4 = swapBias(JincBias_bg_ba_rk4);

    // 6) Save matrices to txt files (space-separated, same as saveMatrix in propag_by_preint.cpp).
    saveMatrixTxt(out_dir + "/covRK4_from_openvins_pack.txt", covRK4);
    saveMatrixTxt(out_dir + "/jacRK4_from_openvins_pack.txt", jacRK4);
    saveMatrixTxt(out_dir + "/Sigma_z_from_openvins_pack.txt", Sigma_z);
    saveMatrixTxt(out_dir + "/JincBias_bg_ba_rk4.txt", JincBias_bg_ba_rk4);
    saveMatrixTxt(out_dir + "/JincBias_ba_bg_rk4.txt", JincBias_ba_bg_rk4);

    std::cout << "wrote:\n"
              << "  " << out_dir << "/covRK4_from_openvins_pack.txt\n"
              << "  " << out_dir << "/jacRK4_from_openvins_pack.txt\n"
              << "  " << out_dir << "/Sigma_z_from_openvins_pack.txt\n"
              << "  " << out_dir << "/JincBias_bg_ba_rk4.txt\n"
              << "  " << out_dir << "/JincBias_ba_bg_rk4.txt\n";
    return EXIT_SUCCESS;

  } catch (const std::exception &e) {
    std::cerr << "preint_from_openvins_pack failed: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
