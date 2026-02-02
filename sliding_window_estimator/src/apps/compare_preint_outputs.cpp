#include <Eigen/Dense>

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

namespace {

static inline Eigen::Matrix3d crossMx(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

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
    throw std::runtime_error("unable to open file: " + path);
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

static bool header_matches(const std::string &line, const std::string &name, int rows, int cols) {
  const std::string t = trim(line);
  if (t.rfind(name, 0) != 0) {
    return false;
  }
  const size_t lp = t.find('(');
  const size_t xp = t.find('x', lp == std::string::npos ? 0 : lp + 1);
  const size_t rp = t.find(')', xp == std::string::npos ? 0 : xp + 1);
  if (lp == std::string::npos || xp == std::string::npos || rp == std::string::npos || !(lp < xp && xp < rp)) {
    return false;
  }
  const std::string rs = trim(t.substr(lp + 1, xp - (lp + 1)));
  const std::string cs = trim(t.substr(xp + 1, rp - (xp + 1)));
  if (rs.empty() || cs.empty())
    return false;
  int r = 0, c = 0;
  try {
    r = std::stoi(rs);
    c = std::stoi(cs);
  } catch (...) {
    return false;
  }
  return (r == rows) && (c == cols);
}

template <int R, int C>
static Eigen::Matrix<double, R, C> parse_block_matrix(const std::string &path, const std::string &name) {
  const auto lines = read_lines(path);
  size_t start = std::string::npos;
  for (size_t i = 0; i < lines.size(); i++) {
    if (header_matches(lines[i], name, R, C)) {
      start = i + 1;
      break;
    }
  }
  if (start == std::string::npos) {
    throw std::runtime_error("missing block '" + name + " (" + std::to_string(R) + "x" + std::to_string(C) + ")' in " + path);
  }

  Eigen::Matrix<double, R, C> M;
  M.setZero();
  size_t i = start;
  for (int r = 0; r < R; r++) {
    while (i < lines.size() && trim(lines[i]).empty())
      i++;
    if (i >= lines.size()) {
      throw std::runtime_error("unexpected EOF while reading '" + name + "' in " + path);
    }
    std::istringstream iss(lines[i]);
    for (int c = 0; c < C; c++) {
      double v = 0.0;
      if (!(iss >> v)) {
        throw std::runtime_error("failed to parse number at " + name + "(" + std::to_string(r) + "," + std::to_string(c) + ") in " +
                                 path);
      }
      M(r, c) = v;
    }
    i++;
  }
  return M;
}

// -------- OpenVINS pack yaml parsing (copied from preint_from_openvins_pack.cpp) --------

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
};

static bool parse_scalar_double_top(const std::vector<std::string> &lines, const std::string &key, double &out) {
  const std::string needle = key + ":";
  for (const auto &line : lines) {
    const std::string t = trim(line);
    if (t.rfind(needle, 0) != 0)
      continue;
    const std::string rest = trim(t.substr(needle.size()));
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

static Eigen::Vector3d to_vec3(const std::vector<double> &v, const std::string &what) {
  if (v.size() != 3) {
    throw std::runtime_error("expected 3 elements for " + what);
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

static Eigen::Matrix3d quat_xyzw_to_R_GtoI_JPL(const std::vector<double> &q, const std::string &what) {
  if (q.size() != 4) {
    throw std::runtime_error("expected 4 elements for " + what);
  }
  // OpenVINS stores JPL quaternion (x,y,z,w). Its corresponding rotation matrix is the transpose
  // of the Hamilton convention used by Eigen::Quaterniond for the same coefficients.
  Eigen::Vector4d qjpl(q[0], q[1], q[2], q[3]);
  qjpl.normalize();
  const Eigen::Vector3d qv = qjpl.head<3>();
  const double qw = qjpl(3);
  const Eigen::Matrix3d qx = crossMx(qv);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  return (2.0 * qw * qw - 1.0) * I - 2.0 * qw * qx + 2.0 * qv * qv.transpose();
}

static OvPack load_ov_pack_yaml(const std::string &path) {
  const auto lines = read_lines(path);
  OvPack pack;

  if (!parse_scalar_double_top(lines, "dt", pack.dt) || !(pack.dt > 0.0)) {
    throw std::runtime_error("missing/invalid dt in ov_pack_yaml");
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
    pack.xs.R_GtoI = quat_xyzw_to_R_GtoI_JPL(q, "xs_nominal.q_GtoI_xyzw");
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
    pack.xe.R_GtoI = quat_xyzw_to_R_GtoI_JPL(q, "xe_nominal.q_GtoI_xyzw");
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

  return pack;
}

static double rot_angle_rad(const Eigen::Matrix3d &R) {
  const double c = std::max(-1.0, std::min(1.0, 0.5 * (R.trace() - 1.0)));
  return std::acos(c);
}

template <typename Derived>
static double maxAbs(const Eigen::MatrixBase<Derived> &M) {
  return M.cwiseAbs().maxCoeff();
}

static double minEigenSymmetric(const Eigen::MatrixXd &M) {
  const Eigen::MatrixXd S = 0.5 * (M + M.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S);
  if (es.info() != Eigen::Success) {
    throw std::runtime_error("eigen decomposition failed");
  }
  return es.eigenvalues().minCoeff();
}

static Eigen::Matrix<double, 9, 6> swap_bg_ba_to_ba_bg(const Eigen::Matrix<double, 9, 6> &J_bg_ba) {
  Eigen::Matrix<double, 9, 6> J_ba_bg = Eigen::Matrix<double, 9, 6>::Zero();
  J_ba_bg.block<9, 3>(0, 0) = J_bg_ba.block<9, 3>(0, 3);
  J_ba_bg.block<9, 3>(0, 3) = J_bg_ba.block<9, 3>(0, 0);
  return J_ba_bg;
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::string ov_pack_yaml;
    std::string ov_all;
    std::string gtsam_all;

    for (int i = 1; i < argc; i++) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc)
          throw std::runtime_error("missing value after " + a);
        return std::string(argv[++i]);
      };
      if (a == "--ov_pack_yaml") {
        ov_pack_yaml = next();
      } else if (a == "--ov_all") {
        ov_all = next();
      } else if (a == "--gtsam_all") {
        gtsam_all = next();
      } else if (a == "--help" || a == "-h") {
        std::cout << "usage: " << argv[0] << " --ov_pack_yaml <imu_prop_pack.yaml> --ov_all <preint_from_openvins_pack_all.txt>"
                  << " --gtsam_all <gtsam_ref_preint_all.txt>\n";
        return EXIT_SUCCESS;
      } else {
        throw std::runtime_error("unknown argument: " + a);
      }
    }

    if (ov_pack_yaml.empty() || ov_all.empty() || gtsam_all.empty()) {
      std::cerr << "usage: " << argv[0] << " --ov_pack_yaml <imu_prop_pack.yaml> --ov_all <preint_from_openvins_pack_all.txt>"
                << " --gtsam_all <gtsam_ref_preint_all.txt>\n";
      return EXIT_FAILURE;
    }

    // -------- Parse inputs --------

    const OvPack pack = load_ov_pack_yaml(ov_pack_yaml);

    Eigen::Matrix<double, 15, 15> Sigma_z_ov = parse_block_matrix<15, 15>(ov_all, "Sigma_z_from_openvins_pack");

    Eigen::Matrix<double, 9, 6> JincBias_ba_bg_ov;
    bool has_ba_bg = true;
    try {
      JincBias_ba_bg_ov = parse_block_matrix<9, 6>(ov_all, "JincBias_ba_bg_rk4");
    } catch (...) {
      has_ba_bg = false;
    }
    if (!has_ba_bg) {
      const Eigen::Matrix<double, 9, 6> J_bg_ba = parse_block_matrix<9, 6>(ov_all, "JincBias_bg_ba_rk4");
      JincBias_ba_bg_ov = swap_bg_ba_to_ba_bg(J_bg_ba);
    }

    const Eigen::Matrix3d dR_gtsam = parse_block_matrix<3, 3>(gtsam_all, "dR_gtsam");
    const Eigen::Matrix<double, 3, 1> dP_gtsam = parse_block_matrix<3, 1>(gtsam_all, "dP_gtsam");
    const Eigen::Matrix<double, 3, 1> dV_gtsam = parse_block_matrix<3, 1>(gtsam_all, "dV_gtsam");
    const Eigen::Matrix<double, 1, 1> DT_gtsam_mat = parse_block_matrix<1, 1>(gtsam_all, "DT_gtsam");
    const double dt_gtsam = DT_gtsam_mat(0, 0);
    const Eigen::Matrix<double, 15, 15> Sigma_z_gtsam = parse_block_matrix<15, 15>(gtsam_all, "Sigma_z_gtsam");
    const Eigen::Matrix<double, 9, 6> JincBias_ba_bg_gtsam = parse_block_matrix<9, 6>(gtsam_all, "JincBias_ba_bg_gtsam");

    // -------- Compute dR/dP/dV from ov_pack_yaml (must match preint_from_openvins_pack.cpp) --------

    const Eigen::Matrix3d Rws_s = pack.xs.R_GtoI.transpose();
    const Eigen::Matrix3d Rws_e = pack.xe.R_GtoI.transpose();
    const Eigen::Vector3d pws_s = pack.xs.p_IinG;
    const Eigen::Vector3d vws_s = pack.xs.v_IinG;
    const Eigen::Vector3d pws_e = pack.xe.p_IinG;
    const Eigen::Vector3d vws_e = pack.xe.v_IinG;

    const Eigen::Matrix3d dR_ov = Rws_s.transpose() * Rws_e;
    const Eigen::Vector3d dP_ov = Rws_s.transpose() * (pws_e - pws_s - vws_s * pack.dt - 0.5 * pack.gravity_G * pack.dt * pack.dt);
    const Eigen::Vector3d dV_ov = Rws_s.transpose() * (vws_e - vws_s - pack.gravity_G * pack.dt);

    // -------- Metrics --------

    const double ang_dR = rot_angle_rad(dR_ov.transpose() * dR_gtsam);
    const double err_dP = (dP_ov - dP_gtsam).norm();
    const double err_dV = (dV_ov - dV_gtsam).norm();
    const double err_dt = std::abs(pack.dt - dt_gtsam);
    const double rel_dP = err_dP / std::max(1.0, dP_gtsam.norm());
    const double rel_dV = err_dV / std::max(1.0, dV_gtsam.norm());

    const double Sigma_diff_max = maxAbs(Sigma_z_ov - Sigma_z_gtsam);
    const double Sigma_ref_max = maxAbs(Sigma_z_gtsam);
    const double Sigma_rel = Sigma_diff_max / std::max(1.0, Sigma_ref_max);

    const double J_diff_max = maxAbs(JincBias_ba_bg_ov - JincBias_ba_bg_gtsam);
    const double J_ref_max = maxAbs(JincBias_ba_bg_gtsam);
    const double J_rel = J_diff_max / std::max(1.0, J_ref_max);

    const double sym_ov = maxAbs(Sigma_z_ov - Sigma_z_ov.transpose());
    const double sym_gs = maxAbs(Sigma_z_gtsam - Sigma_z_gtsam.transpose());
    const double minEig_ov = minEigenSymmetric(Sigma_z_ov);
    const double minEig_gs = minEigenSymmetric(Sigma_z_gtsam);

    // -------- PASS/FAIL --------

    // Mean increments can differ slightly because OpenVINS nominal uses RK4, while GTSAM preintegration
    // uses its own discrete integration model. Use a relative threshold similar in spirit to expectNearAbsRel().
    constexpr double kThAngle = 5e-3; // rad
    constexpr double kThVecAbs = 1e-6;
    constexpr double kThVecRel = 1e-3;
    constexpr double kThDt = 1e-12;
    constexpr double kThRel = 1e-3;
    constexpr double kThSym = 1e-8;
    constexpr double kThMinEig = -1e-8;

    bool pass = true;
    auto check = [&](const std::string &label, bool ok) {
      std::cout << label << ": " << (ok ? "PASS" : "FAIL") << "\n";
      pass = pass && ok;
    };

    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(12);
    std::cout << "Delta checks:\n";
    std::cout << "  angle(dR_ov^T*dR_gtsam) = " << ang_dR << " rad\n";
    std::cout << "  ||dP_ov-dP_gtsam||      = " << err_dP << "\n";
    std::cout << "  ||dV_ov-dV_gtsam||      = " << err_dV << "\n";
    std::cout << "  |dt_ov-dt_gtsam|        = " << err_dt << "\n";
    std::cout << "  rel_dP                  = " << rel_dP << "\n";
    std::cout << "  rel_dV                  = " << rel_dV << "\n";
    check("  angle(dR) < 5e-3 rad", ang_dR < kThAngle);
    check("  dP abs/rel", err_dP <= kThVecAbs + kThVecRel * std::max(1.0, dP_gtsam.norm()));
    check("  dV abs/rel", err_dV <= kThVecAbs + kThVecRel * std::max(1.0, dV_gtsam.norm()));
    check("  dt < 1e-12", err_dt < kThDt);

    std::cout << "\nSigma_z checks:\n";
    std::cout << "  maxAbs(diff)            = " << Sigma_diff_max << "\n";
    std::cout << "  rel                     = " << Sigma_rel << "   (den=max(1,maxAbs(ref)))\n";
    check("  Sigma_z rel < 1e-3", Sigma_rel < kThRel);

    std::cout << "\nJincBias checks:\n";
    std::cout << "  maxAbs(diff)            = " << J_diff_max << "\n";
    std::cout << "  rel                     = " << J_rel << "   (den=max(1,maxAbs(ref)))\n";
    check("  JincBias rel < 1e-3", J_rel < kThRel);

    std::cout << "\nSanity checks:\n";
    std::cout << "  Sigma_z_ov symmetry maxAbs(S-S^T)   = " << sym_ov << "\n";
    std::cout << "  Sigma_z_gtsam symmetry maxAbs(S-S^T)= " << sym_gs << "\n";
    std::cout << "  Sigma_z_ov min eigen (sym)          = " << minEig_ov << "\n";
    std::cout << "  Sigma_z_gtsam min eigen (sym)       = " << minEig_gs << "\n";
    check("  Sigma_z_ov symmetry < 1e-8", sym_ov < kThSym);
    check("  Sigma_z_gtsam symmetry < 1e-8", sym_gs < kThSym);
    check("  Sigma_z_ov minEig >= -1e-8", minEig_ov >= kThMinEig);
    check("  Sigma_z_gtsam minEig >= -1e-8", minEig_gs >= kThMinEig);

    std::cout << "\nOverall: " << (pass ? "PASS" : "FAIL") << "\n";
    return pass ? EXIT_SUCCESS : EXIT_FAILURE;

  } catch (const std::exception &e) {
    std::cerr << "compare_preint_outputs failed: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
