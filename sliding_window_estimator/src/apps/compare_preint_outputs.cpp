#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
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

// swift_vio_ws/devel/include/swift_vio/imu/CovPropConfig.hpp line 54
// Same criterion as swift_vio/imu/CovPropConfig.hpp::expectNearAbsRel:
// tol = absTol + relTol * max(|ref|, |est|), checked entry-wise.
struct AbsRelFailEntry {
  int r = -1;
  int c = -1;
  double ref = 0.0;
  double est = 0.0;
  double diff = 0.0;
  double tol = 0.0;
  double violation = 0.0; // diff - tol
};

struct AbsRelCheckResult {
  bool ok = true;
  int fail_count = 0;
  std::vector<AbsRelFailEntry> fails;
  int worst_r = -1;
  int worst_c = -1;
  double worst_ref = 0.0;
  double worst_est = 0.0;
  double worst_diff = 0.0;
  double worst_tol = 0.0;
  double worst_violation = -std::numeric_limits<double>::infinity(); // diff - tol
};

static AbsRelCheckResult absRelCheck(const Eigen::MatrixXd &ref, const Eigen::MatrixXd &est, double absTol, double relTol) {
  if (ref.rows() != est.rows() || ref.cols() != est.cols()) {
    throw std::runtime_error("absRelCheck: dimension mismatch");
  }

  AbsRelCheckResult out;
  for (int r = 0; r < ref.rows(); ++r) {
    for (int c = 0; c < ref.cols(); ++c) {
      const double a = ref(r, c);
      const double b = est(r, c);
      const double diff = std::abs(a - b);
      const double scale = std::max(std::abs(a), std::abs(b));

      // abs+rel 混合容差:
      // - 元素很小（scale≈0）：tol≈absTol，退化成“绝对误差比较”，避免除零/夸大。
      // - 元素很大：tol≈relTol*scale，退化成“相对误差比较”，允许与量级成比例的误差。
      const double tol = absTol + relTol * scale;
      const double violation = diff - tol;

      if (violation > out.worst_violation) {
        out.worst_violation = violation;
        out.worst_r = r;
        out.worst_c = c;
        out.worst_ref = a;
        out.worst_est = b;
        out.worst_diff = diff;
        out.worst_tol = tol;
      }

      if (diff > tol) {
        out.ok = false;
        out.fail_count++;
        out.fails.push_back(AbsRelFailEntry{r, c, a, b, diff, tol, violation});
      }
    }
  }
  return out;
}

static const char *axis_name(int idx) {
  switch (idx) {
  case 0:
    return "x";
  case 1:
    return "y";
  default:
    return "z";
  }
}

static std::string describe_jincbias_entry(int r, int c) {
  // Rows: [dphi(0-2), dp(3-5), dv(6-8)] ; Cols: [dba(0-2), dbg(3-5)]
  const int row_group = r / 3;
  const int row_axis = r % 3;
  const int col_group = c / 3;
  const int col_axis = c % 3;

  const char *row_name = (row_group == 0) ? "dphi" : (row_group == 1) ? "dp" : "dv";
  const char *col_name = (col_group == 0) ? "dba" : "dbg";

  std::ostringstream oss;
  oss << row_name << "_" << axis_name(row_axis) << " / " << col_name << "_" << axis_name(col_axis);
  return oss.str();
}

static std::string describe_jac15_entry(int r, int c) {
  auto z_label = [](int idx) -> std::string {
    const int g = idx / 3;
    const int a = idx % 3;
    const char *name = (g == 0) ? "dphi" : (g == 1) ? "dp" : (g == 2) ? "dv" : (g == 3) ? "dba" : "dbg";
    std::ostringstream oss;
    oss << name << "_" << axis_name(a);
    return oss.str();
  };
  auto x_label = [](int idx) -> std::string {
    const int g = idx / 3;
    const int a = idx % 3;
    const char *name = (g == 0) ? "dp" : (g == 1) ? "dtheta" : (g == 2) ? "dv" : (g == 3) ? "dba" : "dbg";
    std::ostringstream oss;
    oss << name << "_" << axis_name(a);
    return oss.str();
  };
  std::ostringstream oss;
  oss << z_label(r) << " / " << x_label(c);
  return oss.str();
}

static std::string describe_sigmaz_entry(int r, int c) {
  // z_order in YAML: [dphi, dp, dv, dba, dbg] (each 3D)
  auto label = [](int idx) -> std::string {
    const int g = idx / 3;
    const int a = idx % 3;
    const char *name = (g == 0) ? "dphi" : (g == 1) ? "dp" : (g == 2) ? "dv" : (g == 3) ? "dba" : "dbg";
    std::ostringstream oss;
    oss << name << "_" << axis_name(a);
    return oss.str();
  };
  std::ostringstream oss;
  oss << label(r) << " / " << label(c);
  return oss.str();
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

struct PreintFactorJacobians {
  Eigen::Matrix<double, 15, 15> J_s = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 15> J_e = Eigen::Matrix<double, 15, 15>::Zero();
};

static inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

static Eigen::Vector3d so3_log(const Eigen::Matrix3d &R) {
  const double cos_theta = clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  Eigen::Vector3d vee;
  vee << (R(2, 1) - R(1, 2)), (R(0, 2) - R(2, 0)), (R(1, 0) - R(0, 1));
  if (theta < 1e-9) {
    return 0.5 * vee;
  }
  const double sin_theta = std::sin(theta);
  if (std::abs(sin_theta) < 1e-12) {
    return 0.5 * vee;
  }
  return (theta / (2.0 * sin_theta)) * vee;
}

static Eigen::Matrix3d so3_right_jacobian(const Eigen::Vector3d &phi) {
  const double theta = phi.norm();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d Phi = crossMx(phi);
  const Eigen::Matrix3d Phi2 = Phi * Phi;

  if (theta < 1e-8) {
    return I - 0.5 * Phi + (1.0 / 12.0) * Phi2;
  }

  const double theta2 = theta * theta;
  const double theta3 = theta2 * theta;
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  const double c1 = (1.0 - c) / theta2;
  const double c2 = (theta - s) / theta3;
  return I - c1 * Phi + c2 * Phi2;
}

static Eigen::Matrix3d so3_right_jacobian_inverse(const Eigen::Vector3d &phi) {
  const double theta = phi.norm();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d Phi = crossMx(phi);
  const Eigen::Matrix3d Phi2 = Phi * Phi;

  if (theta < 1e-8) {
    return I + 0.5 * Phi + (1.0 / 12.0) * Phi2;
  }

  const double theta2 = theta * theta;
  const double half_theta = 0.5 * theta;
  const double cot_half_theta = std::cos(half_theta) / std::sin(half_theta);
  const double a = (1.0 / theta2) - (0.5 / theta) * cot_half_theta;
  return I + 0.5 * Phi + a * Phi2;
}

static PreintFactorJacobians build_preint_factor_jacobians_local(const Eigen::Matrix3d &dR, const Eigen::Vector3d &dP,
                                                                  const Eigen::Vector3d &dV, const double dt,
                                                                  const Eigen::Matrix<double, 9, 6> &JincBias_ba_bg) {
  const Eigen::Vector3d phi = so3_log(dR);
  const Eigen::Matrix3d Jr = so3_right_jacobian(phi);
  const Eigen::Matrix3d Jr_inv = so3_right_jacobian_inverse(phi);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
  F.block<3, 3>(0, 0) = I;
  F.block<3, 3>(0, 3) = -crossMx(dP);
  F.block<3, 3>(0, 6) = dt * I;
  F.block<3, 3>(3, 3) = I;
  F.block<3, 3>(6, 3) = -crossMx(dV);
  F.block<3, 3>(6, 6) = I;
  F.block<3, 3>(9, 9) = I;
  F.block<3, 3>(12, 12) = I;

  Eigen::Matrix<double, 15, 15> G = Eigen::Matrix<double, 15, 15>::Zero();
  G.block<3, 3>(0, 3) = I;
  G.block<3, 3>(3, 0) = Jr;
  G.block<3, 3>(6, 6) = I;
  G.block<3, 3>(9, 9) = -I;
  G.block<3, 3>(12, 12) = -I;

  Eigen::Matrix<double, 15, 15> G_inv = G.transpose();
  G_inv.block<3, 3>(0, 3) = Jr_inv;

  Eigen::Matrix<double, 15, 15> J = F;
  J.topRightCorner<9, 6>() += G.topLeftCorner<9, 9>() * JincBias_ba_bg;

  PreintFactorJacobians out;
  out.J_e = G_inv;
  out.J_s = -G_inv * J;
  return out;
}

static bool parse_bracket_list(const std::string &raw, std::vector<double> &out) {
  const size_t lb = raw.find('[');
  if (lb == std::string::npos)
    return false;
  const size_t rb = raw.find(']', lb + 1);
  if (rb == std::string::npos || rb <= lb)
    return false;
  const std::string inside = raw.substr(lb + 1, rb - (lb + 1));
  std::stringstream ss(inside);
  std::string tok;
  std::vector<double> vals;
  while (std::getline(ss, tok, ',')) {
    const std::string t = trim(tok);
    if (t.empty())
      continue;
    vals.push_back(std::stod(t));
  }
  if (vals.empty())
    return false;
  out = std::move(vals);
  return true;
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

// -------- OpenVINS pack yaml parsing (shared with check_jincbias_from_openvins_yaml.cpp) --------

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
  Eigen::Matrix<double, 15, 15> Sigma_z = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 9, 6> JincBias_ba_bg = Eigen::Matrix<double, 9, 6>::Zero();
  Eigen::Matrix<double, 15, 15> J_e_preint = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 15> J_s_preint = Eigen::Matrix<double, 15, 15>::Zero();
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

template <int R, int C>
static Eigen::Matrix<double, R, C> parse_yaml_list_matrix_in_section(const std::vector<std::string> &lines, const std::string &section,
                                                                     const std::string &key) {
  const std::string section_hdr = section + ":";
  const std::string key_hdr = key + ":";
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

    if (t != key_hdr)
      continue;

    Eigen::Matrix<double, R, C> M;
    M.setZero();
    size_t j = i + 1;
    for (int r = 0; r < R; r++) {
      while (j < lines.size() && trim(lines[j]).empty())
        j++;
      if (j >= lines.size()) {
        throw std::runtime_error("unexpected EOF while reading '" + section + "." + key + "'");
      }
      std::vector<double> row;
      if (!parse_bracket_list(lines[j], row) || static_cast<int>(row.size()) != C) {
        throw std::runtime_error("failed to parse '" + section + "." + key + "' row " + std::to_string(r));
      }
      for (int c = 0; c < C; c++) {
        M(r, c) = row[c];
      }
      j++;
    }
    return M;
  }
  throw std::runtime_error("missing block '" + section + "." + key + "' in YAML");
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

  // OpenVINS tool exports GTSAM-tangent preintegration outputs inside the same YAML.
  pack.Sigma_z = parse_yaml_list_matrix_in_section<15, 15>(lines, "gtsam_tangent_preint", "Sigma_z_15x15");
  pack.JincBias_ba_bg = parse_yaml_list_matrix_in_section<9, 6>(lines, "gtsam_tangent_preint", "JincBias_ba_bg_9x6");
  pack.J_e_preint = parse_yaml_list_matrix_in_section<15, 15>(lines, "gtsam_tangent_preint", "J_e_preint_15x15");
  pack.J_s_preint = parse_yaml_list_matrix_in_section<15, 15>(lines, "gtsam_tangent_preint", "J_s_preint_15x15");

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

} // namespace

int main(int argc, char **argv) {
  try {
    std::string ov_pack_yaml;
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
      } else if (a == "--gtsam_all") {
        gtsam_all = next();
      } else if (a == "--help" || a == "-h") {
        std::cout << "usage: " << argv[0] << " --ov_pack_yaml <imu_openvins_prop_preint.yaml> --gtsam_all <gtsam_ref_preint_all.txt>\n";
        return EXIT_SUCCESS;
      } else {
        throw std::runtime_error("unknown argument: " + a);
      }
    }

    if (ov_pack_yaml.empty() || gtsam_all.empty()) {
      std::cerr << "usage: " << argv[0] << " --ov_pack_yaml <imu_openvins_prop_preint.yaml> --gtsam_all <gtsam_ref_preint_all.txt>\n";
      return EXIT_FAILURE;
    }

    // -------- Parse inputs --------

    const OvPack pack = load_ov_pack_yaml(ov_pack_yaml);
    const Eigen::Matrix<double, 15, 15> Sigma_z_ov = pack.Sigma_z;
    const Eigen::Matrix<double, 9, 6> JincBias_ba_bg_ov = pack.JincBias_ba_bg;
    const Eigen::Matrix<double, 15, 15> J_e_preint_ov = pack.J_e_preint;
    const Eigen::Matrix<double, 15, 15> J_s_preint_ov = pack.J_s_preint;

    const Eigen::Matrix3d dR_gtsam = parse_block_matrix<3, 3>(gtsam_all, "dR_gtsam");
    const Eigen::Matrix<double, 3, 1> dP_gtsam = parse_block_matrix<3, 1>(gtsam_all, "dP_gtsam");
    const Eigen::Matrix<double, 3, 1> dV_gtsam = parse_block_matrix<3, 1>(gtsam_all, "dV_gtsam");
    const Eigen::Matrix<double, 1, 1> DT_gtsam_mat = parse_block_matrix<1, 1>(gtsam_all, "DT_gtsam");
    const double dt_gtsam = DT_gtsam_mat(0, 0);
    const Eigen::Matrix<double, 15, 15> Sigma_z_gtsam = parse_block_matrix<15, 15>(gtsam_all, "Sigma_z_gtsam");
    const Eigen::Matrix<double, 9, 6> JincBias_ba_bg_gtsam = parse_block_matrix<9, 6>(gtsam_all, "JincBias_ba_bg_gtsam");
    const PreintFactorJacobians jac_gtsam =
        build_preint_factor_jacobians_local(dR_gtsam, dP_gtsam.col(0), dV_gtsam.col(0), dt_gtsam, JincBias_ba_bg_gtsam);

    // -------- Compute dR/dP/dV from ov_pack_yaml (same formulas as preint reconstruction) --------

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

    // Teacher-style per-entry abs+rel check (same as CovPropConfig.hpp::expectNearAbsRel).
    constexpr double kAbsTol = 1e-4;
    constexpr double kRelTol = 1.5e-2;
    const AbsRelCheckResult Sigma_chk = absRelCheck(Sigma_z_gtsam, Sigma_z_ov, kAbsTol, kRelTol);
    const AbsRelCheckResult J_chk = absRelCheck(JincBias_ba_bg_gtsam, JincBias_ba_bg_ov, kAbsTol, kRelTol);
    const AbsRelCheckResult Je_chk = absRelCheck(jac_gtsam.J_e, J_e_preint_ov, kAbsTol, kRelTol);
    const AbsRelCheckResult Js_chk = absRelCheck(jac_gtsam.J_s, J_s_preint_ov, kAbsTol, kRelTol);

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
    std::cout << "  absTol=" << kAbsTol << " relTol=" << kRelTol << "\n";
    check("  Sigma_z abs+rel (entrywise)", Sigma_chk.ok);
    if (!Sigma_chk.ok) {
      std::vector<AbsRelFailEntry> fails = Sigma_chk.fails;
      std::sort(fails.begin(), fails.end(), [](const AbsRelFailEntry &a, const AbsRelFailEntry &b) { return a.violation > b.violation; });
      std::cout << "  Sigma_z failing entries      = " << Sigma_chk.fail_count << "\n";
      std::cout << "  Sigma_z worst entry          = (" << Sigma_chk.worst_r << "," << Sigma_chk.worst_c << ")  ["
                << describe_sigmaz_entry(Sigma_chk.worst_r, Sigma_chk.worst_c) << "]\n";
      std::cout << "    ref=" << Sigma_chk.worst_ref << " est=" << Sigma_chk.worst_est << "\n";
      std::cout << "    diff=" << Sigma_chk.worst_diff << " tol=" << Sigma_chk.worst_tol
                << " (violation=" << Sigma_chk.worst_violation << ")\n";
      std::cout << "  Sigma_z all failing entries:\n";
      for (const auto &f : fails) {
        std::cout << "    (" << f.r << "," << f.c << ")  [" << describe_sigmaz_entry(f.r, f.c) << "]"
                  << " ref=" << f.ref << " est=" << f.est << " diff=" << f.diff << " tol=" << f.tol << " (violation=" << f.violation
                  << ")\n";
      }
    }

    std::cout << "\nJincBias checks:\n";
    std::cout << "  absTol=" << kAbsTol << " relTol=" << kRelTol << "\n";
    check("  JincBias abs+rel (entrywise)", J_chk.ok);
    if (!J_chk.ok) {
      std::vector<AbsRelFailEntry> fails = J_chk.fails;
      std::sort(fails.begin(), fails.end(), [](const AbsRelFailEntry &a, const AbsRelFailEntry &b) { return a.violation > b.violation; });
      std::cout << "  JincBias failing entries     = " << J_chk.fail_count << "\n";
      std::cout << "  JincBias worst entry         = (" << J_chk.worst_r << "," << J_chk.worst_c << ")  ["
                << describe_jincbias_entry(J_chk.worst_r, J_chk.worst_c) << "]\n";
      std::cout << "    ref=" << J_chk.worst_ref << " est=" << J_chk.worst_est << "\n";
      std::cout << "    diff=" << J_chk.worst_diff << " tol=" << J_chk.worst_tol << " (violation=" << J_chk.worst_violation << ")\n";
      std::cout << "  JincBias all failing entries:\n";
      for (const auto &f : fails) {
        std::cout << "    (" << f.r << "," << f.c << ")  [" << describe_jincbias_entry(f.r, f.c) << "]"
                  << " ref=" << f.ref << " est=" << f.est << " diff=" << f.diff << " tol=" << f.tol << " (violation=" << f.violation
                  << ")\n";
      }
    }

    std::cout << "\nJ_e_preint checks:\n";
    std::cout << "  absTol=" << kAbsTol << " relTol=" << kRelTol << "\n";
    check("  J_e_preint abs+rel (entrywise)", Je_chk.ok);
    if (!Je_chk.ok) {
      std::vector<AbsRelFailEntry> fails = Je_chk.fails;
      std::sort(fails.begin(), fails.end(), [](const AbsRelFailEntry &a, const AbsRelFailEntry &b) { return a.violation > b.violation; });
      std::cout << "  J_e_preint failing entries   = " << Je_chk.fail_count << "\n";
      std::cout << "  J_e_preint worst entry       = (" << Je_chk.worst_r << "," << Je_chk.worst_c << ")  ["
                << describe_jac15_entry(Je_chk.worst_r, Je_chk.worst_c) << "]\n";
      std::cout << "    ref=" << Je_chk.worst_ref << " est=" << Je_chk.worst_est << "\n";
      std::cout << "    diff=" << Je_chk.worst_diff << " tol=" << Je_chk.worst_tol << " (violation=" << Je_chk.worst_violation << ")\n";
      std::cout << "  J_e_preint all failing entries:\n";
      for (const auto &f : fails) {
        std::cout << "    (" << f.r << "," << f.c << ")  [" << describe_jac15_entry(f.r, f.c) << "]"
                  << " ref=" << f.ref << " est=" << f.est << " diff=" << f.diff << " tol=" << f.tol << " (violation=" << f.violation
                  << ")\n";
      }
    }

    std::cout << "\nJ_s_preint checks:\n";
    std::cout << "  absTol=" << kAbsTol << " relTol=" << kRelTol << "\n";
    check("  J_s_preint abs+rel (entrywise)", Js_chk.ok);
    if (!Js_chk.ok) {
      std::vector<AbsRelFailEntry> fails = Js_chk.fails;
      std::sort(fails.begin(), fails.end(), [](const AbsRelFailEntry &a, const AbsRelFailEntry &b) { return a.violation > b.violation; });
      std::cout << "  J_s_preint failing entries   = " << Js_chk.fail_count << "\n";
      std::cout << "  J_s_preint worst entry       = (" << Js_chk.worst_r << "," << Js_chk.worst_c << ")  ["
                << describe_jac15_entry(Js_chk.worst_r, Js_chk.worst_c) << "]\n";
      std::cout << "    ref=" << Js_chk.worst_ref << " est=" << Js_chk.worst_est << "\n";
      std::cout << "    diff=" << Js_chk.worst_diff << " tol=" << Js_chk.worst_tol << " (violation=" << Js_chk.worst_violation << ")\n";
      std::cout << "  J_s_preint all failing entries:\n";
      for (const auto &f : fails) {
        std::cout << "    (" << f.r << "," << f.c << ")  [" << describe_jac15_entry(f.r, f.c) << "]"
                  << " ref=" << f.ref << " est=" << f.est << " diff=" << f.diff << " tol=" << f.tol << " (violation=" << f.violation
                  << ")\n";
      }
    }

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
