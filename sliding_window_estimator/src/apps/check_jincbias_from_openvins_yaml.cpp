#include <Eigen/Dense>

#include <gtsam/geometry/Rot3.h>

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
  if (!ifs.is_open())
    throw std::runtime_error("unable to open file: " + path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    lines.push_back(line);
  }
  return lines;
}

static std::vector<double> parse_inline_list(const std::string &line, int expected_n, const std::string &what) {
  const size_t lb = line.find('[');
  const size_t rb = line.find(']', lb == std::string::npos ? 0 : lb + 1);
  if (lb == std::string::npos || rb == std::string::npos || rb <= lb)
    throw std::runtime_error("failed to parse list for " + what);

  const std::string inside = line.substr(lb + 1, rb - (lb + 1));
  std::stringstream ss(inside);
  std::string tok;
  std::vector<double> vals;
  while (std::getline(ss, tok, ',')) {
    const std::string t = trim(tok);
    if (!t.empty())
      vals.push_back(std::stod(t));
  }
  if (expected_n > 0 && static_cast<int>(vals.size()) != expected_n)
    throw std::runtime_error("expected " + std::to_string(expected_n) + " elements for " + what);
  return vals;
}

static inline Eigen::Matrix3d crossMx(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

// OpenVINS JPL quaternion (x,y,z,w) -> R_GtoI
static Eigen::Matrix3d quat_xyzw_to_R_GtoI_JPL(const std::vector<double> &q_xyzw) {
  if (q_xyzw.size() != 4)
    throw std::runtime_error("quat_xyzw_to_R_GtoI_JPL: expected 4");
  Eigen::Vector4d qjpl(q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3]);
  qjpl.normalize();
  const Eigen::Vector3d qv = qjpl.head<3>();
  const double qw = qjpl(3);
  const Eigen::Matrix3d qx = crossMx(qv);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  return (2.0 * qw * qw - 1.0) * I - 2.0 * qw * qx + 2.0 * qv * qv.transpose();
}

static inline Eigen::Matrix<double, 9, 6> swap_ba_bg_and_bg_ba(const Eigen::Matrix<double, 9, 6> &J) {
  Eigen::Matrix<double, 9, 6> out = Eigen::Matrix<double, 9, 6>::Zero();
  out.block<9, 3>(0, 0) = J.block<9, 3>(0, 3);
  out.block<9, 3>(0, 3) = J.block<9, 3>(0, 0);
  return out;
}

template <typename Derived>
static void print_matrix(const std::string &name, const Eigen::MatrixBase<Derived> &M) {
  std::cout << name << " (" << M.rows() << "x" << M.cols() << ")\n";
  std::cout.setf(std::ios::fixed);
  std::cout << std::setprecision(18);
  for (int r = 0; r < M.rows(); r++) {
    for (int c = 0; c < M.cols(); c++) {
      std::cout << M(r, c);
      if (c + 1 < M.cols())
        std::cout << " ";
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

template <typename Derived>
static double maxAbs(const Eigen::MatrixBase<Derived> &M) {
  return M.cwiseAbs().maxCoeff();
}

struct Parsed {
  Eigen::Matrix3d Rws_s = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rws_e = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 15, 15> Phi = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 9, 6> J_ba_bg_yaml = Eigen::Matrix<double, 9, 6>::Zero();
  bool has_xs_q = false;
  bool has_xe_q = false;
  bool has_Phi = false;
  bool has_J_ba_bg = false;
};

static Parsed parse_one_yaml_strict(const std::string &path) {
  // Strongly constrained to the single YAML we use:
  // - we only parse: xs_nominal.q_GtoI_xyzw, xe_nominal.q_GtoI_xyzw, Phi_15x15, and JincBias_ba_bg_9x6
  const auto lines = read_lines(path);
  Parsed out;

  enum class Sec { None, Xs, Xe, Phi, Gtsam };
  Sec sec = Sec::None;
  int phi_row = -1;
  int jb_ba_bg_row = -1;

  for (size_t i = 0; i < lines.size(); i++) {
    const std::string &raw = lines[i];
    const std::string t = trim(raw);
    if (t.empty())
      continue;

    // Consume fixed-size blocks first (exact row counts).
    if (phi_row >= 0) {
      const std::vector<double> row = parse_inline_list(t, 15, "Phi_15x15 row");
      for (int c = 0; c < 15; c++)
        out.Phi(phi_row, c) = row[static_cast<size_t>(c)];
      phi_row++;
      if (phi_row == 15) {
        phi_row = -1;
        out.has_Phi = true;
      }
      continue;
    }
    if (jb_ba_bg_row >= 0) {
      const std::vector<double> row = parse_inline_list(t, 6, "JincBias_ba_bg_9x6 row");
      for (int c = 0; c < 6; c++)
        out.J_ba_bg_yaml(jb_ba_bg_row, c) = row[static_cast<size_t>(c)];
      jb_ba_bg_row++;
      if (jb_ba_bg_row == 9) {
        jb_ba_bg_row = -1;
        out.has_J_ba_bg = true;
      }
      continue;
    }

    // Top-level section switches (raw[0] != ' ').
    if (!raw.empty() && raw[0] != ' ') {
      if (t == "xs_nominal:") {
        sec = Sec::Xs;
      } else if (t == "xe_nominal:") {
        sec = Sec::Xe;
      } else if (t == "Phi_15x15:") {
        sec = Sec::Phi;
        phi_row = 0;
      } else if (t == "gtsam_tangent_preint:") {
        sec = Sec::Gtsam;
      } else {
        sec = Sec::None;
      }
      continue;
    }

    // Keys inside sections (we only parse what we need).
    if (sec == Sec::Xs && t.rfind("q_GtoI_xyzw:", 0) == 0) {
      const std::vector<double> q = parse_inline_list(t, 4, "xs_nominal.q_GtoI_xyzw");
      out.Rws_s = quat_xyzw_to_R_GtoI_JPL(q).transpose();
      out.has_xs_q = true;
    } else if (sec == Sec::Xe && t.rfind("q_GtoI_xyzw:", 0) == 0) {
      const std::vector<double> q = parse_inline_list(t, 4, "xe_nominal.q_GtoI_xyzw");
      out.Rws_e = quat_xyzw_to_R_GtoI_JPL(q).transpose();
      out.has_xe_q = true;
    } else if (sec == Sec::Gtsam && t == "JincBias_ba_bg_9x6:") {
      jb_ba_bg_row = 0;
    }
  }

  if (!out.has_xs_q || !out.has_xe_q || !out.has_Phi || !out.has_J_ba_bg) {
    throw std::runtime_error("YAML missing required blocks (format changed?): need xs/xe q, Phi_15x15, and JincBias_ba_bg_9x6");
  }
  return out;
}

} // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 2) {
      std::cerr << "usage: " << argv[0] << " <imu_openvins_prop_preint.yaml>\n";
      return EXIT_FAILURE;
    }

    const Parsed pack = parse_one_yaml_strict(argv[1]);

    // Need only rotation part for Jr/Jr_inv.
    const Eigen::Matrix3d dR = pack.Rws_s.transpose() * pack.Rws_e;
    const gtsam::Vector3 phi = gtsam::Rot3::Logmap(gtsam::Rot3(dR));
    const Eigen::Matrix3d Jr_inv = gtsam::so3::DexpFunctor(phi).rightJacobianInverse();

    // Build the 9x9 block: [x_dp, x_dtheta, x_dv] -> [z_dphi, z_dp, z_dv]
    // x uses: dp (world), dtheta (world), dv (world)
    // z uses: dphi, dp, dv (GTSAM tangent preint)
    const Eigen::Matrix3d Rws_s_T = pack.Rws_s.transpose();
    const Eigen::Matrix3d Rwe_from_s = pack.Rws_s * dR; // == Rws_e

    Eigen::Matrix<double, 9, 9> G9_inv = Eigen::Matrix<double, 9, 9>::Zero();
    G9_inv.block<3, 3>(0, 3) = Jr_inv * Rwe_from_s.transpose(); // dphi <- dtheta
    G9_inv.block<3, 3>(3, 0) = Rws_s_T;                         // dp   <- dp_e
    G9_inv.block<3, 3>(6, 6) = Rws_s_T;                         // dv   <- dv_e

    // Build x-error bias Jacobian from OV Phi:
    // OV state order: [dtheta_body, dp_world, dv_world, dbg_body, dba_body]
    // x-error order (for G9_inv): [dp_world, dtheta_world, dv_world]
    // with dtheta_world = Rws_e * dtheta_body.
    const Eigen::Matrix<double, 3, 6> J_dp_bias = pack.Phi.block<3, 6>(3, 9);
    const Eigen::Matrix<double, 3, 6> J_dtheta_bias = pack.Rws_e * pack.Phi.block<3, 6>(0, 9);
    const Eigen::Matrix<double, 3, 6> J_dv_bias = pack.Phi.block<3, 6>(6, 9);

    Eigen::Matrix<double, 9, 6> J_x_bias;
    J_x_bias.block<3, 6>(0, 0) = J_dp_bias;
    J_x_bias.block<3, 6>(3, 0) = J_dtheta_bias;
    J_x_bias.block<3, 6>(6, 0) = J_dv_bias;

    const Eigen::Matrix<double, 9, 6> J_bg_ba_recomputed = G9_inv * J_x_bias;
    const Eigen::Matrix<double, 9, 6> J_ba_bg_recomputed = swap_ba_bg_and_bg_ba(J_bg_ba_recomputed);

    print_matrix("JincBias_ba_bg_from_yaml", pack.J_ba_bg_yaml);
    print_matrix("JincBias_ba_bg_recomputed", J_ba_bg_recomputed);

    std::cout << std::setprecision(18) << "maxAbs diff: " << maxAbs(pack.J_ba_bg_yaml - J_ba_bg_recomputed) << "\n";
    return EXIT_SUCCESS;

  } catch (const std::exception &e) {
    std::cerr << "check_jincbias_from_openvins_yaml failed: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
