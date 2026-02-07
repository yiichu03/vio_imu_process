#include <Eigen/Dense>

#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct ImuRow {
  double t = 0.0;
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
};

struct Config {
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
  double sigma_g_c = 0.0;
  double sigma_a_c = 0.0;
  double sigma_gw_c = 0.0;
  double sigma_aw_c = 0.0;
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
};

struct GtsamPreintOut {
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  Eigen::Vector3d dP = Eigen::Vector3d::Zero();
  Eigen::Vector3d dV = Eigen::Vector3d::Zero();
  double DT = 0.0;
  Eigen::Matrix<double, 9, 9> Sigma_z9 = Eigen::Matrix<double, 9, 9>::Zero();        // z9=[dphi,dp,dv]
  Eigen::Matrix<double, 6, 6> Sigma_bias_rw = Eigen::Matrix<double, 6, 6>::Zero();   // z6=[dba,dbg]
  Eigen::Matrix<double, 15, 15> Sigma_z15 = Eigen::Matrix<double, 15, 15>::Zero();   // blockdiag(z9,z6)
  Eigen::Matrix<double, 9, 6> JincBias_ba_bg = Eigen::Matrix<double, 9, 6>::Zero();  // rows=[dphi,dp,dv], cols=[dba,dbg]
};

static inline std::string trim(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])))
    b++;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
    e--;
  return s.substr(b, e - b);
}

static std::string slurp_file(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("unable to open file: " + path);
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  return buffer.str();
}

static bool find_yaml_scalar_double(const std::string& content, const std::string& key, double& out) {
  const std::string needle = key + ":";
  size_t pos = content.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos])))
    pos++;
  size_t end = pos;
  while (end < content.size()) {
    char c = content[end];
    if (std::isdigit(static_cast<unsigned char>(c)) || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E') {
      end++;
      continue;
    }
    break;
  }
  if (end == pos)
    return false;
  out = std::stod(content.substr(pos, end - pos));
  return true;
}

static bool find_yaml_inline_list(const std::string& content, const std::string& key, std::vector<double>& out) {
  const std::string needle = key + ":";
  size_t pos = content.find(needle);
  if (pos == std::string::npos)
    return false;
  pos = content.find('[', pos + needle.size());
  if (pos == std::string::npos)
    return false;
  size_t end = content.find(']', pos + 1);
  if (end == std::string::npos)
    return false;
  std::string inside = content.substr(pos + 1, end - (pos + 1));
  std::vector<double> vals;
  std::stringstream ss(inside);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    std::string t = trim(tok);
    if (t.empty())
      continue;
    vals.push_back(std::stod(t));
  }
  out = std::move(vals);
  return !out.empty();
}

static bool find_yaml_inline_list_under_section(const std::string& content, const std::string& section, const std::string& key,
                                                std::vector<double>& out) {
  std::istringstream iss(content);
  std::string line;
  bool in_section = false;
  const std::string section_hdr = section + ":";
  const std::string key_hdr = key + ":";

  while (std::getline(iss, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    size_t first = 0;
    while (first < line.size() && (line[first] == ' ' || line[first] == '\t'))
      first++;
    if (first == line.size())
      continue;

    if (!in_section) {
      if (first == 0 && line.compare(0, section_hdr.size(), section_hdr) == 0) {
        in_section = true;
      }
      continue;
    }

    if (first == 0) {
      break;
    }

    if (line.compare(first, key_hdr.size(), key_hdr) != 0) {
      continue;
    }

    size_t lb = line.find('[', first + key_hdr.size());
    if (lb == std::string::npos)
      return false;
    size_t rb = line.find(']', lb + 1);
    if (rb == std::string::npos)
      return false;
    std::string inside = line.substr(lb + 1, rb - (lb + 1));

    std::vector<double> vals;
    std::stringstream ss(inside);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      std::string t = trim(tok);
      if (t.empty())
        continue;
      vals.push_back(std::stod(t));
    }
    out = std::move(vals);
    return !out.empty();
  }
  return false;
}

static Eigen::Vector3d to_vec3(const std::vector<double>& v, const std::string& what) {
  if (v.size() != 3) {
    throw std::runtime_error("expected 3 elements for " + what);
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

static Config load_config_yaml(const std::string& path) {
  const std::string content = slurp_file(path);
  Config cfg;

  // gravity: [x,y,z]
  std::vector<double> g;
  if (!find_yaml_inline_list(content, "gravity", g) || g.size() != 3) {
    throw std::runtime_error("config_yaml missing 'gravity: [x,y,z]': " + path);
  }
  cfg.gravity = to_vec3(g, "gravity");

  if (!find_yaml_scalar_double(content, "sigma_g_c", cfg.sigma_g_c) || !find_yaml_scalar_double(content, "sigma_a_c", cfg.sigma_a_c) ||
      !find_yaml_scalar_double(content, "sigma_gw_c", cfg.sigma_gw_c) || !find_yaml_scalar_double(content, "sigma_aw_c", cfg.sigma_aw_c)) {
    throw std::runtime_error("config_yaml missing one of imu_params.sigma_*: " + path);
  }

  // biases (optional)
  std::vector<double> bg, ba;
  if (find_yaml_inline_list_under_section(content, "biases", "gyro", bg) && bg.size() == 3) {
    cfg.bias_gyro = to_vec3(bg, "biases.gyro");
  }
  if (find_yaml_inline_list_under_section(content, "biases", "accel", ba) && ba.size() == 3) {
    cfg.bias_accel = to_vec3(ba, "biases.accel");
  }
  return cfg;
}

static std::vector<ImuRow> read_imu_txt(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("unable to open imu_txt: " + path);
  }

  std::vector<ImuRow> rows;
  std::string line;
  while (std::getline(ifs, line)) {
    size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
      i++;
    if (i == line.size() || line[i] == '#')
      continue;

    std::istringstream iss(line);
    ImuRow r;
    double gx, gy, gz, ax, ay, az;
    if (!(iss >> r.t >> gx >> gy >> gz >> ax >> ay >> az))
      continue;
    r.w << gx, gy, gz;
    r.a << ax, ay, az;
    rows.push_back(r);
  }
  if (rows.size() < 2) {
    throw std::runtime_error("imu_txt has too few samples: " + path);
  }
  std::sort(rows.begin(), rows.end(), [](const ImuRow& a, const ImuRow& b) { return a.t < b.t; });
  return rows;
}

template <typename Derived>
static void appendMatrixBlock(std::ostream& os, const std::string& name, const Eigen::MatrixBase<Derived>& mat) {
  os << name << " (" << mat.rows() << "x" << mat.cols() << ")\n";
  os.setf(std::ios::fixed);
  os << std::setprecision(18);
  for (int r = 0; r < mat.rows(); ++r) {
    for (int c = 0; c < mat.cols(); ++c) {
      os << mat(r, c);
      if (c < mat.cols() - 1) {
        os << ' ';
      }
    }
    os << '\n';
  }
  os << '\n';
}

static std::string join_path(const std::string& dir, const std::string& name) {
  if (dir.empty())
    return name;
  if (dir.back() == '/')
    return dir + name;
  return dir + "/" + name;
}

static GtsamPreintOut run_gtsam_tangent_preintegration_9d(const std::vector<ImuRow>& rows, const Config& cfg) {
  if (rows.size() < 2) {
    throw std::runtime_error("need >= 2 imu samples to preintegrate");
  }

  const gtsam::Vector3 n_gravity(cfg.gravity.x(), cfg.gravity.y(), cfg.gravity.z());
  auto params = std::make_shared<gtsam::PreintegratedImuMeasurements::Params>(n_gravity);
  params->gyroscopeCovariance = (cfg.sigma_g_c * cfg.sigma_g_c) * Eigen::Matrix3d::Identity();
  params->accelerometerCovariance = (cfg.sigma_a_c * cfg.sigma_a_c) * Eigen::Matrix3d::Identity();
  params->integrationCovariance = 1e-16 * Eigen::Matrix3d::Identity();
  params->use2ndOrderCoriolis = false;

  // NOTE: ConstantBias takes (accelBias, gyroBias).
  const gtsam::imuBias::ConstantBias bias_gtsam(cfg.bias_accel, cfg.bias_gyro);
  gtsam::PreintegratedImuMeasurementsT<gtsam::TangentPreintegration> pim(params, bias_gtsam);

  for (size_t k = 0; k + 1 < rows.size(); k++) {
    const ImuRow& r0 = rows[k];
    const ImuRow& r1 = rows[k + 1];
    const double dt = r1.t - r0.t;
    if (!(dt > 0.0))
      continue;
    const Eigen::Vector3d omega = 0.5 * (r0.w + r1.w);
    const Eigen::Vector3d acc = 0.5 * (r0.a + r1.a);
    pim.integrateMeasurement(acc, omega, dt);
  }

  GtsamPreintOut out;
  out.dR = pim.deltaRij().matrix();
  out.dP = pim.deltaPij();
  out.dV = pim.deltaVij();
  out.DT = pim.deltaTij();

  out.Sigma_z9 = pim.preintMeasCov();

  const Eigen::Matrix<double, 9, 3> H_bg = pim.preintegrated_H_biasOmega();
  const Eigen::Matrix<double, 9, 3> H_ba = pim.preintegrated_H_biasAcc();
  out.JincBias_ba_bg.setZero();
  out.JincBias_ba_bg.block<9, 3>(0, 0) = H_ba;
  out.JincBias_ba_bg.block<9, 3>(0, 3) = H_bg;

  out.Sigma_bias_rw.setZero();
  out.Sigma_bias_rw.block<3, 3>(0, 0) = (cfg.sigma_aw_c * cfg.sigma_aw_c * out.DT) * Eigen::Matrix3d::Identity();  // dba
  out.Sigma_bias_rw.block<3, 3>(3, 3) = (cfg.sigma_gw_c * cfg.sigma_gw_c * out.DT) * Eigen::Matrix3d::Identity();  // dbg

  out.Sigma_z15.setZero();
  out.Sigma_z15.block<9, 9>(0, 0) = out.Sigma_z9;
  out.Sigma_z15.block<6, 6>(9, 9) = out.Sigma_bias_rw;
  return out;
}

} // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 4) {
      std::cerr << "usage: " << argv[0] << " <imu_txt> <config_yaml> <out_dir>\n";
      return EXIT_FAILURE;
    }

    const std::string imu_txt = argv[1];
    const std::string config_yaml = argv[2];
    const std::string out_dir = argv[3];
    std::filesystem::create_directories(out_dir);

    const Config cfg = load_config_yaml(config_yaml);
    const std::vector<ImuRow> rows = read_imu_txt(imu_txt);
    const GtsamPreintOut out = run_gtsam_tangent_preintegration_9d(rows, cfg);

    const std::string all_path = join_path(out_dir, "gtsam_ref_orb_preint_all.txt");
    std::ofstream ofs(all_path, std::ios::trunc);
    if (!ofs.is_open()) {
      throw std::runtime_error("failed to open for writing: " + all_path);
    }

    ofs << "# gtsam_ref_orb_preint_from_txt\n";
    ofs << "# imu_txt: " << imu_txt << "\n";
    ofs << "# config_yaml: " << config_yaml << "\n";
    ofs << std::setprecision(17) << "# interval: ts=" << rows.front().t << " te=" << rows.back().t << " DT=" << out.DT << "\n";
    ofs << "# z9_order: [dphi, dp, dv]\n";
    ofs << "# z6_order: [dba, dbg]\n";
    ofs << "# z15_order: [dphi, dp, dv, dba, dbg] (blockdiag)\n\n";

    appendMatrixBlock(ofs, "dR_gtsam", out.dR);
    appendMatrixBlock(ofs, "dP_gtsam", out.dP);
    appendMatrixBlock(ofs, "dV_gtsam", out.dV);
    Eigen::Matrix<double, 1, 1> DT_mat;
    DT_mat(0, 0) = out.DT;
    appendMatrixBlock(ofs, "DT_gtsam", DT_mat);

    appendMatrixBlock(ofs, "Sigma_z9_gtsam", out.Sigma_z9);
    appendMatrixBlock(ofs, "JincBias_ba_bg_gtsam", out.JincBias_ba_bg);
    appendMatrixBlock(ofs, "Sigma_bias_rw_gtsam", out.Sigma_bias_rw);
    appendMatrixBlock(ofs, "Sigma_z15_gtsam", out.Sigma_z15);

    std::cout << std::setprecision(17) << "integrated interval: ts=" << rows.front().t << " te=" << rows.back().t << " DT=" << out.DT << "\n";
    std::cout << "wrote:\n"
              << "  " << all_path << "\n";
    return EXIT_SUCCESS;

  } catch (const std::exception& e) {
    std::cerr << "gtsam_ref_orb_preint_from_txt failed: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}

