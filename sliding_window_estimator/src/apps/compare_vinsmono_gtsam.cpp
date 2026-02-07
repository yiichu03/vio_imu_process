#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

static inline std::string trim(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])))
    b++;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
    e--;
  return s.substr(b, e - b);
}

static bool readNextDataLine(std::istream& is, std::string& out) {
  std::string line;
  while (std::getline(is, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    const std::string t = trim(line);
    if (t.empty() || t[0] == '#')
      continue;
    out = t;
    return true;
  }
  return false;
}

static bool parseHeaderLine(const std::string& line, std::string& name, int& rows, int& cols) {
  const size_t lb = line.find('(');
  const size_t rb = line.find(')', lb == std::string::npos ? 0 : lb + 1);
  if (lb == std::string::npos || rb == std::string::npos || rb <= lb + 1)
    return false;

  name = trim(line.substr(0, lb));
  const std::string inside = trim(line.substr(lb + 1, rb - (lb + 1))); // "RxC"
  const size_t x = inside.find('x');
  if (x == std::string::npos)
    return false;
  rows = std::stoi(trim(inside.substr(0, x)));
  cols = std::stoi(trim(inside.substr(x + 1)));
  return !name.empty() && rows > 0 && cols > 0;
}

static std::unordered_map<std::string, Eigen::MatrixXd> readMatrixBlocks(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("unable to open: " + path);
  }

  std::unordered_map<std::string, Eigen::MatrixXd> blocks;
  std::string line;
  while (readNextDataLine(ifs, line)) {
    std::string name;
    int rows = 0, cols = 0;
    if (!parseHeaderLine(line, name, rows, cols)) {
      throw std::runtime_error("failed to parse matrix header line: '" + line + "' in " + path);
    }

    Eigen::MatrixXd mat(rows, cols);
    for (int r = 0; r < rows; ++r) {
      std::string rowLine;
      if (!readNextDataLine(ifs, rowLine)) {
        throw std::runtime_error("unexpected EOF reading matrix '" + name + "' from " + path);
      }
      std::istringstream iss(rowLine);
      for (int c = 0; c < cols; ++c) {
        double v = 0.0;
        if (!(iss >> v)) {
          throw std::runtime_error("failed to parse matrix '" + name + "' row " + std::to_string(r) + " from " + path);
        }
        mat(r, c) = v;
      }
    }
    blocks[name] = std::move(mat);
  }
  return blocks;
}

static bool expectNearAbsRel(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double absTol, double relTol, const std::string& what) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    std::cerr << "[FAIL] " << what << ": shape mismatch: " << a.rows() << "x" << a.cols() << " vs " << b.rows() << "x" << b.cols()
              << "\n";
    return false;
  }

  double maxViolation = -1.0;
  int maxR = 0;
  int maxC = 0;
  double maxDiff = 0.0;
  double maxTol = 0.0;
  double maxA = 0.0;
  double maxB = 0.0;

  for (int r = 0; r < a.rows(); ++r) {
    for (int c = 0; c < a.cols(); ++c) {
      const double va = a(r, c);
      const double vb = b(r, c);
      const double diff = std::abs(va - vb);
      const double scale = std::max(std::abs(va), std::abs(vb));
      const double tol = absTol + relTol * scale;
      const double violation = diff - tol;
      if (violation > maxViolation) {
        maxViolation = violation;
        maxR = r;
        maxC = c;
        maxDiff = diff;
        maxTol = tol;
        maxA = va;
        maxB = vb;
      }
    }
  }

  if (maxViolation > 0.0) {
    std::cerr << std::setprecision(18);
    std::cerr << "[FAIL] " << what << ": max violation at (" << maxR << "," << maxC << ")\n";
    std::cerr << "  a=" << maxA << " b=" << maxB << " |a-b|=" << maxDiff << " tol=" << maxTol << " (abs=" << absTol << ", rel=" << relTol
              << ")\n";
    return false;
  }

  std::cout << "[ OK ] " << what << "\n";
  return true;
}

static Eigen::MatrixXd getBlockOrThrow(const std::unordered_map<std::string, Eigen::MatrixXd>& blocks, const std::string& name,
                                       const std::string& path) {
  auto it = blocks.find(name);
  if (it == blocks.end()) {
    throw std::runtime_error("missing block '" + name + "' in " + path);
  }
  return it->second;
}

static void compareAll(const std::string& vinsAll, const std::string& gtsamAll, double absTol, double relTol) {
  const auto vins = readMatrixBlocks(vinsAll);
  const auto gtsam = readMatrixBlocks(gtsamAll);

  const Eigen::MatrixXd dR_vins = getBlockOrThrow(vins, "dR_vins", vinsAll);
  const Eigen::MatrixXd dR_gtsam = getBlockOrThrow(gtsam, "dR_gtsam", gtsamAll);
  const Eigen::MatrixXd dP_vins = getBlockOrThrow(vins, "dP_vins", vinsAll);
  const Eigen::MatrixXd dP_gtsam = getBlockOrThrow(gtsam, "dP_gtsam", gtsamAll);
  const Eigen::MatrixXd dV_vins = getBlockOrThrow(vins, "dV_vins", vinsAll);
  const Eigen::MatrixXd dV_gtsam = getBlockOrThrow(gtsam, "dV_gtsam", gtsamAll);
  const Eigen::MatrixXd DT_vins = getBlockOrThrow(vins, "DT_vins", vinsAll);
  const Eigen::MatrixXd DT_gtsam = getBlockOrThrow(gtsam, "DT_gtsam", gtsamAll);

  const Eigen::MatrixXd Sigma_z_vins = getBlockOrThrow(vins, "Sigma_z_vins_gtsam", vinsAll);
  const Eigen::MatrixXd Sigma_z_gtsam = getBlockOrThrow(gtsam, "Sigma_z_gtsam", gtsamAll);
  const Eigen::MatrixXd JincBias_vins = getBlockOrThrow(vins, "JincBias_ba_bg_vins", vinsAll);
  const Eigen::MatrixXd JincBias_gtsam = getBlockOrThrow(gtsam, "JincBias_ba_bg_gtsam", gtsamAll);

  bool ok = true;
  ok &= expectNearAbsRel(dR_vins, dR_gtsam, absTol, relTol, "dR");
  ok &= expectNearAbsRel(dP_vins, dP_gtsam, absTol, relTol, "dP");
  ok &= expectNearAbsRel(dV_vins, dV_gtsam, absTol, relTol, "dV");
  ok &= expectNearAbsRel(DT_vins, DT_gtsam, absTol, relTol, "DT");
  ok &= expectNearAbsRel(Sigma_z_vins, Sigma_z_gtsam, absTol, relTol, "Sigma_z (z=[dphi,dp,dv,dba,dbg])");
  ok &= expectNearAbsRel(JincBias_vins, JincBias_gtsam, absTol, relTol, "JincBias_ba_bg (rows=[dphi,dp,dv])");

  if (!ok) {
    throw std::runtime_error("comparison failed");
  }
}

} // namespace

int main(int argc, char** argv) {
  try {
    std::string vins_all;
    std::string gtsam_all;
    double absTol = 1e-4;
    double relTol = 1.5e-2;

    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("missing value after " + a);
        }
        return std::string(argv[++i]);
      };
      if (a == "--vins_all") {
        vins_all = next();
      } else if (a == "--gtsam_all") {
        gtsam_all = next();
      } else if (a == "--abs_tol") {
        absTol = std::stod(next());
      } else if (a == "--rel_tol") {
        relTol = std::stod(next());
      } else if (a == "--help" || a == "-h") {
        std::cout << "usage: " << argv[0]
                  << " --vins_all <export_vins_preint_pack.txt> --gtsam_all <gtsam_ref_preint_all.txt> [--abs_tol A] [--rel_tol R]\n";
        return EXIT_SUCCESS;
      } else {
        throw std::runtime_error("unknown argument: " + a);
      }
    }

    if (vins_all.empty() || gtsam_all.empty()) {
      std::cerr << "usage: " << argv[0]
                << " --vins_all <export_vins_preint_pack.txt> --gtsam_all <gtsam_ref_preint_all.txt> [--abs_tol A] [--rel_tol R]\n";
      return EXIT_FAILURE;
    }

    compareAll(vins_all, gtsam_all, absTol, relTol);
    return EXIT_SUCCESS;

  } catch (const std::exception& e) {
    std::cerr << "compare_vinsmono_gtsam failed: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}

