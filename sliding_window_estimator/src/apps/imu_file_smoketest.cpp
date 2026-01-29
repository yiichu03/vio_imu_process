#include <Eigen/Dense>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Rot3.h>
#include <gflags/gflags.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

DEFINE_string(imu_txt, "", "Path to imu txt: t gx gy gz ax ay az");
DEFINE_double(sigma_w, 1.7e-4, "gyro white noise density (rad/s/sqrt(Hz))");
DEFINE_double(sigma_a, 2.0e-3, "acc white noise density (m/s^2/sqrt(Hz))");
DEFINE_double(sigma_wb, 1.9e-5, "gyro random walk (rad/s^2/sqrt(Hz))");
DEFINE_double(sigma_ab, 3.0e-4, "acc random walk (m/s^3/sqrt(Hz))");
DEFINE_double(g, 9.81, "gravity magnitude");
DEFINE_double(bg0_x, 0.0, "initial gyro bias x");
DEFINE_double(bg0_y, 0.0, "initial gyro bias y");
DEFINE_double(bg0_z, 0.0, "initial gyro bias z");
DEFINE_double(ba0_x, 0.0, "initial accel bias x");
DEFINE_double(ba0_y, 0.0, "initial accel bias y");
DEFINE_double(ba0_z, 0.0, "initial accel bias z");

struct ImuRow {
  double t;
  Eigen::Vector3d w;
  Eigen::Vector3d a;
};

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_imu_txt.empty()) {
    std::cerr << "Missing --imu_txt\n";
    return 2;
  }

  std::ifstream ifs(FLAGS_imu_txt);
  if (!ifs) {
    std::cerr << "Failed to open: " << FLAGS_imu_txt << "\n";
    return 2;
  }

  std::vector<ImuRow> rows;
  rows.reserve(10000);

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    ImuRow r;
    double gx, gy, gz, ax, ay, az;
    if (!(iss >> r.t >> gx >> gy >> gz >> ax >> ay >> az)) continue;
    r.w = Eigen::Vector3d(gx, gy, gz);
    r.a = Eigen::Vector3d(ax, ay, az);
    rows.push_back(r);
  }

  if (rows.size() < 2) {
    std::cerr << "Need >= 2 imu rows, got " << rows.size() << "\n";
    return 2;
  }

  const double t0 = rows.front().t;
  const double t1 = rows.back().t;

  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(FLAGS_g);
  p->gyroscopeCovariance = Eigen::Matrix3d::Identity() * (FLAGS_sigma_w * FLAGS_sigma_w);
  p->accelerometerCovariance = Eigen::Matrix3d::Identity() * (FLAGS_sigma_a * FLAGS_sigma_a);
  p->biasAccCovariance = Eigen::Matrix3d::Identity() * (FLAGS_sigma_ab * FLAGS_sigma_ab);
  p->biasOmegaCovariance = Eigen::Matrix3d::Identity() * (FLAGS_sigma_wb * FLAGS_sigma_wb);
  p->integrationCovariance = Eigen::Matrix3d::Identity() * 1e-8;

  gtsam::imuBias::ConstantBias bias(
    Eigen::Vector3d(FLAGS_ba0_x, FLAGS_ba0_y, FLAGS_ba0_z),
    Eigen::Vector3d(FLAGS_bg0_x, FLAGS_bg0_y, FLAGS_bg0_z)
  );

  gtsam::PreintegratedCombinedMeasurements pim(p, bias);

  for (size_t i = 0; i + 1 < rows.size(); i++) {
    const double dt = rows[i + 1].t - rows[i].t;
    if (dt <= 0) continue;
    pim.integrateMeasurement(rows[i].a, rows[i].w, dt);
  }

  const auto delta = pim.deltaRij().matrix();
  const auto cov = pim.preintMeasCov();

  std::cout << "IMU rows: " << rows.size() << "\n";
  std::cout << "t0=" << t0 << " t1=" << t1 << " dt=" << (t1 - t0) << "\n";
  std::cout << "deltaR (matrix):\n" << delta << "\n";
  std::cout << "deltaP: " << pim.deltaPij().transpose() << "\n";
  std::cout << "deltaV: " << pim.deltaVij().transpose() << "\n";
  std::cout << "cov diag (first 15): " << cov.diagonal().head(15).transpose() << "\n";
  return 0;
}
