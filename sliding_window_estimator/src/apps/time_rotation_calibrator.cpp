#include <cstdlib>
#include <iostream>
#include <glog/logging.h>
#include "vio/derivedLinePatterns.h"

#include "swift_vio/TimeAndRotationCalibrator.h"

class PlainImuPattern : public vio::LinePattern {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PlainImuPattern() {}

  virtual ~PlainImuPattern() {}

  std::ostream &print(std::ostream &os) const override {
    char delim = ' ';
    os << sec_ << std::setw(9) << std::setfill('0') << nsec_ << delim
       << std::setprecision(8) << w_[0] << delim << w_[1] << delim << w_[2]
       << delim << a_[0] << delim << a_[1] << delim << a_[2] << delim;
    return os;
  }

  std::istream &read(std::istream &is) override {
    std::string time;
    getline(is, time, ' ');
    std::string trunk = time.substr(0, time.length() - 9);
    std::istringstream ss1(trunk);
    ss1 >> sec_;

    std::string residuals = time.substr(time.length() - 9);
    std::istringstream ss2(residuals);
    ss2 >> nsec_;
    char delim;
    is >> w_[0] >> w_[1] >> w_[2] >> a_[0] >> a_[1] >> a_[2];
    return is;
  }
public:
  uint32_t sec_;
  uint32_t nsec_;
  Eigen::Vector3d w_;
  Eigen::Vector3d a_;
};

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <tumTrajFile> <imuFile> [winsize=20]"
              << std::endl;
    return EXIT_FAILURE;
  }
  std::string tumTrajFile(argv[1]);
  std::string imuFile(argv[2]);
  double winsize = 20;
  if (argc > 3) {
    winsize = std::atof(argv[3]);
  }

  int imurate = 200;
  swift_vio::TimeAndRotationCalibrator imuCamCalibrator(winsize, imurate);

  Eigen::AlignedVector<vio::TumTrajPattern> traj;
  vio::loadCsv(tumTrajFile, traj);

  Eigen::AlignedVector<PlainImuPattern> imudata;
  vio::loadCsv(imuFile, imudata);

  std::vector<int> check_points;
  check_points.reserve(10);
  for (int i = 1; i <= 10; ++i) {
    check_points.push_back(i * imudata.size() / 10);
  }

  double samplingInterval = 1.0 / imurate;
  int numSuccessfulCalibration = 0;
  int nextCheckIndex = 0;

  auto poseit = traj.begin();
  int imucount = 0;
  int posecount = 0;
  for (const auto &imusample : imudata) {
    okvis::Time time(imusample.sec_, imusample.nsec_);
    Eigen::Vector3d gyro = imusample.w_; // the input file has angular imurate before linear acceleration.
    imuCamCalibrator.addImuAngularRate(time, gyro);
    if (imucount < 5) {
      std::cout << imucount << " " << time << " " << gyro.transpose() << " " << imusample.a_.transpose() << "\n";
    }
    okvis::Time ptime(poseit->sec_, poseit->nsec_);
    while (ptime < time) {
      imuCamCalibrator.addTargetOrientation(ptime, poseit->q_WS_);
      ++poseit;
      ptime = okvis::Time(poseit->sec_, poseit->nsec_);
      if (posecount < 5) {
        std::cout << ptime << " " << *poseit << "\n";
      }
      ++posecount;
    }
    ++imucount;
    if (imucount == check_points[nextCheckIndex]) {
      swift_vio::TimeAndRotationCalibrator::CalibrationStatus status =
          imuCamCalibrator.calibrate();
      ++nextCheckIndex;

      if (status == swift_vio::TimeAndRotationCalibrator::CalibrationStatus::Successful) {
        ++numSuccessfulCalibration;
        double timeOffset = imuCamCalibrator.relativeTimeOffset();
        std::cout << nextCheckIndex << " computed time offset of cam frames " << timeOffset << "\n";
        Eigen::Quaterniond q_IG = imuCamCalibrator.relativeOrientation();
        std::cout << "Computed imu_R_cam\n"
            << q_IG.toRotationMatrix() << "\n";
      }
    }
    if (imucount % imurate == 0) {
      int removed = imuCamCalibrator.slideWindow();
    }
  }

  LOG(INFO) << "#Successful calibrations " << numSuccessfulCalibration;
  return EXIT_SUCCESS;
}
