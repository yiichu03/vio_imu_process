#include <simul/SimulatorBase.hpp>

#include <random>

#include "okvis/IdProvider.hpp"

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/IoUtil.hpp>
#include <swift_vio/imu/BoundedImuDeque.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <swift_vio/PointLandmarkSimulation.hpp>
#include <swift_vio/ProjectionIntrinsicReps.h>

#ifdef HAVE_BSPLINES
#include <bsplines/BSplineFitter.hpp>
#endif
#include <gflags/gflags.h>

DEFINE_int32(sim_start_index, 0, "start index in real data");

DEFINE_double(sim_skip_first_seconds, 0, "skip first few seconds of the simulated data.");

DEFINE_double(r3_spline_smoothing_lambda, 0.1,
              "Smoothing the spline curve fitted to the translation. Larger "
              "value means more smooth curve.");

DEFINE_double(so3_spline_smoothing_lambda, 0.1,
              "Smoothing the spline curve fitted to the rotation. Larger value "
              "means more smooth curve.");

DEFINE_int32(spline_knot_delta, 5000000, "Spline knot delta in nanoseconds.");

namespace simul {
const double SimulatorBase::imageNoiseMag_ = 1.0;

const okvis::Duration SimulatorBase::halfImuBuffer_(0.3);

okvis::ImuSensorReadings interpolate(const okvis::ImuSensorReadings &left,
                                     const okvis::ImuSensorReadings &right,
                                     double ratio) {
  Eigen::Vector3d gyro =
      left.gyroscopes + (right.gyroscopes - left.gyroscopes) * ratio;
  Eigen::Vector3d accel = left.accelerometers +
                          (right.accelerometers - left.accelerometers) * ratio;
  return okvis::ImuSensorReadings(gyro, accel);
}

void loadImuYaml(const std::string &imuYaml,
                 okvis::ImuParameters *imuParameters) {
  cv::FileStorage file(imuYaml, cv::FileStorage::READ);
  OKVIS_ASSERT_TRUE(std::runtime_error, file.isOpened(),
                    "Could not open IMU config yaml: " << imuYaml);

  imuParameters->sigma_a_c = file["accelerometer_noise_density"];
  imuParameters->sigma_aw_c = file["accelerometer_random_walk"];
  imuParameters->sigma_g_c = file["gyroscope_noise_density"];
  imuParameters->sigma_gw_c = file["gyroscope_random_walk"];
  imuParameters->rate = file["update_rate"];
  if (file["initial_gyro_bias"].isSeq()) {
    Eigen::Vector3d g0;
    for (int i = 0; i < 3; ++i) {
      g0[i] = file["initial_gyro_bias"][i];
    }
    imuParameters->setInitialGyroBias(g0);
  }
  if (file["initial_accelerometer_bias"].isSeq()) {
    Eigen::Vector3d a0;
    for (int i = 0; i < 3; ++i) {
      a0[i] = file["initial_accelerometer_bias"][i];
    }
    imuParameters->setInitialAccelBias(a0);
  }

  const cv::FileNode &gravityNode = file["gravity_in_target"];
  if (gravityNode.isSeq()) {
    Eigen::Vector3d g_W;
    for (int i = 0; i < 3; ++i) {
      g_W[i] = gravityNode[i];
    }
    imuParameters->g = g_W.norm();
    imuParameters->setGravityDirection(g_W.normalized());
  }
  file.release();
}

void SimulatorBase::navStateAtStart(
    const swift_vio::InitialNavState &vioInitialStateAndCov,
    swift_vio::InitialNavState *initialStateAndCov) const {
  *initialStateAndCov = vioInitialStateAndCov;
  initialStateAndCov->p_WS = ref_T_WS_list_.front().r();
  initialStateAndCov->q_WS = ref_T_WS_list_.front().q();
  initialStateAndCov->v_WS = ref_v_WS_list_.front();
  initialStateAndCov->initializeToCustomPose = true;
}

void SimulatorBase::resetImuBiases(const okvis::ImuParameters &imuParameters,
                                      const std::string &imuLogFile) {
  if (addImuNoise_) {
    std::shared_ptr<std::ofstream> inertialStream;
    if (!imuLogFile.empty()) {
      inertialStream.reset(new std::ofstream(imuLogFile, std::ofstream::out));
      (*inertialStream)
          << "% timestamp, gx, gy, gz[rad/sec], acc x, acc y, acc "
             "z[m/s^2], gyro bias xyz, acc bias xyz, noisy gxyz, acc xyz"
          << std::endl;
    }
    simul::addNoiseToImuReadings(imuParameters, &imuMeasurements_, &refBiases_,
                                 inertialStream.get());
  } else {
    refBiases_ = originalBiases_;
  }
}

void SimulatorBase::saveRefMotion(const std::string &truthFile) {
  std::ofstream truthStream;
  truthStream.open(truthFile, std::ofstream::out);
  truthStream << "#state timestamp, T_WS(xyz, qxyzw), v_WS" << std::endl;

  if (!rewind()) {
    LOG(WARNING) << "Skip saving reference motion because of failed rewinding!";
    return;
  }

  do {
    truthStream << currentTime() << " " << std::setfill(' ')
                << currentPose().parameters().transpose().format(
                       swift_vio::kSpaceInitFmt)
                << " "
                << currentVelocity().transpose().format(
                       swift_vio::kSpaceInitFmt)
                << std::endl;
  } while (nextNFrame());
  truthStream.close();
}

void SimulatorBase::saveLandmarkGrid(const std::string &gridFile) const {
  return simul::saveLandmarkGrid(homogeneousPoints_, lmIds_, gridFile);
}

okvis::ImuSensorReadings SimulatorBase::currentBiases() const {
  okvis::Time timenow = currentTime();
  if (refBiasIter_->timeStamp < timenow) {
    if ((timenow - refBiasIter_->timeStamp).toSec() < 1e-5) {
      return refBiasIter_->measurement;
    }
    auto left = refBiasIter_;
    auto right = refBiasIter_;
    ++right;
    if (right == refBiases_.end()) {
      return left->measurement;
    }
    OKVIS_ASSERT_TRUE(
        std::runtime_error, refBiasIter_->timeStamp < right->timeStamp,
        "Current bias timestamp should be close to current time!");
    double ratio = (timenow - left->timeStamp).toSec() /
                   (right->timeStamp - left->timeStamp).toSec();
    return interpolate(left->measurement, right->measurement, ratio);
  } else {  // refBiasIter_->timeStamp >= timenow
    if ((refBiasIter_->timeStamp - timenow).toSec() < 1e-5) {
      return refBiasIter_->measurement;
    }
    auto left = refBiasIter_;
    auto right = refBiasIter_;
    if (left == refBiases_.begin()) {
      return left->measurement;
    }
    --left;
    OKVIS_ASSERT_TRUE(
        std::runtime_error, refBiasIter_->timeStamp > left->timeStamp,
        "Current bias timestamp should be close to current time!");
    double ratio = (timenow - left->timeStamp).toSec() /
                   (right->timeStamp - left->timeStamp).toSec();
    return interpolate(left->measurement, right->measurement, ratio);
  }
  return refBiasIter_->measurement;
}

size_t
SimulatorBase::generatePoints(const okvis::kinematics::Transformation &T_WS_ref,
                              const swift_vio::CameraRig &refCameraSystem,
                              size_t camId, size_t numNewPoints,
                              double maxDepth, double minDepth) {
  Eigen::AlignedVector<Eigen::Vector4d> newPoints;
  newPoints.reserve(numNewPoints);
  std::vector<uint64_t> newIds;
  newIds.reserve(numNewPoints);

  for (size_t i = 0; i < numNewPoints; ++i) {
    Eigen::Vector4d ptCam =
        refCameraSystem.cameraGeometry(camId)
            ->createRandomVisibleHomogeneousPoint(minDepth, maxDepth);
    uint64_t id = okvis::IdProvider::instance().newId();
    Eigen::Vector4d pt = T_WS_ref * (*refCameraSystem.T_SC(camId)) * ptCam;
    newPoints.push_back(pt);
    newIds.push_back(id);
  }
  homogeneousPoints_.insert(homogeneousPoints_.end(), newPoints.begin(),
                            newPoints.end());
  lmIds_.insert(lmIds_.end(), newIds.begin(), newIds.end());
  return newIds.size();
}

void saveCameraParameters(
    std::shared_ptr<swift_vio::CameraRig> cameraSystem,
    const std::string cameraFile) {
  std::ofstream stream(cameraFile);
  for (size_t i = 0u; i < cameraSystem->numCameras(); ++i) {
    Eigen::VectorXd allIntrinsics;
    cameraSystem->cameraGeometry(i)->getIntrinsics(allIntrinsics);

    std::vector<std::string> camProjectionIntrinsicLabels = {"f_x", "f_y", "c_x", "c_y"};
    std::vector<std::string> camDistortionLabels;
    swift_vio::DistortionTypeToDimensionLabels(
        cameraSystem->distortionType(i), &camDistortionLabels);
    stream << "#p_SC_x p_SC_y p_SC_z q_SC_x q_SC_y q_SC_z q_SC_w";
    for (const auto &v : camProjectionIntrinsicLabels) {
      stream << " " << v;
    }
    for (const auto &v : camDistortionLabels) {
      stream << " " << v;
    }
    stream << " t_d t_r\n";
    stream << cameraSystem->getCameraExtrinsic(i).parameters().transpose().format(
                  swift_vio::kSpaceInitFmt)
           << " " << allIntrinsics.transpose().format(swift_vio::kSpaceInitFmt)
           << " " << cameraSystem->cameraGeometry(i)->imageDelay() << " "
           << cameraSystem->cameraGeometry(i)->readoutTime() << std::endl;
  }
  stream.close();
}

SimFromRealData::SimFromRealData(const std::string &dataDir,
                                 const okvis::ImuParameters& imuParameters,
                                 bool addExtraLandmarks,
                                 bool addImageNoise, bool addImuNoise)
    : SimulatorBase(addExtraLandmarks, addImageNoise, addImuNoise) {
  vimap_.loadVimapFromFolder(dataDir);
  const auto& imuData = vimap_.imuData();

  for (const auto& entry : imuData) {
    okvis::Time time(entry.sec_, entry.nsec_);
    okvis::ImuSensorReadings measurement(entry.w_, entry.a_);
    imuMeasurements_.emplace_back(time, measurement);
  }

  const int64_t secToNanos = 1000000000;
  const auto& times = vimap_.vertexTimestamps();
  times_.reserve(times.size() - FLAGS_sim_start_index);

  int index = 0;
  for (const auto &time : times) {
    if (index >= FLAGS_sim_start_index) {
      times_.emplace_back(time / secToNanos, time % secToNanos);
    }
    ++index;
  }

  startTime_ = times_.front();
  finishTime_ = times_.back();

  if (startTime_ > imuMeasurements_.back().timeStamp ||
      finishTime_ < imuMeasurements_.front().timeStamp) {
    LOG(ERROR)
        << "Camera and IMU data have no overlap in time. \n"
        << "Images start at " << startTime_ << " finish at " << finishTime_ << ".\n"
        << "IMU starts at " << imuMeasurements_.front().timeStamp << " finishes at "
        << imuMeasurements_.back().timeStamp << "\n";
  }

  if (std::fabs(startTime_.toSec() - imuMeasurements_.front().timeStamp.toSec()) > 1 ||
      std::fabs(finishTime_.toSec() - imuMeasurements_.back().timeStamp.toSec()) > 1) {
    LOG(ERROR) << "IMU data barely cover the image sequence duration.\n"
                 << "Images start at " << startTime_ << " finish at " << finishTime_ << ".\n"
                 << "IMU starts at " << imuMeasurements_.front().timeStamp << " finishes at "
                 << imuMeasurements_.back().timeStamp << "\n";
  }
  {
    // hack: pad the beginning and end with IMU data in order to cover the visual measurement duration.
    okvis::Duration delta(1.0 / imuParameters.rate);
    okvis::Time leftTimeLimit = startTime_ - okvis::Duration(0.5);
    swift_vio::padImuToLeft(leftTimeLimit, delta, &imuMeasurements_);
    okvis::Time rightTimeLimit = finishTime_ + okvis::Duration(0.5);
    swift_vio::padImuToRight(rightTimeLimit, delta, &imuMeasurements_);
  }

  const auto &vertices = vimap_.vertices();
  ref_T_WS_list_.reserve(vertices.size());
  ref_v_WS_list_.reserve(vertices.size());
  originalBiases_.clear();
  index = 0;
  for (const auto &vertex : vertices) {
    if (index >= FLAGS_sim_start_index) {
      ref_T_WS_list_.emplace_back(vertex.p_WS_, vertex.q_WS_);
      ref_v_WS_list_.emplace_back(vertex.v_WS_);
      originalBiases_.emplace_back(
          okvis::Time(vertex.time_ns / secToNanos, vertex.time_ns % secToNanos),
          okvis::ImuSensorReadings(vertex.gyroBias_, vertex.accelBias_));
    }
    ++index;
  }
  refBiases_ = originalBiases_;
}

void SimFromRealData::initializeLandmarkGrid(LandmarkGridType /*landmarkDistribution*/,
                                             double /*landmarkCylinderRadius*/,
                                             double,
                                             const swift_vio::CameraRig *) {
  homogeneousPoints_.reserve(vimap_.landmarks().size());
  lmIds_.reserve(vimap_.landmarks().size());
  const auto &landmarks = vimap_.landmarks();
  for (const auto &lmk : landmarks) {
    Eigen::Vector4d hPoint;
    hPoint << lmk.position, 1.0;
    homogeneousPoints_.push_back(T_newW_oldW_ * hPoint);
    lmIds_.push_back(okvis::IdProvider::instance().newId());
  }
}

bool SimFromRealData::nextNFrame() {
  ++refIndex_;
  if (refIndex_ == ref_T_WS_list_.size()) {
    return false;
  }
  ++refTimeIter_;

  while (refBiasIter_ != refBiases_.end() && refBiasIter_->timeStamp < *refTimeIter_) {
    ++refBiasIter_;
  }
  if (refBiasIter_ == refBiases_.end()) {
    return false;
  }

  okvis::Time imuDataEndTime = currentTime() + halfImuBuffer_;
  okvis::Time imuDataBeginTime = lastRefNFrameTime_ - halfImuBuffer_;
  latestImuMeasurements_ = swift_vio::getImuMeasurements(
      imuDataBeginTime, imuDataEndTime, imuMeasurements_, nullptr);
  lastRefNFrameTime_ = currentTime();
  return true;
}

void SimFromRealData::addFeaturesToNFrame(
    const swift_vio::CameraRig &refCameraSystem,
    std::shared_ptr<swift_vio::MultiFrame> multiFrame,
    std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks)
    const {
  addFeaturesToNFrameImpl(refCameraSystem, multiFrame,
                          keypointIndexForLandmarks);
}

void SimFromRealData::addFeaturesToNFrame(
    const swift_vio::CameraRig &refCameraSystem,
    std::shared_ptr<okvis::MultiFrame> multiFrame,
    std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks)
    const {
  addFeaturesToNFrameImpl(refCameraSystem, multiFrame,
                          keypointIndexForLandmarks);
}

bool SimFromRealData::rewind() {
  if (refBiases_.size() < 1) {
    LOG(WARNING) << "Call resetImuBiases to init IMU biases before going through the sim data!";
    return false;
  }

  refIndex_ = 0u;
  refTimeIter_ = times_.begin();
  latestImuMeasurements_.clear();
  lastRefNFrameTime_ = currentTime();

  refBiasIter_ = refBiases_.begin();
  while (refBiasIter_->timeStamp < *refTimeIter_) {
    ++refBiasIter_;
  }

  okvis::Time imuDataEndTime = currentTime() + halfImuBuffer_;
  okvis::Time imuDataBeginTime = lastRefNFrameTime_ - halfImuBuffer_;
  latestImuMeasurements_ = swift_vio::getImuMeasurements(
      imuDataBeginTime, imuDataEndTime, imuMeasurements_, nullptr);
  return true;
}

int SimFromRealData::expectedNumNFrames() const {
  return vimap_.vertices().size();
}

#ifdef HAVE_BSPLINES
SimFromSplineData::SimFromSplineData(const std::string &tumTrajFile,
                                     const std::string &lmkCsv,
                                     const okvis::ImuParameters &imuParameters,
                                     bool addExtraLandmarks,
                                     bool addImageNoise, bool addImuNoise) :
  SimulatorBase(addExtraLandmarks, addImageNoise, addImuNoise),
  translation_(new bsplines::EuclideanBSpline<kSplineOrder, 3, bsplines::NsecTimePolicy>()),
  rotation_(new bsplines::SO3BSpline<kSplineOrder, bsplines::NsecTimePolicy>()),
  dtCamera(1.0 / kCameraRate),
  dtImu(1.0 / kImuRate), landmarkCsv(lmkCsv) {
  if (!lmkCsv.empty()) {
    loadLandmarks(lmkCsv);
    if (homogeneousPoints_.size() == 0u)
      LOG(WARNING) << "Failed to load any landmark from '" << landmarkCsv << "'!";
  }

  Eigen::AlignedVector<vio::TumTrajPattern> traj;
//  Eigen::AlignedVector<vio::MaplabVertexPattern> traj;
  vio::loadCsv(tumTrajFile, traj, 1);

  int64_t knotDelta = FLAGS_spline_knot_delta;

  std::vector<int64_t> times;
  times.reserve(traj.size());
  Eigen::AlignedVector<Eigen::Vector4d> quaternions;
  quaternions.reserve(traj.size());
  Eigen::AlignedVector<Eigen::Vector3d> positions;
  positions.reserve(traj.size());
  for (const auto &a : traj) {
//    times.push_back(a.time_ns);
    times.push_back(((int64_t)a.sec_) * 1000000000 + a.nsec_);
    quaternions.push_back(a.q_WS_.coeffs().normalized());
    positions.push_back(a.p_WS_);
  }

  bsplines::BSplineFitter<
      bsplines::EuclideanBSpline<kSplineOrder, 3, bsplines::NsecTimePolicy>>::
      initUniformSplineWithKnotDelta(*translation_, times, positions, knotDelta,
                                     FLAGS_r3_spline_smoothing_lambda);

  bsplines::BSplineFitter<
      bsplines::SO3BSpline<kSplineOrder, bsplines::NsecTimePolicy>>::
      initUniformSplineWithKnotDelta(*rotation_, times, quaternions, knotDelta,
                                     FLAGS_so3_spline_smoothing_lambda);

  okvis::Duration buffer(1.0);
  startTime_.fromNSec(translation_->getMinTime());
  startTime_ += (buffer + okvis::Duration(FLAGS_sim_skip_first_seconds));
  finishTime_.fromNSec(translation_->getMaxTime());
  finishTime_ -= buffer;

  int numNFrames = (finishTime_ - startTime_).toSec() * kCameraRate;
  times_.reserve(numNFrames + 1);
  ref_T_WS_list_.reserve(numNFrames + 1);
  ref_v_WS_list_.reserve(numNFrames + 1);
  for (okvis::Time t = startTime_; t < finishTime_; t += dtCamera) {
    times_.push_back(t);
    bsplines::SO3BSpline<kSplineOrder, bsplines::NsecTimePolicy>::Evaluator<0> eval =
        rotation_->getEvaluatorAt<0>(t.toNSec());
    Eigen::Quaterniond q_WB(eval.eval().data());
    bsplines::EuclideanBSpline<kSplineOrder, 3, bsplines::NsecTimePolicy>::Evaluator<1> eval2 =
        translation_->getEvaluatorAt<1>(t.toNSec());
    Eigen::Vector3d dotp = eval2.evalD(1);
    Eigen::Vector3d p = eval2.eval();
    ref_T_WS_list_.emplace_back(p, q_WB);
    ref_v_WS_list_.push_back(dotp);
  }

  for (okvis::Time t = startTime_ - buffer; t < finishTime_ + buffer;
       t += dtImu) {
    bsplines::SO3BSpline<kSplineOrder, bsplines::NsecTimePolicy>::Evaluator<1> eval =
        rotation_->getEvaluatorAt<1>(t.toNSec());
    Eigen::Quaterniond q_WB(eval.eval().data());
    Eigen::Vector3d gyroscopes = eval.evalAngularVelocityLocal();
    bsplines::EuclideanBSpline<kSplineOrder, 3, bsplines::NsecTimePolicy>::Evaluator<2> eval2 =
        translation_->getEvaluatorAt<2>(t.toNSec());
    Eigen::Vector3d ddotp = eval2.evalD(2);
    Eigen::Vector3d accelerometers =
        q_WB.conjugate() * (ddotp - imuParameters.gravity());
    okvis::ImuSensorReadings reading(gyroscopes, accelerometers);
    okvis::ImuMeasurement meas(t, reading);
    imuMeasurements_.push_back(meas);
  }

  originalBiases_ = imuMeasurements_;
  for (size_t i = 0; i < imuMeasurements_.size(); ++i) {
    originalBiases_[i].measurement.gyroscopes.setZero();
    originalBiases_[i].measurement.accelerometers.setZero();
  }
  refBiases_ = originalBiases_;
}

void SimFromSplineData::initializeLandmarkGrid(
    LandmarkGridType /*landmarkDistribution*/, double maxDepth, double minDepth,
    const swift_vio::CameraRig *refCameraSystem) {
  size_t numLoadedLandmarks = homogeneousPoints_.size();
  size_t numAdditionalLandmarks = 0u;
  if (addLandmarksForEnoughObservations_) {
    LOG(INFO) << "Generating additional random landmarks to make frames "
                 "observe enough landmarks...";
    okvis::Duration dt(5.0 / kCameraRate);
    for (okvis::Time t = startTime_; t < finishTime_; t += dt) {
      std::shared_ptr<swift_vio::MultiFrame> mf(
          new swift_vio::MultiFrame(refCameraSystem->numCameras(), t, 0));
      for (size_t j = 0u; j < refCameraSystem->numCameras(); ++j) {
        mf->setTimestamp(j, t);
      }

      std::vector<std::unordered_map<size_t, size_t>> kpIdForLandmarks; 
      okvis::Time trueCentralRowEpoch = t;
      typename bsplines::SO3BSpline<kSplineOrder, bsplines::NsecTimePolicy>::template Evaluator<0> rotEval =
          rotation_->template getEvaluatorAt<0>(trueCentralRowEpoch.toNSec());
      Eigen::Vector4d q_WS_raw = rotEval.eval();
      typename bsplines::EuclideanBSpline<kSplineOrder, 3, bsplines::NsecTimePolicy>::template Evaluator<0> transEval =
          translation_->template getEvaluatorAt<0>(trueCentralRowEpoch.toNSec());
      Eigen::Vector3d t_WS_ref = transEval.eval();
      Eigen::Quaterniond q_WS_ref(q_WS_raw);
      okvis::kinematics::Transformation T_WS_ref(t_WS_ref, q_WS_ref);
      // use the global shutter model for efficiency.
      PointLandmarkSimulation::projectLandmarksToNFrame(
          homogeneousPoints_, T_WS_ref, *refCameraSystem, mf,
          nullptr, &kpIdForLandmarks, nullptr);

      size_t camId = 0u;
      for (const auto &pts : kpIdForLandmarks) {
        if (pts.size() < kNumPointInFrame) {
          numAdditionalLandmarks +=
              generatePoints(T_WS_ref, *refCameraSystem, camId,
                             kNumPointInFrame - pts.size(), maxDepth, minDepth);
        }
        VLOG(2) << "Projected " << pts.size() << " into camera " << camId
                << " of frame at " << t << ".";
        ++camId;
        break; // only use the left camera.
      }
    }
  }
  LOG(INFO) << "#Loaded landmarks " << numLoadedLandmarks
            << ", #Additional artificial landmarks " << numAdditionalLandmarks
            << ".";
}

size_t SimFromSplineData::loadLandmarks(const std::string& lmkCsv) {
  std::vector<vio::MaplabLandmarkPattern,
              Eigen::aligned_allocator<vio::MaplabLandmarkPattern>>
      landmarks;
  vio::loadCsvData(lmkCsv, landmarks, 1);

  size_t numPoints = landmarks.size();
  homogeneousPoints_.reserve(numPoints);
  lmIds_.reserve(numPoints);

  for (size_t i = 0; i < numPoints; ++i) {
      Eigen::Vector4d pt;
      pt << landmarks.at(i).position, 1.0;
      uint64_t id = okvis::IdProvider::instance().newId();
      homogeneousPoints_.push_back(pt);
      lmIds_.push_back(id);
  }
  return numPoints;
}

bool SimFromSplineData::rewind() {
  if (refBiases_.size() < 1) {
    LOG(WARNING) << "Call resetImuBiases() to initialize IMU biases before going through the sim data!";
    return false;
  }

  refIndex_ = 0u;
  refTimeIter_ = times_.begin();
  latestImuMeasurements_.clear();
  lastRefNFrameTime_ = currentTime();

  refBiasIter_ = refBiases_.begin();
  while (refBiasIter_->timeStamp < *refTimeIter_) {
    ++refBiasIter_;
  }

  okvis::Time imuDataEndTime = currentTime() + halfImuBuffer_;
  okvis::Time imuDataBeginTime = lastRefNFrameTime_ - halfImuBuffer_;
  latestImuMeasurements_ = swift_vio::getImuMeasurements(
      imuDataBeginTime, imuDataEndTime, imuMeasurements_, nullptr);
  return true;
}

bool SimFromSplineData::nextNFrame() {
  ++refIndex_;
  if (refIndex_ == ref_T_WS_list_.size()) {
    return false;
  }
  ++refTimeIter_;

  while (refBiasIter_ != refBiases_.end() && refBiasIter_->timeStamp < *refTimeIter_) {
    ++refBiasIter_;
  }
  if (refBiasIter_ == refBiases_.end()) {
    return false;
  }

  okvis::Time imuDataEndTime = currentTime() + halfImuBuffer_;
  okvis::Time imuDataBeginTime = lastRefNFrameTime_ - halfImuBuffer_;
  latestImuMeasurements_ = swift_vio::getImuMeasurements(
      imuDataBeginTime, imuDataEndTime, imuMeasurements_, nullptr);
  lastRefNFrameTime_ = currentTime();
  return true;
}

void SimFromSplineData::addFeaturesToNFrame(
    const swift_vio::CameraRig &refCameraSystem,
    std::shared_ptr<swift_vio::MultiFrame> multiFrame,
    std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks)
    const {
  addFeaturesToNFrameImpl(refCameraSystem, multiFrame,
                          keypointIndexForLandmarks);
}

void SimFromSplineData::addFeaturesToNFrame(
    const swift_vio::CameraRig &refCameraSystem,
    std::shared_ptr<okvis::MultiFrame> multiFrame,
    std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks)
    const {
  addFeaturesToNFrameImpl(refCameraSystem, multiFrame,
                          keypointIndexForLandmarks);
}

int SimFromSplineData::expectedNumNFrames() const {
  return (int)((finishTime_ - startTime_).toSec() * kCameraRate);
}

#endif

CurveData::CurveData(const SimParameters &simParams,
                     const okvis::ImuParameters& imuParameters,
                     double duration, bool addExtraLandmarks,
                     bool addImageNoise, bool addImuNoise) :
  SimulatorBase(addExtraLandmarks, addImageNoise, addImuNoise) {
  startTime_ = okvis::Time(100);
  finishTime_ =  startTime_ + okvis::Duration(duration);

  trajectory_ = simul::createSimulatedTrajectory(
      simParams.trajectoryId, imuParameters.rate, imuParameters.gravity(),
      startTime_, simParams.slopeAngleRad);

  trajectory_->getSampleTimes(startTime_, finishTime_, times_);
  trajectory_->getTruePoses(times_, ref_T_WS_list_);
  trajectory_->getTrueVelocities(times_, ref_v_WS_list_);
  trajectory_->getTrueInertialMeasurements(
      startTime_ - okvis::Duration(1), finishTime_ + okvis::Duration(1), imuMeasurements_);
  originalBiases_ = imuMeasurements_;
  for (size_t i = 0; i < imuMeasurements_.size(); ++i) {
    originalBiases_[i].measurement.gyroscopes.setZero();
    originalBiases_[i].measurement.accelerometers.setZero();
  }
  refBiases_ = originalBiases_;
}

void CurveData::initializeLandmarkGrid(LandmarkGridType landmarkDistribution,
                                       double landmarkCylinderRadius,
                                       double minDepth,
                                       const swift_vio::CameraRig *refCameraSystem) {
  double halfz = 1.5;
  bool addFloorCeiling = false;
  switch (landmarkDistribution) {
  case LandmarkGridType::FourWalls:
    createBoxLandmarkGrid(&homogeneousPoints_, &lmIds_, halfz, addFloorCeiling);
    break;
  case LandmarkGridType::FourWallsFloorCeiling:
    halfz = 2.5;
    addFloorCeiling = true;
    createBoxLandmarkGrid(&homogeneousPoints_, &lmIds_, halfz, addFloorCeiling);
    break;
  case LandmarkGridType::Cylinder:
    createCylinderLandmarkGrid(&homogeneousPoints_, &lmIds_, landmarkCylinderRadius);
    break;
  case LandmarkGridType::Random:
      {
        const int kCameraRate = 20;
        const size_t kNumPointInFrame = 50;
        double maxDepth = landmarkCylinderRadius;
        size_t numAdditionalLandmarks = 0u;
        if (addLandmarksForEnoughObservations_) {
          LOG(INFO) << "Generating additional random landmarks to make frames "
                       "observe enough landmarks...";
          okvis::Duration dt(5.0 / kCameraRate);
          for (okvis::Time t = startTime_; t < finishTime_; t += dt) {
            std::shared_ptr<swift_vio::MultiFrame> mf(
                new swift_vio::MultiFrame(refCameraSystem->numCameras(), t, 0));

            for (size_t j = 0u; j < refCameraSystem->numCameras(); ++j) {
              mf->setTimestamp(j, t);
            }

            std::vector<std::unordered_map<size_t, size_t>> kpIdForLandmarks;
            okvis::kinematics::Transformation T_WS_ref =
                trajectory_->computeGlobalPose(t);
            PointLandmarkSimulation::projectLandmarksToNFrame(
                homogeneousPoints_, T_WS_ref,
                *refCameraSystem, mf, nullptr, &kpIdForLandmarks,
                nullptr);

            size_t camId = 0u;
            for (const auto &pts : kpIdForLandmarks) {
              if (pts.size() < kNumPointInFrame) {
                numAdditionalLandmarks +=
                    generatePoints(T_WS_ref, *refCameraSystem, camId,
                                   kNumPointInFrame - pts.size(), maxDepth, minDepth);
              }
              VLOG(2) << "Projected " << pts.size() << " into camera " << camId
                      << " of frame at " << t << ".";
              ++camId;
              break; // only use the left camera.
            }
          }
        }
        LOG(INFO) << "#Artificial landmarks " << numAdditionalLandmarks << ".";
      }
      break;
  default:
    LOG(ERROR) << "Unknown landmark distribution " << landmarkDistribution << ".";
    break;
  }
}

bool CurveData::rewind() {
  if (refBiases_.size() < 1) {
    LOG(WARNING) << "Call resetImuBiases to init IMU biases before going through the sim data!";
    return false;
  }

  refIndex_ = 0u;
  refTimeIter_ = times_.begin();

  refBiasIter_ = refBiases_.begin();
  while (refBiasIter_->timeStamp < *refTimeIter_ &&
         (*refTimeIter_ - refBiasIter_->timeStamp).toSec() > 1e-3) {
    ++refBiasIter_;
  }
  OKVIS_ASSERT_LT(std::runtime_error,
                  (refBiasIter_->timeStamp - *refTimeIter_).toSec(), 1e-3,
                  "IMU data from curves are not perfectly synced with poses!");

  latestImuMeasurements_.clear();
  lastRefNFrameTime_ = currentTime();

  okvis::Time imuDataEndTime = currentTime() + halfImuBuffer_;
  okvis::Time imuDataBeginTime = lastRefNFrameTime_ - halfImuBuffer_;
  latestImuMeasurements_ = swift_vio::getImuMeasurements(
      imuDataBeginTime, imuDataEndTime, imuMeasurements_, nullptr);
  return true;
}

bool CurveData::nextNFrame() {
  if (refIndex_ + kCameraIntervalRatio >= ref_T_WS_list_.size()) {
    return false;
  }

  refTimeIter_ += kCameraIntervalRatio;
  refIndex_ += kCameraIntervalRatio;
  refBiasIter_ += kCameraIntervalRatio;

  okvis::Time imuDataEndTime = currentTime() + halfImuBuffer_;
  okvis::Time imuDataBeginTime = lastRefNFrameTime_ - halfImuBuffer_;
  latestImuMeasurements_ = swift_vio::getImuMeasurements(
      imuDataBeginTime, imuDataEndTime, imuMeasurements_, nullptr);

  lastRefNFrameTime_ = currentTime();
  return true;
}

void CurveData::addFeaturesToNFrame(
    const swift_vio::CameraRig &refCameraSystem,
    std::shared_ptr<swift_vio::MultiFrame> multiFrame,
    std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const {
  addFeaturesToNFrameImpl(refCameraSystem, multiFrame, keypointIndexForLandmarks);
}

void CurveData::addFeaturesToNFrame(
    const swift_vio::CameraRig &refCameraSystem,
    std::shared_ptr<okvis::MultiFrame> multiFrame,
    std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const {
  addFeaturesToNFrameImpl(refCameraSystem, multiFrame, keypointIndexForLandmarks);
}

} // namespace simul
