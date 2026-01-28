#ifndef INCLUDE_SWIFT_VIO_SIM_DATA_INTERFACE_HPP_
#define INCLUDE_SWIFT_VIO_SIM_DATA_INTERFACE_HPP_

#include <simul/ImuNoiseSimulator.h>
#include <simul/SimParameters.h>

#include <simul/curves.h>
#include <simul/PointLandmarkSimulationRS.hpp>

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <swift_vio/imu/BoundedImuDeque.hpp>

#include <vio/VimapContainer.h>
#ifdef HAVE_BSPLINES
#include <bsplines/SO3BSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>
#endif

DECLARE_int32(sim_start_index);

namespace simul {
okvis::ImuSensorReadings interpolate(const okvis::ImuSensorReadings &left,
                                     const okvis::ImuSensorReadings &right,
                                     double ratio);

class SimulatorBase {
 protected:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const double imageNoiseMag_; // pixel unit
  static const okvis::Duration halfImuBuffer_;

  bool addLandmarksForEnoughObservations_; ///< Generate additional landmarks to ensure enough observations in frames.
  bool addImageNoise_;                ///< Add noise to image observations
  bool addImuNoise_;

  okvis::Time startTime_;
  okvis::Time finishTime_;

  // reference variables of the same length used for computing RMSEs.
  std::vector<okvis::Time> times_;
  Eigen::AlignedVector<okvis::kinematics::Transformation> ref_T_WS_list_;
  Eigen::AlignedVector<Eigen::Vector3d> ref_v_WS_list_;

  okvis::ImuMeasurementDeque originalBiases_; // biases for the original data.

  okvis::ImuMeasurementDeque refBiases_; // newly simulated biases.

  // imu meas. covering at least [start, finish].
  okvis::ImuMeasurementDeque imuMeasurements_;

  // landmarks of coordinates in a world frame.
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      homogeneousPoints_;
  // Warn: To avoid conflicits with ids used in the backend estimator,
  // okvis::IdProvider should be used for generating Ids.
  std::vector<uint64_t> lmIds_;

  // variables for iterations
  std::vector<okvis::Time>::const_iterator refTimeIter_;
  size_t refIndex_;
  okvis::ImuMeasurementDeque::const_iterator refBiasIter_;

  okvis::ImuMeasurementDeque latestImuMeasurements_;
  okvis::Time lastRefNFrameTime_;

public:
  SimulatorBase(bool addExtraLandmarks,
                bool addImageNoise, bool addImuNoise)
      : addLandmarksForEnoughObservations_(addExtraLandmarks),
        addImageNoise_(addImageNoise), addImuNoise_(addImuNoise) {}

  virtual ~SimulatorBase() {}

  /**
   * @brief rewind prepare for iteration at the start. Call this after resetImuBiases.
   * @return whether rewind is successful.
   */
  virtual bool rewind() = 0;

  virtual bool nextNFrame() = 0;

  /**
   * @brief addFeaturesToNFrame
   * @param[in] refCameraSystem
   * @param[out] multiFrame will be assigned keypoints for individual frames.
   * @param[out] keypointIndexForLandmarks keypoint indices for landmarks in frames.
   */
  virtual void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<swift_vio::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const = 0;

  virtual void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<okvis::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const = 0;

  virtual int expectedNumNFrames() const = 0;

  /**
   * @brief navStateAtStart
   * @param vioInitialStateAndCov
   * @param initialStateAndCov Because the estimators use a world frame with z along negative gravity,
   * the initialStateAndCov needs to respect this.
   * @param simInitialSpeedStd
   */
  virtual void
  navStateAtStart(const swift_vio::InitialNavState &vioInitialStateAndCov,
                  swift_vio::InitialNavState *initialStateAndCov) const;

  virtual void initializeLandmarkGrid(LandmarkGridType landmarkDistribution, double landmarkCylinderRadius, double minDepth = 1.5,
                                      const swift_vio::CameraRig *refCameraSystem = nullptr) = 0;

  /**
   * @brief reset and simulate IMU biases again.
   */
  virtual void resetImuBiases(const okvis::ImuParameters &imuParameters,
                              const std::string &imuLogFile);

  size_t generatePoints(const okvis::kinematics::Transformation &T_WS_ref,
                        const swift_vio::CameraRig &refCameraSystem,
                        size_t camId, size_t numNewPoints, double maxDepth,
                        double minDepth);

  void saveRefMotion(const std::string &truthFile);

  void saveLandmarkGrid(const std::string &gridFile) const;

  okvis::Time currentTime() const {
    return *refTimeIter_;
  }

  okvis::kinematics::Transformation currentPose() const {
    return ref_T_WS_list_[refIndex_];
  }

  Eigen::Vector3d currentVelocity() const {
    return ref_v_WS_list_[refIndex_];
  }

  okvis::ImuSensorReadings currentBiases() const;

  /**
   * @brief imuMeasurementsSinceLastNFrame
   * @return IMU measurement since the last NFrame's time - delta until current NFrame's time + delta.
   * So it has overlap with previous IMU segment.
   */
  okvis::ImuMeasurementDeque imuMeasurementsSinceLastNFrame() const {
    return latestImuMeasurements_;
  }

  okvis::ImuMeasurementDeque getImuMeasurements(okvis::Time imuDataBeginTime,
                                                okvis::Time imuDataEndTime) const {
    return swift_vio::getImuMeasurements(imuDataBeginTime, imuDataEndTime,
                                         imuMeasurements_, nullptr);
  }

  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      &homogeneousPoints() const {
    return homogeneousPoints_;
  }

  const std::vector<uint64_t> &landmarkIds() const { return lmIds_; }

  Eigen::Vector4d landmarkInWorld(uint64_t lmId) const {
    std::vector<uint64_t>::const_iterator it = std::find(lmIds_.begin(), lmIds_.end(), lmId);
    if (it == lmIds_.end()) {
      return Eigen::Vector4d(0, 0, 0, 0);
    } else {
      return homogeneousPoints_[std::distance(lmIds_.begin(), it)];
    }
  }
};

/**
 * @brief Load and play VIO discrete time trajectory, landmarks, and observations, in maplab csv format.
 * This class may add noise to the observations and IMU data.
 */
class SimFromRealData : public SimulatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief SimFromRealData
   * @param dataDir
   * @param imuParameters
   * @param addExtraLandmarks add extra random landmarks to ensure enough observations.
   * @param addImageNoise
   * @param addImuNoise
   */
  SimFromRealData(const std::string &dataDir,
                  const okvis::ImuParameters &imuParameters,
                  bool addExtraLandmarks, bool addImageNoise, bool addImuNoise);

  virtual ~SimFromRealData() {}

  void initializeLandmarkGrid(LandmarkGridType /*landmarkDistribution*/, double /*maxDepth*/, double minDepth,
                              const swift_vio::CameraRig *refCameraSystem) final;

  bool rewind() final;

  bool nextNFrame() final;

  void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<swift_vio::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const final;

  void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<okvis::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const final;

  template <typename MultiFrameT>
  void addFeaturesToNFrameImpl(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<MultiFrameT> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const {
    multiFrame->createTestImages(refCameraSystem.cameraGeometry(0)->imageHeight(),
                                 refCameraSystem.cameraGeometry(0)->imageWidth());
    int numCameras = multiFrame->numFrames();
    keypointIndexForLandmarks->resize(numCameras);

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, imageNoiseMag_};

    const std::vector<vio::CornersInImage,
                      Eigen::aligned_allocator<vio::CornersInImage>>
        &keypointsInAllFrames = vimap_.validKeypoints();
    for (size_t camId = 0u; camId < vimap_.numberCameras(); ++camId) {
      const vio::CornersInImage &corners =
          keypointsInAllFrames.at((refIndex_ + FLAGS_sim_start_index) * vimap_.numberCameras() + camId);
      std::vector<cv::KeyPoint> keypoints;
      keypoints.reserve(corners.corner_ids.size());
      for (size_t i = 0u; i < corners.corner_ids.size(); ++i) {
        const Eigen::Vector2d &uv = corners.corners.at(i);
        int cornerid = corners.corner_ids.at(i);
        const auto iter = keypointIndexForLandmarks->at(corners.cam_id).find(cornerid);
        if (iter == keypointIndexForLandmarks->at(corners.cam_id).end()) {
          keypointIndexForLandmarks->at(corners.cam_id)[cornerid] = i;
        } else {
          LOG(WARNING) << "Two keypoints " << iter->second << " and " << i
                       << " for one corner " << cornerid << "!";
        }

        if (addImageNoise_) {
          keypoints.emplace_back(uv[0] + d(gen), uv[1] + d(gen),
                                 corners.radii[i]);
        } else {
          keypoints.emplace_back(uv[0], uv[1], corners.radii[i]);
        }
      }
      multiFrame->resetKeypoints(camId, keypoints);
    }

    // check
    for (size_t camId = 0u; camId < keypointIndexForLandmarks->size(); ++camId) {
      const auto &keypointIndices = keypointIndexForLandmarks->at(camId);
      for (auto l2k : keypointIndices) {
        int landmarkId = (int)l2k.first;
        size_t kpi = l2k.second;
        OKVIS_ASSERT_EQ(std::runtime_error, landmarkId,
                        keypointsInAllFrames[(refIndex_ + FLAGS_sim_start_index) *
                                                 vimap_.numberCameras() +
                                             camId]
                            .corner_ids[kpi],
                        "Wrong landmark ID association!");
        Eigen::Vector2d keypoint;
        multiFrame->getKeypoint(camId, kpi, keypoint);
        Eigen::Vector2d projection(100, 100);
        Eigen::Vector4d hpC =
            (ref_T_WS_list_[refIndex_] * refCameraSystem.getCameraExtrinsic(camId))
                .inverse() *
            homogeneousPoints_[landmarkId];
        auto status = refCameraSystem.cameraGeometry(camId)->projectHomogeneous(
            hpC, &projection);
        if (status != okvis::cameras::CameraBase::ProjectionStatus::Successful ||
            (keypoint - projection).norm() > 30) {
          VLOG(2) << "keypoint " << keypoint.transpose() << " reprojection "
                  << projection.transpose() << " status " << (int)status;
        }
      }
    }
  }

  int expectedNumNFrames() const final;

private:
  okvis::kinematics::Transformation
      T_newW_oldW_; // To transform entities in the world frame used by the real
                    // data to a new world frame with z along negative gravity.

  vio::VimapContainer vimap_;
};

/**
 * @brief Given a specified continuous-time curve, and predefined landmark grid,
 * simulate IMU data, and landmark observations.
 */
class CurveData : public SimulatorBase {
private:
  static const int kCameraIntervalRatio = 10; // #imu meas. for 1 camera nframe.
  std::shared_ptr<TrajectoryBase> trajectory_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief CurveData
   * @param trajectoryType
   * @param imuParameters
   * @param duration
   * @param addExtraLandmarks add extra random landmarks to ensure enough observations.
   * @param addImageNoise
   * @param addImuNoise
   */
  CurveData(const SimParameters &simParams,
            const okvis::ImuParameters &imuParameters,
            double duration, bool addExtraLandmarks,
            bool addImageNoise, bool addImuNoise);

  virtual ~CurveData() {}

  void initializeLandmarkGrid(LandmarkGridType landmarkDistribution, double landmarkCylinderRadius, double minDepth,
                              const swift_vio::CameraRig *refCameraSystem) final;

  bool rewind() final;

  bool nextNFrame() final;

  void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<swift_vio::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const final;

  void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<okvis::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const final;

  template<typename MultiFrameT>
  void addFeaturesToNFrameImpl(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<MultiFrameT> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const {
    multiFrame->createTestImages(refCameraSystem.cameraGeometry(0)->imageHeight(),
                                 refCameraSystem.cameraGeometry(0)->imageWidth());
    PointLandmarkSimulationRS::projectLandmarksToNFrame(
        homogeneousPoints_, trajectory_, currentTime(),
        refCameraSystem, multiFrame, nullptr, keypointIndexForLandmarks,
        addImageNoise_ ? &imageNoiseMag_ : nullptr);
  }

  int expectedNumNFrames() const final {
    return times_.size() / kCameraIntervalRatio + 1;
  }
};

#ifdef HAVE_BSPLINES
/**
 * @brief Given a discrete-time trajectory in TUM format, fit a spline to the trajectory,
 * then either simulate the landmarks so that there are enough observations in each frame or load predefined landmarks,
 * at last generate the landmark observations.
 */
class SimFromSplineData : public SimulatorBase {
public:
  const int kCameraRate = 20;
  const int kImuRate = 200;
  const size_t kNumPointInFrame = 50;
  static const int kSplineOrder = 5;

private:
  std::shared_ptr<bsplines::EuclideanBSpline<kSplineOrder, 3, bsplines::NsecTimePolicy>> translation_;
  std::shared_ptr<bsplines::SO3BSpline<kSplineOrder, bsplines::NsecTimePolicy>> rotation_;

  okvis::Duration dtCamera;
  okvis::Duration dtImu;
  std::string landmarkCsv;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief SimFromSplineData
   * @param tumTrajFile traj file in TUM format.
   * @param landmarkCsv landmark file has a header as the first line. Following lines have the format [id,x,y,z].
   * The coordinate frame for x,y,z is the same as the world frame of the trajectory.
   * @param imuParameters
   * @param addExtraLandmarks add extra random landmarks to ensure enough observations.
   * @param addImageNoise
   * @param addImuNoise
   */
  SimFromSplineData(const std::string &tumTrajFile,
                    const std::string &landmarkCsv,
                    const okvis::ImuParameters &imuParameters,
                    bool addExtraLandmarks,
                    bool addImageNoise, bool addImuNoise);

  virtual ~SimFromSplineData() {}

  void initializeLandmarkGrid(LandmarkGridType landmarkDistribution, double maxDepth, double minDepth,
                              const swift_vio::CameraRig *refCameraSystem) final;

  bool rewind() final;

  bool nextNFrame() final;

  void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<swift_vio::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const final;

  void addFeaturesToNFrame(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<okvis::MultiFrame> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const final;

  template <typename MultiFrameT>
  void addFeaturesToNFrameImpl(
      const swift_vio::CameraRig &refCameraSystem,
      std::shared_ptr<MultiFrameT> multiFrame,
      std::vector<std::unordered_map<size_t, size_t>> *keypointIndexForLandmarks) const {
    multiFrame->createTestImages(refCameraSystem.cameraGeometry(0)->imageHeight(),
                                 refCameraSystem.cameraGeometry(0)->imageWidth());
    PointLandmarkSimulationRS::projectLandmarksToNFrame<MultiFrameT, kSplineOrder>(
        homogeneousPoints_, translation_, rotation_, currentTime(),
        refCameraSystem, multiFrame, nullptr, keypointIndexForLandmarks,
        addImageNoise_ ? &imageNoiseMag_ : nullptr);
    size_t camId = 0;
    for (const auto & pts : *keypointIndexForLandmarks) {
      if (pts.size() < kNumPointInFrame) {
        VLOG(1) << "Inadequate landmarks observed in camera " << camId << " of frame at "
                << multiFrame->timestamp(camId) << ": " << pts.size() << " < " << kNumPointInFrame << "!";
      }
      ++camId;
    }
  }

  int expectedNumNFrames() const final;

  /**
   * @brief load landmarks.
   * The landmark ids are regenerated from IdProvider to avoid conflicts to Ids in estimators.
   * @param lmkCsv
   * @return number of landmarks.
   */
  size_t loadLandmarks(const std::string& lmkCsv);
};
#endif

void saveCameraParameters(
    std::shared_ptr<swift_vio::CameraRig> cameraSystem,
    const std::string cameraFile);

/**
 * @brief loadImuYaml
 * @param[in] imuYaml imu yaml in the format of the Kalibr output.
 * @param[out] imuParams. If imuYaml has the gravity_in_target field, then it will override the gravity in imuParams.
 */
void loadImuYaml(const std::string& imuYaml, okvis::ImuParameters* imuParams);

} // namespace simul
#endif // INCLUDE_SWIFT_VIO_SIM_DATA_INTERFACE_HPP_
