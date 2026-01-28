#ifndef ESTIMATIONHELPER_H
#define ESTIMATIONHELPER_H

#include "okvis/ceres/Map.hpp"
#include "okvis/kinematics/Transformation.hpp"
#include "okvis/Time.hpp"

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/ceres/RsReprojectionErrorAidpAdapter.hpp>
#include <swift_vio/imu/BoundedImuDeque.hpp>
#include <swift_vio/imu/ImuRig.hpp>
#include <swift_vio/MapPoint.h>
#include <swift_vio/MultiFrame.hpp>
#include <swift_vio/PointLandmark.hpp>
#include <swift_vio/PointSharedData.hpp>

#include <swift_vio/States.h>

namespace swift_vio {
class EstimationHelper
{
public:
  EstimationHelper();

  static void
  addNavStateVariables(const okvis::kinematics::Transformation &T_WS,
                       const Eigen::Vector3d &v_WS, States *latestState,
                       std::shared_ptr<okvis::ceres::Map> mapPtr,
                       bool fixLinPoint);

  static void addGravityVariable(const Eigen::Vector3d &gW, States *latestState,
                          std::shared_ptr<okvis::ceres::Map> mapPtr,
                          bool estimate_gravity_direction);

  static void addImuBiasParameters(const ImuRig &imuRig, States *latestState, std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void addImuParameters(const ImuRig &imuRig, std::vector<std::shared_ptr<okvis::ImuNoiseParameters>> &noiseVec,
                               States *latestState, std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void addCameraParameters(const CameraRig &cameraRig,
                                  const okvis::CameraNoiseParametersVec &noiseVec,
                                  States *latestState,
                                  std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void addNavStatePriorFactor(const InitialNavState &initialNavState,
                                     States *latestState,
                                     std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void addCameraParamPriorFactor(
      const okvis::CameraNoiseParametersVec &cameraNoiseParametersList,
      const CameraRig &cameraRig, States *latestState,
      std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void addImuPriorFactor(
      const std::vector<std::shared_ptr<okvis::ImuNoiseParameters>>
          &imuParametersVec,
      States *latestState, std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void usePreviousGravity(const States &previousState, States *currentState);

  static void usePreviousImuBias(const States &previousState, States *currentState);

  static void usePreviousImuAugmentedParameters(const States &previousState, States *currentState);

  static void usePreviousCameraParameters(const States &previousState, States *currentState);

  static void setImuWindow(const swift_vio::BoundedImuDeque &inertialMeasForStates, States *oneState, okvis::Duration halfBuffer);

  static int predictNavState(const okvis::ImuMeasurementDeque &imuMeasurements,
                             const States &previousState,
                             std::shared_ptr<const okvis::ceres::Map> mapPtr,
                             const okvis::ImuParameters &imuParameters,
                             okvis::Time currentStateTime,
                             okvis::kinematics::Transformation *T_WS,
                             Eigen::Vector3d *v_WS);

  static int
  propagateNavStateAndCov(const okvis::ImuMeasurementDeque &imuMeasurements,
                          const States &previousState,
                          std::shared_ptr<const okvis::ceres::Map> mapPtr,
                          okvis::ImuParameters imuParameters,
                          okvis::Time currentStateTime,
                          okvis::kinematics::Transformation *T_WS,
                          Eigen::Vector3d *v_WS, Eigen::MatrixXd *covariance);

  static void
  addImuObservations(const okvis::ImuMeasurementDeque &imuMeasurements, size_t imuId,
                     int imuModelId, const okvis::ImuParameters &imuParameters,
                     const States &lastState, const States &currentState,
                     std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void
  addImuObservationsConstBias(const okvis::ImuMeasurementDeque &imuMeasurements, size_t imuId,
                     int imuModelId, const okvis::ImuParameters &imuParameters,
                     const States &lastState, const States &currentState,
                     std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void addCameraParameterConstraints(
      const okvis::CameraNoiseParametersVec
          &cameraNoiseParametersList,
      const States &lastState, const States &currentState,
      std::shared_ptr<okvis::ceres::Map> mapPtr);

  static void updateFeatureTracks(
      const FeatureTrackMap &newObservations,
      std::shared_ptr<const MultiFrame> currentNFrame, PointMap *trackMap,
      std::unordered_map<uint64_t, BareMultiFrame> *nframe2LandmarkIds);

  /**
   * @brief computeKeypointOpticFlow
   * @param landmarksMap observations in nframes
   * @param nframeMap all the nframes
   * @param camId
   * @param framewiseDisparity: list of tuples (time gap, average optic flow)
   * between consecutive nframes based on images of camId.
   */
  static void computeKeypointOpticFlow(
      const PointMap &landmarksMap,
      const std::map<uint64_t, std::vector<okvis::Time>> &nframeMap, size_t camId,
      std::vector<std::pair<okvis::Duration, float>> *framewiseDisparity);

  static void createMinimalNFrameMap(
      const std::unordered_map<uint64_t, BareMultiFrame> &nframe2LandmarkIds,
      std::map<uint64_t, std::vector<okvis::Time>> *miniNFrameMap);

  static size_t removeOldFrames(
      uint64_t oldestNFrameId,
      std::unordered_map<uint64_t, BareMultiFrame> *nframeToLandmarkIds);

  static size_t removeOldObservations(uint64_t oldestNFrameId,
                                    PointMap *landmarksMap);

  static size_t gatherMapPointObservationsRs(
      const MapPoint &mp, std::shared_ptr<const okvis::ceres::Map> mapPtr,
      const BareMultiFrameMap &frameTimeMap, const CameraRig &cameraRig,
      swift_vio::PointSharedData *pointDataPtr,
      Eigen::AlignedVector<Eigen::Vector3d> *obsDirections,
      Eigen::AlignedVector<Eigen::Vector2d> *obsList,
      Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
      std::vector<std::pair<uint64_t, int>> *orderedBadFrameIds);

  static size_t gatherMapPointObservations(
      const MapPoint &mp,
      std::shared_ptr<const okvis::ceres::Map> mapPtr,
      const CameraRig &cameraRig,
      swift_vio::PointSharedData *pointDataPtr,
      Eigen::AlignedVector<Eigen::Vector3d> *obsDirections,
      Eigen::AlignedVector<Eigen::Vector2d> *obsList,
      Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
      std::vector<std::pair<uint64_t, int>> *orderedBadFrameIds);

  static void getCameraTimeParameterPtrs(
      const States &currentState,
      std::shared_ptr<const okvis::ceres::Map> mapPtr,
      std::vector<const double *> *cameraDelayParameterPtrs,
      std::vector<const double *> *cameraReadoutTimeParameterPtrs);

  static void getImuAugmentedParameterPtrs(const States &currentState,
      std::shared_ptr<const okvis::ceres::Map> mapPtr,
      const size_t imuIdx, 
      std::vector<const double *> *imuAugmentedParamPtrs,
      std::vector<size_t> *imuAugmentedParamDims);

  static void propagatePoseAndVelocityForMapPoint(
      const StateMap & statesMap, std::shared_ptr<const okvis::ceres::Map> mapPtr,
      std::shared_ptr<const okvis::ImuParameters> &imuParams,
      swift_vio::PointSharedData* pointDataPtr);

  /**
   * @brief triangulateMapPoint considering the rolling shutter effect
   * @param mp
   * @param statesMap
   * @param mapPtr
   * @param frameTimeMap
   * @param cameraRig
   * @param imuParams
   * @param landmark
   * @param obsList
   * @param obsStdList
   * @param pointDataPtr
   * @param orderedFrameIdsToUse
   * @param minTrackLength
   * @param minCulledFrames
   * @param checkDisparity
   * @param anchorInKeyframe is the anchor of the point required to be a keyframe?
   * @return
   */
  static TriangulationStatus triangulateMapPointRs(
      const MapPoint &mp, const StateMap &statesMap,
      std::shared_ptr<const okvis::ceres::Map> mapPtr,
      const BareMultiFrameMap &frameTimeMap, const CameraRig &cameraRig,
      std::shared_ptr<const okvis::ImuParameters> imuParams,
      swift_vio::PointLandmark *landmark,
      Eigen::AlignedVector<Eigen::Vector2d> *obsList,
      Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
      swift_vio::PointSharedData *pointDataPtr,
      std::vector<uint64_t> *orderedFrameIdsToUse, size_t minTrackLength,
      size_t minCulledFrames, bool checkDisparity = false,
      bool anchorInKeyframe = false);

  /**
   * @brief triangulateMapPoint
   * @param mp
   * @param mapPtr
   * @param cameraRig
   * @param landmark
   * @param obsList
   * @param obsStdList
   * @param pointDataPtr
   * @param minTrackLength
   * @param checkDisparity
   * @return
   */
  static TriangulationStatus triangulateMapPoint(
      const MapPoint &mp,
      std::shared_ptr<const okvis::ceres::Map> mapPtr,
      const CameraRig &cameraRig,
      swift_vio::PointLandmark *landmark,
      Eigen::AlignedVector<Eigen::Vector2d> *obsList,
      Eigen::AlignedVector<Eigen::Vector2d> *obsStdList,
      swift_vio::PointSharedData *pointDataPtr,
      size_t minTrackLength, bool checkDisparity);

  static Eigen::Vector4d anchoredInverseDepthToWorldCoordinates(
      std::shared_ptr<const okvis::ceres::Map> mapPtr, const CameraRig &cameraRig,
      const Eigen::Vector4d &ab1rho, const CameraIdentifier &anchorCamera);

  static void addCameraObservations(
      const StateMap &statesMap,
      const std::unordered_map<uint64_t, BareMultiFrame> &frameTimeMap,
      const CameraRig &cameraRig, PointMap *landmarksMap,
      std::shared_ptr<okvis::ceres::Map> mapPtr,
      std::shared_ptr<const okvis::ImuParameters> imuParameters,
      ::ceres::LossFunction *lossFuncPtr);

  static void configureSolver(std::shared_ptr<okvis::ceres::Map> mapPtr, size_t numIter,
                                         size_t numThreads, bool verbose);

  // transfer the state variables in map to a new world frame, so that
  // the state pose at poseId is T_W0B0.
  static void transferWorldFrame(
      const StateMap &statesMap, const okvis::kinematics::Transformation &T_W0B0, uint64_t poseId,
      std::shared_ptr<okvis::ceres::Map> mapPtr, PointMap *landmarks);

  static void updateLandmarksFromMap(
      std::shared_ptr<const okvis::ceres::Map> mapPtr, const CameraRig &cameraRig,
      PointMap *landmarksMap);


  static void updateCameraRig(const States &lastState,
                              std::shared_ptr<const okvis::ceres::Map> mapPtr,
                              const okvis::CameraNoiseParametersVec &noiseVec,
                              CameraRig *cameraRig);

  static void updateImuRig(const States &lastState,
                           std::shared_ptr<const okvis::ceres::Map> mapPtr,
                           ImuRig *imuRig);

  static void updateImuParametersVector(
      const States &lastState, std::shared_ptr<const okvis::ceres::Map> mapPtr,
      std::vector<std::shared_ptr<okvis::ImuNoiseParameters>> *imuParamsVec);

  template <class GEOMETRY_TYPE>
  static ::ceres::ResidualBlockId addPointFrameAidpResidual(
      const States &targetState, const States &hostState,
      std::shared_ptr<const GEOMETRY_TYPE> cameraGeometry, uint64_t landmarkId,
      const okvis::KeypointIdentifier &kpi, const KeypointObservation &obs,
      const CameraIdentifier &hostCamera, std::shared_ptr<okvis::ceres::Map> mapPtr,
      std::shared_ptr<const okvis::ImuParameters> imuParameters,
      okvis::Time targetImageTime, ::ceres::LossFunction *lossFunctionPtr) {
    double obsVar = (obs.size * obs.size) / 64.0;
    Eigen::Matrix2d covariance = Eigen::Vector2d(obsVar, obsVar).asDiagonal();
    std::shared_ptr<okvis::ceres::RsReprojectionErrorAidpAdapter<GEOMETRY_TYPE>>
        reprojectionError(
            new okvis::ceres::RsReprojectionErrorAidpAdapter<GEOMETRY_TYPE>(
                CameraIdentifier(kpi.frameId, kpi.cameraIndex), hostCamera,
                obs.uv.cast<double>(), covariance, cameraGeometry,
                targetState.imuReadingWindow, imuParameters,
                targetState.timestamp, targetImageTime));
    reprojectionError->setParameterBlockAndResidualSizes();
    size_t imuIdx = imuParameters->imuIdx;
    const SpecificSensorStatesContainer &imuInfos =
        targetState.sensors.at(SensorStates::Imu).at(imuIdx);
    std::vector<std::shared_ptr<okvis::ceres::ParameterBlock>>
        parameterBlockPtrs{
            mapPtr->parameterBlockPtr(targetState.id),
            mapPtr->parameterBlockPtr(landmarkId),
            mapPtr->parameterBlockPtr(hostCamera.frameId),
            mapPtr->parameterBlockPtr(
                targetState.sensors.at(SensorStates::Camera)
                    .at(kpi.cameraIndex)
                    .at(CameraSensorStates::T_XCi)
                    .id),
            mapPtr->parameterBlockPtr(hostState.sensors.at(SensorStates::Camera)
                                          .at(hostCamera.cameraIndex)
                                          .at(CameraSensorStates::T_XCi)
                                          .id),
            mapPtr->parameterBlockPtr(
                targetState.sensors.at(SensorStates::Camera)
                    .at(kpi.cameraIndex)
                    .at(CameraSensorStates::Intrinsics)
                    .id),
            mapPtr->parameterBlockPtr(
                targetState.sensors.at(SensorStates::Camera)
                    .at(kpi.cameraIndex)
                    .at(CameraSensorStates::TR)
                    .id),
            mapPtr->parameterBlockPtr(
                targetState.sensors.at(SensorStates::Camera)
                    .at(kpi.cameraIndex)
                    .at(CameraSensorStates::TD)
                    .id),
            mapPtr->parameterBlockPtr(
                targetState.global.at(GlobalStates::v_WS).id),
            mapPtr->parameterBlockPtr(imuInfos.at(ImuSensorStates::Bias).id)};

    if (imuInfos.at(ImuSensorStates::MG).exists) {
      parameterBlockPtrs.emplace_back(
          mapPtr->parameterBlockPtr(imuInfos.at(ImuSensorStates::MG).id));
      parameterBlockPtrs.emplace_back(
          mapPtr->parameterBlockPtr(imuInfos.at(ImuSensorStates::TS).id));
      parameterBlockPtrs.emplace_back(
          mapPtr->parameterBlockPtr(imuInfos.at(ImuSensorStates::MA).id));
    }
    reprojectionError->uniqueBlocks(&parameterBlockPtrs);
    ::ceres::ResidualBlockId retVal = mapPtr->addResidualBlock(
        reprojectionError, lossFunctionPtr, parameterBlockPtrs);

    return retVal;
  }

  static void get_T_WB(std::shared_ptr<const okvis::ceres::Map> mapPtr,
                       uint64_t poseId,
                       okvis::kinematics::Transformation &T_WB);

  static void get_T_WB_lin(std::shared_ptr<const okvis::ceres::Map> mapPtr,
                           uint64_t poseId,
                           okvis::kinematics::Transformation &T_WB);

  static void getVariableCameraIntrinsics(
      const States &currentState,
      std::shared_ptr<const okvis::ceres::Map> mapPtr,
      size_t camIdx, const okvis::CameraNoiseParameters &camNoiseParams,
      Eigen::Matrix<double, Eigen::Dynamic, 1> *cameraParams);

  // Remove an observation from a landmark's feature track.
  static bool removeObservationAndResidual(
      ::ceres::ResidualBlockId residualBlockId,
      std::shared_ptr<okvis::ceres::Map> mapPtr, PointMap *landmarksMap,
      std::unordered_map<uint64_t, BareMultiFrame> *nframeToLandmarkIds);

  static void removeObservationInFrames(
      const std::vector<uint64_t> &orderedRemoveFrames,
      Eigen::AlignedMap<okvis::KeypointIdentifier, KeypointObservation>
          *observations,
      std::shared_ptr<okvis::ceres::Map> mapPtr,
      std::unordered_map<uint64_t, BareMultiFrame> *nframeToLandmarkIds);

  // Redundant states are ordinary states except for the last numImuFrames and
  // those key states more distant in time to the current state.
  template <typename StateMapT>
  static void findRedundantNavStates(const StateMapT &statesMap,
                                     std::vector<uint64_t> *toRemoveStateIds,
                                     size_t numImuFrames,
                                     size_t minCulledFrames) {
    int closeFrames(0), oldFrames(0);
    uint64_t latestKeyframeId = 0;
    toRemoveStateIds->clear();
    toRemoveStateIds->reserve(minCulledFrames);
    auto rit = statesMap.rbegin();
    for (size_t j = 0; j < numImuFrames; ++j) {
      if (rit->second.isKeyframe && latestKeyframeId == 0) {
        latestKeyframeId = rit->first;
      }
      ++rit;
    }
    for (; rit != statesMap.rend(); ++rit) {
      if (toRemoveStateIds->size() >= minCulledFrames) {
        break;
      }
      if (!rit->second.isKeyframe) {
        toRemoveStateIds->push_back(rit->first);
        ++closeFrames;
      } else {
        if (latestKeyframeId == 0) {
          latestKeyframeId = rit->first;
        }
      }
    }
    if (toRemoveStateIds->size() < minCulledFrames) {
      for (auto it = statesMap.begin(); it != --statesMap.end(); ++it) {
        if (it->first == latestKeyframeId) {
          break;
        }
        if (it->second.isKeyframe) {
          toRemoveStateIds->push_back(it->first);
          ++oldFrames;
        }
        if (toRemoveStateIds->size() >= minCulledFrames)
          break;
      }
    }
    sort(toRemoveStateIds->begin(), toRemoveStateIds->end());
  }

  /**
   * @brief Find redundant nav states using a distance heuristic.
   *   Redundant states are ordinary states except for the last numImuFrames and
   *   those key states more distant in position to the current state.
   * @param[out] toRemoveStateIds sorted in ascending order.
   */
  template <typename StateMapT>
  static void
  findRedundantNavStates(const StateMapT &statesMap,
                         std::shared_ptr<const okvis::ceres::Map> mapPtr,
                         std::vector<uint64_t> *toRemoveStateIds,
                         size_t numImuFrames, size_t minCulledFrames) {
    toRemoveStateIds->clear();
    toRemoveStateIds->reserve(minCulledFrames);
    auto rit = statesMap.rbegin();
    for (size_t j = 0; j < numImuFrames; ++j) {
      ++rit;
    }
    for (; rit != statesMap.rend(); ++rit) {
      if (toRemoveStateIds->size() >= minCulledFrames) {
        break;
      }
      if (!rit->second.isKeyframe) {
        toRemoveStateIds->push_back(rit->first);
      }
    }
    if (toRemoveStateIds->size() < minCulledFrames) {
      // pick up the keyframes most distant to the current frame.
      okvis::kinematics::Transformation T_WBc =
          std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
              mapPtr->parameterBlockPtr(statesMap.rbegin()->first))
              ->estimate();
      std::vector<std::pair<uint64_t, double>> keyframeIdToDistances;
      keyframeIdToDistances.reserve(4);
      auto rend = statesMap.end();
      for (size_t j = 0; j < numImuFrames; ++j) {
        --rend;
      }
      for (auto it = statesMap.begin(); it != rend; ++it) {
        if (it->second.isKeyframe) {
          okvis::kinematics::Transformation T_WBk =
              std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
                  mapPtr->parameterBlockPtr(it->first))
                  ->estimate();
          double dist = (T_WBc.r() - T_WBk.r()).norm();
          keyframeIdToDistances.emplace_back(it->first, dist);
        }
      }
      std::sort(keyframeIdToDistances.begin(), keyframeIdToDistances.end(),
                [](const std::pair<uint64_t, double> &a,
                   const std::pair<uint64_t, double> &b) -> bool {
                  return a.second > b.second;
                });
      for (auto it = keyframeIdToDistances.begin();
           it != --keyframeIdToDistances.end(); ++it) {
        toRemoveStateIds->push_back(it->first);
        if (toRemoveStateIds->size() >= minCulledFrames) {
          break;
        }
      }
    }
    sort(toRemoveStateIds->begin(), toRemoveStateIds->end());
  }

  /**
   * find the keyframe outside the IMU window and most distant to the latest frame.
   */
  template <typename StateMapT>
  static uint64_t
  findDistantKeyframe(const StateMapT &statesMap,
                         std::shared_ptr<const okvis::ceres::Map> mapPtr,
                         size_t numImuFrames) {
    okvis::kinematics::Transformation T_WBc =
        std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
            mapPtr->parameterBlockPtr(statesMap.rbegin()->first))
            ->estimate();
    std::vector<std::pair<uint64_t, double>> keyframeIdToDistances;
    keyframeIdToDistances.reserve(4);
    auto rend = statesMap.end();
    for (size_t j = 0; j < numImuFrames; ++j) {
      --rend;
    }
    for (auto it = statesMap.begin(); it != rend; ++it) {
      if (it->second.isKeyframe) {
        okvis::kinematics::Transformation T_WBk =
            std::static_pointer_cast<const okvis::ceres::PoseParameterBlock>(
                mapPtr->parameterBlockPtr(it->first))
                ->estimate();
        double dist = (T_WBc.r() - T_WBk.r()).norm();
        keyframeIdToDistances.emplace_back(it->first, dist);
      }
    }
    std::sort(keyframeIdToDistances.begin(), keyframeIdToDistances.end(),
              [](const std::pair<uint64_t, double> &a,
                 const std::pair<uint64_t, double> &b) -> bool {
                return a.second > b.second;
              });
    return keyframeIdToDistances.front().first;
  }

  static uint64_t shouldMarginalizeSensorParamBlock(
      int sensorType, int sensorId, int sensorParamType,
      std::map<uint64_t, States>::const_iterator it,
      std::shared_ptr<const okvis::ceres::Map> mapPtr);
};
}  // namespace swift_vio
#endif // ESTIMATIONHELPER_H
