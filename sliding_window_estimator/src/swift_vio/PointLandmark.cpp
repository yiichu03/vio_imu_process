#include <swift_vio/PointLandmark.hpp>

#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>

#include <swift_vio/FeatureTriangulation.hpp>
#include <swift_vio/ParallaxAnglePoint.hpp>
#include <swift_vio/PointLandmarkModels.hpp>
#include <swift_vio/FrameTypedefs.hpp>

namespace swift_vio {

TriangulationStatus PointLandmark::initialize(
    const std::vector<
        okvis::kinematics::Transformation,
        Eigen::aligned_allocator<okvis::kinematics::Transformation>>& T_WSs,
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>>& obsDirections,
    const std::vector<
        okvis::kinematics::Transformation,
        Eigen::aligned_allocator<okvis::kinematics::Transformation>>& T_BCs,
    const std::vector<
        okvis::kinematics::Transformation,
        Eigen::aligned_allocator<okvis::kinematics::Transformation>>& T_WCa_list,
    const std::vector<size_t>& cameraIndices,
    const std::vector<size_t>& anchorSeqIds) {
  std::vector<okvis::kinematics::Transformation,
              Eigen::aligned_allocator<okvis::kinematics::Transformation>>
      cam_states(obsDirections.size());
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      measurements(obsDirections.size());
  int jack = 0;
  for (auto obs3 : obsDirections) {
    measurements[jack] = obs3.head<2>();
    ++jack;
  }
  int joel = 0;
  switch (anchorSeqIds.size()) {
    case 1: { // AIDP
      // Ca will play the role of W
      okvis::kinematics::Transformation T_CaW = T_WCa_list[0].inverse();
      for (auto T_WS : T_WSs) {
        okvis::kinematics::Transformation T_WCi = T_WS * T_BCs[cameraIndices[joel]];
        cam_states[joel] = T_CaW * T_WCi;
        ++joel;
      }
      break;
    }
    case 2: { // PAP
      for (auto iter = T_WSs.begin(); iter != T_WSs.end(); ++iter, ++joel) {
        cam_states[joel] = *iter * T_BCs[cameraIndices[joel]];
      }
      swift_vio::ParallaxAnglePoint pap;
      TriangulationStatus status;
      status.triangulationOk =
          pap.initializePosition(obsDirections, cam_states, anchorSeqIds);
      status.triangulationOk = status.triangulationOk &&
          pap.optimizePosition(obsDirections, cam_states, anchorSeqIds);

      std::vector<double> params;
      pap.getParameters(&params);
      setEstimate(params);

      status.chi2Small = true;
      status.flipped = false;
      status.raysParallel = false;
      return status;
    }
    case 0: // HPP
    default:
      for (auto iter = T_WSs.begin(); iter != T_WSs.end(); ++iter, ++joel) {
        cam_states[joel] = *iter * T_BCs[cameraIndices[joel]];
      }
      break;
  }

  swift_vio::Feature feature(measurements, cam_states);
  feature.initializePosition();
  Eigen::Vector4d hp = feature.homogeneousPosition();
  if (modelId_ == InverseDepthParameterization::kModelId) {
    double inverseDepth = 1.0 / hp[2];
    hp *= inverseDepth;
    hp[2] = 1.0;
  }
  setEstimate(hp);

  TriangulationStatus status;
  status.triangulationOk = feature.is_initialized && feature.is_chi2_small;
  status.chi2Small = feature.is_chi2_small;
  status.flipped = feature.is_flipped;
  status.raysParallel = feature.is_parallel;
  return status;
}
} // namespace swift_vio
