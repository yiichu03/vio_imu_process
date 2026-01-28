#include "simul/PointLandmarkSimulationRS.hpp"

#include <glog/logging.h>

namespace simul {
okvis::cameras::CameraBase::ProjectionStatus PointLandmarkSimulationRS::projectLandmark(
    std::shared_ptr<const simul::TrajectoryBase> simulatedTrajectory,
    const Eigen::Vector4d& hpW,
    const okvis::kinematics::Transformation& T_SCi,
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry,
    okvis::Time centralRowTime, Eigen::Vector2d* projection, Eigen::Vector4d *point_C) {
  double dt = 1e6; // f(t) / f'(t)
  double relativeFeatureTime = 0; // feature epoch relative to central row, i.e., frame timestamp.
  const double tol = 1e-5;
  int numIter = 0;
  const int maxIter = 5;

  double tr = cameraGeometry->readoutTime();
  double height = cameraGeometry->imageHeight();
  okvis::cameras::CameraBase::ProjectionStatus status;
  while (numIter < maxIter && std::fabs(dt) > tol) {
      okvis::Time featureTime = centralRowTime + okvis::Duration(relativeFeatureTime);
      okvis::kinematics::Transformation T_WBt =
          simulatedTrajectory->computeGlobalPose(featureTime);
      *point_C = T_SCi.inverse() * T_WBt.inverse() * hpW;
      Eigen::Matrix<double, 2, 4> pointJacobian;
      status = cameraGeometry->projectHomogeneous(
          *point_C, projection, &pointJacobian);
      if (status != okvis::cameras::CameraBase::ProjectionStatus::Successful) {
          break;
      }
      double f_of_t = (*projection)[1] - (relativeFeatureTime / tr + 0.5) * height;

      // compute Jacobians required by Newton Raphson method.
      Eigen::AlignedVector<okvis::kinematics::Transformation> transformList{T_SCi, T_WBt};
      std::vector<int> exponentList{-1, -1};
      swift_vio::MultipleTransformPointJacobian mtpj(transformList, exponentList, hpW);
      mtpj.computeJacobians();
      Eigen::Matrix<double, 4, 6> dpC_dT_WB = mtpj.dp_dT(0);
      okvis::kinematics::Transformation T_identity;
      Eigen::Vector3d v_WB = simulatedTrajectory->computeGlobalLinearVelocity(featureTime);
      Eigen::Vector3d omega_W =  simulatedTrajectory->computeGlobalAngularRate(featureTime);
      Eigen::Vector3d omega_B;
      omega_B.noalias() = T_WBt.C().transpose() * omega_W;
      swift_vio::TransformMultiplyJacobian tmj(T_WBt, T_identity, v_WB, omega_B);
      Eigen::Matrix<double, 6, 1> dT_WB_dt;
      dT_WB_dt.head<3>() = tmj.dp_dt();
      dT_WB_dt.tail<3>() = tmj.dtheta_dt();

      double fprime_of_t = pointJacobian.row(1) * dpC_dT_WB * dT_WB_dt - height / tr;
      dt = f_of_t / fprime_of_t;
//            LOG(INFO) << "Rs projection iter " << numIter << " dt " << dt;
      relativeFeatureTime -= dt;
      ++numIter;
  }
  return status;
}

inline std::pair<int, int> pixelToGridCoordinates(float x, float y, float cw, float ch) {
  return std::make_pair(std::floor(y / ch), std::floor(x / cw));
}

size_t PointLandmarkSimulationRS::sparsifyKeypointsByDistance(
    size_t imageWidth, size_t imageHeight,
    const std::vector<cv::KeyPoint> &keypoints,
    const std::vector<size_t> &landmarkIndices,
    std::vector<cv::KeyPoint> *sparseKeypoints,
    std::vector<size_t> *sparseLandmarkIndices) {
  typedef std::vector<std::pair<size_t, float>> GridCell;
  typedef std::vector<GridCell> GridRow;
  typedef std::vector<GridRow> Grid;
  Grid grid;
  // cell of size cw x ch
  float cw = 32.f;
  float ch = 24.f;
  int gw = std::ceil((float)imageWidth / cw);
  int gh = std::ceil((float)imageHeight / ch);
  grid.resize(gh, GridRow());
  for (GridRow &gr : grid) {
    gr.resize(gw, GridCell());
  }
  // assign keypoints and distances to each grid cell.
  for (size_t l = 0; l < landmarkIndices.size(); ++l) {
    const cv::KeyPoint &kp = keypoints[l];
    std::pair<int, int> rc = pixelToGridCoordinates(kp.pt.x, kp.pt.y, cw, ch);
    grid[rc.first][rc.second].emplace_back(l, kp.response);
  }

  // for each cell, find the keypoint of the shortest distance, and record it.
  sparseKeypoints->clear();
  sparseKeypoints->reserve(gw * gh);
  sparseLandmarkIndices->clear();
  sparseLandmarkIndices->reserve(gw * gh);
  for (const GridRow &gr : grid) {
    for (const GridCell &c : gr) {
      if (c.size()) {
        std::vector<std::pair<size_t, float>>::const_iterator it =
            std::min_element(c.begin(), c.end(),
                             [](const std::pair<size_t, float> &a,
                                const std::pair<size_t, float> &b) -> bool {
                               return a.second < b.second;
                             });
        sparseKeypoints->push_back(keypoints.at(it->first));
        sparseLandmarkIndices->push_back(landmarkIndices.at(it->first));
      }
    }
  }
  return sparseKeypoints->size();
}

}  // namespace simul
