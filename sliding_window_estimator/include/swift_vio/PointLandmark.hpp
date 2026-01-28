#ifndef INCLUDE_SWIFT_VIO_POINT_LANDMARK_HPP_
#define INCLUDE_SWIFT_VIO_POINT_LANDMARK_HPP_

#include <okvis/FrameTypedefs.hpp>
#include <okvis/ceres/LocalParamizationAdditionalInterfaces.hpp>
#include <vector>

namespace swift_vio {
struct TriangulationStatus {
  bool triangulationOk; // True if the landmark is in front of every camera.
  bool chi2Small;
  bool raysParallel; // True if rotation compensated observation directions are
                     // parallel.
  bool flipped;      // True if the landmark is flipped to be in front of every
                     // camera.
  bool lackObservations; // True if #obs is too few.
  bool noKeyframeHost; // If a landmark is not observed in any keyframe, it cannot be hosted by a keyframe.
  TriangulationStatus()
      : triangulationOk(false), chi2Small(true), raysParallel(false),
        flipped(false), lackObservations(false), noKeyframeHost(false) {}
};

enum class MeasurementJacobianStatus {
  Successful = 0,
  GeneralProjectionFailed = 1,
  MainAnchorProjectionFailed = 2,
  AssociateAnchorProjectionFailed = 3
};

struct FeatureJacobianStatus {
  TriangulationStatus triangulationStatus;
  bool successful;

  FeatureJacobianStatus() : successful(false) {}
};

inline int
eraseBadObservations(const std::vector<std::pair<uint64_t, int>> &dudIds,
                     std::vector<uint64_t> *candidateFrameIds) {
  int numErased = 0;
  for (auto dud : dudIds) {
    uint64_t frameId = dud.first;
    auto iter =
        std::find_if(candidateFrameIds->begin(), candidateFrameIds->end(),
                     [frameId](const uint64_t &s) { return s == frameId; });
    if (iter != candidateFrameIds->end()) {
      candidateFrameIds->erase(iter);
      ++numErased;
    }
  }
  return numErased;
}

class PointLandmark {
public:
  PointLandmark(uint64_t id, int modelId,
                okvis::ceres::LocalParamizationAdditionalInterfaces
                    *localParameterizationPtr)
      : id_(id), modelId_(modelId), fixed_(false), initialized_(false), linPointFixed_(false),
        localParameterizationPtr_(localParameterizationPtr) {
  }

  /**
   * @brief initialize landmark parameters.
   * @param T_WSs
   * @param obsDirections a list of image coordinates at z=1, [x, y, 1]
   * @param T_BC0
   * @param anchorSeqId main anchor, associate anchor id
   * @return
   */
  TriangulationStatus initialize(
      const std::vector<
          okvis::kinematics::Transformation,
          Eigen::aligned_allocator<okvis::kinematics::Transformation>> &T_WSs,
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>>
          &obsDirections,
      const std::vector<
          okvis::kinematics::Transformation,
          Eigen::aligned_allocator<okvis::kinematics::Transformation>> &T_BCs,
      const std::vector<
          okvis::kinematics::Transformation,
          Eigen::aligned_allocator<okvis::kinematics::Transformation>>
          &T_WCa_list,
      const std::vector<size_t> &cameraIndices,
      const std::vector<size_t> &anchorSeqIds);

  double *data() { return parameters_.data(); }

  const double *data() const { return parameters_.data(); }

  size_t size() const { return parameters_.size(); }

  uint64_t id() const { return id_; }

  okvis::ceres::LocalParamizationAdditionalInterfaces *
  localParameterizationPtr() const {
    return localParameterizationPtr_;
  }

  Eigen::VectorXd estimate() const {
    Eigen::VectorXd result;
    result.resize(parameters_.size());
    memcpy(result.data(), parameters_.data(),
           parameters_.size() * sizeof(double));
    return result;
  }

  void setEstimate(const std::vector<double> &params) {
    parameters_ = params;
    if (!linPointFixed_) {
      parametersLin_ = parameters_;
    }
    initialized_ = true;
  }

  void setEstimate(const Eigen::VectorXd &params) {
    parameters_.resize(params.size());
    memcpy(parameters_.data(), params.data(), params.size() * sizeof(double));
    if (!linPointFixed_) {
      parametersLin_ = parameters_;
    }
    initialized_ = true;
  }

  int modelId() const { return modelId_; }

  void setModelId(int modelId) { modelId_ = modelId; }

  void setLocalParameterizationPtr(
      okvis::ceres::LocalParamizationAdditionalInterfaces
          *localParameterizationPtr) {
    localParameterizationPtr_ = localParameterizationPtr;
  }

  Eigen::VectorXd getLinearizationPoint() const {
    Eigen::VectorXd result;
    result.resize(parameters_.size());
    memcpy(result.data(), parametersLin_.data(),
           parametersLin_.size() * sizeof(double));
    return result;
  }

  /**
   * @brief fix the lin point, but does not change current estimate.
   * @param params
   */
  void fixLinPoint(const Eigen::VectorXd &params) {
    memcpy(parametersLin_.data(), params.data(), params.size() * sizeof(double));
    linPointFixed_ = true;
  }

  /**
   * @brief set lin point fixed.
   */
  void fixLinPoint() {
    linPointFixed_ = true;
  }

  /**
   * @brief Plus update both the lin point if needed and the current estimate.
   * @param Delta_Chi
   */
  void Plus(const double *Delta_Chi) {
    localParameterizationPtr_->Plus(parameters_.data(), Delta_Chi, parameters_.data());
    if (!linPointFixed_) {
      parametersLin_ = parameters_;
    }
  }

  /**
   * @brief PlusForJacobian update only the lin point.
   * @param Delta_Chi
   */
  void PlusForJacobian(const double *Delta_Chi) {
    localParameterizationPtr_->Plus(parametersLin_.data(), Delta_Chi,
                                    parametersLin_.data());
  }

  void minus(const double *x0, const double *x0_plus_Delta,
             double *Delta_Chi) const {
    localParameterizationPtr_->Minus(x0, x0_plus_Delta, Delta_Chi);
  }

  bool ComputeJacobian(double* jacobian) const {
    return localParameterizationPtr_->ComputeJacobian(parametersLin_.data(), jacobian);
  }

private:
  uint64_t id_;
  int modelId_;
  std::vector<double> parameters_;
  std::vector<double> parametersLin_;
  bool fixed_;
  bool initialized_;
  bool linPointFixed_;

  okvis::ceres::LocalParamizationAdditionalInterfaces
      *localParameterizationPtr_;
};
} // namespace swift_vio
#endif // INCLUDE_SWIFT_VIO_POINT_LANDMARK_HPP_
