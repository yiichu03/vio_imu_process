#ifndef SWIFT_VIO_VI_BUNDLE_ADJUSTMENT_H
#define SWIFT_VIO_VI_BUNDLE_ADJUSTMENT_H

#include <boost/range/iterator_range.hpp>

#include <ceres/problem.h>

#include <okvis/Parameters.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/LocalParamizationAdditionalInterfaces.hpp>
#include <okvis/ceres/ParameterBlock.hpp>

#include <simul/SimulatorBase.hpp>
#include <simul/SimParameters.h>

#include <swift_vio/InitialNavState.hpp>
#include <swift_vio/ProjectionIntrinsicReps.h>

namespace swift_vio {
typedef boost::iterator_range<std::vector<std::pair<double, double>>::iterator> HistogramType;

struct StateInfo {
  okvis::Time time;
  uint64_t vertexId;       // parameter block id.
  uint64_t speedAndBiasId; // parameter block id.
  std::shared_ptr<const okvis::ImuMeasurementDeque> imuMeasCanopy;

  StateInfo(okvis::Time stateTime, uint64_t vid, uint64_t sbId,
            std::shared_ptr<const okvis::ImuMeasurementDeque> imuSegment)
      : time(stateTime), vertexId(vid), speedAndBiasId(sbId),
        imuMeasCanopy(imuSegment) {}
};

struct CameraBlockInfo {
  uint64_t TBCId;
  uint64_t IntrinsicsId;
  uint64_t trId;
  uint64_t tdId;

  CameraBlockInfo(uint64_t TBCId, uint64_t IntrinsicsId, uint64_t trId,
                  uint64_t tdId)
      : TBCId(TBCId), IntrinsicsId(IntrinsicsId), trId(trId), tdId(tdId) {}
};

struct ReprojectionStatistics {
  double rawErrorSquared;
  size_t validErrors;
  size_t totalErrors;

  ReprojectionStatistics() {
  }

  ReprojectionStatistics(double _rawErrorSquared, size_t _validErrors,
                         size_t _totalErrors)
      : rawErrorSquared(_rawErrorSquared), validErrors(_validErrors),
        totalErrors(_totalErrors) {}

  double rmseError() const {
    return std::sqrt(rawErrorSquared / validErrors);
  }

  std::string toString(const std::string delimiter=",") const {
     std::stringstream ss;
     ss << "Reprojection raw RMSE [px], valid terms, total terms\n";
     ss << rmseError() << delimiter << validErrors << delimiter << totalErrors;
     return ss.str();
  }
};

struct VisualInertialBAOptions {
  bool estimateCameraIntrinsics;
  bool estimateCameraDelay;
  bool estimateReadoutTime;
  VisualInertialBAOptions(bool estimateIntrinsics = false,
                          bool estimateTd = true, bool estimateTr = true)
      : estimateCameraIntrinsics(estimateIntrinsics),
        estimateCameraDelay(estimateTd), estimateReadoutTime(estimateTr) {}
};

/**
 * @brief The ViBundleAdjustment optimizes the visual inertial problem given in
 * maplab csv format. The original motivation for this class is to optimize
 * camera - IMU extrinsic parameters, camera time offsets, and frame readout
 * times for multiple cameras given camera observations of landmarks on a
 * calibration pattern and corresponding IMU data.
 *
 * Modeling: State variables
 * trajectory vertices at specified epochs, each vertex has pose (rotation and
 * translation), velocity landmarks in R^3. for each camera intrinsic parameters
 * including distortion, time offset, frame readout time, camera extrinsics for
 * each IMU IMU intrinsic parameters, and IMU biases IMU biases are at specified
 * epochs (maybe not coincident to the vertices)
 *
 * Among these variables,
 *
 * Poses T_WBt
 * velocities v_WBt
 * biases bg_t
 * ba_t
 * landmarks are known constants, Li
 *
 * camera parameters,  Xci = {T_BC, td, tr, pi}
 * pi proj and dist intrinsics are knonw constants.
 * and IMU intrinsic parameters are unknown constants,
 * Tg = eye(3) Ts = zero(3, 3) Ta = eye(3)
 * [1, 0, 0, 0, 1, 0, 0,0, 0]
 *
 * gravity vector
 * gW
 * W: calibration target XYZ
 *
 * poses, velocites and IMU biases are time varying variables.
 *
 * Factors
 * one Prior(T_WB0) not necessary.
 * cov 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2
 *
 *
 * multiple camera observations
 * e = z - h(T_WBt, Li, Xci, gravityW, Tg Ts Ta)
 * covariance = Matrix2d
 *
 * multiple IMU errors
 * e = e(T_WBi , v_WBi, bgi, bai, T_WBj, v_WBj, bgj, baj, gravityW, z_imu,
 * imuInfo);
 *
 */
class ViBundleAdjustment {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ViBundleAdjustment();

  virtual ~ViBundleAdjustment();

  int addCameraParameterStds(
      const okvis::CameraNoiseParameters &cameraNoiseParameters);

  void
  addCameraSystem(std::shared_ptr<const CameraRig> cameras);

  // Add an IMU to the configuration.
  int addImu(const okvis::ImuParameters &imuParameters);

  void buildOptimizationProblem(std::shared_ptr<simul::SimulatorBase> data,
                                const VisualInertialBAOptions &baOptions);

  /**
   * @brief add all state variables.
   */
  void addStateVariables(const VisualInertialBAOptions& baOptions);

  /**
   * @brief add time-varying IMU biases at motion vertex epochs, and optional
   * time-invariant IMU intrinsic parameters.
   */
  void addImuParameters();

  /**
   * @brief add intrinsic, extrinsic, and temporal parameters for all cameras.
   */
  void addCameraParameters(const VisualInertialBAOptions& baOptions);

  /**
   * @brief add pose and velocities at given epochs with initial values.
   */
  void addMotionVariables();

  /**
   * @brief add landmarks with initial values.
   */
  void addLandmarks();

  /**
   * @brief add all error terms.
   */
  void addErrorTerms();

  /**
   * @brief add camera observations
   */
  void addCameraObservations();

  /**
   * @brief add IMU preintegration factors and IMU bias relative contraints.
   */
  void addImuFactors();

  /**
   * @brief add prior on the first pose (XYZ, yaw) mainly by specifying the
   * information matrix.
   */
  void addPrior();

  void computeStatistics(double maxProjectionErrorTol = 7.0);

  /**
   * @brief solve the ceres problem.
   */
  void solve();

  /**
   * @brief solve the ceres problem with an outlier rejection scheme.
   */
  void solveWithOutlierRejection();

  /**
   * @brief save the optimization results to outputdir
   * @param outputdir
   */
  void saveResults(const std::string &outputdir);

  /**
   * @brief saveImuParams e.g., Tg Ts Ta.
   * @param ImuYaml with yaml format readable by python yaml
   */
  void saveImuParams(const std::string& imuYaml) const;

  /**
   * @brief saveCameraParameters
   * @param cameraImuYaml with yaml format readable by python yaml
   */
  void saveCameraParameters(const std::string cameraImuYaml) const;

  void saveResidualStats(const std::string residualStatFile);

  void saveMotion(const std::string &truthFile) const;

  void checkHist(const std::string& msg = "") const;

private:
  std::shared_ptr<simul::SimulatorBase> data_;

  std::shared_ptr<::ceres::Problem> problem_;
  std::unordered_map<uint64_t, std::shared_ptr<okvis::ceres::ParameterBlock>>
      idToParameterBlocks_;

  std::unordered_map<uint64_t, StateInfo> vertexToState_;

  std::shared_ptr<const CameraRig> cameraSystem_;
  Eigen::AlignedVector<okvis::CameraNoiseParameters>
      cameraNoiseParametersVec_; ///< Extrinsics parameters.
  std::vector<std::shared_ptr<okvis::ImuParameters>>
      imuParametersVec_; ///< IMU parameters.

  // Local parameterizations
  std::shared_ptr<::ceres::LocalParameterization>
      extrinsicLocalParameterization_;
  std::shared_ptr<::ceres::LocalParameterization> poseLocalParameterization_;
  std::shared_ptr<okvis::ceres::LocalParamizationAdditionalInterfaces>
      landmarkParameterizationPtr_;
  std::shared_ptr<okvis::ceres::LocalParamizationAdditionalInterfaces>
      normalVectorParameterizationPtr_;

  std::shared_ptr<::ceres::LossFunction> lossFunctionPtr_;

  // hold the cost function pointers lest they are released.
  std::vector<std::shared_ptr<::ceres::CostFunction>> costFunctions_;
  std::vector<std::vector<std::shared_ptr<::ceres::CostFunction>>> keypointCostFunctions_; // observations for multiple cameras
  std::vector<std::vector<::ceres::ResidualBlockId>> keypointResidualIds_;  // It one to one maps to keypointCostFunctions_.

  std::unordered_map<::ceres::ResidualBlockId, std::vector<double*>> residualIdToParameterBlocks_;

  std::vector<uint64_t> vertexIdContainer;
  std::vector<CameraBlockInfo> CameraBlockIdContainer;
  std::vector<uint64_t> landmarkIdContainer;
  uint64_t gravityDirectionBlockId_;

  swift_vio::InitialNavState initialNavState_;

  std::vector<ReprojectionStatistics> reprojectionStats_;  // for multiple cameras.
  std::vector<HistogramType> reprojectionErrorHist_;
  std::vector<std::vector<double>> errors_;
  std::vector<std::vector<double>> normalizedErrors_;
};

typedef okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>
    DistortedPinholeCameraGeometry;
static const int kDistortionDim =
    DistortedPinholeCameraGeometry::distortion_t::NumDistortionIntrinsics;
static const int kProjIntrinsicDim =
    swift_vio::ProjIntrinsic_FXY_CXY::kNumParams;

void outputHistogram(const std::string &featureHistFile,
                     const std::vector<HistogramType> &hist);

} // namespace swift_vio
#endif // SWIFT_VIO_VI_BUNDLE_ADJUSTMENT_H
