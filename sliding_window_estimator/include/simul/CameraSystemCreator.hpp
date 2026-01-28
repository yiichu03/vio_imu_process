#ifndef INCLUDE_SWIFT_VIO_CAMERA_SYSTEM_CREATOR_HPP_
#define INCLUDE_SWIFT_VIO_CAMERA_SYSTEM_CREATOR_HPP_

#include <memory>
#include <string>

#include <vio/Sample.h>

#include <okvis/cameras/CameraBase.hpp>
#include <swift_vio/CameraRig.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>

#include <okvis/Parameters.hpp>
#include <swift_vio/memory.h>

namespace simul {
// Sideways orientation is geometrically favorable for motion estimation.
enum CameraOrientation {
  Forward = 0,
  Backward,
  Left,
  Right,
  SIZE_OF_CAMERAORIENTATION
};

bool EnumFromString(std::string description, CameraOrientation *e);
std::string EnumToString(CameraOrientation e);
inline std::ostream &operator<<(std::ostream &s, CameraOrientation e) {
  return s << EnumToString(e);
}

enum SimCameraModelType {
  EUROC = 0,
  EUROC88,
  TUMVI512,
  FromConfigYaml,
  SIZE_OF_SIMCAMERAMODELTYPE
};

bool EnumFromString(std::string description, SimCameraModelType *e);
std::string EnumToString(SimCameraModelType e);
inline std::ostream &operator<<(std::ostream &s, SimCameraModelType e) {
  return s << EnumToString(e);
}

/**
 * @brief create_T_BC
 * The body frame is forward-left-up. Relative to the camera,
 * the camera frame is right-down-forward.
 * @param orientationId: forward, backward, left, right
 * @return T_BC
 */
okvis::kinematics::Transformation create_T_BC(CameraOrientation orientationId,
                                              int camIdx);

/**
 * @brief createNoisyCameraSystem add noise to a reference camera system.
 * @warning Currently no noise is added to camera relative orientation.
 * @param cameraSystem reference system
 * @param cameraNoiseParams
 * @return noisy camera system.
 */
std::shared_ptr<swift_vio::CameraRig> createNoisyCameraSystem(
    std::shared_ptr<const swift_vio::CameraRig> cameraSystem,
    const okvis::CameraNoiseParameters &cameraNoiseParams,
    const std::vector<std::string>& extrinsicRepNames);

/**
 * @brief loadCameraSystemYaml
 * @param camImuChainYaml in OpenCV YAML 1.0 format of the output of kalibr camera imu calibration.
 * For a matrix attribute say T_cam_imu, two forms are supported in this function.
 * 1. seq form
 *   T_cam_imu:
 *     [-0.9995110484978581, 0.030299116376600627, -0.0077218830287333565, -0.053697434688869734,
 *      0.008104079263822521, 0.012511643720192351, -0.9998888851620987, -0.046131737923635924,
 *      -0.030199136245891378, -0.9994625667418545, -0.012751072573940885, -0.07149261284195751,
 *      0.0, 0.0, 0.0, 1.0]
 * 2. mat form
 *   T_cam_imu: !!opencv-matrix
 *     rows: 4
 *     cols: 4
 *     dt: d
 *     data: [-0.9995110484978581, 0.030299116376600627, -0.0077218830287333565, -0.053697434688869734,
 *            0.008104079263822521, 0.012511643720192351, -0.9998888851620987, -0.046131737923635924,
 *            -0.030199136245891378, -0.9994625667418545, -0.012751072573940885, -0.07149261284195751,
 *            0.0, 0.0, 0.0, 1.0]
 * @return
 */
std::shared_ptr<swift_vio::CameraRig>
loadCameraSystemYaml(const std::string &camImuChainYaml);


struct CameraProjectionIntrinsics {
  int imageWidth;
  int imageHeight;
  double focalLengthU;
  double focalLengthV;
  double imageCenterU;
  double imageCenterV;

  CameraProjectionIntrinsics() {}

  CameraProjectionIntrinsics(int w, int h, double fx, double fy, double cx,
                             double cy)
      : imageWidth(w), imageHeight(h), focalLengthU(fx), focalLengthV(fy),
        imageCenterU(cx), imageCenterV(cy) {}
};


struct CameraSpatioTemporalParameters {
  Eigen::Matrix4d T_imu_cam;
  double timeshift_cam_imu;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraSpatioTemporalParameters() {}

  CameraSpatioTemporalParameters(const Eigen::Matrix4d& T_IC, double timeshift) :
      T_imu_cam(T_IC), timeshift_cam_imu(timeshift) {}
};


/**
 * @brief loadInitialCameraImuParams
 * The initial value yaml contains spatiotemporal parameters for each camera and the gravity vector.
 * @param initialValueYaml
 */
void loadInitialCameraImuParams(
    const std::string initialValueYaml,
    Eigen::AlignedVector<CameraSpatioTemporalParameters> *stParams,
    Eigen::Vector3d *gravity);


class CameraSystemCreator {
 public:
  CameraSystemCreator(SimCameraModelType cameraModelId,
                      CameraOrientation cameraOrientationId,
                      const std::string projIntrinsicRep,
                      double td, double tr);

  /**
   * @brief createNominalCameraSystem
   * @param cameraSystem
   * @param distortionType
   * @param extrinsicRepNames size should be no greater than 4.
   * @return
   */
  void createNominalCameraSystem(
      std::shared_ptr<swift_vio::CameraRig> *cameraSystem,
      okvis::cameras::DistortionType distortionType,
      const std::vector<std::string>& extrinsicRepNames);

  static std::shared_ptr<okvis::cameras::CameraBase>
  createTestCameraGeometry(okvis::cameras::DistortionType distortionType);

private:
  std::shared_ptr<okvis::cameras::CameraBase>
  createCameraGeometry(SimCameraModelType cameraModelId,
                       okvis::cameras::DistortionType distortionType);

  static const std::map<SimCameraModelType, CameraProjectionIntrinsics>
      cameraModels_;

  static std::map<SimCameraModelType, CameraProjectionIntrinsics>
  initCameraModels() {
    std::map<SimCameraModelType, CameraProjectionIntrinsics> models;
    models[SimCameraModelType::TUMVI512] = CameraProjectionIntrinsics(
        512, 512, 190, 190, 256, 256); // diagonal FOV 125 deg.
    models[SimCameraModelType::EUROC88] = CameraProjectionIntrinsics(
        752, 480, 460, 460, 376, 240); // diagonal FOV 88 deg.
    models[SimCameraModelType::EUROC] = CameraProjectionIntrinsics(
        752, 480, 350, 360, 378, 238); // diagonal FOV 103 deg.
    return models;
   }

   const SimCameraModelType cameraModelId_;
   const CameraOrientation cameraOrientationId_;
   const std::string projIntrinsicRep_;
   const double timeOffset_;
   const double readoutTime_;
};
} // namespace simul

#endif // INCLUDE_SWIFT_VIO_CAMERA_SYSTEM_CREATOR_HPP_
