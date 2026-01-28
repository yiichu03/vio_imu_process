#include <simul/CameraSystemCreator.hpp>

#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/FovDistortion.hpp>
#include <okvis/cameras/NoDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/CameraModelSwitch.hpp>
#include <okvis/VioParametersReader.hpp>

#include <glog/logging.h>
#include <opencv2/core/eigen.hpp>
#include <swift_vio/memory.h>

namespace simul {
const std::map<SimCameraModelType, CameraProjectionIntrinsics>
    CameraSystemCreator::cameraModels_ =
        CameraSystemCreator::initCameraModels();

bool EnumFromString(std::string description, CameraOrientation *e) {
  std::transform(description.begin(), description.end(), description.begin(),
                 ::toupper);
  std::unordered_map<std::string, CameraOrientation> descriptionToId{
      {"FORWARD", CameraOrientation::Forward},
      {"BACKWARD", CameraOrientation::Backward},
      {"LEFT", CameraOrientation::Left},
      {"RIGHT", CameraOrientation::Right}};
  auto iter = descriptionToId.find(description);
  if (iter == descriptionToId.end()) {
    *e = CameraOrientation::Forward;
    return false;
  } else {
    *e = iter->second;
  }
  return true;
}

std::string EnumToString(CameraOrientation e) {
  const std::string names[] = {"Forward", "Backward",
                               "Left", "Right"};
  return names[static_cast<int>(e)];
}

bool EnumFromString(std::string description, SimCameraModelType *e) {
  std::transform(description.begin(), description.end(), description.begin(),
                 ::toupper);
  std::unordered_map<std::string, SimCameraModelType> descriptionToId{
      {"EUROC", SimCameraModelType::EUROC},
      {"EUROC88", SimCameraModelType::EUROC88},
      {"TUM512", SimCameraModelType::TUMVI512},
      {"FROMCONFIGYAML", SimCameraModelType::FromConfigYaml}};
  auto iter = descriptionToId.find(description);
  if (iter == descriptionToId.end()) {
    *e = SimCameraModelType::EUROC;
    return false;
  } else {
    *e = iter->second;
  }
  return true;
}

std::string EnumToString(SimCameraModelType e) {
  const std::string names[] = {"EUROC", "EUROC88",
                               "TUMVI512", "FromConfigYaml"};
  return names[static_cast<int>(e)];
}

okvis::kinematics::Transformation create_T_BC(CameraOrientation orientationId, int camIdx) {
  Eigen::Matrix4d matT_SC0;
  switch (orientationId) {
    case CameraOrientation::Backward: // Backward motion: The camera faces backward when the device goes straight forward.
      matT_SC0 << 0, 0, -1, 0,
                  1, 0, 0, 0,
                  0, -1, 0, 0,
                  0, 0, 0, 1;
      break;
    case CameraOrientation::Left: // Sideways motion: The camera faces left if the device goes straight forward.
      matT_SC0 << 1, 0, 0, 0,
                  0, 0, 1, 0,
                  0, -1, 0, 0,
                  0, 0, 0, 1;
      break;
    case CameraOrientation::Right: // Sideways motion: The camera faces right if the device goes straight forward.
      matT_SC0 << -1, 0, 0, 0,
                  0, 0, -1, 0,
                  0, -1, 0, 0,
                  0, 0, 0, 1;
      break;
    case CameraOrientation::Forward: // Forward motion: The camera faces forward when the device goes straight forward.
    default:
      matT_SC0 << 0, 0, 1, 0,
                 -1, 0, 0, 0,
                 0, -1, 0, 0,
                 0, 0, 0, 1;
      break;
  }

  okvis::kinematics::Transformation T_SC0(matT_SC0);
  Eigen::AlignedVector<okvis::kinematics::Transformation> T_C0C(4, okvis::kinematics::Transformation::Identity());

  // TODO(jhuai): Frame readout times converged poorly in the below setup with a small relative tilt,
  // but converged well with a large relative rotation or no relative rotation.
//  T_C0C[1].set(Eigen::Vector3d(0.2, 0.0, 0.0), okvis::kinematics::randomRotation(5 * M_PI / 180) * T_C0C[0].q());
  T_C0C[1].set(Eigen::Vector3d(0.2, 0.0, 0.0),
               Eigen::Quaterniond(Eigen::AngleAxisd(10 * M_PI / 180,
                                                    Eigen::Vector3d(0, 1, 0))) *
                   T_C0C[0].q());
  T_C0C[2].set(Eigen::Vector3d(0.2, 0.2, 0), okvis::kinematics::randomRotation(5 * M_PI / 180) * T_C0C[0].q());
  T_C0C[3].set(Eigen::Vector3d(0.0, 0.2, 0), okvis::kinematics::randomRotation(5 * M_PI / 180) * T_C0C[0].q());
  return T_SC0 * T_C0C[camIdx];
}

std::shared_ptr<swift_vio::CameraRig> createNoisyCameraSystem(
    std::shared_ptr<const swift_vio::CameraRig> cameraSystem,
    const okvis::CameraNoiseParameters &cameraNoiseParams,
    const std::vector<std::string>& extrinsicRepNames) {
  std::shared_ptr<swift_vio::CameraRig> noisyCameraSystem(
      new swift_vio::CameraRig());
  for (size_t camIdx = 0u; camIdx < cameraSystem->numCameras(); ++camIdx) {
    std::shared_ptr<const okvis::kinematics::Transformation> ref_T_SC =
        cameraSystem->getCameraExtrinsicPtr(camIdx);
    std::shared_ptr<okvis::kinematics::Transformation> T_SC_noisy;

    if (extrinsicRepNames[camIdx].compare("P_BC_Q_BC") == 0) {
      Eigen::Matrix<double, 6, 1> delta;
      for (int jack = 0; jack < 3; ++jack) {
        delta[jack] =
            vio::gauss_rand(0, cameraNoiseParams.sigma_absolute_translation);
      }
      for (int jack = 3; jack < 6; ++jack) {
        delta[jack] =
            vio::gauss_rand(0, cameraNoiseParams.sigma_absolute_orientation);
      }
      okvis::kinematics::Transformation T_SC(*ref_T_SC);
      T_SC.oplus(delta);
      T_SC_noisy.reset(new okvis::kinematics::Transformation(T_SC));
    } else if (extrinsicRepNames[camIdx].compare("P_CB") == 0) {
      Eigen::Vector3d p_CBNoise;
      for (int jack = 0; jack < 3; ++jack) {
        p_CBNoise[jack] =
            vio::gauss_rand(0, cameraNoiseParams.sigma_absolute_translation);
      }
      T_SC_noisy.reset(new okvis::kinematics::Transformation(
          ref_T_SC->r() - ref_T_SC->C() * p_CBNoise, ref_T_SC->q()));

    } else {
      LOG(ERROR) << "We do not support creating noisy " << extrinsicRepNames[camIdx]
                 << "!";
    }

    std::shared_ptr<const okvis::cameras::CameraBase> refCameraGeometry =
        cameraSystem->cameraGeometry(camIdx);
    Eigen::VectorXd intrinsics;
    refCameraGeometry->getIntrinsics(intrinsics);
    std::shared_ptr<okvis::cameras::CameraBase> cameraGeometry;

    Eigen::Matrix<double, 4, 1> fcNoise = vio::Sample::gaussian(1, 4);
    fcNoise.head<2>() *= cameraNoiseParams.sigma_focal_length;
    fcNoise.tail<2>() *= cameraNoiseParams.sigma_principal_point;

    Eigen::Matrix<double, -1, 1> kpNoise(
        cameraNoiseParams.sigma_distortion.size() > 4u ? cameraNoiseParams.sigma_distortion.size() : 4u);
    kpNoise.setZero();
    for (size_t jack = 0; jack < cameraNoiseParams.sigma_distortion.size();
         ++jack) {
      kpNoise[jack] =
          vio::Sample::gaussian(1) * cameraNoiseParams.sigma_distortion[jack];
    }
    switch (cameraSystem->distortionType(camIdx)) {
    case okvis::cameras::DistortionType::Fov:
      cameraGeometry.reset(
          new okvis::cameras::PinholeCamera<okvis::cameras::FovDistortion>(
              refCameraGeometry->imageWidth(), refCameraGeometry->imageHeight(),
              intrinsics[0] + fcNoise[0],
              intrinsics[1] + fcNoise[1],
              intrinsics[2] + fcNoise[2],
              intrinsics[3] + fcNoise[3],
              okvis::cameras::FovDistortion(intrinsics[4] +
                                            kpNoise[0]),
              vio::gauss_rand(refCameraGeometry->imageDelay(),
                              cameraNoiseParams.sigma_td),
              std::fabs(vio::gauss_rand(refCameraGeometry->readoutTime(),
                                        cameraNoiseParams.sigma_tr))));
      break;

    case okvis::cameras::DistortionType::Equidistant:
      cameraGeometry.reset(new okvis::cameras::PinholeCamera<
                           okvis::cameras::EquidistantDistortion>(
          refCameraGeometry->imageWidth(), refCameraGeometry->imageHeight(),
          intrinsics[0] + fcNoise[0],
          intrinsics[1] + fcNoise[1],
          intrinsics[2] + fcNoise[2],
          intrinsics[3] + fcNoise[3],
          okvis::cameras::EquidistantDistortion(
              intrinsics[4] + kpNoise[0],
              intrinsics[5] + kpNoise[1],
              intrinsics[6] + kpNoise[2],
              intrinsics[7] + kpNoise[3]),
          vio::gauss_rand(refCameraGeometry->imageDelay(),
                          cameraNoiseParams.sigma_td),
          std::fabs(vio::gauss_rand(refCameraGeometry->readoutTime(),
                                    cameraNoiseParams.sigma_tr))));
      break;
    case okvis::cameras::DistortionType::No:
      cameraGeometry.reset(
          new okvis::cameras::PinholeCamera<okvis::cameras::NoDistortion>(
              refCameraGeometry->imageWidth(), refCameraGeometry->imageHeight(),
              intrinsics[0] + fcNoise[0],
              intrinsics[1] + fcNoise[1],
              intrinsics[2] + fcNoise[2],
              intrinsics[3] + fcNoise[3],
              okvis::cameras::NoDistortion(),
              vio::gauss_rand(refCameraGeometry->imageDelay(),
                              cameraNoiseParams.sigma_td),
              std::fabs(vio::gauss_rand(refCameraGeometry->readoutTime(),
                                        cameraNoiseParams.sigma_tr))));
      break;
    case okvis::cameras::DistortionType::Eucm:
      cameraGeometry.reset(new okvis::cameras::EUCM(
          refCameraGeometry->imageWidth(), refCameraGeometry->imageHeight(),
          intrinsics[0] + fcNoise[0],
          intrinsics[1] + fcNoise[1],
          intrinsics[2] + fcNoise[2],
          intrinsics[3] + fcNoise[3],
          intrinsics[4] + kpNoise[0],
          intrinsics[5] + kpNoise[1],
          vio::gauss_rand(refCameraGeometry->imageDelay(),
                          cameraNoiseParams.sigma_td),
          std::fabs(vio::gauss_rand(refCameraGeometry->readoutTime(),
                                    cameraNoiseParams.sigma_tr))));
      break;
    case okvis::cameras::DistortionType::RadialTangential:
    default:
      cameraGeometry.reset(new okvis::cameras::PinholeCamera<
                           okvis::cameras::RadialTangentialDistortion>(
          refCameraGeometry->imageWidth(), refCameraGeometry->imageHeight(),
          intrinsics[0] + fcNoise[0],
          intrinsics[1] + fcNoise[1],
          intrinsics[2] + fcNoise[2],
          intrinsics[3] + fcNoise[3],
          okvis::cameras::RadialTangentialDistortion(
              intrinsics[4] + kpNoise[0],
              intrinsics[5] + kpNoise[1],
              intrinsics[6] + kpNoise[2],
              intrinsics[7] + kpNoise[3]),
          vio::gauss_rand(refCameraGeometry->imageDelay(),
                          cameraNoiseParams.sigma_td),
          std::fabs(vio::gauss_rand(refCameraGeometry->readoutTime(),
                                    cameraNoiseParams.sigma_tr))));
      break;
    }
    noisyCameraSystem->addCamera(T_SC_noisy, cameraGeometry,
                                 cameraSystem->getProjectionIntrinsicRepId(camIdx),
                                 cameraSystem->getExtrinsicRepId(camIdx));
  }
  return noisyCameraSystem;
}

std::shared_ptr<swift_vio::CameraRig>
loadCameraSystemYaml(const std::string &camChainYaml) {
  cv::FileStorage file(camChainYaml, cv::FileStorage::READ);
  OKVIS_ASSERT_TRUE(std::runtime_error, file.isOpened(),
                    "Could not open config file: " << camChainYaml);
  int camIdx = 0;
  for (; camIdx < 10; ++camIdx) {
    const cv::FileNode &camNode = file["cam" + std::to_string(camIdx)];
    if (camNode.isMap() &&
        camNode["resolution"].isSeq() && camNode["resolution"].size() == 2 &&
        camNode["distortion_coeffs"].isSeq() &&
        camNode["distortion_coeffs"].size() >= 1 &&
        camNode["distortion_model"].isString() &&
        camNode["intrinsics"].isSeq() && camNode["intrinsics"].size() == 4) {
    } else {
      break;
    }
  }
  int numCameras = camIdx;
  LOG(INFO) << "Found calibration in configuration file for #camera "
            << numCameras;

  std::shared_ptr<swift_vio::CameraRig> cameraSystem(
      new swift_vio::CameraRig());
  for (camIdx = 0; camIdx < numCameras; ++camIdx) {
    const cv::FileNode &camNode = file["cam" + std::to_string(camIdx)];
    okvis::kinematics::Transformation T_CS;
    okvis::kinematics::Transformation T_SC;
    Eigen::MatrixXd T_eigen(4, 4);
    T_eigen.setIdentity();
    bool status1 =
        okvis::parseMatrixInYaml(camNode["T_cam_imu"], &T_eigen, 4, 4);
    if (!status1) {
      bool status2 =
          okvis::parseMatrixInYaml(camNode["T_imu_cam"], &T_eigen, 4, 4);
      if (status2) {
        T_SC.set(T_eigen);
        T_CS = T_SC.inverse();
      } else {
        LOG(WARNING) << "Unable to parse T_cam_imu or T_imu_cam for camera "
                     << camIdx;
      }
    } else {
      T_CS.set(T_eigen);
      T_SC = T_CS.inverse();
    }

    const cv::FileNode &resolutionNode = camNode["resolution"];
    std::vector<int> imageDimension{static_cast<int>(resolutionNode[0]),
                                    static_cast<int>(resolutionNode[1])};

    const cv::FileNode &intrinsicsNode = camNode["intrinsics"];
    std::vector<double> projectionIntrinsics(intrinsicsNode.size());
    for (size_t i = 0u; i < intrinsicsNode.size(); ++i) {
      projectionIntrinsics[i] = intrinsicsNode[i];
    }
    const cv::FileNode &distortionCoeffsNode = camNode["distortion_coeffs"];
    std::vector<double> distortionCoeffs(distortionCoeffsNode.size());
    for (size_t i = 0u; i < distortionCoeffsNode.size(); ++i) {
      distortionCoeffs[i] = distortionCoeffsNode[i];
    }
    std::string distortionType = camNode["distortion_model"];
    double imageDelaySecs(0.0);
    if (camNode["timeshift_cam_imu"].isReal()) {
      camNode["timeshift_cam_imu"] >> imageDelaySecs;
    }
    double readoutTimeSecs;
    camNode["line_delay_nanoseconds"] >> readoutTimeSecs;
    readoutTimeSecs *= imageDimension[1];
    readoutTimeSecs /= 1000000000;
    std::string extrinsicRepName;
    if (camNode["extrinsic_rep"].isString()) {
      extrinsicRepName =
          static_cast<std::string>(camNode["extrinsic_rep"]);
    }

    std::string projIntrinsicRepName;
    if (camNode["projection_intrinsic_rep"].isString()) {
      projIntrinsicRepName =
          static_cast<std::string>(camNode["projection_intrinsic_rep"]);
    }

    std::shared_ptr<okvis::kinematics::Transformation> T_SC_ptr(
        new okvis::kinematics::Transformation(T_SC.r(), T_SC.q().normalized()));

    std::transform(distortionType.begin(), distortionType.end(),
                   distortionType.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (strcmp(distortionType.c_str(), "equidistant") == 0) {
      cameraSystem->addCamera(
          T_SC_ptr,
          std::shared_ptr<okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion>(
                  imageDimension[0], imageDimension[1], projectionIntrinsics[0],
                  projectionIntrinsics[1], projectionIntrinsics[2],
                  projectionIntrinsics[3],
                  okvis::cameras::EquidistantDistortion(
                      distortionCoeffs[0], distortionCoeffs[1],
                      distortionCoeffs[2], distortionCoeffs[3]),
                  imageDelaySecs, readoutTimeSecs
                  /*, id ?*/)),
          projIntrinsicRepName,
          extrinsicRepName);
      std::stringstream s;
      s << T_SC.T();
      LOG(INFO) << "Equidistant pinhole camera " << camIdx << " with T_SC=\n"
                << s.str();
    } else if (strcmp(distortionType.c_str(), "radialtangential") == 0 ||
               strcmp(distortionType.c_str(), "plumb_bob") == 0) {
      cameraSystem->addCamera(
          T_SC_ptr,
          std::shared_ptr<okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion>(
                  imageDimension[0], imageDimension[1], projectionIntrinsics[0],
                  projectionIntrinsics[1], projectionIntrinsics[2],
                  projectionIntrinsics[3],
                  okvis::cameras::RadialTangentialDistortion(
                      distortionCoeffs[0], distortionCoeffs[1],
                      distortionCoeffs[2], distortionCoeffs[3]),
                  imageDelaySecs, readoutTimeSecs
                  /*, id ?*/)),
          projIntrinsicRepName,
          extrinsicRepName);
      std::stringstream s;
      s << T_SC.T();
      LOG(INFO) << "Radial tangential pinhole camera " << camIdx
                << " with T_SC=\n"
                << s.str();
    } else if (strcmp(distortionType.c_str(), "radialtangential8") == 0 ||
               strcmp(distortionType.c_str(), "plumb_bob8") == 0) {
      cameraSystem->addCamera(
          T_SC_ptr,
          std::shared_ptr<okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion8>(
                  imageDimension[0], imageDimension[1], projectionIntrinsics[0],
                  projectionIntrinsics[1], projectionIntrinsics[2],
                  projectionIntrinsics[3],
                  okvis::cameras::RadialTangentialDistortion8(
                      distortionCoeffs[0], distortionCoeffs[1],
                      distortionCoeffs[2], distortionCoeffs[3],
                      distortionCoeffs[4], distortionCoeffs[5],
                      distortionCoeffs[6], distortionCoeffs[7]),
                  imageDelaySecs, readoutTimeSecs
                  /*, id ?*/)),
          projIntrinsicRepName,
          extrinsicRepName);
      std::stringstream s;
      s << T_SC.T();
      LOG(INFO) << "Radial tangential 8 pinhole camera " << camIdx
                << " with T_SC=\n"
                << s.str();
    } else if (strcmp(distortionType.c_str(), "fov") == 0) {
      std::shared_ptr<okvis::cameras::CameraBase> camPtr(
          new okvis::cameras::PinholeCamera<okvis::cameras::FovDistortion>(
              imageDimension[0], imageDimension[1], projectionIntrinsics[0],
              projectionIntrinsics[1], projectionIntrinsics[2],
              projectionIntrinsics[3],
              okvis::cameras::FovDistortion(distortionCoeffs[0]),
              imageDelaySecs, readoutTimeSecs
              /*, id ?*/));
      Eigen::VectorXd intrin(5);
      intrin[0] = projectionIntrinsics[0];
      intrin[1] = projectionIntrinsics[1];
      intrin[2] = projectionIntrinsics[2];
      intrin[3] = projectionIntrinsics[3];
      intrin[4] = distortionCoeffs[0];
      camPtr->setIntrinsics(intrin);
      cameraSystem->addCamera(T_SC_ptr, camPtr,
                              projIntrinsicRepName, extrinsicRepName);
      std::stringstream s;
      s << T_SC.T();
      LOG(INFO) << "FOV pinhole camera " << camIdx << " with Omega "
                << distortionCoeffs[0] << " with T_SC=\n"
                << s.str();
    } else if (strcmp(distortionType.c_str(), "EUCM") == 0 || strcmp(distortionType.c_str(), "eucm") == 0) {
      std::shared_ptr<okvis::cameras::CameraBase> camPtr(
          new okvis::cameras::EUCM(
              imageDimension[0], imageDimension[1], projectionIntrinsics[0],
              projectionIntrinsics[1], projectionIntrinsics[2],
              projectionIntrinsics[3],
              distortionCoeffs[0], distortionCoeffs[1],
              imageDelaySecs, readoutTimeSecs
              /*, id ?*/));
      cameraSystem->addCamera(T_SC_ptr, camPtr,
                              projIntrinsicRepName, extrinsicRepName);
      std::stringstream s;
      s << T_SC.T();
      LOG(INFO) << "Extended unified camera " << camIdx << " with alpah "
                << distortionCoeffs[0] << " beta " << distortionCoeffs[1]
                << " with T_SC=\n" << s.str();
    } else {
      LOG(ERROR) << "unrecognized distortion type " << distortionType;
    }
  }
  file.release();
  cameraSystem->computeOverlaps();
  return cameraSystem;
}

void loadInitialCameraImuParams(
    const std::string initialValueYaml,
    Eigen::AlignedVector<CameraSpatioTemporalParameters> *stParams,
    Eigen::Vector3d *gravity) {
  cv::FileStorage file(initialValueYaml, cv::FileStorage::READ);
  OKVIS_ASSERT_TRUE(std::runtime_error, file.isOpened(),
                    "Could not open config file: " << initialValueYaml);
  int camIdx = 0;
  for (; camIdx < 10; ++camIdx) {
    const cv::FileNode &camNode = file["cam" + std::to_string(camIdx)];
    if (camNode.isMap() && !camNode["T_imu_cam"].empty() &&
        camNode["timeshift_cam_imu"].isReal()) {
    } else {
      break;
    }
  }
  int numCameras = camIdx;
  LOG(INFO) << "Found initial spatiotemporal parameters for #camera "
            << numCameras;
  stParams->resize(numCameras);
  for (camIdx = 0; camIdx < numCameras; ++camIdx) {
    const cv::FileNode &camNode = file["cam" + std::to_string(camIdx)];
    Eigen::MatrixXd T_eigen(4, 4);
    T_eigen.setIdentity();
    bool status = okvis::parseMatrixInYaml(camNode["T_imu_cam"], &T_eigen, 4, 4);
    LOG_IF(INFO, !status) << "Unable to parse T_cam_imu for camera " << camIdx;

    double imageDelaySecs(0.0);
    camNode["timeshift_cam_imu"] >> imageDelaySecs;
    stParams->at(camIdx).T_imu_cam = T_eigen;
    stParams->at(camIdx).timeshift_cam_imu = imageDelaySecs;
  }

  if (file["gravity_in_target"].isSeq()) {
    for (int i = 0; i < 3; ++i) {
      (*gravity)[i] = file["gravity_in_target"][i];
    }
  }
}

CameraSystemCreator::CameraSystemCreator(SimCameraModelType cameraModelId,
                    CameraOrientation cameraOrientationId,
                    const std::string projIntrinsicRep,
                    double td, double tr)
    : cameraModelId_(cameraModelId),
      cameraOrientationId_(cameraOrientationId),
      projIntrinsicRep_(projIntrinsicRep),
      timeOffset_(td),
      readoutTime_(tr) {}

void CameraSystemCreator::createNominalCameraSystem(
    std::shared_ptr<swift_vio::CameraRig> *cameraSystem,
    okvis::cameras::DistortionType distortionType,
    const std::vector<std::string>& extrinsicRepNames) {
  size_t numCameras = extrinsicRepNames.size();
  cameraSystem->reset(new swift_vio::CameraRig());
  for (size_t i = 0; i < numCameras; ++i) {
    std::shared_ptr<okvis::kinematics::Transformation> T_SC(
        new okvis::kinematics::Transformation(
            create_T_BC(cameraOrientationId_, i)));

    std::shared_ptr<okvis::cameras::CameraBase> cameraGeometry =
        createCameraGeometry(cameraModelId_, distortionType);
    (*cameraSystem)
        ->addCamera(T_SC, cameraGeometry, projIntrinsicRep_,
                    extrinsicRepNames[i]);
  }
  (*cameraSystem)->computeOverlaps();
}

std::shared_ptr<okvis::cameras::CameraBase>
CameraSystemCreator::createTestCameraGeometry(
    okvis::cameras::DistortionType distortionType) {

#define DISTORTION_MODEL_CASE(camera_geometry_t)                               \
  { return camera_geometry_t::createTestObject(); }

  switch (distortionType) { DISTORTION_MODEL_SWITCH_CASES }

#undef DISTORTION_MODEL_CASE
}

std::shared_ptr<okvis::cameras::CameraBase>
CameraSystemCreator::createCameraGeometry(
    SimCameraModelType cameraModelId,
    okvis::cameras::DistortionType distortionType) {
  std::shared_ptr<okvis::cameras::CameraBase> cameraGeometry;
  switch (distortionType) {
  case okvis::cameras::DistortionType::RadialTangential: {
    cameraGeometry.reset(new okvis::cameras::PinholeCamera<
                         okvis::cameras::RadialTangentialDistortion>(
        cameraModels_.at(cameraModelId).imageWidth,
        cameraModels_.at(cameraModelId).imageHeight,
        cameraModels_.at(cameraModelId).focalLengthU,
        cameraModels_.at(cameraModelId).focalLengthV,
        cameraModels_.at(cameraModelId).imageCenterU,
        cameraModels_.at(cameraModelId).imageCenterV,
        okvis::cameras::RadialTangentialDistortion(0.0, 0.0, 0.0, 0.0),
        timeOffset_, readoutTime_));
    break;
  }
  case okvis::cameras::DistortionType::Equidistant: {
    cameraGeometry.reset(new okvis::cameras::PinholeCamera<
                         okvis::cameras::EquidistantDistortion>(
        cameraModels_.at(cameraModelId).imageWidth,
        cameraModels_.at(cameraModelId).imageHeight,
        cameraModels_.at(cameraModelId).focalLengthU,
        cameraModels_.at(cameraModelId).focalLengthV,
        cameraModels_.at(cameraModelId).imageCenterU,
        cameraModels_.at(cameraModelId).imageCenterV,
        okvis::cameras::EquidistantDistortion(0.0, 0.0, 0.0, 0.0), timeOffset_,
        readoutTime_));
    break;
  }
  case okvis::cameras::DistortionType::RadialTangential8: {
    cameraGeometry.reset(new okvis::cameras::PinholeCamera<
                         okvis::cameras::RadialTangentialDistortion8>(
        cameraModels_.at(cameraModelId).imageWidth,
        cameraModels_.at(cameraModelId).imageHeight,
        cameraModels_.at(cameraModelId).focalLengthU,
        cameraModels_.at(cameraModelId).focalLengthV,
        cameraModels_.at(cameraModelId).imageCenterU,
        cameraModels_.at(cameraModelId).imageCenterV,
        okvis::cameras::RadialTangentialDistortion8(0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0),
        timeOffset_, readoutTime_));
    break;
  }
  case okvis::cameras::DistortionType::Fov: {
    cameraGeometry.reset(
        new okvis::cameras::PinholeCamera<okvis::cameras::FovDistortion>(
            cameraModels_.at(cameraModelId).imageWidth,
            cameraModels_.at(cameraModelId).imageHeight,
            cameraModels_.at(cameraModelId).focalLengthU,
            cameraModels_.at(cameraModelId).focalLengthV,
            cameraModels_.at(cameraModelId).imageCenterU,
            cameraModels_.at(cameraModelId).imageCenterV,
            okvis::cameras::FovDistortion(0.0), timeOffset_, readoutTime_));
    break;
  }
  case okvis::cameras::DistortionType::Eucm: {
    cameraGeometry.reset(
        new okvis::cameras::EUCM(cameraModels_.at(cameraModelId).imageWidth,
                                 cameraModels_.at(cameraModelId).imageHeight,
                                 cameraModels_.at(cameraModelId).focalLengthU,
                                 cameraModels_.at(cameraModelId).focalLengthV,
                                 cameraModels_.at(cameraModelId).imageCenterU,
                                 cameraModels_.at(cameraModelId).imageCenterV,
                                 0.6, 1.0, timeOffset_, readoutTime_));
    break;
  }
  default:
    OKVIS_THROW(std::runtime_error, "Unsupported distortion type.")
    break;
  }
  return cameraGeometry;
}

} // namespace simul
