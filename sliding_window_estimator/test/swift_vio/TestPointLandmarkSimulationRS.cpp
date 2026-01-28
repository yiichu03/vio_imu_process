#include <gtest/gtest.h>

#include <swift_vio/memory.h>

#include <simul/CameraSystemCreator.hpp>
#include <simul/curves.h>
#include <simul/PointLandmarkSimulationRS.hpp>

class PointLandmarkSimulationRSTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointLandmarkSimulationRSTest()
      : td(0.0), tr(28e-3), centralRowEpoch(20), mf(new swift_vio::MultiFrame()) {
    // create the trajectory
    double imuRate = 100;
    double gravity = 9.80;
    cst.reset(new simul::WavyCircle(imuRate, Eigen::Vector3d(0, 0, -gravity)));

    // create multiframe
    uint64_t id = 1000u;
    mf->setId(id);
    okvis::Time frameStamp = centralRowEpoch - okvis::Duration(td);
    mf->setTimestamp(frameStamp);
  }

  void checkProjection(bool withDistortion, bool centerRow, double eps) {
    // create camera system
    simul::CameraSystemCreator csc(simul::SimCameraModelType::EUROC,
                                   simul::CameraOrientation::Forward,
                                   "FXY_CXY", td, tr);
    if (withDistortion) {
      okvis::CameraNoiseParameters cameraNoiseParameters;
      cameraNoiseParameters.sigma_distortion =
          std::vector<double>{0.05, 0.01, 0.001, 0.001, 0.0001};
      cameraNoiseParameters.updateParameterStatus();
      csc.createNominalCameraSystem(
          &cameraSystem,
          okvis::cameras::DistortionType::RadialTangential,
          {"P_BC_Q_BC", "P_BC_Q_BC"});
      cameraSystem = simul::createNoisyCameraSystem(
          cameraSystem, cameraNoiseParameters, {"P_BC_Q_BC", "P_BC_Q_BC"});
    } else {
      csc.createNominalCameraSystem(
          &cameraSystem,
          okvis::cameras::DistortionType::RadialTangential,
          {"P_BC_Q_BC", "P_BC_Q_BC"});
    }

    mf->resetFrames(cameraSystem->numCameras());
    mf->setTimestamp(0u, mf->timestamp());

    size_t totalLandmarks = 5u;
    Eigen::AlignedVector<Eigen::Vector4d> homogeneousPoints;
    homogeneousPoints.reserve(totalLandmarks);
    Eigen::AlignedVector<Eigen::Vector2d> imagePoints;
    imagePoints.reserve(totalLandmarks);

    size_t checkCameraIndex = 1u;
    cameraGeometry = cameraSystem->cameraGeometry(checkCameraIndex);

    for (size_t j = 0; j < totalLandmarks; ++j) {
      // backproject to get the landmark position
      Eigen::Vector2d imagePoint = cameraGeometry->createRandomImagePoint();
      if (centerRow) {
        imagePoint[1] = cameraGeometry->imageHeight() * 0.5;
      }
      Eigen::Vector4d xy11;
      cameraGeometry->backProjectHomogeneous(imagePoint, &xy11);
      Eigen::Vector4d pCt = xy11;
      int max = 10;
      int min = 1;
      int range = max - min + 1;
      int distance = rand() % range + min;
      pCt.head<3>() *= distance;
      okvis::kinematics::Transformation T_WBt = cst->computeGlobalPose(
          centralRowEpoch +
          okvis::Duration(
              (imagePoint[1] / cameraGeometry->imageHeight() - 0.5) * tr));
      Eigen::Vector4d pW = T_WBt * cameraSystem->getCameraExtrinsic(checkCameraIndex) * pCt;
      homogeneousPoints.push_back(pW);
      imagePoints.push_back(imagePoint);
    }

    // project the landmark via RS model
    std::vector<std::vector<size_t>> frameLandmarkIndices;
    std::vector<std::unordered_map<size_t, size_t>> keypointIndices;
    simul::PointLandmarkSimulationRS::projectLandmarksToNFrame(
        homogeneousPoints, cst, centralRowEpoch, *cameraSystem, mf,
        &frameLandmarkIndices, &keypointIndices, nullptr);

    // check the predicted projection is exactly the original input
    for (auto iter : keypointIndices[checkCameraIndex]) {
      Eigen::Vector2d keypoint;
      mf->getKeypoint(checkCameraIndex, iter.second, keypoint);
      EXPECT_LT((keypoint - imagePoints[iter.first]).lpNorm<Eigen::Infinity>(), eps)
          << "rs projection " << keypoint.transpose() << " true point " << imagePoints[iter.first].transpose();
    }
  }

  void SetUp() override{};

 private:
  std::shared_ptr<simul::TrajectoryBase> cst;
  double td;
  double tr;
  std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry;
  std::shared_ptr<swift_vio::CameraRig> cameraSystem;
  okvis::Time centralRowEpoch;
  std::shared_ptr<swift_vio::MultiFrame> mf;
};

TEST_F(PointLandmarkSimulationRSTest, centerRow) {
  checkProjection(false, true, 1e-4);
}

TEST_F(PointLandmarkSimulationRSTest, randomRow) {
  checkProjection(false, false, 5e-3);
}

TEST_F(PointLandmarkSimulationRSTest, distortedCenterRow) {
  checkProjection(true, true, 1e-4);
}

TEST_F(PointLandmarkSimulationRSTest, distortedRandomRow) {
  checkProjection(true, false, 1e-2);
}
