#include <swift_vio/ViBundleAdjustment.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/bind.hpp>

#include <swift_vio/YamlHelpers.h>

#include <okvis/FrameTypedefs.hpp>
#include <okvis/IdProvider.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseError.hpp>

#include <swift_vio/ExtrinsicReps.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/timing/Timer.hpp>

#include <swift_vio/CameraRig.hpp>
#include <swift_vio/IoUtil.hpp>
#include <swift_vio/MultiFrame.hpp>
#include <swift_vio/ParallaxAnglePoint.hpp>
#include <swift_vio/PointLandmarkModels.hpp>
#include <swift_vio/ProjectionIntrinsicReps.h>
#include <swift_vio/ceres/CameraTimeParamBlock.hpp>
#include <swift_vio/ceres/EuclideanParamBlock.hpp>
#include <swift_vio/ceres/EuclideanParamBlockSized.hpp>
#include <swift_vio/ceres/ImuErrorWithGravity.hpp>
#include <swift_vio/ceres/NormalVectorParameterBlock.hpp>

#include <swift_vio/ceres/RsReprojectionError.hpp>

DEFINE_double(
    robust_loss_width, 5.0,
    "Divide the reprojection error by this factor when applying the robust "
    "loss function. We set this value kind of large by default because "
    "outliers in the detected target corners are rare and have been screened "
    "by a geometric check in corner extraction!");

namespace swift_vio {
ViBundleAdjustment::ViBundleAdjustment() {}

ViBundleAdjustment::~ViBundleAdjustment() {}

int ViBundleAdjustment::addCameraParameterStds(
    const okvis::CameraNoiseParameters &cameraNoiseParameters) {
  cameraNoiseParametersVec_.push_back(cameraNoiseParameters);
  return cameraNoiseParametersVec_.size() - 1;
}

void ViBundleAdjustment::addCameraSystem(
    std::shared_ptr<const CameraRig> cameras) {
  cameraSystem_ = cameras;

  keypointCostFunctions_.resize(cameraSystem_->numCameras());
  keypointResidualIds_.resize(cameraSystem_->numCameras());
}

int ViBundleAdjustment::addImu(const okvis::ImuParameters &imuParameters) {
  if (imuParametersVec_.size() > 0u) {
    LOG(ERROR) << "only one IMU currently supported";
    return -1;
  }
  imuParametersVec_.emplace_back(new okvis::ImuParameters(imuParameters));
  return imuParametersVec_.size() - 1;
}

void ViBundleAdjustment::buildOptimizationProblem(
    std::shared_ptr<simul::SimulatorBase> data, const VisualInertialBAOptions& baOptions) {
  ::ceres::Problem::Options problemOptions;
  problemOptions.local_parameterization_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.loss_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.cost_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_.reset(new ::ceres::Problem(problemOptions));

  extrinsicLocalParameterization_.reset(
      new swift_vio::PoseLocalParameterizationSimplified());
  poseLocalParameterization_.reset(
      new swift_vio::PoseLocalParameterizationSimplified());
  landmarkParameterizationPtr_.reset(
      new swift_vio::InverseDepthParameterization());
  normalVectorParameterizationPtr_.reset(
      new swift_vio::NormalVectorParameterization());
  lossFunctionPtr_.reset(new ::ceres::CauchyLoss(FLAGS_robust_loss_width));

  costFunctions_.reserve(200);
  data_ = data;

  addStateVariables(baOptions);
  addErrorTerms();
}

void ViBundleAdjustment::addStateVariables(const VisualInertialBAOptions& baOptions) {
  data_->rewind();
  okvis::Time lastNFrameTime(0);
  do {
    okvis::Time refNFrameTime = data_->currentTime();
    okvis::kinematics::Transformation T_WS_ref = data_->currentPose();
    Eigen::Vector3d v_WS_ref = data_->currentVelocity();
    okvis::ImuSensorReadings biasRef = data_->currentBiases();

    okvis::ImuMeasurementDeque imuSegment;
    if (lastNFrameTime == okvis::Time(0)) {
      imuSegment = data_->imuMeasurementsSinceLastNFrame();
    } else {
      double initialCamTimeOffset =
          cameraSystem_->cameraGeometry(0u)->imageDelay();
      imuSegment = data_->getImuMeasurements(
          lastNFrameTime + okvis::Duration(initialCamTimeOffset) -
              okvis::Duration(1.0),
          refNFrameTime + okvis::Duration(initialCamTimeOffset) +
              okvis::Duration(1.0));
    }
    // add speed and IMU biases
    okvis::SpeedAndBiases speedAndBias;
    speedAndBias.head<3>() = v_WS_ref;
    speedAndBias.tail<6>() = biasRef.toVector();

    uint64_t sbId = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::SpeedAndBiasParameterBlock>
        speedAndBiasParameterBlock(new okvis::ceres::SpeedAndBiasParameterBlock(
            speedAndBias, sbId, refNFrameTime));
    idToParameterBlocks_.emplace(sbId, speedAndBiasParameterBlock);
    problem_->AddParameterBlock(speedAndBiasParameterBlock->parameters(),
                                speedAndBiasParameterBlock->dimension());

    uint64_t vertexId = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock(
        new okvis::ceres::PoseParameterBlock(T_WS_ref, vertexId,
                                             refNFrameTime));
    idToParameterBlocks_.emplace(vertexId, poseParameterBlock);
    problem_->AddParameterBlock(poseParameterBlock->parameters(),
                                poseParameterBlock->dimension(),
                                poseLocalParameterization_.get());
    poseParameterBlock->setLocalParameterizationPtr(
        poseLocalParameterization_.get());
    vertexIdContainer.push_back(vertexId);

    vertexToState_.emplace(
        std::piecewise_construct, std::forward_as_tuple(vertexId),
        std::forward_as_tuple(refNFrameTime, vertexId, sbId,
                              std::shared_ptr<okvis::ImuMeasurementDeque>(
                                  new okvis::ImuMeasurementDeque(imuSegment))));
    lastNFrameTime = refNFrameTime;
  } while (data_->nextNFrame());

  // Add the gravity direction.
  uint64_t id = okvis::IdProvider::instance().newId();
  std::shared_ptr<okvis::ceres::NormalVectorParameterBlock>
      gravityDirectionBlock(new okvis::ceres::NormalVectorParameterBlock(
          imuParametersVec_.at(0)->gravityDirection(), id));
  problem_->AddParameterBlock(gravityDirectionBlock->parameters(),
                              gravityDirectionBlock->dimension(),
                              normalVectorParameterizationPtr_.get());
  idToParameterBlocks_.emplace(id, gravityDirectionBlock);
  gravityDirectionBlockId_ = id;
  VLOG(1) << "Initial gravity direction " << gravityDirectionBlock->estimate().transpose();
  addImuParameters();
  addCameraParameters(baOptions);
  addLandmarks();
}

void ViBundleAdjustment::addImuParameters() {
  // TODO: add IMU parameters including Mg, Ts, Ma.
  //  typedef okvis::ceres::DynamicImuError<swift_vio::Imu_BG_BA_MG_TS_MA> DynamicImuErrorT;
  // see TestDynamicImuError.cpp
  // They may be fixed by setVariableConstant().
}

void ViBundleAdjustment::addCameraParameters(const VisualInertialBAOptions& baOptions) {
  okvis::Time startTime = data_->currentTime();
  for (size_t i = 0; i < cameraSystem_->numCameras(); ++i) {
    // extrinsics
    okvis::kinematics::Transformation T_SC = cameraSystem_->getCameraExtrinsic(i);
    uint64_t ext_id = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::PoseParameterBlock>
        extrinsicsParameterBlockPtr(new okvis::ceres::PoseParameterBlock(
            T_SC, ext_id, startTime));
    idToParameterBlocks_.emplace(ext_id, extrinsicsParameterBlockPtr);
    problem_->AddParameterBlock(extrinsicsParameterBlockPtr->parameters(),
                                extrinsicsParameterBlockPtr->dimension(),
                                poseLocalParameterization_.get());
    extrinsicsParameterBlockPtr->setLocalParameterizationPtr(
        poseLocalParameterization_.get());
    VLOG(1) << "Initial T_SC" << i << "\n" << T_SC.T3x4();
    std::shared_ptr<const okvis::cameras::CameraBase> mycameraGeometry =
        cameraSystem_->cameraGeometry(i);
    Eigen::VectorXd intrinsicParams;
    mycameraGeometry->getIntrinsics(intrinsicParams);

    uint64_t int_id = okvis::IdProvider::instance().newId();
    int kIntrinsicDim = kProjIntrinsicDim + kDistortionDim;
    std::shared_ptr<okvis::ceres::EuclideanParamBlock> intrinsicParamBlock(
        new okvis::ceres::EuclideanParamBlock(intrinsicParams, int_id,
                                              startTime, kIntrinsicDim));

    idToParameterBlocks_.emplace(int_id, intrinsicParamBlock);
    problem_->AddParameterBlock(intrinsicParamBlock->parameters(),
                                intrinsicParamBlock->dimension());
    if (baOptions.estimateCameraIntrinsics) {
      problem_->SetParameterBlockVariable(intrinsicParamBlock->parameters());
    } else {
      problem_->SetParameterBlockConstant(intrinsicParamBlock->parameters());
    }

    // frame readout time
    double tr = mycameraGeometry->readoutTime();
    uint64_t tr_id = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::CameraTimeParamBlock> trParamBlock(
        new okvis::ceres::CameraTimeParamBlock(tr, tr_id, startTime));
    idToParameterBlocks_.emplace(tr_id, trParamBlock);
    problem_->AddParameterBlock(trParamBlock->parameters(), 1);
    if (baOptions.estimateReadoutTime) {
      problem_->SetParameterBlockVariable(trParamBlock->parameters());
    } else {
      problem_->SetParameterBlockConstant(trParamBlock->parameters());
    }

    // time offset
    double td = mycameraGeometry->imageDelay();
    uint64_t td_id = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::CameraTimeParamBlock> tdParamBlock(
        new okvis::ceres::CameraTimeParamBlock(td, td_id, startTime));
    idToParameterBlocks_.emplace(td_id, tdParamBlock);
    problem_->AddParameterBlock(tdParamBlock->parameters(),
                                okvis::ceres::CameraTimeParamBlock::Dimension);
    VLOG(1) << "Initial line delay " << tr * (1e9 / cameraSystem_->cameraGeometry(i)->imageHeight())
            << " ns, td " << td << " sec.";
    if (baOptions.estimateCameraDelay) {
      problem_->SetParameterBlockVariable(tdParamBlock->parameters());
    } else {
      problem_->SetParameterBlockConstant(tdParamBlock->parameters());
    }

    CameraBlockIdContainer.push_back(
        CameraBlockInfo(ext_id, int_id, tr_id, td_id));
  }
}

void ViBundleAdjustment::addLandmarks() {
  // Note the landmarks are in homogeneous world coordinates
  for (size_t i = 0; i < data_->homogeneousPoints().size(); ++i) {
    uint64_t lm_id = okvis::IdProvider::instance().newId();
    std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock>
        HomogeneousPointParameterBlock(
            new okvis::ceres::HomogeneousPointParameterBlock(
                data_->homogeneousPoints()[i],
                lm_id));
    idToParameterBlocks_.emplace(lm_id, HomogeneousPointParameterBlock);
    problem_->AddParameterBlock(HomogeneousPointParameterBlock->parameters(),
                                HomogeneousPointParameterBlock->dimension(),
                                landmarkParameterizationPtr_.get());
    HomogeneousPointParameterBlock->setLocalParameterizationPtr(
        landmarkParameterizationPtr_.get());
    problem_->SetParameterBlockConstant(
        HomogeneousPointParameterBlock->parameters());
    landmarkIdContainer.push_back(lm_id);
  }
}

void ViBundleAdjustment::addErrorTerms() {
  addImuFactors();
//  addPrior();
  addCameraObservations();
}

void ViBundleAdjustment::addCameraObservations() {
  data_->rewind();
  size_t refIndex = 0;
  do {
    okvis::Time frameStamp = data_->currentTime();

    // assemble a multi-frame
    std::shared_ptr<MultiFrame> mf(new MultiFrame(cameraSystem_->numCameras(), frameStamp, refIndex));
    for (size_t j = 0u; j < cameraSystem_->numCameras(); ++j) {
      mf->setTimestamp(j, frameStamp);
    }

    // add landmark observations
    std::vector<std::unordered_map<size_t, size_t>> keypointIndices;
    data_->addFeaturesToNFrame(*cameraSystem_, mf, &keypointIndices);
    std::shared_ptr<const okvis::ImuMeasurementDeque> imuMeasDequePtr =
        vertexToState_.at(vertexIdContainer[refIndex]).imuMeasCanopy;

    for (size_t i = 0u; i < cameraSystem_->numCameras(); i++) {
      std::shared_ptr<okvis::cameras::CameraBase> temp =
          std::const_pointer_cast<okvis::cameras::CameraBase>(
              cameraSystem_->cameraGeometry(i));
      std::shared_ptr<swift_vio::DistortedPinholeCameraGeometry>
          mycameraGeometry = std::dynamic_pointer_cast<
              swift_vio::DistortedPinholeCameraGeometry>(temp);
      Eigen::Matrix2d keypointCovariance = Eigen::Matrix2d::Identity() *
          std::pow(cameraNoiseParametersVec_.at(i).sigma_observation, 2);
      for (const auto & l2k : keypointIndices[i]) {
          Eigen::Vector2d keypoint;
          mf->getKeypoint(i, l2k.second, keypoint);

          std::shared_ptr<::ceres::CostFunction> costFunctionPtr;
          std::shared_ptr<okvis::ceres::RsReprojectionError<
              swift_vio::DistortedPinholeCameraGeometry>>
              localCostFunctionPtr(new okvis::ceres::RsReprojectionError<
                                   swift_vio::DistortedPinholeCameraGeometry>(
                  mycameraGeometry, keypoint, keypointCovariance,
                  imuMeasDequePtr,
                  std::shared_ptr<const Eigen::Matrix<double, 6, 1>>(),
                  frameStamp, frameStamp,
                  imuParametersVec_.at(0)->g));
          costFunctionPtr = std::static_pointer_cast<::ceres::CostFunction>(
              localCostFunctionPtr);

          std::vector<double *> parameter_blocks = {
              idToParameterBlocks_[vertexIdContainer[refIndex]]->parameters(),
              idToParameterBlocks_[landmarkIdContainer[l2k.first]]->parameters(),
              idToParameterBlocks_[CameraBlockIdContainer[i].TBCId]
                  ->parameters(),
              idToParameterBlocks_[CameraBlockIdContainer[i].IntrinsicsId]
                  ->parameters(),
              idToParameterBlocks_[CameraBlockIdContainer[i].trId]
                  ->parameters(),
              idToParameterBlocks_[CameraBlockIdContainer[i].tdId]
                  ->parameters(),
              idToParameterBlocks_[vertexToState_
                                       .at(vertexIdContainer[refIndex])
                                       .speedAndBiasId]
                  ->parameters(),
              idToParameterBlocks_[gravityDirectionBlockId_]->parameters()};

          auto residualId = problem_->AddResidualBlock(
              costFunctionPtr.get(),
              lossFunctionPtr_ ? lossFunctionPtr_.get() : NULL,
              parameter_blocks);
          keypointCostFunctions_.at(i).emplace_back(
              costFunctionPtr); // remember in order to avert premature cost
                                // function destruction.
          keypointResidualIds_.at(i).emplace_back(residualId);
          residualIdToParameterBlocks_.emplace(residualId, parameter_blocks);
      }
    }
    refIndex++;
  } while (data_->nextNFrame());
  for (size_t i = 0u; i < cameraSystem_->numCameras(); ++i) {
    VLOG(1) << "Added " << keypointCostFunctions_[i].size() << " reprojection errors for camera " << i << ".";
  }
}

void ViBundleAdjustment::computeStatistics(double maxProjectionErrorTol) {
  size_t numCameras = cameraSystem_->numCameras();
  normalizedErrors_.resize(numCameras);
  errors_.resize(numCameras);
  reprojectionStats_.resize(numCameras);
  reprojectionErrorHist_.resize(numCameras);

  for (size_t i = 0u; i < cameraSystem_->numCameras(); i++) {
    Eigen::Matrix2d keypointRootCov =
        Eigen::Matrix2d::Identity() *
        cameraNoiseParametersVec_.at(i).sigma_observation;
    int j = 0;
    int valid = 0;
    double squaredError = 0;
    boost::accumulators::accumulator_set<
        double,
        boost::accumulators::stats<boost::accumulators::tag::count,
                                   boost::accumulators::tag::density>>
        reprojectionErrorTally(boost::accumulators::tag::density::num_bins = 20,
                               boost::accumulators::tag::density::cache_size =
                                   40);
    size_t numResiduals = keypointCostFunctions_[i].size();
    normalizedErrors_[i].reserve(numResiduals);
    errors_[i].reserve(numResiduals);
    for (auto costFunction : keypointCostFunctions_[i]) {
      auto residualId = keypointResidualIds_[i][j];
      Eigen::Vector2d weightedResidual;
      bool status = costFunction->Evaluate(
          residualIdToParameterBlocks_[residualId].data(),
          weightedResidual.data(), nullptr);
      if (status) {
        Eigen::Vector2d rawError =
            keypointRootCov * weightedResidual; // recover the raw error.
        double errornorm = rawError.norm();
        reprojectionErrorTally(errornorm);
        if (errornorm > maxProjectionErrorTol) {
          ++j;
          continue;
        }
        errors_[i].emplace_back(errornorm);
        normalizedErrors_[i].emplace_back(
            std::sqrt(weightedResidual.transpose() * weightedResidual));
        squaredError += rawError.transpose() * rawError;
        ++valid;
      }
      ++j;
    }
    reprojectionStats_[i] = ReprojectionStatistics(
        squaredError, valid, numResiduals);
    reprojectionErrorHist_[i] =
        boost::accumulators::density(reprojectionErrorTally);
  }
}

void ViBundleAdjustment::addImuFactors() {
  data_->rewind();
  size_t refIndex = 0;
  okvis::Time lastFrameTime = data_->currentTime();
  refIndex++;
  data_->nextNFrame();
  do {
    okvis::Time refNFrameTime = data_->currentTime();

    // add IMU error terms
    std::shared_ptr<const okvis::ImuMeasurementDeque> imuMeasDequePtr =
        vertexToState_.at(vertexIdContainer[refIndex]).imuMeasCanopy;
    for (size_t i = 0; i < imuParametersVec_.size(); ++i) {
      // use DynamicImuError to consider extra IMU parameters.
      std::shared_ptr<okvis::ceres::ImuErrorWithGravity> ImuErrorWithGravity(
          new okvis::ceres::ImuErrorWithGravity(*imuMeasDequePtr,
                                                *imuParametersVec_.at(i),
                                                lastFrameTime, refNFrameTime));
      // parameter_blocks
      std::vector<double *> parameter_blocks = {
          idToParameterBlocks_[vertexIdContainer[refIndex - 1]]->parameters(),
          idToParameterBlocks_[vertexToState_
                                   .at(vertexIdContainer[refIndex - 1])
                                   .speedAndBiasId]
              ->parameters(),
          idToParameterBlocks_[vertexIdContainer[refIndex]]->parameters(),
          idToParameterBlocks_[vertexToState_.at(vertexIdContainer[refIndex])
                                   .speedAndBiasId]
              ->parameters(),
          idToParameterBlocks_[gravityDirectionBlockId_]->parameters()};

      problem_->AddResidualBlock(ImuErrorWithGravity.get(), NULL, parameter_blocks);
      costFunctions_.push_back(
          ImuErrorWithGravity); // remember in order to avert premature cost
                                // function destruction.
    }
    lastFrameTime = refNFrameTime;
    refIndex++;
  } while (data_->nextNFrame()); //*/
}

void ViBundleAdjustment::addPrior() {
  uint64_t headVertexId = vertexIdContainer[0];
  auto blockIt = idToParameterBlocks_.find(headVertexId);

  okvis::kinematics::Transformation T_WS =
      std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(blockIt->second)->estimate();
  size_t refIndex = 0;
  okvis::Time refNFrameTime = data_->currentTime();

  Eigen::Matrix<double, 6, 6> information;
  initialNavState_.toInformation(&information);
  std::shared_ptr<okvis::ceres::PoseError> poseError(
      new okvis::ceres::PoseError(T_WS, information));
  problem_->AddResidualBlock(poseError.get(), NULL,
                             idToParameterBlocks_[vertexIdContainer[refIndex]]
                                 ->parameters());
  costFunctions_.push_back(poseError); // remember in order to avert premature
                                       // cost function destruction.
}

void ViBundleAdjustment::solve() {
  ::ceres::Solver::Options options;
  // options.check_gradients=true;
  // options.numeric_derivative_relative_step_size = 1e-6;
  // options.gradient_check_relative_precision=1e-2;
  options.minimizer_progress_to_stdout = false;
  ::FLAGS_stderrthreshold =
      google::WARNING; // enable console warnings (Jacobian verification)
  ::ceres::Solver::Summary summary;
  okvis::timing::Timer timer("ceres solve", false);
  Solve(options, problem_.get(), &summary);
  double elapsed = timer.stop();
  LOG(INFO) << "Ceres Solve takes " << elapsed << " sec.";
  LOG(INFO) << summary.BriefReport();
}

void ViBundleAdjustment::solveWithOutlierRejection() {
  // refer to maplab solveWithOutlierRejection. Do not implement for now.
}

void outputHistogram(const std::string &featureHistFile,
                     const std::vector<HistogramType> &hist) {
  std::ofstream featureHistStream(featureHistFile, std::ios_base::out);
  for (size_t j = 0u; j < hist.size(); ++j) {
    featureHistStream << "For camera " << j
                      << ", histogram of reprojection errors [px] (bin "
                      << "lower bound, value)" << std::endl;
    for (size_t i = 0; i < hist[j].size(); i++) {
      featureHistStream << hist[j][i].first << " " << hist[j][i].second << "\n";
    }
  }
  featureHistStream.close();
}

void ViBundleAdjustment::saveResults(const std::string &outputdir) {
  // save trajectory in maplab vertex format
  // time(sec), pxyz, qxyzw, velocity, bias gyro, bias accel,
  std::string truthFile = outputdir + "/trajectory.txt";
  saveMotion(truthFile);

  std::string cameraFile = outputdir + "/camchain-imucam.yaml";
  saveCameraParameters(cameraFile);

  std::string imuSampleFile = outputdir + "/imu.yaml";
  saveImuParams(imuSampleFile);

  std::string residualStatFile = outputdir + "/results-imucam.txt";
  saveResidualStats(residualStatFile);

  checkHist("Reprojection error histogram");
  std::string reprojectionHistFile = outputdir + "/reprojectionHist.txt";
  outputHistogram(reprojectionHistFile, reprojectionErrorHist_);
}

void ViBundleAdjustment::checkHist(const std::string &msg) const {
  for (size_t j = 0; j < reprojectionErrorHist_.size(); ++j) {
    double total = 0;
    for (size_t i = 0; i < reprojectionErrorHist_[j].size(); i++) {
      total += reprojectionErrorHist_[j][i].second;
    }
    if (std::fabs(total - 1.0) > 1e-4) {
      LOG(ERROR) << msg << ": cumulative bin length is " << total << " for "
                 << j << "th histogram!";
    }
  }
}

void ViBundleAdjustment::saveImuParams(const std::string& imuFile) const {
  std::ofstream stream(imuFile);
  stream << "imu0:\n";
  std::string pad = "  ";
  Eigen::Matrix<double, 4, 4> T_i_b = Eigen::Matrix<double, 4, 4>::Identity();
  YamlHelpers::writeMatToYaml(T_i_b, "T_i_b", stream, pad);

  auto iter = idToParameterBlocks_.find(gravityDirectionBlockId_);
  if (iter != idToParameterBlocks_.end()) {
    std::shared_ptr<okvis::ceres::NormalVectorParameterBlock> sptr =
        std::static_pointer_cast<okvis::ceres::NormalVectorParameterBlock>(iter->second);
    Eigen::Vector3d unitgW = sptr->estimate();
    Eigen::Vector3d gW = imuParametersVec_.at(0)->g * unitgW;
    YamlHelpers::writeMatToYaml(gW, "gravity_in_target", stream, pad);
  }
  stream << pad << "model: calibrated\n";
  stream << pad << "rostopic: /imu0\n";
  stream << pad << "time_offset: 0.0\n";
  stream << pad << "update_rate: " << imuParametersVec_.at(0)->rate << "\n";
  stream.close();
}

void ViBundleAdjustment::saveCameraParameters(
    const std::string cameraImuYaml) const {
  /* output format looks like:
cam0:
  T_cam_imu:
  - - 0.0007105942420218558
    - 0.9999995894052528
    - 0.0005623567815139246
    - -0.00012200652080512381
  - - -0.9999997193627065
    - 0.0007107275795398826
    - -0.0002369405342396697
    - 0.09911398058049195
  - - -0.00023734011942729415
    - -0.0005621882551163069
    - 0.9999998138069998
    - -0.00034246939036863415
  - - 0.0
    - 0.0
    - 0.0
    - 1.0
  cam_overlaps:
  - 1
  - 3
  camera_model: pinhole
  distortion_coeffs:
  - 0
  - 0
  - 0
  - 0
  distortion_model: equidistant
  image_noise_std_dev: 1.0
  intrinsics:
  - 460.0
  - 460.0
  - 356.0
  - 240.0
  line_delay_nanoseconds: 41171
  resolution:
  - 752
  - 480
  rostopic: /cam0/image_raw
  timeshift_cam_imu: -4.264682698254876e-05
  update_rate: 20
*/

  std::ofstream stream(cameraImuYaml, std::ofstream::out);
  std::string pad = "  ";
  std::string lead = "- ";
  for (size_t i = 0u; i < cameraSystem_->numCameras(); ++i) {
    stream << "cam" << i << ":\n";
    double *extemp = nullptr;
    auto exit = idToParameterBlocks_.find(CameraBlockIdContainer[i].TBCId);
    if (exit != idToParameterBlocks_.end()) {
      extemp = exit->second->parameters();
      Eigen::Map<const Eigen::Vector3d> p_BC(extemp);
      Eigen::Map<const Eigen::Quaterniond> q_BC(extemp + 3);
      Eigen::Matrix3d R_BC = q_BC.toRotationMatrix();
      Eigen::Matrix<double, 4, 4> T_CB;
      T_CB.setIdentity();
      T_CB.topLeftCorner<3, 3>().noalias() = R_BC.transpose();
      T_CB.topRightCorner<3, 1>() = -R_BC.transpose() * p_BC;

      YamlHelpers::writeMatToYaml(T_CB, "T_cam_imu", stream, pad);
    }
    stream << pad << "cam_overlaps: []\n";
    stream << pad << "camera_model: pinhole\n";
    size_t numDistortionCoeffs =
        cameraSystem_->cameraGeometry(i)->noDistortionParameters();
    size_t numIntrinsics =
        cameraSystem_->cameraGeometry(i)->noIntrinsicsParameters();

    double *pintemp = nullptr;
    auto iit =
        idToParameterBlocks_.find(CameraBlockIdContainer[i].IntrinsicsId);
    Eigen::Matrix<double, -1, 1> intrinsics(numIntrinsics, 1);
    if (iit != idToParameterBlocks_.end()) {
      pintemp = iit->second->parameters();
      memcpy(intrinsics.data(), pintemp, sizeof(double) * numIntrinsics);
    }

    YamlHelpers::writeMatToYaml(intrinsics.tail(numDistortionCoeffs), "distortion_coeffs",
                   stream, pad);

    stream << pad << "distortion_model: "
           << swift_vio::DistortionTypeToKalibrModel(
                  cameraSystem_->distortionType(i))
           << "\n";
    double imageNoiseStdDev = cameraNoiseParametersVec_.at(i).sigma_observation;
    stream << pad << "image_noise_std_dev: " << imageNoiseStdDev << "\n";

    int width = cameraSystem_->cameraGeometry(i)->imageWidth();
    int height = cameraSystem_->cameraGeometry(i)->imageHeight();
    YamlHelpers::writeMatToYaml(intrinsics.head<4>(), "intrinsics", stream, pad);

    YamlHelpers::writeMatToYaml(Eigen::Vector2d(width, height), "resolution", stream, pad);

    double *tdtemp = nullptr;
    auto tdit = idToParameterBlocks_.find(CameraBlockIdContainer[i].tdId);
    if (tdit != idToParameterBlocks_.end()) {
      tdtemp = tdit->second->parameters();
      stream << pad << "timeshift_cam_imu: " << *tdtemp << "\n";
    }

    double *trtemp = nullptr;
    auto trit = idToParameterBlocks_.find(CameraBlockIdContainer[i].trId);
    if (trit != idToParameterBlocks_.end()) {
      trtemp = trit->second->parameters();
      double linedelayns =
          *(trtemp) * (1e9 / cameraSystem_->cameraGeometry(i)->imageHeight());
      stream << pad << "line_delay_nanoseconds: " << linedelayns << "\n";
    }
    stream << pad << "rostopic: /cam" << i << "/image_raw\n";
    stream << pad << "update_rate: 20\n";
  }
  stream.close();
}

void ViBundleAdjustment::saveResidualStats(const std::string residualStatFile) {
  /*
Calibration results
===================
Normalized Residuals
----------------------------
Reprojection error (cam0):     mean 1.247403674, median 1.17063900269, std: 0.654463536436
Gyroscope error (imu0):        mean 1.59561415242, median 1.53223967147, std: 0.676550035809
Accelerometer error (imu0):    mean 1.54302280484, median 1.42166941965, std: 0.789351377661

Residuals
----------------------------
Reprojection error (cam0) [px]:     mean 1.24755984693, median 1.17079619952, std: 0.654490493075
Gyroscope error (imu0) [rad/s]:     mean 0.109449686557, median 0.105102572272, std: 0.0464073280168
Accelerometer error (imu0) [m/s^2]: mean 0.280510869214, median 0.258449663473, std: 0.143498618666
   */
  std::ofstream ofs(residualStatFile, std::ofstream::out);
  ofs << "Calibration results\n";
  ofs << "===================\n";
  ofs << "Normalized Residuals\n";
  ofs << "----------------------------\n";

  for (size_t i = 0u; i < cameraSystem_->numCameras(); ++i) {
    ofs << "cam" << i << ": " << reprojectionStats_[i].toString(",") << "\n";
    boost::accumulators::accumulator_set<
        double,
        boost::accumulators::features<boost::accumulators::tag::mean,
                                      boost::accumulators::tag::max,
                                      boost::accumulators::tag::min,
                                      boost::accumulators::tag::median,
                                      boost::accumulators::tag::variance>>
        nacc;
    for_each(normalizedErrors_[i].begin(), normalizedErrors_[i].end(),
             boost::bind<void>(std::ref(nacc), _1));
    ofs << "Reprojection error (cam" << i << "):\tmean ";
    ofs << boost::accumulators::mean(nacc) << ", median "
        << boost::accumulators::median(nacc) << ", std "
        << std::sqrt(boost::accumulators::variance(nacc)) << "\n";
    ofs << "min " << boost::accumulators::min(nacc) 
        << " max " << boost::accumulators::max(nacc) << "\n";
  }

  ofs << "\n";
  ofs << "Residuals\n";
  ofs << "----------------------------\n";
  for (size_t i = 0u; i < cameraSystem_->numCameras(); ++i) {
    boost::accumulators::accumulator_set<
        double,
        boost::accumulators::features<boost::accumulators::tag::mean,
                                      boost::accumulators::tag::max,
                                      boost::accumulators::tag::min,
                                      boost::accumulators::tag::median,
                                      boost::accumulators::tag::variance>>
        acc;
    for_each(errors_[i].begin(), errors_[i].end(),
             boost::bind<void>(std::ref(acc), _1));
    ofs << "Reprojection error (cam" << i << ") [px]:\tmean ";
    ofs << boost::accumulators::mean(acc) << ", median "
        << boost::accumulators::median(acc) << ", std "
        << std::sqrt(boost::accumulators::variance(acc)) << "\n";
    ofs << "min " << boost::accumulators::min(acc) 
        << " max " << boost::accumulators::max(acc) << "\n";
  }
  ofs.close();
}

void ViBundleAdjustment::saveMotion(const std::string &truthFile) const {
  std::ofstream truthStream;
  truthStream.open(truthFile, std::ofstream::out);
  truthStream << "%state timestamp, nframeId, T_WS(xyz, qxyzw), v_WS, bg, ba"
              << std::endl;
  for (size_t id = 0u; id < vertexIdContainer.size(); ++id) {
    auto stateIt = vertexToState_.find(vertexIdContainer[id]);
    if (stateIt != vertexToState_.end()) {
      truthStream << stateIt->second.time << " " << id << " "
                  << std::setfill(' ');
    }
    double *Posetemp = nullptr;
    auto poseit = idToParameterBlocks_.find(vertexIdContainer[id]);
    if (poseit != idToParameterBlocks_.end()) {
      Posetemp = poseit->second->parameters();
      for (size_t j = 0; j < poseit->second->dimension(); j++) {
        truthStream << *(Posetemp + j) << " ";
      }
    }
    double *Veltemp = nullptr;
    auto velit = idToParameterBlocks_.find(
        vertexToState_.at(vertexIdContainer[id]).speedAndBiasId);
    if (velit != idToParameterBlocks_.end()) {
      Veltemp = velit->second->parameters();
      for (size_t j = 0; j < velit->second->dimension(); j++) {
        truthStream << *(Veltemp + j) << " ";
      }
    }
    truthStream << std::endl;
  }
  truthStream.close();
}

} // namespace swift_vio
