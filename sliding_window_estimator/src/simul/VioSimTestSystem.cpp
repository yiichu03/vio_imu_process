#include "simul/VioSimTestSystem.hpp"

#include <gtsam/FixedLagSmoother.hpp>

#include <okvis/IdProvider.hpp>

#include <swift_vio/IoUtil.hpp>
#include <swift_vio/MapPoint.h>
#include <swift_vio/PointLandmarkModels.hpp>
#include <swift_vio/SlidingWindowFilter.h>
#include <swift_vio/StatAccumulator.h>
#include <swift_vio/VioEvaluationCallback.hpp>
#include <swift_vio/VioFactoryMethods.hpp>
#include <swift_vio/VioSystem.hpp>

#include <simul/CameraSystemCreator.hpp>
#include <simul/simGflags.hpp>
#include <simul/numeric_ceres_residual_Jacobian.hpp>

namespace simul {
typedef boost::iterator_range<std::vector<std::pair<double, double>>::iterator>
    HistogramType;

void outputFeatureHistogram(const std::string &featureHistFile,
                            const HistogramType &hist) {
  std::ofstream featureHistStream(featureHistFile, std::ios_base::out);
  double total = 0.0;
  featureHistStream << "Histogram of number of features in images (bin "
                    << "lower bound, value)" << std::endl;
  for (size_t i = 0; i < hist.size(); i++) {
    featureHistStream << hist[i].first << " " << hist[i].second << std::endl;
    total += hist[i].second;
  }
  if (std::fabs(total - 1.0) > 1e-4) {
    std::cerr << "Total of feature histogram densities " << total << " != 1.\n";
  }
  featureHistStream.close();
}

Eigen::Matrix<double, 6, 1>
computeNormalizedErrors(const Eigen::VectorXd &errors,
                        const Eigen::MatrixXd &covariance) {
  Eigen::Matrix<double, 6, 1> normalizedSquaredError;
  Eigen::Vector3d deltaP = errors.head<3>();
  Eigen::Vector3d alpha = errors.segment<3>(3);

  normalizedSquaredError[0] =
      deltaP.transpose() * covariance.topLeftCorner<3, 3>().inverse() * deltaP;
  normalizedSquaredError[1] =
      alpha.transpose() * covariance.block<3, 3>(3, 3).inverse() * alpha;

  Eigen::Matrix<double, 6, 1> deltaPose = errors.head<6>();
  Eigen::Matrix<double, 6, 1> tempPoseError =
      covariance.topLeftCorner<6, 6>().ldlt().solve(deltaPose);
  normalizedSquaredError[2] = deltaPose.transpose() * tempPoseError;

  Eigen::Vector3d deltaV = errors.segment<3>(6);
  Eigen::Vector3d deltaBg = errors.segment<3>(9);
  Eigen::Vector3d deltaBa = errors.segment<3>(12);
  normalizedSquaredError[3] =
      deltaV.transpose() * covariance.block<3, 3>(6, 6).inverse() * deltaV;
  normalizedSquaredError[4] =
      deltaBg.transpose() * covariance.block<3, 3>(9, 9).inverse() * deltaBg;
  normalizedSquaredError[5] =
      deltaBa.transpose() * covariance.block<3, 3>(12, 12).inverse() * deltaBa;
  return normalizedSquaredError;
}

VioSimTestSystem::VioSimTestSystem(const CheckMseCallback &checkMseCallback,
                 const CheckNeesCallback &checkNeesCallback)
    : checkMseCallback_(checkMseCallback),
      checkNeesCallback_(checkNeesCallback), checkedLmks_(0),
      failedTriangulationLmks_(0), poorTriangulationLmks_(0),
      goodTriangulationLmks_(0), checkedJacobianLmks_(0),
      diffJacStatusLmks_(0), poorJacobianLmks_(0), goodJacobianLmks_(0),
      viewerNamePrefix_("Reprojected landmarks for camera"), publisher_(nullptr) {}

VioSimTestSystem::~VioSimTestSystem() {
}

void VioSimTestSystem::registerCallbacks(swift_vio::StreamPublisher* publisher) {
  publisher_ = publisher;
  fullStateCallback_ =
      std::bind(&swift_vio::StreamPublisher::publishFullStateAsCallback, publisher,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4);
  landmarksCallback_ = std::bind(
      &swift_vio::StreamPublisher::publishLandmarksAsCallback, publisher,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}

void VioSimTestSystem::createRefSensorSystem(const SimParameters &simParameters,
                                          const okvis::cameras::NCameraSystem &vioCameraSystem) {
  if (simParameters.cameraParams.cameraModelId ==
      simul::SimCameraModelType::FromConfigYaml) {
    refCameraSystem_ = swift_vio::CameraRig::deepCopyPtr(vioCameraSystem);
  } else {
    simul::CameraSystemCreator csc(
        simParameters.cameraParams.cameraModelId,
        simParameters.cameraParams.cameraOrientationId,
        vioCameraSystem.projectionIntrinsicRep(0),
        vioCameraSystem.cameraGeometry(0)->imageDelay(),
        vioCameraSystem.cameraGeometry(0)->readoutTime());

    csc.createNominalCameraSystem(&refCameraSystem_,
                                  vioCameraSystem.distortionType(0),
                                  vioCameraSystem.extrinsicRepNames());
  }

  refImuParameters_ = simParameters.imuNoiseParams;
  // gravity in the world frame of the real data.
  Eigen::Vector3d gW = refImuParameters_.gravity();
  if ((gW - Eigen::Vector3d(0, 0, -9.80665)).norm() > 1.0) {
    if (refImuParameters_.isGravityDirectionFixed())
      LOG(WARNING) << "Gravity direction is unconventional and is locked up as " << gW.transpose() << ".";
  }
}

void VioSimTestSystem::createInitSensorSystem(const SimParameters &simParameters,
                                              okvis::VioParameters *vioParameters) {
  simData_->navStateAtStart(vioParameters->initialState, &refNavState_);
  initialNavState_ = refNavState_;
  initialNavState_.v_WS = refNavState_.v_WS +
        vio::Sample::gaussian(1, 3) * simParameters.sigma_initial_speed;

  initialCameraSystem_ = createNoisyCameraSystem(
        refCameraSystem_, simParameters.camNoiseParams,
        vioParameters->nCameraSystem.extrinsicRepNames());

  okvis::ImuParameters initialImuParameters = vioParameters->imu;
  {
    initialImuParameters.rate = refImuParameters_.rate;
    initialImuParameters.setInitialGyroBias(
        refImuParameters_.initialGyroBias() +
        vio::Sample::gaussian(refImuParameters_.sigma_bg, 3));
    initialImuParameters.setInitialAccelBias(
        refImuParameters_.initialAccelBias() +
        vio::Sample::gaussian(refImuParameters_.sigma_ba, 3));
    initialImuParameters.setGyroCorrectionMatrix(
        refImuParameters_.gyroCorrectionMatrix() +
        vio::Sample::gaussian(refImuParameters_.sigma_Mg_element, 9));
    initialImuParameters.setGyroGSensitivity(
        refImuParameters_.gyroGSensitivity() +
        vio::Sample::gaussian(refImuParameters_.sigma_Ts_element, 9));
    initialImuParameters.setAccelCorrectionMatrix(
        refImuParameters_.accelCorrectionMatrix() +
        vio::Sample::gaussian(refImuParameters_.sigma_Ma_element, 6));

    Eigen::Vector3d refUnitG = refImuParameters_.gravityDirection();
    swift_vio::NormalVectorElement nve(refUnitG), nvePlus;
    nve.boxPlus(vio::Sample::gaussian(refImuParameters_.sigma_gravity_direction, 2), nvePlus);
    Eigen::Vector3d noisyUnitG = nvePlus.getVec();
    initialImuParameters.setGravityDirection(noisyUnitG);
  }

  evaluationCallback_.reset(new swift_vio::VioEvaluationCallback());

  vioParameters->initialState = initialNavState_;
  vioParameters->imu = initialImuParameters;

  initialCameraSystem_->cloneTo(&vioParameters->nCameraSystem);
//  estimatedCameraSystem_ = initialCameraSystem_->deepCopyPtr();
  estimatedOkvisCameraSystem_.reset(new okvis::cameras::NCameraSystem());
  initialCameraSystem_->cloneTo(estimatedOkvisCameraSystem_.get());
}

void VioSimTestSystem::createEstimator(const okvis::VioParameters &vioParameters, const swift_vio::BackendParams *flsParams) {
  if (vioParameters.optimization.algorithm == swift_vio::EstimatorAlgorithm::OkvisEstimator ||
      vioParameters.optimization.algorithm == swift_vio::EstimatorAlgorithm::FixedLagSmoother ||
      vioParameters.optimization.algorithm == swift_vio::EstimatorAlgorithm::RiFixedLagSmoother) {
    okvisEstimator_ = okvis::createBackend(vioParameters.optimization, *flsParams);
    swift_vio::configureEstimator(vioParameters, okvisEstimator_);
  } else {
    estimator_ = swift_vio::createBackend(vioParameters.optimization);
    swift_vio::configureEstimator(vioParameters, estimator_);

    initializer_.reset(
        new swift_vio::VioInitializer(vioParameters.optimization));
    swift_vio::configureEstimator(vioParameters, initializer_);
  }
}

void VioSimTestSystem::runOkvisEstimator(const simul::SimParameters &simParameters,
                                         const swift_vio::BackendParams &flsParams,
                                         okvis::VioParameters *vioParameters) {
  swift_vio::StatAccumulator neesAccumulator;
  swift_vio::StatAccumulator rmseAccumulator;

  srand((unsigned int)time(0)); // comment out to make tests deterministic.

  // number of features tracked in a frame.
  boost::accumulators::accumulator_set<
      double, boost::accumulators::features<boost::accumulators::tag::count,
                                            boost::accumulators::tag::density>>
      frameFeatureTally(boost::accumulators::tag::density::num_bins = 20,
                        boost::accumulators::tag::density::cache_size = 40);
  std::string featureHistFile = simParameters.outputdir + "/FeatureHist.txt";

  okvis::timing::Timer runTimer("Estimation run", true);
  std::string testIdentifier = swift_vio::EnumToString(vioParameters->optimization.algorithm) + "_" +  simParameters.trajLabel;
  std::string pathEstimatorTrajectory = simParameters.outputdir + "/" + testIdentifier;
  std::string neesFile = pathEstimatorTrajectory + "_NEES.txt";
  std::string rmseFile = pathEstimatorTrajectory + "_RMSE.txt";
  std::string metadataFile = pathEstimatorTrajectory + "_metadata.txt";
  std::string headerLine;
  std::string rmseHeaderLine;
  std::ofstream metaStream;
  metaStream.open(metadataFile, std::ofstream::out);

  createRefSensorSystem(simParameters, vioParameters->nCameraSystem);
  visualizer_ = swift_vio::VioVisualizer(*vioParameters, viewerNamePrefix_);
  visualizer_.setCameraSystem(*refCameraSystem_);
  if (publisher_)
    publisher_->setCameraSystem(*refCameraSystem_);

  loadSimulatedData(simParameters);
  LOG(INFO) << simParameters.toString();

  for (int run = 0; run <  simParameters.numRuns; ++run) {
//    bool verbose = neesAccumulator.succeededRuns() == 0;
    runTimer.start();
    LOG(INFO) << "Run " << run << " " << testIdentifier << ".";

    std::stringstream ss;
    ss << run;
    std::string outputFile = pathEstimatorTrajectory + "_" + ss.str() + ".txt";

    SimFrontendOptions frontendOptions(60, vioParameters->frontendOptions.numKeyframesToMatch);
    frontendOptions.useTrueLandmarkPosition_ =
         simParameters.cameraParams.useTrueLandmarkPosition;
    coupledFrontend_.reset(new CoupledSimulationFrontend(
        simData_->homogeneousPoints(), simData_->landmarkIds(),
        refCameraSystem_->numCameras(), frontendOptions));

    createInitSensorSystem(simParameters, vioParameters);
    createEstimator(*vioParameters, &flsParams);

    std::ofstream debugStream;
    debugStream.open(outputFile, std::ofstream::out);
    headerLine = okvisEstimator_->headerLine();
    rmseHeaderLine = okvisEstimator_->rmseHeaderLine();

    std::vector<std::string> perturbationLabels = okvisEstimator_->perturbationLabels();

    debugStream << headerLine << std::endl;

    bool hasStarted = false;
    int frameCount = 0;     // number of frames used in estimator
    int trackedFeatures = 0; // feature tracks observed in a frame
    bool runSuccessful = true;

    simData_->resetImuBiases(refImuParameters_, "");
    simData_->rewind();
    if (publisher_)
      publisher_->rewind();

    int expectedNumFrames = simData_->expectedNumNFrames();
    neesAccumulator.refreshBuffer(expectedNumFrames);
    rmseAccumulator.refreshBuffer(expectedNumFrames);
    try {
      do {
        okvis::Time refNFrameTime = simData_->currentTime();
        okvis::kinematics::Transformation T_WS_ref = simData_->currentPose();
        Eigen::Vector3d v_WS_ref = simData_->currentVelocity();
        Eigen::Matrix<double, 6, 1> biasRef = simData_->currentBiases().toVector();
        const okvis::ImuParameters &refImuParams = refImuParameters_;

        okvis::ImuMeasurementDeque imuSegment =
            simData_->imuMeasurementsSinceLastNFrame();

        // assemble a multi-frame
        uint64_t id = okvis::IdProvider::instance().newId();
        okvis::Time frameStamp = refNFrameTime - okvis::Duration(refCameraSystem_->cameraGeometry(0)->imageDelay());
        std::shared_ptr<okvis::MultiFrame> mf(new okvis::MultiFrame(refCameraSystem_->numCameras(), frameStamp, id));
        okvisEstimator_->getEstimatedCameraSystem(estimatedOkvisCameraSystem_.get());
        mf->resetCameraSystemAndFrames(*estimatedOkvisCameraSystem_);
        for (size_t j = 0u; j < refCameraSystem_->numCameras(); ++j) {
          mf->setTimestamp(j, frameStamp);
        }

        VLOG(1) << "Processing frame " << id << " of index " << frameCount;

        bool asKeyframe = false;
        if (!hasStarted) {
          hasStarted = true;
          okvisEstimator_->setInitialNavState(initialNavState_);
          okvisEstimator_->setInitializationStatus(okvis::EstimatorBase::InitializationStatus::RunningNonlinEst);
          asKeyframe = true;
          okvisEstimator_->addStates(mf, imuSegment, asKeyframe);
        } else {
          asKeyframe = false;
          okvisEstimator_->addStates(mf, imuSegment, asKeyframe);
        }
        ++frameCount;
        if (vioParameters->frontendOptions.allAreKeyframes) {
          asKeyframe = true;
        }

        // add landmark observations
        trackedFeatures = 0;
        if (simParameters.cameraParams.useImageObservations) {
          std::vector<std::unordered_map<size_t, size_t>> keypointIndices;
          simData_->addFeaturesToNFrame(*refCameraSystem_, mf, &keypointIndices);
          trackedFeatures = coupledFrontend_->dataAssociationAndInitialization(
              *okvisEstimator_, keypointIndices, mf, &asKeyframe);
          okvisEstimator_->setKeyframe(mf->id(), asKeyframe);
        }

        frameFeatureTally(trackedFeatures);

        size_t maxIterations = 10u;
        size_t numThreads = 2u;
        okvisEstimator_->optimize(maxIterations, numThreads, false);

        okvis::MapPointVector removedLandmarks;
        okvisEstimator_->applyMarginalizationStrategy(removedLandmarks);

        publish<okvis::EstimatorBase>(okvisEstimator_, *vioParameters);
        visualize<okvis::EstimatorBase, okvis::MultiFrame>(
            okvisEstimator_, *vioParameters,
            okvisEstimator_->multiFrame(okvisEstimator_->currentFrameId()),
            okvisEstimator_->multiFrame(okvisEstimator_->currentKeyframeId()));

        Eigen::MatrixXd covariance;
        okvisEstimator_->computeCovariance(&covariance);

        okvisEstimator_->printStatesAndStdevs(debugStream, &covariance);

        Eigen::VectorXd errors;
        okvisEstimator_->computeErrors(T_WS_ref, v_WS_ref, biasRef, refImuParams,
                                       refCameraSystem_, &errors);
        Eigen::VectorXd squaredError = errors.cwiseAbs2();
        Eigen::VectorXd normalizedSquaredError =
            computeNormalizedErrors(errors, covariance);

        if (errors.head<3>().lpNorm<Eigen::Infinity>() > FLAGS_sim_max_position_Rmse) {
          runSuccessful = false;
        }

        neesAccumulator.push_back(refNFrameTime, normalizedSquaredError);
        rmseAccumulator.push_back(refNFrameTime, squaredError);
      } while (simData_->nextNFrame());

      Eigen::VectorXd desiredStdevs;
      okvisEstimator_->getDesiredStdevs(&desiredStdevs);
      checkMseCallback_(rmseAccumulator.lastValue(), desiredStdevs, perturbationLabels);
      checkNeesCallback_(neesAccumulator.lastValue());

      if (runSuccessful) {
        neesAccumulator.accumulate();
        rmseAccumulator.accumulate();
      }

      std::stringstream messageStream;
      messageStream << "Run " << run << " finishes with #processed frames " << frameCount
                    << " #tracked features in last frame " << trackedFeatures
                    << " #keyframes " << coupledFrontend_->numKeyframes() << ". Successful? " << runSuccessful;
      LOG(INFO) << messageStream.str();
      metaStream << messageStream.str() << std::endl;

      // output track length distribution
      std::string trackStatFile =
          pathEstimatorTrajectory + "_trackstat_" + ss.str() + ".txt";
      std::ofstream trackStatStream(trackStatFile, std::ios_base::out);
      okvisEstimator_->printTrackLengthHistogram(trackStatStream);
      trackStatStream.close();
    } catch (std::exception &e) {
      std::stringstream messageStream;
      messageStream << "Run " << run << " aborts with #processed frames "
                    << frameCount << " #tracked features in last frame "
                    << trackedFeatures << " #keyframes "
                    << coupledFrontend_->numKeyframes() << " and error: " << e.what();
      LOG(INFO) << messageStream.str();
      metaStream << messageStream.str() << std::endl;
      if (debugStream.is_open()) {
        debugStream.close();
      }
    }
    double elapsedTime =  runTimer.stop();
    std::stringstream sstream;
    sstream << "Run " << run << " used " << elapsedTime << " seconds.";
    LOG(INFO) << sstream.str();
    metaStream << sstream.str() << std::endl;
  }  // next run

  HistogramType hist = boost::accumulators::density(frameFeatureTally);
  outputFeatureHistogram(featureHistFile, hist);

  int numSucceededRuns = neesAccumulator.succeededRuns();
  std::stringstream message;
  message << "#successful runs " << numSucceededRuns << " out of "
          <<  simParameters.numRuns << " runs.";

  std::string neesHeaderLine =
      "%state timestamp, NEES of p_WS, \\alpha_WS, T_WS, v_WS, b_g, b_a";
  neesAccumulator.computeMean();
  neesAccumulator.dump(neesFile, neesHeaderLine);

  rmseAccumulator.computeRootMean();
  rmseAccumulator.dump(rmseFile, rmseHeaderLine);

  LOG(INFO) << message.str();
  metaStream << message.str() << std::endl;
  metaStream.close();
}

void VioSimTestSystem::loadSimulatedData(const simul::SimParameters &simParameters) {
  if (!simParameters.simDataPath.empty()) {
    simData_ = std::shared_ptr<SimulatorBase>(new SimFromRealData(
        simParameters.simDataPath, refImuParameters_,
        simParameters.cameraParams.addExtraLandmarks,
        simParameters.cameraParams.addImageNoise,
        simParameters.imuParams.addImuNoise));
  } else {
    std::string landmarkCsv = FLAGS_sim_landmark_csv;
    std::string trajectoryCsv = FLAGS_sim_trajectory_csv;
    if (!trajectoryCsv.empty()) {
#ifdef HAVE_BSPLINES
      simData_ = std::shared_ptr<SimulatorBase>(new SimFromSplineData(
          trajectoryCsv, landmarkCsv, refImuParameters_,
          simParameters.cameraParams.addExtraLandmarks,
          simParameters.cameraParams.addImageNoise,
          simParameters.imuParams.addImuNoise));
#endif
    } else {
      simData_ = std::shared_ptr<SimulatorBase>(new CurveData(
          simParameters, refImuParameters_, simParameters.duration,
          simParameters.cameraParams.addExtraLandmarks,
          simParameters.cameraParams.addImageNoise,
          simParameters.imuParams.addImuNoise));
    }
  }

  simData_->initializeLandmarkGrid(
      simParameters.cameraParams.landmarkDistribution,
      simParameters.cameraParams.landmarkCylinderRadius, 5.0,
      refCameraSystem_.get());

  std::string pointFile =
       simParameters.outputdir + "/" + simParameters.trajLabel + "_Points.txt";
  simData_->saveLandmarkGrid(pointFile);

  std::string imuSampleFile =
       simParameters.outputdir + "/" + simParameters.trajLabel + "_IMU.txt";
  simData_->resetImuBiases(refImuParameters_, imuSampleFile);

  std::string truthFile =  simParameters.outputdir + "/" + simParameters.trajLabel + ".txt";
  simData_->saveRefMotion(truthFile);

  std::string cameraFile =  simParameters.outputdir + "/" + simParameters.trajLabel + "_CameraSystem.txt";
  saveCameraParameters(refCameraSystem_, cameraFile);
}

void VioSimTestSystem::run(const simul::SimParameters &simParameters,
                           okvis::VioParameters *vioParameters) {
  swift_vio::StatAccumulator neesAccumulator;
  swift_vio::StatAccumulator rmseAccumulator;

  srand((unsigned int)time(0)); // comment out to make tests deterministic.

  // number of features tracked in a frame.
  boost::accumulators::accumulator_set<
      double, boost::accumulators::features<boost::accumulators::tag::count,
                                            boost::accumulators::tag::density>>
      frameFeatureTally(boost::accumulators::tag::density::num_bins = 20,
                        boost::accumulators::tag::density::cache_size = 40);
  const std::string outputPath = simParameters.outputdir;
  std::string featureHistFile = outputPath + "/FeatureHist.txt";

  okvis::timing::Timer runTimer("Estimation run", true);

  std::string testIdentifier = swift_vio::EnumToString(vioParameters->optimization.algorithm) + "_" + simParameters.trajLabel;
  std::string pathEstimatorTrajectory = outputPath + "/" + testIdentifier;
  std::string neesFile = pathEstimatorTrajectory + "_NEES.txt";
  std::string rmseFile = pathEstimatorTrajectory + "_RMSE.txt";
  std::string metadataFile = pathEstimatorTrajectory + "_metadata.txt";
  std::string headerLine;
  std::string rmseHeaderLine;
  std::ofstream metaStream;
  metaStream.open(metadataFile, std::ofstream::out);

  createRefSensorSystem(simParameters, vioParameters->nCameraSystem);
  visualizer_ = swift_vio::VioVisualizer(*vioParameters, viewerNamePrefix_);
  visualizer_.setCameraSystem(*refCameraSystem_);
  if (publisher_)
    publisher_->setCameraSystem(*refCameraSystem_);

  loadSimulatedData(simParameters);
  LOG(INFO) << simParameters.toString();
  for (int run = 0; run < simParameters.numRuns; ++run) {
//    bool verbose = neesAccumulator.succeededRuns() == 0;
    runTimer.start();
    LOG(INFO) << "Run " << run << " " << testIdentifier << ".";

    std::stringstream ss;
    ss << run;
    std::string outputFile = pathEstimatorTrajectory + "_" + ss.str() + ".txt";

    SimFrontendOptions frontendOptions(60, vioParameters->frontendOptions.numKeyframesToMatch);
    frontendOptions.useTrueLandmarkPosition_ =
        simParameters.cameraParams.useTrueLandmarkPosition;
    frontend_.reset(new SimulationFrontend(
        simData_->homogeneousPoints(), simData_->landmarkIds(),
        refCameraSystem_->numCameras(), frontendOptions));

    createInitSensorSystem(simParameters, vioParameters);
    createEstimator(*vioParameters);

    std::ofstream debugStream;
    debugStream.open(outputFile, std::ofstream::out);
    headerLine = estimator_->headerLine();
    rmseHeaderLine = estimator_->rmseHeaderLine();

    std::vector<std::string> perturbationLabels = estimator_->perturbationLabels();

    debugStream << headerLine << std::endl;

    bool hasStarted = false;
    int frameCount = 0;     // number of frames used in estimator
    int trackedFeatures = 0; // feature tracks observed in a frame
    bool runSuccessful = true;

    simData_->resetImuBiases(refImuParameters_, "");
    simData_->rewind();
    if (publisher_)
      publisher_->rewind();

    int expectedNumFrames = simData_->expectedNumNFrames();
    neesAccumulator.refreshBuffer(expectedNumFrames);
    rmseAccumulator.refreshBuffer(expectedNumFrames);

    try {
      do {
        okvis::Time refNFrameTime = simData_->currentTime();
        okvis::kinematics::Transformation T_WS_ref = simData_->currentPose();
        Eigen::Vector3d v_WS_ref = simData_->currentVelocity();
        Eigen::Matrix<double, 6, 1> biasRef = simData_->currentBiases().toVector();
        const okvis::ImuParameters &refImuParams = refImuParameters_;

        okvis::ImuMeasurementDeque imuSegment =
            simData_->imuMeasurementsSinceLastNFrame();

        // assemble a multi-frame
        uint64_t id = okvis::IdProvider::instance().newId();
        okvis::Time frameStamp = refNFrameTime - okvis::Duration(refCameraSystem_->cameraGeometry(0)->imageDelay());
        std::shared_ptr<swift_vio::MultiFrame> mf(new swift_vio::MultiFrame(refCameraSystem_->numCameras(), frameStamp, id));

//        estimator_->getEstimatedCameraSystem(estimatedCameraSystem_.get());
        for (size_t j = 0u; j < refCameraSystem_->numCameras(); ++j) {
          mf->setTimestamp(j, frameStamp);
        }

        VLOG(1) << "Processing frame " << id << " of index " << frameCount;

        bool asKeyframe = false;
        if (!hasStarted) {
          asKeyframe = true;
        }
        if (vioParameters->frontendOptions.allAreKeyframes) {
          asKeyframe = true;
        }

        // add landmark observations
        trackedFeatures = 0;
        mf->setKeyframe(asKeyframe);
        std::shared_ptr<swift_vio::VisualMatcherOutput> featureMatches(
            new swift_vio::VisualMatcherOutput(mf, imuSegment));
        if (simParameters.cameraParams.useImageObservations) {
          std::vector<std::unordered_map<size_t, size_t>> keypointIndices;
          simData_->addFeaturesToNFrame(*refCameraSystem_, mf, &keypointIndices);
          frontend_->dataAssociation(mf, T_WS_ref, keypointIndices,
                                     featureMatches.get());
          trackedFeatures = featureMatches->featureTracks.size();
        }

        if (!hasStarted) {
          hasStarted = true;
          estimator_->initializeFromState(initialNavState_, mf);
        } else {
          frameFeatureTally(trackedFeatures);
          estimator_->estimate(featureMatches);

          swift_vio::MapPointVector removedLandmarks;
          estimator_->applyMarginalizationStrategy(removedLandmarks);
        }

        ++frameCount;

        publish<swift_vio::EstimatorBase>(estimator_, *vioParameters);
        visualize<swift_vio::EstimatorBase>(estimator_, *vioParameters,
                                            featureMatches->currentNFrame,
                                            featureMatches->closestKeyframe);

        Eigen::MatrixXd covariance;
        estimator_->computeCovariance(&covariance);

        estimator_->printStatesAndStdevs(debugStream, &covariance);

        Eigen::VectorXd errors;
        estimator_->computeErrors(T_WS_ref, v_WS_ref, biasRef, refImuParams, refCameraSystem_,
                                  &errors);
        Eigen::VectorXd squaredError = errors.cwiseAbs2();
        Eigen::VectorXd normalizedSquaredError =
            computeNormalizedErrors(errors, covariance);

        if (errors.head<3>().lpNorm<Eigen::Infinity>() > FLAGS_sim_max_position_Rmse) {
          runSuccessful = false;
        }

        neesAccumulator.push_back(refNFrameTime, normalizedSquaredError);
        rmseAccumulator.push_back(refNFrameTime, squaredError);
      } while (simData_->nextNFrame());

      Eigen::VectorXd desiredStdevs;
      estimator_->getDesiredStdevs(&desiredStdevs);
      checkMseCallback_(rmseAccumulator.lastValue(), desiredStdevs, perturbationLabels);
      checkNeesCallback_(neesAccumulator.lastValue());

      if (runSuccessful) {
        neesAccumulator.accumulate();
        rmseAccumulator.accumulate();
      }

      std::stringstream messageStream;
      messageStream << "Run " << run << " finishes with #processed frames " << frameCount
                    << " #tracked features in last frame " << trackedFeatures
                    << " #keyframes " << frontend_->numKeyframes() << ". Successful? " << runSuccessful;
      LOG(INFO) << messageStream.str();
      metaStream << messageStream.str() << std::endl;

      // output track length distribution
      std::string trackStatFile =
          pathEstimatorTrajectory + "_trackstat_" + ss.str() + ".txt";
      std::ofstream trackStatStream(trackStatFile, std::ios_base::out);
      estimator_->printTrackLengthHistogram(trackStatStream);
      trackStatStream.close();

    } catch (std::exception &e) {
      std::stringstream messageStream;
      messageStream << "Run " << run << " aborts with #processed frames "
                    << frameCount << " #tracked features in last frame "
                    << trackedFeatures << " #keyframes "
                    << frontend_->numKeyframes() << " and error: " << e.what();
      LOG(INFO) << messageStream.str();
      metaStream << messageStream.str() << std::endl;
      if (debugStream.is_open()) {
        debugStream.close();
      }
    }

    double elapsedTime = runTimer.stop();
    std::stringstream sstream;
    sstream << "Run " << run << " used " << elapsedTime << " seconds.";
    LOG(INFO) << sstream.str();
    metaStream << sstream.str() << std::endl;
  }  // next run

  HistogramType hist = boost::accumulators::density(frameFeatureTally);
  outputFeatureHistogram(featureHistFile, hist);

  int numSucceededRuns = neesAccumulator.succeededRuns();
  std::stringstream message;
  message << "#successful runs " << numSucceededRuns << " out of "
          << simParameters.numRuns << " runs.";

  std::string neesHeaderLine =
      "%state timestamp, NEES of p_WS, \\alpha_WS, T_WS, v_WS, b_g, b_a";
  neesAccumulator.computeMean();
  neesAccumulator.dump(neesFile, neesHeaderLine);

  rmseAccumulator.computeRootMean();
  rmseAccumulator.dump(rmseFile, rmseHeaderLine);

  LOG(INFO) << message.str();
  metaStream << message.str() << std::endl;
  metaStream.close();
}

void VioSimTestSystem::runFromInitializer(const simul::SimParameters &simParameters,
                                          okvis::VioParameters *vioParameters) {
  swift_vio::StatAccumulator neesAccumulator;
  swift_vio::StatAccumulator rmseAccumulator;

  srand((unsigned int)time(0)); // comment out to make tests deterministic.

  // number of features tracked in a frame.
  boost::accumulators::accumulator_set<
      double, boost::accumulators::features<boost::accumulators::tag::count,
                                            boost::accumulators::tag::density>>
      frameFeatureTally(boost::accumulators::tag::density::num_bins = 20,
                        boost::accumulators::tag::density::cache_size = 40);
  std::string featureHistFile = simParameters.outputdir + "/FeatureHist.txt";

  okvis::timing::Timer runTimer("Estimation run", true);
  okvis::timing::Timer optimizationTimer("opt in initialization", true);
  okvis::timing::Timer convertEstimatorTimer("initializer to estimator", true);

  std::string testIdentifier = swift_vio::EnumToString(vioParameters->optimization.algorithm) + "_" + simParameters.trajLabel;
  std::string pathEstimatorTrajectory = simParameters.outputdir + "/" + testIdentifier;
  std::string neesFile = pathEstimatorTrajectory + "_NEES.txt";
  std::string rmseFile = pathEstimatorTrajectory + "_RMSE.txt";
  std::string metadataFile = pathEstimatorTrajectory + "_metadata.txt";
  std::string headerLine;
  std::string rmseHeaderLine;
  std::ofstream metaStream;
  metaStream.open(metadataFile, std::ofstream::out);

  createRefSensorSystem(simParameters, vioParameters->nCameraSystem);
  visualizer_ = swift_vio::VioVisualizer(*vioParameters, viewerNamePrefix_);
  visualizer_.setCameraSystem(*refCameraSystem_);
  if (publisher_)
    publisher_->setCameraSystem(*refCameraSystem_);

  loadSimulatedData(simParameters);
  LOG(INFO) << simParameters.toString();
  for (int run = 0; run < simParameters.numRuns; ++run) {
//    bool verbose = neesAccumulator.succeededRuns() == 0;
    runTimer.start();
    LOG(INFO) << "Run " << run << " " << testIdentifier << ".";

    std::stringstream ss;
    ss << run;
    std::string outputFile = pathEstimatorTrajectory + "_" + ss.str() + ".txt";

    SimFrontendOptions frontendOptions(60, vioParameters->frontendOptions.numKeyframesToMatch);
    frontendOptions.useTrueLandmarkPosition_ =
        simParameters.cameraParams.useTrueLandmarkPosition;
    frontend_.reset(new SimulationFrontend(
        simData_->homogeneousPoints(), simData_->landmarkIds(),
        refCameraSystem_->numCameras(), frontendOptions));

    createInitSensorSystem(simParameters, vioParameters);
    createEstimator(*vioParameters);

    std::ofstream debugStream;
    debugStream.open(outputFile, std::ofstream::out);
    headerLine = estimator_->headerLine();
    rmseHeaderLine = estimator_->rmseHeaderLine();

    debugStream << headerLine << std::endl;

    int frameCount = 0;     // number of frames used in estimator
    int trackedFeatures = 0; // feature tracks observed in a frame
    bool runSuccessful = true;

    simData_->resetImuBiases(refImuParameters_, "");
    simData_->rewind();
    if (publisher_)
      publisher_->rewind();

    int expectedNumFrames = simData_->expectedNumNFrames();
    neesAccumulator.refreshBuffer(expectedNumFrames);
    rmseAccumulator.refreshBuffer(expectedNumFrames);

    std::shared_ptr<const swift_vio::EstimatorBase> estimatorToPublish;

    try {
      do {
        okvis::Time refNFrameTime = simData_->currentTime();
        okvis::kinematics::Transformation T_WS_ref = simData_->currentPose();
        Eigen::Vector3d v_WS_ref = simData_->currentVelocity();
        Eigen::Matrix<double, 6, 1> biasRef = simData_->currentBiases().toVector();
        const okvis::ImuParameters &refImuParams = refImuParameters_;

        okvis::ImuMeasurementDeque imuSegment =
            simData_->imuMeasurementsSinceLastNFrame();

        // assemble a multi-frame
        uint64_t id = okvis::IdProvider::instance().newId();
        okvis::Time frameStamp =
            refNFrameTime -
            okvis::Duration(refCameraSystem_->cameraGeometry(0)->imageDelay());
        std::shared_ptr<swift_vio::MultiFrame> mf(new swift_vio::MultiFrame(
            refCameraSystem_->numCameras(), frameStamp, id));

//        estimator_->getEstimatedCameraSystem(estimatedCameraSystem_.get());

        for (size_t j = 0u; j < refCameraSystem_->numCameras(); ++j) {
          mf->setTimestamp(j, frameStamp);
        }

        VLOG(1) << "Processing frame " << id << " of index " << frameCount;

        bool asKeyframe = false;
        if (vioParameters->frontendOptions.allAreKeyframes) {
          asKeyframe = true;
        }

        // add landmark observations
        trackedFeatures = 0;
        mf->setKeyframe(asKeyframe);
        std::shared_ptr<swift_vio::VisualMatcherOutput> featureMatches(
            new swift_vio::VisualMatcherOutput(mf, imuSegment));
        if (simParameters.cameraParams.useImageObservations) {
          std::vector<std::unordered_map<size_t, size_t>> keypointIndices;
          simData_->addFeaturesToNFrame(*refCameraSystem_, mf, &keypointIndices);
          frontend_->dataAssociation(mf, T_WS_ref, keypointIndices,
                                     featureMatches.get());
          trackedFeatures = featureMatches->featureTracks.size();
        }

        bool readyToShowMatches = true;
        if (!initializer_->wellInitialized()) {
          optimizationTimer.start();
          initializer_->estimate(featureMatches);
          readyToShowMatches = featureMatches->currentNFrame->isKeyframe();
//          Eigen::MatrixXd fullCov;
//          std::vector<uint64_t> varIdList;
//          bool res = initializer_->computeFullCovarianceCeres(
//              &fullCov, &varIdList,
//              ::ceres::CovarianceAlgorithmType::SPARSE_QR);
//          if (res) {
//            std::string outputcov;
//            std::stringstream ss;
//            ss << FLAGS_log_dir << "/cov_" << frameCount << ".txt";
//            std::ofstream fs(ss.str());
//            fs << "#rows " << fullCov.rows() << "\n";
//            fs << fullCov.format(swift_vio::kSpaceInitFmt);
//            fs.close();
//          }
          optimizationTimer.stop();

          if (initializer_->wellInitialized()) {
            convertEstimatorTimer.start();
            estimator_->initializeFrom(initializer_, featureMatches->currentNFrame);
//            initializer_->clear();
            convertEstimatorTimer.stop();
            estimatorToPublish = estimator_;
          } else {
            estimatorToPublish = initializer_;
          }
        } else {
          frameFeatureTally(trackedFeatures);
          estimator_->estimate(featureMatches);

          swift_vio::MapPointVector removedLandmarks;
          estimator_->applyMarginalizationStrategy(removedLandmarks);
          estimatorToPublish = estimator_;
        }

        ++frameCount;

        publish<swift_vio::EstimatorBase>(estimatorToPublish, *vioParameters);
        if (readyToShowMatches) {
          visualize<swift_vio::EstimatorBase>(
              estimatorToPublish, *vioParameters, featureMatches->currentNFrame,
              featureMatches->closestKeyframe);
        }

        Eigen::MatrixXd covariance;
        estimatorToPublish->computeCovariance(&covariance);

        estimatorToPublish->printStatesAndStdevs(debugStream, &covariance);

        Eigen::VectorXd errors;
        estimatorToPublish->computeErrors(T_WS_ref, v_WS_ref, biasRef, refImuParams, refCameraSystem_,
                                  &errors);
        Eigen::VectorXd squaredError = errors.cwiseAbs2();
        Eigen::VectorXd normalizedSquaredError =
            computeNormalizedErrors(errors, covariance);

        if (errors.head<3>().lpNorm<Eigen::Infinity>() > FLAGS_sim_max_position_Rmse) {
          runSuccessful = false;
        }

        neesAccumulator.push_back(refNFrameTime, normalizedSquaredError);
        rmseAccumulator.push_back(refNFrameTime, squaredError);
      } while (simData_->nextNFrame());

      Eigen::VectorXd desiredStdevs;
      estimatorToPublish->getDesiredStdevs(&desiredStdevs);
      std::vector<std::string> perturbationLabels = estimatorToPublish->perturbationLabels();
      checkMseCallback_(rmseAccumulator.lastValue(), desiredStdevs, perturbationLabels);
      checkNeesCallback_(neesAccumulator.lastValue());

      if (runSuccessful) {
        neesAccumulator.accumulate();
        rmseAccumulator.accumulate();
      }

      std::stringstream messageStream;
      messageStream << "Run " << run << " finishes with #processed frames " << frameCount
                    << " #tracked features in last frame " << trackedFeatures
                    << " #keyframes " << frontend_->numKeyframes() << ". Successful? " << runSuccessful;
      LOG(INFO) << messageStream.str();
      metaStream << messageStream.str() << std::endl;

      // output track length distribution
      std::string trackStatFile =
          pathEstimatorTrajectory + "_trackstat_" + ss.str() + ".txt";
      std::ofstream trackStatStream(trackStatFile, std::ios_base::out);
      estimatorToPublish->printTrackLengthHistogram(trackStatStream);
      trackStatStream.close();
    } catch (std::exception &e) {
      std::stringstream messageStream;
      messageStream << "Run " << run << " aborts with #processed frames "
                    << frameCount << " #tracked features in last frame "
                    << trackedFeatures << " #keyframes "
                    << frontend_->numKeyframes() << " and error: " << e.what();
      LOG(INFO) << messageStream.str();
      metaStream << messageStream.str() << std::endl;
      if (debugStream.is_open()) {
        debugStream.close();
      }
    }

    double elapsedTime = runTimer.stop();
    std::stringstream sstream;
    sstream << "Run " << run << " used " << elapsedTime << " seconds.";
    LOG(INFO) << sstream.str();
    metaStream << sstream.str() << std::endl;
  }  // next run

  HistogramType hist = boost::accumulators::density(frameFeatureTally);
  outputFeatureHistogram(featureHistFile, hist);

  int numSucceededRuns = neesAccumulator.succeededRuns();
  std::stringstream message;
  message << "#successful runs " << numSucceededRuns << " out of "
          << simParameters.numRuns << " runs.";

  std::string neesHeaderLine =
      "%state timestamp, NEES of p_WS, \\alpha_WS, T_WS, v_WS, b_g, b_a";
  neesAccumulator.computeMean();
  neesAccumulator.dump(neesFile, neesHeaderLine);

  rmseAccumulator.computeRootMean();
  rmseAccumulator.dump(rmseFile, rmseHeaderLine);

  LOG(INFO) << message.str();
  metaStream << message.str() << std::endl;
  metaStream.close();
}

void VioSimTestSystem::checkTriangulation(int landmarkModelId) {
  std::shared_ptr<
      okvis::ceres::LocalParamizationAdditionalInterfaces>
      landmarkParameterizationPtr =
          swift_vio::createLandmarkLocalParameterization(
              landmarkModelId);

  swift_vio::PointMap landmarkMap;
  estimator_->getLandmarks(landmarkMap);

  std::shared_ptr<swift_vio::SlidingWindowFilter> filter =
      std::static_pointer_cast<swift_vio::SlidingWindowFilter>(estimator_);

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (auto iter = landmarkMap.begin(); iter != landmarkMap.end(); ++iter) {
    if (dis(gen) < 0.9) {
      continue;
    }
    std::shared_ptr<swift_vio::PointSharedData> pointDataPtr(
        new swift_vio::PointSharedData());
    Eigen::AlignedVector<Eigen::Vector2d> obsList;
    Eigen::AlignedVector<Eigen::Vector2d> obsStdList;

    const swift_vio::MapPoint &mp = iter->second;
    swift_vio::PointLandmark pointLandmark(
        mp.id, landmarkModelId,
        landmarkParameterizationPtr.get());
    std::vector<uint64_t> selectedObservations;
    selectedObservations.reserve(mp.observations.size());
    for (const auto &obs : mp.observations) {
      selectedObservations.push_back(obs.first.frameId);
    }
    swift_vio::TriangulationStatus status = filter->triangulateMapPoint(
        mp, &obsList, &pointLandmark, &obsStdList,
        pointDataPtr.get(), &selectedObservations, false);
    Eigen::Vector4d hpW = simData_->landmarkInWorld(mp.id);
    ++checkedLmks_;
    if (!status.triangulationOk) {
      LOG(WARNING) << "Failed triangulation for ref " << hpW.transpose();
      ++failedTriangulationLmks_;
      continue;
    }
    // compare with the reference position
    Eigen::Vector4d hpEst = pointLandmark.estimate();
    Eigen::Vector4d hpWEst;
    if (landmarkModelId == swift_vio::InverseDepthParameterization::kModelId) {
      // anchor id is the last frame and last camera
      const std::vector<swift_vio::AnchorFrameIdentifier>& anchorIds = pointDataPtr->anchorIds();
      uint64_t anchorFrameId = anchorIds[0].frameId_;
      size_t anchorCameraId = anchorIds[0].cameraIndex_;
      okvis::kinematics::Transformation T_WBa;
      estimator_->get_T_WS(anchorFrameId, T_WBa);
      okvis::kinematics::Transformation T_BCa = refCameraSystem_->getCameraExtrinsic(anchorCameraId);
      hpEst /= hpEst[3];
      hpWEst = T_WBa * T_BCa * hpEst;
    } else {
      hpWEst = hpEst;
    }
    if ((hpWEst - hpW).lpNorm<Eigen::Infinity>() > 5e-4) {
      LOG(WARNING) << "Poor triangulation: ref " << hpW.transpose()
                   << " est " << hpWEst.transpose() << " diff " << (hpW - hpWEst).transpose();
      ++poorTriangulationLmks_;
    } else {
      ++goodTriangulationLmks_;
    }
  }
}

void VioSimTestSystem::printTriangulationSummary() const {
  LOG(INFO) << "Triangulation checked landmarks " << checkedLmks_ << ", failed "
            << failedTriangulationLmks_ << ", poor " << poorTriangulationLmks_
            << ", good " << goodTriangulationLmks_ << ".";
}

void VioSimTestSystem::printJacobianSummary() const {
  LOG(INFO) << "Measurement Jacobian checked landmarks " << checkedJacobianLmks_
            << ", landmarks of different Jacobian status " << diffJacStatusLmks_
            << ", landmarks of poor Jacobians " << poorJacobianLmks_
            << ", good Jacobian landmarks " << goodJacobianLmks_ << ".";
}

void VioSimTestSystem::checkMeasurementJacobian(int landmarkModelId) {
  std::shared_ptr<
      okvis::ceres::LocalParamizationAdditionalInterfaces>
      landmarkParameterizationPtr =
          swift_vio::createLandmarkLocalParameterization(
              landmarkModelId);

  swift_vio::PointMap landmarkMap;
  estimator_->getLandmarks(landmarkMap);

  std::shared_ptr<swift_vio::SlidingWindowFilter> filter =
      std::static_pointer_cast<swift_vio::SlidingWindowFilter>(estimator_);

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(0.0, 1.0);
  for (auto iter = landmarkMap.begin(); iter != landmarkMap.end(); ++iter) {
    if (dis(gen) < 0.9) {
      continue;
    }

    std::shared_ptr<swift_vio::PointSharedData> pointDataPtr(
        new swift_vio::PointSharedData());
    Eigen::AlignedVector<Eigen::Vector2d> obsList;
    Eigen::AlignedVector<Eigen::Vector2d> obsStdList;

    const swift_vio::MapPoint &mp = iter->second;
    swift_vio::PointLandmark pointLandmark(
        mp.id, landmarkModelId,
        landmarkParameterizationPtr.get());
    std::vector<uint64_t> selectedObservations;
    selectedObservations.reserve(mp.observations.size());
    for (const auto &obs : mp.observations) {
      selectedObservations.push_back(obs.first.frameId);
    }

    auto status = filter->triangulateMapPoint(
        mp, &obsList, &pointLandmark, &obsStdList,
        pointDataPtr.get(), &selectedObservations, false);

    if (!status.triangulationOk) {
      continue;
    }

    pointDataPtr->computePoseAndVelocityForJacobians();

    int stateMapSize = estimator_->statesMapSize();
    int featureVariableDim =
        filter->minimalDimOfAllCameraParams() +
        filter->kClonedStateMinimalDim * stateMapSize;

    // Jacobians computed without aliasing the camera projection Jacobians.
    // analyticDiff computes Jacobians by using state linearization points.
    Eigen::Matrix<double, 2, Eigen::Dynamic> J_x(2, featureVariableDim);
    Eigen::Matrix<double, 2, 3> J_pfi;
    Eigen::Vector2d residual;
    size_t observationIndex = std::rand() % 2;
    bool result = filter->measurementJacobian(
          pointLandmark, obsList[observationIndex], observationIndex,
          *pointDataPtr, &J_x, &J_pfi, &residual, false);

    // autoDiff computes Jacobians using state estimates.
    Eigen::Matrix<double, 2, Eigen::Dynamic> J_x_auto(2, featureVariableDim);
    Eigen::Matrix<double, 2, 3> J_pfi_auto(2, 3);
    Eigen::Vector2d residualAuto(2, 1);
    bool resultAuto = filter->measurementJacobianAutoDiff(
          pointLandmark, obsList[observationIndex], observationIndex,
          *pointDataPtr, &J_x_auto, &J_pfi_auto, &residualAuto);

    // Jacobians computed by aliasing the camera projection Jacobians.
    // analyticDiff computes Jacobians by using state linearization points.
    Eigen::Matrix<double, 2, Eigen::Dynamic> J_x_mix(2, featureVariableDim);
    Eigen::Matrix<double, 2, 3> J_pfi_mix;
    Eigen::Vector2d residualMix;
    bool resultMix = filter->measurementJacobian(
          pointLandmark, obsList[observationIndex], observationIndex,
          *pointDataPtr, &J_x_mix, &J_pfi_mix, &residualMix, true);

    // numericDiff computes Jacobians using state linearization points.
    Eigen::Matrix<double, 2, Eigen::Dynamic> J_x_num(2, featureVariableDim);
    Eigen::Matrix<double, 2, 3> J_pfi_num;
    Eigen::Vector2d residualNum;
    bool resultNum = filter->measurementJacobianNumeric(
          pointLandmark, obsList[observationIndex], observationIndex,
          *pointDataPtr, &J_x_num, &J_pfi_num, &residualNum);

    ++checkedJacobianLmks_;
    if (!(result && resultAuto && resultMix && resultNum)) {
      LOG(WARNING) << "Different measurement Jacobian results.";
      ++diffJacStatusLmks_;
    } else {
//    const auto& anchorIds = pointDataPtr->anchorIds();
//    if (anchorIds.size()) {
//      LOG(INFO) << "Camera index: target " << pointDataPtr->cameraIndex(observationIndex)
//                << " host " << anchorIds[0].cameraIndex_;
//    }
      std::shared_ptr<bool> res1(new bool(false));
      ARE_MATRICES_CLOSE_IN_VALUE(residualAuto, residual, 1e-3, *res1);

      std::shared_ptr<bool> res2(new bool(false));
      ARE_MATRICES_CLOSE_IN_VALUE(J_pfi_auto, J_pfi, 1e-3, *res2);

      std::shared_ptr<bool> res3(new bool(false));
      ARE_MATRICES_CLOSE_IN_VALUE(J_x_auto, J_x, 1e-3, *res3);

      std::shared_ptr<bool> res4(new bool(false));
      ARE_MATRICES_CLOSE_IN_VALUE(J_pfi_num, J_pfi, 5e-3, *res4);

  //      std::shared_ptr<bool> res5(new bool(false));
  //      ARE_MATRICES_CLOSE_IN_VALUE(J_x_num, J_x, 1e-3, *res5);

      if (!(*res1) || !(*res2) || !(*res3)) {
        ++poorJacobianLmks_;
      } else {
        ++goodJacobianLmks_;
      }
    }
  }
}

void VioSimTestSystem::runNoiseFree(const simul::SimParameters &simParameters,
                                    okvis::VioParameters *vioParameters) {
  srand((unsigned int)time(0)); // comment out to make tests deterministic.
  okvis::timing::Timer runTimer("Estimation run", true);
  std::string testIdentifier =
      swift_vio::EnumToString(vioParameters->optimization.algorithm) + "_" +
      simParameters.trajLabel;
  const std::string outputPath = simParameters.outputdir;
  std::string pathEstimatorTrajectory = outputPath + "/" + testIdentifier;
  std::string headerLine;

  createRefSensorSystem(simParameters, vioParameters->nCameraSystem);
  visualizer_ = swift_vio::VioVisualizer(*vioParameters, viewerNamePrefix_);
  visualizer_.setCameraSystem(*refCameraSystem_);
  if (publisher_)
    publisher_->setCameraSystem(*refCameraSystem_);

  simData_ = std::shared_ptr<SimulatorBase>(new CurveData(
      simParameters, refImuParameters_, simParameters.duration,
      simParameters.cameraParams.addExtraLandmarks,
      simParameters.cameraParams.addImageNoise,
      simParameters.imuParams.addImuNoise));

  LOG(INFO) << simParameters.toString();

  simData_->initializeLandmarkGrid(
      simParameters.cameraParams.landmarkDistribution,
      simParameters.cameraParams.landmarkCylinderRadius, 5.0,
      refCameraSystem_.get());

  std::string pointFile =
      outputPath + "/" + simParameters.trajLabel + "_Points.txt";
  simData_->saveLandmarkGrid(pointFile);

  std::string imuSampleFile =
      outputPath + "/" + simParameters.trajLabel + "_IMU.txt";
  simData_->resetImuBiases(refImuParameters_, imuSampleFile);

  std::string truthFile = outputPath + "/" + simParameters.trajLabel + ".txt";
  simData_->saveRefMotion(truthFile);

  std::string cameraFile = outputPath + "/cameraSystem.txt";
  saveCameraParameters(refCameraSystem_, cameraFile);

  for (int run = 0; run < simParameters.numRuns; ++run) {
    runTimer.start();
    LOG(INFO) << "Run " << run << " " << testIdentifier << ".";

    std::stringstream ss;
    ss << run;
    std::string outputFile = pathEstimatorTrajectory + "_" + ss.str() + ".txt";

    SimFrontendOptions frontendOptions(
        60, vioParameters->frontendOptions.numKeyframesToMatch);
    frontendOptions.useTrueLandmarkPosition_ =
        simParameters.cameraParams.useTrueLandmarkPosition;
    frontend_.reset(new SimulationFrontend(
        simData_->homogeneousPoints(), simData_->landmarkIds(),
        refCameraSystem_->numCameras(), frontendOptions));
    createInitSensorSystem(simParameters, vioParameters);
    createEstimator(*vioParameters);

    std::ofstream debugStream;
    debugStream.open(outputFile, std::ofstream::out);
    headerLine = estimator_->headerLine();

    std::vector<std::string> perturbationLabels =
        estimator_->perturbationLabels();
    debugStream << headerLine << std::endl;

    bool hasStarted = false;
    int frameCount = 0;      // number of frames used in estimator
    int trackedFeatures = 0; // feature tracks observed in a frame
    bool runSuccessful = true;

    simData_->resetImuBiases(refImuParameters_, "");
    simData_->rewind();
    if (publisher_)
      publisher_->rewind();

    try {
      do {
        okvis::Time refNFrameTime = simData_->currentTime();
        okvis::kinematics::Transformation T_WS_ref = simData_->currentPose();
        Eigen::Vector3d v_WS_ref = simData_->currentVelocity();
        Eigen::Matrix<double, 6, 1> biasRef =
            simData_->currentBiases().toVector();
        const okvis::ImuParameters &refImuParams = refImuParameters_;
        okvis::ImuMeasurementDeque imuSegment =
            simData_->imuMeasurementsSinceLastNFrame();

        // assemble a multi-frame
        uint64_t id = okvis::IdProvider::instance().newId();
        okvis::Time frameStamp =
            refNFrameTime -
            okvis::Duration(refCameraSystem_->cameraGeometry(0)->imageDelay());
        std::shared_ptr<swift_vio::MultiFrame> mf(new swift_vio::MultiFrame(
            refCameraSystem_->numCameras(), frameStamp, id));
        //          estimator_->getEstimatedCameraSystem(estimatedCameraSystem_.get());
        for (size_t j = 0u; j < refCameraSystem_->numCameras(); ++j) {
          mf->setTimestamp(j, frameStamp);
        }

        VLOG(1) << "Processing frame " << id << " of index " << frameCount;

        bool asKeyframe = false;
        if (!hasStarted) {
          asKeyframe = true;
        }
        if (vioParameters->frontendOptions.allAreKeyframes) {
          asKeyframe = true;
        }

        // add landmark observations
        trackedFeatures = 0;
        mf->setKeyframe(asKeyframe);
        std::shared_ptr<swift_vio::VisualMatcherOutput> featureMatches(
            new swift_vio::VisualMatcherOutput(mf, imuSegment));
        if (simParameters.cameraParams.useImageObservations) {
          std::vector<std::unordered_map<size_t, size_t>> keypointIndices;
          simData_->addFeaturesToNFrame(*refCameraSystem_, mf, &keypointIndices);
          frontend_->dataAssociation(mf, T_WS_ref, keypointIndices,
                                     featureMatches.get());
        }

        if (!hasStarted) {
          hasStarted = true;
          estimator_->initializeFromState(initialNavState_, mf);
        } else {
          estimator_->estimate(featureMatches);
          swift_vio::MapPointVector removedLandmarks;
          estimator_->applyMarginalizationStrategy(removedLandmarks);
        }
        if (frameCount > 3) {
          if (simParameters.checkTriangulation) {
            checkTriangulation(
                vioParameters->pointLandmarkOptions.landmarkModelId);
          }
          if (simParameters.checkMeasurementJacobian) {
            checkMeasurementJacobian(
                vioParameters->pointLandmarkOptions.landmarkModelId);
          }
        }

        ++frameCount;

        Eigen::MatrixXd covariance;
        estimator_->computeCovariance(&covariance);

        estimator_->printStatesAndStdevs(debugStream, &covariance);

        Eigen::VectorXd errors;
        estimator_->computeErrors(T_WS_ref, v_WS_ref, biasRef, refImuParams,
                                  refCameraSystem_, &errors);
        Eigen::VectorXd squaredError = errors.cwiseAbs2();
        Eigen::VectorXd normalizedSquaredError =
            computeNormalizedErrors(errors, covariance);

        if (errors.head<3>().lpNorm<Eigen::Infinity>() >
            FLAGS_sim_max_position_Rmse) {
          runSuccessful = false;
        }

        if (frameCount > 15) {
          break;
        }
      } while (simData_->nextNFrame());

      if (simParameters.checkTriangulation) {
        printTriangulationSummary();
      }
      if (simParameters.checkMeasurementJacobian) {
        printJacobianSummary();
      }

      Eigen::VectorXd desiredStdevs;
      estimator_->getDesiredStdevs(&desiredStdevs);

      std::stringstream messageStream;
      messageStream << "Run " << run << " finishes with #processed frames "
                    << frameCount << " #tracked features in last frame "
                    << trackedFeatures << " #keyframes "
                    << frontend_->numKeyframes() << ". Successful? "
                    << runSuccessful;
      LOG(INFO) << messageStream.str();

      // output track length distribution
      std::string trackStatFile =
          pathEstimatorTrajectory + "_trackstat_" + ss.str() + ".txt";
      std::ofstream trackStatStream(trackStatFile, std::ios_base::out);
      estimator_->printTrackLengthHistogram(trackStatStream);
      trackStatStream.close();
    } catch (std::exception &e) {
      std::stringstream messageStream;
      messageStream << "Run " << run << " aborts with #processed frames "
                    << frameCount << " #tracked features in last frame "
                    << trackedFeatures << " #keyframes "
                    << frontend_->numKeyframes() << " and error: " << e.what();
      LOG(INFO) << messageStream.str();
      if (debugStream.is_open()) {
        debugStream.close();
      }
    }
    double elapsedTime = runTimer.stop();
    std::stringstream sstream;
    sstream << "Run " << run << " used " << elapsedTime << " seconds.";
    LOG(INFO) << sstream.str();
  } // next run
}
} // namespace simul
