
/**
 * @file VioSystem.cpp
 * @brief Source file for the VioSystem class.
 */

#include <swift_vio/VioSystem.hpp>
#include <fstream>
#include <map>

#include <Eigen/Core>
#include <glog/logging.h>

#include <okvis/assert_macros.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <loop_closure/KeyframeForLoopDetection.hpp>

#include <swift_vio/checkSensorRig.hpp>
#include <swift_vio/imu/BoundedImuDeque.hpp>
#include <swift_vio/imu/ImuOdometry.h>
#include <swift_vio/TimingUtilities.h>

namespace swift_vio {

static const int max_camera_input_queue_size = 10;

// overlap of imu data before and after two consecutive frames [seconds].
// Upon receiving a multiframe, the frontend waits until
// max(frame k time + camera k delay + camera k readout time * 0.5) + temporal_imu_data_overlap
// for the corresponding IMU data.

static const okvis::Duration temporal_imu_data_overlap(0.02);

VioSystem::VioSystem(
    okvis::VioParameters &parameters,
    std::shared_ptr<EstimatorBase> initializer,
    std::shared_ptr<EstimatorBase> estimator,
    std::shared_ptr<CameraFrontendBase> frontend)
    : speedAndBiases_propagated_(okvis::SpeedAndBias::Zero()),
      imu_params_(parameters.imu), repropagationNeeded_(false),
      frameSynchronizer_(parameters),
      lastAddedImageTimestamp_(okvis::Time(0, 0)), initializer_(initializer),
      estimator_(estimator), frontend_(frontend),
      parameters_(parameters),
      viewerNamePrefix_("Reprojected landmarks for camera"),
      visualizer_(parameters_, viewerNamePrefix_),
      maxImuInputQueueSize_(2 * max_camera_input_queue_size *
                            parameters.imu.rate /
                            parameters.sensors_information.cameraRate) {
  setBlocking(false);
  init();
  startThreads();
}

void VioSystem::setBlocking(bool blocking) {
  blocking_ = blocking;

  // disable time limit for optimization
  if(blocking_) {
    optTimeLimitMillisec = -1001;
  } else {
    optTimeLimitMillisec = 30;
  }
}

// Initialises settings and calls startThreads().
void VioSystem::init() {
  assert(parameters_.nCameraSystem.numCameras() > 0);
  numCameras_ = parameters_.nCameraSystem.numCameras();
  for (size_t i = 0; i < numCameras_; ++i) {
    cameraMeasurementsReceived_.emplace_back(
          std::shared_ptr<okvis::threadsafe::ThreadSafeQueue<std::shared_ptr<okvis::CameraMeasurement> > >
          (new okvis::threadsafe::ThreadSafeQueue<std::shared_ptr<okvis::CameraMeasurement> >()));
  }
  numCameraPairs_ = 1;

  configureCameraFrontend(parameters_.nCameraSystem, frontend_);

  lastOptimizedCameraSystem_ = CameraRig::deepCopyPtr(parameters_.nCameraSystem);
  lastOptimizedImuRig_.reset(new ImuRig());
  lastOptimizedImuRig_->addImu(parameters_.imu);
  lastOptimizedStateTimestamp_ = okvis::Time(0.0);
  lastAddedStateTimestamp_ = okvis::Time(0.0);

  swift_vio::doesExtrinsicRepFitImuModel(parameters_.nCameraSystem.extrinsicRep(0u),
                                parameters_.imu.model_name);
  swift_vio::doesExtrinsicRepFitOkvisBackend(parameters_.nCameraSystem,
                                    parameters_.optimization.algorithm);

  configureEstimator(parameters_, initializer_);
  // This is necessary because the publishing callback requires a header row
  // which in turn requires the IMU parameters.
  configureEstimator(parameters_, estimator_);


  // set up windows so things don't crash on Mac OS
  if(parameters_.visualization.displayImages){
    for (size_t im = 0; im < parameters_.nCameraSystem.numCameras(); im++) {
      std::stringstream windowname;
      windowname << viewerNamePrefix_ << " " << im;
  	  cv::namedWindow(windowname.str());
    }
  }
}


// Start all threads.
void VioSystem::startThreads() {
  // consumer threads
  for (size_t i = 0; i < numCameras_; ++i) {
    frameConsumerThreads_.emplace_back(&VioSystem::frameConsumerLoop, this, i);
  }
  for (size_t i = 0; i < numCameraPairs_; ++i) {
    keypointConsumerThreads_.emplace_back(&VioSystem::matchingLoop, this);
  }
  imuConsumerThread_ = std::thread(&VioSystem::imuConsumerLoop, this);
  positionConsumerThread_ = std::thread(&VioSystem::positionConsumerLoop,
                                        this);
  gpsConsumerThread_ = std::thread(&VioSystem::gpsConsumerLoop, this);
  magnetometerConsumerThread_ = std::thread(
      &VioSystem::magnetometerConsumerLoop, this);
  differentialConsumerThread_ = std::thread(
      &VioSystem::differentialConsumerLoop, this);

  // algorithm threads
  visualizationThread_ = std::thread(&VioSystem::visualizationLoop, this);
  optimizationThread_ = std::thread(&VioSystem::optimizationLoop, this);
  publisherThread_ = std::thread(&VioSystem::publisherLoop, this);

}

// Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
VioSystem::~VioSystem() {
  for (size_t i = 0; i < numCameras_; ++i) {
    cameraMeasurementsReceived_.at(i)->Shutdown();
  }
  keypointMeasurements_.Shutdown();
  matchedFrames_.Shutdown();
  imuMeasurementsReceived_.Shutdown();
  optimizationResults_.Shutdown();
  visualizationData_.Shutdown();
  imuFrameSynchronizer_.shutdown();
  positionMeasurementsReceived_.Shutdown();

  // consumer threads
  for (size_t i = 0; i < numCameras_; ++i) {
    frameConsumerThreads_.at(i).join();
  }
  for (size_t i = 0; i < numCameraPairs_; ++i) {
    keypointConsumerThreads_.at(i).join();
  }
  imuConsumerThread_.join();
  positionConsumerThread_.join();
  gpsConsumerThread_.join();
  magnetometerConsumerThread_.join();
  differentialConsumerThread_.join();
  visualizationThread_.join();
  optimizationThread_.join();
  publisherThread_.join();

#ifndef DEACTIVATE_TIMERS
  LOG(INFO) << okvis::timing::Timing::print();
#endif
}

// Add a new image.
bool VioSystem::addImage(const okvis::Time & stamp, size_t cameraIndex,
                             const cv::Mat & image,
                             const std::vector<cv::KeyPoint> * keypoints,
                             bool* /*asKeyframe*/) {
  assert(cameraIndex<numCameras_);

  if (lastAddedImageTimestamp_ > stamp
      && fabs((lastAddedImageTimestamp_ - stamp).toSec())
          > parameters_.sensors_information.frameTimestampTolerance) {
    LOG(ERROR)
        << "Received image from the past. Dropping the image.";
    return false;
  }
  lastAddedImageTimestamp_ = stamp;

  std::shared_ptr<okvis::CameraMeasurement> frame = std::make_shared<
      okvis::CameraMeasurement>();
  frame->measurement.image = image;
  frame->timeStamp = stamp;
  frame->sensorId = cameraIndex;

  if (keypoints != nullptr) {
    frame->measurement.deliversKeypoints = true;
    frame->measurement.keypoints = *keypoints;
  } else {
    frame->measurement.deliversKeypoints = false;
  }

  if (blocking_) {
    cameraMeasurementsReceived_[cameraIndex]->PushBlockingIfFull(frame, 1);
    return true;
  } else {
    cameraMeasurementsReceived_[cameraIndex]->PushNonBlockingDroppingIfFull(
        frame, max_camera_input_queue_size);
    size_t measSize = cameraMeasurementsReceived_[cameraIndex]->Size();
    if (measSize * 2 > max_camera_input_queue_size) {
      LOG(WARNING) << "Exceptional camera meas size " << measSize;
    }
    return measSize == 1;
  }
}

// Add an abstracted image observation.
bool VioSystem::addKeypoints(
    const okvis::Time & /*stamp*/, size_t /*cameraIndex*/,
    const std::vector<cv::KeyPoint> & /*keypoints*/,
    const std::vector<uint64_t> & /*landmarkIds*/,
    const cv::Mat & /*descriptors*/,
    bool* /*asKeyframe*/) {
  OKVIS_THROW(
      Exception,
      "VioSystem::addKeypoints() not implemented anymore since changes to _keypointMeasurements queue.");
  return false;
}

// Add an IMU measurement.
bool VioSystem::addImuMeasurement(const okvis::Time & stamp,
                                      const Eigen::Vector3d & alpha,
                                      const Eigen::Vector3d & omega) {

  okvis::ImuMeasurement imu_measurement;
  imu_measurement.measurement.accelerometers = alpha;
  imu_measurement.measurement.gyroscopes = omega;
  imu_measurement.timeStamp = stamp;

  if (blocking_) {
    imuMeasurementsReceived_.PushBlockingIfFull(imu_measurement, 1);
    return true;
  } else {
    imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(
        imu_measurement, maxImuInputQueueSize_);
    return imuMeasurementsReceived_.Size() == 1;
  }
}

// Add a position measurement.
void VioSystem::addPositionMeasurement(const okvis::Time & stamp,
                                           const Eigen::Vector3d & position,
                                           const Eigen::Vector3d & positionOffset,
                                           const Eigen::Matrix3d & positionCovariance) {
  okvis::PositionMeasurement position_measurement;
  position_measurement.measurement.position = position;
  position_measurement.measurement.positionOffset = positionOffset;
  position_measurement.measurement.positionCovariance = positionCovariance;
  position_measurement.timeStamp = stamp;

  if (blocking_) {
    positionMeasurementsReceived_.PushBlockingIfFull(position_measurement, 1);
    return;
  } else {
    positionMeasurementsReceived_.PushNonBlockingDroppingIfFull(
        position_measurement, maxPositionInputQueueSize_);
    return;
  }
}

// Add a GPS measurement.
void VioSystem::addGpsMeasurement(const okvis::Time &, double, double,
                                      double, const Eigen::Vector3d &,
                                      const Eigen::Matrix3d &) {
  OKVIS_THROW(Exception, "GPS measurements not supported")
}

// Add a magnetometer measurement.
void VioSystem::addMagnetometerMeasurement(const okvis::Time &,
                                               const Eigen::Vector3d &, double) {
  OKVIS_THROW(Exception, "Magnetometer measurements not supported")
}

// Add a static pressure measurement.
void VioSystem::addBarometerMeasurement(const okvis::Time &, double, double) {

  OKVIS_THROW(Exception, "Barometer measurements not supported")
}

// Add a differential pressure measurement.
void VioSystem::addDifferentialPressureMeasurement(const okvis::Time &,
                                                       double, double) {

  OKVIS_THROW(Exception, "Differential pressure measurements not supported")
}

// Loop to process frames from camera with index cameraIndex
void VioSystem::frameConsumerLoop(size_t cameraIndex) {
  okvis::TimerSwitchable beforeDetectTimer("1.1 frameLoopBeforeDetect"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable waitForFrameSynchronizerMutexTimer("1.1.1 waitForFrameSynchronizerMutex"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable addNewFrameToSynchronizerTimer("1.1.2 addNewFrameToSynchronizer"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable waitForStateVariablesMutexTimer("1.1.3 waitForStateVariablesMutex"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable propagationTimer("1.1.4 propagationTimer"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable detectTimer("1.2 detectAndDescribe"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable afterDetectTimer("1.3 afterDetect"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable waitForFrameSynchronizerMutexTimer2("1.3.1 waitForFrameSynchronizerMutex2"+std::to_string(cameraIndex),true);
  okvis::TimerSwitchable waitForMatchingThreadTimer("1.4 waitForMatchingThread"+std::to_string(cameraIndex),true);

  FrameDescriptionTimerRefs timerRefs(beforeDetectTimer, waitForFrameSynchronizerMutexTimer,
                                   addNewFrameToSynchronizerTimer, waitForStateVariablesMutexTimer, propagationTimer,
                                   detectTimer, afterDetectTimer, waitForFrameSynchronizerMutexTimer2, waitForMatchingThreadTimer);

  for (;;) {
   bool terminate = extractVisualFeatures(cameraIndex, timerRefs);
   if (terminate) {
       return;
   }
  }
}

bool VioSystem::extractVisualFeatures(size_t cameraIndex, FrameDescriptionTimerRefs &timerRefs) {
  std::shared_ptr<okvis::CameraMeasurement> frame;
  std::shared_ptr<MultiFrame> multiFrame;
  // get data and check for termination request
  if (cameraMeasurementsReceived_[cameraIndex]->PopBlocking(&frame) == false) {
    return true;
  }
  timerRefs.beforeDetectTimer.start();
  { // lock the frame synchronizer
    timerRefs.waitForFrameSynchronizerMutexTimer.start();
    std::lock_guard<std::mutex> lock(frameSynchronizer_mutex_);
    timerRefs.waitForFrameSynchronizerMutexTimer.stop();
    // add new frame to frame synchronizer and get the MultiFrame containing it
    timerRefs.addNewFrameToSynchronizerTimer.start();
    multiFrame = frameSynchronizer_.addNewFrame(frame);
    timerRefs.addNewFrameToSynchronizerTimer.stop();
  } // unlock frameSynchronizer only now as we can be sure that not two states
    // are added for the same timestamp
  timerRefs.beforeDetectTimer.stop();

  timerRefs.detectTimer.start();
  frontend_->detectAndDescribe(frame->sensorId, multiFrame);
  timerRefs.detectTimer.stop();

  timerRefs.afterDetectTimer.start();
  bool push = false;
  { // we now tell frame synchronizer that detectAndDescribe is done for MF with
    // our timestamp
    timerRefs.waitForFrameSynchronizerMutexTimer2.start();
    std::lock_guard<std::mutex> lock(frameSynchronizer_mutex_);
    timerRefs.waitForFrameSynchronizerMutexTimer2.stop();
    frameSynchronizer_.detectionEndedForMultiFrame(multiFrame->id());

    if (frameSynchronizer_.detectionCompletedForAllCameras(multiFrame->id())) {
      //        LOG(INFO) << "detection completed for multiframe with id "<<
      //        multi_frame->id();
      push = true;
    }
  } // unlocking frame synchronizer
  timerRefs.afterDetectTimer.stop();
  if (push) {
    // use queue size 1 to propagate a congestion to the
    // _cameraMeasurementsReceived queue and check for termination request

    timerRefs.waitForMatchingThreadTimer.start();
    if (keypointMeasurements_.PushBlockingIfFull(multiFrame, 1) == false) {
      timerRefs.waitForMatchingThreadTimer.stop();
      return true;
    }
    timerRefs.waitForMatchingThreadTimer.stop();
  }
  return false;
}

// Loop that matches frames with existing frames.
void VioSystem::matchingLoop() {
  okvis::TimerSwitchable copyLatestStateTimer("2.1 copyLatestState",true);
  okvis::TimerSwitchable waitForImuDataTimer("2.2 waitForImuData",true);
  okvis::TimerSwitchable getSubmapTimer("2.3 getSubmap",true);
  okvis::TimerSwitchable matchingTimer("2.4 matching",true);

  VisualMatchingTimerRefs timerRefs(copyLatestStateTimer, waitForImuDataTimer, getSubmapTimer, matchingTimer);

  for (;;) {
    bool terminate = matchVisualFeatures(timerRefs);
    if (terminate) {
      return;
    }
  }
}

bool VioSystem::matchVisualFeatures(VisualMatchingTimerRefs &timerRefs) {
  std::shared_ptr<MultiFrame> frame;
  // get data and check for termination request
  if (keypointMeasurements_.PopBlocking(&frame) == false)
    return true;

  // get latest received IMU data which will be used by the estimator to predict poses at image features.
  // To ensure the estimator receive all IMU data, the IMU data need to start from
  // lastAddedStateTimestamp with an overlap.
  // To ensure pose for image features can be predicted, the IMU data should end at the latest frame time.
  // After all, relevant IMU data should have been available because of the wait in frameConsumerLoop.

  okvis::kinematics::Transformation T_WS;
  Eigen::Vector3d v_WS, bg, omega_S;
  okvis::Time lastStateTime;

  timerRefs.copyLatestStateTimer.start();
  {
    std::lock_guard<std::mutex> lock(lastState_mutex_);
    // Use the optimized camera system for the upcoming feature association.
    frontend_->updateCameraSystem(lastOptimizedCameraSystem_);

    T_WS = lastOptimized_T_WS_;
    v_WS = lastOptimizedSpeed_;
    bg = lastOptimizedImuRig_->at(0).getGyroBias();
    lastStateTime = lastOptimizedStateTimestamp_;
  }
  timerRefs.copyLatestStateTimer.stop();

  okvis::Time imuDataEndTime = TimingUtilities::computeImuDataWaitUntilTime(
      frontend_->cameraSystem(), frame, temporal_imu_data_overlap);

  okvis::Time imuDataBeginTime;
  if (lastAddedStateTimestamp_ == okvis::Time()) {  // first state not yet added
    imuDataBeginTime = okvis::minusSafe(frame->timestamp(), EstimatorBase::half_window_);
  } else {
    imuDataBeginTime = lastAddedStateTimestamp_ - EstimatorBase::half_window_;
  }

  if (imuDataEndTime - imuDataBeginTime > okvis::Duration(8)) {
    LOG(WARNING) << "Warn: Too long interval between two frames "
                 << lastAddedStateTimestamp_.toSec() << " and "
                 << frame->timestamp().toSec();
  }
  OKVIS_ASSERT_TRUE_DBG(Exception, imuDataBeginTime < imuDataEndTime,
                        "imu data end time is smaller than begin time."
                            << "current frametimestamp: " << frame->timestamp()
                            << " id: " << frame->id());

  timerRefs.waitForImuDataTimer.start();
  // wait until all relevant imu messages have arrived and check for termination request
  if (imuFrameSynchronizer_.waitForUpToDateImuData(
      okvis::Time(imuDataEndTime)) == false) {
      timerRefs.waitForImuDataTimer.stop();
      return true;
  }
  timerRefs.waitForImuDataTimer.stop();
  OKVIS_ASSERT_TRUE_DBG(Exception,
      imuDataEndTime < imuMeasurements_.back().timeStamp,
      "Waiting for up to date imu data seems to have failed!");

  okvis::ImuMeasurementDeque imuData = swift_vio::getImuMeasurements(
      imuDataBeginTime, imuDataEndTime, imuMeasurements_, &imuMeasurements_mutex_);

  // if imu_data is empty, either end_time > begin_time or
  // no measurements in timeframe, should not happen, as we waited for measurements
  if (imuData.size() == 0)
    return false;

  omega_S = imuData.back().measurement.gyroscopes - bg;
  std::shared_ptr<VisualSubmap> vsubmap(new VisualSubmap());
  if (lastStateTime != okvis::Time()) {
    T_WS = propagationConstVelocity(T_WS, v_WS, omega_S, (frame->timestamp() - lastStateTime).toSec());
    timerRefs.getSubmapTimer.start();
    std::lock_guard<std::mutex> visualMapLock(visualMap_mutex_);
    visualMap_.getSubmap(T_WS, vsubmap.get());
    timerRefs.getSubmapTimer.stop();
  }

  okvis::Time t0Matching = okvis::Time::now();
  timerRefs.matchingTimer.start();
  std::shared_ptr<VisualMatcherOutput> matcherOutput(new VisualMatcherOutput(frame, imuData));
  frontend_->dataAssociation(frame, vsubmap, matcherOutput.get());
  timerRefs.matchingTimer.stop();

  if(!blocking_) {
    double timeLimit = parameters_.optimization.timeLimitForMatchingAndOptimization
                       -(okvis::Time::now()-t0Matching).toSec();
    optTimeLimitMillisec = std::max<double>(0.0, timeLimit) * 1000;
  }
  // A rough value is good enough because we will retrieve more IMU data by half_window_.
  lastAddedStateTimestamp_ = TimingUtilities::computeStateTimestamp(*(frontend_->cameraSystem()), frame);

  // use queue size 1 to propagate a congestion to the _matchedFrames queue
  if (matchedFrames_.PushBlockingIfFull(matcherOutput, 1) == false)
    return true;
  return false;
}

// Loop to process IMU measurements.
void VioSystem::imuConsumerLoop() {
  okvis::ImuMeasurement data;
  okvis::TimerSwitchable processImuTimer("0 processImuMeasurements",true);
  for (;;) {
    // get data and check for termination request
    if (imuMeasurementsReceived_.PopBlocking(&data) == false)
      return;
    processImuTimer.start();
    okvis::Time start;
    const okvis::Time* end;  // do not need to copy end timestamp
    {
      std::lock_guard<std::mutex> imuLock(imuMeasurements_mutex_);
      OKVIS_ASSERT_TRUE(Exception,
                        imuMeasurements_.empty()
                        || imuMeasurements_.back().timeStamp < data.timeStamp,
                        "IMU measurement from the past received");

      if (parameters_.publishing.publishImuPropagatedState) {
        if (!repropagationNeeded_ && imuMeasurements_.size() > 0) {
          start = imuMeasurements_.back().timeStamp;
        } else {
          start = okvis::Time(0, 0);
        }
        end = &data.timeStamp;
      }
      imuMeasurements_.push_back(data);
    }  // unlock _imuMeasurements_mutex

    Eigen::AlignedVector<okvis::kinematics::Transformation> vector_of_T_SCi;
    vector_of_T_SCi.resize(parameters_.nCameraSystem.numCameras());
    if (parameters_.publishing.publishImuPropagatedState) {
      std::lock_guard<std::mutex> lastStateLock(lastState_mutex_);
      if (repropagationNeeded_) {
        start = lastOptimizedStateTimestamp_;
        T_WS_propagated_ = lastOptimized_T_WS_;
        speedAndBiases_propagated_.head<3>() = lastOptimizedSpeed_;
        speedAndBiases_propagated_.segment<3>(3) =
            lastOptimizedImuRig_->at(0).getGyroBias();
        speedAndBiases_propagated_.tail<3>() =
            lastOptimizedImuRig_->at(0).getAccelBias();
        repropagationNeeded_ = false;
      }
      for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
        vector_of_T_SCi[i] =
            *(lastOptimizedCameraSystem_->getCameraExtrinsicPtr(i));
      }
      // unlock lastState_mutex
    }

    // notify other threads that imu data with timeStamp is here.
    imuFrameSynchronizer_.gotImuData(data.timeStamp);

    if (parameters_.publishing.publishImuPropagatedState) {
      Eigen::Matrix<double, 15, 15> covariance;
      Eigen::Matrix<double, 15, 15> jacobian;

      if (imuMeasurements_.size() < 2) {
        LOG(WARNING) << "- Skipping propagation as only one IMU measurement "
                        "has been given to frontend."
                     << " Normal when starting up.";
      } else {
        okvis::ceres::ImuError::propagation(
            imuMeasurements_, imu_params_, T_WS_propagated_,
            speedAndBiases_propagated_, start, *end, &covariance, &jacobian);
      }

      OptimizationResults result;
      result.stamp = *end;
      result.T_WS = T_WS_propagated_;
      result.speedAndBiases = speedAndBiases_propagated_;
      result.omega_S = imuMeasurements_.back().measurement.gyroscopes
          - speedAndBiases_propagated_.segment<3>(3);
      result.vector_of_T_SCi = vector_of_T_SCi;

      result.onlyPublishLandmarks = false;
      optimizationResults_.PushNonBlockingDroppingIfFull(result,1);
    }
    processImuTimer.stop();
  }
}

// Loop to process position measurements.
void VioSystem::positionConsumerLoop() {
  okvis::PositionMeasurement data;
  for (;;) {
    // get data and check for termination request
    if (positionMeasurementsReceived_.PopBlocking(&data) == false)
      return;
    // collect
    {
      std::lock_guard<std::mutex> positionLock(positionMeasurements_mutex_);
      positionMeasurements_.push_back(data);
    }
  }
}

// Loop to process GPS measurements.
void VioSystem::gpsConsumerLoop() {
}

// Loop to process magnetometer measurements.
void VioSystem::magnetometerConsumerLoop() {
}

// Loop to process differential pressure measurements.
void VioSystem::differentialConsumerLoop() {
}

// Loop that visualizes completed frames.
void VioSystem::visualizationLoop() {
  for (;;) {
    bool terminate = visualize();
    if (terminate) {
      return;
    }
  }
}

bool VioSystem::visualize() {
  VioVisualizer::VisualizationData::Ptr new_data;
  if (visualizationData_.PopBlocking(&new_data) == false)
    return true;
  // visualizer_.showDebugImages(new_data);
  const bool drawMatches = true;
  std::vector<cv::Mat> out_images(parameters_.nCameraSystem.numCameras());
  if (drawMatches) {
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
      out_images[i] = visualizer_.drawMatches(new_data, i);
    }
  } else {
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
      out_images[i] = visualizer_.drawColoredKeypoints(new_data, i);
    }
  }
  displayImages_.PushNonBlockingDroppingIfFull(out_images, 1);
  return false;
}

// trigger display (needed because OSX won't allow threaded display)
void VioSystem::display() {
  std::vector<cv::Mat> out_images;
  if (displayImages_.Size() == 0)
	return;
  if (displayImages_.PopBlocking(&out_images) == false)
    return;
  // draw
  for (size_t im = 0; im < parameters_.nCameraSystem.numCameras(); im++) {
    std::stringstream windowname;
    windowname << viewerNamePrefix_ << " " << im;
    cv::imshow(windowname.str(), out_images[im]);
  }
  cv::waitKey(1);
}

// Loop that performs the optimization and marginalisation.
void VioSystem::optimizationLoop() {
  okvis::TimerSwitchable convertEstimatorTimer("3.1 convertEstimator",true);
  okvis::TimerSwitchable optimizationTimer("3.2 optimization",true);
  okvis::TimerSwitchable marginalizationTimer("3.3 marginalization",true);
  okvis::TimerSwitchable latestStateTimer("3.4 getLatestState",true);
  okvis::TimerSwitchable estimationResultTimer("3.5 getEstimationResult",true);
  okvis::TimerSwitchable visualizationDataTimer("3.6 getVisualizationData",true);

  EstimationTimerRefs timerRefs(convertEstimatorTimer, optimizationTimer, marginalizationTimer,
                                latestStateTimer, estimationResultTimer, visualizationDataTimer);

  for (;;) {
    bool terminate = estimate(timerRefs);
    if (terminate) {
      return;
    }
  }
}

bool VioSystem::estimate(EstimationTimerRefs &timerRefs) {
  std::shared_ptr<VisualMatcherOutput> matchedFrame;
  if (matchedFrames_.PopBlocking(&matchedFrame) == false)
    return true;
  OptimizationResults result;
  std::shared_ptr<const EstimatorBase> estimatorToPublish;
  if (!initializer_->wellInitialized()) {
    if (!blocking_) {
      initializer_->setOptimizationTimeLimit(optTimeLimitMillisec / 1000);
    }
    timerRefs.optimizationTimer.start();
    initializer_->estimate(matchedFrame);
    timerRefs.optimizationTimer.stop();

    if (initializer_->wellInitialized()) {
      timerRefs.convertEstimatorTimer.start();
      estimator_->initializeFrom(initializer_, matchedFrame->currentNFrame);
//      initializer_->clear(); // TODO(jhuai): clear the data, but keep status.
      timerRefs.convertEstimatorTimer.stop();

      estimatorToPublish = estimator_;
    } else {
      estimatorToPublish = initializer_;
    }
  } else {
    if (!blocking_) {
      estimator_->setOptimizationTimeLimit(optTimeLimitMillisec / 1000);
    }

    timerRefs.optimizationTimer.start();
    estimator_->estimate(matchedFrame);
    timerRefs.optimizationTimer.stop();

    timerRefs.marginalizationTimer.start();
    estimator_->applyMarginalizationStrategy(result.transferredLandmarks);
    timerRefs.marginalizationTimer.stop();

    // get timestamp of last frame in IMU window. Need to do this before
    // marginalization as it will be removed there (if not keyframe)
    okvis::Time deleteImuMeasurementsUntil =
        estimator_->oldestFrameTimestamp() - EstimatorBase::half_window_;

    swift_vio::deleteImuMeasurements(deleteImuMeasurementsUntil,
                                     imuMeasurements_, &imuMeasurements_mutex_);
    estimatorToPublish = estimator_;
  }

  timerRefs.latestStateTimer.start();
  getLatestState(estimatorToPublish);
  estimatorToPublish->getEstimatedCameraSystem(&visualizer_.parametersMutable().nCameraSystem);
  timerRefs.latestStateTimer.stop();
  repropagationNeeded_ = true;

  // saving optimized state and saving it in OptimizationResults struct
  timerRefs.estimationResultTimer.start();
  getEstimationResult(estimatorToPublish, estimatorToPublish->currentFrameId(),
                      &result, parameters_.nCameraSystem.numCameras(),
                      parameters_.publishing.publishImuPropagatedState);
  timerRefs.estimationResultTimer.stop();
  optimizationResults_.Push(result);

  if (estimatorToPublish->wellInitialized()) {
    std::shared_ptr<swift_vio::LoopQueryKeyframeMessage> queryKeyframe;
    okvis::TimerSwitchable getQueryTimer("getQuery");
    estimatorToPublish->getLoopQueryKeyframeMessage(
        matchedFrame->currentNFrame, visualizer_.parameters().nCameraSystem,
        parameters_.poseGraphOptions.lcdCameras, &queryKeyframe);
    double queryTime = getQueryTimer.stop();

    if (queryKeyframe && keyframeCallback_) {
      okvis::TimerSwitchable pubKeyframeTimer("publishKeyframe");
      keyframeCallback_(*queryKeyframe);
      double pubKfTime = pubKeyframeTimer.stop();
      // LOG(INFO) << "getQuery costs " << queryTime << " sec,  publish keyframe costs " << pubKfTime << " sec.";
      // TODO(jhuai): publishing to ROS takes 3 times the time for getLoopQueryKeyframeMessage,
      // about 0.1 ms. To improve efficiency, we may push it to a queue and then publish it as in the publisherLoop.
    }
    // TODO(jhuai): dump the submap possibly for the FeatureAssociation Frontend?
  }

  VioVisualizer::VisualizationData::Ptr visualizationDataPtr = createVisualizationData(
      estimatorToPublish, matchedFrame, timerRefs.visualizationDataTimer);
  if (visualizationDataPtr) {
    visualizationData_.PushNonBlockingDroppingIfFull(visualizationDataPtr, 1);
  }
  return false;
}

VioVisualizer::VisualizationData::Ptr VioSystem::createVisualizationData(
    std::shared_ptr<const EstimatorBase> estimatorToPublish,
    std::shared_ptr<const VisualMatcherOutput> matchedFrame,
    okvis::TimerSwitchable &vizDataTimer) const {
  vizDataTimer.start();
  VioVisualizer::VisualizationData::Ptr visualizationDataPtr;
  if (parameters_.visualization.displayImages &&
      matchedFrame->currentNFrame->id() ==
          estimatorToPublish->currentFrameId() &&
      estimatorToPublish->isFrameInStateMap(
          matchedFrame->closestKeyframe->id())) {
    // Visualize when three conditions are met.
    // 1. display images
    // 2. only visualize frames whose observations are used by the estimator.
    // Some estimator may only use keyframe observations.
    // 3. The closest keyframe chosen by the frontend is in the state map so
    // its pose is available through get_T_WS.
    // Some estimator may marginalizes the keyframe earlier.
    visualizationDataPtr = VioVisualizer::VisualizationData::Ptr(
        new VioVisualizer::VisualizationData());
    getVisualizationData(estimatorToPublish, matchedFrame->currentNFrame,
                         matchedFrame->closestKeyframe, visualizationDataPtr);
  }
  vizDataTimer.stop();
  return visualizationDataPtr;
}

void VioSystem::getLatestState(std::shared_ptr<const EstimatorBase> estimator) {
  okvis::kinematics::Transformation latest_T_WS;
  okvis::SpeedAndBias latestSpeedAndBias;
  uint64_t latestNFrameId = estimator->currentFrameId();
  estimator->get_T_WS(latestNFrameId, latest_T_WS);
  Eigen::Vector3d latest_v_WS;
  estimator->getSpeed(latestNFrameId, latest_v_WS);
  okvis::Time latestStateTime = estimator->currentFrameTimestamp();

  std::lock_guard<std::mutex> lock(lastState_mutex_);
  lastOptimized_T_WS_ = latest_T_WS;
  lastOptimizedSpeed_ = latest_v_WS;
  estimator->getEstimatedImuRig(lastOptimizedImuRig_.get());
  estimator->getEstimatedCameraSystem(lastOptimizedCameraSystem_.get());
  lastOptimizedStateTimestamp_ = latestStateTime;
}

// Loop that publishes the newest state and landmarks.
void VioSystem::publisherLoop() {
  for (;;) {
    bool terminate = publish();
    if (terminate) {
      return;
    }
  }
}

bool VioSystem::publish() {
  // get the result data
  OptimizationResults result;
  if (optimizationResults_.PopBlocking(&result) == false)
    return true;

  // call all user callbacks
  if (stateCallback_ && !result.onlyPublishLandmarks)
    stateCallback_(result.stamp, result.T_WS);
  if (fullStateCallback_ && !result.onlyPublishLandmarks)
    fullStateCallback_(result.stamp, result.T_WS, result.speedAndBiases,
                       result.omega_S, result.nframeId);
  if (fullStateCallbackWithExtrinsics_ && !result.onlyPublishLandmarks)
    fullStateCallbackWithExtrinsics_(result.stamp, result.T_WS,
                                     result.speedAndBiases, result.omega_S,
                                     result.nframeId, result.vector_of_T_SCi);
  if (fullStateCallbackWithAllCalibration_) {
    fullStateCallbackWithAllCalibration_(
        result.stamp, result.nframeId, result.T_WS, result.speedAndBiases,
        result.omega_S, result.imuExtraParams_, result.variableCameraParams_,
        result.stateStd_, result.vector_of_T_SCi);
  }
  if (landmarksCallback_ && !result.landmarksVector.empty())
    landmarksCallback_(
        result.stamp, result.landmarksVector,
        result.transferredLandmarks);
  return false;
}

void VioSystem::saveStatistics(const std::string &filename) {
  std::ofstream stream(filename, std::ios_base::app);
  if (!stream.is_open()) {
    LOG(WARNING) << "VioSystem failed to open:" << filename << ".";
    return;
  }
  estimator_->printTrackLengthHistogram(stream);
  if (stream.is_open())
    stream.close();
}

bool VioSystem::serialProcessCameraNFrame() {
  for (size_t cameraIndex = 0u; cameraIndex < numCameras_; ++cameraIndex) {
    okvis::TimerSwitchable beforeDetectTimer(
        "1.1 frameLoopBeforeDetect" + std::to_string(cameraIndex), true);
    okvis::TimerSwitchable waitForFrameSynchronizerMutexTimer(
        "1.1.1 waitForFrameSynchronizerMutex" + std::to_string(cameraIndex),
        true);
    okvis::TimerSwitchable addNewFrameToSynchronizerTimer(
        "1.1.2 addNewFrameToSynchronizer" + std::to_string(cameraIndex), true);
    okvis::TimerSwitchable waitForStateVariablesMutexTimer(
        "1.1.3 waitForStateVariablesMutex" + std::to_string(cameraIndex), true);
    okvis::TimerSwitchable propagationTimer(
        "1.1.4 propagationTimer" + std::to_string(cameraIndex), true);
    okvis::TimerSwitchable detectTimer(
        "1.2 detectAndDescribe" + std::to_string(cameraIndex), true);
    okvis::TimerSwitchable afterDetectTimer(
        "1.3 afterDetect" + std::to_string(cameraIndex), true);
    okvis::TimerSwitchable waitForFrameSynchronizerMutexTimer2(
        "1.3.1 waitForFrameSynchronizerMutex2" + std::to_string(cameraIndex),
        true);
    okvis::TimerSwitchable waitForMatchingThreadTimer(
        "1.4 waitForMatchingThread" + std::to_string(cameraIndex), true);

    FrameDescriptionTimerRefs timerRefs1(
        beforeDetectTimer, waitForFrameSynchronizerMutexTimer,
        addNewFrameToSynchronizerTimer, waitForStateVariablesMutexTimer,
        propagationTimer, detectTimer, afterDetectTimer,
        waitForFrameSynchronizerMutexTimer2, waitForMatchingThreadTimer);
    extractVisualFeatures(cameraIndex, timerRefs1);
  }

  okvis::TimerSwitchable copyLatestStateTimer("2.1 copyLatestState", true);
  okvis::TimerSwitchable waitForImuDataTimer("2.2 waitForImuData", true);
  okvis::TimerSwitchable getSubmapTimer("2.3 getSubmap", true);
  okvis::TimerSwitchable matchingTimer("2.4 matching", true);
  VisualMatchingTimerRefs timerRefs2(copyLatestStateTimer, waitForImuDataTimer,
                                    getSubmapTimer, matchingTimer);
  matchVisualFeatures(timerRefs2);

  okvis::TimerSwitchable convertEstimatorTimer("3.1 convertEstimator", true);
  okvis::TimerSwitchable optimizationTimer("3.2 optimization", true);
  okvis::TimerSwitchable marginalizationTimer("3.3 marginalization", true);
  okvis::TimerSwitchable latestStateTimer("3.4 getLatestState", true);
  okvis::TimerSwitchable estimationResultTimer("3.5 getEstimationResult", true);
  okvis::TimerSwitchable visualizationDataTimer("3.6 getVisualizationData",
                                                true);
  EstimationTimerRefs timerRefs3(convertEstimatorTimer, optimizationTimer,
                                 marginalizationTimer, latestStateTimer,
                                 estimationResultTimer, visualizationDataTimer);
  estimate(timerRefs3);

  publish();

  visualize();

  display();
  return true;
}

void configureCameraFrontend(const okvis::cameras::NCameraSystem &cameraSystem,
                             std::shared_ptr<CameraFrontendBase> frontend) {
  frontend->setCameraSystem(cameraSystem);
}

}  // namespace swift_vio
