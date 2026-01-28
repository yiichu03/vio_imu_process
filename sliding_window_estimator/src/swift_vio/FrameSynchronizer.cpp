
/**
 * @file FrameSynchronizer.cpp
 * @brief Source file for the FrameSynchronizer class.
 */

#include <glog/logging.h>

#include <swift_vio/FrameSynchronizer.hpp>
#include <okvis/IdProvider.hpp>

namespace swift_vio {

static const int max_frame_sync_buffer_size = 3;

// Constructor. Calls init().
FrameSynchronizer::FrameSynchronizer(okvis::VioParameters& parameters)
: numCameras_(parameters.nCameraSystem.numCameras()),
  timeTol_(parameters.sensors_information.frameTimestampTolerance),
  lastCompletedFrameId_(0)
{
  frameBuffer_.resize(max_frame_sync_buffer_size,
                      std::pair<std::shared_ptr<MultiFrame>,size_t>(nullptr,0));
  bufferPosition_ = 0;
}

FrameSynchronizer::FrameSynchronizer(size_t numCameras, double timeTol)
: numCameras_(numCameras),
  timeTol_(timeTol),
  lastCompletedFrameId_(0)
{
  frameBuffer_.resize(max_frame_sync_buffer_size,
                      std::pair<std::shared_ptr<MultiFrame>,size_t>(nullptr,0));
  bufferPosition_ = 0;
}

// Trivial destructor.
FrameSynchronizer::~FrameSynchronizer() {
}

// Adds a new frame to the internal buffer and returns the Multiframe containing the frame.
std::shared_ptr<MultiFrame> FrameSynchronizer::addNewFrame(std::shared_ptr<okvis::CameraMeasurement>& frame) {
  assert(numCameras_ > 0);
  okvis::Time frame_stamp = frame->timeStamp;
  std::shared_ptr<MultiFrame> multiFrame;
  int position;
  if(findFrameByTime(frame_stamp,position)) {
    multiFrame = frameBuffer_[position].first;
    OKVIS_ASSERT_TRUE_DBG(Exception,multiFrame->image(frame->sensorId).empty(),
                       "Frame for this camera has already been added to multiframe!");
    if(frame_stamp != multiFrame->timestamp()) {
      // timestamps do not agree. setting timestamp to middlepoint
      frame_stamp += (multiFrame->timestamp()-frame_stamp)*0.5;
      multiFrame->setTimestamp(frame_stamp);
    }
    multiFrame->setImage(frame->sensorId,frame->measurement.image);
    multiFrame->setTimestamp(frame->sensorId, frame->timeStamp);
  }
  else {
    multiFrame = std::shared_ptr<MultiFrame>(new MultiFrame(numCameras_, frame_stamp,
                                                            okvis::IdProvider::instance().newId()));
    multiFrame->setImage(frame->sensorId,frame->measurement.image);
    multiFrame->setTimestamp(frame->sensorId, frame->timeStamp);
    bufferPosition_ = (bufferPosition_+1) % max_frame_sync_buffer_size;
    if(frameBuffer_[bufferPosition_].first != nullptr
       && frameBuffer_[bufferPosition_].second != numCameras_) {
     LOG(ERROR) << "Dropping frame with id " << frameBuffer_[bufferPosition_].first->id();
    }
    frameBuffer_[bufferPosition_].first = multiFrame;
    frameBuffer_[bufferPosition_].second= 0;
  }
  return multiFrame;
}

// Inform the synchronizer that a frame in the multiframe has completed keypoint detection and description.
bool FrameSynchronizer::detectionEndedForMultiFrame(uint64_t multiFrameId) {
  int position;
  bool found = findFrameById(multiFrameId,position);
  if(found) {
    ++frameBuffer_[position].second;
    OKVIS_ASSERT_TRUE_DBG(Exception,frameBuffer_[position].second<=numCameras_,
                       "Completion counter is larger than the amount of cameras in the system!");
  }
  return found;
}

// This will return true if the internal counter on how many times detectionEndedForMultiFrame()
// has been called for this multiframe equals the number of cameras in the system.
bool FrameSynchronizer::detectionCompletedForAllCameras(uint64_t multiFrameId) {
  int position;
  if(findFrameById(multiFrameId,position)) {
    if(frameBuffer_[position].second == numCameras_) {
      OKVIS_ASSERT_TRUE(Exception,frameBuffer_[position].first->timestamp() > lastCompletedFrameTimestamp_
                            && (lastCompletedFrameId_==0 || frameBuffer_[position].first->id() > lastCompletedFrameId_) ,
                     "wrong order!\ntimestamp last: " << lastCompletedFrameTimestamp_
                     << "\ntimestamp new:  " << frameBuffer_[position].first->timestamp()
                     << "\nid last: " << lastCompletedFrameId_
                     << "\nid new:  " << frameBuffer_[position].first->id());
      lastCompletedFrameId_ = frameBuffer_[position].first->id();
      lastCompletedFrameTimestamp_ = frameBuffer_[position].first->timestamp();
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

// Find a multiframe in the buffer that has a timestamp within the tolerances of the given one. The tolerance
// is given as a parameter in okvis::VioParameters::sensors_information::frameTimestampTolerance
bool FrameSynchronizer::findFrameByTime(const okvis::Time& timestamp, int& position) const{
  bool found = false;
  for(int i=0; i < max_frame_sync_buffer_size; ++i) {
    position = (bufferPosition_+i)%max_frame_sync_buffer_size;
    if(frameBuffer_[position].first != nullptr &&
       (frameBuffer_[position].first->timestamp() == timestamp ||
        fabs((frameBuffer_[position].first->timestamp()-timestamp).toSec()) < timeTol_)) {
      found = true;
      break;
    }
  }
  return found;
}

// Find a multiframe in the buffer for a given multiframe ID.
bool FrameSynchronizer::findFrameById(uint64_t mfId, int& position) const {
  bool found = false;
  for(int i=0; i < max_frame_sync_buffer_size; ++i) {
    position = (bufferPosition_+i)%max_frame_sync_buffer_size;
    if(frameBuffer_[position].first != nullptr &&
       frameBuffer_[position].first->id() == mfId) {
      found = true;
      break;
    }
  }
  return found;
}


} /* namespace swift_vio */
