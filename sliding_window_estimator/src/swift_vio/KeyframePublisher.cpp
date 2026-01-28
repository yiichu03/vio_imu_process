#include <swift_vio/KeyframePublisher.hpp>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

#include <glog/logging.h>

#include <csfm_msgs/QueryMultiFrame.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace swift_vio {
KeyframePublisher::KeyframePublisher(ros::NodeHandle &n) : numPublishedKeyframes_(0u) {
  publisher_ = n.advertise<csfm_msgs::QueryMultiFrame>("query_keyframe", maxLoopFrameQueueSize_);
}

size_t KeyframePublisher::waitForSubscriber(bool performLoopClosure) {
  LOG(INFO) << "Waiting for keyframe subscribers, e.g., a loop closure node...";
  if (performLoopClosure) {
    while (publisher_.getNumSubscribers() == 0u) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  LOG(INFO) << "Got " << publisher_.getNumSubscribers() << " subscribers for keyframes.";
}

void toRosTransformation(const okvis::kinematics::Transformation &T,
                         geometry_msgs::Pose *P) {
  P->position.x = T.r().x();
  P->position.y = T.r().y();
  P->position.z = T.r().z();
  P->orientation.x = T.q().x();
  P->orientation.y = T.q().y();
  P->orientation.z = T.q().z();
  P->orientation.w = T.q().w();
}

void toRosPoseCovariance(const Eigen::Matrix<double, 6, 6> &ecov,
                         boost::array<double, 36> *bcov) {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0 ; j < 6; ++j) {
      (*bcov)[i * 6 + j] = ecov(i, j);
    }
  }
}

void toKeyframeRosMessage(const LoopQueryKeyframeMessage &message, size_t seq,
                          csfm_msgs::QueryMultiFrame *rosMsg) {
  rosMsg->header.stamp = ros::Time(message.stamp_.sec, message.stamp_.nsec);
  rosMsg->header.frame_id = std::to_string(message.id_);
  rosMsg->myseq = seq;

  toRosTransformation(message.T_WB_, &(rosMsg->T_WB.pose));
  toRosPoseCovariance(message.getCovariance(), &(rosMsg->T_WB.covariance));
  rosMsg->use_uniform_cov = message.useUniformCov();

  rosMsg->nFrameHeader.stamp = ros::Time(message.nframe_->timestamp().sec, message.nframe_->timestamp().nsec);
  rosMsg->nFrameHeader.frame_id = std::to_string(message.nframe_->id());

  size_t numcameras = message.nframe_->numFrames();
  rosMsg->query.resize(numcameras);
  rosMsg->system.resize(numcameras);
  const auto &cameraSystem = message.cameraSystem_;
  for (size_t i = 0; i < numcameras; ++i) {
    auto &camera = rosMsg->system.at(i);
    camera.id = cameraSystem.cameraGeometry(i)->id();
    toRosTransformation(*cameraSystem.T_SC(i), &camera.T_BC);

    camera.type = cameraSystem.cameraGeometry(i)->type();
    camera.width = cameraSystem.cameraGeometry(i)->imageWidth();
    camera.height = cameraSystem.cameraGeometry(i)->imageHeight();
    int ni = cameraSystem.cameraGeometry(i)->noIntrinsicsParameters();
    camera.intrinsics.resize(ni);

    Eigen::VectorXd eigenIntrinsic(ni);
    cameraSystem.cameraGeometry(i)->getIntrinsics(eigenIntrinsic);
    memcpy(camera.intrinsics.data(), eigenIntrinsic.data(),
           sizeof(double) * ni);
    camera.overlaps.resize(numcameras);
    for (size_t j = 0; j < numcameras; ++j) {
      camera.overlaps[j] = cameraSystem.overlaps().at(i).at(j);
    }

    auto &frame = rosMsg->query.at(i);
    size_t camId = i;
    frame.cameraId = camId;
    frame.stamp.sec = message.nframe_->timestamp(camId).sec;
    frame.stamp.nsec = message.nframe_->timestamp(camId).nsec;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = frame.stamp;
    out_msg.image = message.nframe_->image(camId);
    if (out_msg.image.channels() == 1 && out_msg.image.depth() == CV_8U) {
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    } else if (out_msg.image.channels() == 3 &&
               out_msg.image.depth() == CV_8U) {
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    } else {
      LOG(INFO) << "Unknown image format of #channels "
                << out_msg.image.channels() << ", depth "
                << out_msg.image.depth() << ".";
    }
    frame.image = *out_msg.toImageMsg();

    const auto &kpList = message.nframe_->getKeypoints(camId);
    size_t nk = kpList.size();
    frame.keypointList.resize(nk);
    for (size_t k = 0; k < nk; ++k) {
      frame.keypointList.at(k).x = kpList[k].pt.x;
      frame.keypointList.at(k).y = kpList[k].pt.y;
      frame.keypointList.at(k).z = kpList[k].size;
    }

    const auto &desMat = message.nframe_->getDescriptors(camId);
    frame.descriptors.layout.dim.resize(2);
    auto &dimList = frame.descriptors.layout.dim;
    size_t desSize = desMat.rows * desMat.cols;
    dimList[0].label = "height";
    dimList[0].size = desMat.rows;
    dimList[0].stride = desSize;
    dimList[1].label = "width";
    dimList[1].size = desMat.cols;
    dimList[1].stride = desMat.cols;
    frame.descriptors.layout.data_offset = 0;
    frame.descriptors.data.resize(desSize);
    memcpy(frame.descriptors.data.data(), desMat.data,
           sizeof(uint8_t) * desSize);

    size_t nl = message.landmarkPositionList_.at(i).size();
    frame.landmarkPositionList.resize(nl);
    for (size_t l = 0; l < nl; ++l) {
      frame.landmarkPositionList.at(l).x =
          message.landmarkPositionList_[i][l][0];
      frame.landmarkPositionList.at(l).y =
          message.landmarkPositionList_[i][l][1];
      frame.landmarkPositionList.at(l).z =
          message.landmarkPositionList_[i][l][2];
      frame.landmarkPositionList.at(l).w =
          message.landmarkPositionList_[i][l][3];
    }
    
    frame.keypointIndexForLandmarkList = message.keypointIndexForLandmarkList_[i];
  }

  size_t nc = message.odometryConstraintList_.size();
  rosMsg->odometryConstraintList.resize(nc);
  for (size_t c = 0u; c < nc; ++c) {
    rosMsg->odometryConstraintList.at(c).header.stamp.sec =
        message.odometryConstraintList_.at(c)->core_.stamp_.sec;
    rosMsg->odometryConstraintList.at(c).header.stamp.nsec =
        message.odometryConstraintList_.at(c)->core_.stamp_.nsec;
    rosMsg->odometryConstraintList.at(c).header.frame_id =
        std::to_string(message.odometryConstraintList_.at(c)->core_.id_);
    rosMsg->odometryConstraintList.at(c).type = static_cast<int32_t>(
        message.odometryConstraintList_.at(c)->core_.type_);
    toRosTransformation(message.odometryConstraintList_.at(c)->core_.T_BBr_,
                        &(rosMsg->odometryConstraintList.at(c).T_BBr));
    toRosTransformation(message.odometryConstraintList_.at(c)->T_WB_,
                        &(rosMsg->odometryConstraintList.at(c).T_WB.pose));
    toRosPoseCovariance(
        message.odometryConstraintList_.at(c)->cov_T_WB_,
        &(rosMsg->odometryConstraintList.at(c).T_WB.covariance));
    toRosPoseCovariance(message.odometryConstraintList_.at(c)->cov_T_WBr_T_WB_,
                        &(rosMsg->odometryConstraintList.at(c).cov_T_WBr_T_WB));
  }
}

void KeyframePublisher::publish(const LoopQueryKeyframeMessage &message) const {
  boost::shared_ptr<csfm_msgs::QueryMultiFrame> keyframeMsg(
      new csfm_msgs::QueryMultiFrame());
  ++numPublishedKeyframes_;
  toKeyframeRosMessage(message, numPublishedKeyframes_, keyframeMsg.get());
  publisher_.publish(keyframeMsg);
  // LOG(INFO) << "Publishing keyframe of seq " << numPublishedKeyframes_ 
  //     << " of id " << keyframeMsg->header.frame_id << ".";
}

} // namespace swift_vio
