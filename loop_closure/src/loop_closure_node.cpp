/**
 * @file loop_closure_node.cpp
 * @brief This file includes the ROS node implementation.
 * @author Jianzhu Huai
 */

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#pragma GCC diagnostic pop

#include <image_transport/image_transport.h>
#include <glog/logging.h>

#include <csfm_msgs/QueryMultiFrame.h>
#include <cv_bridge/cv_bridge.h>

#include <loop_closure/LoopClosureDetectorParams.h>
#include <loop_closure/FactoryMethods.hpp>
#include <loop_closure/LoopClosureMethod.hpp>
#include <loop_closure/LoopClosureModule.hpp>
#include <loop_closure/LoopClosureParameters.hpp>
#include <loop_closure/KeyframeForLoopDetection.hpp>
#include <loop_closure/PgoPublisher.hpp>

#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/cameras/FovDistortion.hpp>

#include <swift_vio/IoUtil.hpp>

DECLARE_string(output_dir);

DECLARE_string(vocabulary_path);

void toOkvisTransformation(const geometry_msgs::Pose &P, okvis::kinematics::Transformation *T) {
  T->parameterMutablePtr()[0] = P.position.x;
  T->parameterMutablePtr()[1] = P.position.y;
  T->parameterMutablePtr()[2] = P.position.z;
  T->parameterMutablePtr()[3] = P.orientation.x;
  T->parameterMutablePtr()[4] = P.orientation.y;
  T->parameterMutablePtr()[5] = P.orientation.z;
  T->parameterMutablePtr()[6] = P.orientation.w;
}

void toRosPoseCovariance(const Eigen::Matrix<double, 6, 6> &ecov,
                         boost::array<double, 36> *bcov) {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0 ; j < 6; ++j) {
      (*bcov)[i * 6 + j] = ecov(i, j);
    }
  }
}

void toEigenPoseCovariance(const boost::array<double, 36> &bcov, Eigen::Matrix<double, 6, 6, Eigen::RowMajor> *ecov) {
  for (int i = 0; i < 6; ++i) {
    memcpy(ecov->row(i).data(), bcov.data() + i * 6, sizeof(double) * 6);
  }
}

void testPoseCovConversion() {
  Eigen::Matrix<double, 6, 6> ecov;
  int k = 0;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0 ; j < 6; ++j) {
      ecov(i, j) = k++;
    }
  }

  boost::array<double, 36> bcov;
  for (int i = 0; i < 36; ++i) {
    bcov[i] = i;
  }
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> ecov2;
  toEigenPoseCovariance(bcov, &ecov2);
  CHECK_LT((ecov2 - ecov).norm(), 1e-8) << "ecov\n" << ecov << "\necov2\n" << ecov2;
}

void testPoseCovConversionInverse() {
  Eigen::Matrix<double, 6, 6> ecov;
  int k = 0;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0 ; j < 6; ++j) {
      ecov(i, j) = k++;
    }
  }

  boost::array<double, 36> bcov;
  for (int i = 0; i < 36; ++i) {
    bcov[i] = i;
  }
  boost::array<double, 36> bcov2;
  toRosPoseCovariance(ecov, &bcov2);

  double diff = 0;
  for (int i = 0; i < 36; ++i) {
    diff += std::abs(bcov[i] - bcov2[i]);
  }
  CHECK_LT(diff, 1e-8);
}

uint64_t stringToUint64(std::string str) {
  uint64_t value;
  std::istringstream iss(str);
  iss >> value;
  return value;
}

void toKeyframeCppMessage(
    const boost::shared_ptr<const csfm_msgs::QueryMultiFrame> &rosMsg,
    swift_vio::LoopQueryKeyframeMessage *message) {
  message->stamp_.sec = rosMsg->header.stamp.sec;
  message->stamp_.nsec = rosMsg->header.stamp.nsec;
  message->id_ = stringToUint64(rosMsg->header.frame_id);

  toOkvisTransformation(rosMsg->T_WB.pose, &message->T_WB_);
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cov_T_WB;
  toEigenPoseCovariance(rosMsg->T_WB.covariance, &cov_T_WB);
  message->setCovariance(cov_T_WB, rosMsg->use_uniform_cov);

  size_t numcameras = rosMsg->system.size();
  std::vector<std::vector<bool>> overlaps(numcameras);
  okvis::cameras::NCameraSystem& cameraSystem = message->cameraSystem_;
  for (size_t i = 0; i < numcameras; ++i) {
    const auto &camera = rosMsg->system.at(i);
    okvis::cameras::DistortionType distType = okvis::cameras::DistortionType::RadialTangential;
    std::shared_ptr<okvis::cameras::CameraBase> camModel;
    if (strcmp(camera.type.c_str(), "PinholeCamera<RadialTangentialDistortion>") ==
        0) {
      camModel.reset(new okvis::cameras::PinholeCamera<
                     okvis::cameras::RadialTangentialDistortion>(
          camera.width, camera.height, camera.intrinsics[0],
          camera.intrinsics[1], camera.intrinsics[2], camera.intrinsics[3],
          okvis::cameras::RadialTangentialDistortion(
              camera.intrinsics[4], camera.intrinsics[5], camera.intrinsics[6],
              camera.intrinsics[7]),
          0.0, 0.0, camera.id));
      distType = okvis::cameras::DistortionType::RadialTangential;
    } else if (strcmp(camera.type.c_str(), "PinholeCamera<EquidistantDistortion>") ==
               0) {
      camModel.reset(new okvis::cameras::PinholeCamera<
                     okvis::cameras::EquidistantDistortion>(
          camera.width, camera.height, camera.intrinsics[0],
          camera.intrinsics[1], camera.intrinsics[2], camera.intrinsics[3],
          okvis::cameras::EquidistantDistortion(
              camera.intrinsics[4], camera.intrinsics[5], camera.intrinsics[6],
              camera.intrinsics[7]),
          0.0, 0.0, camera.id));
      distType = okvis::cameras::DistortionType::Equidistant;
    } else if (strcmp(camera.type.c_str(),
                      "PinholeCamera<RadialTangentialDistortion8>") == 0) {
      camModel.reset(new okvis::cameras::PinholeCamera<
                     okvis::cameras::RadialTangentialDistortion8>(
          camera.width, camera.height, camera.intrinsics[0],
          camera.intrinsics[1], camera.intrinsics[2], camera.intrinsics[3],
          okvis::cameras::RadialTangentialDistortion8(
              camera.intrinsics[4], camera.intrinsics[5], camera.intrinsics[6],
              camera.intrinsics[7], camera.intrinsics[8], camera.intrinsics[9],
              camera.intrinsics[10], camera.intrinsics[11]),
          0.0, 0.0, camera.id));
      distType = okvis::cameras::DistortionType::RadialTangential8;
    } else if (strcmp(camera.type.c_str(), "PinholeCamera<FovDistortion>") == 0) {
      camModel.reset(new okvis::cameras::PinholeCamera<okvis::cameras::FovDistortion>(
              camera.width, camera.height, camera.intrinsics[0],
              camera.intrinsics[1], camera.intrinsics[2], camera.intrinsics[3],
              okvis::cameras::FovDistortion(camera.intrinsics[4]), 0.0, 0.0,
              camera.id));
      distType = okvis::cameras::DistortionType::Fov;
    } else {
      LOG(WARNING) << "Unrecognized camera model " << camera.type << ".";
    }

    std::shared_ptr<okvis::kinematics::Transformation> T_BC(
        new okvis::kinematics::Transformation());
    toOkvisTransformation(camera.T_BC, T_BC.get());
    overlaps.at(i).resize(numcameras);
    for (size_t j = 0; j < numcameras; ++j) {
      overlaps[i][j] = camera.overlaps.at(j);
    }

    cameraSystem.addCamera(T_BC, camModel, distType, "", "", false);
  }
  cameraSystem.setOverlaps(overlaps);

  okvis::Time nframeTime(rosMsg->nFrameHeader.stamp.sec,
                         rosMsg->nFrameHeader.stamp.nsec);
  message->nframe_.reset(
      new swift_vio::MultiFrame(cameraSystem.numCameras(), nframeTime,
                                stringToUint64(rosMsg->nFrameHeader.frame_id)));
  message->landmarkPositionList_.resize(numcameras);
  message->keypointIndexForLandmarkList_.resize(numcameras);
  for (size_t i = 0; i < numcameras; ++i) {
    const auto &frame = rosMsg->query.at(i);
    size_t camId = i;
    message->nframe_->setTimestamp(
        camId, okvis::Time(frame.stamp.sec, frame.stamp.nsec));

    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(frame.image, rosMsg, frame.image.encoding);
    message->nframe_->setImage(camId, cv_ptr->image);

    const auto &kpList = frame.keypointList;
    size_t nk = kpList.size();
    std::vector<cv::KeyPoint> cvKpList;
    cvKpList.resize(nk);
    for (size_t k = 0; k < nk; ++k) {
      cvKpList[k].pt.x = kpList[k].x;
      cvKpList[k].pt.y = kpList[k].y;
      cvKpList[k].size = kpList[k].z;
    }
    message->nframe_->resetKeypoints(camId, cvKpList);

    uint32_t dh = frame.descriptors.layout.dim[0].size;
    uint32_t dw = frame.descriptors.layout.dim[1].size;

    cv::Mat desMat(dh, dw, CV_8UC1);
    memcpy(desMat.data, frame.descriptors.data.data(),
           dh * dw * sizeof(uint8_t));
    message->nframe_->resetDescriptors(camId, desMat);

    size_t nl = frame.landmarkPositionList.size();
    message->landmarkPositionList_.at(i).resize(nl);
    for (size_t l = 0; l < nl; ++l) {
      message->landmarkPositionList_[i][l][0] =
          frame.landmarkPositionList.at(l).x;
      message->landmarkPositionList_[i][l][1] =
          frame.landmarkPositionList.at(l).y;
      message->landmarkPositionList_[i][l][2] =
          frame.landmarkPositionList.at(l).z;
      message->landmarkPositionList_[i][l][3] =
          frame.landmarkPositionList.at(l).w;
    }
    message->keypointIndexForLandmarkList_[i] = frame.keypointIndexForLandmarkList;
  }

  size_t nc = rosMsg->odometryConstraintList.size();
  message->odometryConstraintList_.resize(nc);

  for (size_t c = 0u; c < nc; ++c) {
    message->odometryConstraintList_.at(c).reset(new swift_vio::NeighborConstraintMessage());
    message->odometryConstraintList_.at(c)->core_.stamp_.sec =
        rosMsg->odometryConstraintList.at(c).header.stamp.sec;
    message->odometryConstraintList_.at(c)->core_.stamp_.nsec =
        rosMsg->odometryConstraintList.at(c).header.stamp.nsec;
    message->odometryConstraintList_.at(c)->core_.id_ =
        stringToUint64(rosMsg->odometryConstraintList.at(c).header.frame_id);
    message->odometryConstraintList_.at(c)->core_.type_ =
        static_cast<swift_vio::PoseConstraintType>(
            rosMsg->odometryConstraintList.at(c).type);

    toOkvisTransformation(
        rosMsg->odometryConstraintList.at(c).T_BBr,
        &(message->odometryConstraintList_.at(c)->core_.T_BBr_));
    toOkvisTransformation(rosMsg->odometryConstraintList.at(c).T_WB.pose,
                          &(message->odometryConstraintList_.at(c)->T_WB_));
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cov;
    toEigenPoseCovariance(rosMsg->odometryConstraintList.at(c).T_WB.covariance, &cov);
    message->odometryConstraintList_.at(c)->cov_T_WB_ = cov;
    toEigenPoseCovariance(rosMsg->odometryConstraintList.at(c).cov_T_WBr_T_WB, &cov);
    message->odometryConstraintList_.at(c)->cov_T_WBr_T_WB_ = cov;
  }
}

void keyframeHandler(const boost::shared_ptr<csfm_msgs::QueryMultiFrame const>& msg, 
    swift_vio::LoopClosureModule& loopClosureModule, size_t& lastMessageSeqNum) {
  if (msg->myseq != lastMessageSeqNum + 1) {
    LOG(WARNING) << "Inconsistent message seq number may imply message loss or dropout. "
        << "Last seq num " << lastMessageSeqNum << ", new seq num " << msg->myseq << ".";
  }
  lastMessageSeqNum = msg->myseq;
  std::shared_ptr<swift_vio::LoopQueryKeyframeMessage> keyframeCpp(
    new swift_vio::LoopQueryKeyframeMessage());
  toKeyframeCppMessage(msg, keyframeCpp.get());
  loopClosureModule.push(keyframeCpp);
  VLOG(1) << "Pushing a keyframe of seq id " << msg->myseq << " kf id " << keyframeCpp->id_ << " to the LCD.";
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true); // true to strip gflags
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  const std::string nodeName = "loop_closure_node";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh(nodeName);

  std::string lcdConfigFilename;
  if (argc >= 2) {
    lcdConfigFilename = argv[1];
  } else {
    LOG(ERROR) << "Usage:" << argv[0] << " <config yml> [extra gflags]";
    return 1;
  }

  swift_vio::PgoPublisher pgoPublisher;
  std::shared_ptr<swift_vio::LoopClosureDetectorParams> lcParams(
      new swift_vio::LoopClosureDetectorParams());
  if (lcdConfigFilename.empty()) {
    LOG(WARNING) << "Default parameters for loop closure will be used as no "
                    "configuration filename is provided!";
  } else {
    lcParams->parseYAML(lcdConfigFilename);
  }

  std::shared_ptr<swift_vio::LoopClosureMethod> loopClosureMethod =
      swift_vio::createLoopClosureMethod(lcParams);
  Eigen::IOFormat myformat = swift_vio::kSpaceInitFmt;
  swift_vio::LoopClosureModule loopClosureModule(loopClosureMethod);
  std::string path = swift_vio::removeTrailingSlash(FLAGS_output_dir);
  pgoPublisher.setCsvFile(path + "/online_pgo.csv");
  loopClosureModule.appendStateCallback(
      std::bind(&swift_vio::PgoPublisher::csvSaveStateAsCallback, &pgoPublisher,
                std::placeholders::_1, std::placeholders::_2));

  loopClosureModule.startThreads();

  size_t lastMessageSeqNum = 0;
  boost::function<void(const boost::shared_ptr<csfm_msgs::QueryMultiFrame const>&)> callback =
      boost::bind(keyframeHandler, _1, boost::ref(loopClosureModule), boost::ref(lastMessageSeqNum));
  ros::Subscriber sub_keyframe = nh.subscribe("/swift_vio_node/query_keyframe", 100, 
      callback);

  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  loopClosureModule.shutdown();
  return 0;
}
