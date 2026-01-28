
/**
 * @file VioVisualizer.cpp
 * @brief Source file for the VioVisualizer class.
 */


#include <okvis/kinematics/Transformation.hpp>

#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/FrameTypedefs.hpp>

#include "swift_vio/VioVisualizer.hpp"

// cameras and distortions
#include <okvis/CameraModelSwitch.hpp>
#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/cameras/FovDistortion.hpp>

namespace swift_vio {
VioVisualizer::VioVisualizer() {}

VioVisualizer::VioVisualizer(const okvis::VioParameters& parameters,
                             const std::string viewerNamePrefix)
    : parameters_(parameters),
      viewerNamePrefix_(viewerNamePrefix) {
  if (parameters.nCameraSystem.numCameras() > 0) {
    updateParameters(parameters);
  }
}

VioVisualizer::~VioVisualizer() {

}

void VioVisualizer::updateParameters(const okvis::VioParameters &parameters) {
  parameters_ = parameters;
  setupDisplays();
}

void VioVisualizer::setCameraSystem(const swift_vio::CameraRig &cameraRig) {
  cameraRig.cloneTo(&parameters_.nCameraSystem);
}

void VioVisualizer::setupDisplays() {
  for (size_t im = 0; im < 4; im++) {
    std::stringstream windowname;
    windowname << viewerNamePrefix_ << " " << im;
  }
  if(parameters_.visualization.displayImages) {
    for (size_t im = 0; im < parameters_.nCameraSystem.numCameras(); im++) {
      std::stringstream windowname;
      windowname << viewerNamePrefix_ << " " << im;
      cv::namedWindow(windowname.str());
    }
  }
}

cv::Mat VioVisualizer::drawMatches(VisualizationData::Ptr& data,
                                   size_t image_number) {

  std::shared_ptr<const MultiFrame> keyframe = data->keyFrames;
  std::shared_ptr<const MultiFrame> frame = data->currentFrames;

  int downscaleFactor = parameters_.visualization.downscaleFactor;
  if (keyframe == nullptr)
    return downscaleImage(frame->image(image_number), downscaleFactor);

  // allocate an image
  const unsigned int im_cols = frame->image(image_number).cols / downscaleFactor;
  const unsigned int im_rows = frame->image(image_number).rows / downscaleFactor;
  const unsigned int rowJump = im_rows;

  cv::Mat outimg(2 * im_rows, im_cols, CV_8UC3);
  // copy current images Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
  cv::Mat current = outimg(cv::Rect(0, rowJump, im_cols, im_rows));
  cv::Mat actKeyframe = outimg(cv::Rect(0, 0, im_cols, im_rows));
  const size_t pad = 2;
  cv::Mat border = outimg(cv::Rect(0, rowJump - pad, im_cols, pad * 2));

  cv::cvtColor(downscaleImage(frame->image(image_number), downscaleFactor), current, cv::COLOR_GRAY2BGR);
  cv::cvtColor(downscaleImage(keyframe->image(image_number), downscaleFactor), actKeyframe, cv::COLOR_GRAY2BGR);
  border.setTo(cv::Scalar(0x9e, 0xff, 0x8c));

  // the keyframe trafo
  Eigen::Vector2d keypoint;
  Eigen::Vector4d landmark;
  okvis::kinematics::Transformation lastKeyframeT_CW = parameters_.nCameraSystem
      .T_SC(image_number)->inverse() * data->T_WS_keyFrame.inverse();

  // find distortion type
  okvis::cameras::DistortionType distortionType = parameters_.nCameraSystem
      .distortionType(0);
  for (size_t i = 1; i < parameters_.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception,
                      distortionType == parameters_.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet");
  }

  for (auto it = data->observations.begin(); it != data->observations.end();
      ++it) {
    if (it->cameraIdx != image_number)
      continue;

    cv::Scalar color;

    if (it->landmarkId != 0) {
      color = cv::Scalar(255, 0, 0);  // blue
    } else {
      color = cv::Scalar(0, 0, 255);  // red
    }

    // draw matches to keyframe
    keypoint = it->keypointMeasurement;
    if (fabs(it->landmark_W[3]) > 1.0e-8) {
      Eigen::Vector4d hPoint = it->landmark_W;
      if (it->isInitialized) {
        color = cv::Scalar(0, 255, 0);  // green
      } else {
        color = cv::Scalar(0, 255, 255);  // yellow
      }
      Eigen::Vector2d keyframePt;
      bool isVisibleInKeyframe = false;
      Eigen::Vector4d hP_C = lastKeyframeT_CW * hPoint;

#define DISTORTION_MODEL_CASE(camera_geometry_t)                               \
  if (parameters_.nCameraSystem.cameraGeometry(image_number)                       \
          ->projectHomogeneous(hP_C, &keyframePt) ==                           \
      okvis::cameras::CameraBase::ProjectionStatus::Successful)                \
      isVisibleInKeyframe = true;

      switch (distortionType) { DISTORTION_MODEL_NO_NODISTORTION_SWITCH_CASES }

#undef DISTORTION_MODEL_CASE

      if (fabs(hP_C[3]) > 1.0e-8) {
        if (hP_C[2] / hP_C[3] < 0.4) {
          isVisibleInKeyframe = false;
        }
      }

      if (isVisibleInKeyframe) {
        // found in the keyframe. draw line
        cv::line(outimg, cv::Point2f(keyframePt[0] / downscaleFactor, keyframePt[1] / downscaleFactor),
                 cv::Point2f(keypoint[0] / downscaleFactor, keypoint[1] / downscaleFactor + rowJump), color, 1,
                 cv::LINE_AA);
        cv::circle(actKeyframe, cv::Point2f(keyframePt[0] / downscaleFactor, keyframePt[1] / downscaleFactor),
                   0.5 * it->keypointSize, color, 1, cv::LINE_AA);
      }
    }
    // draw keypoint
    const double r = 0.5 * it->keypointSize;
    cv::circle(current, cv::Point2f(keypoint[0] / downscaleFactor, keypoint[1] / downscaleFactor), r, color, 1,
    cv::LINE_AA);
    cv::KeyPoint cvKeypoint;
    frame->getCvKeypoint(image_number, it->keypointIdx, cvKeypoint);
    const double angle = cvKeypoint.angle / 180.0 * M_PI;
    cv::line(
        outimg,
        cv::Point2f(keypoint[0] / downscaleFactor, keypoint[1] / downscaleFactor + rowJump),
        cv::Point2f(keypoint[0] / downscaleFactor, keypoint[1] / downscaleFactor + rowJump)
            + cv::Point2f(cos(angle), sin(angle)) * r,
        color, 1,
        cv::LINE_AA);
  }
  return outimg;
}

cv::Mat VioVisualizer::drawColoredKeypoints(
    VisualizationData::Ptr& data, size_t image_number) const {
  const size_t slidingWindowSize = (parameters_.optimization.numKeyframes +
                                    parameters_.optimization.numImuFrames) / 2;
  std::shared_ptr<const MultiFrame> frame = data->currentFrames;
  cv::Mat outimg;
  int downscaleFactor = parameters_.visualization.downscaleFactor;
  cv::cvtColor(downscaleImage(frame->image(image_number), downscaleFactor), outimg, cv::COLOR_GRAY2BGR);

  for (auto it = data->observations.begin(); it != data->observations.end();
       ++it) {
    if (it->cameraIdx != image_number) continue;
    Eigen::Vector2d keypoint = it->keypointMeasurement;
    if (it->numObservations) {
      double len = std::min(1.0, 1.0 * it->numObservations / slidingWindowSize);
      cv::Scalar color = cv::Scalar(0, 255 * len, 255 * (1 - len)); // Long tracks are green.
      // draw keypoint
      const double r = 0.5 * it->keypointSize;
      cv::circle(outimg, cv::Point2f(keypoint[0] / downscaleFactor, keypoint[1] / downscaleFactor), r, color, 2,
                 cv::LINE_AA);
      cv::KeyPoint cvKeypoint;
      frame->getCvKeypoint(image_number, it->keypointIdx, cvKeypoint);
      const double angle = cvKeypoint.angle / 180.0 * M_PI;
      cv::line(outimg, cv::Point2f(keypoint[0] / downscaleFactor, keypoint[1] / downscaleFactor),
               cv::Point2f(keypoint[0] / downscaleFactor, keypoint[1] / downscaleFactor) +
                   cv::Point2f(cos(angle), sin(angle)) * r,
               color, 1, cv::LINE_AA);
    }
  }
  return outimg;
}

cv::Mat VioVisualizer::drawKeypoints(VisualizationData::Ptr& data,
                                     size_t cameraIndex) {

  std::shared_ptr<const MultiFrame> currentFrames = data->currentFrames;
  const cv::Mat currentImage = currentFrames->image(cameraIndex);
  int downscaleFactor = parameters_.visualization.downscaleFactor;
  cv::Mat outimg;
  cv::cvtColor(downscaleImage(currentImage, downscaleFactor), outimg, cv::COLOR_GRAY2BGR);
  cv::Scalar greenColor(0, 255, 0);  // green

  cv::KeyPoint keypoint;
  for (size_t k = 0; k < currentFrames->numKeypoints(cameraIndex); ++k) {
    currentFrames->getCvKeypoint(cameraIndex, k, keypoint);

    double radius = keypoint.size;
    double angle = keypoint.angle / 180.0 * M_PI;

    cv::circle(outimg, keypoint.pt / downscaleFactor, radius, greenColor);
    cv::line(
        outimg,
        keypoint.pt / downscaleFactor,
        cv::Point2f(keypoint.pt.x / downscaleFactor + radius * cos(angle),
                    keypoint.pt.y / downscaleFactor - radius * sin(angle)),
        greenColor);
  }

  return outimg;
}

void VioVisualizer::showDebugImages(VisualizationData::Ptr& data) {
  const bool showMatches = true;
  std::vector<cv::Mat> out_images(parameters_.nCameraSystem.numCameras());
  if (showMatches) {
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
      out_images[i] = drawMatches(data, i);
    }
  } else {
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
      out_images[i] = drawColoredKeypoints(data, i);
    }
  }
  // draw
  for (size_t im = 0; im < parameters_.nCameraSystem.numCameras(); im++) {
    std::stringstream windowname;
    windowname << viewerNamePrefix_ << " " << im;
    cv::imshow(windowname.str(), out_images[im]);
  }
  cv::waitKey(1);
}

cv::Mat downscaleImage(cv::Mat in, int downscaleFactor) {
  cv::Mat out = in.clone();
  while (downscaleFactor > 1) {
    cv::Mat dst;
    cv::pyrDown(out, dst);
    downscaleFactor /= 2;
    out = dst;
  }
  return out;
}
} /* namespace swift_vio */
