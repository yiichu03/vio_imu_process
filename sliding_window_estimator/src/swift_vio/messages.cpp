#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <swift_vio/messages.h>

namespace swift_vio {
cv::Mat drawFeatureMatches(const PointMap &featureTracks,
                           std::shared_ptr<const MultiFrame> frame,
                           std::shared_ptr<const MultiFrame> keyframe,
                           size_t image_number) {
  if (keyframe == nullptr)
    return frame->image(image_number);

  // allocate an image
  const unsigned int im_cols = frame->image(image_number).cols;
  const unsigned int im_rows = frame->image(image_number).rows;
  const unsigned int rowJump = im_rows;

  cv::Mat outimg(2 * im_rows, im_cols, CV_8UC3);
  // copy current images Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
  cv::Mat current = outimg(cv::Rect(0, rowJump, im_cols, im_rows));
  cv::Mat actKeyframe = outimg(cv::Rect(0, 0, im_cols, im_rows));

  cv::cvtColor(frame->image(image_number), current, cv::COLOR_GRAY2BGR);
  cv::cvtColor(keyframe->image(image_number), actKeyframe, cv::COLOR_GRAY2BGR);

  for (auto it = featureTracks.begin(); it != featureTracks.end(); ++it) {
    const auto &obsList = it->second.observations;
    auto oit = std::find_if(obsList.begin(), obsList.end(),
                            IsObservedInFrame(frame->id(), image_number));
    auto oitk = std::find_if(obsList.begin(), obsList.end(),
                             IsObservedInFrame(keyframe->id(), image_number));

    if (oit == obsList.end())
      continue;
    const Eigen::Vector2f &keypoint = oit->second.uv;

    cv::Scalar color;
    if (oitk != obsList.end()) {
      color = cv::Scalar(255, 0, 0); // blue
      // found in the keyframe. draw line
      const Eigen::Vector2f &keyframePt = oitk->second.uv;
      cv::line(outimg, cv::Point2f(keyframePt[0], keyframePt[1]),
               cv::Point2f(keypoint[0], keypoint[1] + rowJump), color, 1,
               cv::LINE_AA);
      cv::circle(actKeyframe, cv::Point2f(keyframePt[0], keyframePt[1]),
                 0.5 * oitk->second.size, color, 1, cv::LINE_AA);

    } else {
      color = cv::Scalar(0, 0, 255); // red
    }

    // draw keypoint in current frame.
    const double r = 0.5 * oit->second.size;
    cv::circle(current, cv::Point2f(keypoint[0], keypoint[1]), r, color, 1,
               cv::LINE_AA);
    cv::KeyPoint cvKeypoint;
    frame->getCvKeypoint(image_number, oit->first.keypointIndex, cvKeypoint);
    const double angle = cvKeypoint.angle / 180.0 * M_PI;
    cv::line(outimg, cv::Point2f(keypoint[0], keypoint[1] + rowJump),
             cv::Point2f(keypoint[0], keypoint[1] + rowJump) +
                 cv::Point2f(cos(angle), sin(angle)) * r,
             color, 1, cv::LINE_AA);

  } // for every track
  return outimg;
}

void showMatchImages(const PointMap &pointMap,
    std::shared_ptr<const MultiFrame> frame,
    std::shared_ptr<const MultiFrame> keyframe) {
  size_t numCameras = frame->numFrames();
  std::vector<cv::Mat> out_images(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    out_images[i] = drawFeatureMatches(pointMap,
                                       frame,
                                       keyframe, i);
  }

  // draw
  for (size_t im = 0; im < numCameras; im++) {
    std::stringstream windowname;
    windowname << "Frontend matches for camera "
               << " " << im;
    cv::imshow(windowname.str(), out_images[im]);
    cv::waitKey(30);
  }
}

} // namespace swift_vio
