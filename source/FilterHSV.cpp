#include "FilterHSV.h"

FilterHSV::FilterHSV(std::ifstream &is, cv::VideoCapture &stream) : vc(stream) {
  is >> low_H >> low_S >> low_V >> high_H >> high_S >> high_V;
}
FilterHSV::FilterHSV(int lh, int ls, int lv, int hh, int hs, int hv,
                     cv::VideoCapture &stream)
    : low_H(lh), low_S(ls), low_V(lv), high_H(hh), high_S(hs), high_V(hv),
      vc(stream) {}

inline cv::Scalar FilterHSV::retLow() {
  return cv::Scalar(low_H, low_S, low_V);
}
inline cv::Scalar FilterHSV::retHigh() {
  return cv::Scalar(high_H, high_S, high_V);
}

void FilterHSV::calibrate(cv::VideoCapture &vc) {
  try {
    if (!vc.isOpened())
      throw std::runtime_error("Camera isn't open");

    cv::namedWindow("Calibration", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("LowH", "Calibration", &low_H, 255);
    cv::createTrackbar("LowS", "Calibration", &low_S, 255);
    cv::createTrackbar("LowV", "Calibration", &low_V, 255);
    cv::createTrackbar("HighH", "Calibration", &high_H, 255);
    cv::createTrackbar("HighS", "Calibration", &high_S, 255);
    cv::createTrackbar("HighV", "Calibration", &high_V, 255);

    while (cv::waitKey(10) == -1) {
      cv::imshow("Calibration", filter());
    }
    cv::destroyAllWindows();
  } catch (std::runtime_error err) {
    std::cerr << "Please check your connection and/or software dependecies"
              << std::endl;
  }
}

cv::Mat FilterHSV::filter() {
  try {
    if (!vc.isOpened())
      throw std::runtime_error("Camera isn't open");
    cv::Mat frame, hsv_frame;
    vc >> frame;
    cv::flip(frame, frame, +1);
    return filter(frame);
  } catch (std::runtime_error err) {
    std::cerr << "Please check your connection and/or software dependecies"
              << std::endl;
  }
}
cv::Mat FilterHSV::filter(cv::Mat &frame) {
  cv::Mat hsv_frame;
  cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
  cv::inRange(hsv_frame, retLow(), retHigh(), hsv_frame);
  return hsv_frame;
}

cv::Point2d FilterHSV::getCenter() {
  cv::Moments m = cv::moments(filter(), true);
  return cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
}

double FilterHSV::getRadius() {
  cv::Moments m = cv::moments(filter(), true);
  return sqrt(m.m00 / PI);
}