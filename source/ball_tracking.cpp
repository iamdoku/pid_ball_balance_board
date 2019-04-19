#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <wiringSerial.h>

class FilterHSV {
public:
  FilterHSV(std::ifstream &is) {
    is >> low_H >> low_S >> low_V >> high_H >> high_S >> high_V;
  }
  FilterHSV(int lh, int ls, int lv, int hh, int hs, int hv)
      : low_H(lh), low_S(ls), low_V(lv), high_H(hh), high_S(hs), high_V(hv) {}
  FilterHSV() = default;

  cv::Scalar retLow() { return cv::Scalar(low_H, low_S, low_V); }
  cv::Scalar retHigh() { return cv::Scalar(high_H, high_S, high_V); }

  void calibrate(cv::VideoCapture &vc) {
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
        cv::imshow("Calibration", filter(vc));
      }
      cv::destroyAllWindows();
    } catch (std::runtime_error err) {
      std::cerr << "Please check your connection and/or software dependecies"
                << std::endl;
    }
  }

  cv::Mat filter(cv::VideoCapture &vc) {
    try {
      if (!vc.isOpened())
        throw std::runtime_error("Camera isn't open");
      cv::Mat frame, hsv_frame;
      vc >> frame;
      cv::flip(frame, frame, 0);
      return filter(frame);
    } catch (std::runtime_error err) {
      std::cerr << "Please check your connection and/or software dependecies"
                << std::endl;
    }
  }
  cv::Mat filter(cv::Mat &frame) {
    cv::Mat hsv_frame;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, retLow(), retHigh(), hsv_frame);
    return hsv_frame;
  }

private:
  int low_H = 0, high_H = 255;
  int low_S = 0, high_S = 255;
  int low_V = 0, high_V = 255;
};

int main() {
  int fd = serialOpen("/dev/ttyUSB0", 9600);
  if (fd < 0)
    return -1;

  serialPuts(fd, "a90");
  serialPuts(fd, "b90");
  serialPuts(fd, "c90");
  cv::VideoCapture camera(0);
  camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  FilterHSV ball(0, 83, 177, 30, 255, 255);
  FilterHSV board;

  ball.calibrate(camera);
  board.calibrate(camera);
  std::vector<cv::Vec3f> vec;
  cv::Mat frame_bgr, frame_filtered, threshold;
  while (cv::waitKey(1) == -1) {
    serialPuts(fd, "a90");
    serialPuts(fd, "b90");
    serialPuts(fd, "c90");
    camera >> frame_bgr;
    cv::flip(frame_bgr, frame_bgr, 0);
    frame_filtered = ball.filter(frame_bgr);
    // cv::GaussianBlur(frame_filtered, frame_filtered, cv::Size(9, 9), 2, 2);
    // cv::dilate(frame_filtered, frame_filtered, 0);
    // cv::erode(frame_filtered, frame_filtered, 0);
    // cv::threshold(frame_filtered, threshold, 100, 255, cv::THRESH_BINARY);
    /*cv::HoughCircles(frame_filtered, vec, cv::HOUGH_GRADIENT, 4,
            frame_filtered.rows / 1, 50, 35, 20, 50);
    for (size_t i = 0; i != vec.size(); ++i) {
            std::cout << "Loop" << std::endl;
            int radius = cvCeil(vec[0][2]);
            cv::Point center(vec[0][0], vec[0][1]);
            cv::circle(frame_bgr, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
            cv::circle(frame_bgr, center, radius, cv::Scalar(0, 0, 255), 3, 8,
    0);
    }*/
    cv::Moments m = cv::moments(frame_filtered, true);
    cv::Point p(m.m10 / m.m00, m.m01 / m.m00);
    cv::circle(frame_bgr, p, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::imshow("Ball", frame_bgr);
  }
}