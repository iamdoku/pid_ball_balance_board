#include <iostream>
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


int main(){
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
  FilterHSV board(0, 0, 130, 255, 255, 255);

  ball.calibrate(camera);
  board.calibrate(camera);

  cv::Mat frame_bgr, frame_filtered_ball, frame_filtered_board, threshold;
  while (cv::waitKey(1) == -1) {
    serialPuts(fd, "a90");
    serialPuts(fd, "b90");
    serialPuts(fd, "c90");
    camera >> frame_bgr;
    cv::flip(frame_bgr, frame_bgr, 0);

    frame_filtered_ball = ball.filter(frame_bgr);
    frame_filtered_board = ball.filter(frame_bgr);


    cv::Moments m_ball = cv::moments(frame_filtered_ball, true);
    cv::Moments m_board = cv::moments(frame_filtered_board, true);

    cv::Point center_ball(m_ball.m10 / m_ball.m00, m_ball.m01 / m_ball.m00);
    cv::Point center_board(m_board.m10 / m_board.m00, m_board.m01 / m_board.m00);

    cv::circle(frame_bgr, center_ball, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::circle(frame_bgr, center_board, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::line(frame_bgr, center_ball, center_board, cv::Scalar(0, 0, 255), 3);

    cv::imshow("Ball", frame_bgr);
  }
}