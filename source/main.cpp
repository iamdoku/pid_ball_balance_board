#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <wiringSerial.h>
#include "FilterHSV.h"
#include "PID.h"

double calculateAngle(const cv::Vec2d &v) {
  if (v[0] > 0 && v[1] > 0) {
    return atan(v[1] / v[0]);
  } else if (v[0] < 0 && v[1] > 0) {
    return PI - atan(v[1] / (-1 * v[0]));
  } else if (v[0] < 0 && v[1] < 0) {
    return PI + atan((-1 * v[1]) / (-1 * v[0]));
  } else if (v[0] > 0 && v[1] < 0) {
    return 2 * PI - atan((-1 * v[1]) / v[0]);
  } else
    return 0;
}



int main() {
  int fd = serialOpen("/dev/ttyUSB0", 9600);
  if (fd < 0)
    return -1;

  serialPuts(fd, "a1500");
  serialPuts(fd, "b1500");
  serialPuts(fd, "c1500");

  int a_level = 1750, b_level = 1750, c_level = 1750;

  cv::VideoCapture camera(0);
  camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  PID regulator(1, 0.8, 0.6);

  double x;

  FilterHSV ball(0, 83, 177, 30, 255, 255, camera);
  FilterHSV board(0, 0, 130, 255, 255, 255, camera);

  ball.calibrate(camera);
  board.calibrate(camera);

  cv::Point2d center_board = board.getCenter();
  double radius = board.getRadius();

  cv::Mat frame_bgr, frame_filtered_ball, frame_filtered_board, threshold;
  while (cv::waitKey(1) == -1) {
    camera >> frame_bgr;

    cv::flip(frame_bgr, frame_bgr, +1);
    frame_filtered_ball = ball.filter(frame_bgr);
    frame_filtered_board = board.filter(frame_bgr);

    cv::Moments m_ball = cv::moments(frame_filtered_ball, true);

    cv::Point2d center_ball(m_ball.m10 / m_ball.m00, m_ball.m01 / m_ball.m00);

    cv::Point2d diff = center_ball - center_board;
    cv::Vec2d ball_vec(diff);

    double amplitude = sqrt(pow(ball_vec[0], 2) + pow(ball_vec[1], 2));
    double angle = calculateAngle(ball_vec);
    // int x = 180 - (std::trunc(((1 * regulator.calculateError(0, amplitude) *
    // 90) / radius))+90);
    int x1 = std::trunc(3 * regulator.control(0, amplitude) * 500 / radius);
    int x2 = std::trunc(1.5 * regulator.control(0, amplitude) * 500 / radius);
    std::cout << x << std::endl;
    // std::cout << x << std::endl;

    if (angle <= (2.0 / 3.0) * PI) {
      a_level = 2000 + x1;
      c_level = 2000 + x1;
      b_level = 1500 - x2;
    } else if (angle > (2.0 / 3.0) * PI && angle <= (4.0 / 3.0) * PI) {
      a_level = 2000 + x1;
      b_level = 2000 + x1;
      c_level = 1500 - x2;
    } else if (angle > (4.0 / 3.0) * PI && angle <= 2 * PI) {
      b_level = 2000 + x1;
      c_level = 2000 + x1;
      a_level = 1500 - x2;
    }

    std::stringstream ss_a, ss_b, ss_c;
    ss_a << "a" << a_level;
    ss_b << "b" << b_level;
    ss_c << "c" << c_level;

    serialPuts(fd, ss_a.str().c_str());
    serialPuts(fd, ss_b.str().c_str());
    serialPuts(fd, ss_c.str().c_str());

    ss_a.str(std::string());
    ss_b.str(std::string());
    ss_c.str(std::string());

    cv::Point2d alpha(radius * std::cos(PI * 2.0 / 3.0),
                      radius * std::sin(PI * 2.0 / 3.0));
    cv::Point2d beta(radius * std::cos(PI * 4.0 / 3.0),
                     radius * std::sin(PI * 4.0 / 3.0));
    cv::Point2d gamma(radius * std::cos(PI * 2), radius * std::sin(PI * 2));

    alpha = alpha + center_board;
    beta = beta + center_board;
    gamma = gamma + center_board;

    cv::circle(frame_bgr, center_ball, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::circle(frame_bgr, center_board, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    // cv::line(frame_bgr, center_ball, center_board, cv::Scalar(0, 0, 255), 3);
    cv::line(frame_bgr, center_board, alpha, cv::Scalar(0, 0, 255), 3);
    cv::line(frame_bgr, center_board, beta, cv::Scalar(0, 255, 000), 3);
    cv::line(frame_bgr, center_board, gamma, cv::Scalar(255, 0, 0), 3);

    cv::imshow("Ball", frame_bgr);
  }
}