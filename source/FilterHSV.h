#ifndef FILTER_HSV_H
#define FILTER_HSV_H
#include <iostream>
#include <stdexcept>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


const double PI = 3.14159265;

class FilterHSV {
public:
  FilterHSV(std::ifstream &is, cv::VideoCapture &stream);
  FilterHSV(int lh, int ls, int lv, int hh, int hs, int hv,
            cv::VideoCapture &stream);
  FilterHSV() = default;

  cv::Scalar retLow();
  cv::Scalar retHigh();

  void calibrate(cv::VideoCapture &vc);

  cv::Mat filter();
  cv::Mat filter(cv::Mat &frame);

  cv::Point2d getCenter();

  double getRadius();

private:
  cv::VideoCapture &vc;
  int low_H = 0, high_H = 255;
  int low_S = 0, high_S = 255;
  int low_V = 0, high_V = 255;
};
#endif