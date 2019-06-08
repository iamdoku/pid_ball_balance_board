#include "PID.h"

PID::PID(double p, double i, double d) : kp(p), ki(i), kd(d) {}

inline double PID::calculateError(const double &w, const double &y) {
  e = w - y;
  return e;
}

double PID::control(const double &w, const double &y) {
  double prev_e = e;
  calculateError(w, y);
  ie += e * (1.0 / 30.0);
  double x = (kp * e); //(ki * ie) + (kd * ((e - prev_e) / (1.0 / 30.0)));
  return x;
}