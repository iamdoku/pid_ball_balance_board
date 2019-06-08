#ifndef PID_H
#define PID_H

class PID {
public:
  PID(double p, double i, double d);

  double calculateError(const double &w, const double &y);
  double control(const double &w, const double &y);


private:
  double kp, ki, kd;
  double e = 0, ie = 0;
};

#endif