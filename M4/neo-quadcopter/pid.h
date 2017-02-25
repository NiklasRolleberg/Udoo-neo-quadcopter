#ifndef PID_H
#define PID_H

#include <Arduino.h>

class Pidcontroller
{
private:
  uint32_t lastUpdate = 0;

  double* input;
  double* output;
  double* ref;

  double* kP;
  double* kI;
  double* kD;

  double I = 0;
  double lasterror = 0;

  double output_limit = 0;
  double output_offset = 0;
  double Ilimit = 0;


public:
  Pidcontroller(double* in, double* out, double* referenceValue,
                double* P, double* I, double* D,
                double minOutput, double maxOutput,
                double IntegralLimits);


  void update();
  void reset();
};

#endif //PID_H
