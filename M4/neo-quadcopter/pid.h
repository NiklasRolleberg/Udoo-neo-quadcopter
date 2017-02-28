#ifndef PID_H
#define PID_H

#include <Arduino.h>

class Pidcontroller
{
private:
  uint32_t lastUpdate = 0;

  float* input;
  float* output;
  float* ref;

  float* kP;
  float* kI;
  float* kD;

  float I = 0;
  float lasterror = 0;

  float output_limit = 0;
  float output_offset = 0;
  float Ilimit = 0;


public:
  Pidcontroller(float* in, float* out, float* referenceValue,
                float* P, float* I, float* D,
                float minOutput, float maxOutput,
                float IntegralLimits);


  void update();
  void reset();
};

#endif //PID_H
