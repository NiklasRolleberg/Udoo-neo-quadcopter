#include "pid.h"

Pidcontroller::Pidcontroller(double* in, double* out, double* referenceValue,
                double* P, double* I, double* D,
                double minOutput, double maxOutput,
                double IntegralLimits)
{
  input = in;
  output = out;
  ref = referenceValue;

  kP = P;
  kI = I;
  kD = D;

  output_limit = (maxOutput - minOutput) / 2.0;
  output_offset = maxOutput - output_limit;
  Ilimit = IntegralLimits;
}


void Pidcontroller::update()
{  
  if(lastUpdate == 0) //first update/ first update after restart
  {
    lastUpdate = micros();
    lasterror = *ref - *input;
  }  
  double timediff = millis() - lastUpdate;
  double error = *ref - *input;
  I = min(Ilimit, max(-Ilimit, I + (*kI)*error ));

  double p = (*kP)*error;
  double d = 0;
  if(timediff>0)
  {
    d = (*kD) * (error-lasterror);
    lasterror = error;
  }
  *output = output_offset + min(output_limit, max(-output_limit, p + I + d ));
  /*
  Serial.print(error);
  Serial.print(",");
  Serial.print(p);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print(d);
  Serial.print(",");
  */
  //Serial.print("pid Error: ");
  //Serial.println(*output);
}

void Pidcontroller::reset()
{

}
