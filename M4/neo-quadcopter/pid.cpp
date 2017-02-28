#include "pid.h"

Pidcontroller::Pidcontroller(float* in, float* out, float* referenceValue,
                float* P_value, float* I_value, float* D_value,
                float minOutput, float maxOutput,
                float IntegralLimits)
{
  input = in;
  output = out;
  ref = referenceValue;

  kP = P_value;
  kI = I_value;
  kD = D_value;

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
  float timediff = millis() - lastUpdate;
  float error = *ref - *input;
  I = min(Ilimit, max(-Ilimit, I + (*kI)*error ));

  float p = (*kP)*error;
  float d = 0;
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
