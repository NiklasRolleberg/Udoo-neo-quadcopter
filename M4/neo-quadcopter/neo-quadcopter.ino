#include <Wire.h>
#include "FXAS21002C.h"
#include "FXOS8700CQ.h"
#include "MadgwickAHRS.h"

#define M_pi 3.14159265359

FXAS21002C gyro = FXAS21002C(0x20); // SA0=1 0x21
FXOS8700CQ accMag = FXOS8700CQ(0x1E);

extern volatile float q0, q1, q2, q3;  // quaternion elements representing the estimated orientation
float phi, theta, psi;

//serial buffers
char byteRead;
char buffer[64];
volatile uint8_t buffptr = 0;
char csBuffer[2];
char tempBuffer[10];

void setup()
{
  // initialize digital pin 13 as an output
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(115200);
  Wire1.begin();

  // Initialize the FXAS21002C
  gyro.init();

  // Initialize the FXOS8700CQ
  accMag.init();

  //gyro.calibrate(gyro.gBias);
}

void loop()
{
  // Gyro resolution?
  gyro.getGres();

  //calibrate gyro
  digitalWrite(13, HIGH);
  gyro.calibrate();
  digitalWrite(13, LOW);


  uint32_t lastWrite = micros();
  uint32_t lastUpdate = micros();

  uint32_t calculationTime;
  while(true)
  {
    //200hz update frequency
    if(micros() - lastUpdate >= 5000)
    {
      lastUpdate = micros();
      update();
      calculationTime = micros()-lastUpdate;
    }

    if(micros() - lastWrite >= 200000)
    {
      lastWrite = micros();

      Serial.print("$QCEUL,");
      Serial.print(phi*(180/M_pi));
      Serial.print(",");
      Serial.print(theta*(180/M_pi));
      Serial.print(",");
      Serial.print(psi*(180/M_pi));
      Serial.println("*00");
    }

    /*
    //Check serial port
    if (Serial.available())
    {
      Serial.println("Reading");
      // read the most recent byte
      byteRead = Serial.read();
      if(byteRead == '\n')
      {
        decodeMessage();
        buffptr = 0;

        //reset buffers
        for(int8_t i=0;i<64;i++)
        {
          buffer[i] = 0;
          if(i<10)
          {
            tempBuffer[i] = 0;
          }
        }
      }
      else
      {
        buffer[buffptr % 64] = byteRead;
        buffptr++;
      }
    }
    */
  }
}


void update()
{
  // Query the sensors
  gyro.readGyroData();
  accMag.readAccelData();
  accMag.readMagData();

  MadgwickAHRSupdate((M_pi/180)*gyro.gyroData.x*gyro.gRes
                    ,(M_pi/180)*gyro.gyroData.y*gyro.gRes
                    ,(M_pi/180)*gyro.gyroData.z*gyro.gRes
                    ,-accMag.accelData.x*accMag.getAres()
                    , accMag.accelData.y*accMag.getAres()
                    ,-accMag.accelData.z*accMag.getAres()
                    ,-accMag.magData.x*accMag.getMres()
                    , accMag.magData.y*accMag.getMres()
                    ,-accMag.magData.z*accMag.getMres());

  float qq0 = q0;
  float qq1 = -q1;
  float qq2 = -q2;
  float qq3 = -q3;

  float R11 = 2.*qq0*qq0 -1 +2.*qq1*qq1;
  float R21 = 2.*(qq1*qq2 - qq0*qq3);
  float R31 = 2.*(qq1*qq3 + qq0*qq2);
  float R32 = 2.*(qq2*qq3 - qq0*qq1);
  float R33 = 2.*qq0*qq0 -1 +2.*qq3*qq3;

  phi = atan2(R32, R33 );
  theta = -atan(R31 / sqrt(1-R31*R31) );
  psi = -atan2(R21, R11 );
}

void decodeMessage()
{
  Serial.println("Decode messsage");
  //check for start char '$'
  int8_t startIndex = -1;
  int8_t stopIndex = -1;
  int8_t it;
  for(it=0;it<128;it++) {
    if(buffer[it]=='$') {startIndex = it; break;}
  }
  for(it=0;it<128;it++) {
    if(buffer[it]=='*') {stopIndex = it; break;}
  }
  if(128-stopIndex < 2) {
    return;
  }
  if(startIndex == -1 || stopIndex == -1) {
    return;
  }
  csBuffer[0] = buffer[stopIndex+1];
  csBuffer[1] = buffer[stopIndex+2];
  int checksumIn = (int)strtol(csBuffer,NULL,16);
  if(calculateChecksum(startIndex,stopIndex) != checksumIn && checksumIn != 0)
    return;
  Serial.println("Checksum checks out");

  /**Decode messages*/
  if (strncmp(buffer+startIndex+1,"QCANG",5)==0) {
    Serial.print("Current angle: " );
    Serial.print(phi*(180/M_pi));
    Serial.print(",");
    Serial.print(theta*(180/M_pi));
    Serial.print(",");
    Serial.println(psi*(180/M_pi));
  }
}

int calculateChecksum(uint8_t start,uint8_t stop)
{
    int c = 0;
    for(uint8_t i=start;i<stop;i++)
      c ^= buffer[i];
    return c;
}

