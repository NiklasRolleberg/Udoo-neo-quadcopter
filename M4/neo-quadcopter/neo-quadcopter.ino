#include <Wire.h>
#include "FXAS21002C.h"
#include "FXOS8700CQ.h"
#include "MadgwickAHRS.h"
#include <Servo.h>
#include "pid.h"


#define M_pi 3.14159265359

FXAS21002C gyro = FXAS21002C(0x20); // SA0=1 0x21
FXOS8700CQ accMag = FXOS8700CQ(0x1E);

extern volatile float q0, q1, q2, q3;  // quaternion elements representing the estimated orientation
double phi, theta, psi;

//phi = pitch (fram/bak)
//theta = roll (luta höger/vänster)
//psi = yaw (snurra)

//serial buffers
char byteRead;
char buffer[64];
volatile uint8_t buffptr = 0;
char csBuffer[2];
char tempBuffer[10];

// servo stuff
uint8_t esc0_pin = 4;
uint8_t esc1_pin = 5;
uint8_t esc2_pin = 6;
uint8_t esc3_pin = 7;


Servo esc0;
Servo esc1;
Servo esc2;
Servo esc3;


int minPulse = 1000;
int maxPulse = 2000;
int esc0_pulse = 1500;
int esc1_pulse = 1500;
int esc2_pulse = 1500;
int esc3_pulse = 1500;

//for PID controllers
double limit_min = -500;
double limit_max = 500;
double Ilimit = 100;


double p_pitch = 0;
double i_pitch = 0;
double d_pitch = 0;
double output_pitch = 0;
double target_pitch = 0;
Pidcontroller pid_pitch(&phi,&output_pitch,&target_pitch,
                       p_pitch,i_pitch,d_pitch,
                       limit_min,limit_max,Ilimit);

double p_roll = 0;
double i_roll = 0;
double d_roll = 0;
double output_roll = 0;
double target_roll = 0;
Pidcontroller pid_roll(&theta,&output_roll,&target_roll,
                       p_roll,i_roll,d_roll,
                       limit_min,limit_max,Ilimit);

double p_yaw = 0;
double i_yaw = 0;
double d_yaw = 0;
double output_yaw = 0;
double target_yaw = 0;
Pidcontroller pid_yaw(&psi,&output_yaw,&target_yaw,
                       p_yaw,i_yaw,d_yaw,
                       limit_min,limit_max,Ilimit);



/**
 * Modes:
 * 0 = standby -> send no pulse or idle
 * 1 = calculate pulse length based on aircraft orientation
 * 2 = send pulse length set by the operator
*/
int mode = 0;

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
  
  esc0.attach(esc0_pin);
  esc1.attach(esc1_pin);
  esc2.attach(esc2_pin);
  esc3.attach(esc3_pin);
  
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
  uint32_t servoupdate = micros();

  uint32_t calculationTime;
  while(true)
  {
    //200hz update frequency
    if(micros() - lastUpdate >= 5000)
    {
      lastUpdate = micros();
      update(); //update angles
      updateServo(); //update pwn signals
      calculationTime = micros()-lastUpdate;
    }

    if(micros() - lastWrite >= 500000)
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

    //Check serial port
    if (Serial.available())
    {
      //Serial.println("Reading");
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

void updateServo()
{
  switch (mode){
    case 1:
    {
      /*
      pid_roll.update();
      pid_pitch.update();
      pid_yaw.update();
      */
      
      // TODO generate pulses based on pid output
      int front_left = 1500 + 500.0*sin(phi);
      int front_right = 1500 + 500.0*sin(theta);
      int rear_left = 1500 + 500.0*sin(psi);
      int rear_right = 1500 + 500.0*sin(phi);

      esc0.writeMicroseconds(min(maxPulse,max(minPulse,front_left)));
      esc1.writeMicroseconds(min(maxPulse,max(minPulse,front_right)));
      esc2.writeMicroseconds(min(maxPulse,max(minPulse,rear_left)));
      esc3.writeMicroseconds(min(maxPulse,max(minPulse,rear_right)));
      break;
    }
    case 2:
    {
      esc0.writeMicroseconds(esc0_pulse);
      esc1.writeMicroseconds(esc1_pulse);
      esc2.writeMicroseconds(esc2_pulse);
      esc3.writeMicroseconds(esc3_pulse);
      break;
    }
    default:
    {
      esc0.writeMicroseconds(1000);
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      break;
    }
  }
}

void decodeMessage()
{
  //Serial.println("Decode messsage");
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
  //Serial.println("Checksum checks out");

  /**Decode messages*/

  /**Write current angle*/
  if (strncmp(buffer+startIndex+1,"QCANG",5)==0) {
    Serial.print("Current angle: " );
    Serial.print(phi*(180/M_pi));
    Serial.print(",");
    Serial.print(theta*(180/M_pi));
    Serial.print(",");
    Serial.println(psi*(180/M_pi));
  }

  /**Set Target Angles*/
  /*"$QCSTA,roll,pitch,yaw,*CS"*/
  if (strncmp(buffer+startIndex+1,"QCSTA",5)==0) {
    //Serial.println("Set Target Angles message");

    //find 1:st pulse
    startIndex+=7;
    for(it=startIndex;it<=stopIndex;it++) {
      if(buffer[it] == ',' || buffer[it] == '*') {
        strncpy (tempBuffer, buffer+startIndex, it-startIndex );
        target_roll = max(-M_pi/4,min(M_pi/4,strtod(tempBuffer,NULL)));
        break;
      }
    }

    //find 2:nd pulse
    startIndex = it+1;
    for(it=startIndex;it<=stopIndex;it++) {
      if(buffer[it] == ',' || buffer[it] == '*') {
        strncpy (tempBuffer, buffer+startIndex, it-startIndex );
        target_pitch = max(-M_pi/4,min(M_pi/4,strtod(tempBuffer,NULL)));
        break;
      }
    }

    //find 3:rd yaw
    startIndex = it+1;
    for(it=startIndex;it<=stopIndex;it++) {
      if(buffer[it] == ',' || buffer[it] == '*') {
        strncpy (tempBuffer, buffer+startIndex, it-startIndex );
        target_yaw = max(-M_pi/4,min(M_pi/4,strtod(tempBuffer,NULL)));
        break;
      }
    }
    /*
    Serial.print("Target_roll:");
    Serial.println(target_roll);
    Serial.print("Target_pitch:");
    Serial.println(target_pitch);
    Serial.print("Target_yaw:");
    Serial.println(target_yaw);
    */

    mode = 1; //set mode: fly!
  }

  /**set pulse length*/
  /*"$QCPUL,pulse1,pulse2,pulse3,pulse4,*CS"*/
  if (strncmp(buffer+startIndex+1,"QCPUL",5)==0) {
    //Serial.println("Set pulse length message received");
  
    //find 1:st pulse
    startIndex+=7;
    for(it=startIndex;it<=stopIndex;it++) {
      if(buffer[it] == ',' || buffer[it] == '*') {
        strncpy (tempBuffer, buffer+startIndex, it-startIndex );
        esc0_pulse = max(minPulse,min(maxPulse,(int)strtol(tempBuffer,NULL,10)));
        break;
      }
    }

    //find 2:nd pulse
    startIndex = it+1;
    for(it=startIndex;it<=stopIndex;it++) {
      if(buffer[it] == ',' || buffer[it] == '*') {
        strncpy (tempBuffer, buffer+startIndex, it-startIndex );
        esc1_pulse = max(minPulse,min(maxPulse,(int)strtol(tempBuffer,NULL,10)));
        break;
      }
    }

    //find 3:rd pulse
    startIndex = it+1;
    for(it=startIndex;it<=stopIndex;it++) {
      if(buffer[it] == ',' || buffer[it] == '*') {
        strncpy (tempBuffer, buffer+startIndex, it-startIndex );
        esc2_pulse = max(minPulse,min(maxPulse,(int)strtol(tempBuffer,NULL,10)));
        break;
      }
    }

    //find 4:th pulse
    startIndex = it+1;
    for(it=startIndex;it<=stopIndex;it++) {
      if(buffer[it] == ',' || buffer[it] == '*') {
        strncpy (tempBuffer, buffer+startIndex, it-startIndex );
        esc3_pulse = max(minPulse,min(maxPulse,(int)strtol(tempBuffer,NULL,10)));
        break;
      }
    }
    /*
    Serial.println("Pulses:");
    Serial.print("esc0: ");
    Serial.println(esc0_pulse);
    Serial.print("esc1: ");
    Serial.println(esc1_pulse);
    Serial.print("esc2: ");
    Serial.println(esc2_pulse);
    Serial.print("esc3: ");
    Serial.println(esc3_pulse);
    */
    mode = 2; //send these pulses
  }
}

int calculateChecksum(uint8_t start,uint8_t stop)
{
    int c = 0;
    for(uint8_t i=start;i<stop;i++)
      c ^= buffer[i];
    return c;
}
