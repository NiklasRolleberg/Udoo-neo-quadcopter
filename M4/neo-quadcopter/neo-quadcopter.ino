#include <Wire.h>
#include "FXAS21002C.h"
#include "FXOS8700CQ.h"
#include "MadgwickAHRS.h"
#include <Servo.h>
#include "pid.h"


#define M_pi 3.14159265359
#define bufferSize 64

FXAS21002C gyro = FXAS21002C(0x20); // SA0=1 0x21
FXOS8700CQ accMag = FXOS8700CQ(0x1E);

extern volatile float q0, q1, q2, q3;  // quaternion elements representing the estimated orientation
float phi, theta, psi; //orientation
float dphi,dtheta,dpsi; //angular velocity

//phi = pitch (fram/bak)
//theta = roll (luta höger/vänster)
//psi = yaw (snurra)

//const char base61_numbers[]="0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
//char eulerBuffer[] = "<eAABBCC>";

//serial buffers
char byteRead;
char buffer[bufferSize];
volatile uint8_t buffptr = 0;
char csBuffer[2];
char tempBuffer[10];

//Serial0 Buffers
char radioBuffer[bufferSize];
volatile uint8_t serial0ptr = 0;

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
int esc0_pulse = 1000;
int esc1_pulse = 1000;
int esc2_pulse = 1000;
int esc3_pulse = 1000;

//for PID controllers
float limit_min = -128;
float limit_max = 128;
float Ilimit = 100;
float max_angularVel = 10; //10 degrees/sec


float p_pitch = 0.5;
float i_pitch = 0;
float d_pitch = 0;
float output_pitch = 0;
float target_pitchV = 0; //target angular velocity
float target_pitch = 0;  //target absolute angle
Pidcontroller pid_pitch(&dphi,&output_pitch,&target_pitchV,
                       &p_pitch,&i_pitch,&d_pitch,
                       limit_min,limit_max,Ilimit);

float p_roll = 0.5;
float i_roll = 0;
float d_roll = 0;
float output_roll = 0;
float target_rollV = 0;
float target_roll = 0;
Pidcontroller pid_roll(&dtheta,&output_roll,&target_rollV,
                       &p_roll,&i_roll,&d_roll,
                       limit_min,limit_max,Ilimit);

float p_yaw = 0;
float i_yaw = 0;
float d_yaw = 0;
float output_yaw = 0;
float target_yawV = 0;
float target_yaw = 0;
Pidcontroller pid_yaw(&dpsi,&output_yaw,&target_yawV,
                       &p_yaw,&i_yaw,&d_yaw,
                       limit_min,limit_max,Ilimit);


/**
 * Throttle: base pulse length before pid addiations
 * 0 to maxthrottle
 */
int maxThrottle = 600;
int throttle = 0;

/**
 * Modes:
 * 0 = idle. Send idle pulse
 * 1 = armed
 * 2 = calculate pulse length based on aircraft orientation
 * 3 = send pulse length set by the operator
*/
int mode = 0; //should be 0 to begin with

void setup()
{
  // initialize digital pin 13 as an output
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(115200);
  //Serial0.begin(115200);
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

  
  esc0.writeMicroseconds(1000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  
}

void loop()
{

  //delay(10000); //10s delay
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
    if(micros() - lastUpdate >= 5000) //5000
    {
      lastUpdate = micros();
      update(); //update angles
      updateServo(); //update pwm signals
      calculationTime = micros()-lastUpdate;
    }

    if(micros() - lastWrite >= 10000000)
    {
      lastWrite = micros();
      Serial.println("\nM4 is still running");
      /*
      Serial.print(" angular Vel (pitch):  ");
      Serial.print(dphi);
      Serial.print(" Output pitch: ");
      Serial.println(output_pitch);

      Serial.print(" angular Vel (roll):   ");
      Serial.print(dtheta);
      Serial.print(" Output roll:  ");
      Serial.println(output_roll);

      Serial.print(" angular Vel (yaw):    ");
      Serial.print(dpsi);
      Serial.print(" Output yaw:  ");
      Serial.println(output_yaw);
      */
    }
    
    //Check serial port
    if (Serial.available())
    {
      //Serial.println("Reading");
      // read the most recent byte
      byteRead = Serial.read();
      if(byteRead == '>')
      {
        buffer[buffptr % bufferSize] = byteRead;
        buffptr++;
        decodeBuffer(&buffer[0],buffptr);
        buffptr = 0;

        //reset buffers
        for(int8_t i=0;i<bufferSize;i++)
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
        buffer[buffptr % bufferSize] = byteRead;
        buffptr++;
      }
    }
    /*
    //Check other serial port
    if (Serial0.available())
    {
      byteRead = Serial0.read();
      if(byteRead == '\n')
      {
        radioBuffer[serial0ptr % 64] = byteRead;
        serial0ptr++;
        decodeBuffer(&radioBuffer[0],buffptr);
        serial0ptr = 0;

        //reset buffers
        for(int8_t i=0;i<64;i++)
        {
          radioBuffer[i] = 0;
        }
      }
      else
      {
        radioBuffer[serial0ptr % bufferSize] = byteRead;
        serial0ptr++;
      }
    }*/
  }
}

void update()
{
  // Query the sensors
  gyro.readGyroData();
  accMag.readAccelData();
  accMag.readMagData();

  dphi = gyro.gyroData.x*gyro.gRes;
  dtheta = gyro.gyroData.y*gyro.gRes;
  dpsi = gyro.gyroData.z*gyro.gRes;
  
  MadgwickAHRSupdate((M_pi/180)*dphi  //X
                    ,(M_pi/180)*dtheta  // Y
                    ,(M_pi/180)*dpsi // Z
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
    case 2:
    {
      /*
      //set target angular velocity based on aircraft orientation
      target_pitchV = 0.5*max(-max_angularVel,min(max_angularVel,phi-target_pitch));
      target_rollV = 0.5*max(-max_angularVel,min(max_angularVel,theta-target_roll));
      target_yawV = 0.5*max(-max_angularVel,min(max_angularVel,psi-target_yaw));
      */
      
      target_pitchV = 0;
      target_rollV = 0;
      target_yawV = 0;
      
      pid_pitch.update();
      pid_roll.update();
      //pid_yaw.update();
      
      // TODO add yaw
      int front_left  = 1000 + throttle - output_roll - output_pitch;
      int front_right = 1000 + throttle + output_roll - output_pitch;
      int rear_left   = 1000 + throttle - output_roll + output_pitch;
      int rear_right  = 1000 + throttle + output_roll + output_pitch;
     
      esc0.writeMicroseconds(min(maxPulse,max(minPulse,front_left)));
      esc1.writeMicroseconds(min(maxPulse,max(minPulse,front_right)));
      esc2.writeMicroseconds(min(maxPulse,max(minPulse,rear_left)));
      esc3.writeMicroseconds(min(maxPulse,max(minPulse,rear_right)));      
      break;
    }
    case 3:
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

void decodeBuffer(char* buff, uint8_t ptr)
{
  uint8_t startChar = bufferSize+1; // '<'
  uint8_t endChar = bufferSize+1;   // '>'

  //find start character '<'
  for(int i=0;i<ptr;i++)
  {
    if( (*(buff+i)) == '<')
    {
      startChar = i;
    }
    if((*(buff+i)) == '>')
    {
      endChar = i;
      break;
    }
  }
  //Did not find start and end character
  if(startChar>endChar || startChar == bufferSize+1 || endChar == bufferSize+1)
    return;

  //find type
  char type = *(buff+startChar+1);
  uint8_t message_length = endChar-startChar;

  //Decode the message
  switch(type) {
    case 'A':
      //Serial.println("Arm");
      if(message_length == 3)
        decodeArm(buff,startChar,endChar);
      break;
    case 'S':
      //Serial.println("Stop");
      if(message_length == 3)
        decodeStop(buff,startChar,endChar);
      break;
    case 'E':
      //Serial.println("control data");
      if(message_length == 10)
        decodeEulerData(buff,startChar,endChar);
      break;
    case 'P':
      //Serial.println("Manual control data");
      if(message_length == 10)
        decodePulseData(buff,startChar,endChar);
      break;
    case 'e':
      if(message_length == 3)
        sendEulerData(0);
      break;
    case 'C':
      if(message_length == 3)
        calibrate();
      break;    
  }
  //Serial.print("Curreant mode: ");
  //Serial.println(mode);
}

void decodeArm(char* buff, uint8_t startPtr, uint8_t endPtr) 
{
  if(mode == 0 || mode == 1)
  {
    mode = 1;
    throttle = 0;
    target_pitch = 0;
    target_roll= 0;
    target_yaw= 0;

    target_pitchV = 0;
    target_rollV= 0;
    target_yawV= 0;
  }
  if((*(buff+startPtr+2)=='1'))
  {
    Serial.println("<A2>");
  }
}

void decodeStop(char* buff, uint8_t startPtr, uint8_t endPtr) 
{
  mode = 0;
  throttle = 0;
  target_pitchV = 0;
  target_rollV= 0;
  target_yawV= 0;

  target_pitch = 0;
  target_roll= 0;
  target_yaw= 0;
  
  if((*(buff+startPtr+2)=='1'))
  {
    Serial.println("<S2>");
  }
}

void decodeEulerData(char* buff, uint8_t startPtr, uint8_t endPtr)
{
  if(mode == 0 || mode == 3) //idle or pulse-mode
    return;
 
   mode = 2; //set mode to 2 (it could have been "armed" before)

   int t_pitch = twoByteToInt(buff+startPtr+2);
   target_pitch = max(-M_pi,min(M_pi,(t_pitch-500)*0.05*(M_pi/180)));
   int t_roll = twoByteToInt(buff+startPtr+4);
   target_roll = max(-M_pi,min(M_pi,(t_roll-500)*0.05*(M_pi/180)));
   int t_yaw = twoByteToInt(buff+startPtr+6);   
   target_yaw = max(-M_pi,min(M_pi,(t_yaw-500)*0.05*(M_pi/180)));
   throttle = max(0,min(maxThrottle,twoByteToInt(buff+startPtr+8)));
}

void decodePulseData(char* buff, uint8_t startPtr, uint8_t endPtr)
{
  if(mode == 0 || mode == 2) //idle or flight mode
    return;
  
  mode = 3; //set mode to 3 (it could have been "armed" before)
  
  esc0_pulse = 1000 + max(0,min(1000,twoByteToInt(buff+startPtr+2)));
  esc1_pulse = 1000 + max(0,min(1000,twoByteToInt(buff+startPtr+4)));
  esc2_pulse = 1000 + max(0,min(1000,twoByteToInt(buff+startPtr+6)));   
  esc3_pulse = 1000 + max(0,min(1000,twoByteToInt(buff+startPtr+8)));

  
  Serial.println("Pulses:");
  Serial.print("esc0: ");
  Serial.println(esc0_pulse);
  Serial.print("esc1: ");
  Serial.println(esc1_pulse);
  Serial.print("esc2: ");
  Serial.println(esc2_pulse);
  Serial.print("esc3: ");
  Serial.println(esc3_pulse);
  
}

int twoByteToInt(char* b)
{
  char c_MSB = *(b);
  char c_LSB = *(b+1);
  uint8_t i_MSB = (int) c_MSB;
  uint8_t i_LSB = (int) c_LSB;

  int MSB = base61To10(i_MSB);
  int LSB = base61To10(i_LSB);

  int total = MSB*61+LSB;
  return total;
}

int base61To10(int b61) 
{ 
  if(48<=b61 && b61<=57) //Number between 0-9
    return b61-48;
  if(97<=b61 && b61<=122) //lower case letter a-z
    return 10 + b61-97;
  if(65<=b61 && b61<=90) //Upper case letter A-Z
    return 36 + b61-65;
  return 0;
}

void sendEulerData(int serialPort)
{
  Serial.print("<e,");
  Serial.print(phi*(180/M_pi));
  Serial.print(",");
  Serial.print(theta*(180/M_pi));
  Serial.print(",");
  Serial.print(psi*(180/M_pi));
  Serial.println(">");
}

void calibrate()
{
  //TODO add rotation matrix to accelerometer and find a way to calibrate the gyro
  if(mode == 0 || mode == 1) // Don't allow calibration when flying..
  {
    Serial.println("<tCALIBRATE>");
    digitalWrite(13, HIGH);
    gyro.calibrate();
    digitalWrite(13, LOW);
    Serial.println("<tDONE>"); 
  }
}

