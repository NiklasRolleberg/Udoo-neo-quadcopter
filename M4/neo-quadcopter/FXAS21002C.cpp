/*
 * derived from: https://github.com/sabas1080/FXAS21002C_Arduino_Library
 *
 * modified by Francesco Ferraro 01/2016
 * francesco.ferrarogm@gmail.com
 *
*/

#include <Wire.h>
#include <math.h>

#include "FXAS21002C.h"

// Public Methods //////////////////////////////////////////////////////////////


FXAS21002C::FXAS21002C(byte addr)
{
	address = addr;
	gyroODR = GODR_200HZ; //
	gyroFSR = GFS_1000DPS;
}

void FXAS21002C::writeReg(byte reg, byte value)
{
	Wire1.beginTransmission(address);
	Wire1.write(reg);
	Wire1.write(value);
	Wire1.endTransmission();
}

// Reads a register
byte FXAS21002C::readReg(byte reg)
{
	byte value;

	Wire1.beginTransmission(address);
	Wire1.write(reg);
	Wire1.endTransmission();
	Wire1.requestFrom(address, (uint8_t)1);
	value = Wire1.read();
	Wire1.endTransmission();

	return value;
}

void FXAS21002C::readRegs(byte reg, uint8_t count, byte dest[])
{
	uint8_t i = 0;

	Wire1.beginTransmission(address);   // Initialize the Tx buffer
	Wire1.write(reg);            	   // Put slave register address in Tx buffer
	Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	Wire1.requestFrom(address, count);  // Read bytes from slave register address

	while (Wire1.available()) {
		dest[i++] = Wire1.read();   // Put read results in the Rx buffer
	}
}

// Read the temperature data
void FXAS21002C::readTempData()
{
	tempData = readReg(FXAS21002C_H_TEMP);
}

// Put the FXAS21002C into standby mode.
// It must be in standby for modifying most registers
void FXAS21002C::standby()
{
	byte c = readReg(FXAS21002C_H_CTRL_REG1);
	writeReg(FXAS21002C_H_CTRL_REG1, c & ~(0x03));// Clear bits 0 and 1; standby mode
}
// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21002C::ready()
{
  byte c = readReg(FXAS21002C_H_CTRL_REG1);
  writeReg(FXAS21002C_H_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  writeReg(FXAS21002C_H_CTRL_REG1, c |   0x01);   // Set bit 0 to 1, ready mode; no data acquisition yet
}

// Put the FXAS21002C into active mode.
// Needs to be in this mode to output data.
void FXAS21002C::active()
{
	byte c = readReg(FXAS21002C_H_CTRL_REG1);
	writeReg(FXAS21002C_H_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  	writeReg(FXAS21002C_H_CTRL_REG1, c |   0x02);   // Set bit 1 to 1, active mode; data acquisition enabled
}

void FXAS21002C::init()
{

	/*
	standby();  // Must be in standby to change registers


	// Set up the full scale range to 250, 500, 1000, or 2000 deg/s.
	writeReg(FXAS21002C_H_CTRL_REG0, gyroFSR);
	 // Setup the 3 data rate bits, 4:2
	if (gyroODR < 8)
		writeReg(FXAS21002C_H_CTRL_REG1, gyroODR << 2);

	// Disable FIFO, route FIFO and rate threshold interrupts to INT2, enable data ready interrupt, route to INT1
	// Active HIGH, push-pull output driver on interrupts
	writeReg(FXAS21002C_H_CTRL_REG2, 0x0E);

	 // Set up rate threshold detection; at max rate threshold = FSR; rate threshold = THS*FSR/128
	writeReg(FXAS21002C_H_RT_CFG, 0x07);         // enable rate threshold detection on all axes
	writeReg(FXAS21002C_H_RT_THS, 0x00 | 0x0D);  // unsigned 7-bit THS, set to one-tenth FSR; set clearing debounce counter
	writeReg(FXAS21002C_H_RT_COUNT, 0x04);       // set to 4 (can set up to 255)
	// Configure interrupts 1 and 2
	//writeReg(CTRL_REG3, readReg(CTRL_REG3) & ~(0x02)); // clear bits 0, 1
	//writeReg(CTRL_REG3, readReg(CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
	//writeReg(CTRL_REG4, readReg(CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
	//writeReg(CTRL_REG4, readReg(CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled
	//writeReg(CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2
	active();  // Set to active to start reading
	*/


		// write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
	 // [7]: ZR_cond=0
	 // [6]: RST=0
	 // [5]: ST=0 self test disabled
	 // [4-2]: DR[2-0]=000 for 800Hz
	 // [1-0]: Active=0, Ready=0 for Standby mode
	 writeReg(FXAS21002_CTRL_REG1, 0x00);

  // write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
  // [7-6]: BW[1-0]=00, LPF disabled
  // [5]: SPIW=0 4 wire SPI (irrelevant)
  // [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
  // [2]: HPF_EN=0 disable HPF
  // [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
	writeReg(FXAS21002_CTRL_REG0, 0b00000001);

  // [7-4]: BW[1-0]=00, LPF disabled
  // [3]: Auto-increment read address pointer roll-over behavior:
  // [2]: External power mode control input
  // [1] disabled
  // [0]: Full-scale range expansion enable
  writeReg(FXAS21002_CTRL_REG3, 0b00000000);

  // write 0000 1010 = 0x0A to CTRL_REG1 to configure 200Hz ODR and enter Active mode
  // [7]: ZR_cond=0
  // [6]: RST=0
  // [5]: ST=0 self test disabled
  // [4-2]: DR[2-0]=010 for 200Hz ODR
  // [1-0]: Active=1, Ready=0 for Active mode
  writeReg(FXAS21002_CTRL_REG1, 0x0A);


}

// Read the gyroscope data
void FXAS21002C::readGyroData()
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readRegs(FXAS21002C_H_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	gyroData.x = ((int16_t)( rawData[0] << 8 | rawData[1])) - gBias[0];// >> 2;
	gyroData.y = ((int16_t)( rawData[2] << 8 | rawData[3])) - gBias[1];// >> 2;
	gyroData.z = ((int16_t)( rawData[4] << 8 | rawData[5])) - gBias[2];// >> 2;
}

// Get accelerometer resolution
float FXAS21002C::getGres(void)
{
	switch (gyroFSR)
	{
		// Possible gyro scales (and their register bit settings) are:
  // 250 DPS (11), 500 DPS (10), 1000 DPS (01), and 2000 DPS  (00).
    case GFS_2000DPS:
          gRes = 1600.0/8192.0;
          break;
    case GFS_1000DPS:
          gRes = 0.03125;
          break;
    case GFS_500DPS:
          gRes = 400.0/8192.0;
          break;
    case GFS_250DPS:
          gRes = 200.0/8192.0;
	}
}

void FXAS21002C::calibrate()
{
  int32_t gyro_bias[3] = {0, 0, 0};
  uint16_t ii = 0, fcount = 200;
  int16_t temp[3];

  // Clear all interrupts by reading the data output and STATUS registers
  //readGyroData(temp);
  readReg(FXAS21002C_H_STATUS);

  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
	uint32_t lastRead = micros();
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
	  readRegs(FXAS21002C_H_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
	  temp[0] = ((int16_t)( rawData[0] << 8 | rawData[1]));
	  temp[1] = ((int16_t)( rawData[2] << 8 | rawData[3]));
	  temp[2] = ((int16_t)( rawData[4] << 8 | rawData[5]));

	  gyro_bias[0] += (int32_t) temp[0];
	  gyro_bias[1] += (int32_t) temp[1];
	  gyro_bias[2] += (int32_t) temp[2];

	  delayMicroseconds(5000-(micros()-lastRead)); // wait for next data sample at 200 Hz rate
		lastRead = micros();
  }

  gBias[0] = gyro_bias[0] / (int32_t) fcount; // get average values
  gBias[1] = gyro_bias[1] / (int32_t) fcount;
  gBias[2] = gyro_bias[2] / (int32_t) fcount;

	Serial.print("Calibration complete");
	Serial.println(gBias[0]);
	Serial.println(gBias[1]);
	Serial.println(gBias[2]);

}

void FXAS21002C::reset()
{
	writeReg(FXAS21002C_H_CTRL_REG1, 0x20); // set reset bit to 1 to assert software reset to zero at end of boot process
	delay(100);
while(!(readReg(FXAS21002C_H_INT_SRC_FLAG) & 0x08))  { // wait for boot end flag to be set
}

}
// Private Methods //////////////////////////////////////////////////////////////
