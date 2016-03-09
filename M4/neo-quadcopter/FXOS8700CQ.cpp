/*
 * derived from: https://github.com/mlwarner/fxos8700cq-arduino
 *
 * modified by Francesco Ferraro 01/2016
 * francesco.ferrarogm@gmail.com
 *
*/

#include <Wire.h>
#include <math.h>

#include "FXOS8700CQ.h"

// Public Methods //////////////////////////////////////////////////////////////

FXOS8700CQ::FXOS8700CQ(byte addr)
{
	address = addr;
	accelFSR = AFS_2g;     // Set the scale below either 2, 4 or 8
	accelODR = AODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
	magOSR = MOSR_5;     // Choose magnetometer oversample rate
}

// Writes a register
void FXOS8700CQ::writeReg(byte reg, byte value)
{
	Wire1.beginTransmission(address);
	Wire1.write(reg);
	Wire1.write(value);
	Wire1.endTransmission();
}

// Reads a register
byte FXOS8700CQ::readReg(byte reg)
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

void FXOS8700CQ::readRegs(byte reg, uint8_t count, byte dest[])
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

// Read the accelerometer data
// original version is bugged !!!
// modified by fefr 150116
void FXOS8700CQ::readAccelData()
{
	uint8_t rawData[6];  // x/y/z accel register data stored here

	readRegs(FXOS8700CQ_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	// bug !!accelData.x = ( (int16_t)rawData[0] << 8 | rawData[1]) >> 2;
	accelData.x = ((int16_t)( rawData[0] << 8 | rawData[1])) >> 2;
	accelData.y = ((int16_t)( rawData[2] << 8 | rawData[3])) >> 2;
	accelData.z = ((int16_t)( rawData[4] << 8 | rawData[5])) >> 2;
}

// Read the magnometer data
// original version is bugged !!!
// modified by fefr 150116
void FXOS8700CQ::readMagData()
{
	uint8_t rawData[6];  // x/y/z accel register data stored here

	readRegs(FXOS8700CQ_M_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	/*
	magData.x = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2;
	magData.y = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
	magData.z = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
	*/
	magData.x = (int16_t)( rawData[0] << 8 | rawData[1]);//+2516.260913705584;
	magData.y = (int16_t)( rawData[2] << 8 | rawData[3]);//+613.820304568528;;
	magData.z = (int16_t)( rawData[4] << 8 | rawData[5]);//+990.813197969543;;
}

// Read the temperature data
void FXOS8700CQ::readTempData()
{
	tempData = readReg(FXOS8700CQ_TEMP);
}

// test if accelerometer data is ready
bool FXOS8700CQ::isAccelDataRdy(void)
{
	  if(readReg(FXOS8700CQ_DR_STATUS) & 0x08)
		  return (true);
	  else
		  return (false);
}

// test if magnetometer data is ready
bool FXOS8700CQ::isMagDataRdy(void)
{
	  if(readReg(FXOS8700CQ_M_DR_STATUS) & 0x08)
		  return (true);
	  else
		  return (false);
}

// Put the FXOS8700CQ into standby mode.
// It must be in standby for modifying most registers
void FXOS8700CQ::standby()
{
	byte c = readReg(FXOS8700CQ_CTRL_REG1);
	writeReg(FXOS8700CQ_CTRL_REG1, c & ~(0x01));
}

// Put the FXOS8700CQ into active mode.
// Needs to be in this mode to output data.
void FXOS8700CQ::active()
{
	byte c = readReg(FXOS8700CQ_CTRL_REG1);
	writeReg(FXOS8700CQ_CTRL_REG1, c | 0x01);
}

void FXOS8700CQ::reset(void)
{
	writeReg(FXOS8700CQ_CTRL_REG2, 0x40);
	writeReg(FXOS8700CQ_M_CTRL_REG2, 0x40);

	//printf("accelerometer reset offset registers \n");
	writeReg(FXOS8700CQ_A_CTRL_REG1, 0x00);
  writeReg(FXOS8700CQ_A_OFF_X_REG, 0x00);
	writeReg(FXOS8700CQ_A_OFF_Y_REG, 0x00);
	writeReg(FXOS8700CQ_A_OFF_Z_REG, 0x00);

	//Magnetometer
	writeReg(FXOS8700CQ_M_OFF_X_MSB_REG, 0x00);
	writeReg(FXOS8700CQ_M_OFF_X_LSB_REG, 0x00);

	writeReg(FXOS8700CQ_M_OFF_Y_MSB_REG, 0x00);
	writeReg(FXOS8700CQ_M_OFF_Y_LSB_REG, 0x00);

	writeReg(FXOS8700CQ_M_OFF_Z_MSB_REG, 0x00);
	writeReg(FXOS8700CQ_M_OFF_Z_LSB_REG, 0x00);

	writeReg(FXOS8700CQ_A_CTRL_REG1, 0x0D);
}

#ifdef ORIGINAL
void FXOS8700CQ::init()
{
	standby();  // Must be in standby to change registers

	// Configure the accelerometer
	writeReg(FXOS8700CQ_XYZ_DATA_CFG, accelFSR);  // Choose the full scale range to 2, 4, or 8 g.
	//writeReg(FXOS8700CQ_CTRL_REG1, readReg(FXOS8700CQ_CTRL_REG1) & ~(0x38)); // Clear the 3 data rate bits 5:3
	if (accelODR <= 7)
		writeReg(FXOS8700CQ_CTRL_REG1, readReg(FXOS8700CQ_CTRL_REG1) | (accelODR << 3));
	//writeReg(FXOS8700CQ_CTRL_REG2, readReg(FXOS8700CQ_CTRL_REG2) & ~(0x03)); // clear bits 0 and 1
	//writeReg(FXOS8700CQ_CTRL_REG2, readReg(FXOS8700CQ_CTRL_REG2) |  (0x02)); // select normal(00) or high resolution (10) mode

	// Configure the magnetometer
	writeReg(FXOS8700CQ_M_CTRL_REG1, 0x80 | magOSR << 2 | 0x03); // Set auto-calibration, set oversampling, enable hybrid mode

	// Configure interrupts 1 and 2
	//writeReg(CTRL_REG3, readReg(CTRL_REG3) & ~(0x02)); // clear bits 0, 1
	//writeReg(CTRL_REG3, readReg(CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
	//writeReg(CTRL_REG4, readReg(CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
	//writeReg(CTRL_REG4, readReg(CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled
	//writeReg(CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

	active();  // Set to active to start reading
}
#else
void FXOS8700CQ::init()
{
	reset();
	delayMicroseconds(1);

	standby();  // Must be in standby to change registers

	writeReg(FXOS8700CQ_XYZ_DATA_CFG, 0x00);  // Choose the full scale range to 2, 4, or 8 g.

	// write 0001 1111 = 0x1F to magnetometer control register 1
	// [7]: m_acal=1: auto calibration enabled
	// [6]: m_rst=0: no one-shot magnetic reset
	// [5]: m_ost=0: no one-shot magnetic measurement
	// [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
	// [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
	writeReg(FXOS8700CQ_M_CTRL_REG1, 0x1F);


	// write 0010 0000 = 0x20 to magnetometer control register 2
	// [7]: reserved
	// [6]: reserved
	// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
	// accelerometer registers
	// [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
	// [3]: m_maxmin_dis_ths=0
	// [2]: m_maxmin_rst=0
	// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
	//writeReg(FXOS8700CQ_M_CTRL_REG2, 0x20);

	//writeReg(FXOS8700CQ_CTRL_REG2, 0x02); // High Resolution mode

	//writeReg(FXOS8700CQ_CTRL_REG3, 0x01); // Open-drain, active low interrupt

	// write 0000 1100 = 0x0C to accelerometer control register 1
	// [7-6]: aslp_rate=00
	// [5-3]: dr=001 for 200Hz data rate (when in hybrid mode)
	// [2]: lnoise=1 for low noise mode
	// [1]: f_read=0 for normal 16 bit reads
	// [0]: active=0
	writeReg(FXOS8700CQ_CTRL_REG1, 0x0C);
//writeReg(FXOS8700CQ_CTRL_REG1, 0x34);

	active();  // Set to active to start reading
}
#endif

// Get accelerometer resolution
float FXOS8700CQ::getAres(void)
{
	switch (accelFSR)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 gs (00), 4 gs (01), 8 gs (10).
		case AFS_2g:
			return 2.0/8192.0;
		break;
		case AFS_4g:
			return 4.0/8192.0;
		break;
		case AFS_8g:
			return 8.0/8192.0;
		break;
	}

	return 0.0;
}

// Get magnometer resolution
float FXOS8700CQ::getMres(void)
{
	return 10./32768.;
}
// Private Methods //////////////////////////////////////////////////////////////
