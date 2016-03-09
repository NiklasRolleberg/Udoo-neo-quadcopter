/* FXAS21002C
 * Author: Andres Sabas <s@theinventorhouse.org>
 *
 *Basada en:
 * FXOS8700CQ
 *
 * Author: Matt Warner <mlw2224@rit.edu>
 *
 * The FXAS21002C is an gyroscope I2C sensor which is 3.3V tolerant.
 */


#ifndef FXAS21002C_H_
#define FXAS21002C_H_

#include <Arduino.h> // for byte data type

// register addresses FXAS21002C_H_
#define FXAS21002C_H_STATUS           0x00
#define FXAS21002C_H_DR_STATUS        0x07
#define FXAS21002C_H_F_STATUS         0x08
#define FXAS21002C_H_OUT_X_MSB        0x01
#define FXAS21002C_H_OUT_X_LSB        0x02
#define FXAS21002C_H_OUT_Y_MSB        0x03
#define FXAS21002C_H_OUT_Y_LSB        0x04
#define FXAS21002C_H_OUT_Z_MSB        0x05
#define FXAS21002C_H_OUT_Z_LSB        0x06
#define FXAS21002C_H_F_SETUP          0x09
#define FXAS21002C_H_F_EVENT          0x0A
#define FXAS21002C_H_INT_SRC_FLAG     0x0B
#define FXAS21002C_H_WHO_AM_I         0x0C
#define FXAS21002C_H_CTRL_REG0        0x0D
#define FXAS21002C_H_RT_CFG       	  0x0E
#define FXAS21002C_H_RT_SRC       	  0x0F
#define FXAS21002C_H_RT_THS       	  0x10
#define FXAS21002C_H_RT_COUNT         0x11
#define FXAS21002C_H_TEMP             0x12
#define FXAS21002C_H_CTRL_REG1        0x13
#define FXAS21002C_H_CTRL_REG2        0x14
#define FXAS21002C_H_CTRL_REG3        0x15

//egna
#define FXAS21002_CTRL_REG0  0x0D
#define FXAS21002_CTRL_REG3  0x15
#define FXAS21002_CTRL_REG1  0x13



enum GyroODR {
	GODR_800HZ = 0, // 200 Hz
	GODR_400HZ,
	GODR_200HZ,
	GODR_100HZ,
	GODR_50HZ,
	GODR_12_5HZ, // 12.5 Hz, etc.
	GODR_6_25HZ,
	GODR_1_56HZ
};
// Set initial input parameters
enum gyroFSR {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS
};

class FXAS21002C
{
	public:
	typedef struct
	{
	    int16_t	x;
	    int16_t	y;
	    int16_t	z;
	} SRAWDATA;

	// Sensor data
	SRAWDATA gyroData; // RAW acceleration sensor data
	int8_t tempData; // RAW temperature data

	// Sensor configuration
	uint8_t gyroFSR = GFS_1000DPS;
	uint8_t gyroODR = GODR_200HZ;
	float gRes;
	int32_t gBias[3] = {0, 0, 0}; // scale resolutions per LSB for the sensors

	FXAS21002C(byte addr);

	// Register functions
	void writeReg(byte reg, byte value);
	byte readReg(byte reg);
	void readRegs(byte startReg, uint8_t count, byte dest[]);

	// FXAS21002C functions
	// Initialization & Termination
	void init(void);
	void standby(void);
	void active(void);
	void ready(void);

	// Query sensor data
	void readGyroData(void);
	void readTempData(void);

	// Resolution
	float getGres(void);

	//Calibrate
	void calibrate();

	//Reset
	void reset(void);

	private:
	// Sensor address
	byte address;
};

#endif
