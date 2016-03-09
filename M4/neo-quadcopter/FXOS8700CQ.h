/*
 * FXOS8700CQ
 * Author: Matt Warner <mlw2224@rit.edu>
 *
 * The FXOS8700CQ is an accelerometer I2C sensor which is 3.3V tolerant.
 */
#ifndef FXOS8700CQ_H_
#define FXOS8700CQ_H_

#include <Arduino.h> // for byte data type

// register addresses
#define FXOS8700CQ_STATUS           0x00
#define FXOS8700CQ_DR_STATUS        0x00
#define FXOS8700CQ_F_STATUS         0x00
#define FXOS8700CQ_OUT_X_MSB        0x01
#define FXOS8700CQ_OUT_X_LSB        0x02
#define FXOS8700CQ_OUT_Y_MSB        0x03
#define FXOS8700CQ_OUT_Y_LSB        0x04
#define FXOS8700CQ_OUT_Z_MSB        0x05
#define FXOS8700CQ_OUT_Z_LSB        0x06
#define FXOS8700CQ_F_SETUP          0x09
#define FXOS8700CQ_TRIG_CFG         0x0A
#define FXOS8700CQ_SYSMOD           0x0B
#define FXOS8700CQ_INT_SOURCE       0x0C
#define FXOS8700CQ_WHO_AM_I         0x0D
#define FXOS8700CQ_XYZ_DATA_CFG     0x0E
#define FXOS8700CQ_HP_FILTER_CUTOFF 0x0F
#define FXOS8700CQ_PL_STATUS        0x10
#define FXOS8700CQ_PL_CFG           0x11
#define FXOS8700CQ_PL_COUNT         0x12
#define FXOS8700CQ_PL_BF_ZCOMP      0x13
#define FXOS8700CQ_P_L_THS_REG      0x14
#define FXOS8700CQ_A_FFMT_CFG       0x15
#define FXOS8700CQ_A_FFMT_SRC       0x16
#define FXOS8700CQ_A_FFMT_THS       0x17
#define FXOS8700CQ_A_FFMT_COUNT     0x18
#define FXOS8700CQ_TRANSIENT_CFG    0x1D
#define FXOS8700CQ_TRANSIENT_SRC    0x1E
#define FXOS8700CQ_TRANSIENT_THS    0x1F
#define FXOS8700CQ_TRANSIENT_COUNT  0x20
#define FXOS8700CQ_PULSE_CFG        0x21
#define FXOS8700CQ_PULSE_SRC        0x22
#define FXOS8700CQ_PULSE_THSX       0x23
#define FXOS8700CQ_PULSE_THSY       0x24
#define FXOS8700CQ_PULSE_THSZ       0x25
#define FXOS8700CQ_PULSE_TMLT       0x26
#define FXOS8700CQ_PULSE_LTCY       0x27
#define FXOS8700CQ_PULSE_WIND       0x28
#define FXOS8700CQ_ASLP_COUNT       0x29
#define FXOS8700CQ_CTRL_REG1        0x2A
#define FXOS8700CQ_CTRL_REG2        0x2B
#define FXOS8700CQ_CTRL_REG3        0x2C
#define FXOS8700CQ_CTRL_REG4        0x2D
#define FXOS8700CQ_CTRL_REG5        0x2E
#define FXOS8700CQ_OFF_X            0x2F
#define FXOS8700CQ_OFF_Y            0x30
#define FXOS8700CQ_OFF_Z            0x31
#define FXOS8700CQ_M_DR_STATUS      0x32
#define FXOS8700CQ_M_OUT_X_MSB      0x33
#define FXOS8700CQ_M_OUT_X_LSB      0x34
#define FXOS8700CQ_M_OUT_Y_MSB      0x35
#define FXOS8700CQ_M_OUT_Y_LSB      0x36
#define FXOS8700CQ_M_OUT_Z_MSB      0x37
#define FXOS8700CQ_M_OUT_Z_LSB      0x38
#define FXOS8700CQ_CMP_OUT_X_MSB    0x39
#define FXOS8700CQ_CMP_OUT_X_LSB    0x3A
#define FXOS8700CQ_CMP_OUT_Y_MSB    0x3B
#define FXOS8700CQ_CMP_OUT_Y_LSB    0x3C
#define FXOS8700CQ_CMP_OUT_Z_MSB    0x3D
#define FXOS8700CQ_CMP_OUT_Z_LSB    0x3E
#define FXOS8700CQ_M_OFF_X_MSB      0x3F
#define FXOS8700CQ_M_OFF_X_LSB      0x40
#define FXOS8700CQ_M_OFF_Y_MSB      0x41
#define FXOS8700CQ_M_OFF_Y_LSB      0x42
#define FXOS8700CQ_M_OFF_Z_MSB      0x43
#define FXOS8700CQ_M_OFF_Z_LSB      0x44
#define FXOS8700CQ_MAX_X_MSB        0x45
#define FXOS8700CQ_MAX_X_LSB        0x46
#define FXOS8700CQ_MAX_Y_MSB        0x47
#define FXOS8700CQ_MAX_Y_LSB        0x48
#define FXOS8700CQ_MAX_Z_MSB        0x49
#define FXOS8700CQ_MAX_Z_LSB        0x4A
#define FXOS8700CQ_MIN_X_MSB        0x4B
#define FXOS8700CQ_MIN_X_LSB        0x4C
#define FXOS8700CQ_MIN_Y_MSB        0x4D
#define FXOS8700CQ_MIN_Y_LSB        0x4E
#define FXOS8700CQ_MIN_Z_MSB        0x4F
#define FXOS8700CQ_MIN_Z_LSB        0x50
#define FXOS8700CQ_TEMP             0x51
#define FXOS8700CQ_M_THS_CFG        0x52
#define FXOS8700CQ_M_THS_SRC        0x53
#define FXOS8700CQ_M_THS_X_MSB      0x54
#define FXOS8700CQ_M_THS_X_LSB      0x55
#define FXOS8700CQ_M_THS_Y_MSB      0x56
#define FXOS8700CQ_M_THS_Y_LSB      0x57
#define FXOS8700CQ_M_THS_Z_MSB      0x58
#define FXOS8700CQ_M_THS_Z_LSB      0x59
#define FXOS8700CQ_M_THS_COUNT      0x5A
#define FXOS8700CQ_M_CTRL_REG1      0x5B
#define FXOS8700CQ_M_CTRL_REG2      0x5C
#define FXOS8700CQ_M_CTRL_REG3      0x5D
#define FXOS8700CQ_M_INT_SRC        0x5E
#define FXOS8700CQ_A_VECM_CFG       0x5F
#define FXOS8700CQ_A_VECM_THS_MSB   0x60
#define FXOS8700CQ_A_VECM_THS_LSB   0x61
#define FXOS8700CQ_A_VECM_CNT       0x62
#define FXOS8700CQ_A_VECM_INITX_MSB 0x63
#define FXOS8700CQ_A_VECM_INITX_LSB 0x64
#define FXOS8700CQ_A_VECM_INITY_MSB 0x65
#define FXOS8700CQ_A_VECM_INITY_LSB 0x66
#define FXOS8700CQ_A_VECM_INITZ_MSB 0x67
#define FXOS8700CQ_A_VECM_INITZ_LSB 0x68
#define FXOS8700CQ_M_VECM_CFG       0x69
#define FXOS8700CQ_M_VECM_THS_MSB   0x6A
#define FXOS8700CQ_M_VECM_THS_LSB   0x6B
#define FXOS8700CQ_M_VECM_CNT       0x6C
#define FXOS8700CQ_M_VECM_INITX_MSB 0x6D
#define FXOS8700CQ_M_VECM_INITX_LSB 0x6E
#define FXOS8700CQ_M_VECM_INITY_MSB 0x6F
#define FXOS8700CQ_M_VECM_INITY_LSB 0x70
#define FXOS8700CQ_M_VECM_INITZ_MSB 0x71
#define FXOS8700CQ_M_VECM_INITZ_LSB 0x72
#define FXOS8700CQ_A_FFMT_THS_X_MSB 0x73
#define FXOS8700CQ_A_FFMT_THS_X_LSB 0x74
#define FXOS8700CQ_A_FFMT_THS_Y_MSB 0x75
#define FXOS8700CQ_A_FFMT_THS_Y_LSB 0x76
#define FXOS8700CQ_A_FFMT_THS_Z_MSB 0x77
#define FXOS8700CQ_A_FFMT_THS_Z_LSB 0x78

//egna
#define FXOS8700CQ_A_CTRL_REG1   0x2A

#define FXOS8700CQ_A_OFF_X_REG   0x2F
#define FXOS8700CQ_A_OFF_Y_REG   0x30
#define FXOS8700CQ_A_OFF_Z_REG   0x31

#define FXOS8700CQ_M_OFF_X_MSB_REG   0x3F
#define FXOS8700CQ_M_OFF_X_LSB_REG   0x40

#define FXOS8700CQ_M_OFF_Y_MSB_REG   0x41
#define FXOS8700CQ_M_OFF_Y_LSB_REG   0x42

#define FXOS8700CQ_M_OFF_Z_MSB_REG   0x43
#define FXOS8700CQ_M_OFF_Z_LSB_REG   0x44

// Set initial input parameters
enum accelFSR {
	AFS_2g = 0,
	AFS_4g,
	AFS_8g
};

enum accelODR {
	AODR_800HZ = 0, // 200 Hz
	AODR_400HZ,
	AODR_200HZ,
	AODR_100HZ,
	AODR_50HZ,
	AODR_12_5HZ, // 12.5 Hz, etc.
	AODR_6_25HZ,
	AODR_1_56HZ
};

enum magOSR {
	MOSR_0 = 0,  // oversample ratio 2 at 50 and 200 Hz ODR
	MOSR_1,
	MOSR_2,
	MOSR_3,
	MOSR_4,
	MOSR_5,
	MOSR_6,
	MOSR_7      // oversample ratio 8 at 200 Hz ODR, 32 at 50 HZ ODR
};


class FXOS8700CQ
{
	public:
	typedef struct
	{
	    int16_t	x;
	    int16_t	y;
	    int16_t	z;
	} SRAWDATA;

	// Sensor data
	SRAWDATA accelData; // RAW acceleration sensor data
	SRAWDATA magData; // RAW magnometer sensor data
	int8_t tempData; // RAW temperature data

	// Sensor configuration
	uint8_t accelFSR;
	uint8_t accelODR;
	uint8_t magOSR;

	FXOS8700CQ(byte addr);

	// Register functions
	void writeReg(byte reg, byte value);
	byte readReg(byte reg);
	void readRegs(byte startReg, uint8_t count, byte dest[]);

	// FXOS8700CQ functions
	// Initialization & Termination
	void init(void);
	void standby(void);
	void active(void);
	void reset(void);

	// Query sensor data
	void readAccelData(void);
	void readMagData(void);
	void readTempData(void);
	bool isAccelDataRdy (void);
	bool isMagDataRdy(void);
	// Resolution
	float getAres(void);
	float getMres(void);

	private:
	// Sensor address
	byte address;
};

#endif
