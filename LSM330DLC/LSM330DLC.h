/***************************************************************************
  This is a library for the LSM330 Accelerometer and gyroscope

  uses I2C to communicate, 2 pins are required to interface.

  Written by Daniel de Kock for Bushveldlabs.
  As is, et al, don't use in life saving ( or taking ) equipment , do not ingest etc.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef LSM330DLC_H_
#define LSM330DLC_H_

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

 //  7 bit IIC device address for the accelerometer,
//  the IIC library will shift it and amend the 8th bit for read or write operations
#define LSM330DLC_ADDRESS_ACCEL          B0011000        // 0011001x
#define LSM330DLC_ADDRESS_GYRO           B1101010        // 0011110x
#define LSM330DLC_ID                     (0b11010100)


/*
some Maccros for the Gyro and Accelerometer TODO : put them in enums so they are easier to use
*/
//ACCELEROMETER CONTROL REGISTER 1
#define ACC_REG1_LOW_POWER (0x01<<3)

#define ACC_REG1_X_ENABLE (0x01)
#define ACC_REG1_Y_ENABLE (0x01<<1)
#define ACC_REG1_Z_ENABLE (0x01<<2)

#define ACC_REG1_ODR_1HZ		(0x01<<4)
#define ACC_REG1_ODR_10HZ		(0x02<<4)
#define ACC_REG1_ODR_25HZ		(0x03<<4)
#define ACC_REG1_ODR_50HZ		(0x04<<4)
#define ACC_REG1_ODR_100HZ		(0x05<<4)
#define ACC_REG1_ODR_200HZ		(0x06<<4)
#define ACC_REG1_ODR_400HZ		(0x07<<4)
#define ACC_REG1_ODR_LOW_POWER	(0x08<<4)
#define ACC_REG1_ODR_NORMAL		(0x09<<4)

// We leave Register 2 at default
// We leave Register 3 at default
// We leave Register 4 at default ( defaults to +-2g range )
#define ACC_REG4_HIGH_RES		(0x08)
// We leave Register 5 at default ( FIFO disabled )
// We leave Register 6 at default
#define ACCEL_RANGE 4.00 // +- 2g  per axis
#define ACCEL_SENSITIVITY 0.001 // ( 0.001 g / digit )
#define GYRO_RANGE 500.00 // +- 250 dps per axis
#define GYRO_SENSITIVITY 0.00875 // ( 0.00875 dps / digit )

//GYRO CONTROL REGISTER 1
#define GYRO_REG1_POWER_ON (0x01<<3) // Gyro Power normal mode

#define GYRO_REG1_DR_95HZ (0x00<<4) // Gyro Data Rate
#define GYRO_REG1_DR_380HZ (0x02 <<6)

#define GYRO_REG1_BW_100 (0x03<<6) // Gyro Bandwidth
#define GYRO_REG1_BW_125 (0x00<<6) 

#define GYRO_REG1_X_ENABLE (0x01<<1)
#define GYRO_REG1_Y_ENABLE (0x01)
#define GYRO_REG1_Z_ENABLE (0x01<<2)
// We leave Register 2 at default
// We leave Register 3 at default
// We leave Register 4 at default ( defaults to 250dps range )
// We leave Register 5 at default ( FIFO disabled )


class LSM330DLC
{
  public:
    typedef enum
    {                                                     // DEFAULT    TYPE
		LSM330DLC_REGISTER_ACCEL_CTRL_REG1_A = 0x20,
		LSM330DLC_REGISTER_ACCEL_CTRL_REG2_A = 0x21,
		LSM330DLC_REGISTER_ACCEL_CTRL_REG3_A = 0x22,
		LSM330DLC_REGISTER_ACCEL_CTRL_REG4_A = 0x23,
		LSM330DLC_REGISTER_ACCEL_CTRL_REG5_A = 0x24,
		LSM330DLC_REGISTER_ACCEL_CTRL_REG6_A = 0x25,
		LSM330DLC_REGISTER_ACCEL_REFERENCE = 0x26,
		LSM330DLC_REGISTER_ACCEL_STATUS_REG_A = 0x27,
		LSM330DLC_REGISTER_ACCEL_OUT_X_L_A = 0x28,
		LSM330DLC_REGISTER_ACCEL_OUT_X_H_A = 0x29,
		LSM330DLC_REGISTER_ACCEL_OUT_Y_L_A = 0x2A,
		LSM330DLC_REGISTER_ACCEL_OUT_Y_H_A = 0x2B,
		LSM330DLC_REGISTER_ACCEL_OUT_Z_L_A = 0x2C,
		LSM330DLC_REGISTER_ACCEL_OUT_Z_H_A = 0x2D,
		LSM330DLC_REGISTER_ACCEL_FIFO_CTRL_REG_A = 0x2E,
		LSM330DLC_REGISTER_ACCEL_FIFO_SRC_REG_A = 0x2F,
		LSM330DLC_REGISTER_ACCEL_INT1_CFG_A = 0x30,
		LSM330DLC_REGISTER_ACCEL_INT1_SRC_A = 0x31,
		LSM330DLC_REGISTER_ACCEL_INT1_THS_A = 0x32,
		LSM330DLC_REGISTER_ACCEL_INT1_DURATION_A = 0x33,
		LSM330DLC_REGISTER_ACCEL_CLICK_CFG_A = 0x38,
		LSM330DLC_REGISTER_ACCEL_CLICK_SRC_A = 0x39,
		LSM330DLC_REGISTER_ACCEL_CLICK_THS_A = 0x3A,
		LSM330DLC_REGISTER_ACCEL_TIME_LIMIT_A = 0x3B,
		LSM330DLC_REGISTER_ACCEL_TIME_LATENCY_A = 0x3C,
		LSM330DLC_REGISTER_ACCEL_TIME_WINDOW_A = 0x3D,
		LSM330DLC_REGISTER_ACCEL_Act_THS = 0x3E,
		LSM330DLC_REGISTER_ACCEL_Act_DUR = 0x3F
    } lsm330AccelRegisters_t;

	typedef enum
    {
		LSM330DLC_REGISTER_GYRO_WHO_AM_I_G = 0x0F,
		LSM330DLC_REGISTER_GYRO_CTRL_REG1_G = 0x20,
		LSM330DLC_REGISTER_GYRO_CTRL_REG2_G = 0x21,
		LSM330DLC_REGISTER_GYRO_CTRL_REG3_G = 0x22,
		LSM330DLC_REGISTER_GYRO_CTRL_REG4_G = 0x23,
		LSM330DLC_REGISTER_GYRO_CTRL_REG5_G = 0x24,
		LSM330DLC_REGISTER_GYRO_REFERENCE_G = 0x25,
		LSM330DLC_REGISTER_GYRO_OUT_TEMP_G = 0x26,
		LSM330DLC_REGISTER_GYRO_STATUS_REG_G = 0x27,
		LSM330DLC_REGISTER_GYRO_OUT_X_L_G = 0x28,
		LSM330DLC_REGISTER_GYRO_OUT_X_H_G = 0x29,
		LSM330DLC_REGISTER_GYRO_OUT_Y_L_G = 0x2A,
		LSM330DLC_REGISTER_GYRO_OUT_Y_H_G = 0x2B,
		LSM330DLC_REGISTER_GYRO_OUT_Z_L_G = 0x2C,
		LSM330DLC_REGISTER_GYRO_OUT_Z_H_G = 0x2D,
		LSM330DLC_REGISTER_GYRO_FIFO_CTRL_REG_G = 0x2E,
		LSM330DLC_REGISTER_GYRO_FIFO_SRC_REG_G = 0x2F,
		LSM330DLC_REGISTER_GYRO_INT1_CFG_G = 0x30,
		LSM330DLC_REGISTER_GYRO_INT1_SRC_G = 0x31,
		LSM330DLC_REGISTER_GYRO_INT1_THS_XH_G = 0x32,
		LSM330DLC_REGISTER_GYRO_INT1_THS_XL_G = 0x33,
		LSM330DLC_REGISTER_GYRO_INT1_THS_YH_G = 0x34,
		LSM330DLC_REGISTER_GYRO_INT1_THS_YL_G = 0x35,
		LSM330DLC_REGISTER_GYRO_INT1_THS_ZH_G = 0x36,
		LSM330DLC_REGISTER_GYRO_INT1_THS_ZL_G = 0x37,
		LSM330DLC_REGISTER_GYRO_INT1_DURATION_G = 0x38
    } lsm330GyroRegisters_t;



    typedef struct lsm303AccelData_s
    {
      float x;
      float y;
      float z;
    } lsm330AccelData;
	
	typedef struct lsm303MagData_s
	{
      float x;
      float y;
      float z;
	} lsm330GyroData;

    bool begin(void);
    void read(void);

	lsm330AccelData accelData;    // Last read accelerometer data will be available here
	lsm330GyroData gyroData;        // Last read gyro data will be available here

	void getAccReadings(uint8_t addr);
	void getGyroReadings(uint8_t addr);

	uint8_t writeWire8(uint8_t devAddr, uint8_t registerAddress, uint8_t data, bool sendStop = true);
	uint8_t writeWire(uint8_t devAddr, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop = true);
    byte readWire(uint8_t devAddr, uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

  private:
};

#endif
