/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <LSM330DLC.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
bool LSM330DLC::begin()
{
  Wire.begin();

  // Accelerometer set to +-2g ( 0.001 g / digit ), all axes enable
  writeWire8(LSM330DLC_ADDRESS_ACCEL, (uint8_t)LSM330DLC_REGISTER_ACCEL_CTRL_REG1_A, (uint8_t)(ACC_REG1_ODR_400HZ | ACC_REG1_X_ENABLE | ACC_REG1_Y_ENABLE | ACC_REG1_Z_ENABLE), true); //  enable the device //
  writeWire8(LSM330DLC_ADDRESS_ACCEL, (uint8_t)LSM330DLC_REGISTER_ACCEL_CTRL_REG4_A, (uint8_t)(ACC_REG4_HIGH_RES), true); //  12 bit resolution in Normal mode (uint8_t)(ACC_REG4_HIGH_RES)
// Gyro set to +-250 dps ( 0.00875 dps / digit ), all axes enable
  writeWire8(LSM330DLC_ADDRESS_GYRO, (uint8_t)LSM330DLC_REGISTER_GYRO_CTRL_REG1_G, (uint8_t)(GYRO_REG1_POWER_ON | GYRO_REG1_BW_100 | GYRO_REG1_DR_380HZ | GYRO_REG1_X_ENABLE | GYRO_REG1_Y_ENABLE | GYRO_REG1_Z_ENABLE), true);

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void LSM330DLC::read()
{
  // Read the accelerometer

	getAccReadings(LSM330DLC_ADDRESS_ACCEL);
 
  
  // Read the magnetometer
	getGyroReadings(LSM330DLC_ADDRESS_GYRO);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/


/*
Retrieves Accelerometer reading ( acceleration for all axes) and saves them in the global accel struct for later use
*/
void LSM330DLC::getAccReadings(uint8_t addr)
{
	int16_t temp;
	uint8_t i2cData[6];
	readWire(addr, (uint8_t)LSM330DLC_REGISTER_ACCEL_OUT_X_L_A, i2cData, 6); // read 6 bytes from the Accelerometer : Xl XH, Yl yH,zl zH
												   // need to convert the readings to g
	temp = (i2cData[1] << 8) | i2cData[0]; // this number will be -32k to 32 k ( range of 16 bit signed int )
	accelData.x = ((((double)temp) / 32767.00)*(ACCEL_RANGE / 2));  // 1mg/digit || 0.001g per digit , 0-4096 == -2g - 2g, so 2048 counte per 2g

	temp = (i2cData[3] << 8) | i2cData[2];
	accelData.y = ((((double)temp) / 32767.00)*(ACCEL_RANGE / 2));

	temp = (i2cData[5] << 8) | i2cData[4];
	accelData.z = ((((double)temp) / 32767.00)*(ACCEL_RANGE / 2));

	// all of the acceleration values are in "g" , yay for SI units
}

/*
Retrieves Gyro reading ( roll rate ) and saves them in the global gyro struct for later use
*/
void  LSM330DLC::getGyroReadings(uint8_t addr)
{
	int16_t temp;
	uint8_t i2cData[6];
	readWire(addr, (uint8_t)LSM330DLC_REGISTER_GYRO_OUT_X_L_G, i2cData, 6); // read 6 bytes from the Accelerometer : Xl XH, Yl yH,zl zH
												   // need to convert the readings to g
	temp = (i2cData[1] << 8) | i2cData[0]; // this number will be -32k to 32 k ( range of 16 bit signed int )
	gyroData.x = (((double)temp) / 32767.00)*(GYRO_RANGE / 2);  //+-250 dps ( 500 dps ) range over signed 16 bit int , so 250dps/32767

	temp = (i2cData[3] << 8) | i2cData[2];
	gyroData.y = (((double)temp) / 32767.00)*(GYRO_RANGE / 2);

	temp = (i2cData[5] << 8) | i2cData[4];
	gyroData.z = (((double)temp) / 32767.00)*(GYRO_RANGE / 2);

	// all of the acceleration values are in "dps" , yay for SI units
}


uint8_t LSM330DLC::writeWire8(uint8_t devAddr, uint8_t registerAddress, uint8_t data, bool sendStop) {
	Wire.beginTransmission(devAddr);
	Wire.write(registerAddress);
	Wire.write(data);
	uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
													// if (rcode) {
													//  Serial.print(F("one i2cWrite failed: "));
													//  Serial.println(rcode);
													//  }
	return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t LSM330DLC::writeWire(uint8_t devAddr, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
	Wire.beginTransmission(devAddr);
	Wire.write(registerAddress);
	Wire.write(data, length);
	uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
													// if (rcode) {
													//   Serial.print(F("multi i2cWrite failed: "));
													//   Serial.println(rcode);
													// }
	return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}


uint8_t LSM330DLC::readWire(uint8_t devAddr, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
	uint32_t timeOutTimer;

	Wire.beginTransmission(devAddr);
	if (nbytes >1) {
		Wire.write(registerAddress | 0x80); // set  "multi byte read flag for the slave
	}
	else {
		Wire.write(registerAddress);
	}

	uint8_t rcode = Wire.endTransmission(false); // send request

	Wire.requestFrom(devAddr, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
	for (uint8_t i = 0; i < nbytes; i++) {
		if (Wire.available()) {
			data[i] = Wire.read();
			//   SerialUSB.print(data[i],HEX );
			//   SerialUSB.print(" ");
		}
		else {
			timeOutTimer = micros();
			while (((micros() - timeOutTimer) < 1000) && !Wire.available());
			if (Wire.available()) {
				data[i] = Wire.read();
				//    SerialUSB.print(data[i],HEX );
				//   SerialUSB.print(" ");
			}
			else {
				//    SerialUSB.println(F("i2cRead timeout"));
				return 5; // This error value is not already taken by endTransmission
			}
		}
	}
	//  SerialUSB.println("........................");
	return 0; // Success
}
