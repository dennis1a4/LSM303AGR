/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the LSM303AGR

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include <limits.h>

#include "LSM303AGR_U.h"

/* enabling this #define will enable the debug print blocks
#define LSM303_DEBUG
*/

static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 4 or 16 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

/***************************************************************************
 ACCELEROMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte Adafruit_LSM303_Accel_Unified::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif
  Wire.endTransmission();

  return value;
}

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::read()
{
  // Read the accelerometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
  #if ARDUINO >= 100
    Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  #else
    Wire.send(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  #if ARDUINO >= 100
    uint8_t xlo = Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t zhi = Wire.read();
  #else
    uint8_t xlo = Wire.receive();
    uint8_t xhi = Wire.receive();
    uint8_t ylo = Wire.receive();
    uint8_t yhi = Wire.receive();
    uint8_t zlo = Wire.receive();
    uint8_t zhi = Wire.receive();
  #endif

  // Shift values to create properly formed integer (low byte first)
  raw.x = (int16_t)(xlo | (xhi << 8)) >> 4;
  raw.y = (int16_t)(ylo | (yhi << 8)) >> 4;
  raw.z = (int16_t)(zlo | (zhi << 8)) >> 4;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
Adafruit_LSM303_Accel_Unified::Adafruit_LSM303_Accel_Unified(int32_t sensorID) {
  _sensorID = sensorID;

  // Clear the raw accel data
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::begin(byte ctrlReg)
{
  // Enable I2C
  Wire.begin();

  // Enable the accelerometer (100Hz)
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, ctrlReg);

  // LSM303AGR has WHOAMI register so read it back to check
  // if we are connected or not
  uint8_t who_am_i_a = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_WHO_AM_I_A);
  if (who_am_i_a != 0x33)
  {
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = millis();
  event->acceleration.x = (float)raw.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = (float)raw.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = (float)raw.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F; // TBD
  sensor->min_value   = 0.0F; // TBD
  sensor->resolution  = 0.0F; // TBD
}

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte Adafruit_LSM303_Mag_Unified::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif
  Wire.endTransmission();

  return value;
}

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::read()
{
  // Read the magnetometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
  #if ARDUINO >= 100
    Wire.write(LSM303_REGISTER_MAG_OUTX_L_REG_M);
  #else
    Wire.send(LSM303_REGISTER_MAG_OUTX_L_REG_M);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Note high before low (different than accel)
  #if ARDUINO >= 100
    uint8_t xlo = Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t zhi = Wire.read();
  #else
    uint8_t xlo = Wire.receive();
    uint8_t xhi = Wire.receive();
    uint8_t ylo = Wire.receive();
    uint8_t yhi = Wire.receive();
    uint8_t zlo = Wire.receive();
    uint8_t zhi = Wire.receive();
    
  #endif

  // Shift values to create properly formed integer (low byte first)
  raw.x = (int16_t)(xlo | ((int16_t)xhi << 8));
  raw.y = (int16_t)(ylo | ((int16_t)yhi << 8));
  raw.z = (int16_t)(zlo | ((int16_t)zhi << 8));
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
Adafruit_LSM303_Mag_Unified::Adafruit_LSM303_Mag_Unified(int32_t sensorID) {
  _sensorID = sensorID;

  // Clear the raw mag data
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_LSM303_Mag_Unified::begin()
{
  // Enable I2C
  Wire.begin();

  // Enable the magnetometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CFG_REG_A_M, 0x00);

  // LSM303AGR has no WHOAMI register so read MAG_INT_CTRL_REG_M to check
  // the default value (0b11100000/0xE0)
  uint8_t reg1_a = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_INT_CTRL_REG_M);
  if (reg1_a != 0xE0)
  {
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's update rate
*/
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::setMagRate(lsm303MagRate rate)
{
	byte reg_m = ((byte)rate & 0x0C);
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CFG_REG_A_M, reg_m);
}


/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_LSM303_Mag_Unified::getEvent(sensors_event_t *event) {
  bool readingValid = false;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

// checks if the data ready register is set. In continuous mode this is never set.
	// defoult setup is continuous mode.
  //uint8_t reg_mg = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_SR_REG_Mg);
  //if (!(reg_mg & 0x1)) {
//	return false;
 // }

  /* Read new data */
  read();


  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = millis();
  event->magnetic.x = (float)raw.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y = (float)raw.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z = (float)raw.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

	return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F; // TBD
  sensor->min_value   = 0.0F; // TBD
  sensor->resolution  = 0.0F; // TBD
}

/* --- The code below is no longer maintained and provided solely for */
/* --- compatibility reasons! */

/***************************************************************************
 DEPRECATED (NON UNIFIED) DRIVER (Adafruit_LSM303.c/h)
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
bool Adafruit_LSM303::begin()
{
  Wire.begin();

  // Enable the accelerometer
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

  // Enable the magnetometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CFG_REG_A_M, 0x00);

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303::read()
{
  // Read the accelerometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
  Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  accelData.x = (int16_t)((uint16_t)xlo | ((uint16_t)xhi << 8)) >> 4;
  accelData.y = (int16_t)((uint16_t)ylo | ((uint16_t)yhi << 8)) >> 4;
  accelData.z = (int16_t)((uint16_t)zlo | ((uint16_t)zhi << 8)) >> 4;

  // Read the magnetometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
  Wire.write(LSM303_REGISTER_MAG_OUTX_L_REG_M);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Note high before low (different than accel)
  xlo = Wire.read();
  xhi = Wire.read();
  ylo = Wire.read();
  yhi = Wire.read();
  zlo = Wire.read();
  zhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  magData.x = (int16_t)((uint16_t)xlo | ((uint16_t)xhi << 8));
  magData.y = (int16_t)((uint16_t)ylo | ((uint16_t)yhi << 8));
  magData.z = (int16_t)((uint16_t)zlo | ((uint16_t)zhi << 8));
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte Adafruit_LSM303::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}
