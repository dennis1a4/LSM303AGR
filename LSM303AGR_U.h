/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303AGR

  These devices use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __LSM303AGR_H__
#define __LSM303AGR_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
    #define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                                     // DEFAULT    TYPE
      LSM303_REGISTER_ACCEL_STATUS_REG_AUX_A    = 0x07,   //            r   Temp overrun/new data
      LSM303_REGISTER_ACCEL_OUT_TEMP_L_A        = 0x0C,   //            r   Temp output low
      LSM303_REGISTER_ACCEL_OUT_TEMP_H_A        = 0x0D,   //            r   Temp output high
      LSM303_REGISTER_ACCEL_INT_COUNTER_REG_A   = 0x0E,   //            r
      LSM303_REGISTER_ACCEL_WHO_AM_I_A          = 0x0F,   // 00110011   r
      LSM303_REGISTER_ACCEL_TEMP_CFG_REG_A      = 0x1C,   // 00000000   rw  Temp config reg
      LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
      LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
      LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
      LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
      LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
      LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
      LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
      LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
      LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
      LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
      LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
      LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
      LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
      LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
      LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
      LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
      LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
      LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
      LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
      LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
      LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
      LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
      LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
      LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
      LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
      LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D,
      LSM303_REGISTER_ACCEL_Act_THS_A           = 0x3E,   // 00000000   rw Sleep-to-wake, return-to-sleep activation threshold in low-power mode
      LSM303_REGISTER_ACCEL_Act_DUR_A           = 0x3F    // 00000000   rw Sleep-to-wake, return-to-sleep duration
    } lsm303AccelRegisters_t;

    typedef enum
    {
      LSM303_REGISTER_MAG_OFFSET_X_REG_L_M      = 0x45,   //            rw
      LSM303_REGISTER_MAG_OFFSET_X_REG_H_M      = 0x46,   //            rw
      LSM303_REGISTER_MAG_OFFSET_Y_REG_L_M      = 0x47,   //            rw
      LSM303_REGISTER_MAG_OFFSET_Y_REG_H_M      = 0x48,   //            rw
      LSM303_REGISTER_MAG_OFFSET_Z_REG_L_M      = 0x49,   //            rw
      LSM303_REGISTER_MAG_OFFSET_Z_REG_H_M      = 0x4A,   //            rw
      LSM303_REGISTER_MAG_WHO_AM_I_M            = 0x4F,   //            r
      LSM303_REGISTER_MAG_CFG_REG_A_M           = 0x60,   // 00000011   rw
      LSM303_REGISTER_MAG_CFG_REG_B_M           = 0x61,   //            rw
      LSM303_REGISTER_MAG_CFG_REG_C_M           = 0x62,   //            rw
      LSM303_REGISTER_MAG_INT_CTRL_REG_M        = 0x63,   // 11100000   rw
      LSM303_REGISTER_MAG_INT_SOURCE_REG_M      = 0x64,   //            r
      LSM303_REGISTER_MAG_INT_THS_L_REG_M       = 0x65,   //            rw
      LSM303_REGISTER_MAG_INT_THS_H_REG_M       = 0x66,   //            rw
      LSM303_REGISTER_MAG_STATUS_REG_M          = 0x67,   //            r
      LSM303_REGISTER_MAG_OUTX_L_REG_M          = 0x68,   //            r
      LSM303_REGISTER_MAG_OUTX_H_REG_M          = 0x69,   //            r
      LSM303_REGISTER_MAG_OUTY_L_REG_M          = 0x6A,   //            r
      LSM303_REGISTER_MAG_OUTY_H_REG_M          = 0x6B,   //            r
      LSM303_REGISTER_MAG_OUTZ_L_REG_M          = 0x6C,   //            r
      LSM303_REGISTER_MAG_OUTZ_H_REG_M          = 0x6D    //            r
    } lsm303MagRegisters_t;
/*=========================================================================*/

/*=========================================================================
    TEMPERATURE CONGIF SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      LSM303_TEMP_ON                            = 0xC0,  //
      LSM303_TEMP_OFF                           = 0x00   //
    } lsm303TempCfg;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER UPDATE RATE SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      LSM303_MAGRATE_10                         = 0x00,  // 10 Hz
      LSM303_MAGRATE_20                         = 0x04,  // 20 Hz
      LSM303_MAGRATE_50                         = 0x08,  // 50 Hz
      LSM303_MAGRATE_100                        = 0x0C   // 100 Hz
    } lsm303MagRate;
/*=========================================================================*/

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct lsm303MagData_s
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } lsm303MagData;
/*=========================================================================*/

/*=========================================================================
    INTERNAL ACCELERATION DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct lsm303AccelData_s
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } lsm303AccelData;
/*=========================================================================*/

/* Unified sensor driver for the accelerometer */
class Adafruit_LSM303_Accel_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_LSM303_Accel_Unified(int32_t sensorID = -1);

    bool begin(void);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);

    lsm303AccelData raw;   // Last read accelerometer data will be available here

  private:
    int32_t         _sensorID;

    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
    void read(void);
};

/* Unified sensor driver for the magnetometer */
class Adafruit_LSM303_Mag_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_LSM303_Mag_Unified(int32_t sensorID = -1);

    bool begin(byte ctrlReg);
    void setMagRate(lsm303MagRate rate);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);
    
    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
    void read(void);

    lsm303MagData   raw;     // Last read magnetometer data will be available here

  private:
    int32_t         _sensorID;

};

/* Non Unified (old) driver for compatibility reasons */
class Adafruit_LSM303
{
  public:
    bool begin(void);
    void read(void);

    lsm303AccelData accelData;    // Last read accelerometer data will be available here
    lsm303MagData magData;        // Last read magnetometer data will be available here

    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
};

#endif
