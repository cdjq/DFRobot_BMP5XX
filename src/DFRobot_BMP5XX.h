/**
 * @file  DFRobot_BMP5XX.cpp
 * @brief  Define the infrastructure of DFRobot_BMP5XX class
 * @n      This is a pressure and temperature sensor that can be controlled through I2C/SPI/UART ports.
 * @n      BMP (581/585) has functions such as temperature compensation, data oversampling, IIR filtering, etc
 * @n      These features improve the accuracy of data collected by BMP (581/585) sensors.
 * @n      BMP (581/585) also has a FIFO data buffer, greatly improving its availability.
 * @n      Similarly, BMP (581/585) has an interrupt pin that can be used in an energy-efficient manner without using software algorithms.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version     V1.0.0
 * @date        2025-06-06
 * @url         https://github.com/DFRobot/DFRobot_BMP5XX
 */
#ifndef __DFRobot_BMP5XX_H
#define __DFRobot_BMP5XX_H
#include "Arduino.h"
#include "DFRobot_RTU.h"
#include "Wire.h"
#include "stdint.h"
#include <SPI.h>

#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...)                                                               \
  {                                                                            \
    Serial.print("[");                                                         \
    Serial.print(__FUNCTION__);                                                \
    Serial.print("(): ");                                                      \
    Serial.print(__LINE__);                                                    \
    Serial.print(" ] ");                                                       \
    Serial.println(__VA_ARGS__);                                               \
  }
#else
#define DBG(...)
#endif

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

class DFRobot_BMP5XX {
private:
#define BMP5_REG_BIT_MASK(POS, WIDTH) (((1 << (WIDTH)) - 1) << (POS))
#define BMP5_REG_SET_BITS(reg, prefix, val)                                         \
  ((reg) = ((reg) & ~BMP5_REG_BIT_MASK(prefix##_POS, prefix##_WIDTH)) |             \
           ((val) << (prefix##_POS)))
#define BMP5_REG_GET_BITS(reg, prefix, type)                                        \
  static_cast<type>((((reg) >> (prefix##_POS)) & ((1 << (prefix##_WIDTH)) - 1)))
#define REG_DATA_LEN_MAX 0x04
  /** 寄存器 */
#define REG_I_CHIP_ID 0x01
#define REG_I_REV_ID 0x02
#define REG_I_CHIP_STATUS 0x11
#define REG_H_DRIVE_CONFIG 0x13
#define REG_H_INT_CONFIG 0x14
#define REG_H_INT_SOURCE 0x15
#define REG_H_FIFO_CONFIG 0x16
#define REG_I_FIFO_COUNT 0x17
#define REG_H_FIFO_SEL 0x18
#define REG_I_TEMP_DATA_XLSB 0x1D
#define REG_I_TEMP_DATA_LSB 0x1E
#define REG_I_TEMP_DATA_MSB 0x1F
#define REG_I_PRESS_DATA_XLSB 0x20
#define REG_I_PRESS_DATA_LSB 0x21
#define REG_I_PRESS_DATA_MSB 0x22
#define REG_I_INT_STATUS 0x27
#define REG_I_STATUS 0x28
#define REG_I_FIFO_DATA 0x29
#define REG_H_NVM_ADDR 0x2B
#define REG_H_NVM_DATA_LSB 0x2C
#define REG_H_NVM_DATA_MSB 0x2D
#define REG_H_DSP_CONFIG 0x30
#define REG_H_DSP_IIR 0x31
#define REG_H_OOR_THR_P_LSB 0x32
#define REG_H_OOR_THR_P_MSB 0x33
#define REG_H_OOR_RANGE 0x34
#define REG_H_OOR_CONFIG 0x35
#define REG_H_OSR_CONFIG 0x36
#define REG_H_ODR_CONFIG 0x37
#define REG_I_OSR_EFF 0x38
#define REG_H_CMD 0x7E

/** 使能 */
#define ENABLE 0x01
#define DISABLE 0x00

#define DEEP_ENABLE 0x00
#define DEEP_DISABLE 0x01

#define BMP581_CHIP_ID 0x50
#define BMP585_CHIP_ID 0x51

/*! @name Soft reset command */
#define SOFT_RESET_CMD UINT8_C(0xB6)

  /** 位位置和宽度宏定义 */
  // REG_INT_CONFIG (0x14)
#define INT_MODE_POS 0
#define INT_MODE_WIDTH 1
#define INT_POL_POS 1
#define INT_POL_WIDTH 1
#define INT_OD_POS 2
#define INT_OD_WIDTH 1
#define INT_EN_POS 3
#define INT_EN_WIDTH 1
#define PAD_INT_DRV_POS 4
#define PAD_INT_DRV_WIDTH 4

// REG_INT_SRC_SEL (0x15)
#define DRDY_DATA_REG_EN_POS 0
#define DRDY_DATA_REG_EN_WIDTH 1
#define FIFO_FULL_EN_POS 1
#define FIFO_FULL_EN_WIDTH 1
#define FIFO_THS_EN_POS 2
#define FIFO_THS_EN_WIDTH 1
#define OOR_P_EN_POS 3
#define OOR_P_EN_WIDTH 1

// REG_FIFO_CONFIG (0x16)
#define FIFO_THRESHOLD_POS 0
#define FIFO_THRESHOLD_WIDTH 5
#define FIFO_MODE_POS 5
#define FIFO_MODE_WIDTH 1

// REG_FIFO_COUNT (0x17)
#define FIFO_COUNT_POS 0
#define FIFO_COUNT_WIDTH 6

// REG_FIFO_SEL_CONFIG (0x18)
#define FIFO_FRAME_SEL_POS 0
#define FIFO_FRAME_SEL_WIDTH 2
#define FIFO_DEC_SEL_POS 2
#define FIFO_DEC_SEL_WIDTH 3

// REG_NVM_ADDRESS (0x2B)
#define NVM_ROW_ADDRESS_POS 0
#define NVM_ROW_ADDRESS_WIDTH 6
#define NVM_PROG_EN_POS 6
#define NVM_PROG_EN_WIDTH 1

// REG_DSP_CONFIG (0x30)
#define IIR_FLUSH_FORCED_EN_POS 2
#define IIR_FLUSH_FORCED_EN_WIDTH 1
#define SHDW_SEL_IIR_T_POS 3
#define SHDW_SEL_IIR_T_WIDTH 1
#define FIFO_SEL_IIR_T_POS 4
#define FIFO_SEL_IIR_T_WIDTH 1
#define SHDW_SEL_IIR_P_POS 5
#define SHDW_SEL_IIR_P_WIDTH 1
#define FIFO_SEL_IIR_P_POS 6
#define FIFO_SEL_IIR_P_WIDTH 1
#define OOR_SEL_IIR_P_POS 7
#define OOR_SEL_IIR_P_WIDTH 1

// REG_DSP_IIR_CONFIG (0x31)
#define SET_IIR_T_POS 0
#define SET_IIR_T_WIDTH 3
#define SET_IIR_P_POS 3
#define SET_IIR_P_WIDTH 3

// REG_OOR_CONFIG (0x35)
#define OOR_THR_P_16_POS 0
#define OOR_THR_P_16_WIDTH 1
#define CNT_LIM_POS 6
#define CNT_LIM_WIDTH 2

// REG_OSR_CONFIG (0x36)
#define OSR_T_POS 0
#define OSR_T_WIDTH 3
#define OSR_P_POS 3
#define OSR_P_WIDTH 3
#define PRESS_EN_POS 6
#define PRESS_EN_WIDTH 1

// REG_ODR_CONFIG (0x37)
#define PWR_MODE_POS 0
#define PWR_MODE_WIDTH 2
#define ODR_POS 2
#define ODR_WIDTH 5
#define DEEP_DIS_POS 7
#define DEEP_DIS_WIDTH 1

// REG_EFF_OSR_CONFIG (0x38)
#define OSR_T_EFF_POS 0
#define OSR_T_EFF_WIDTH 3
#define OSR_P_EFF_POS 3
#define OSR_P_EFF_WIDTH 3
#define ODR_IS_VALID_POS 7
#define ODR_IS_VALID_WIDTH 1

#define STANDARD_SEA_LEVEL_PRESSURE_PA  101325   ///< Standard sea level pressure, unit: pa

  typedef struct {
    uint8_t osr_t_eff;
    uint8_t osr_p_eff;
    uint8_t odr_is_valid;
  } sOSR_ODR_EFF_t;

public:
#define RET_CODE_OK 0
#define RET_CODE_ERROR 1

  DFRobot_BMP5XX(/* args */);
  ~DFRobot_BMP5XX();

  typedef enum {
    eDISABLE,
    eENABLE,
  } eEnable_t;

  typedef enum {
    eDEEP_ENABLE,
    eDEEP_DISABLE,
  } eDeepEnable_t;

  typedef enum {
    eODR_240_HZ = 0x00U,
    eODR_218_5_HZ,
    eODR_199_1_HZ,
    eODR_179_2_HZ,
    eODR_160_HZ,
    eODR_149_3_HZ,
    eODR_140_HZ,
    eODR_129_8_HZ,
    eODR_120_HZ,
    eODR_110_1_HZ,
    eODR_100_2_HZ,
    eODR_89_6_HZ,
    eODR_80_HZ,
    eODR_70_HZ,
    eODR_60_HZ,
    eODR_50_HZ,
    eODR_45_HZ,
    eODR_40_HZ,
    eODR_35_HZ,
    eODR_30_HZ,
    eODR_25_HZ,
    eODR_20_HZ,
    eODR_15_HZ,
    eODR_10_HZ,
    eODR_05_HZ,
    eODR_04_HZ,
    eODR_03_HZ,
    eODR_02_HZ,
    eODR_01_HZ,
    eODR_0_5_HZ,
    eODR_0_250_HZ,
    eODR_0_125_HZ,
  } eODR_t;

  typedef enum {
    eOVERSAMPLING_1X = 0x00U,
    eOVERSAMPLING_2X,
    eOVERSAMPLING_4X,
    eOVERSAMPLING_8X,
    eOVERSAMPLING_16X,
    eOVERSAMPLING_32X,
    eOVERSAMPLING_64X,
    eOVERSAMPLING_128X,
  } eOverSampling_t;

  typedef enum {
    eIIR_FILTER_BYPASS = 0x00U,
    eIIR_FILTER_COEFF_1,
    eIIR_FILTER_COEFF_3,
    eIIR_FILTER_COEFF_7,
    eIIR_FILTER_COEFF_15,
    eIIR_FILTER_COEFF_31,
    eIIR_FILTER_COEFF_63,
    eIIR_FILTER_COEFF_127,
  } eIIRFilter_t;

  typedef enum {
    eOOR_COUNT_LIMIT_1 = 0x00U,
    eOOR_COUNT_LIMIT_3,
    eOOR_COUNT_LIMIT_7,
    eOOR_COUNT_LIMIT_15,
  } eOORCountLimit_t;

  typedef enum {
    ePOWERMODE_STANDBY,
    ePOWERMODE_NORMAL,
    ePOWERMODE_FORCED,
    ePOWERMODE_CONTINOUS,
    ePOWERMODE_DEEP_STANDBY,
  } ePowerMode_t;

  typedef enum {
    /*! Fifo disabled */
    eFIFO_NOT_ENABLED,
    /*! Fifo temperature data only enabled */
    eFIFO_TEMPERATURE_DATA,
    /*! Fifo pressure data only enabled */
    eFIFO_PRESSURE_DATA,
    /*! Fifo pressure and temperature data enabled */
    eFIFO_PRESS_TEMP_DATA
  } eFIFOFrameSel_t;

  /** 2^fifo_dec_sel */
  typedef enum {
    eFIFO_NO_DOWNSAMPLING,
    eFIFO_DOWNSAMPLING_2X,
    eFIFO_DOWNSAMPLING_4X,
    eFIFO_DOWNSAMPLING_8X,
    eFIFO_DOWNSAMPLING_16X,
    eFIFO_DOWNSAMPLING_32X,
    eFIFO_DOWNSAMPLING_64X,
    eFIFO_DOWNSAMPLING_128X,
  } eFIFODecSel_t;

  typedef enum {
    eFIFO_STREAM_TO_FIFO_MODE,
    eFIFO_STOP_ON_FULL_MODE
  } eFIFOMode_t;

  typedef enum {
    eINT_STATUS_DRDY = 0x01,
    eINT_STATUS_FIFO_FULL = 0x02,
    eINT_STATUS_FIFO_THRES = 0x04,
    eINT_STATUS_PRESSURE_OOR = 0x08,
    eINT_STATUS_POR_SOFTRESET_COMPLETE = 0x10,
  } eIntStatus_t;

  typedef enum { eINT_MODE_PULSED = 0x00, eINT_MODE_LATCHED = 0x01 } eIntMode_t;

  typedef enum {
    eINT_POL_ACTIVE_LOW = 0x00,
    eINT_POL_ACTIVE_HIGH = 0x01
  } eIntPolarity_t;

  typedef enum {
    eINT_OD_PUSH_PULL = 0x00,
    eINT_OD_OPEN_DRAIN = 0x01
  } eIntOpenDrain_t;

  typedef enum { eINT_DISABLED = 0x00, eINT_ENABLED = 0x01 } eIntEnable_t;

  typedef enum {
    eINT_DATA_DRDY = 0x01,
    eINT_FIFO_FULL = 0x02,
    eINT_FIFO_THRES = 0x04,
    eINT_PRESSURE_OOR = 0x08,
  } eIntSource_t;

  typedef struct {
    uint8_t len;
    float pressure[32];
    float temperature[32];
  } sFIFOData_t;
  /**
   * @fn begin
   * @brief Initializes the sensor hardware interface
   * @return true if initialization succeeds, false on failure
   */
  bool begin(void);

  /**
   * @fn setODR
   * @brief Configures sensor output data rate (ODR)
   * @param odr Output data rate selection (see: eODR_t)
   * @n Available rates:
   * @n - eODR_240_HZ:    240 Hz
   * @n - eODR_218_5_HZ:  218.5 Hz
   * @n - eODR_199_1_HZ:  199.1 Hz
   * @n - eODR_179_2_HZ:  179.2 Hz
   * @n - eODR_160_HZ:    160 Hz
   * @n - eODR_149_3_HZ:  149.3 Hz
   * @n - eODR_140_HZ:    140 Hz
   * @n - eODR_129_8_HZ:  129.8 Hz
   * @n - eODR_120_HZ:    120 Hz
   * @n - eODR_110_1_HZ:  110.1 Hz
   * @n - eODR_100_2_HZ:  100.2 Hz
   * @n - eODR_89_6_HZ:   89.6 Hz
   * @n - eODR_80_HZ:     80 Hz
   * @n - eODR_70_HZ:     70 Hz
   * @n - eODR_60_HZ:     60 Hz
   * @n - eODR_50_HZ:     50 Hz
   * @n - eODR_45_HZ:     45 Hz
   * @n - eODR_40_HZ:     40 Hz
   * @n - eODR_35_HZ:     35 Hz
   * @n - eODR_30_HZ:     30 Hz
   * @n - eODR_25_HZ:     25 Hz
   * @n - eODR_20_HZ:     20 Hz
   * @n - eODR_15_HZ:     15 Hz
   * @n - eODR_10_HZ:     10 Hz
   * @n - eODR_05_HZ:     5 Hz
   * @n - eODR_04_HZ:     4 Hz
   * @n - eODR_03_HZ:     3 Hz
   * @n - eODR_02_HZ:     2 Hz
   * @n - eODR_01_HZ:     1 Hz
   * @n - eODR_0_5_HZ:    0.5 Hz
   * @n - eODR_0_250_HZ:  0.250 Hz
   * @n - eODR_0_125_HZ:  0.125 Hz
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setODR(eODR_t odr);

  /**
   * @fn setOSR
   * @brief Sets oversampling ratios for temperature and pressure
   * @param osr_t Temperature oversampling (see: eOverSampling_t)
   * @param osr_p Pressure oversampling (see: eOverSampling_t)
   * @n Supported values:
   * @n - eOVERSAMPLING_1X:   1x oversampling
   * @n - eOVERSAMPLING_2X:   2x oversampling
   * @n - eOVERSAMPLING_4X:   4x oversampling
   * @n - eOVERSAMPLING_8X:   8x oversampling
   * @n - eOVERSAMPLING_16X:  16x oversampling
   * @n - eOVERSAMPLING_32X:  32x oversampling
   * @n - eOVERSAMPLING_64X:  64x oversampling
   * @n - eOVERSAMPLING_128X: 128x oversampling
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setOSR(eOverSampling_t osr_t, eOverSampling_t osr_p);

  /**
   * @fn setMeasureMode
   * @brief Configures sensor power/measurement mode
   * @param mode Operation mode (see: ePowerMode_t)
   * @n Available modes:
   * @n - ePOWERMODE_STANDBY:       Standby mode
   * @n - ePOWERMODE_NORMAL:        Normal measurement mode
   * @n - ePOWERMODE_FORCED:        Single-shot measurement
   * @n - ePOWERMODE_CONTINOUS:     Continuous measurement
   * @n - ePOWERMODE_DEEP_STANDBY:  Deep standby mode
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setMeasureMode(ePowerMode_t mode);

  /**
   * @fn reset
   * @brief Performs software reset of the sensor
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t reset(void);

  /**
   * @fn getTemperature
   * @brief Reads calibrated temperature data
   * @return float Temperature in degrees Celsius
   */
  float getTemperature(void);

  /**
   * @fn getPressure
   * @brief Reads calibrated pressure data
   * @return float Pressure in Pascals (Pa)
   */
  float getPressure(void);

  /**
   * @fn getAltitude
   * @brief Calculates altitude based on pressure reading
   * @note Uses formula:
   * @n altitude = (1 - (P/101325)^0.190284) * 44307.7
   * @n where P = current pressure in Pa
   * @return float Altitude in meters
   */
  float getAltitude(void);

  /**
   * @fn configIIR
   * @brief Configures IIR filter coefficients
   * @param iir_t Temperature IIR filter (see: eIIRFilter_t)
   * @param iir_p Pressure IIR filter (see: eIIRFilter_t)
   * @n Available coefficients:
   * @n - eIIR_FILTER_BYPASS:   Bypass filter
   * @n - eIIR_FILTER_COEFF_1:  1st order filter
   * @n - eIIR_FILTER_COEFF_3:  3rd order filter
   * @n - eIIR_FILTER_COEFF_7:  7th order filter
   * @n - eIIR_FILTER_COEFF_15: 15th order filter
   * @n - eIIR_FILTER_COEFF_31: 31st order filter
   * @n - eIIR_FILTER_COEFF_63: 63rd order filter
   * @n - eIIR_FILTER_COEFF_127:127th order filter
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t configIIR(eIIRFilter_t iir_t, eIIRFilter_t iir_p);

  /**
   * @fn configFIFO
   * @brief Configures FIFO operation parameters
   * @param frame_sel Data frame type (see: eFIFOFrameSel_t)
   * @n Available types:
   * @n - eFIFO_NOT_ENABLED:    FIFO disabled
   * @n - eFIFO_TEMPERATURE_DATA: Temperature data only
   * @n - eFIFO_PRESSURE_DATA:    Pressure data only
   * @n - eFIFO_PRESS_TEMP_DATA:  Pressure and temperature data
   *
   * @param dec_sel Downsampling ratio (see: eFIFODecSel_t)
   * @n Available ratios:
   * @n - eFIFO_NO_DOWNSAMPLING: No downsampling
   * @n - eFIFO_DOWNSAMPLING_2X:  2x downsampling
   * @n - eFIFO_DOWNSAMPLING_4X:  4x downsampling
   * @n - eFIFO_DOWNSAMPLING_8X:  8x downsampling
   * @n - eFIFO_DOWNSAMPLING_16X: 16x downsampling
   * @n - eFIFO_DOWNSAMPLING_32X: 32x downsampling
   * @n - eFIFO_DOWNSAMPLING_64X: 64x downsampling
   * @n - eFIFO_DOWNSAMPLING_128X:128x downsampling
   *
   * @param mode FIFO operation mode (see: eFIFOMode_t)
   * @n Available modes:
   * @n - eFIFO_STREAM_TO_FIFO_MODE: Stream data continuously
   * @n - eFIFO_STOP_ON_FULL_MODE:   Stop when FIFO full
   *
   * @param threshold FIFO trigger threshold (0=disable, 1-31=frames)]
   * @n - 0x0F: 15 frames. This is the maximum setting in PT-mode. The most
   * significant bit is ignored.
   * @n - 0x1F: 31 frames. This is the maximum setting in P- or T-mode.
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t configFIFO(eFIFOFrameSel_t frame_sel, eFIFODecSel_t dec_sel,
                     eFIFOMode_t mode, uint8_t threshold);

  /**
   * @fn getFIFOCount
   * @brief Gets current number of frames in FIFO
   * @return uint8_t Number of stored data frames (0-31)
   */
  uint8_t getFIFOCount(void);
  
  /**
   * @fn getFIFOData
   * @brief Reads all data from FIFO
   * @return sFIFOData_t Struct containing pressure and temperature data
   * @n - len: Number of stored data frames (0-31)
   * @n - pressure: Array of pressure values
   * @n - temperature: Array of temperature values
   */
  sFIFOData_t getFIFOData(void);

  /**
   * @fn configInterrupt
   * @brief Configures interrupt behavior
   * @param int_mode Trigger mode (see: eIntMode_t)
   * @n Available modes:
   * @n - eINT_MODE_PULSED: Pulsed mode
   * @n - eINT_MODE_LATCHED: Latched mode
   *
   * @param int_pol Signal polarity (see: eIntPolarity_t)
   * @n Available polarities:
   * @n - eINT_POL_ACTIVE_LOW: Active low
   * @n - eINT_POL_ACTIVE_HIGH: Active high
   *
   * @param int_od Output driver type (see: eIntOpenDrain_t)
   * @n Available types:
   * @n - eINT_OD_PUSH_PULL: Push-pull output
   * @n - eINT_OD_OPEN_DRAIN: Open-drain output
   *
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t configInterrupt(eIntMode_t int_mode, eIntPolarity_t int_pol,
                          eIntOpenDrain_t int_od);

  /**
   * @fn setIntSource
   * @brief Enables specific interrupt sources
   * @param source Bitmask of triggers
   * @n Available sources:
   * @n - eINT_DATA_DRDY:    Data ready interrupt
   * @n - eINT_FIFO_FULL:    FIFO full interrupt
   * @n - eINT_FIFO_THRES:   FIFO threshold interrupt
   * @n - eINT_PRESSURE_OOR: Pressure out-of-range interrupt
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setIntSource(uint8_t source);

  /**
   * @fn getIntStatus
   * @brief Reads current interrupt status flags
   * @return uint16_t Bitmask of active interrupts
   * @n Possible flags:
   * @n - eINT_STATUS_DRDY: Data ready
   * @n - eINT_STATUS_FIFO_FULL: FIFO full
   * @n - eINT_STATUS_FIFO_THRES: FIFO threshold reached
   * @n - eINT_STATUS_PRESSURE_OOR: Pressure out-of-range
   * @n - eINT_STATUS_POR_SOFTRESET_COMPLETE: Reset complete
   */
  uint16_t getIntStatus(void);

  /**
   * @fn setOORPress
   * @brief Configures pressure out-of-range detection
   * @param oor Threshold pressure value (0x00000-0x1FFFF)
   * @param range Hysteresis range (0-255)
   * @n oor - range < Pressure < oor + range
   * @param cnt_lim Trigger persistence count
   * @n Available persistence settings:
   * @n - eOOR_COUNT_LIMIT_1:  1 count
   * @n - eOOR_COUNT_LIMIT_3:  3 counts
   * @n - eOOR_COUNT_LIMIT_7:  7 counts
   * @n - eOOR_COUNT_LIMIT_15: 15 counts
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setOORPress(uint32_t oor, uint8_t range, eOORCountLimit_t cnt_lim);

  /**
   * @fn calibratedAbsoluteDifference
   * @brief use the given current altitude as a reference value, eliminate the absolute difference of subsequent pressure and altitude data
   * @param altitude current altitude
   * @return boolean, indicates whether the reference value is set successfully
   * @retval True indicates the reference value is set successfully
   * @retval False indicates fail to set the reference value
   */
  bool calibratedAbsoluteDifference(float altitude);

private:
  /**
   * @fn enablePressure
   * @brief Enables/disables pressure measurement
   * @param enable Enable control (see ::eEnable_t)
   * @n Available options:
   * @n - eENABLE:  Enable pressure measurement
   * @n - eDISABLE: Disable pressure measurement
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t enablePressure(eEnable_t enable);

  /**
   * @fn setDeepStandbyMode
   * @brief Configures deep standby mode
   * @param deepMode Deep standby control
   * @n Valid parameters:
   * @n - eDEEP_ENABLE: Enter deep standby mode
   * @n - eDEEP_DISABLE: Exit deep standby mode
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setDeepStandbyMode(eDeepEnable_t deepMode);

  /**
   * @fn setPowerMode
   * @brief Sets sensor power operation mode
   * @param mode Power mode (see ::ePowerMode_t)
   * @n Available modes:
   * @n - ePOWERMODE_STANDBY:       Standby mode (low power)
   * @n - ePOWERMODE_NORMAL:        Normal measurement mode
   * @n - ePOWERMODE_FORCED:        Single-shot measurement
   * @n - ePOWERMODE_CONTINOUS:     Continuous measurement
   * @n - ePOWERMODE_DEEP_STANDBY:  Deep standby (lowest power)
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setPowerMode(ePowerMode_t mode);

  /**
   * @fn getPowerMode
   * @brief Reads current power mode
   * @return ePowerMode_t Current power mode (see ::ePowerMode_t)
   * @n Possible return values:
   * @n - ePOWERMODE_STANDBY
   * @n - ePOWERMODE_NORMAL
   * @n - ePOWERMODE_FORCED
   * @n - ePOWERMODE_CONTINOUS
   * @n - ePOWERMODE_DEEP_STANDBY
   */
  ePowerMode_t getPowerMode(void);

  /**
   * @fn verifyDeepStandbyMode
   * @brief Verifies if sensor is in deep standby
   * @return ePowerMode_t Current mode with deep standby status
   * @n Special return case:
   * @n - Returns ePOWERMODE_DEEP_STANDBY if sensor is unresponsive
   */
  ePowerMode_t verifyDeepStandbyMode(void);

  /**
   * @fn setFIFOThreshold
   * @brief Configures FIFO threshold with data pattern
   * @param data Pointer to threshold data pattern (16-bit array)
   * @param frame_sel Data frame type (see ::eFIFOFrameSel_t)
   * @n Supported types:
   * @n - eFIFO_NOT_ENABLED
   * @n - eFIFO_TEMPERATURE_DATA
   * @n - eFIFO_PRESSURE_DATA
   * @n - eFIFO_PRESS_TEMP_DATA
   * @param threshold Trigger threshold (0-31 frames)
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setFIFOThreshold(uint16_t *data, eFIFOFrameSel_t frame_sel,
                           uint8_t threshold);

  /**
   * @fn calculateAltitude
   * @brief Calculates altitude with custom reference parameters
   * @param temperature_c Current temperature in Celsius
   * @param pressure_pa Current pressure in Pascals
   * @note Uses international barometric formula:
   * @n altitude = 44330 * [1 - (P/P0)^(1/5.255)]
   * @n where P0 = 101325 Pa (standard sea-level pressure)
   * @return float Calculated altitude in meters
   */
  float calculateAltitude(float temperature_c, float pressure_pa);
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len) = 0;
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len) = 0;
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len) = 0;

  float _sealevelAltitude = 0.0f;
  bool _calibrated = false;
};

class DFRobot_BMP5XX_I2C : public DFRobot_BMP5XX {
private:
  TwoWire *_pWire;
  uint16_t _i2cAddr;
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len);
  bool readReg(uint8_t reg, void *data, uint8_t len);
  bool writeReg(uint8_t reg, void *data, uint8_t len);

public:
  DFRobot_BMP5XX_I2C(TwoWire *pWire, uint16_t addr);
  ~DFRobot_BMP5XX_I2C(void);
  bool begin(void);
};

class DFRobot_BMP5XX_SPI : public DFRobot_BMP5XX {
private:
  SPIClass *_pSpi;
  uint8_t _csPin;
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len);
  bool readReg(uint8_t reg, void *data, uint8_t len);
  bool writeReg(uint8_t reg, void *data, uint8_t len);

public:
  DFRobot_BMP5XX_SPI(SPIClass *pSpi, uint8_t csPin);
  ~DFRobot_BMP5XX_SPI(void);
  bool begin(void);
};

class DFRobot_BMP5XX_UART : public DFRobot_BMP5XX, public DFRobot_RTU {
private:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  SoftwareSerial *_serial;
#else
  HardwareSerial *_serial;
#endif
  uint32_t __baud;
  uint8_t __rxpin;
  uint8_t __txpin;
  uint16_t __addr;
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len);

public:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  DFRobot_BMP5XX_UART(SoftwareSerial *sSerial, uint32_t Baud, uint16_t addr);
#else
  DFRobot_BMP5XX_UART(HardwareSerial *hSerial, uint32_t Baud, uint16_t addr,
                      uint8_t rxpin = 0, uint8_t txpin = 0);
#endif
  ~DFRobot_BMP5XX_UART(void);
  bool begin(void);
};
#endif