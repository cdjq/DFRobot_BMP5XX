# DFRobot_RTU

* [English Version](./README_CN.md)

This is a library for BMP5XX, which reads temperature and pressure. BMP(585/581) is a digital sensor for pressure and temperature measurement based on reliable sensing principles.

![正反面svg效果图](https://github.com/Arya11111/DFRobot_MCP23017/blob/master/resources/images/SEN0245svg1.png)


## Product Link（链接到英文商城）
    
   
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
* This library supports BMP585/581 sensors.​
* This library supports reading temperature and pressure.​
* This library supports setting the sensor's working mode.​
* This library supports setting the sensor's output data rate.​
* This library supports setting the sensor's oversampling rate.​
* This library supports setting the sensor's IIR filter coefficients.​
* This library supports setting the sensor's FIFO operations.​
* This library supports setting the sensor's interrupt behavior.​
* This library supports setting the sensor's pressure out-of-range detection.​
* This library supports reading the sensor's interrupt status.

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
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
 * @n - 0x0F: 15 frames. This is the maximum setting in PT-mode. The most significant bit is ignored. 
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

```

## Compatibility


| MCU                | Work Well | Work Wrong | Untested | Remarks |
| ------------------ |:---------:|:----------:|:--------:| ------- |
| Arduino uno        | √         |            |          |         |
| FireBeetle esp32   | √         |            |          |         |
| FireBeetle esp8266 | √         |            |          |         |
| FireBeetle m0      | √         |            |          |         |
| Leonardo           | √         |            |          |         |
| Microbit           | √         |            |          |         |
| Arduino MEGA2560   | √         |            |          |         |


## History

- Data 2025-06-06
- Version V1.0

## Credits

Written by(yuanlong.yu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
