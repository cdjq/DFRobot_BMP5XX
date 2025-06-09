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

def begin(self):
'''!
  @fn begin
  @brief Initializes the sensor hardware interface
  @return true if initialization succeeds, false on failure
'''

def set_odr(self, odr):
'''!
  @fn set_odr
  @brief Configures sensor output data rate (ODR)
  @param odr Output data rate selection
  @n Available rates:
  @n - eODR_240_HZ:    240 Hz
  @n - eODR_218_5_HZ:  218.5 Hz
  @n - eODR_199_1_HZ:  199.1 Hz
  @n - eODR_179_2_HZ:  179.2 Hz
  @n - eODR_160_HZ:    160 Hz
  @n - eODR_149_3_HZ:  149.3 Hz
  @n - eODR_140_HZ:    140 Hz
  @n - eODR_129_8_HZ:  129.8 Hz
  @n - eODR_120_HZ:    120 Hz
  @n - eODR_110_1_HZ:  110.1 Hz
  @n - eODR_100_2_HZ:  100.2 Hz
  @n - eODR_89_6_HZ:   89.6 Hz
  @n - eODR_80_HZ:     80 Hz
  @n - eODR_70_HZ:     70 Hz
  @n - eODR_60_HZ:     60 Hz
  @n - eODR_50_HZ:     50 Hz
  @n - eODR_45_HZ:     45 Hz
  @n - eODR_40_HZ:     40 Hz
  @n - eODR_35_HZ:     35 Hz
  @n - eODR_30_HZ:     30 Hz
  @n - eODR_25_HZ:     25 Hz
  @n - eODR_20_HZ:     20 Hz
  @n - eODR_15_HZ:     15 Hz
  @n - eODR_10_HZ:     10 Hz
  @n - eODR_05_HZ:     5 Hz
  @n - eODR_04_HZ:     4 Hz
  @n - eODR_03_HZ:     3 Hz
  @n - eODR_02_HZ:     2 Hz
  @n - eODR_01_HZ:     1 Hz
  @n - eODR_0_5_HZ:    0.5 Hz
  @n - eODR_0_250_HZ:  0.250 Hz
  @n - eODR_0_125_HZ:  0.125 Hz
  @return true if configuration succeeds, false on failure
'''

def set_osr(self, osr_t, osr_p):
'''!
  @fn set_osr
  @brief Sets oversampling ratios for temperature and pressure
  @param osr_t Temperature oversampling
  @param osr_p Pressure oversampling
  @n Supported values:
  @n - eOVERSAMPLING_1X:   1x oversampling
  @n - eOVERSAMPLING_2X:   2x oversampling
  @n - eOVERSAMPLING_4X:   4x oversampling
  @n - eOVERSAMPLING_8X:   8x oversampling
  @n - eOVERSAMPLING_16X:  16x oversampling
  @n - eOVERSAMPLING_32X:  32x oversampling
  @n - eOVERSAMPLING_64X:  64x oversampling
  @n - eOVERSAMPLING_128X: 128x oversampling
  @return true if configuration succeeds, false on failure
'''

def set_measure_mode(self, mode):
'''!
  @fn setMeasureMode
  @brief  set the measurement mode of the sensor
  @param  mode: measurement mode
  @n      ePOWERMODE_STANDBY = 0x00.        #// standby mode
  @n      ePOWERMODE_NORMAL = 0x01.        #// normal mode
  @n      ePOWERMODE_FORCED = 0x02.         #// forced mode > only perform once
  @n      ePOWERMODE_CONTINOUS = 0x03.      #// continuous mode
  @n      ePOWERMODE_DEEP_STANDBY = 0x04.   #// deep standby mode
  @return True if the setting is successful, False otherwise.
'''

def reset(self):
'''!
  @fn reset
  @brief  reset the sensor
  @return True if the reset is successful, False otherwise
'''

def get_temperature(self):
'''!
  @fn get_temperature
  @brief  get the temperature of the sensor
  @return temperature in Celsius
'''

def get_pressure(self):
'''!
  @fn get_pressure
  @brief  get the pressure of the sensor
  @return pressure in Pascal
'''

def get_altitude(self):
'''!
  @fn get_altitude
  @brief  get the altitude of the sensor
  @return altitude in meters
'''

def config_iir(self, iir_t, iir_p):
'''!
  @fn config_iir
  @brief Configures IIR filter coefficients
  @param iir_t Temperature IIR filter
  @param iir_p Pressure IIR filter
  @n Available coefficients:
  @n - eIIR_FILTER_BYPASS:   Bypass filter
  @n - eIIR_FILTER_COEFF_1:  1st order filter
  @n - eIIR_FILTER_COEFF_3:  3rd order filter
  @n - eIIR_FILTER_COEFF_7:  7th order filter
  @n - eIIR_FILTER_COEFF_15: 15th order filter
  @n - eIIR_FILTER_COEFF_31: 31st order filter
  @n - eIIR_FILTER_COEFF_63: 63rd order filter
  @n - eIIR_FILTER_COEFF_127:127th order filter
  @return True if configuration is successful, False otherwise
'''

def config_fifo(self, frame_sel=eFIFO_PRESS_TEMP_DATA, dec_sel=eFIFO_NO_DOWNSAMPLING, mode=eFIFO_STREAM_TO_FIFO_MODE, threshold=0x00):
'''!
  @fn config_fifo
  @brief Configures FIFO operation parameters
  @param frame_sel Data frame type
  @n Available types:
  @n - eFIFO_NOT_ENABLED:    FIFO disabled
  @n - eFIFO_TEMPERATURE_DATA: Temperature data only
  @n - eFIFO_PRESSURE_DATA:    Pressure data only
  @n - eFIFO_PRESS_TEMP_DATA:  Pressure and temperature data
  
  @param dec_sel Downsampling ratio
  @n Available ratios:
  @n - eFIFO_NO_DOWNSAMPLING: No downsampling
  @n - eFIFO_DOWNSAMPLING_2X:  2x downsampling
  @n - eFIFO_DOWNSAMPLING_4X:  4x downsampling
  @n - eFIFO_DOWNSAMPLING_8X:  8x downsampling
  @n - eFIFO_DOWNSAMPLING_16X: 16x downsampling
  @n - eFIFO_DOWNSAMPLING_32X: 32x downsampling
  @n - eFIFO_DOWNSAMPLING_64X: 64x downsampling
  @n - eFIFO_DOWNSAMPLING_128X:128x downsampling
  
  @param mode FIFO operation mode
  @n Available modes:
  @n - eFIFO_STREAM_TO_FIFO_MODE: Stream data continuously
  @n - eFIFO_STOP_ON_FULL_MODE:   Stop when FIFO full
  
  @param threshold FIFO trigger threshold (0=disable, 1-31=frames)]
  @n - 0x0F: 15 frames. This is the maximum setting in PT-mode. The most
  significant bit is ignored.
  @n - 0x1F: 31 frames. This is the maximum setting in P- or T-mode.
  @return True if configuration is successful, False otherwise.
'''

def get_fifo_count(self):
'''!
  @fn get_fifo_count
  @brief  Get the number of frames in the FIFO.
  @return Number of frames in the FIFO.
'''

def get_fifo_data(self):
'''!
  @fn getFIFOData
  @brief Reads all data from FIFO
  @return sFIFOData_t Struct containing pressure and temperature data
  @n - len: Number of stored data frames (0-31)
  @n - pressure: Array of pressure values
  @n - temperature: Array of temperature values
'''

def config_interrupt(self, int_mode, int_pol, int_od):
'''!
  @fn config_interrupt
  @brief Configures interrupt behavior
  @param int_mode Trigger mode
  @n Available modes:
  @n - eINT_MODE_PULSED: Pulsed mode
  @n - eINT_MODE_LATCHED: Latched mode
  
  @param int_pol Signal polarity
  @n Available polarities:
  @n - eINT_POL_ACTIVE_LOW: Active low
  @n - eINT_POL_ACTIVE_HIGH: Active high
  
  @param int_od Output driver type
  @n Available types:
  @n - eINT_OD_PUSH_PULL: Push-pull output
  @n - eINT_OD_OPEN_DRAIN: Open-drain output
  @return True if configuration is successful, False otherwise.
'''

def set_int_source(self, source):
'''!
  @fn set_int_source
  @brief Enables specific interrupt sources
  @param source Bitmask of triggers
  @n Available sources:
  @n - eINT_DATA_DRDY:    Data ready interrupt
  @n - eINT_FIFO_FULL:    FIFO full interrupt
  @n - eINT_FIFO_THRES:   FIFO threshold interrupt
  @n - eINT_PRESSURE_OOR: Pressure out-of-range interrupt
  @return True if configuration is successful, False otherwise.
'''

def get_int_status(self):
'''!
  @brief  Get the interrupt status of the sensor.
  @return Interrupt status.
  @n      eINT_STATUS_DRDY = 0x01,                   # // data ready
  @n      eINT_STATUS_FIFO_FULL = 0x02,              # // FIFO full
  @n      eINT_STATUS_FIFO_THRES = 0x04,             # // FIFO threshold
  @n      eINT_STATUS_PRESSURE_OOR = 0x08,           # // pressure out of range
  # // power on reset/soft reset complete
  @n      eINT_STATUS_POR_SOFTRESET_COMPLETE = 0x10,
'''

def set_oor_press(self, oor, range, cnt_lim):
'''!
  @brief  Set the out of range pressure of the sensor.
  @param  oor: Out of range pressure.
  @n      0x00000 - 0x1FFFF: Out of range pressure
  @param  range: Out of range pressure range.
  @n      0x00 - 0xFF: Out of range pressure range  (oor - range, oor + range)
  @param  cnt_lim: Out of range pressure count limit.
  @n      eOOR_COUNT_LIMIT_1  = 0x00
  @n      eOOR_COUNT_LIMIT_3  = 0x01
  @n      eOOR_COUNT_LIMIT_7  = 0x02
  @n      eOOR_COUNT_LIMIT_15 = 0x03
  @return True if configuration is successful, False otherwise.
'''
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
