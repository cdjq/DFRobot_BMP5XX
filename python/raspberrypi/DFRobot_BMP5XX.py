# -*- coding: utf-8 -*
'''!
  @file  DFRobot_BMP5XX.py
  @brief  Define the infrastructure of DFRobot_BMP5XX class
  @n      This is a pressure and temperature sensor that can be controlled through I2C/SPI/UART ports.
  @n      BMP (581/585) has functions such as temperature compensation, data oversampling, IIR filtering, etc
  @n      These features improve the accuracy of data collected by BMP (581/585) sensors.
  @n      BMP (581/585) also has a FIFO data buffer, greatly improving its availability.
  @n      Similarly, BMP (581/585) has an interrupt pin that can be used in an energy-efficient manner without using software algorithms.
  @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
  @version     V1.0.0
  @date        2025-06-06
  @url         https://github.com/DFRobot/DFRobot_BMP5XX
'''
  
import struct
from ctypes import *
from DFRobot_RTU import *
import sys
import time

import smbus
import spidev
import RPi.GPIO as GPIO

import logging
from ctypes import *

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


logger = logging.getLogger()
# logger.setLevel(logging.INFO)   # Display all print information
# If you don’t want to display too many prints, only print errors, please use this option
logger.setLevel(logging.FATAL)
ph = logging.StreamHandler()
formatter = logging.Formatter(
    "%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter)
logger.addHandler(ph)


class DFRobot_BMP5XX(object):
    REG_DATA_LEN_MAX = 0x04
    REG_I_CHIP_ID = 0x01
    REG_I_REV_ID = 0x02
    REG_I_CHIP_STATUS = 0x11
    REG_H_DRIVE_CONFIG = 0x13
    REG_H_INT_CONFIG = 0x14
    REG_H_INT_SOURCE = 0x15
    REG_H_FIFO_CONFIG = 0x16
    REG_I_FIFO_COUNT = 0x17
    REG_H_FIFO_SEL = 0x18
    REG_I_TEMP_DATA_XLSB = 0x1D
    REG_I_TEMP_DATA_LSB = 0x1E
    REG_I_TEMP_DATA_MSB = 0x1F
    REG_I_PRESS_DATA_XLSB = 0x20
    REG_I_PRESS_DATA_LSB = 0x21
    REG_I_PRESS_DATA_MSB = 0x22
    REG_I_INT_STATUS = 0x27
    REG_I_STATUS = 0x28
    REG_I_FIFO_DATA = 0x29
    REG_H_NVM_ADDR = 0x2B
    REG_H_NVM_DATA_LSB = 0x2C
    REG_H_NVM_DATA_MSB = 0x2D
    REG_H_DSP_CONFIG = 0x30
    REG_H_DSP_IIR = 0x31
    REG_H_OOR_THR_P_LSB = 0x32
    REG_H_OOR_THR_P_MSB = 0x33
    REG_H_OOR_RANGE = 0x34
    REG_H_OOR_CONFIG = 0x35
    REG_H_OSR_CONFIG = 0x36
    REG_H_ODR_CONFIG = 0x37
    REG_I_OSR_EFF = 0x38
    REG_H_CMD = 0x7E

    # /** 使能 */
    ENABLE = 0x01
    DISABLE = 0x00

    DEEP_ENABLE = 0x00
    DEEP_DISABLE = 0x01

    BMP581_CHIP_ID = 0x50
    BMP585_CHIP_ID = 0x51

    # /*! @name Soft reset command */
    SOFT_RESET_CMD = 0xB6

    # /** 位位置和宽度宏定义 */
    # // REG_INT_CONFIG (0x14)
    INT_MODE_POS = 0
    INT_MODE_WIDTH = 1
    INT_POL_POS = 1
    INT_POL_WIDTH = 1
    INT_OD_POS = 2
    INT_OD_WIDTH = 1
    INT_EN_POS = 3
    INT_EN_WIDTH = 1
    PAD_INT_DRV_POS = 4
    PAD_INT_DRV_WIDTH = 4

    # // REG_INT_SRC_SEL (0x15)
    DRDY_DATA_REG_EN_POS = 0
    DRDY_DATA_REG_EN_WIDTH = 1
    FIFO_FULL_EN_POS = 1
    FIFO_FULL_EN_WIDTH = 1
    FIFO_THS_EN_POS = 2
    FIFO_THS_EN_WIDTH = 1
    OOR_P_EN_POS = 3
    OOR_P_EN_WIDTH = 1

    # // REG_FIFO_CONFIG (0x16)
    FIFO_THRESHOLD_POS = 0
    FIFO_THRESHOLD_WIDTH = 5
    FIFO_MODE_POS = 5
    FIFO_MODE_WIDTH = 1

    # // REG_FIFO_COUNT (0x17)
    FIFO_COUNT_POS = 0
    FIFO_COUNT_WIDTH = 6

    # // REG_FIFO_SEL_CONFIG (0x18)
    FIFO_FRAME_SEL_POS = 0
    FIFO_FRAME_SEL_WIDTH = 2
    FIFO_DEC_SEL_POS = 2
    FIFO_DEC_SEL_WIDTH = 3

    # // REG_NVM_ADDRESS (0x2B)
    NVM_ROW_ADDRESS_POS = 0
    NVM_ROW_ADDRESS_WIDTH = 6
    NVM_PROG_EN_POS = 6
    NVM_PROG_EN_WIDTH = 1

    # // REG_DSP_CONFIG (0x30)
    IIR_FLUSH_FORCED_EN_POS = 2
    IIR_FLUSH_FORCED_EN_WIDTH = 1
    SHDW_SEL_IIR_T_POS = 3
    SHDW_SEL_IIR_T_WIDTH = 1
    FIFO_SEL_IIR_T_POS = 4
    FIFO_SEL_IIR_T_WIDTH = 1
    SHDW_SEL_IIR_P_POS = 5
    SHDW_SEL_IIR_P_WIDTH = 1
    FIFO_SEL_IIR_P_POS = 6
    FIFO_SEL_IIR_P_WIDTH = 1
    OOR_SEL_IIR_P_POS = 7
    OOR_SEL_IIR_P_WIDTH = 1

    # // REG_DSP_IIR_CONFIG (0x31)
    SET_IIR_T_POS = 0
    SET_IIR_T_WIDTH = 3
    SET_IIR_P_POS = 3
    SET_IIR_P_WIDTH = 3

    # // REG_OOR_CONFIG (0x35)
    OOR_THR_P_16_POS = 0
    OOR_THR_P_16_WIDTH = 1
    CNT_LIM_POS = 6
    CNT_LIM_WIDTH = 2

    # // REG_OSR_CONFIG (0x36)
    OSR_T_POS = 0
    OSR_T_WIDTH = 3
    OSR_P_POS = 3
    OSR_P_WIDTH = 3
    PRESS_EN_POS = 6
    PRESS_EN_WIDTH = 1

    # // REG_ODR_CONFIG (0x37)
    PWR_MODE_POS = 0
    PWR_MODE_WIDTH = 2
    ODR_POS = 2
    ODR_WIDTH = 5
    DEEP_DIS_POS = 7
    DEEP_DIS_WIDTH = 1

    # // REG_EFF_OSR_CONFIG (0x38)
    OSR_T_EFF_POS = 0
    OSR_T_EFF_WIDTH = 3
    OSR_P_EFF_POS = 3
    OSR_P_EFF_WIDTH = 3
    ODR_IS_VALID_POS = 7
    ODR_IS_VALID_WIDTH = 1

    STANDARD_SEA_LEVEL_PRESSURE_PA = 101325   #///< Standard sea level pressure, unit: pa

    eDISABLE = 0
    eENABLE = 1

    eDEEP_ENABLE = 0
    eDEEP_DISABLE = 1

    eODR_240_HZ = 0x00
    eODR_218_5_HZ = 0x01
    eODR_199_1_HZ = 0x02
    eODR_179_2_HZ = 0x03
    eODR_160_HZ = 0x04
    eODR_149_3_HZ = 0x05
    eODR_140_HZ = 0x06
    eODR_129_8_HZ = 0x07
    eODR_120_HZ = 0x08
    eODR_110_1_HZ = 0x09
    eODR_100_2_HZ = 0x0A
    eODR_89_6_HZ = 0x0B
    eODR_80_HZ = 0x0C
    eODR_70_HZ = 0x0D
    eODR_60_HZ = 0x0E
    eODR_50_HZ = 0x0F
    eODR_45_HZ = 0x10
    eODR_40_HZ = 0x11
    eODR_35_HZ = 0x12
    eODR_30_HZ = 0x13
    eODR_25_HZ = 0x14
    eODR_20_HZ = 0x15
    eODR_15_HZ = 0x16
    eODR_10_HZ = 0x17
    eODR_05_HZ = 0x18
    eODR_04_HZ = 0x19
    eODR_03_HZ = 0x1A
    eODR_02_HZ = 0x1B
    eODR_01_HZ = 0x1C
    eODR_0_5_HZ = 0x1D
    eODR_0_250_HZ = 0x1E
    eODR_0_125_HZ = 0x1F

    eOVERSAMPLING_1X = 0x00
    eOVERSAMPLING_2X = 0x01
    eOVERSAMPLING_4X = 0x02
    eOVERSAMPLING_8X = 0x03
    eOVERSAMPLING_16X = 0x04
    eOVERSAMPLING_32X = 0x05
    eOVERSAMPLING_64X = 0x06
    eOVERSAMPLING_128X = 0x07

    eIIR_FILTER_BYPASS = 0x00
    eIIR_FILTER_COEFF_1 = 0x01
    eIIR_FILTER_COEFF_3 = 0x02
    eIIR_FILTER_COEFF_7 = 0x03
    eIIR_FILTER_COEFF_15 = 0x04
    eIIR_FILTER_COEFF_31 = 0x05
    eIIR_FILTER_COEFF_63 = 0x06
    eIIR_FILTER_COEFF_127 = 0x07

    eOOR_COUNT_LIMIT_1 = 0x00
    eOOR_COUNT_LIMIT_3 = 0x01
    eOOR_COUNT_LIMIT_7 = 0x02
    eOOR_COUNT_LIMIT_15 = 0x03

    ePOWERMODE_STANDBY = 0x00  # // 待机模式
    ePOWERMODE_NORMAL = 0x01  # // 普通模式
    ePOWERMODE_FORCED = 0x02  # // 强制模式 > 只进行一次
    ePOWERMODE_CONTINOUS = 0x03  # // 连续模式
    ePOWERMODE_DEEP_STANDBY = 0x04  # // 深度待机模式

    # /*! Fifo disabled */
    eFIFO_NOT_ENABLED = 0x00
    # v/*! Fifo temperature data only enabled */
    eFIFO_TEMPERATURE_DATA = 0x01
    # /*! Fifo pressure data only enabled */
    eFIFO_PRESSURE_DATA = 0x02
    # /*! Fifo pressure and temperature data enabled */
    eFIFO_PRESS_TEMP_DATA = 0x03

    eFIFO_NO_DOWNSAMPLING = 0x00
    eFIFO_DOWNSAMPLING_2X = 0x01
    eFIFO_DOWNSAMPLING_4X = 0x02
    eFIFO_DOWNSAMPLING_8X = 0x03
    eFIFO_DOWNSAMPLING_16X = 0x04
    eFIFO_DOWNSAMPLING_32X = 0x05
    eFIFO_DOWNSAMPLING_64X = 0x06
    eFIFO_DOWNSAMPLING_128X = 0x07

    eFIFO_STREAM_TO_FIFO_MODE = 0x00
    eFIFO_STOP_ON_FULL_MODE = 0x01

    eINT_STATUS_DRDY = 0x01                   # // 数据就绪
    eINT_STATUS_FIFO_FULL = 0x02              # // FIFO已满
    eINT_STATUS_FIFO_THRES = 0x04             # // FIFO达到阈值
    eINT_STATUS_PRESSURE_OOR = 0x08           # // 气压超出测量范围
    eINT_STATUS_POR_SOFTRESET_COMPLETE = 0x10  # // 上电复位/软复位完成

    eINT_DATA_DRDY = 0x01
    eINT_FIFO_FULL = 0x02
    eINT_FIFO_THRES = 0x04
    eINT_PRESSURE_OOR = 0x08

    eINT_MODE_PULSED = 0x00
    eINT_MODE_LATCHED = 0x01

    eINT_POL_ACTIVE_LOW = 0x00
    eINT_POL_ACTIVE_HIGH = 0x01

    eINT_OD_PUSH_PULL = 0x00
    eINT_OD_OPEN_DRAIN = 0x01

    eINT_DISABLE = 0x00
    eINT_ENABLE = 0x01



    class sFIFOData_t(Structure):
        _fields_ = [
            ("temperature", c_float * 32),
            ("pressure", c_float * 32),
            ("len", c_uint8)
        ]

        def clear(self):
            self.len = 0
            for i in range(32):
                self.pressure[i] = 0.0
            for i in range(32):
                self.temperature[i] = 0.0

    def __init__(self):
        pass

    def begin(self):
        '''!
          @fn begin
          @brief Initializes the sensor hardware interface
          @return true if initialization succeeds, false on failure
        '''
        self.reset()
        chip_data = self._read_input_reg(self.REG_I_CHIP_ID, 1)
        self._calibrated = False
        self._sealevelAltitude = 0.0
        if chip_data[0] not in (self.BMP581_CHIP_ID, self.BMP585_CHIP_ID):
            return False
        self._enable_pressure(self.eENABLE)
        return True

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
        if odr not in range(self.eODR_240_HZ, self.eODR_0_125_HZ + 1):
            return False
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.ODR_POS, self.ODR_WIDTH, odr)
        self._write_holding_reg(self.REG_H_ODR_CONFIG, data)
        return True

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
        if osr_t not in range(self.eOVERSAMPLING_1X, self.eOVERSAMPLING_128X + 1):
            return False
        if osr_p not in range(self.eOVERSAMPLING_1X, self.eOVERSAMPLING_128X + 1):
            return False
        data = self._read_holding_reg(self.REG_H_OSR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.OSR_T_POS, self.OSR_T_WIDTH, osr_t)
        data[0] = self._REG_SET_BITS(
            data[0], self.OSR_P_POS, self.OSR_P_WIDTH, osr_p)
        self._write_holding_reg(self.REG_H_OSR_CONFIG, data)
        return True

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
        if mode not in (self.ePOWERMODE_STANDBY, self.ePOWERMODE_NORMAL, self.ePOWERMODE_FORCED, self.ePOWERMODE_CONTINOUS, self.ePOWERMODE_DEEP_STANDBY):
            return False
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        currMode = self._REG_GET_BITS(
            data[0], self.PWR_MODE_POS, self.PWR_MODE_WIDTH)
        if currMode != self.ePOWERMODE_STANDBY:
            self._set_power_mode(self.ePOWERMODE_STANDBY)
            time.sleep(0.0025)
        if mode == self.ePOWERMODE_DEEP_STANDBY:
            self._set_deep_standby_mode(self.DEEP_ENABLE)
        elif mode in (self.ePOWERMODE_CONTINOUS, self.ePOWERMODE_FORCED, self.ePOWERMODE_NORMAL):
            self._set_power_mode(mode)
        return True

    def reset(self):
        '''!
          @fn reset
          @brief  reset the sensor
          @return True if the reset is successful, False otherwise
        '''
        self._write_holding_reg(self.REG_H_CMD, self.SOFT_RESET_CMD)
        time.sleep(0.002)
        self._read_input_reg(self.REG_I_CHIP_ID, 1)

        intStatus = self.get_int_status()
        if intStatus & self.eINT_STATUS_POR_SOFTRESET_COMPLETE:
            return True
        return False

    def get_temperature(self):
        '''!
          @fn get_temperature
          @brief  get the temperature of the sensor
          @return temperature in Celsius
        '''
        data = self._read_input_reg(self.REG_I_TEMP_DATA_XLSB, 3)

        tmpData = self.convert_data(data)

        return tmpData / 65536.0

    def get_pressure(self):
        '''!
          @fn get_pressure
          @brief  get the pressure of the sensor
          @return pressure in Pascal
        '''
        data = self._read_input_reg(self.REG_I_PRESS_DATA_XLSB, 3)

        pressData = self.convert_data(data) / 64.0
        if self._calibrated:
            seaLevelPressPa = (pressData / (1.0 - (self._sealevelAltitude / 44307.7)) ** 5.255302)
            pressData = pressData - seaLevelPressPa + self.STANDARD_SEA_LEVEL_PRESSURE_PA
        return pressData

    def get_altitude(self):
        '''!
          @fn get_altitude
          @brief  get the altitude of the sensor
          @return altitude in meters
        '''
        data = self._read_input_reg(self.REG_I_TEMP_DATA_XLSB, 6)

        tmpData = self.convert_data(data)

        pressData = self.convert_data(data[3:6])

        temperature = tmpData / 65536.0
        pressure = pressData / 64.0
        if self._calibrated:
            seaLevelPressPa = (pressure / (1.0 - (self._sealevelAltitude / 44307.7)) ** 5.255302)
            pressure = pressure - seaLevelPressPa + self.STANDARD_SEA_LEVEL_PRESSURE_PA
        return self._calculate_altitude(temperature, pressure)

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
        if iir_t not in range(self.eIIR_FILTER_BYPASS, self.eIIR_FILTER_COEFF_127 + 1):
            return False
        if iir_p not in range(self.eIIR_FILTER_BYPASS, self.eIIR_FILTER_COEFF_127 + 1):
            return False

        currMode = self._get_power_mode()
        self.set_measure_mode(self.ePOWERMODE_STANDBY)
        data = self._read_holding_reg(self.REG_H_DSP_CONFIG, 2)
        data[0] = self._REG_SET_BITS(
            data[0], self.SHDW_SEL_IIR_T_POS, self.SHDW_SEL_IIR_T_WIDTH, self.eENABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.SHDW_SEL_IIR_P_POS, self.SHDW_SEL_IIR_P_WIDTH, self.eENABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.IIR_FLUSH_FORCED_EN_POS, self.IIR_FLUSH_FORCED_EN_WIDTH, self.eENABLE)

        data[1] = self._REG_SET_BITS(
            data[1], self.SET_IIR_T_POS, self.SET_IIR_T_WIDTH, iir_t)
        data[1] = self._REG_SET_BITS(
            data[1], self.SET_IIR_T_POS, self.SET_IIR_P_WIDTH, iir_p)

        self._write_holding_reg(self.REG_H_DSP_CONFIG, data)
        if currMode not in (self.ePOWERMODE_DEEP_STANDBY, self.ePOWERMODE_STANDBY):
            return self.set_measure_mode(currMode)
        return True

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
        if frame_sel not in range(self.eFIFO_NOT_ENABLED, self.eFIFO_PRESS_TEMP_DATA + 1):
            return False
        if dec_sel not in range(self.eFIFO_NO_DOWNSAMPLING, self.eFIFO_DOWNSAMPLING_128X + 1):
            return False
        if mode not in (self.eFIFO_STREAM_TO_FIFO_MODE, self.eFIFO_STOP_ON_FULL_MODE):
            return False
        currMode = self._get_power_mode()
        self.set_measure_mode(self.ePOWERMODE_STANDBY)
        data = self._read_holding_reg(self.REG_H_DSP_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_SEL_IIR_T_POS, self.FIFO_SEL_IIR_T_WIDTH, self.eENABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_SEL_IIR_P_POS, self.FIFO_SEL_IIR_P_WIDTH, self.eENABLE)
        self._write_holding_reg(self.REG_H_DSP_CONFIG, data)

        data = self._read_holding_reg(self.REG_H_FIFO_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_MODE_POS, self.FIFO_MODE_WIDTH, mode)
        data[0] = self._set_fifo_threshold(data[0], frame_sel, threshold)
        self._write_holding_reg(self.REG_H_FIFO_CONFIG, data)

        data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH, frame_sel)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_DEC_SEL_POS, self.FIFO_DEC_SEL_WIDTH, dec_sel)
        self._write_holding_reg(self.REG_H_FIFO_SEL, data)

        if currMode not in (self.ePOWERMODE_DEEP_STANDBY, self.ePOWERMODE_STANDBY):
            return self.set_measure_mode(currMode)
        return True

    def get_fifo_count(self):
        '''!
          @fn get_fifo_count
          @brief  Get the number of frames in the FIFO.
          @return Number of frames in the FIFO.
        '''
        data = self._read_input_reg(self.REG_I_FIFO_COUNT, 1)
        count = self._REG_GET_BITS(
            data[0], self.FIFO_COUNT_POS, self.FIFO_COUNT_WIDTH)
        return count

    def get_fifo_data(self):
        '''!
          @fn getFIFOData
          @brief Reads all data from FIFO
          @return sFIFOData_t Struct containing pressure and temperature data
          @n - len: Number of stored data frames (0-31)
          @n - pressure: Array of pressure values
          @n - temperature: Array of temperature values
        '''
        fifo_data = self.sFIFOData_t()
        fifo_data.clear()
        fifo_count = self.get_fifo_count()
        fifo_data.len = fifo_count
        data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
        fifo_frame_sel = self._REG_GET_BITS(
            data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH)
        if fifo_frame_sel == self.eFIFO_PRESSURE_DATA:
            for i in range(fifo_count):
                data = self._read_input_reg(self.REG_I_FIFO_DATA, 3)
                pressure = self.convert_data(data) / 64.0
                if self._calibrated:
                    seaLevelPressPa = (pressure / (1.0 - (self._sealevelAltitude / 44307.7)) ** 5.255302)
                    pressure = pressure - seaLevelPressPa + self.STANDARD_SEA_LEVEL_PRESSURE_PA
                fifo_data.pressure[i] = pressure
        elif fifo_frame_sel == self.eFIFO_TEMPERATURE_DATA:
            for i in range(fifo_count):
                data = self._read_input_reg(self.REG_I_FIFO_DATA, 3)
                fifo_data.temperature[i] = self.convert_data(data) / 65536.0
        elif fifo_frame_sel == self.eFIFO_PRESS_TEMP_DATA:
            for i in range(fifo_count):
                data = self._read_input_reg(self.REG_I_FIFO_DATA, 6)
                tmpData = self.convert_data(data)
                pressData = self.convert_data(data[3:6])
                fifo_data.temperature[i] = tmpData / 65536.0
                pressure = pressData / 64.0
                if self._calibrated:
                    seaLevelPressPa = (pressure / (1.0 - (self._sealevelAltitude / 44307.7)) ** 5.255302)
                    pressure = pressure - seaLevelPressPa + self.STANDARD_SEA_LEVEL_PRESSURE_PA
                fifo_data.pressure[i] = pressure
        return fifo_data

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
        if int_mode not in (self.eINT_MODE_PULSED, self.eINT_MODE_LATCHED):
            return False
        if int_pol not in (self.eINT_POL_ACTIVE_LOW, self.eINT_POL_ACTIVE_HIGH):
            return False
        if int_od not in (self.eINT_OD_PUSH_PULL, self.eINT_OD_OPEN_DRAIN):
            return False
        '''
          Any change between latched/pulsed mode has to be applied while interrupt is disabled
          Step 1 : Turn off all INT sources (INT_SOURCE -> 0x00)
        '''
        self._write_holding_reg(self.REG_H_INT_SOURCE, [0x00])
        data = self._read_holding_reg(self.REG_H_INT_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_MODE_POS, self.INT_MODE_WIDTH, int_mode)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_POL_POS, self.INT_POL_WIDTH, int_pol)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_OD_POS, self.INT_OD_WIDTH, int_od)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_EN_POS, self.INT_EN_WIDTH, self.eINT_ENABLE)
        self._write_holding_reg(self.REG_H_INT_CONFIG, data)
        return True

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
        VALID_INT_MASK = self.eINT_DATA_DRDY | self.eINT_FIFO_FULL | self.eINT_FIFO_THRES | self.eINT_PRESSURE_OOR
        if source & ~VALID_INT_MASK:
            return False
        data = source & VALID_INT_MASK
        self._write_holding_reg(self.REG_H_INT_SOURCE, data)
        return True

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
        data = self._read_input_reg(self.REG_I_INT_STATUS, 1)
        return data[0]

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
        if oor not in range(0x00000, 0x1FFFF + 1):
            return False
        if range not in range(0x00, 0xFF + 1):
            return False
        if cnt_lim not in (self.eOOR_COUNT_LIMIT_1, self.eOOR_COUNT_LIMIT_3, self.eOOR_COUNT_LIMIT_7, self.eOOR_COUNT_LIMIT_15):
            return False
        currMode = self._get_power_mode()
        self.set_measure_mode(self.ePOWERMODE_STANDBY)

        data = self._read_holding_reg(self.REG_H_OOR_THR_P_LSB, 4)
        data[0] = oor & 0xFF
        data[1] = (oor >> 8) & 0xFF
        self._REG_SET_BITS(data[3], self.OOR_THR_P_16_POS,
                           self.OOR_THR_P_16_WIDTH, (oor >> 16) & 0x01)

        data[2] = range
        self._REG_SET_BITS(data[3], self.CNT_LIM_POS,
                           self.CNT_LIM_WIDTH, cnt_lim)
        self._write_holding_reg(self.REG_H_OOR_THR_P_LSB, data)
        if currMode not in (self.ePOWERMODE_DEEP_STANDBY, self.ePOWERMODE_STANDBY):
            return self.set_measure_mode(currMode)
        return True

    def calibrated_absolute_difference(self, altitude):
        '''!
            @fn calibratedAbsoluteDifference
            @brief use the given current altitude as a reference value, eliminate the absolute difference of subsequent pressure and altitude data
            @param altitude current altitude
            @return boolean, indicates whether the reference value is set successfully
            @retval True indicates the reference value is set successfully
            @retval False indicates fail to set the reference value
        '''
        ret = False
        if altitude > 0:
            self._calibrated = True
            self._sealevelAltitude = altitude
            ret = True
        return ret

    def _enable_pressure(self, enable):
        '''!
            @brief  enable or disable pressure measurement
            @param  enable: enable or disable.
            @n      eENABLE = 0x01
            @n      eDISABLE = 0x00
            @return True if the setting is successful, False otherwise
        '''
        if enable not in (self.eENABLE, self.eDISABLE):
            return False
        data = self._read_holding_reg(self.REG_H_OSR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.PRESS_EN_POS, self.PRESS_EN_WIDTH, enable)
        self._write_holding_reg(self.REG_H_OSR_CONFIG, data)
        return True

    def _set_deep_standby_mode(self, deepMode):
        '''!
            @brief  Set the deep standby mode of the sensor.
            @param  deepMode: Deep standby mode.
            @n      eDEEP_DISABLE = 0x00,
            @n      eDEEP_ENABLE = 0x01,
            @return True if configuration is successful, False otherwise.
        '''
        if deepMode == self.DEEP_ENABLE:
            data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
            data[0] = self._REG_SET_BITS(
                data[0], self.DEEP_DIS_POS, self.DEEP_DIS_WIDTH, self.DEEP_ENABLE)
            data[0] = self._REG_SET_BITS(
                data[0], self.ODR_POS, self.ODR_WIDTH, self.eODR_01_HZ)
            self._write_holding_reg(self.REG_H_ODR_CONFIG, data)

            data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
            data[0] = self._REG_SET_BITS(
                data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH, self.eFIFO_NOT_ENABLED)
            self._write_holding_reg(self.REG_H_FIFO_SEL, data)

            data = self._read_holding_reg(self.REG_H_DSP_IIR, 1)
            data[0] = self._REG_SET_BITS(
                data[0], self.SET_IIR_T_POS, self.SET_IIR_T_WIDTH, self.eIIR_FILTER_BYPASS)
            data[0] = self._REG_SET_BITS(
                data[0], self.SET_IIR_P_POS, self.SET_IIR_P_WIDTH, self.eIIR_FILTER_BYPASS)
            self._write_holding_reg(self.REG_H_DSP_IIR, data)
        else:
            currMode = self._get_power_mode()
            if currMode == self.ePOWERMODE_DEEP_STANDBY:
                return self.set_measure_mode(self.ePOWERMODE_STANDBY)
        return True

    def _set_power_mode(self, mode):
        '''!
            @brief  Set the power mode of the sensor.
            @param  mode: Power mode.
            @n      ePOWERMODE_STANDBY = 0x00,
            @n      ePOWERMODE_NORMAL = 0x01,
            @n      ePOWERMODE_FORCED = 0x02,
            @n      ePOWERMODE_CONTINOUS = 0x03,
            @n      ePOWERMODE_DEEP_STANDBY = 0x04,
            @return True if configuration is successful, False otherwise.
        '''
        if mode not in (self.ePOWERMODE_STANDBY, self.ePOWERMODE_NORMAL, self.ePOWERMODE_FORCED, self.ePOWERMODE_CONTINOUS, self.ePOWERMODE_DEEP_STANDBY):
            return False
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.DEEP_DIS_POS, self.DEEP_DIS_WIDTH, self.DEEP_DISABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.PWR_MODE_POS, self.PWR_MODE_WIDTH, mode)
        self._write_holding_reg(self.REG_H_ODR_CONFIG, data)
        return True

    def _get_power_mode(self):
        '''!
            @brief  Get the power mode of the sensor.
            @return Power mode.
            @n      ePOWERMODE_STANDBY = 0x00,
            @n      ePOWERMODE_NORMAL = 0x01,
            @n      ePOWERMODE_FORCED = 0x02,
            @n      ePOWERMODE_CONTINOUS = 0x03,
            @n      ePOWERMODE_DEEP_STANDBY = 0x04,
        '''
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        currMode = self._REG_GET_BITS(
            data[0], self.PWR_MODE_POS, self.PWR_MODE_WIDTH)
        deep_dis = self._REG_GET_BITS(
            data[0], self.DEEP_DIS_POS, self.DEEP_DIS_WIDTH)
        if currMode == self.ePOWERMODE_STANDBY and deep_dis == self.DEEP_ENABLE:
            return self._verify_deep_standby_mode()
        return currMode

    def _verify_deep_standby_mode(self):
        '''!
            @brief  Verify the deep standby mode of the sensor.
            @return True if the deep standby mode is verified, False otherwise.
        '''
        mode = self.ePOWERMODE_STANDBY
        data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
        fifo_frame_sel = self._REG_GET_BITS(
            data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH)

        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        odr = self._REG_GET_BITS(data[0], self.ODR_POS, self.ODR_WIDTH)

        data = self._read_holding_reg(self.REG_H_DSP_IIR, 1)
        iir_t = self._REG_GET_BITS(
            data[0], self.SET_IIR_T_POS, self.SET_IIR_T_WIDTH)
        iir_p = self._REG_GET_BITS(
            data[0], self.SET_IIR_P_POS, self.SET_IIR_P_WIDTH)

        if odr > self.eODR_05_HZ and fifo_frame_sel == self.DISABLE and iir_t == self.eIIR_FILTER_BYPASS and iir_p == self.eIIR_FILTER_BYPASS:
            mode = self.ePOWERMODE_STANDBY
        return mode

    def _set_fifo_threshold(self, data, frame_sel, threshold):
        '''!
            @brief  Set the FIFO threshold of the sensor.
            @param  data: Data to be set.
            @param  frame_sel: Frame selection.
            @param  threshold: Threshold.
            @return Data to be set.
        '''
        if frame_sel == self.eFIFO_TEMPERATURE_DATA or frame_sel == self.eFIFO_PRESSURE_DATA:
            if threshold <= 0x1F:
                return self._REG_SET_BITS(data, self.FIFO_THRESHOLD_POS, self.FIFO_THRESHOLD_WIDTH, threshold)
        elif frame_sel == self.eFIFO_PRESS_TEMP_DATA:
            if threshold <= 0x0F:
                return self._REG_SET_BITS(data, self.FIFO_THRESHOLD_POS, self.FIFO_THRESHOLD_WIDTH, threshold)
        return 0

    def _calculate_altitude(self, temperature_c, pressure_pa):
        '''!
            @brief  Calculate the altitude of the sensor.
            @param  temperature_c: Temperature in Celsius.
            @param  pressure_pa: Pressure in Pascal.
            @return Altitude in meters.
        '''
        return (1.0 - (pressure_pa / 101325) ** 0.190284) * 44307.7

    def _REG_BIT_MASK(self, pos, width):
        return ((1 << width) - 1) << pos

    def _REG_SET_BITS(self, reg, pos, width, val):
        cleared_reg = reg & ~self._REG_BIT_MASK(pos, width)
        return cleared_reg | ((val & ((1 << width) - 1)) << pos)

    def _REG_GET_BITS(self, reg, pos, width):
        return (reg >> pos) & ((1 << width) - 1)

    def convert_data(self, data):
        '''!
            @brief  Convert data to int.
            @param  data: Data to be converted.
            @return Converted data.
        '''
        byte0 = data[0] & 0xFF
        byte1 = data[1] & 0xFF
        byte2 = data[2] & 0xFF

        if byte2 & 0x80:
            byte2_signed = byte2 - 0x100
        else:
            byte2_signed = byte2

        data = (byte2_signed << 16) | (byte1 << 8) | byte0
        return data

    def _write_holding_reg(self, reg, data):
        pass

    def _read_input_reg(self, reg, len):
        pass

    def _read_holding_reg(self, reg, len):
        pass


class DFRobot_BMP5XX_I2C(DFRobot_BMP5XX):
    def __init__(self, bus, addr):
        self.__addr = addr
        self.__i2cbus = smbus.SMBus(bus)
        super(DFRobot_BMP5XX_I2C, self).__init__()

    def _write_holding_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
        ret = self.__i2cbus.write_i2c_block_data(self.__addr, reg, data)
        time.sleep(0.002)
        return ret

    def _read_input_reg(self, reg, len):
        return self.__i2cbus.read_i2c_block_data(self.__addr, reg, len)

    def _read_holding_reg(self, reg, len):
        return self.__i2cbus.read_i2c_block_data(self.__addr, reg, len)


class DFRobot_BMP5XX_SPI(DFRobot_BMP5XX):
    def __init__(self, cs=8, bus=0, dev=0, speed=8000000):
        self.__cs = cs
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.__cs, GPIO.OUT, initial=1)
        GPIO.output(self.__cs, GPIO.LOW)
        self.__spi = spidev.SpiDev()
        self.__spi.open(bus, dev)
        self.__spi.no_cs = True
        self.__spi.max_speed_hz = speed
        super(DFRobot_BMP5XX_SPI, self).__init__()

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        if isinstance(data, int):
            data = [data]
            # logger.info(data)
        reg_addr = [reg & 0x7f]
        GPIO.output(self.__cs, GPIO.LOW)
        self.__spi.xfer(reg_addr)
        self.__spi.xfer(data)
        GPIO.output(self.__cs, GPIO.HIGH)

    def _read_reg(self, reg, len):
        '''!
          @brief read the data from the register
          @param reg register address
          @param len read data length
          @return read data list
        '''
        reg_addr = [reg | 0x80]
        GPIO.output(self._cs, GPIO.LOW)
        # logger.info(reg_addr)
        self.__spi.xfer(reg_addr)
        time.sleep(0.01)
        self.__spi.readbytes(1)
        rslt = self.__spi.readbytes(len)
        GPIO.output(self.__cs, GPIO.HIGH)
        return rslt

    def _write_holding_reg(self, reg, data):
        self._write_reg(reg, data)

    def _read_input_reg(self, reg, len):
        return self._read_reg(reg, len)

    def _read_holding_reg(self, reg, len):
        return self._read_reg(reg, len)


class DFRobot_BMP5XX_UART(DFRobot_BMP5XX, DFRobot_RTU):
    def __init__(self, baud, addr):
        self.__baud = baud
        self.__addr = addr
        DFRobot_BMP5XX.__init__(self)
        DFRobot_RTU.__init__(self, baud, 8, 'N', 1)

    def _write_holding_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
        ret = self.write_holding_registers(self.__addr, reg, data)
        return ret

    def _read_input_reg(self, reg, len):
        try:
            data = self.read_input_registers(self.__addr, reg, len)

            return data
        except Exception as e:
            return [0]

    def _read_holding_reg(self, reg, len):
        try:
            data = self.read_holding_registers(self.__addr, reg, len)

            return data
        except Exception as e:
            return [0]
