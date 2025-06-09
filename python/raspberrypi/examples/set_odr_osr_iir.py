# -*- coding: utf-8 -*-
'''!
  @file  set_odr_osr_iir.py
  @brief  Configure ODR (Output Data Rate), OSR (Oversampling Rate) and IIR filter for BMP5XX sensor
  @details  Demonstrate how to set key parameters of the sensor, including:
            - Configure ODR (output data rate)
            - Configure OSR (temperature and pressure oversampling rates)
            - Configure IIR filter coefficients
            - Enable data ready interrupt and read sensor data
  @copyright  Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     yuanlong.yu(yuanlong.yu@dfrobot.com)
  @version    V1.0.0
  @date       2025-06-06
  @url        https://github.com/DFRobot/DFRobot_BMP5XX
'''

import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_BMP5XX import *

DEV_ADDR   = 0x47

# Please choose your communication method below:
# mode = "UART"
# mode = "SPI"
mode = "I2C"


if mode == "I2C":
    I2C_BUS    = 0x01
    bmp5 = DFRobot_BMP5XX_I2C(I2C_BUS, DEV_ADDR)
elif mode == "SPI":
    CS         = 8
    bmp5 = DFRobot_BMP5XX_SPI(cs=CS, bus=0, dev=0, speed=8000000)
elif mode == "UART":
    bmp5 = DFRobot_BMP5XX_UART(9600, DEV_ADDR)

global interrupt_flag
interrupt_flag = False

def drdy_callback():
    global interrupt_flag
    interrupt_flag = True

gpio_interrupt = 27
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_interrupt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(gpio_interrupt, GPIO.RISING, callback=drdy_callback)

def setup():
    while not bmp5.begin():
        print("sensor init error,please check connect!")
        time.sleep(1)
    '''!
      Maximum nominal ODR setting per OSR settings in NORMAL mode
      max ODR [Hz] represents the maximum output data rate (unit: Hertz).
      OSR_T is a parameter related to the temperature oversampling rate, 
      with values of 1, 2, 4, 8, 16, 32, 64, 128.
      OSR_P is a parameter related to the pressure oversampling rate, 
      with values of 1, 2, 4, 8, 16, 32, 64, 128.
      The table content is as follows:
      | max ODR [Hz] | OSR_T = 1 | OSR_T = 2 | OSR_T = 4 | OSR_T = 8 | OSR_T = 16 | OSR_T = 32 | OSR_T = 64 | OSR_T = 128 |
      | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
      | OSR_P = 1 | 240.00 | 240.00 | 240.00 | 240.00 | 200.00 | 130.00 | 80.00 | 40.00 |
      | OSR_P = 2 | 240.00 | 240.00 | 240.00 | 220.00 | 180.00 | 120.00 | 70.00 | 40.00 |
      | OSR_P = 4 | 220.00 | 220.00 | 200.00 | 180.00 | 140.00 | 100.00 | 70.00 | 40.00 |
      | OSR_P = 8 | 140.00 | 140.00 | 130.00 | 120.00 | 100.00 | 80.00 | 50.00 | 35.00 |
      | OSR_P = 16 | 80.00 | 80.00 | 80.00 | 70.00 | 70.00 | 50.00 | 45.00 | 30.00 |
      | OSR_P = 32 | 45.00 | 45.00 | 40.00 | 40.00 | 40.00 | 35.00 | 30.00 | 20.00 |
      | OSR_P = 64 | 20.00 | 20.00 | 20.00 | 20.00 | 20.00 | 20.00 | 15.00 | 15.00 |
      | OSR_P = 128 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 5.00 |

      Maximum nominal ODR setting per OSR settings in NORMAL mode 
      for temperature only measurements
      max ODR [Hz] represents the maximum output data rate (unit: Hertz).
      OSR_T is a parameter related to the temperature oversampling rate, 
      with values of 1, 2, 4, 8, 16, 32, 64, 128.
      The table content is as follows:
      | max ODR [Hz] | OSR_T = 1 | OSR_T = 2 | OSR_T = 4 | OSR_T = 8 | OSR_T = 16 | OSR_T = 32 | OSR_T = 64 | OSR_T = 128 |
      | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
      | - | 240.00 | 240.00 | 240.00 | 240.00 | 200.00 | 130.00 | 80.00 | 40.00 |    
    '''
    bmp5.set_odr(bmp5.eODR_05_HZ)
    bmp5.set_osr(bmp5.eOVERSAMPLING_16X, bmp5.eOVERSAMPLING_16X)
    '''!
      @brief Configures IIR filter coefficients
      @param iir_t Temperature IIR filter (see: eIIRFilter_t)
      @param iir_p Pressure IIR filter (see: eIIRFilter_t)
      @n Available coefficients:
      @n - eIIR_FILTER_BYPASS:   Bypass filter
      @n - eIIR_FILTER_COEFF_1:  1st order filter
      @n - eIIR_FILTER_COEFF_3:  3rd order filter
      @n - eIIR_FILTER_COEFF_7:  7th order filter
      @n - eIIR_FILTER_COEFF_15: 15th order filter
      @n - eIIR_FILTER_COEFF_31: 31st order filter
      @n - eIIR_FILTER_COEFF_63: 63rd order filter
      @n - eIIR_FILTER_COEFF_127:127th order filter
    '''
    bmp5.config_iir(bmp5.eIIR_FILTER_COEFF_15, bmp5.eIIR_FILTER_COEFF_15)

    '''!
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
    '''
    
    bmp5.config_interrupt(int_mode=bmp5.eINT_MODE_PULSED,
                            int_pol=bmp5.eINT_POL_ACTIVE_HIGH,
                            int_od=bmp5.eINT_OD_PUSH_PULL)
    '''!
      @brief Enables specific interrupt sources
      @param source Bitmask of triggers
      @n Available sources:
      @n - eINT_DATA_DRDY:    Data ready interrupt
      @n - eINT_FIFO_FULL:    FIFO full interrupt
      @n - eINT_FIFO_THRES:   FIFO threshold interrupt
      @n - eINT_PRESSURE_OOR: Pressure out-of-range interrupt
    '''
    bmp5.set_int_source(bmp5.eINT_DATA_DRDY)

    '''!
      @brief Configures sensor power/measurement mode
      @param mode Operation mode
      @n Available modes:
      @n - ePOWERMODE_STANDBY:       Standby mode
      @n - ePOWERMODE_NORMAL:        Normal measurement mode
      @n - ePOWERMODE_FORCED:        Single-shot measurement
      @n - ePOWERMODE_CONTINOUS:     Continuous measurement
      @n - ePOWERMODE_DEEP_STANDBY:  Deep standby mode
    '''
    bmp5.set_measure_mode(bmp5.ePOWERMODE_NORMAL)

def loop():
    global interrupt_flag
    if interrupt_flag:
        interrupt_flag = False
        print("temperature : %.2f (C)" % (bmp5.get_temperature()))
        print("Pressure : %.2f (Pa)" % (bmp5.get_pressure()))
        print("Altitude : %.2f (M)" % (bmp5.get_altitude()))
        print("")

if __name__ == "__main__":
    setup()
    while True:
        loop()