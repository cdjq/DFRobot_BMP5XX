# -*- coding: utf-8 -*
'''!
  @file  interrupt_data_drdy.py
  @brief  Get the temperature and pressure data of the BMP58X through interrupts
  @details  Obtain BMP58X data by using data in  ready interrupt.
  @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
  @version     V1.0.0
  @date        2025-06-06
  @url         https://github.com/DFRobot/DFRobot_BMP5XX
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

# Configure the interrupt mode you need to use:
# Using Ture is a latch interrupt, otherwise it is a pulse interrupt
BMP5_INT_MODE_LATCHED = True

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

def drdy_callback(channel):
    global interrupt_flag
    interrupt_flag = True

gpio_interrupt = 27
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_interrupt, GPIO.IN)
GPIO.add_event_detect(gpio_interrupt, GPIO.RISING, callback=drdy_callback)

def setup():
    while not bmp5.begin():
        print("sensor init error,please check connect!")
        time.sleep(1)
    
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
    if BMP5_INT_MODE_LATCHED:
        bmp5.config_interrupt(int_mode=bmp5.eINT_MODE_LATCHED,
                              int_pol=bmp5.eINT_POL_ACTIVE_HIGH,
                              int_od=bmp5.eINT_OD_PUSH_PULL)
    else:
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
    if BMP5_INT_MODE_LATCHED:
        bmp5.get_int_status()
        
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