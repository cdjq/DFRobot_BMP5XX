# -*- coding: utf-8 -*
'''!
  @file  get_temp_press.py
  @brief  Get the temperature and pressure data of the BMP58X
  @details  The temperature and pressure data of the BMP58X is obtained by calling the get_temperature() and get_pressure() functions respectively.
  @n  The altitude data is obtained by calling the get_altitude() function.
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

def setup():
    while not bmp5.begin():
        print("sensor init error ,please check connect!")
        time.sleep(1)
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
    print("temperature : %.2f (C)" % (bmp5.get_temperature()))

    print("Pressure : %.2f (Pa)" % (bmp5.get_pressure()))

    print("Altitude : %.2f (M)" % (bmp5.get_altitude()))
    
    print()
    time.sleep(0.5)

if __name__ == "__main__":
    setup()
    while True:
        loop()