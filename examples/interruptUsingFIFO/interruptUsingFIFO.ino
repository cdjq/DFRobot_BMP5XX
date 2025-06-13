/**
 * @file  interruptUsingFIFO.ino
 * @brief  Acquire temperature/pressure data via FIFO threshold or FIFO full interrupts
 * @details  Demonstrate data acquisition from BMP58X using:
 *           - FIFO threshold interrupt (when FIFO reaches configured frame count)
 *           - FIFO full interrupt (when FIFO buffer is completely filled)
 *           - Data is read from FIFO buffer upon interrupt trigger
 * @copyright  Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license    The MIT License (MIT)
 * @author     yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version    V1.0.0
 * @date       2025-06-06
 * @url        https://github.com/DFRobot/DFRobot_BMP5XX
 */

#include "DFRobot_BMP5XX.h"
#include "DFRobot_RTU.h"

// Please choose your communication method below:
// #define BMP5_COMM_UART
#define BMP5_COMM_I2C
// #define BMP5_COMM_SPI

// Configure the interrupt mode you need to use:
// Opening the macro below is a latch interrupt, otherwise it is a pulse interrupt
// #define BMP5_INT_MODE_LATCHED

/** If there is no need to eliminate the absolute measurement error, please annotate the following line */
// #define CALIBRATE_ABSOLUTE_DIFFERENCE

const uint8_t ADDR = 0x47;

#if defined(BMP5_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
#endif
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
DFRobot_BMP5XX_UART bmp5(&mySerial, 9600, ADDR);
#elif defined(ESP32)
DFRobot_BMP5XX_UART bmp5(&Serial1, 9600, ADDR, /*rx*/ D2, /*tx*/ D3);
#else
DFRobot_BMP5XX_UART bmp5(&Serial1, 9600, ADDR);
#endif
#elif defined(BMP5_COMM_I2C)
DFRobot_BMP5XX_I2C bmp5(&Wire, ADDR);
#elif defined(BMP5_COMM_SPI)
DFRobot_BMP5XX_SPI bmp5(&SPI, 10);
#else
#error
#endif

volatile uint8_t flag = 0;
void interrupt() {
  if (flag == 0) {
    flag = 1;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Start..");
  while (!bmp5.begin()) {
    Serial.println("Sensor init fail!");
    delay(1000);
  }

  /**
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
   */
  bmp5.configFIFO(bmp5.eFIFO_PRESS_TEMP_DATA, bmp5.eFIFO_NO_DOWNSAMPLING, bmp5.eFIFO_STREAM_TO_FIFO_MODE, 0x02);


  /**
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
   */
#if defined(BMP5_INT_MODE_LATCHED)
  bmp5.configInterrupt(bmp5.eINT_MODE_LATCHED, bmp5.eINT_POL_ACTIVE_HIGH, bmp5.eINT_OD_PUSH_PULL);
#else
  bmp5.configInterrupt(bmp5.eINT_MODE_PULSED, bmp5.eINT_POL_ACTIVE_HIGH, bmp5.eINT_OD_PUSH_PULL);
#endif

  /**
   * @brief Enables specific interrupt sources
   * @param source Bitmask of triggers
   * @n Available sources:
   * @n - eINT_DATA_DRDY:    Data ready interrupt
   * @n - eINT_FIFO_FULL:    FIFO full interrupt
   * @n - eINT_FIFO_THRES:   FIFO threshold interrupt
   * @n - eINT_PRESSURE_OOR: Pressure out-of-range interrupt
   */
  // bmp5.setIntSource(bmp5.eINT_FIFO_FULL); // 

  bmp5.setIntSource(bmp5.eINT_FIFO_THRES);

  #if defined(CALIBRATE_ABSOLUTE_DIFFERENCE)
  /**
   * Calibrate the sensor according to the current altitude
   * In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). 
   * Please change to the local altitude when using it.
   * If this interface is not called, the measurement data will not eliminate the absolute difference.
   */
    bmp5.calibratedAbsoluteDifference(540.0);
  #endif

  /**
   * @brief Configures sensor power/measurement mode
   * @param mode Operation mode (see: ePowerMode_t)
   * @n Available modes:
   * @n - ePOWERMODE_STANDBY:       Standby mode
   * @n - ePOWERMODE_NORMAL:        Normal measurement mode
   * @n - ePOWERMODE_FORCED:        Single-shot measurement
   * @n - ePOWERMODE_CONTINOUS:     Continuous measurement
   * @n - ePOWERMODE_DEEP_STANDBY:  Deep standby mode
   */
  bmp5.setMeasureMode(bmp5.ePOWERMODE_NORMAL);

#if defined(ESP32) || defined(ESP8266)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  attachInterrupt(digitalPinToInterrupt(D6) /* Query the interrupt number of the D6 pin */, interrupt, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 5 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  attachInterrupt(digitalPinToInterrupt(5) /* Query the interrupt number of the 5 pin */, interrupt, RISING);
#else
  /* The Correspondence Table of AVR Series Arduino Interrupt Pins And Terminal Numbers
     * ---------------------------------------------------------------------------------------
     * |                                        |  DigitalPin  | 2  | 3  |                   |
     * |    Uno, Nano, Mini, other 328-based    |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  |                   |
     * |-------------------------------------------------------------------------------------|
     * |                                        |    Pin       | 2  | 3  | 21 | 20 | 19 | 18 |
     * |               Mega2560                 |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  | 5  |
     * |-------------------------------------------------------------------------------------|
     * |                                        |    Pin       | 3  | 2  | 0  | 1  | 7  |    |
     * |    Leonardo, other 32u4-based          |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  |    |
     * |--------------------------------------------------------------------------------------
     * ---------------------------------------------------------------------------------------------------------------------------------------------
     *                      The Correspondence Table of micro:bit Interrupt Pins And Terminal Numbers
     * ---------------------------------------------------------------------------------------------------------------------------------------------
     * |             micro:bit                       | DigitalPin |P0-P20 can be used as an external interrupt                                     |
     * |  (When using as an external interrupt,      |---------------------------------------------------------------------------------------------|
     * |no need to set it to input mode with pinMode)|Interrupt No|Interrupt number is a pin digital value, such as P0 interrupt number 0, P1 is 1 |
     * |-------------------------------------------------------------------------------------------------------------------------------------------|
     */
  attachInterrupt(/*Interrupt No*/ 0, interrupt, RISING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                           // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif
  
}
void loop() {
#if defined(BMP5_INT_MODE_LATCHED)
  bmp5.getIntStatus();
#endif
  if (flag == 1) {
    DFRobot_BMP5XX::sFIFOData_t data;
    data = bmp5.getFIFOData();
    Serial.print("FIFO len: ");
    Serial.println(data.len);
    for(int i = 0; i < data.len; ++i){
      Serial.print("temp: ");
      Serial.print(data.temperature[i]);
      Serial.print(" (C) pressure: ");
      Serial.print(data.pressure[i]);
      Serial.println(" (Pa)");
    }
    flag = 0;
  }
}