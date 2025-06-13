/**
 * @file  setODR_OSR_IIR.ino
 * @brief  Configure ODR (Output Data Rate), OSR (Oversampling Rate) and IIR filter for BMP5XX sensor
 * @details  Demonstrate how to set key parameters of the sensor, including:
 *           - Configure ODR (output data rate)
 *           - Configure OSR (temperature and pressure oversampling rates)
 *           - Configure IIR filter coefficients
 *           - Enable data ready interrupt and read sensor data
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

/** If there is no need to eliminate the absolute measurement error, please annotate the following line */
#define CALIBRATE_ABSOLUTE_DIFFERENCE

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
   * Maximum nominal ODR setting per OSR settings in NORMAL mode
   * max ODR [Hz] represents the maximum output data rate (unit: Hertz).
   * OSR_T is a parameter related to the temperature oversampling rate, 
   * with values of 1, 2, 4, 8, 16, 32, 64, 128.
   * OSR_P is a parameter related to the pressure oversampling rate, 
   * with values of 1, 2, 4, 8, 16, 32, 64, 128.
   * The table content is as follows:
   * | max ODR [Hz] | OSR_T = 1 | OSR_T = 2 | OSR_T = 4 | OSR_T = 8 | OSR_T = 16 | OSR_T = 32 | OSR_T = 64 | OSR_T = 128 |
   * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
   * | OSR_P = 1 | 240.00 | 240.00 | 240.00 | 240.00 | 200.00 | 130.00 | 80.00 | 40.00 |
   * | OSR_P = 2 | 240.00 | 240.00 | 240.00 | 220.00 | 180.00 | 120.00 | 70.00 | 40.00 |
   * | OSR_P = 4 | 220.00 | 220.00 | 200.00 | 180.00 | 140.00 | 100.00 | 70.00 | 40.00 |
   * | OSR_P = 8 | 140.00 | 140.00 | 130.00 | 120.00 | 100.00 | 80.00 | 50.00 | 35.00 |
   * | OSR_P = 16 | 80.00 | 80.00 | 80.00 | 70.00 | 70.00 | 50.00 | 45.00 | 30.00 |
   * | OSR_P = 32 | 45.00 | 45.00 | 40.00 | 40.00 | 40.00 | 35.00 | 30.00 | 20.00 |
   * | OSR_P = 64 | 20.00 | 20.00 | 20.00 | 20.00 | 20.00 | 20.00 | 15.00 | 15.00 |
   * | OSR_P = 128 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 5.00 |
   */

  /**
   * Maximum nominal ODR setting per OSR settings in NORMAL mode 
   * for temperature only measurements
   * max ODR [Hz] represents the maximum output data rate (unit: Hertz).
   * OSR_T is a parameter related to the temperature oversampling rate, 
   * with values of 1, 2, 4, 8, 16, 32, 64, 128.
   * The table content is as follows:
   * | max ODR [Hz] | OSR_T = 1 | OSR_T = 2 | OSR_T = 4 | OSR_T = 8 | OSR_T = 16 | OSR_T = 32 | OSR_T = 64 | OSR_T = 128 |
   * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
   * | - | 240.00 | 240.00 | 240.00 | 240.00 | 200.00 | 130.00 | 80.00 | 40.00 |
   */

  bmp5.setODR(bmp5.eODR_05_HZ);
  bmp5.setOSR(bmp5.eOVERSAMPLING_16X, bmp5.eOVERSAMPLING_16X);

  /**
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
   */
  bmp5.configIIR(bmp5.eIIR_FILTER_COEFF_15, bmp5.eIIR_FILTER_COEFF_15);


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
  bmp5.configInterrupt(bmp5.eINT_MODE_PULSED, bmp5.eINT_POL_ACTIVE_HIGH, bmp5.eINT_OD_PUSH_PULL);

  /**
   * @brief Enables specific interrupt sources
   * @param source Bitmask of triggers
   * @n Available sources:
   * @n - eINT_DATA_DRDY:    Data ready interrupt
   * @n - eINT_FIFO_FULL:    FIFO full interrupt
   * @n - eINT_FIFO_THRES:   FIFO threshold interrupt
   * @n - eINT_PRESSURE_OOR: Pressure out-of-range interrupt
   */
  bmp5.setIntSource(bmp5.eINT_DATA_DRDY);

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
  // bmp5.setMeasureMode(bmp5.ePOWERMODE_CONTINOUS);


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
  if (flag == 1) {
    flag = 0;
    Serial.print("temp: ");
    Serial.print(bmp5.getTemperature());
    Serial.print(" (C)  press: ");
    Serial.print(bmp5.getPressure());
    Serial.print(" (Pa)  alt: ");
    Serial.print(bmp5.getAltitude());
    Serial.println(" (M)");
  }
}