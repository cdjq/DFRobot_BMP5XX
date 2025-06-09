/**
 * @file  getTempPress.ino
 * @brief  Get the temperature and pressure data of the BMP58X
 * @details  The temperature and pressure data of the BMP58X is obtained by calling the get_temperature() and get_pressure() functions respectively.
 * @n  The altitude data is obtained by calling the get_altitude() function.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version     V1.0.0
 * @date        2025-06-06
 * @url         https://github.com/DFRobot/DFRobot_BMP5XX
 */
#include "DFRobot_BMP5XX.h"
#include "DFRobot_RTU.h"

// #define BMP5_COMM_UART
#define BMP5_COMM_I2C
// #define BMP5_COMM_SPI

const uint8_t ADDR = 0x47;

#if defined(BMP5_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
  #if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  #include <SoftwareSerial.h>
  #endif
  #if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
    SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
    DFRobot_BMP5XX_UART bmp5(&mySerial, 9600, ADDR);
  #elif defined(ESP32)
    DFRobot_BMP5XX_UART bmp5(&Serial1, 9600, ADDR, /*rx*/ D2, /*tx*/ D3);
  #else
    DFRobot_BMP5XX_UART bmp5(&Serial1,9600, ADDR);
  #endif
#elif defined(BMP5_COMM_I2C)
  DFRobot_BMP5XX_I2C bmp5(&Wire, ADDR);
#elif defined(BMP5_COMM_SPI)
  DFRobot_BMP5XX_SPI bmp5(&SPI, 10);
#else
  #error
#endif

void setup() {
  Serial.begin(9600);
  while(!bmp5.begin()){
    Serial.println("Sensor init fail!");
    delay(1000);
  }
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
}

void loop() {
  delay(1000);
  Serial.print("temp: ");
  Serial.print(bmp5.getTemperature());
  Serial.print(" (C)  press: ");
  Serial.print(bmp5.getPressure());
  Serial.print(" (Pa)  alt: ");
  Serial.print(bmp5.getAltitude());
  Serial.println(" (M)");
}
