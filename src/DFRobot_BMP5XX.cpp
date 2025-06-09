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

#include "DFRobot_BMP5XX.h"
#include "Arduino.h"
#include "SPI.h"

DFRobot_BMP5XX::DFRobot_BMP5XX() {}

DFRobot_BMP5XX::~DFRobot_BMP5XX() {}

bool DFRobot_BMP5XX::begin(void) {
  uint16_t data = 0;
  reset();
  if (readInputReg(REG_I_CHIP_ID, &data, 1) != RET_CODE_OK ||
      (data != BMP581_CHIP_ID && data != BMP585_CHIP_ID)) {
    return false;
  }
  enablePressure(eENABLE);
  return true;
}

uint8_t DFRobot_BMP5XX::setODR(eODR_t odr) {
  uint16_t data;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, ODR, odr);
  writeHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP5XX::setOSR(eOverSampling_t osr_t, eOverSampling_t osr_p) {
  uint16_t data;
  readHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, OSR_T, osr_t);
  BMP5_REG_SET_BITS(data, OSR_P, osr_p);
  writeHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP5XX::enablePressure(eEnable_t enable) {
  uint16_t data = 0;
  readHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, PRESS_EN, enable);
  writeHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP5XX::setMeasureMode(ePowerMode_t mode) {
  ePowerMode_t currMode;
  uint16_t data = 0;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  currMode = BMP5_REG_GET_BITS(data, PWR_MODE, ePowerMode_t);
  if (currMode != ePOWERMODE_STANDBY) {
    setPowerMode(ePOWERMODE_STANDBY);
    delayMicroseconds(2500);
  }
  switch (mode) {
  case ePOWERMODE_DEEP_STANDBY:
    setDeepStandbyMode(eDEEP_ENABLE);
    break;
  case ePOWERMODE_STANDBY:
    break;
  case ePOWERMODE_CONTINOUS:
  case ePOWERMODE_FORCED:
  case ePOWERMODE_NORMAL:
    setPowerMode(mode);
    break;
  default:
    return RET_CODE_ERROR;
  }
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP5XX::reset(void) {
  uint16_t data = SOFT_RESET_CMD;
  writeHoldingReg(REG_H_CMD, &data, 1);
  delayMicroseconds(2000);

  readInputReg(REG_I_CHIP_ID, &data, 1);

  uint16_t intStatus = getIntStatus();
  if (intStatus & eINT_STATUS_POR_SOFTRESET_COMPLETE) {
    return RET_CODE_OK;
  }
  return RET_CODE_ERROR;
}

float DFRobot_BMP5XX::getTemperature(void) {
  uint16_t data[3] = {0};
  readInputReg(REG_I_TEMP_DATA_XLSB, data, 3);
  int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) |
                    ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
  // (MSB LSB XLSB) / 2^16;
  return (float)tmpData / 65536.0;
}

float DFRobot_BMP5XX::getPressure(void) {
  uint16_t data[3] = {0};
  readInputReg(REG_I_PRESS_DATA_XLSB, data, 3);
  int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) |
                      ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
  // (MSB LSB XLSB) / 2^6;
  return (float)pressData / 64;
}

float DFRobot_BMP5XX::getAltitude(void) {
  uint16_t data[6] = {0};
  readInputReg(REG_I_TEMP_DATA_XLSB, data, 6);
  int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) |
                    ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);

  int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[5])) << 16) |
                      ((uint32_t)(data[4]) << 8) | ((uint32_t)data[3]);

  return calculateAltitude((float)tmpData / 65536, (float)pressData / 64);
}

uint8_t DFRobot_BMP5XX::configIIR(eIIRFilter_t iir_t, eIIRFilter_t iir_p) {
  ePowerMode_t currMode = getPowerMode();
  setMeasureMode(ePOWERMODE_STANDBY);
  uint16_t data[2] = {0};
  readHoldingReg(REG_H_DSP_CONFIG, data, 2);
  BMP5_REG_SET_BITS(data[0], SHDW_SEL_IIR_T, eENABLE);
  BMP5_REG_SET_BITS(data[0], SHDW_SEL_IIR_P, eENABLE);
  BMP5_REG_SET_BITS(data[0], IIR_FLUSH_FORCED_EN, eENABLE);

  BMP5_REG_SET_BITS(data[1], SET_IIR_T, iir_t);
  BMP5_REG_SET_BITS(data[1], SET_IIR_P, iir_p);

  writeHoldingReg(REG_H_DSP_CONFIG, data, 2);
  if (currMode != ePOWERMODE_STANDBY && currMode != ePOWERMODE_DEEP_STANDBY) {
    return setMeasureMode(currMode);
  }
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP5XX::configFIFO(eFIFOFrameSel_t frame_sel,
                                   eFIFODecSel_t dec_sel, eFIFOMode_t mode,
                                   uint8_t threshold) {
  ePowerMode_t currMode = getPowerMode();
  setMeasureMode(ePOWERMODE_STANDBY);
  uint16_t data = 0;
  uint16_t ret = RET_CODE_OK;
  readHoldingReg(REG_H_DSP_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, FIFO_SEL_IIR_T, eENABLE);
  BMP5_REG_SET_BITS(data, FIFO_SEL_IIR_P, eENABLE);
  writeHoldingReg(REG_H_DSP_CONFIG, &data, 1);

  readHoldingReg(REG_H_FIFO_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, FIFO_MODE, mode);
  ret = setFIFOThreshold(&data, frame_sel, threshold);
  if (ret == RET_CODE_OK) {
    writeHoldingReg(REG_H_FIFO_CONFIG, &data, 1);

    readHoldingReg(REG_H_FIFO_SEL, &data, 1);
    BMP5_REG_SET_BITS(data, FIFO_FRAME_SEL, frame_sel);
    BMP5_REG_SET_BITS(data, FIFO_DEC_SEL, dec_sel);
    writeHoldingReg(REG_H_FIFO_SEL, &data, 1);
  }
  if (currMode != ePOWERMODE_STANDBY && currMode != ePOWERMODE_DEEP_STANDBY) {
    return setMeasureMode(currMode);
  }
  return ret;
}

uint8_t DFRobot_BMP5XX::getFIFOCount(void) {
  uint16_t data = 0;
  readInputReg(REG_I_FIFO_COUNT, &data, 1);
  uint8_t count = BMP5_REG_GET_BITS(data, FIFO_COUNT, uint8_t);
  return count;
}

DFRobot_BMP5XX::sFIFOData_t DFRobot_BMP5XX::getFIFOData(void) {
  sFIFOData_t fifoData = { 0,{0},{0}};
  uint16_t regData = 0;
  readHoldingReg(REG_H_FIFO_SEL, &regData, 1);
  uint8_t fifo_frame_sel = BMP5_REG_GET_BITS(regData, FIFO_FRAME_SEL, uint8_t);
  uint8_t count = getFIFOCount();
  if (fifo_frame_sel == eFIFO_TEMPERATURE_DATA) {
    for (uint8_t i = 0; i < count; i++) {
      uint16_t data[3] = {0};
      readInputReg(REG_I_FIFO_DATA, data, 3);
      int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) |
                        ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
      fifoData.temperature[i] = (float)tmpData / 65536;
    }
  } else if (fifo_frame_sel == eFIFO_PRESSURE_DATA) {
    for (uint8_t i = 0; i < count; i++) {
      uint16_t data[3] = {0};
      readInputReg(REG_I_FIFO_DATA, data, 3);
      int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) |
                          ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
      fifoData.pressure[i] = (float)pressData / 64;
    }
  } else if (fifo_frame_sel == eFIFO_PRESS_TEMP_DATA) {
    for (uint8_t i = 0; i < count; i++) {
      uint16_t data[6] = {0};
      readInputReg(REG_I_FIFO_DATA, data, 6);
      int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) |
                        ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
      fifoData.temperature[i] = (float)tmpData / 65536;

      int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[5])) << 16) |
                          ((uint32_t)(data[4]) << 8) | ((uint32_t)data[3]);
      fifoData.pressure[i] = (float)pressData / 64;
    }
  }
  fifoData.len = count;
  return fifoData;
}

uint8_t DFRobot_BMP5XX::configInterrupt(eIntMode_t int_mode,
                                        eIntPolarity_t int_pol,
                                        eIntOpenDrain_t int_od) {
  uint16_t data = 0, source_data = 0;
  readHoldingReg(REG_H_INT_SOURCE, &source_data, 1);
  writeHoldingReg(REG_H_INT_SOURCE, &data, 1);
  readHoldingReg(REG_H_INT_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, INT_MODE, int_mode);
  BMP5_REG_SET_BITS(data, INT_POL, int_pol);
  BMP5_REG_SET_BITS(data, INT_OD, int_od);
  BMP5_REG_SET_BITS(data, INT_EN, eINT_ENABLED);
  writeHoldingReg(REG_H_INT_CONFIG, &data, 1);
  writeHoldingReg(REG_H_INT_SOURCE, &source_data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP5XX::setIntSource(uint8_t source) {
  const uint8_t VALID_INT_MASK =
      eINT_DATA_DRDY | eINT_FIFO_FULL | eINT_FIFO_THRES | eINT_PRESSURE_OOR;
  source &= VALID_INT_MASK;
  uint16_t data = source;
  writeHoldingReg(REG_H_INT_SOURCE, &data, 1);
  return RET_CODE_OK;
}

uint16_t DFRobot_BMP5XX::getIntStatus(void) {
  uint16_t data = 0;
  readInputReg(REG_I_INT_STATUS, &data, 1);
  // DBG(REG_I_INT_STATUS);
  // DBG(data);
  return data;
}

uint8_t DFRobot_BMP5XX::setOORPress(uint32_t oor, uint8_t range,
                                    eOORCountLimit_t cnt_lim) {
  ePowerMode_t currMode = getPowerMode();
  setMeasureMode(ePOWERMODE_STANDBY);

  uint16_t data[4] = {0};
  readHoldingReg(REG_H_OOR_THR_P_LSB, data, 4);
  data[0] = oor & 0xFF;
  data[1] = (oor >> 8) & 0xFF;
  BMP5_REG_SET_BITS(data[3], OOR_THR_P_16, ((oor >> 16) & 0x01));

  data[2] = range;

  BMP5_REG_SET_BITS(data[3], CNT_LIM, cnt_lim);
  writeHoldingReg(REG_H_OOR_THR_P_LSB, data, 4);
  if (currMode != ePOWERMODE_STANDBY && currMode != ePOWERMODE_DEEP_STANDBY) {
    return setMeasureMode(currMode);
  }
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP5XX::setDeepStandbyMode(eDeepEnable_t deepMode) {
  uint16_t data = 0;
  uint8_t ret = RET_CODE_OK;
  if (deepMode == eDEEP_ENABLE) {
    readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
    BMP5_REG_SET_BITS(data, DEEP_DIS, eDEEP_ENABLE);
    BMP5_REG_SET_BITS(data, ODR, eODR_01_HZ);
    writeHoldingReg(REG_H_ODR_CONFIG, &data, 1);

    readHoldingReg(REG_H_FIFO_SEL, &data, 1);
    BMP5_REG_SET_BITS(data, FIFO_FRAME_SEL, eFIFO_NOT_ENABLED);
    writeHoldingReg(REG_H_FIFO_SEL, &data, 1);

    readHoldingReg(REG_H_DSP_IIR, &data, 1);
    BMP5_REG_SET_BITS(data, SET_IIR_T, eIIR_FILTER_BYPASS);
    BMP5_REG_SET_BITS(data, SET_IIR_P, eIIR_FILTER_BYPASS);
    writeHoldingReg(REG_H_DSP_IIR, &data, 1);
    ret = RET_CODE_OK;
  } else {
    ePowerMode_t currMode;
    currMode = getPowerMode();
    if (currMode == ePOWERMODE_DEEP_STANDBY) {
      return setMeasureMode(ePOWERMODE_STANDBY);
    }
  }
  return ret;
}

uint8_t DFRobot_BMP5XX::setPowerMode(ePowerMode_t mode) {
  uint16_t data = 0;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, DEEP_DIS, DEEP_DISABLE);
  BMP5_REG_SET_BITS(data, PWR_MODE, mode);
  writeHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  return 0;
}

DFRobot_BMP5XX::ePowerMode_t DFRobot_BMP5XX::getPowerMode(void) {
  ePowerMode_t currMode;
  uint16_t data = 0, deep_dis = 0;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  currMode = BMP5_REG_GET_BITS(data, PWR_MODE, ePowerMode_t);
  deep_dis = BMP5_REG_GET_BITS(data, DEEP_DIS, uint16_t);
  if (currMode == ePOWERMODE_STANDBY && deep_dis == DEEP_ENABLE) {
    return verifyDeepStandbyMode();
  }
  return currMode;
}

DFRobot_BMP5XX::ePowerMode_t DFRobot_BMP5XX::verifyDeepStandbyMode(void) {
  uint16_t data = 0;
  ePowerMode_t mode = ePOWERMODE_STANDBY;
  readHoldingReg(REG_H_FIFO_SEL, &data, 1);
  uint8_t fifo_frame_sel = BMP5_REG_GET_BITS(data, FIFO_FRAME_SEL, uint8_t);
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  eODR_t odr = BMP5_REG_GET_BITS(data, ODR, eODR_t);
  readHoldingReg(REG_H_DSP_IIR, &data, 1);
  eIIRFilter_t iir_t = BMP5_REG_GET_BITS(data, SET_IIR_T, eIIRFilter_t);
  eIIRFilter_t iir_p = BMP5_REG_GET_BITS(data, SET_IIR_P, eIIRFilter_t);
  if (odr > eODR_05_HZ && fifo_frame_sel == DISABLE &&
      iir_t == eIIR_FILTER_BYPASS && iir_p == eIIR_FILTER_BYPASS) {
    mode = ePOWERMODE_DEEP_STANDBY;
  }
  return mode;
}

uint8_t DFRobot_BMP5XX::setFIFOThreshold(uint16_t *data,
                                         eFIFOFrameSel_t frame_sel,
                                         uint8_t threshold) {
  uint8_t ret = RET_CODE_ERROR;
  if (frame_sel == eFIFO_TEMPERATURE_DATA || frame_sel == eFIFO_PRESSURE_DATA) {
    if (threshold <= 0x1F) {
      BMP5_REG_SET_BITS(*data, FIFO_THRESHOLD, threshold);
      ret = RET_CODE_OK;
    }
  } else if (frame_sel == eFIFO_PRESS_TEMP_DATA) {
    if (threshold <= 0x0F) {
      BMP5_REG_SET_BITS(*data, FIFO_THRESHOLD, threshold);
      ret = RET_CODE_OK;
    }
  }
  return ret;
}

float DFRobot_BMP5XX::calculateAltitude(float temperature_c,
                                        float pressure_pa) {
  (void)temperature_c;
  return (1.0 - pow(pressure_pa / 101325, 0.190284)) * 44307.7;
}

uint8_t DFRobot_BMP5XX_I2C::writeHoldingReg(uint8_t reg, void *data,
                                            uint8_t len) {
  return writeReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR;
}

uint8_t DFRobot_BMP5XX_I2C::readHoldingReg(uint8_t reg, void *data,
                                           uint8_t len) {
  return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR;
}

uint8_t DFRobot_BMP5XX_I2C::readInputReg(uint8_t reg, void *data, uint8_t len) {
  return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR;
}

bool DFRobot_BMP5XX_I2C::readReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  uint8_t count = 0;
  _pWire->beginTransmission((uint8_t)_i2cAddr);
  _pWire->write(reg);
  _pWire->endTransmission();

  _pWire->requestFrom((uint8_t)_i2cAddr, len);
  while (_pWire->available()) {
    tempData[count++] = _pWire->read();
  }
  return count == len;
}

bool DFRobot_BMP5XX_I2C::writeReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  uint8_t regData[REG_DATA_LEN_MAX] = {0};
  for (uint8_t i = 0; i < len; ++i) {
    regData[i] = (uint8_t)(tempData[i] & 0xFF);
  }
  _pWire->beginTransmission((uint8_t)_i2cAddr);
  _pWire->write(reg);
  _pWire->write(regData, len);
  _pWire->endTransmission();
  delay(2);
  return true;
}

DFRobot_BMP5XX_I2C::DFRobot_BMP5XX_I2C(TwoWire *pWire, uint16_t addr) {
  _pWire = pWire;
  _i2cAddr = addr;
}

DFRobot_BMP5XX_I2C::~DFRobot_BMP5XX_I2C(void) {}

bool DFRobot_BMP5XX_I2C::begin(void) {
  _pWire->begin();
  return DFRobot_BMP5XX::begin();
}
uint8_t DFRobot_BMP5XX_SPI::writeHoldingReg(uint8_t reg, void *data,
                                            uint8_t len) {
  return writeReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR;
}
uint8_t DFRobot_BMP5XX_SPI::readHoldingReg(uint8_t reg, void *data,
                                           uint8_t len) {
  return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR;
}
uint8_t DFRobot_BMP5XX_SPI::readInputReg(uint8_t reg, void *data, uint8_t len) {
  return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR;
}
bool DFRobot_BMP5XX_SPI::readReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  _pSpi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _pSpi->transfer(reg | 0x80);
  while (len--) {
    *tempData = _pSpi->transfer(0x00);
    tempData++;
  }
  digitalWrite(_csPin, HIGH);
  _pSpi->endTransaction();
  return true;
}
bool DFRobot_BMP5XX_SPI::writeReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  uint8_t regData[REG_DATA_LEN_MAX] = {0};
  for (uint8_t i = 0; i < len; ++i) {
    regData[i] = (uint8_t)(tempData[i] & 0xFF);
  }
  _pSpi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _pSpi->transfer(reg & 0x7F);
  for (uint8_t i = 0; i < len; ++i) {
    _pSpi->transfer(regData[i]);
  }
  digitalWrite(_csPin, HIGH);
  _pSpi->endTransaction();
  // delay(2);
  return true;
}

DFRobot_BMP5XX_SPI::DFRobot_BMP5XX_SPI(SPIClass *pSpi, uint8_t csPin) {
  _pSpi = pSpi;
  _csPin = csPin;
}

DFRobot_BMP5XX_SPI::~DFRobot_BMP5XX_SPI(void) {}

bool DFRobot_BMP5XX_SPI::begin(void) {
  pinMode(_csPin, OUTPUT);
  _pSpi->begin();
  digitalWrite(_csPin, HIGH);
  return DFRobot_BMP5XX::begin();
}

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
DFRobot_BMP5XX_UART::DFRobot_BMP5XX_UART(SoftwareSerial *sSerial, uint32_t Baud,
                                         uint16_t addr)
    : DFRobot_RTU(sSerial) {
  _serial = sSerial;
  __baud = Baud;
  __addr = addr;
}
#else
DFRobot_BMP5XX_UART::DFRobot_BMP5XX_UART(HardwareSerial *hSerial, uint32_t Baud,
                                         uint16_t addr, uint8_t rxpin,
                                         uint8_t txpin)
    : DFRobot_RTU(hSerial) {
  _serial = hSerial;
  __baud = Baud;
  __rxpin = rxpin;
  __txpin = txpin;
  __addr = addr;
}
#endif
DFRobot_BMP5XX_UART::~DFRobot_BMP5XX_UART(void) {}

bool DFRobot_BMP5XX_UART::begin(void) {
#ifdef ESP32
  _serial->begin(__baud, SERIAL_8N1, __txpin, __rxpin);
  delay(100);
#elif defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  _serial->begin(__baud);
  delay(100);
#else
  _serial->begin(__baud);
#endif
  uint16_t data = 0;
  readInputReg(REG_I_REV_ID, &data, 1);
  return DFRobot_BMP5XX::begin();
}

uint8_t DFRobot_BMP5XX_UART::writeHoldingReg(uint8_t reg, void *data,
                                             uint8_t len) {
  return writeHoldingRegister(__addr, reg, static_cast<uint16_t *>(data), len);
}
uint8_t DFRobot_BMP5XX_UART::readHoldingReg(uint8_t reg, void *data,
                                            uint8_t len) {
  return readHoldingRegister(__addr, reg, static_cast<uint16_t *>(data), len);
}
uint8_t DFRobot_BMP5XX_UART::readInputReg(uint8_t reg, void *data,
                                          uint8_t len) {
  return readInputRegister(__addr, reg, static_cast<uint16_t *>(data), len);
}