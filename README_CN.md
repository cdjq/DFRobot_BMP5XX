# DFRobot_BMP5XX

* [English Version](./README.md)

这是一个BMP5XX的库，功能是读取温度和压力。BMP(585/581)是一款基于可靠传感原理的压力和温度测量数字传感器。

![正反面svg效果图]()


## 产品链接（链接到英文商城）
   
## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述

* 该库支持BMP585/581传感器。
* 该库支持读取温度和压力。
* 该库支持设置传感器的工作模式。
* 该库支持设置传感器的输出数据率。
* 该库支持设置传感器的过采样率。
* 该库支持设置传感器的IIR滤波器系数。
* 该库支持设置传感器的FIFO操作。
* 该库支持设置传感器的中断行为。
* 该库支持设置传感器的压力超限检测。
* 该库支持读取传感器的中断状态。

## 库安装

使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 方法

```C++
/**
 * @fn begin
 * @brief 初始化传感器硬件接口
 * @return true 初始化成功, false 初始化失败
 */
bool begin(void);

/**
 * @fn setODR
 * @brief 配置传感器输出数据率(ODR)
 * @param odr 数据率选择
 * @n 可选数据率:
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
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t setODR(eODR_t odr);

/**
 * @fn setOSR
 * @brief 设置温度/压力过采样率
 * @param osr_t 温度过采样率
 * @param osr_p 压力过采样率
 * @n 支持的值:
 * @n - eOVERSAMPLING_1X:   1倍过采样
 * @n - eOVERSAMPLING_2X:   2倍过采样
 * @n - eOVERSAMPLING_4X:   4倍过采样
 * @n - eOVERSAMPLING_8X:   8倍过采样
 * @n - eOVERSAMPLING_16X:  16倍过采样
 * @n - eOVERSAMPLING_32X:  32倍过采样
 * @n - eOVERSAMPLING_64X:  64倍过采样
 * @n - eOVERSAMPLING_128X: 128倍过采样
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t setOSR(eOverSampling_t osr_t, eOverSampling_t osr_p);

/**
 * @fn setMeasureMode
 * @brief 配置传感器工作模式
 * @param mode 操作模式
 * @n 可选模式:
 * @n - ePOWERMODE_STANDBY:      待机模式
 * @n - ePOWERMODE_NORMAL:       正常测量模式
 * @n - ePOWERMODE_FORCED:       单次测量模式
 * @n - ePOWERMODE_CONTINOUS:    连续测量模式
 * @n - ePOWERMODE_DEEP_STANDBY: 深度待机模式
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t setMeasureMode(ePowerMode_t mode);

/**
 * @fn reset
 * @brief 执行传感器硬件复位
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t reset(void);

/**
 * @fn getTemperature
 * @brief 读取校准后的温度数据
 * @return float 摄氏温度值(°C)
 */
float getTemperature(void);

/**
 * @fn getPressure
 * @brief 读取校准后的压力数据
 * @return float 压力值(帕斯卡Pa)
 */
float getPressure(void);

/**
 * @fn getAltitude
 * @brief 基于气压值计算海拔高度
 * @note 使用公式:
 * @n 海拔 = (1 - (P/101325)^0.190284) * 44307.7
 * @n 其中 P = 当前气压值(Pa)
 * @return float 海拔高度(米)
 */
float getAltitude(void);

/**
 * @fn configIIR
 * @brief 配置IIR滤波器系数
 * @param iir_t 温度IIR滤波器
 * @param iir_p 压力IIR滤波器
 * @n 可选系数:
 * @n - eIIR_FILTER_BYPASS:  旁路滤波器
 * @n - eIIR_FILTER_COEFF_1:  1阶滤波器
 * @n - eIIR_FILTER_COEFF_3:  3阶滤波器
 * @n - eIIR_FILTER_COEFF_7:  7阶滤波器
 * @n - eIIR_FILTER_COEFF_15: 15阶滤波器
 * @n - eIIR_FILTER_COEFF_31: 31阶滤波器
 * @n - eIIR_FILTER_COEFF_63: 63阶滤波器
 * @n - eIIR_FILTER_COEFF_127:127阶滤波器
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t configIIR(eIIRFilter_t iir_t, eIIRFilter_t iir_p);

/**
 * @fn configFIFO
 * @brief 配置FIFO操作参数
 * @param frame_sel 数据类型选择
 * @n 可选类型:
 * @n - eFIFO_NOT_ENABLED:    禁用FIFO
 * @n - eFIFO_TEMPERATURE_DATA: 仅温度数据
 * @n - eFIFO_PRESSURE_DATA:    仅压力数据
 * @n - eFIFO_PRESS_TEMP_DATA:  压力和温度数据
 * 
 * @param dec_sel 降采样比率
 * @n 可选比率:
 * @n - eFIFO_NO_DOWNSAMPLING: 无降采样
 * @n - eFIFO_DOWNSAMPLING_2X:   2倍降采样
 * @n - eFIFO_DOWNSAMPLING_4X:   4倍降采样
 * @n - eFIFO_DOWNSAMPLING_8X:   8倍降采样
 * @n - eFIFO_DOWNSAMPLING_16X:  16倍降采样
 * @n - eFIFO_DOWNSAMPLING_32X:  32倍降采样
 * @n - eFIFO_DOWNSAMPLING_64X:  64倍降采样
 * @n - eFIFO_DOWNSAMPLING_128X: 128倍降采样
 * 
 * @param mode FIFO工作模式
 * @n 可选模式:
 * @n - eFIFO_STREAM_TO_FIFO_MODE: 持续流模式
 * @n - eFIFO_STOP_ON_FULL_MODE:   满时停止模式
 * 
 * @param threshold 触发阈值(0=禁用, 1-31=帧数)
 * @n - 0x0F: 压力和温度数据模式的最大帧数
 * @n - 0x1F: 压力或温度数据模式的最大帧数
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t configFIFO(eFIFOFrameSel_t frame_sel, eFIFODecSel_t dec_sel,
                   eFIFOMode_t mode, uint8_t threshold);

/**
 * @fn getFIFOCount
 * @brief 获取FIFO中当前数据帧数量
 * @return uint8_t 存储的数据帧数量(0-31)
 */
uint8_t getFIFOCount(void);

/**
 * @fn configInterrupt
 * @brief 配置中断行为
 * @param int_mode 触发模式
 * @n 可选模式:
 * @n - eINT_MODE_PULSED: 脉冲模式
 * @n - eINT_MODE_LATCHED: 锁存模式
 * 
 * @param int_pol 信号极性
 * @n 可选极性:
 * @n - eINT_POL_ACTIVE_LOW:  低电平有效
 * @n - eINT_POL_ACTIVE_HIGH: 高电平有效
 * 
 * @param int_od 输出驱动类型
 * @n 可选类型:
 * @n - eINT_OD_PUSH_PULL:  推挽输出
 * @n - eINT_OD_OPEN_DRAIN: 开漏输出
 * 
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t configInterrupt(eIntMode_t int_mode, eIntPolarity_t int_pol,
                        eIntOpenDrain_t int_od);

/**
 * @fn setIntSource
 * @brief 启用特定中断源
 * @param source 中断源位掩码
 * @n 可选中断源:
 * @n - eINT_DATA_DRDY:    数据就绪中断
 * @n - eINT_FIFO_FULL:    FIFO满中断
 * @n - eINT_FIFO_THRES:   FIFO阈值中断
 * @n - eINT_PRESSURE_OOR: 压力超限中断
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t setIntSource(uint8_t source);

/**
 * @fn getIntStatus
 * @brief 读取当前中断状态标志
 * @return uint16_t 活动中断位掩码
 * @n 可能的状态标志:
 * @n - eINT_STATUS_DRDY: 数据就绪
 * @n - eINT_STATUS_FIFO_FULL: FIFO满
 * @n - eINT_STATUS_FIFO_THRES: FIFO阈值达到
 * @n - eINT_STATUS_PRESSURE_OOR: 压力超限
 * @n - eINT_STATUS_POR_SOFTRESET_COMPLETE: 复位完成
 */
uint16_t getIntStatus(void);

/**
 * @fn setOORPress
 * @brief 配置压力超限检测
 * @param oor 压力阈值(0x00000-0x1FFFF)
 * @param range 迟滞范围(0-255)
 * @n 配置的范围为 (oor - range, oor + range)
 * @param cnt_lim 触发持续计数
 * @n 可选计数设置:
 * @n - eOOR_COUNT_LIMIT_1:  1次计数
 * @n - eOOR_COUNT_LIMIT_3:  3次计数
 * @n - eOOR_COUNT_LIMIT_7:  7次计数
 * @n - eOOR_COUNT_LIMIT_15: 15次计数
 * @return uint8_t 0=成功, 1=错误
 */
uint8_t setOORPress(uint32_t oor, uint8_t range, eOORCountLimit_t cnt_lim);

```

## 兼容性

| MCU                | Work Well | Work Wrong | Untested | Remarks |
| ------------------ |:---------:|:----------:|:--------:| ------- |
| Arduino uno        | √         |            |          |         |
| FireBeetle esp32   | √         |            |          |         |
| FireBeetle esp8266 | √         |            |          |         |
| FireBeetle m0      | √         |            |          |         |
| Leonardo           | √         |            |          |         |
| Microbit           | √         |            |          |         |
| Arduino MEGA2560   | √         |            |          |         |

## 历史

- Data 2025-06-06
- Version V1.0.0

## 创作者

Written by(yuanlong.yu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
