/*
 * tmp006.h
 *
 * Created on: Sep 19, 2015
 *      Author: Edward
 *      Adapted from Sparkfun's Arduino Library
 *      https://github.com/sparkfun/TMP006-Temp_Sensor_Breakout
 *      https://learn.sparkfun.com/tutorials/tmp006-hookup-guide/talking-to-the-sensor*
 *
 * User Guide:
 * 		1. call config_TMP006(*I2C_STRUCTURE) in main()
 * 		2. call read_TMP006(*I2C_STRUCTURE) when reading temperature value
 */

#ifndef TMP006_H_
#define TMP006_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

// Configuration Settings
#define TMP006_CFG_RESET    0x8000
#define TMP006_CFG_MODEON   0x7000
#define TMP006_CFG_1SAMPLE  0x0000
#define TMP006_CFG_2SAMPLE  0x0200
#define TMP006_CFG_4SAMPLE  0x0400
#define TMP006_CFG_8SAMPLE  0x0600
#define TMP006_CFG_16SAMPLE 0x0800
#define TMP006_CFG_DRDYEN   0x0100
#define TMP006_CFG_DRDY     0x0080

#define TMP006_VOBJ  0x00
#define TMP006_TAMB 0x01
#define TMP006_CONFIG 0x02

#define TMP006_ADDR_NORMAL 0x40

uint8_t config_TMP006(I2C_HandleTypeDef *hi2c);
uint16_t read_TMP006(I2C_HandleTypeDef *hi2c);

#endif /* TMP006_H_ */
