/*
 * tmp006.c
 *
 *  Created on: Sep 19, 2015
 *      Author: Edward
 */

#include "tmp006.h"

/* Return 1 if TMP006 is successfully configured
 * Otherwise return 0
 */
uint8_t config_TMP006(I2C_HandleTypeDef *hi2c){

	uint8_t data[3] = {0};
	data[0] = TMP006_CONFIG;
	data[1] = (TMP006_CFG_8SAMPLE | TMP006_CFG_MODEON | TMP006_CFG_DRDYEN) >> 8;
	data[2] = TMP006_CFG_8SAMPLE | TMP006_CFG_MODEON | TMP006_CFG_DRDYEN;

	/* configure TMP006 in accordance to user guide */
	if(HAL_I2C_Master_Transmit(hi2c, (uint16_t)(TMP006_ADDR_NORMAL << 1), data, 3, (uint32_t)0xFFFF)!= HAL_OK)
	    {
        	return 0;
	    }

	return 1;
}

/* Read temperature value */
uint16_t read_TMP006(I2C_HandleTypeDef *hi2c)
{
	uint8_t data[2] = {0};
	uint16_t raw_temp = 0;
	double temp = 0;

	/* read from 0x02, memory address in TMP006 */
	if(HAL_I2C_Mem_Read(hi2c, (uint16_t)(TMP006_ADDR_NORMAL << 1), (uint16_t)TMP006_TAMB, 1, data, 2, (uint32_t)0xFFFF) == HAL_OK)
	{
		raw_temp = (data[0] << 8) | data[1];
		raw_temp = raw_temp >> 2;
		temp = raw_temp * 0.03125; // convert to celsius
	}

	return temp*100;
}

