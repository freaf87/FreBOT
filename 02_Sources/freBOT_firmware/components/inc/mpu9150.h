/*
	Copyright 2015-2017 freaf87

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * mpl9150.h
 *
 *  Created on: Dec 12, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
 
*/

#ifndef INC_MPU9150_H_
#define INC_MPU9150_H_

#include "stm32f4xx_hal.h"
/****************************** Defines *******************************/
/***************************Globals *******************************************/
/***************************** Prototypes *****************************/
void MPU_INT_GPIO_Config(void);
void MPU9150_Timer_Init(void) ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void MPU9150_gyroDataReady(void);
void MPU9150_GetReadings(void);
void MPU9150_I2C_Master_Init(void);
int  MPU9150_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr);
int MPU9150_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr);
void MPU9150_Init(void);

void gyro_data_ready_cb(void);
void mdelay(unsigned long nTime);
int get_tick_count(unsigned long *count);


#endif /* INC_MPU9150_H_ */
