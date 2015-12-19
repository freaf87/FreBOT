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
 * motor.h
 *
 *  Created on: Dec 19, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
 
*/

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

	 	 	 	 	 	 	 	 /***  Motors  ***/

/* Definition for TIMx clock resources */
#define MOTOR_TIMx                           			TIM9
#define MOTOR_TIMx_CLK_ENABLE()                			__HAL_RCC_TIM9_CLK_ENABLE()

/* Definition for MOTOR PWM Pins */
#define MOTOR_TIMx_CHANNEL_GPIO_PORT()       				__HAL_RCC_GPIOA_CLK_ENABLE()
#define MOTOR_GPIO_PIN_CHANNEL1              			GPIO_PIN_2
#define MOTOR_GPIO_PIN_CHANNEL2              			GPIO_PIN_3

/* Period Value  */
#define PERIOD_VALUE       									2799
/* Capture Compare 1 Value  */
#define PULSE_VALUE       									2799/2


#define MOTOR1_DIR_PIN                         GPIO_PIN_11
#define MOTOR1_DIR_GPIO_PORT                   GPIOD
#define MOTOR1_DIR_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define MOTOR1_DIR_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

#define MOTOR2_DIR_PIN                         GPIO_PIN_12
#define MOTOR2_DIR_GPIO_PORT                   GPIOD
#define MOTOR2_DIR_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define MOTOR2_DIR_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

#define BATTERIE_VOLTAGE  									12




#endif /* INC_MOTOR_H_ */
