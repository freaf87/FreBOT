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

 * optional.h
 *
 *  Created on: Dec 6, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
 
*/

#ifndef INC_OPTIONAL_H_
#define INC_OPTIONAL_H_

#include "stm32f4xx_hal.h"

 	 	 	 	 	 	 	 /***  LEds  ***/
#define LEDn                             		2
typedef enum
{
		USERled		= 0,
		ERRORled	= 1
} Led_TypeDef;

#define USERled_PIN                         GPIO_PIN_1
#define USERled_GPIO_PORT                   GPIOE
#define USERled_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define USERled_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()

#define ERRORled_PIN                         GPIO_PIN_0
#define ERRORled_GPIO_PORT                   GPIOE
#define ERRORled_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define ERRORled_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)			do{	if((__INDEX__)== 0)  USERled_GPIO_CLK_ENABLE(); else\
																								if((__INDEX__)== 1)  ERRORled_GPIO_CLK_ENABLE();\
																							} while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)			do{	if((__INDEX__)== 0)  USERled_GPIO_CLK_DISABLE(); else\
																								  if((__INDEX__)== 1)  ERRORled_GPIO_CLK_DISABLE();\
																							} while(0)




void     BSP_LED_Init(Led_TypeDef Led);
void     BSP_LED_On(Led_TypeDef Led);
void     BSP_LED_Off(Led_TypeDef Led);
void     BSP_LED_Toggle(Led_TypeDef Led);



	 	 	 	 	 	 	 	 	 /***  Buttons  ***/
#define BUTTONn                          1
typedef enum
{
  BUTTON_KEY = 0,
} Button_TypeDef;

typedef enum
{
	BUTTON_MODE_GPIO = 0,
	BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

/**
 * @brief Wakeup push-button
 */
#define KEY_BUTTON_PIN                GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT          GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE() __GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn          EXTI0_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_ENABLE(); \
                                                }while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_DISABLE(); \
                                                 }while(0)




void     BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t BSP_PB_GetState(Button_TypeDef Button);

void PB_EXTILine0_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_OPTIONAL_H_ */
