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
 * motor.c
 *
 *  Created on: Dec 19, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
 
*/

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "freBOT_error.h"
/* Private variables ---------------------------------------------------------*/
static ERROR_HandleTypeDef errorStruct;
/* Timer handler declaration */
TIM_HandleTypeDef    MOTOR_TimHandle;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
	* @brief  Configure the TIM9 CH1 & CH2 in PWM OUT Mode.
	* @param  None
	* @retval None
	*/
void MOTOR_TIM_init(void)
{
/* Counter Prescaler value */
uint32_t uwPrescalerValue = 0;
/* Compute the prescaler value to have TIM3 counter clock equal to 10 kHz */
  uwPrescalerValue = ((168000000/2) / 28000000 ) - 1;

  /*##-1- Configure the TIM peripheral #######################################*/
  /* Initialize TIMx peripheral as follow:
       + Prescaler = (SystemCoreClock/2)/28000000
       + Period = 2800  (to have an output frequency equal to 20 KHz)
       + ClockDivision = 0
       + Counter direction = Up
  */
  MOTOR_TimHandle.Instance = MOTOR_TIMx;

  MOTOR_TimHandle.Init.Prescaler     = uwPrescalerValue;
  MOTOR_TimHandle.Init.Period        = PERIOD_VALUE;
  MOTOR_TimHandle.Init.ClockDivision = 0;
  MOTOR_TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_PWM_Init(&MOTOR_TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(&errorStruct);
  }
  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_PWM1;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;

	  /* Set the pulse value for channel 1 */
  sConfig.Pulse = PULSE_VALUE;
  if(HAL_TIM_PWM_ConfigChannel(&MOTOR_TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler(&errorStruct);
  }

  /* Set the pulse value for channel 2 */
  sConfig.Pulse = PULSE_VALUE;
  if(HAL_TIM_PWM_ConfigChannel(&MOTOR_TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler(&errorStruct);
  }

  /*##-3- Start PWM signals generation #######################################*/
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&MOTOR_TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /*  Error */
    Error_Handler(&errorStruct);
  }
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&MOTOR_TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /*  Error */
    Error_Handler(&errorStruct);
  }
}

/**
	* @brief  Initialize the Motor Directions pins.
	*					 M1:  	IN 	 PB14
	*					 M2:  	IN 	 PB14
	* @param  None
	* @retval None
	*/
void MOTOR_DIRECTION_PINS_INIT (void)
{
		GPIO_InitTypeDef  GPIO_InitStruct;

		/* GPIPeriph clock enable */
		MOTOR1_DIR_GPIO_CLK_ENABLE();
		MOTOR2_DIR_GPIO_CLK_ENABLE();


		/* Configure the MOTOR1_DIR_PIN pin */
		GPIO_InitStruct.Pin = MOTOR1_DIR_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(MOTOR1_DIR_GPIO_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_PORT, MOTOR1_DIR_PIN, GPIO_PIN_RESET);

		/* Configure the MOTOR2_DIR_PIN pin */
		GPIO_InitStruct.Pin = MOTOR2_DIR_PIN;
		HAL_GPIO_Init(MOTOR2_DIR_GPIO_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_PORT, MOTOR2_DIR_PIN, GPIO_PIN_RESET);
}


/**
	* @brief  set the desired Direction
	* @param  MOTOR - Motor's name(1 or 2)
	*					DIR 	- Motor's Direction pin (1 for Clockwise and -1 for counterclockwise)
	* @retval None
	*/
void MOTOR_CONTROL_SET_DIRECTION(char MOTOR, int DIR)
{
		switch(MOTOR)
		{

			case 1:
				if (DIR==1){
					HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_PORT, MOTOR1_DIR_PIN, GPIO_PIN_SET);
				}
				else{
						HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_PORT, MOTOR1_DIR_PIN, GPIO_PIN_RESET);
					}
			break;

			case 2:
				if (DIR==1){
					HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_PORT, MOTOR2_DIR_PIN, GPIO_PIN_SET);
				}
					else {
						HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_PORT, MOTOR2_DIR_PIN, GPIO_PIN_RESET);
					}

			break;

		}

}
