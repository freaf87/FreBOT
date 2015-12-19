/** 
  ******************************************************************************
  * @file    freBOT.c
  * @author  Frederic Afadjigla
  * @version V1.0.0
  * @date    22-September-2015
  * @brief   This file contains definitions for freBOT's Leds and 
  *          push-button hardware resources.
    ******************************************************************************  
 */ 
/* Includes ------------------------------------------------------------------*/
#include "freBOT.h"
#include "optional.h"
#include "freBOT_error.h"
#include "serial.h"
#include "stm32f4xx_hal.h"

static ERROR_HandleTypeDef errorStruct;

 /**
  * @brief freBOT Driver version number V1.0.0
  */
#define __FREBOT_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __FREBOT_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __FREBOT_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __FREBOT_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __FREBOT_BSP_VERSION         ((__FREBOT_BSP_VERSION_MAIN << 24)\
																		 |(__FREBOT_BSP_VERSION_SUB1 << 16)\
																		 |(__FREBOT_BSP_VERSION_SUB2 << 8 )\
																		 |(__FREBOT_BSP_VERSION_RC)) 
/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Variables
  * @{
  */ 



/**
  * @}
  */ 
	
	
/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Functions
* @{
*/ 
/**
  * @}
  */
/** @defgroup FREBOT_LOW_LEVEL_LED_Functions
  * @{
  */ 	
	/**
  * @brief  This method returns the STM32F4 DISCO BSP Driver revision
  * @param  None
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __FREBOT_BSP_VERSION;
}


	/**
  * @}
  */

/** @defgroup FREBOT_LOW_LEVEL_BUTTON_Functions
  * @{
  */ 

/**
	* @}
	*/ 	
/** @defgroup FREBOT_LOW_LEVEL_MOTOR_Functions
  * @{
  */ 


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void freBOT_init(void)
{
	/* Configure USERled and ERRORled */
	BSP_LED_Init(USERled);
	BSP_LED_Init(ERRORled);

	/* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
	PB_EXTILine0_Config();
	/* Motor init */
	MOTOR_TIM_init();
	MOTOR_DIRECTION_PINS_INIT();
	MOTOR_CONTROL_SET_DIRECTION(1, 1);
	MOTOR_CONTROL_SET_DIRECTION(2, 1);

	UART_Config();
}	

