/** 
  ******************************************************************************
  * @file    freBOT.h
  * @author  Frederic Afadjigla
  * @version V1.0.0
  * @date    22-September-2015
  * @brief   This file contains definitions for freBOT's Leds and 
  *          push-button hardware resources.
    ******************************************************************************  
 */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREBOT_H
#define __FREBOT_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
  

/** @addtogroup BSP
  * @{
  */ 
	
	/** @addtogroup FREBOT
  * @{
  */ 
	
/** @addtogroup FREBOT_LOW_LEVEL
  * @{
  */ 


/** @defgroup FREBOT_LOW_LEVEL_Exported_Types
  * @{
*/
	
typedef enum
{
		USERled		= 0,
		ERRORled	= 1
} Led_TypeDef;

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
  * @}
*/
	
/** @defgroup FREBOT_LOW_LEVEL_Exported_Constants
  * @{
*/ 

/** 
* @brief  Define for freBOT board  
*/ 
	
/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_LED
* @{
*/
#define LEDn                             		2

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
/**
* @}
*/
/** @addtogroup FREBOT_LOW_LEVEL_BUTTON
  * @{
*/  	
#define BUTTONn                          1 

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

/** @addtogroup FREBOT_LOW_LEVEL_MOTOR
  * @{
*/  
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


/**
* @}
*/
/**
* @}
*/
	
/** @addtogroup FREBOT_LOW_LEVEL_BUS
  * @{
  */  	
/*############################# I2C2 #########################################*/
/* I2C clock speed configuration (in Hz) */
#ifndef  BSP_I2C_SPEED
 #define BSP_I2C_SPEED                            100000
#endif /* BSP_I2C_SPEED */

/* I2C peripheral configuration defines (control interface of the audio codec) */
#define FREBOT_I2Cx                            I2C2
#define FREBOT_I2Cx_CLK_ENABLE()               __I2C2_CLK_ENABLE()
#define FREBOT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define FREBOT_I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C2
#define FREBOT_I2Cx_SCL_SDA_GPIO_PORT          GPIOB
#define FREBOT_I2Cx_SCL_PIN                    GPIO_PIN_10
#define FREBOT_I2Cx_SDA_PIN                    GPIO_PIN_11
       
#define FREBOT_I2Cx_FORCE_RESET()              __I2C2_FORCE_RESET()
#define FREBOT_I2Cx_RELEASE_RESET()            __I2C2_RELEASE_RESET()

/* I2C interrupt requests */                  
#define FREBOT_I2Cx_EV_IRQn                    I2C2_EV_IRQn
#define FREBOT_I2Cx_ER_IRQn                    I2C2_ER_IRQn

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define I2Cx_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */
		
/**
* @}
*/
	
/**
* @}
*/	

/**
* @}
*/

/** @defgroup FREBOT_LOW_LEVEL_Exported_Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup FREBOT_LOW_LEVEL_Exported_Functions
* @{
*/
uint32_t BSP_GetVersion(void);
void     BSP_LED_Init(Led_TypeDef Led);
void     BSP_LED_On(Led_TypeDef Led);
void     BSP_LED_Off(Led_TypeDef Led);
void     BSP_LED_Toggle(Led_TypeDef Led);
void     BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t BSP_PB_GetState(Button_TypeDef Button);

void PB_EXTILine0_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void MOTOR_TIM_init(void);
void MOTOR_DIRECTION_PINS_INIT (void);
void MOTOR_CONTROL_SET_DIRECTION(char MOTOR, int DIR);


void freBOT_init(void);

/**
* @}
*/



/**
  * @}
  */
/**
  * @}
*/

#ifdef __cplusplus
}
#endif

#endif /* __FREBOT_H */
