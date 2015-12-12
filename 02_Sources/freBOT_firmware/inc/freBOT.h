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


/**
 *  freBOT HAL Components
 **/
 	 	 	 	 	 	 	 /***  LEds  ***/
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
	 	 	 	 	 	 	 	 	 /***  Buttons  ***/
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

	 	 	 	 	 	 	 	 /***  MPU9150  ***/
/*  Interrupt PIN */
#define MPU9150_INT_PIN                         GPIO_PIN_1
#define MPU9150_INT_GPIO_PORT                   GPIOA
#define MPU9150_INT_GPIO_CLK                    RCC_AHB1Periph_GPIOA
#define MPU9150_INT_EXTI_PORT                   EXTI_PortSourceGPIOA
#define MPU9150_INT_EXTI_PIN                    EXTI_PinSource1
#define MPU9150_INT_EXTI_LINE                   EXTI_Line1
#define MPU9150_INT_EXTI_IRQ                    EXTI1_IRQn

/*  I2C PINs */
#define MPU9150_I2C               				I2C2
#define MPU9150_I2C_SPEED         				400000
#define MPU9150_I2C_ADDRESS           			0x00
#define MPU9150_FLAG_TIMEOUT             ((uint32_t) 900) //0x1100
#define MPU9150_LONG_TIMEOUT             ((uint32_t) (300 * I2Cx_FLAG_TIMEOUT)) //was300

#define MPU9150_I2C_SCL_GPIO_PORT         GPIOB
#define MPU9150_I2C_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define MPU9150_I2C_SCL_GPIO_PIN          GPIO_PIN_10
#define MPU9150_I2C_SCL_GPIO_PINSOURCE    GPIO_PinSource10

#define MPU9150_I2C_SDA_GPIO_PORT         GPIOB
#define MPU9150_I2C_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define MPU9150_I2C_SDA_GPIO_PIN          GPIO_PIN_11
#define MPU9150_I2C_SDA_GPIO_PINSOURCE    GPIO_PinSource11

#define MPU9150_I2C_RCC_CLK               RCC_APB1Periph_I2C2
#define MPU9150_I2C_AF                    GPIO_AF4_I2C2

#define I2Cx_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_FORCE_RESET()               __HAL_RCC_I2C2_FORCE_RESET()
#define I2Cx_RELEASE_RESET()             __HAL_RCC_I2C2_RELEASE_RESET()


#define WAIT_FOR_FLAG(flag, value, timeout, errorcode)  I2CTimeout = timeout;\
          while(I2C_GetFlagStatus(MPU9150_I2C, flag) != value) {\
            if((I2CTimeout--) == 0) return I2Cx_TIMEOUT_UserCallback(errorcode); \
          }\

#define CLEAR_ADDR_BIT      I2C_ReadRegister(MPU9150_I2C, I2C_Register_SR1);\
                            I2C_ReadRegister(MPU9150_I2C, I2C_Register_SR2);\

#define I2C_Config() I2cMaster_Init();

/* Definition for TIMx clock resources */
#define MPU9150_TIMx                           TIM3
#define MPU9150_TIMx_CLK_ENABLE                __HAL_RCC_TIM3_CLK_ENABLE

/* Definition for TIMx's NVIC */
#define MPU9150_TIMx_IRQn                      TIM3_IRQn
#define MPU9150_TIMx_IRQHandler                TIM3_IRQHandler












/**
 *  Private function prototypes
 **/
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



#ifdef __cplusplus
}
#endif

#endif /* __FREBOT_H */
