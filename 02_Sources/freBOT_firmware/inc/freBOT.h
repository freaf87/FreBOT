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


void MOTOR_TIM_init(void);
void MOTOR_DIRECTION_PINS_INIT (void);
void MOTOR_CONTROL_SET_DIRECTION(char MOTOR, int DIR);

void freBOT_init(void);



#ifdef __cplusplus
}
#endif

#endif /* __FREBOT_H */
