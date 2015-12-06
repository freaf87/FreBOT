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
#include "freBOT_error.h"

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
GPIO_TypeDef* GPIO_PORT[LEDn] = {USERled_GPIO_PORT, 
                                 ERRORled_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {USERled_PIN, 
                                 ERRORled_PIN };

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT}; 
const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN}; 
const uint8_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */ 


static I2C_HandleTypeDef    I2cHandle;
/**
  * @}
  */ 
	
	
/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Functions
* @{
*/ 
static void     I2Cx_Init(void);
static void     I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t  I2Cx_ReadData(uint8_t Addr, uint8_t Reg);
static void     I2Cx_MspInit(void);
static void     I2Cx_Error(uint8_t Addr);

/*
// Link functions for Accelerometer peripheral
void            MPU9150_IO_Init(void);
void            MPU9150_IO_ITConfig(void);
void            MPU9150_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void            MPU9150_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
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
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg USERled
  *     @arg ERRORled
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg USERled
  *     @arg ERRORled
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg USERled
  *     @arg ERRORled
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg USERled
  *     @arg ERRORled
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}
	/**
  * @}
  */

/** @defgroup FREBOT_LOW_LEVEL_BUTTON_Functions
  * @{
  */ 
/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void PB_EXTILine0_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_KEY
  * @param  Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  if (Mode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
  
  if (Mode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; 
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));

  }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    /* Toggle LED3 */
    BSP_LED_Toggle(USERled);

  }
}
/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_KEY  
  * @retval The Button GPIO pin value.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}
/**
	* @}
	*/ 	
/** @defgroup FREBOT_LOW_LEVEL_MOTOR_Functions
  * @{
  */ 
/* Timer handler declaration */
TIM_HandleTypeDef    MOTOR_TimHandle;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

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
	*					DIR 	- Motor�s Direction (1 for Clockwise and -1 for counterclockwise) 
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

/**
	* @}
	*/ 	
/** @defgroup FREBOT_LOW_LEVEL_BUS_Functions
  * @{
  */ 
/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
/******************************* I2C Routines**********************************/
/**
  * @brief  Configures I2C interface.
  * @param  None
  * @retval None
*/
static void I2Cx_Init(void)
{
  if(HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET)
  {
    /* DISCOVERY_I2Cx peripheral configuration */
    I2cHandle.Init.ClockSpeed = BSP_I2C_SPEED;
    I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2cHandle.Init.OwnAddress1 = 0x33;
    I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Instance = FREBOT_I2Cx;
      
    /* Init the I2C */
    I2Cx_MspInit();
    HAL_I2C_Init(&I2cHandle);
  }
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written 
  * @retval HAL status
  */
static void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout); 

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  }
}

/**
  * @brief  Read a register of the device through BUS
  * @param  Addr: Device address on BUS  
  * @param  Reg: The target register address to read
  * @retval HAL status
  */
static uint8_t  I2Cx_ReadData(uint8_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1,I2cxTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  }
  return value;
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  Addr: I2C Address
  * @retval None
  */
static void I2Cx_Error(uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(&I2cHandle);
  
  /* Re-Initialize the I2C communication bus */
  I2Cx_Init();
}

/**
  * @brief I2C MSP Initialization
  * @param None
  * @retval None
  */
static void I2Cx_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  FREBOT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  /* DISCOVERY_I2Cx SCL and SDA pins configuration ---------------------------*/
  GPIO_InitStruct.Pin = FREBOT_I2Cx_SCL_PIN | FREBOT_I2Cx_SDA_PIN; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = FREBOT_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(FREBOT_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);     

  /* Enable the DISCOVERY_I2Cx peripheral clock */
  FREBOT_I2Cx_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  FREBOT_I2Cx_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  FREBOT_I2Cx_RELEASE_RESET();

  /* Enable and set I2Cx Interrupt to the highest priority */
  HAL_NVIC_SetPriority(FREBOT_I2Cx_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FREBOT_I2Cx_EV_IRQn);

  /* Enable and set I2Cx Interrupt to the highest priority */
  HAL_NVIC_SetPriority(FREBOT_I2Cx_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FREBOT_I2Cx_ER_IRQn); 
}
/**
	* @}
	*/ 	
	
/**
	* @}
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


/**
	* @}
	*/ 

/**
	* @}
	*/ 
