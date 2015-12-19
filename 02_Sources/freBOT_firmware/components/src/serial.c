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

 * serial.c
 *
 *  Created on: Dec 4, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
 
*/

/* Includes ------------------------------------------------------------------*/
#include "freBOT_error.h"
#include "serial.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef UartHandle; /* UART handler declaration */
static ERROR_HandleTypeDef errorStruct;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
   * @brief  This function configure UART6 to communicate with a PC via an
   * FTDI chip.
   * @param  None
   * @retval None
   */
 void UART_Config(void) {

 	  /*##-1- Configure the UART peripheral ######################################*/
 	  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
 	  /* UART6 configured as follow:
 	      - Word Length = 8 Bits
 	      - Stop Bit = One Stop bit
 	      - Parity = None
 	      - BaudRate = 115200 baud
 	      - Hardware flow control disabled (RTS and CTS signals) */
 	  UartHandle.Instance          = USARTx;
 	  UartHandle.Init.BaudRate     = 115200;
 	  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
 	  UartHandle.Init.StopBits     = UART_STOPBITS_1;
 	  UartHandle.Init.Parity       = UART_PARITY_NONE;
 	  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
 	  UartHandle.Init.Mode         = UART_MODE_TX_RX;
 	  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

 	  if(HAL_UART_Init(&UartHandle) != HAL_OK)
 	   {

 	     Error_Handler(&errorStruct);
 	   }

  }

 /**
  * @brief  This function implements a user defined printf function
  * @param  ch: character to be print out
  * @retval ch: character to be print out
  */
 int custom_fputc(int ch ) {

	 if(HAL_UART_Transmit(&UartHandle, &ch, 1, 5000)!= HAL_OK)
	  {
 		 Error_Handler(&errorStruct);
	  }
	return ch;
 }
