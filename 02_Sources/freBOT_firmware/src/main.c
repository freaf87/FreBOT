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
 * main.c
 *
 *  Created on: Dec 4, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
*/

/* Includes ------------------------------------------------------------------*/
#include "freBOT_error.h"
#include "main.h"
#include "initialiseHardware.h"
#include "cmsis_os.h"
#include "diag/Trace.h"

/* Private variables ---------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void initSTM32F407Hardware(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


int main(void)
{
	/* Microcontoller initialization */
	initSTM32F407Hardware();
	freBOT_init();
	for(int i = 0; i< 20; i++) custom_fputc("51");
	//trace_printf("Starting kernel ... \r\n");
	/* Start scheduler */
	//osKernelStart();



  /* If all is well then main() will never reach here as the scheduler will
  now be running the tasks. If main() does reach here then it is likely that
  there was insufficient heap memory available for the idle task to be created.
  Check then the heap */
  while(1){

	  HAL_Delay(500);
	  BSP_LED_Toggle(USERled);
  }
}

void initSTM32F407Hardware(void) {
	/* STM32F4xx HAL library initialization:
	 - Configure the Flash prefetch, instruction and Data caches
	 - Configure the Systick to generate an interrupt each 1 msec
	 - Set NVIC Group Priority to 4
	 - Global MSP (MCU Support Package) initialization
	 */
	SystemClock_Config(); //Configure the system clock to 168 MHz
	HAL_Init();
}


