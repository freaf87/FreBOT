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

 * error.c
 *
 *  Created on: Dec 4, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
 
*/

/* Includes ------------------------------------------------------------------*/
#include "freBOT.h"
#include "optional.h"
#include "freBOT_error.h"
#include "cmsis_os.h"
#include "diag/Trace.h"
#include <string.h>
/* Private variables ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function sets the error structure.
  * @param  errorStruct
  * 		severity
  * 		error string
  * @retval None
  */
void SetErrorStruct(ERROR_HandleTypeDef *errorStruct, int severiy, char*string, char*comment)
{
	 strcpy(errorStruct -> errorSource, string);
	 strcpy(errorStruct -> errorComment,comment);
	 errorStruct -> errorSeverity = severiy;
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  errorStruct
  * @retval None
  */
void Error_Handler(ERROR_HandleTypeDef *errorStruct)
{
/*
	trace_printf("error Source  : %s\n",errorStruct -> errorSource);
	trace_printf("error Comment : %s\n" ,errorStruct -> errorComment);
	trace_printf("error severity: %d\n",errorStruct -> errorSeverity);
*/
  while(1)
  {
	  /* Toggle ERRORled */
	  BSP_LED_Toggle(ERRORled);
	  HAL_Delay(500);
	  if (errorStruct -> errorSeverity < freBOT_fatal){
		  BSP_LED_Off(ERRORled);
		  break;
	  }
  }
}
