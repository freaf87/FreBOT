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
 * frebot_error.h
 *
 *  Created on: Dec 4, 2015
 *      @Author: freaf87
 *      @version V1.0.0
 *      @brief:
 
*/

#ifndef INC_FREBOT_ERROR_H_
#define INC_FREBOT_ERROR_H_

#define errorQUEUE_SIZE (uint32_t) 10

/**
  * @brief Error structure definition
  */
typedef enum
{
		freBOT_info     = 0x01, /*!< An informative message. No action is necessary. */
		freBOT_warning	= 0x02, /*!< Action must be taken at some stage to prevent a severe error from occurring in the future. */
		freBOT_error    = 0x03, /*!< A severe error that might cause malfunctioning */
		freBOT_fatal    = 0x04  /*!< A severe error that causes the system to crash */
} Error_TypeDef;

/* Define the structure type that will be passed on the queue. */
typedef struct
{
  unsigned int  errorSeverity;
   char errorSource[30];
   char errorComment[30];
} ERROR_HandleTypeDef;


/* Private function prototypes -----------------------------------------------*/
void Error_Handler(ERROR_HandleTypeDef *errorStruct);
void SetErrorStruct(ERROR_HandleTypeDef *errorStruct, int severiy, char*string, char*comment);
#endif /* INC_FREBOT_ERROR_H_ */
