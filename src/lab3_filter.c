/**
  ******************************************************************************
  * @file    lab3_filter.c
  * @author  Group 6
  * @version V1.0.0
  * @date    1-November-2013
  * @brief   This defines public functions for a simple Linear Moving Average Filter:
	*           - Insert value into buffer
	*           - Calculate average of all elements in buffer
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "lab3_filter.h"

/* Public Functions ---------------------------------------------------------*/

/** @defgroup Public_Functions
  * @{
  */

/**
  * @brief  Initializes Moving Average Filter
  * @param  *moving_average: Pointer to a Moving_Average struct
	* @param  size: Length of moving average array
  * @retval None
  */
void init_moving_average(struct Moving_Average *moving_average) {
	moving_average->index = 0;																																									//Initialize all struct members to 0
	moving_average->average = 0;
	uint32_t i;
	for (i=0; i<sizeof(moving_average->moving_values)/sizeof(moving_average->average); i++) {
		moving_average->moving_values[i] = 0;
	}
}

/**
  * @brief  Inserts a value into the Filter's array.
  * @param  *moving_average: Pointer to a Moving_Average struct
	* @param  new_value: New value to insert into filter's buffer
  * @retval None
  */

void insert_value(struct Moving_Average *moving_average, float new_value) {
	if (moving_average->index == sizeof(moving_average->moving_values)/sizeof(moving_average->average)-1) {			//Check if we need to wraparound index
		moving_average->index = 0;																																								//If so, set to 0
	} else {
		moving_average->index++;																																									//Otherwise, increment like normal
	}
	moving_average->moving_values[moving_average->index] = new_value;																						//Insert new value, overwriting oldest one
}

/**
  * @brief  Calculate's the current average value of the Filter's buffer and
	*         stores it in the struct
  * @param  *moving_average: Pointer to a Moving_Average struct
  * @retval None
  */

void calculate_average(struct Moving_Average *moving_average) {
	uint32_t i = 0;																																															//Initialize internal variables
	double sum = 0;
	for (i=0; i<sizeof(moving_average->moving_values)/sizeof(moving_average->average); i++) {										//Loop through entire filter array
		sum += moving_average->moving_values[i];																																	//Add all elements together
	}
	moving_average->average = sum/(sizeof(moving_average->moving_values)/sizeof(moving_average->average));			//Return algrbraic average
}

/**
  * @}
  */
