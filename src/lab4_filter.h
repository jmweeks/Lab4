/**
  ******************************************************************************
  * @file    lab3_filter.h
  * @author  Group 6
  * @version V1.0.0
  * @date    1-November-2013
  * @brief   This defines public functions for a simple Linear Moving Average Filter:
	*           - Insert value into buffer
	*           - Calculate average of all elements in buffer
	*          It also defines a Moving Average Filter struct
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"

/* Defines ------------------------------------------------------------------*/

#ifndef MAX_MOVING_AVERAGE_FILTER_SIZE
#define MAX_MOVING_AVERAGE_FILTER_SIZE 32																									//Size of moving average circular filter window
#endif

/* Exported Types ---------------------------------------------------------*/

/** @defgroup Structs
  * @{
  */

#ifndef MOVING_AVERAGE_STRUCT
#define MOVING_AVERAGE_STRUCT

struct Moving_Average {
	float moving_values[MAX_MOVING_AVERAGE_FILTER_SIZE];
	uint32_t index;
	float average;
	uint32_t filter_size;
};

#endif

/**
  * @}
  */

/* Public Functions ---------------------------------------------------------*/

/** @defgroup Public_Functions
  * @{
  */ 

void init_moving_average(struct Moving_Average *moving_average, uint32_t size);
void insert_value(struct Moving_Average *moving_average, float new_value);
void calculate_average(struct Moving_Average *moving_average);

/**
  * @}
  */
