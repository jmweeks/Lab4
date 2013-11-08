/**
  ******************************************************************************
  * @file    lab4_temp.h
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   This defines a temperature reading function using the built-in
	           temperature sensor and produces a display using the four
						 on-board LEDs
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "lab4_filter.h"

/* Exported Types ---------------------------------------------------------*/

/** @defgroup Structs
  * @{
  */ 

#ifndef TEMPERATURE_READER_STRUCT																						//safety check on temperature reader structure
#define TEMPERATURE_READER_STRUCT																						//define temperature reader structure

struct Temperature_Reader {																									//temperature reader structure
	struct Moving_Average moving_average;																			//contains moving average window
};

#endif

/**
  * @}
  */

/* Temperature Reader Public Functions ---------------------------------------------------------*/

/** @defgroup Temperature_Reader_Public_Functions
  * @{
  */ 

void rotate_led(uint32_t led_number);
void init_temp_reader(struct Temperature_Reader *temperature_reader);
void read_temp(struct Temperature_Reader *temperature_reader);

/**
  * @}
  */
