/**
  ******************************************************************************
  * @file    lab4_temp.c
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   This defines a temperature reading function using the built-in
	           temperature sensor and produces a display using the four
						 on-board LEDs
  */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "lab4_filter.h"
#include "lab4_temp.h"
#include "lab4_init.h"

/* Defines ------------------------------------------------------------------*/

#define V25 .760f
#define AVSLOPE .0025f
#define VDD 2.938f

/* Temperature Reader Private Functions ---------------------------------------------------------*/

/** @defgroup Temperature_Reader_Private_Functions
  * @{
  */

/**
  * @brief  Converts ADC value to temperature (in degrees C).
	* @param  raw_value: Raw ADC temperature sensor reading.
  * @retval float: Converted temperature in degrees C.
  */

static float get_temperature(uint16_t raw_value) {
	return((((VDD*((float)raw_value)/4096 - V25)/AVSLOPE)+25));										//Take raw ADC value, scale it from 0-4095 to 0-3V
}																																								//Return this scaled value after sensor calibration function

/* Temperature Reader Public Functions ---------------------------------------------------------*/

/** @defgroup Temperature_Reader_Public_Functions
  * @{
  */

/**
  * @brief  Turns on the selected LED (modulo 8)
	* @note   This function takes an LED number (from 0-7) and maps it to
            a base-4 system in the following manner:
            led_number -> LED (Pin)
                 0     ->  12
                 1     ->  12
                 2     ->  13
                 3     ->  13
                 4     ->  14
                 5     ->  14
                 6     ->  15
                 7     ->  
	* @param  led_number: LED number to turn on.
  * @retval float: Converted temperature in degrees C.
  */
void rotate_led(uint32_t led_number) {
	GPIO_Write(GPIOD, 0x0); 																												//Reset all LEDs
	switch (led_number) { 																													//case statments based off of led number passed to rotate function
		case 0:																																				//remainder 0
			GPIO_SetBits(GPIOD, GPIO_Pin_12);																						//turn on GPIO_Pin_12
			break;
		case 1:																																				//remainder 1
			GPIO_SetBits(GPIOD, GPIO_Pin_12);																						//turn on GPIO_Pin_12
			break;
		case 2:																																				//remainder 2
			GPIO_SetBits(GPIOD, GPIO_Pin_13);																						//turn on GPIO_Pin_13
			break;
		case 3:																																				//remainder 3
			GPIO_SetBits(GPIOD, GPIO_Pin_13);																						//turn on GPIO_Pin_13
			break;
		case 4:																																				//remainder 4
			GPIO_SetBits(GPIOD, GPIO_Pin_14);																						//turn on GPIO_Pin_14
			break;
		case 5:																																				//remainder 5
			GPIO_SetBits(GPIOD, GPIO_Pin_14);																						//turn on GPIO_Pin_14
			break;
		case 6:																																				//remainder 6
			GPIO_SetBits(GPIOD, GPIO_Pin_15);																						//turn on GPIO_Pin_15
			break;
		default:																																			//remainder 7
			GPIO_SetBits(GPIOD, GPIO_Pin_15);																						//turn on GPIO_Pin_15
			break;
	}
}

/**
  * @}
  */

/**
  * @brief  Initializes Temperature_Reader struct. Should be called before use.
	* @param  *moving_average: Pointer to a Moving_Average filter struct
  * @retval None
  */
void init_temp_reader(struct Temperature_Reader *temperature_reader) {																		//initialize temperature reader
	struct Moving_Average moving_average;																																		//create moving average struct for values
	temperature_reader->moving_average = moving_average;																										//move moving average struct into temperature reader struct
	init_moving_average(&temperature_reader->moving_average, TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE);				//initialize moving average in temperature struct with the currently defined filter size
}

/**
  * @brief  Reads the on-board temperature and updated built-in LEDs
	* @param  *moving_average: Pointer to a Moving_Average filter struct
  * @retval None
  */
void read_temp(struct Temperature_Reader *temperature_reader) {
	ADC_SoftwareStartConv(ADC1); 																														//Start ADC conversion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET); 																	//Wait until conversion is complete
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC); 																											//Reset ADC flag so its ready for the next conversion
	float current = get_temperature(ADC_GetConversionValue(ADC1)); 													//Get current temperature value
	insert_value(&temperature_reader->moving_average, current); 														//Insert newest temperature reading into filter
	calculate_average(&temperature_reader->moving_average); 																//Calculate averaged temperature
}

/**
  * @}
  */
