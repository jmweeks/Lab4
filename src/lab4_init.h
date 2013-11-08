/**
  ******************************************************************************
  * @file    lab4_init.h
  * @author  Group 6
  * @version V1.0.0
  * @date    8-November-2013
  * @brief   This defines public functions for a simple Linear Moving Average Filter:
	*           - Insert value into buffer
	*           - Calculate average of all elements in buffer
	*          It also defines a Moving Average Filter struct
  */


/* Defines ------------------------------------------------------------------*/

#ifndef MAX_PWM_INTENSITY																															//safety check on max intesnity
#define MAX_PWM_INTENSITY 500																													//Number of steps between fully off and fully on
#endif

#ifndef PWM_FREQUENCY																																	//safety check on frequency
#define PWM_FREQUENCY 500																															//Driving PWM frequency (Hz)
#endif

#ifndef PWM_PULSE_SPEED																																//safety check on pulse cycle
#define PWM_PULSE_SPEED 2000																													//Time of a complete pulse cycle, off to off (in ms)
#endif

#ifndef PWM_UPDATE_INTENSITY_FREQUENCY																								//safety check on intensity
#define PWM_UPDATE_INTENSITY_FREQUENCY 5*MAX_PWM_INTENSITY														//Frequency at which to update intensity
#endif

#ifndef TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE																				//safety check on temperature moving average filter size
#define TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE 16																			//set temperature moving average filter size to 16
#endif

//MAX_MOVING_AVERAGE_FILTER_SIZE can be found in lab4_filter.h

#if TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE > MAX_MOVING_AVERAGE_FILTER_SIZE						//safety check on filter size, can overflow stack and freeze system
#undef TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE																					//if we're over limit, undefine filter size
#define TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE MAX_MOVING_AVERAGE_FILTER_SIZE					//set to max allowable filter size
#endif

#ifndef ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE																			//safety check on accelerometer moving average filter size
#define ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE 10																		//set moving average filter size to be 10
#endif

//MAX_MOVING_AVERAGE_FILTER_SIZE can be found in lab4_filter.h
#if ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE > MAX_MOVING_AVERAGE_FILTER_SIZE					//safety check on filter size, can overflow stack and freeze system
#undef ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE																				//if we're over limit, undefine filter size
#define ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE MAX_MOVING_AVERAGE_FILTER_SIZE				//set to max allowable filter size
#endif

/**
  * @}
  */

/* Public Functions ---------------------------------------------------------*/

/** @defgroup Public_Functions
  * @{
  */ 

void init_accelerometer(void);								
void init_TIM2(void);
void init_TIM3(void);
void init_adc(void);
void init_temp_sensor(void);
void init_pushbutton(void);
void init_LEDS(void);
void init_TIM4(void);
void init_LEDS_HW_PWM(void);
void init_EXTI1(void);
void init_TIM5(void);

/**
  * @}
  */
