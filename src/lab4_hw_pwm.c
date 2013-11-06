/**
  ******************************************************************************
  * @file    lab3_pwm.c
  * @author  Group 6
  * @version V1.0.0
  * @date    1-November-2013
  * @brief   Provides driver functions for hardware PWM control using
	*          output compare on TIM4
  */

/* Includes ------------------------------------------------------------------*/

#include <math.h>
#include "stm32f4xx.h"
#include "lab4_hw_pwm.h"

/* Defines ------------------------------------------------------------------*/

#ifndef PI
#define PI 3.14159
#endif

/* Public Functions ---------------------------------------------------------*/

/** @defgroup Public_Functions
  * @{
  */

/**
  * @brief  Initializes LED_PWM struct with default values
	* @param  *led_pwm: Pointer to LED_PWM struct to initialize
	* @param  max_pwm_intensity: Number of intensity steps from off to fully on
	* @param  pwm_pulse_speed: Length of time for LED to complete a pulse cycle, off to off (in ms)
	* @param  phase: Initial LED intensity
	* @param  CCR: Capture compare register for output compare
	* @param  TIMx: Timer to connect LED to
  * @retval None
  */

void init_LED_PWM(struct LED_PWM *led_pwm, uint32_t max_pwm_intensity, uint32_t pwm_pulse_speed, uint32_t phase, uint32_t CCR, TIM_TypeDef* TIMx) {		//Initialize LED with parameters
	led_pwm->pwm_pulse_speed = pwm_pulse_speed;																																																					//Pulse speed, defined constant
	led_pwm->pwm_intensity = phase;																																																											//In-sync vs out of sync pwm
	led_pwm->max_pwm_intensity = max_pwm_intensity;																																																			//Defined constant
	led_pwm->pwm_direction = 0;																																																													//Initialize to 0
	led_pwm->CCR = CCR;																																																																	//Gets specified capture&control register
	led_pwm->led_pwm_update_pulse_count = 0;																																																						//Initialize to 0
	led_pwm->TIMx = TIMx;																																																																//LED is on which TIM?
}

/**
  * @brief  Updates intensity of an LED_PWM.
	* @note   This function should be called periodically, at the desired
	*         frequency that the LED should increment/decrement intensity by 1
	* @param  *led_pwm: Pointer to LED_PWM struct to update intensity of
  * @retval None
  */

void update_led_pwm_intensity_pulse(struct LED_PWM *led_pwm) {
	if (led_pwm->pwm_intensity == led_pwm->max_pwm_intensity) {
		led_pwm->pwm_direction = 1;																															//If the LED is fully on, change pulse direction to start dimming
	} else if (led_pwm->pwm_intensity == 0) {
		led_pwm->pwm_direction = 0;																															//If the LED is fully off, change the pulse direction to brighten
	}
	if (led_pwm->pwm_direction) {																															//Increment or decrement LED intensity according to direction we should be going in
		led_pwm->pwm_intensity--;
	} else {
		led_pwm->pwm_intensity++;
	}
	
	uint32_t real_pwm_intensity = led_pwm->max_pwm_intensity * pow(0.5f*(-cos(2*PI*(float)led_pwm->pwm_intensity / led_pwm->max_pwm_intensity)+1), 2); //Smooth PWM intensity function
	
	switch (led_pwm->CCR) {																																		//Update corresponding CCR with the new intensity value
		case 1:
			TIM_SetCompare1(TIM4, real_pwm_intensity);
			break;
		case 2:
			TIM_SetCompare2(TIM4, real_pwm_intensity);
			break;
		case 3:
			TIM_SetCompare3(TIM4, real_pwm_intensity);
			break;
		case 4:
			TIM_SetCompare4(TIM4, real_pwm_intensity);
			break;
		default:
			break;
	}
}

/**
  * @brief  Updates intensities of all LEDs simultaneously
	* @param  led_intensities[]: Array of intensity values to write to LEDs
	* @param  length: Length of led_intensities array
	* @param  TIMx: TIM to write CCR values to
  * @retval None
  */

void update_led_intensities(uint32_t led_intensities[], uint32_t length, TIM_TypeDef* TIMx) {
	if (length == 4)
	{
		TIM_SetCompare1(TIMx, led_intensities[0]);									//set the 4 TIM4 capture compare register values
		TIM_SetCompare2(TIMx, led_intensities[1]);									//based off LED intensity
		TIM_SetCompare3(TIMx, led_intensities[2]);
		TIM_SetCompare4(TIMx, led_intensities[3]);
	}
}

/**
  * @}
  */
