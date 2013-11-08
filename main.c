/**
  ******************************************************************************
  * @file    main.c
  * @author  Group 6
  * @version V1.0.0
  * @date    8-November-2013
  * @brief   Main entry point for Lab4 - Accelerometer and Temperature Sensor Modes in RTOS
  */

/* Includes ------------------------------------------------------------------*/


#include "arm_math.h"

#include "stm32f4xx.h"
#include "cmsis_os.h"

#include "lab4_init.h"
#include "lab4_accelerometer.h"
#include "lab4_temp.h"

/*!
 @brief Thread to perform menial tasks such as switching LEDs
 @param argument Unused
 */
void accelerometer_thread(void const *argument);						//add accelerometer_thread to active thread list
void temperature_thread(void const *argument);							//add temperature_thread to active thread list
void tap_thread(void const *argument);											//add tap detection thread to active thread list
void pushbutton_thread(void const *argument);								//add pushbutton thread to active thread list
void sw_pwm_thread(void const *argument);										//add sw flashing thread to active thread list
void hw_pwm_thread(void const *argument);										//add hw pwm thread to active thread list

//! Thread structure for above thread
osThreadDef(accelerometer_thread, osPriorityNormal, 1, 0);	//define accelerometer thread priority, instances and stack size (0=default)
osThreadDef(temperature_thread, osPriorityNormal, 1,0);			//define temperature sensor thread priority, instances and stack size (0=default)
osThreadDef(tap_thread, osPriorityNormal, 1, 0);						//define tap detection thread priority, instances and stack size (0=default)
osThreadDef(pushbutton_thread, osPriorityNormal, 1, 0);			//define pushbutton thread priority, instances and stack size (0=default)
osThreadDef(sw_pwm_thread, osPriorityNormal, 1, 0);					//define flashing mode thread priority, instances and stack size (0=default)
osThreadDef(hw_pwm_thread, osPriorityNormal, 1, 0);					//define hw pwm mode thread priority, instances and stack size (0=default)

osMutexDef(led_mutex);																			//define mutex to control access to LEDs
osMutexDef(mode_mutex);																			//define mutex to control access to mode variable

// ID for thread
static osThreadId tid_accelerometer_thread;									//accelerometer thread id
static osThreadId tid_temperature_thread;										//temperature sensor thread id
static osThreadId tid_tap_thread;														//tap detection thread id
static osThreadId tid_pushbutton_thread;										//user pushbutton thread id
static osThreadId tid_sw_pwm_thread;												//flashing mdode thread id
static osThreadId tid_hw_pwm_thread;												//hw pwm mode thread id

static osMutexId led_mutex;																	//mutex ID for controlling access to LEDs
static osMutexId mode_mutex;																//mutex ID for controlling access to MODE variable

/* Private Variables ------------------------------------------------------------------*/

static uint32_t mode;																				//mode variable for controlling the type of LED pattern being displayed

/**
  * @brief  Main entry point
	* @param  None
  * @retval int: Error code
  */
int main (void) {

	init_TIM2();																							//initialize TIM2, controls accelerometer sampling rate
	init_TIM3();																							//initialize TIM3, controls temperature sensor sampling rate
	init_TIM4();																							//initialize TIM4, controls HW pwm mode
	init_TIM5();																							//initialize TIM5, used to update intensity value for HW pwm
	init_accelerometer();																			//initialize accelerometer
	init_adc();																								//initialize ADC for accelerometer
	init_temp_sensor();																				//initialize temperature sensor
	init_pushbutton();																				//initialize user pushbutton & interrupt
	init_LEDS();																							//initialize 4 onboard LEDs
	init_EXTI1();																							//initialize external interrupt 1, triggered via tap detection

	
	led_mutex = osMutexCreate(osMutex(led_mutex));						//create mutex to control access to LEDs
	mode_mutex = osMutexCreate(osMutex(mode_mutex));					//create mutex to control access to mode variable

	// Start thread
	tid_accelerometer_thread = osThreadCreate(osThread(accelerometer_thread), NULL);		//create thread for accelerometer sampling & display using accelerometer thread id and definition
	tid_temperature_thread = osThreadCreate(osThread(temperature_thread), NULL);				//create thread for temperature sensor sampling & display using temperature sensor thread id and definition
	tid_tap_thread = osThreadCreate(osThread(tap_thread), NULL);												//create thread for tap detection, using tap detection thread id and definition
	tid_pushbutton_thread = osThreadCreate(osThread(pushbutton_thread), NULL);					//create thread for pushbutton interrupt using pushbutton thread id and definition
	tid_sw_pwm_thread = osThreadCreate(osThread(sw_pwm_thread), NULL);									//create thread for accelerometer sampling & display using accelerometer thread id and definition
	tid_hw_pwm_thread = osThreadCreate(osThread(hw_pwm_thread), NULL);									//create thread for accelerometer sampling & display using accelerometer thread id and definition

	// The below doesn't really need to be in a loop
	while(1){
		osDelay(osWaitForever);																														//Wait for something to happen
	}
}


/**
  * @}
  */

/* Public Functions ---------------------------------------------------------*/

/** @defgroup Public_Functions
  * @{
  */

/**
  * @brief  Thread triggered by accelerometer sampling interrupt
	* @param  None
  * @retval None
  */

void accelerometer_thread(void const *argument) {				//accelerometer sampling and display thread
	uint32_t current_mode;																//current mode variable
	struct Orientation orientation;												//name for orientation struct for holding pitch, roll, yaw, calibrated values and averages
	init_orientation(&orientation);												//initialize orientation struct for holding pitch, roll, yaw, calibrated values and averages
	
	while(1){
		osSignalWait(0x0001, osWaitForever);								//we're waiting on a signal from TIM2 telling us that we should sample the accelerometer
		update_orientation(&orientation);										//if its time to sample, update orientation and store values in orientation struct
		
		osMutexWait(mode_mutex, osWaitForever);							//wait for access to mode mutex
		current_mode = mode;																//set current mode to mode
		osMutexRelease(mode_mutex);													//release mode mutex
		
		if (current_mode == 1) {														//we're always sampling, but if we're in mode 1, we want to be using the tilt angle display for the LEDs
			osMutexWait(led_mutex, osWaitForever);						//wait on the LED mutex
			display_orientation(&orientation);								//display orientation on LEDs for tilt angle
			osMutexRelease(led_mutex);												//release LEDs
		}
	}
}

/**
  * @brief  Thread triggered by temperature sensor sampling interrupt
	* @param  None
  * @retval None
  */
void temperature_thread(void const *argument) {					//temperature sensor sampling and display thread
	uint32_t current_mode;																//current mode variable, for use with mode mutex
	struct Temperature_Reader temperature_reader;					//create temperature reader struct
	init_temp_reader(&temperature_reader);								//initialize temperature reader struct
	
	while(1){
		osSignalWait(0x0001, osWaitForever);								//waiting on hw TIM3 to tell us to sample from the temperature sensor
		read_temp(&temperature_reader);											//if it's time to sample, pull values into temperature reader struct
		
		osMutexWait(mode_mutex, osWaitForever);							//wait on mode mutex
		current_mode = mode;																//set current mode to mode
		osMutexRelease(mode_mutex);													//release mode mutex
		
		if (current_mode == 0) {														//sampling temp sensor at a constant rate, but if we're in mode 0 we want to display the temperature LED rotation
			uint32_t led_number = (uint32_t)temperature_reader.moving_average.average % 8; 		//Determine which LED should be on depending on temperature
			osMutexWait(led_mutex, osWaitForever);						//wait on LED mutex
			rotate_led(led_number);														//update LEDs based on temp
			osMutexRelease(led_mutex);												//release LED mutex
		}
	}
}

/**
  * @brief  Thread triggered by tap detection interrupt
	* @param  None
  * @retval None
  */
void tap_thread(void const *arguments) {								//thread for tap detection and mode switch based on current mode
	while(1) {
		osSignalWait(0x0001, osWaitForever);								//waiting on EXTI1 interrupt for tap detection flag (signal)
		osMutexWait(mode_mutex, osWaitForever);							//wait on mode mutex
		osMutexWait(led_mutex, osWaitForever);							//wait on LED mutex
		switch (mode) {																			//using mode to control case selection
			case 0:																						//if we are in temperature display (rotating LED) mode
				mode = 1;																				//switch to mode 1 (accelerometer tilt angle display)
				init_LEDS_HW_PWM();															//configure LEDs to function with Alternate Function mode for TIM4 hw ccr pwm
				break;																					//end
			case 1:																						//if we are in accelerometer tilt angle display mode
				mode = 0;																				//switch to temperature rotate LED display mode
				init_LEDS();																		//initialize LEDs with GPIO_Mode_In for rotate led mode
				break;																					//end
			case 2:																						//if we are in sw flashing mode
				mode = 3;																				//switch to hw pwm mode
				init_LEDS_HW_PWM();															//once again, re-initialize LEDs with TIM4 hw pwm Alternate Function
				break;																					//end
			default:																					//default state
				mode = 2;																				//go to sw flashing mode
				init_LEDS();																		//initialize LEDs for GPIO_Mode_In
		}
		osMutexRelease(led_mutex);													//release LED mutex
		osMutexRelease(mode_mutex);													//release mode mutex
	}
}

/**
  * @brief  Thread triggered by pushbutton interrupt
	* @param  None
  * @retval None
  */
void pushbutton_thread(void const *arguments) {					//pushbutton interrupt thread for sub mode switch
	while(1) {
		osSignalWait(0x0001, osWaitForever);								//waiting on pushbutton interrupt handler to send signal that we're switch submode
		osMutexWait(mode_mutex, osWaitForever);							//wait on mode mutex
		switch (mode) {																			//mode determines case selection
			case 0:																						//if we are in temperature display (rotating LED) mode
				mode = 2;																				//switch to LED flashing mode
				break;																					//end
			case 1:																						//if we are in accelerometer tilt angle display mode
				mode = 3;																				//switch to hw pwm mode
				break;																					//end
			case 2:																						//if we are in LED flashing mode
				mode = 0;																				//switch to temperature rotate LED mode
				break;																					//end
			default:																					//default mode case
				mode = 1;																				//accelerometer tilt angle display mode
		}			
		osMutexRelease(mode_mutex);													//release mode mutex
		osDelay(10);																				//delay 10ms (debounce)
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {	//reading pushbutton input
			osDelay(10);																			//delay 10ms (debounce)
		}
		osSignalClear(tid_pushbutton_thread, 0x0001);				//clear signal, safety
	}
}

/**
  * @brief  Thread triggered by pushbutton interrupt from temperature display mode.
	* @param  None
  * @retval None
  */
void sw_pwm_thread(void const *arguments) {							//sw flashing LED mode
	uint32_t current_mode;																//local variable to hold mode
	uint32_t state;																				//local variable for current state (LEDs on/off)
	
	while(1) {
		osMutexWait(mode_mutex, osWaitForever);							//wait on mode mutex
		current_mode = mode;																//set current mode = mode
		osMutexRelease(mode_mutex);													//release mode mutex
		
		if (current_mode == 2) {														//if we are in sw LED flashing mode
			state ^= 1;																				//bitwise or state
			if (state) {																			//if state==1
				osMutexWait(led_mutex, osWaitForever);					//wait on LED mutex
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);							//turn pin 12 LED off
				GPIO_ResetBits(GPIOD, GPIO_Pin_13);							//turn pin 13 LED off
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);							//turn pin 14 LED off
				GPIO_ResetBits(GPIOD, GPIO_Pin_15);							//turn pin 15 LED off
				osMutexRelease(led_mutex);											//release LED mutex
			} else {																					//if state==0
				osMutexWait(led_mutex, osWaitForever);					//wait on LED access
				GPIO_SetBits(GPIOD, GPIO_Pin_12);								//turn pin 12 LED on
				GPIO_SetBits(GPIOD, GPIO_Pin_13);								//turn pin 13 LED on
				GPIO_SetBits(GPIOD, GPIO_Pin_14);								//turn pin 14 LED on
				GPIO_SetBits(GPIOD, GPIO_Pin_15);								//turn pin 15 LED on
				osMutexRelease(led_mutex);											//release LED mutex
			}
		}
		osDelay(500);																				//sw delay, 500ms flash, time spent off, then on etc.
	}
}

/**
  * @brief  Thread triggered by pushbutton interrupt from accelerometer tilt angle display mode
  * @retval None
  */
void hw_pwm_thread(void const *arguments) {							//hw pwm thread
	uint32_t current_mode;																//local current mode variable
	uint32_t intensity;																		//local variable to track LED intensity for pwm
	uint32_t direction;																		//local variable tracking whether LEDs are getting brighter (0) or darker(1)
	
	while(1) {
		osSignalWait(0x0001, osWaitForever);								//waiting on TIM5 interrupt to indicate time to change intensity
		
		osMutexWait(mode_mutex, osWaitForever);							//wait on mode mutex access
		current_mode = mode;																//set current mode to mode
		osMutexRelease(mode_mutex);													//release mode mutex
		
		if (current_mode == 3) {														//if we are in fact in hw pwm mode
			if (intensity == MAX_PWM_INTENSITY) {							//check if we're at max intensity for the LEDs
				direction = 1;																	//set LED direction to be decreasing
			} else if (intensity == 0) {											//check if LEDs are completely off
				direction = 0;																	//set direction to be increasing, getting brighter
			}
			if (direction) {																	//if we're supposed to be decreasing intensity
				intensity--;																		//decrease intensity
			} else {																					//if we're supposed to be increasing intensity
				intensity++;																		//increase intensity
			}
			osMutexWait(led_mutex, osWaitForever);						//wait on LED mutex access
			TIM4->CCR1 = intensity;														//set intensity for pin 12 LED via CCR1
			TIM4->CCR2 = intensity;														//set intensity for pin 13 LED via CCR2
			TIM4->CCR3 = intensity;														//set intensity for pin 14 LED via CCR3
			TIM4->CCR4 = intensity;														//set intensity for pin 15 LED via CCR4
			osMutexRelease(led_mutex);												//release LED mutex
		}
	}
}

/**
  * @brief  IRQ handler for TIM2
	* @param  None
  * @retval None
  */

void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);																//Reset interrupt pending bit
		osSignalSet(tid_accelerometer_thread, 0x0001);														//send signal to accelerometer thread, time to sample and display tilt angle
  }
}

/**
  * @brief  IRQ handler for TIM3
	* @param  None
  * @retval None
  */
void TIM3_IRQHandler(void) {
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);																//Reset interrupt pending bit
		osSignalSet(tid_temperature_thread, 0x0001);															//send signal to temperature sensor thread, time to sample display/rotate LEDs
  }
}

/**
  * @brief  IRQ handler for TIM4
	* @param  None
  * @retval None
  */
void TIM4_IRQHandler(void) {
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);																//Reset interrupt pending bit
  }
}

/**
  * @brief  IRQ handler for TIM5
	* @param  None
  * @retval None
  */
void TIM5_IRQHandler(void) {
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);																//Reset interrupt pending bit
		osSignalSet(tid_hw_pwm_thread, 0x0001);																		//send signal to hw pwm to update LED intensity
  }
}

/**
  * @brief  IRQ handler for EXTI0
	* @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)																		//Checks interrupt status register to ensure an interrupt is pending
  {
    EXTI_ClearITPendingBit(EXTI_Line0);																				//Reset interrupt pending bit
		osSignalSet(tid_pushbutton_thread, 0x0001);																//send signal to pusbutton thread to switch submodes
  }
}

/**
  * @brief  IRQ handler for EXTI1
	* @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)																		//Checks interrupt status register to ensure an interrupt is pending
  {
    EXTI_ClearITPendingBit(EXTI_Line1);																				//Reset interrupt pending bit
		osSignalSet(tid_tap_thread, 0x0001);																			//send signal to tap detection thread to signal to switch modes
  }
}

/**
  * @}
  */