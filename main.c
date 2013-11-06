#include "arm_math.h"

#include "stm32f4xx.h"
#include "cmsis_os.h"

#include "lab4_init.h"
#include "lab4_accelerometer.h"
#include "lab4_temp.h"
#include "lab4_hw_pwm.h"

struct Mode_Arguments {
	struct Orientation orientation;
	struct Temperature_Reader temperature_reader;
};

/*!
 @brief Thread to perform menial tasks such as switching LEDs
 @param argument Unused
 */
void accelerometer_thread(void const *argument);
void temperature_thread(void const *argument);
void mode_thread(void const *argument);
void pushbutton_thread(void const *argument);
void sw_pwm_thread(void const *argument);
void hw_pwm_thread(void const *argument);

void temperature_timer_callback(void const *argument);

//! Thread structure for above thread
osThreadDef(accelerometer_thread, osPriorityNormal, 1, 0);
osThreadDef(temperature_thread, osPriorityNormal, 1, 0);
osThreadDef(mode_thread, osPriorityNormal, 1, 0);
osThreadDef(pushbutton_thread, osPriorityNormal, 1, 0);
osThreadDef(sw_pwm_thread, osPriorityNormal, 1, 0);
osThreadDef(hw_pwm_thread, osPriorityNormal, 1, 0);

osMutexDef(orientation_mutex);
osMutexDef(temperature_reader_mutex);
osMutexDef(mode_mutex);
osMutexDef(sw_pwm_mutex);
osMutexDef(hw_pwm_mutex);

// ID for thread
static osThreadId tid_accelerometer_thread;
static osThreadId tid_temperature_thread;
static osThreadId tid_mode_thread;
static osThreadId tid_pushbutton_thread;
static osThreadId tid_sw_pwm_thread;
static osThreadId tid_hw_pwm_thread;

static osTimerId temperature_timer;

static osMutexId orientation_mutex;
static osMutexId temperature_reader_mutex;
static osMutexId mode_mutex;
static osMutexId sw_pwm_mutex;
static osMutexId hw_pwm_mutex;

osTimerDef(temperature_timer, temperature_timer_callback);

static uint32_t mode;
static uint32_t sw_pwm;
static uint32_t hw_pwm;

/*!
 @brief Program entry point
 */
int main (void) {
	struct Mode_Arguments mode_arguments;
	init_orientation(&mode_arguments.orientation);
	init_temp_reader(&mode_arguments.temperature_reader);
	
	init_accelerometer();
	init_TIM2();
	init_adc();
	init_temp_sensor();
	init_pushbutton();
	init_LEDS();
	init_TIM4();
	init_EXTI();
	
	SysTick_Config(SystemCoreClock / 20);
	
	orientation_mutex = osMutexCreate(osMutex(orientation_mutex));
	temperature_reader_mutex = osMutexCreate(osMutex(temperature_reader_mutex));
	mode_mutex = osMutexCreate(osMutex(mode_mutex));
	sw_pwm_mutex = osMutexCreate(osMutex(sw_pwm_mutex));
	hw_pwm_mutex = osMutexCreate(osMutex(hw_pwm_mutex));

	// Start thread
	tid_accelerometer_thread = osThreadCreate(osThread(accelerometer_thread), &mode_arguments.orientation);
	tid_temperature_thread = osThreadCreate(osThread(temperature_thread), &mode_arguments.temperature_reader);
	tid_mode_thread = osThreadCreate(osThread(mode_thread), &mode_arguments);
	tid_pushbutton_thread = osThreadCreate(osThread(pushbutton_thread), NULL);
	tid_sw_pwm_thread = osThreadCreate(osThread(sw_pwm_thread), NULL);
	//tid_hw_pwm_thread = osThreadCreate(osThread(hw_pwm_thread), NULL);
	
	temperature_timer = osTimerCreate(osTimer(temperature_timer), osTimerPeriodic, NULL);
	osTimerStart(temperature_timer, 5);

	// The below doesn't really need to be in a loop
	while(1){
		osDelay(osWaitForever);
	}
}

void accelerometer_thread(void const *orientation) {
	while(1){
		osSignalWait(0x0001, osWaitForever);
		osMutexWait(orientation_mutex, osWaitForever);
		update_orientation((struct Orientation *)orientation);
		osMutexRelease(orientation_mutex);
		osSignalSet(tid_mode_thread, 0x0001);
	}
}

void temperature_thread(void const *temperature_reader) {
	while(1){
		osSignalWait(0x0001, osWaitForever);
		osMutexWait(temperature_reader_mutex, osWaitForever);
		read_temp((struct Temperature_Reader *)temperature_reader);
		osMutexRelease(temperature_reader_mutex);
		osSignalSet(tid_mode_thread, 0x0002);
	}
}

void mode_thread(void const *mode_arguments) {
	uint32_t current_mode;
	uint32_t direction;
	uint32_t intensity;
	uint32_t previous_mode;
	uint32_t is_initialized;
	while(1) {
		osMutexWait(mode_mutex, osWaitForever);
		current_mode = mode;
		osMutexRelease(mode_mutex);
		
		if (current_mode != previous_mode) {
			is_initialized = 0;
		}
		
		if (current_mode == 0) {
			osSignalWait(0x0002, osWaitForever);
			if (!is_initialized) {
				init_LEDS();
			}
			osMutexWait(temperature_reader_mutex, osWaitForever);
			uint32_t led_number = ((uint32_t)(((struct Mode_Arguments *)mode_arguments)->temperature_reader.moving_average.average) % 8); 		//Determine which LED should be on depending on temperature
			rotate_led(led_number); 																																//Turn on correct (rotating) LED
			osMutexRelease(temperature_reader_mutex);
		} else if (current_mode == 1) {
			osSignalWait(0x0001, osWaitForever);
			if (!is_initialized) {
				init_LEDS_HW_PWM();
			}
			osMutexWait(orientation_mutex, osWaitForever);
			display_orientation(&((struct Mode_Arguments *)mode_arguments)->orientation);
			osMutexRelease(orientation_mutex);
		} else if (current_mode == 2) {
			if (!is_initialized) {
				init_LEDS();
			}
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
		} else {
			if (!is_initialized) {
				init_LEDS_HW_PWM();
			}
			if (!direction) {
				intensity++;
				TIM4->CCR1 = intensity;
				TIM4->CCR2 = intensity;
				TIM4->CCR3 = intensity;
				TIM4->CCR4 = intensity;
				if (intensity == MAX_PWM_INTENSITY) {
					direction = 1;
				}
			} else {
				intensity--;
				TIM4->CCR1 = intensity;
				TIM4->CCR2 = intensity;
				TIM4->CCR3 = intensity;
				TIM4->CCR4 = intensity;
				if (intensity == 0) {
					direction = 0;
				}
			}
		}
		previous_mode = current_mode;
		is_initialized = 1;
		osDelay(1);
	}
}

void pushbutton_thread(void const *arguments) {
	while(1) {
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
			osDelay(2);
			if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
				mode ^= 2;
				while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
					osDelay(2);
				}
			}
		}
		osDelay(2);
	}
}

void sw_pwm_thread(void const *arguments) {
}

void hw_pwm_thread(void const *arguments) {

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
		osSignalSet(tid_accelerometer_thread, 0x0001);
  }
}

void TIM4_IRQHandler(void) {
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);																//Reset interrupt pending bit
  }
}

void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)																		//Checks interrupt status register to ensure an interrupt is pending
  {
    EXTI_ClearITPendingBit(EXTI_Line1);																				//Reset interrupt pending bit
		mode ^= 1;
  }
}

void temperature_timer_callback(void const *argument) {
	osSignalSet(tid_temperature_thread, 0x0001);
}
