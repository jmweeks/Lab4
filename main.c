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
void accelerometer_thread(void const *argument);
void temperature_thread(void const *argument);
void tap_thread(void const *argument);
void pushbutton_thread(void const *argument);
void sw_pwm_thread(void const *argument);
void hw_pwm_thread(void const *argument);

//! Thread structure for above thread
osThreadDef(accelerometer_thread, osPriorityNormal, 1, 0);
osThreadDef(temperature_thread, osPriorityNormal, 1,0);
osThreadDef(tap_thread, osPriorityNormal, 1, 0);
osThreadDef(pushbutton_thread, osPriorityNormal, 1, 0);
osThreadDef(sw_pwm_thread, osPriorityNormal, 1, 0);
osThreadDef(hw_pwm_thread, osPriorityNormal, 1, 0);

osMutexDef(led_mutex);
osMutexDef(mode_mutex);

// ID for thread
static osThreadId tid_accelerometer_thread;
static osThreadId tid_temperature_thread;
static osThreadId tid_tap_thread;
static osThreadId tid_pushbutton_thread;
static osThreadId tid_sw_pwm_thread;
static osThreadId tid_hw_pwm_thread;

static osMutexId led_mutex;
static osMutexId mode_mutex;

static uint32_t mode;

/*!
 @brief Program entry point
 */
int main (void) {
	init_accelerometer();
	init_TIM2();
	init_TIM3();
	init_adc();
	init_temp_sensor();
	init_pushbutton();
	init_LEDS();
	init_TIM4();
	init_EXTI1();
	init_TIM5();
	
	led_mutex = osMutexCreate(osMutex(led_mutex));
	mode_mutex = osMutexCreate(osMutex(mode_mutex));

	// Start thread
	tid_accelerometer_thread = osThreadCreate(osThread(accelerometer_thread), NULL);
	tid_temperature_thread = osThreadCreate(osThread(temperature_thread), NULL);
	tid_tap_thread = osThreadCreate(osThread(tap_thread), NULL);
	tid_pushbutton_thread = osThreadCreate(osThread(pushbutton_thread), NULL);
	tid_sw_pwm_thread = osThreadCreate(osThread(sw_pwm_thread), NULL);
	tid_hw_pwm_thread = osThreadCreate(osThread(hw_pwm_thread), NULL);

	// The below doesn't really need to be in a loop
	while(1){
		osDelay(osWaitForever);
	}
}

void accelerometer_thread(void const *argument) {
	uint32_t current_mode;
	struct Orientation orientation;
	init_orientation(&orientation);
	
	while(1){
		osSignalWait(0x0001, osWaitForever);
		update_orientation(&orientation);
		
		osMutexWait(mode_mutex, osWaitForever);
		current_mode = mode;
		osMutexRelease(mode_mutex);
		
		if (current_mode == 1) {
			osMutexWait(led_mutex, osWaitForever);
			display_orientation(&orientation);
			osMutexRelease(led_mutex);
		}
	}
}

void temperature_thread(void const *argument) {
	uint32_t current_mode;
	struct Temperature_Reader temperature_reader;
	init_temp_reader(&temperature_reader);
	
	while(1){
		osSignalWait(0x0001, osWaitForever);
		read_temp(&temperature_reader);
		
		osMutexWait(mode_mutex, osWaitForever);
		current_mode = mode;
		osMutexRelease(mode_mutex);
		
		if (current_mode == 0) {
			uint32_t led_number = (uint32_t)temperature_reader.moving_average.average % 8; 		//Determine which LED should be on depending on temperature
			osMutexWait(led_mutex, osWaitForever);
			rotate_led(led_number);
			osMutexRelease(led_mutex);
		}
	}
}

void tap_thread(void const *arguments) {
	while(1) {
		osSignalWait(0x0001, osWaitForever);
		osMutexWait(mode_mutex, osWaitForever);
		osMutexWait(led_mutex, osWaitForever);
		switch (mode) {
			case 0:
				mode = 1;
				init_LEDS_HW_PWM();
				break;
			case 1:
				mode = 0;
				init_LEDS();
				break;
			case 2:
				mode = 3;
				init_LEDS_HW_PWM();
				break;
			default:
				mode = 2;
				init_LEDS();
		}
		osMutexRelease(led_mutex);
		osMutexRelease(mode_mutex);
	}
}

void pushbutton_thread(void const *arguments) {
	while(1) {
		osSignalWait(0x0001, osWaitForever);
		osMutexWait(mode_mutex, osWaitForever);
		switch (mode) {
			case 0:
				mode = 2;
				break;
			case 1:
				mode = 3;
				break;
			case 2:
				mode = 0;
				break;
			default:
				mode = 1;
		}
		osMutexRelease(mode_mutex);
		osDelay(10);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
			osDelay(10);
		}
		osSignalClear(tid_pushbutton_thread, 0x0001);
	}
}

void sw_pwm_thread(void const *arguments) {
	uint32_t current_mode;
	uint32_t state;
	
	while(1) {
		osMutexWait(mode_mutex, osWaitForever);
		current_mode = mode;
		osMutexRelease(mode_mutex);
		
		if (current_mode == 2) {
			state ^= 1;
			if (state) {
				osMutexWait(led_mutex, osWaitForever);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				GPIO_ResetBits(GPIOD, GPIO_Pin_15);
				osMutexRelease(led_mutex);
			} else {
				osMutexWait(led_mutex, osWaitForever);
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				GPIO_SetBits(GPIOD, GPIO_Pin_13);
				GPIO_SetBits(GPIOD, GPIO_Pin_14);
				GPIO_SetBits(GPIOD, GPIO_Pin_15);
				osMutexRelease(led_mutex);
			}
		}
		osDelay(500);
	}
}

void hw_pwm_thread(void const *arguments) {
	uint32_t current_mode;
	uint32_t intensity;
	uint32_t direction;
	
	while(1) {
		osSignalWait(0x0001, osWaitForever);
		
		osMutexWait(mode_mutex, osWaitForever);
		current_mode = mode;
		osMutexRelease(mode_mutex);
		
		if (current_mode == 3) {
			if (intensity == MAX_PWM_INTENSITY) {
				direction = 1;
			} else if (intensity == 0) {
				direction = 0;
			}
			if (direction) {
				intensity--;
			} else {
				intensity++;
			}
			osMutexWait(led_mutex, osWaitForever);
			TIM4->CCR1 = intensity;
			TIM4->CCR2 = intensity;
			TIM4->CCR3 = intensity;
			TIM4->CCR4 = intensity;
			osMutexRelease(led_mutex);
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
		osSignalSet(tid_accelerometer_thread, 0x0001);
  }
}

void TIM3_IRQHandler(void) {
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);																//Reset interrupt pending bit
		osSignalSet(tid_temperature_thread, 0x0001);
  }
}

void TIM4_IRQHandler(void) {
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);																//Reset interrupt pending bit
  }
}

void TIM5_IRQHandler(void) {
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)													//Checks interrupt status register to ensure an interrupt is pending
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);																//Reset interrupt pending bit
		osSignalSet(tid_hw_pwm_thread, 0x0001);
  }
}

void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)																		//Checks interrupt status register to ensure an interrupt is pending
  {
    EXTI_ClearITPendingBit(EXTI_Line0);																				//Reset interrupt pending bit
		osSignalSet(tid_pushbutton_thread, 0x0001);
  }
}

void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)																		//Checks interrupt status register to ensure an interrupt is pending
  {
    EXTI_ClearITPendingBit(EXTI_Line1);																				//Reset interrupt pending bit
		osSignalSet(tid_tap_thread, 0x0001);
  }
}
