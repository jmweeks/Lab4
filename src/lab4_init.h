#ifndef MAX_PWM_INTENSITY
#define MAX_PWM_INTENSITY 500																													//Number of steps between fully off and fully on
#endif

#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 500																															//Driving PWM frequency (Hz)
#endif

#ifndef PWM_PULSE_SPEED
#define PWM_PULSE_SPEED 2000																													//Time of a complete pulse cycle, off to off (in ms)
#endif

#ifndef PWM_UPDATE_INTENSITY_FREQUENCY
#define PWM_UPDATE_INTENSITY_FREQUENCY 5*MAX_PWM_INTENSITY														//Frequency at which to update intensity
#endif

#ifndef TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE
#define TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE 16
#endif

#if TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE > MAX_MOVING_AVERAGE_FILTER_SIZE
#undef TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE
#define TEMPERATURE_MOVING_AVERAGE_FILTER_SIZE MAX_MOVING_AVERAGE_FILTER_SIZE
#endif

#ifndef ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE
#define ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE 10
#endif

#if ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE > MAX_MOVING_AVERAGE_FILTER_SIZE
#undef ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE
#define ACCELEROMETER_MOVING_AVERAGE_FILTER_SIZE MAX_MOVING_AVERAGE_FILTER_SIZE
#endif

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
