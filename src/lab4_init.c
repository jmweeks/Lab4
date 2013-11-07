#include "stm32f4xx.h"
#include "stm32f4_discovery_lis302dl.h"

#include "lab4_init.h"

/**
  * @brief  Initialize TIM2 for accelerometer sampling
	* @param  None
  * @retval None
  */

void init_TIM2() {
  NVIC_InitTypeDef NVIC_InitStructure;																															//Create NVIC struct for holding parameters
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;																										//Create TIM struct for holding timer parameters
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 																							//Enable clock to TIM2
	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;																										//No clock division
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;																			//Count down
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;																												//Max period available (2^16-1)
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/(2*25*TIM_TimeBaseStructure.TIM_Period)-1;	//Set prescaler, clock now at sample rate of 25Hz
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;																								//Once counter reaches this value we restart RCR count
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);																										
	
	TIM_Cmd(TIM2, ENABLE);																																						//Enable TIM2 peripheral
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);																												//Enable new interrupt state
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;																										//Specify interrupt request channel to be used
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 																			//Indicates pre-emption priority, 0-15, lower # =higher prriority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 																						//Subpriority value 0-15, lower # =higher prriority
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 																									//Enable interrupt request channel specified earlier
  NVIC_Init(&NVIC_InitStructure);																																		//Initialize struct parameters into tim2 nvic
}

void init_TIM3() {
  NVIC_InitTypeDef NVIC_InitStructure;																															//Create NVIC struct for holding parameters
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;																										//Create TIM struct for holding timer parameters
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 																							//Enable clock to TIM2
	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;																										//No clock division
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;																			//Count down
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;																												//Max period available (2^16-1)
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/(2*20*TIM_TimeBaseStructure.TIM_Period)-1;	//Set prescaler, clock now at sample rate of 25Hz
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;																								//Once counter reaches this value we restart RCR count
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);																										
	
	TIM_Cmd(TIM3, ENABLE);																																						//Enable TIM2 peripheral
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);																												//Enable new interrupt state
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;																										//Specify interrupt request channel to be used
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 																			//Indicates pre-emption priority, 0-15, lower # =higher prriority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 																						//Subpriority value 0-15, lower # =higher prriority
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 																									//Enable interrupt request channel specified earlier
  NVIC_Init(&NVIC_InitStructure);																																		//Initialize struct parameters into tim2 nvic
}

/**
  * @brief  Initialize TIM4 as PWM clock, using output compare
	* @param  None
  * @retval None
  */

void init_TIM4() {
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);																			//Clock enable to TIM4
	
	TIM_TimeBaseStructure.TIM_Period = MAX_PWM_INTENSITY;																															//Period = max intensity for pwm
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / (2 * PWM_FREQUENCY * TIM_TimeBaseStructure.TIM_Period);		//Set prescaler for LED function
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;																																			//No clock divison
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;																												//Count up

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;																					//Set Output Compare mode for TIM4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;															//Enable output
	TIM_OCInitStructure.TIM_Pulse = 0;																												//Set initial CCR value
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;																	//Set output active high

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);																									//Initialize TIM4 OC1
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);																					//Enables TIM4 peripheral Preload register on CCR1
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);																									//Initialize TIM4 OC2
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);																					//Enables TIM4 peripheral Preload register on CCR2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);																									//Initialize TIM4 OC3
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);																					//Enables TIM4 peripheral Preload register on CCR3
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);																									//Initialize TIM4 OC4
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);																					//Enables TIM4 peripheral Preload register on CCR4
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);																												//Enable TIM4 Auto Reload Register 

  TIM_Cmd(TIM4, ENABLE);																																		//Enable TIM4
}

/**
  * @brief  Initialize USER pushbutton
	* @note   This function sets up the input GPIO for the USER pushbutton.
	*         The GPIO speed and direction are set.
	* @param  None
  * @retval None
  */

void init_pushbutton() {
	GPIO_InitTypeDef GPIO_InitStructure;																															//Initialize GPIO structure
	EXTI_InitTypeDef EXTI_InitStructure;																															//Initialize external interrupt structure
	NVIC_InitTypeDef NVIC_InitStructure;																															//Initialize nested vector interrupt controller structure
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);																						//Enable apb2 clock to syscfg, enable external interrupts
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 														//Enable clock to Pushbutton
	
	GPIO_StructInit(&GPIO_InitStructure);																										//Initialize struct, reset to default values
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;																								//Specify which pin to configure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;																							//Takes input data
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;																				//Specify clock speed for pin
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;																						//Specify operating output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;																						//Specify resistor pull down
	GPIO_Init(GPIOA, &GPIO_InitStructure);			

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);																			//Connect/configure external interrupt on pin1 to be connected with gpioe
	
	EXTI_StructInit(&EXTI_InitStructure);																															//Reset external interrupt structure
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;																												//Ext interrupt coming in on line 1
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;																								//Set mode of the interrupt, external
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  																					//Ext interrupt triggered on a rising edge
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;																													//Enable new state of the interrupt
  EXTI_Init(&EXTI_InitStructure);																																		//Configure external interrupt with defined structure parameters
	
	/*The Nested Vectored Interrupt Controller (NVIC) offers very fast interrupt handling and provides the vector table as a set of real vectors (addresses).
	-Saves and restores automatically a set of the CPU registers (R0-R3, R12, PC, PSR, and LR).
	-Does a quick entry to the next pending interrupt without a complete pop/push sequence. (Tail-Chaining)
	-Serves a complete set of 255 (240 external) interrupts.
	*/
	
	// Preemption Priority = used to determine if an interrupt that occurs after can overtake
	// previous interrupt that is currently being serviced
	// SubPriority = used to determine priority if two interrupts occur at the same time
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;																									//Enable interrupt request channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;																			//Set priority of pre-emption interrupt
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;																							//Set sub priority
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;																										//Enable the IRQ channel specified by nvic
  NVIC_Init(&NVIC_InitStructure);																																		//Pass struct to NVIC, initialize
}

/**
  * @brief  Initialize EXTI for tap detection
	* @param  None
  * @retval None
  */

void init_EXTI1() {
	GPIO_InitTypeDef GPIO_InitStructure;																															//Initialize GPIO structure
	EXTI_InitTypeDef EXTI_InitStructure;																															//Initialize external interrupt structure
	NVIC_InitTypeDef NVIC_InitStructure;																															//Initialize nested vector interrupt controller structure
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);																						//Enable apb2 clock to syscfg, enable external interrupts
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);																							//Enable ahb1 clock to GPIOE peripherals
	
	GPIO_StructInit(&GPIO_InitStructure);																															//Define GPIO struct parameters
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;																													//Assign pin 1 of the GPIO set to our struct
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;																											//We want these pins to be outputs
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;																									//Clock freq to pin
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;																										//Push-pull, instead of open drain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;																									//Sets pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStructure);																														//Initialize GPIO with struct parameters

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);																			//Connect/configure external interrupt on pin1 to be connected with gpioe
	
	EXTI_StructInit(&EXTI_InitStructure);																															//Reset external interrupt structure
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;																												//Ext interrupt coming in on line 1
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;																								//Set mode of the interrupt, external
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  																					//Ext interrupt triggered on a rising edge
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;																													//Enable new state of the interrupt
  EXTI_Init(&EXTI_InitStructure);																																		//Configure external interrupt with defined structure parameters
	
	/*The Nested Vectored Interrupt Controller (NVIC) offers very fast interrupt handling and provides the vector table as a set of real vectors (addresses).
	-Saves and restores automatically a set of the CPU registers (R0-R3, R12, PC, PSR, and LR).
	-Does a quick entry to the next pending interrupt without a complete pop/push sequence. (Tail-Chaining)
	-Serves a complete set of 255 (240 external) interrupts.
	*/
	
	// Preemption Priority = used to determine if an interrupt that occurs after can overtake
	// previous interrupt that is currently being serviced
	// SubPriority = used to determine priority if two interrupts occur at the same time
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;																									//Enable interrupt request channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;																			//Set priority of pre-emption interrupt
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;																							//Set sub priority
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;																										//Enable the IRQ channel specified by nvic
  NVIC_Init(&NVIC_InitStructure);																																		//Pass struct to NVIC, initialize
}

/**
  * @brief  Initalize accelerometer for use as a tilt angle sensor with tap detection
	* @param  None
  * @retval None
  */

void init_accelerometer() {
	LIS302DL_InitTypeDef LIS302DL_InitStruct; 																																		//Struct for accelerometer initialization
	
	uint8_t int_ctrl_reg_value = 0x38;																																						//(0011 1000) Active high, push-pull, click interrupt, data ready
	uint8_t click_cfg_reg_value = 0x10;																																						//(0001 0000) Set interrupt on single Z click, don't latch interrupt signal high
	
	uint8_t click_window_reg_value = 0x7F;																																				//(0111 1111) Max time interval after end of latency interval where click detection can start again if device configured for double click detect
	uint8_t click_latency_reg_value = 0x7F;																																				//(0111 1111) Time interval where click detection is disabled after first click, if configured for double click detection
	uint8_t click_time_limit_reg_value = 0x03;																																		//(0000 0011) Maximum time interval that can elapse between the start of the click detect procedure and when the accel goes back below threshold
	uint8_t click_threshold_z_value = 0x0F;																																				//(0000 1010) Z-axis sensitivity threshold
	uint8_t click_threshold_xy_value = 0xFF;																																			//(0000 1010) X&Y-axis sensitivity thresholld
	
	//Accelerometer Configuration
	LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE; 																								//Either lowpower on or off
	LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100; 																									//Options are 100Hz or 400Hz, we are sampling at 25Hz, no sense using faster freq
	LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE; 									//Turn on all 3 axes
	LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3; 																											//Define typical range of measured values, doubtfull we'll hit 9g
	LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL; 																										//Off, otherwise apply actuation force, sensors change their DC levels (way to make sure everything is working)
	LIS302DL_Init(&LIS302DL_InitStruct);																																					//Apply initialization settings to accelerometer
	
	//Advanced accelerometer configuration
	LIS302DL_Write(&int_ctrl_reg_value, LIS302DL_CTRL_REG3_ADDR, sizeof(int_ctrl_reg_value));											//Enable click interrupt on INT0
	LIS302DL_Write(&click_cfg_reg_value, LIS302DL_CLICK_CFG_REG_ADDR, sizeof(click_cfg_reg_value));								//Configure single click detection
  LIS302DL_Write(&click_threshold_xy_value, LIS302DL_CLICK_THSY_X_REG_ADDR, 1); 	    													//Configure Click Threshold on X/Y axis (10 x 0.5g)
  LIS302DL_Write(&click_threshold_z_value, LIS302DL_CLICK_THSZ_REG_ADDR, 1);																		//Configure Click Threshold on Z axis (10 x 0.5g)
  LIS302DL_Write(&click_time_limit_reg_value, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);														//Configure Time Limit
  LIS302DL_Write(&click_latency_reg_value, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);																	//Configure Latency
  LIS302DL_Write(&click_window_reg_value, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);    																//Configure Click Window
}

/**
  * @brief  Initialize LEDs for use with TIM4 OC hardware PWM
	* @param  None
  * @retval None
  */

void init_LEDS_HW_PWM() {
	GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);																			//Enable clock to GPIOD
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;			//Specify which LEDs, on which pins
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;																							//Aternate function mode to work with TIM4
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;																					//Set speed
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;																						//Push-pull
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;																					//No pull - output
  GPIO_Init(GPIOD, &GPIO_InitStructure);																										//Pass struct and initialize
 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);																	//Alternate function configuration for GPIO poins, hardware link to TIM4
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); 
}

void init_LEDS() {
	GPIO_InitTypeDef gpio_init_s;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 															//Enable clock to LEDs
	
	GPIO_StructInit(&gpio_init_s);
	gpio_init_s.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 			//Initialize 4 LEDs for use, attached to these pins
	gpio_init_s.GPIO_Mode = GPIO_Mode_OUT;																							//we want these pins to be outputs
	gpio_init_s.GPIO_Speed = GPIO_Speed_50MHz;																					//clock freq to pin
	gpio_init_s.GPIO_OType = GPIO_OType_PP;																							//push-pull, instead of open drain
	gpio_init_s.GPIO_PuPd = GPIO_PuPd_NOPULL;																						//sets pulldown resistors to be inactive
	GPIO_Init(GPIOD, &gpio_init_s);																											// Initializes the peripherals with the specified params
}

/**
  * @brief  Initialize ADC
	* @note   This function sets up all ADCs for use and configures ADC1 for
	*         reading the built-in temperature sensor.
	* @param  None
  * @retval None
  */

void init_adc() {
	ADC_InitTypeDef adc_init_s;
	ADC_CommonInitTypeDef adc_common_init_s;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 																//Enable clock to ADC
										
	adc_common_init_s.ADC_Mode = ADC_Mode_Independent;																	//indp. mode, operates independently from ADC2/3, no simultaneous conversions
	adc_common_init_s.ADC_Prescaler = ADC_Prescaler_Div2;																//set clock freq for adc
	adc_common_init_s.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;										//not in multi mode, dont need direct memory access, wont affect performance significantly
	adc_common_init_s.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;							//delay between two sampling phases (5 cycles is minimum allowed)
	ADC_CommonInit(&adc_common_init_s);																									//points to struct that holds ADC config info for peripherals
	
	adc_init_s.ADC_Resolution = ADC_Resolution_12b;
	adc_init_s.ADC_ScanConvMode = DISABLE;																							//disable so we do one at a time
	adc_init_s.ADC_ContinuousConvMode = DISABLE;																				//Only poll once, when we want to
	adc_init_s.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;								//internal trigger via software, not external trigger
	adc_init_s.ADC_DataAlign = ADC_DataAlign_Right;																			//12 bit adc value pushed into 16 bit register, specify left or right align
	adc_init_s.ADC_NbrOfConversion = 1;																									//only using 1 ADC, indicates number of conversion to be done by sequencer
	ADC_Init(ADC1, &adc_init_s);
	
	ADC_Cmd(ADC1, ENABLE); //Enable ADC
}

/**
  * @brief  Initialize built-in temperature sensor
	* @note   This function initializes the built in temperature sensor on
	*         ADC1 at channel ADC_Channel_16 for use
	* @param  None
  * @retval None
  */

void init_temp_sensor() {
	ADC_TempSensorVrefintCmd(ENABLE); //Enable temperature sensor
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_480Cycles); 				//Configure temperature sensor channel,  
																																											//rank (1 since in ind. mode, in case of group regular channel conv), and the sample freq.
}

void init_TIM5() {
  NVIC_InitTypeDef NVIC_InitStructure;																											//NVIC initialization struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;																						//Timer initialization struct
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 																			//Enable clock to TIM5
	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;																																											//No clock division
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;																																					//Counts up
	TIM_TimeBaseStructure.TIM_Period = 0x007F;																																													//Period set for largest dynamic range of standard timer frequencies to be used
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / (2 * PWM_UPDATE_INTENSITY_FREQUENCY * TIM_TimeBaseStructure.TIM_Period);		//Set prescaler to determine frequency
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;																																									//Restart RCR count after counting down to this value
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);																																											//Initialize struct parameters to TIM5
	
	TIM_Cmd(TIM5, ENABLE);																																		//Enable specified peripheral
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);																								//Enable new interrupt state
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;																						//Specify interrupt request channel to be used
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 															//Indicates pre-emption priority, 0-15, lower # =higher prriority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; 																		//Subpriority value 0-15, lower # =higher prriority
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 																					//Enable interrupt request channel specified earlier
  NVIC_Init(&NVIC_InitStructure);																														//Initialize NVIC for TIM3 with struct params
}
