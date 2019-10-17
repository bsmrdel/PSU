/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLT_DIV_FACTOR		0.0445 		// assuming R1 = 16.4k and R2 = 1k
#define CURR_DIV_FACTOR 	0.235	// CSA gain is 0.5V/A
#define VOLT_OFFSET			0.3
#define CURR_OFFSET			0.15
#define CURR_REF			2.958		//reference voltage for CSA
#define cc_hysterisis       0.01
#define N					100			// moving avg approx uses 100 past samples

#define UNK                 -1
#define NON_INTR             0
#define INTR                 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

//Elliott UI

int interrupt_mode = UNK;   // which version of putchar/getchar to use.
int error_voltage = 0;   //rename and write conditions for errors
int error_current = 0;
int error_temp = 0;

int User_Voltage_limit = 0;  //set from encoder
int User_Current_limit = 0;  //set from encoder
int OutputCounter = 0;
int VDecimal = 0;
int VDecimalCounter = 0;
int CDecimal = 0;
int CDecimalCounter = 0;
int VDecimalOn = 0;
int VDecimalOff = 0;
int VaState = 0;
int VbState = 0;
int CDecimalOn = 0;
int CDecimalOff = 0;
int CaState = 0;
int CbState = 0;
int VaLastState = 0;
int VbLastState = 0;
int CaLastState = 0;
int CbLastState = 0;
int Watts = 0;  	//Power being displayed to user, calculated from output V and I
int LastWatts = 0;
int Last_v_sense_avg = 0;
int Last_i_sense_avg = 0;
int Lastfarh = 0;

int raw_tempsense_value = 0; //12b value from adc for TempSense
float farh = 0;
int max_trans_current = 0;

//brad's buck converter variables
int raw_voltage = 0;            //12b value from adc for vsense
int raw_current = 0;            //12b value from adc for isense
float pwm_val = 0;			//PWM value for duty cycle adjustment to gate driver
float v_sense = 0;			//buck output voltage sense
float i_sense = 0;			//buck output current sense
float rload = 0;            //buck calculated load from v_sense & i_sense
float pid_error = 0;		//difference from target v or i and sensed v or i
int user_en = 1;            //output enable flag
//float i_lim = 0.010;        //user selected current limit @elliott
//float v_lim = 3;            //user selected voltage limit @elliott
float i_lim = 0;        //user selected current limit @elliott  made changes and uses actuall set limit
float v_lim = 0;           //user selected voltage limit @elliott made changes

float v_sense_avg = 0;		//moving average val of v_sense
float i_sense_avg = 0;		//moving average val of i_sense
float farh_average = 0;
//int cv_cc = 1;				//constant voltage = 1, constant current = 0 (modes of operation)
int cvcc_flag = 0;

float PID_Kp = 400;             //proportional gain
float PID_Ki = 0.001;           //integral gain
float PID_Kd = 0;              //derivative gain


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//User Control

void OutputEnable(int error_voltage, int error_current, int error_temp);
void getVoltage_limit(void);
void getCurrent_limit(void);
void serial_init(void);


static float approxMovingAvg(float avg, float new_sample);
void senseADC(void);
void getV(void);
void getI(void);
void Print_Power(void);
void getTemp(void);
void FanPWM(void);
void getMode(void);
void max_trans(void);
void PIDsetBuckPWM();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	serial_init();	//set up serial for nextion HMI
	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Start PWM for Buck
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Start PWM for FAN

	printf("VoltageL.val=%d%c%c%c",0,255,255,255);		//prints voltage to screen	VoltageL.val=0ÿÿÿ
	printf("CurrentL.val=%d%c%c%c",0,255,255,255);		//prints voltage to screen	VoltageL.val=0ÿÿÿ
	printf("Temp.pco=%d%c%c%c",65535,255,255,255);		//turns temp number black on screen Temp.pco=0ÿÿÿ
	printf("temptxt.pco=%d%c%c%c",65535,255,255,255);	//turns temp text black on screen tempxt.pco=0ÿÿÿ
	printf("ftxt.pco=%d%c%c%c",65535,255,255,255);		//turns F text black on screen ftxt.pco=0ÿÿÿ

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		//User Interface
		//OutputEnable(error_voltage,error_current,error_temp);	//Output enable function, error parameters can be used to send error signal to disable output
		getVoltage_limit();
		getCurrent_limit();

		v_lim = User_Voltage_limit/100.0;
		i_lim = User_Current_limit/100.0;

		//user end
		senseADC();
		getV();
		getI();
		Print_Power();
		rload = v_sense_avg / i_sense_avg;

		getMode();			//1 = Const V, 0 = Const C mode
		PIDsetBuckPWM();	//set new PWM for buck using PID loop

		getTemp();
		max_trans();  		//Checks and displays max transient current when OE is pressed.
		FanPWM();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */
	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC_Init 1 */
	/* USER CODE END ADC_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = ENABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */
	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */
	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */
	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 48-1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 6;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */
	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */
	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */
	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 2000-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 400-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 100;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */
	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, Output_enable_LED_Pin|CC_LED_Pin|CV_LED_Pin|LD4_Pin
			|LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Current_Decimal_ON_Pin Current_Decimal_OFF_Pin Voltage_Encoder_A_Pin Voltage_Encoder_B_Pin */
	GPIO_InitStruct.Pin = Current_Decimal_ON_Pin|Current_Decimal_OFF_Pin|Voltage_Encoder_A_Pin|Voltage_Encoder_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Output_enable_LED_Pin CC_LED_Pin CV_LED_Pin LD4_Pin
                           LD3_Pin */
	GPIO_InitStruct.Pin = Output_enable_LED_Pin|CC_LED_Pin|CV_LED_Pin|LD4_Pin
			|LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Current_Encoder_A_Pin Current_Encoder_B_Pin Output_Enable_ON_Pin Output_Enable_OFF_Pin */
	GPIO_InitStruct.Pin = Current_Encoder_A_Pin|Current_Encoder_B_Pin|Output_Enable_ON_Pin|Output_Enable_OFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Max_transient_Reset_ON_Pin Max_transient_Reset_OFF_Pin Voltage_Decimal_ON_Pin Voltage_Decimal_OFF_Pin */
	GPIO_InitStruct.Pin = Max_transient_Reset_ON_Pin|Max_transient_Reset_OFF_Pin|Voltage_Decimal_ON_Pin|Voltage_Decimal_OFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void OutputEnable(int error_voltage, int error_current, int error_temp){
	int OutputOn = 0;
	int OutputOff = 0;

	OutputOn=HAL_GPIO_ReadPin(Output_Enable_ON_GPIO_Port,Output_Enable_ON_Pin);
	OutputOff=HAL_GPIO_ReadPin(Output_Enable_OFF_GPIO_Port,Output_Enable_OFF_Pin);

	if(error_voltage == 1 || error_current == 1 || error_temp == 1){		//if an error has occurred
		OutputOn = 0;	//make output turn off
		OutputOff = 1;	//make output turn off
	}

	if (OutputOn == 0 && OutputOff != 0 && OutputCounter == 0){		//if button has been pressed once
		OutputCounter ++;
		HAL_GPIO_WritePin(Output_enable_LED_GPIO_Port,Output_enable_LED_Pin , GPIO_PIN_RESET); //turn off output enable led
	}
	else if(OutputOn != 0 && OutputOff == 0 && OutputCounter == 1){	//if button has been pressed twice
		OutputCounter = 0;
		HAL_GPIO_WritePin(Output_enable_LED_GPIO_Port,Output_enable_LED_Pin , GPIO_PIN_SET);  //turn on output enable led
	}
}

void getVoltage_limit(void){

	VDecimalOn = HAL_GPIO_ReadPin(Voltage_Decimal_ON_GPIO_Port,Voltage_Decimal_ON_Pin);		//read input from PA0 Reads the "current" state of the button
	VDecimalOff = HAL_GPIO_ReadPin(Voltage_Decimal_OFF_GPIO_Port,Voltage_Decimal_OFF_Pin);	//read input from PA1 Reads the "current" state of the button

	if (VDecimalOn == 0 && VDecimalOff != 0 && VDecimalCounter == 0){	//if button has been pressed once
		VDecimal = 1;								//start incrementing voltage by 0
		VDecimalCounter ++;							//increment counter
		printf("vd1.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd2.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd3.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd4.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
	}
	else if(VDecimalOn != 0 && VDecimalOff == 0 && VDecimalCounter == 1){	//if button has been pressed twice
		VDecimal = 10;								//start incrementing voltage by 10
		VDecimalCounter ++;							//increment counter
		printf("vd1.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd2.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd3.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
		printf("vd4.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
	}
	else if(VDecimalOn == 0 && VDecimalOff != 0 && VDecimalCounter == 2){	//if button has been pressed three times
		VDecimal = 100;								//start incrementing voltage by 100
		VDecimalCounter ++;							//increment counter
		printf("vd1.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd2.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
		printf("vd3.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd4.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
	}
	else if(VDecimalOn != 0 && VDecimalOff == 0 && VDecimalCounter == 3){	//if button has been pressed four times
		VDecimal = 1000;							//start incrementing voltage by 1000
		VDecimalCounter = 0;						//reset counter
		printf("vd1.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
		printf("vd2.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd3.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("vd4.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
	}

	//VOLTAGE COUNTER
	VaState = HAL_GPIO_ReadPin(Voltage_Encoder_A_GPIO_Port,Voltage_Encoder_A_Pin);	//read input from PA4 // Reads the "current" state of the outputA
	VbState = HAL_GPIO_ReadPin(Voltage_Encoder_B_GPIO_Port,Voltage_Encoder_B_Pin);	//read input from PA5// Reads the "current" state of the outputB

	if (VaState != VaLastState && VaState > VbState){		//if the previous and the current state of the outputA are different, that means a Pulse has occurred and if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Voltage_limit = User_Voltage_limit + VDecimal;	//increment voltage
		if (User_Voltage_limit > 3200) {       				//our power supply cannot go over 32 V
			User_Voltage_limit = 3200;         				//don't allow voltage setting above 32 V
		}
		printf("VoltageL.val=%d%c%c%c",User_Voltage_limit,255,255,255);	//prints voltage to screen VoltageL.val=888ÿÿÿ
	}
	else if (VbState != VbLastState && VbState > VaState){	//if the previous and the current state of the outputA are different, that means a Pulse has occurred and //if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Voltage_limit = User_Voltage_limit - VDecimal;	//increment voltage
		if (User_Voltage_limit < 0) {       				//our power supply cannot go over 32 V
			User_Voltage_limit = 0;         				//don't allow voltage setting above 32 V
		}
		printf("VoltageL.val=%d%c%c%c",User_Voltage_limit,255,255,255);	//prints voltage to screen VoltageL.val=888ÿÿÿ
	}
	VaLastState = VaState;          						//updates the previous state of the outputA with the current state
	VbLastState = VbState;									//updates the previous state of the outputB with the current state
}

void getCurrent_limit(void){

	CDecimalOn = HAL_GPIO_ReadPin(Current_Decimal_ON_GPIO_Port,Current_Decimal_ON_Pin);		//read input from PA2 Reads the "current" state of the button
	CDecimalOff = HAL_GPIO_ReadPin(Current_Decimal_OFF_GPIO_Port,Current_Decimal_OFF_Pin);	//read input from PA3 Reads the "current" state of the button

	if (CDecimalOn == 0 && CDecimalOff != 0 && CDecimalCounter == 0){	//if button has been pressed once
		CDecimal = 1;								//start incrementing voltage by 0
		CDecimalCounter ++;							//increment counter
		printf("cd1.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd2.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd3.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd4.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
	}
	else if(CDecimalOn != 0 && CDecimalOff == 0 && CDecimalCounter == 1){	//if button has been pressed twice
		CDecimal = 10;								//start incrementing voltage by 10
		CDecimalCounter ++;							//increment counter
		printf("cd1.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd2.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd3.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
		printf("cd4.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
	}
	else if(CDecimalOn == 0 && CDecimalOff != 0 && CDecimalCounter == 2){	//if button has been pressed three times
		CDecimal = 100;								//start incrementing voltage by 100
		CDecimalCounter ++;							//increment counter
		printf("cd1.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd2.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
		printf("cd3.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd4.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
	}
	else if(CDecimalOn != 0 && CDecimalOff == 0 && CDecimalCounter == 3){	//if button has been pressed four times
		CDecimal = 1000;							//start incrementing voltage by 1000
		CDecimalCounter = 0;						//reset counter
		printf("cd1.val=%d%c%c%c",1,255,255,255);	//set decimal place indicator
		printf("cd2.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd3.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
		printf("cd4.val=%d%c%c%c",0,255,255,255);	//set decimal place indicator
	}

	//CURRENT COUNTER
	CaState = HAL_GPIO_ReadPin(Current_Encoder_A_GPIO_Port,Current_Encoder_A_Pin);	//read input from PA4 // Reads the "current" state of the outputA
	CbState = HAL_GPIO_ReadPin(Current_Encoder_B_GPIO_Port,Current_Encoder_B_Pin);	//read input from PA5// Reads the "current" state of the outputB

	if (CaState != CaLastState && CaState > CbState){		//if the previous and the current state of the outputA are different, that means a Pulse has occurred and if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Current_limit = User_Current_limit + CDecimal;	//increment voltage
		if (User_Current_limit > 3200) {       				//our power supply cannot go over 32 V
			User_Current_limit = 3200;         				//don't allow voltage setting above 32 V
		}
		printf("CurrentL.val=%d%c%c%c",User_Current_limit,255,255,255);	//prints voltage to screen VoltageL.val=888ÿÿÿ
	}
	else if (CbState != CbLastState && CbState > CaState){	//if the previous and the current state of the outputA are different, that means a Pulse has occurred and //if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Current_limit = User_Current_limit - CDecimal;	//increment voltage
		if (User_Current_limit < 0) {       				//our power supply cannot go over 32 V
			User_Current_limit = 0;         				//don't allow voltage setting above 32 V
		}
		printf("CurrentL.val=%d%c%c%c",User_Current_limit,255,255,255);	//prints voltage to screen VoltageL.val=888ÿÿÿ
	}
	CaLastState = CaState;          						//updates the previous state of the outputA with the current state
	CbLastState = CbState;									//updates the previous state of the outputB with the current state
}

void senseADC (void){

	HAL_ADC_Start(&hadc);                             //start ADC
	HAL_ADC_PollForConversion(&hadc, 10);             //poll until complete
	raw_voltage = HAL_ADC_GetValue(&hadc);            //collect raw voltage

	HAL_ADC_Start(&hadc);                             //start ADC
	HAL_ADC_PollForConversion(&hadc, 10);             //poll until complete
	raw_current = HAL_ADC_GetValue(&hadc) ;            //collect raw current

	HAL_ADC_Start(&hadc);                             //start ADC
	HAL_ADC_PollForConversion(&hadc, 10);             //poll until complete
	raw_tempsense_value = HAL_ADC_GetValue(&hadc);    //collect raw tempval

	HAL_ADC_Stop(&hadc);                              //stop ADC

	return;
}

void getV (void){

	float voltage = 0;             						//0-3V conversion for voltage
	float percent_voltage = 0;      					//%V from 0-3

	percent_voltage = ((float) raw_voltage) / 4092;
	voltage = percent_voltage * 3;						//0-3V ADC signal
	v_sense = voltage / VOLT_DIV_FACTOR;				//0-50V value
	v_sense_avg = approxMovingAvg(v_sense_avg, v_sense);//take average
	int v_sense_avg_int = (int)(v_sense_avg*100);		//make variable compatible with screen

	if (v_sense_avg_int != Last_v_sense_avg){			//if the previous and the current state of the outputA are different, that means a Pulse has occurred
		Last_v_sense_avg = v_sense_avg_int;				//updates the previous state of the average v sense with the current state
		printf("VoltageO.val=%d%c%c%c",v_sense_avg_int,255,255,255);	//prints voltage to screen	VoltageO.val=888ÿÿÿ will have to calibrate and multiply by 100 and change to int
		if (v_sense_avg_int > 3200){						//if voltage is above safe value, return error
			error_voltage = 1;							//set error as 1 to turn off outputs
		}
		else {
			error_voltage = 0;							//set error to 0 if nothing is wrong
		}
	}
	return;
}

void getI (void){

	float current = 0;              					//0-3V conversion for voltage
	float percent_current = 0;      					//%V from 0-3

	percent_current = ((float) raw_current) / 4092;
	current = CURR_REF - (percent_current * 3);			//0-3V ADC signal
	i_sense = (current / CURR_DIV_FACTOR) + CURR_OFFSET;//0-3A value
	i_sense_avg = approxMovingAvg(i_sense_avg, i_sense);//take average
	int i_sense_avg_int = (int)(i_sense_avg*100);		//make variable compatible with screen

	if (i_sense_avg_int != Last_i_sense_avg){			//if the previous and the current state of the outputA are different, that means a Pulse has occurred
		Last_i_sense_avg = i_sense_avg_int;				//updates the previous state of the average current sense with the current state
		printf("CurrentO.val=%d%c%c%c",i_sense_avg_int,255,255,255);	//prints current to screen CurrentO.val=888ÿÿÿ	will have to calibrate and multiply by 100
		if (i_sense_avg_int > 300){						//if voltage is above safe value, return error
			error_current = 1;							//set error as 1 to turn off outputs
		}
		else {
			error_current = 0;							//set error to 0 if nothing is wrong
		}
	}
	return;
}

void Print_Power (void){

	if (VaState != VaLastState || CaState != CaLastState){					//if the previous and the current state of the outputA are different, that means a Pulse has occurred
		Watts = ((int)(v_sense_avg*100) * (int)(i_sense_avg*100)) / 100;	//calculate power
		LastWatts = Watts;													//updates the previous state of the watts with the current state
		printf("Watts.val=%d%c%c%c",Watts,255,255,255);						//prints voltage to screen	Watts.val=888ÿÿÿ
	}
	return;
}

void getTemp (void){

	farh = ((raw_tempsense_value/4096.0)*3000.0)/10.0-30; 					//calculate the farenheit using 5V
	farh_average = approxMovingAvg(farh_average,farh);						//finds average of temperature

	if (farh_average != Lastfarh){											//if the previous and the current state of the temperature are different, that means a Pulse has occurred
		Lastfarh = farh_average;											//updates the previous state of the temperature with the current state
		printf("Temp.val=%d%c%c%c",(int)(farh_average*100)/10,255,255,255);	//prints temperature to screen	temp.val=888ÿÿÿ
		if(farh_average >= 100 && error_temp == 0){												//if temperature rises above 100 F, alert user.
			error_temp = 1;													//an error has occurred
			printf("Temp.pco=%d%c%c%c",63488,255,255,255);					//turns temp number red on screen button.val=888ÿÿÿ
			printf("temptxt.pco=%d%c%c%c",63488,255,255,255);				//turns temp text red on screen button.val=888ÿÿÿ
			printf("ftxt.pco=%d%c%c%c",63488,255,255,255);					//turns F text red on screen button.val=888ÿÿÿ
		}
		else if(farh_average < 100 && error_temp == 1){					//if nothing is wrong and something was wrong before
			error_temp = 0;													//an error has not occurred
			printf("Temp.pco=%d%c%c%c",65535,255,255,255);					//turns temp number black on screen Temp.pco=0ÿÿÿ
			printf("temptxt.pco=%d%c%c%c",65535,255,255,255);				//turns temp text black on screen tempxt.pco=0ÿÿÿ
			printf("ftxt.pco=%d%c%c%c",65535,255,255,255);					//turns F text black on screen ftxt.pco=0ÿÿÿ
		}
	}
	return;
}

void FanPWM (void){

	float fan_duty=0;

	if(farh>80){
		fan_duty =95*4;
	}
	else{
		fan_duty = 30*4;
	}

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,fan_duty);

}

static float approxMovingAvg(float avg, float newsample)
{
	avg -= avg / N;
	avg += newsample / N;

	return avg;
}

void getMode(void){

	if(cvcc_flag == 0){   //check if you were in cc mode
		if(i_sense_avg >=(i_lim - cc_hysterisis)){ //checking lower range
			//cc mode
			cvcc_flag = 0;
		}
		else
			cvcc_flag = 1;		//otherwise assume cv mode
	}
	else{
		if(i_sense_avg >=i_lim){ //checking lower range
			//cc mode
			cvcc_flag = 0;
		}
		else
			cvcc_flag = 1;		//otherwise assume cv mode

	}

	// turn on top STM32F0disc LED for CV, bottom for CC mode for debug
	if(cvcc_flag == 1){
		//dev board LEDS
		//		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		//		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		//breadboard LEDS
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CV_LED_GPIO_Port, CV_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		//dev board LEDs
		//		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		//		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		// breadboard LEDS
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CV_LED_GPIO_Port, CV_LED_Pin, GPIO_PIN_RESET);
	}
	return;
}

void max_trans(void){

	int ResetOn  = HAL_GPIO_ReadPin(Max_transient_Reset_ON_GPIO_Port,Max_transient_Reset_ON_Pin);	//read input from PB0 Reads the "current" state of the button
	int ResetOff = HAL_GPIO_ReadPin(Max_transient_Reset_OFF_GPIO_Port,Max_transient_Reset_OFF_Pin);	//read input from PB1 Reads the "current" state of the button

	if (ResetOn > ResetOff){										//if button has been pressed once
		if ((int)(i_sense_avg*100.0)>= max_trans_current){					//if the previous and the current state of the outputA are different, that means a Pulse has occurred
			max_trans_current = (int)(i_sense_avg*100.0);					//updates the previous state of the average current sense with the current state
			printf("Trans.val=%d%c%c%c",max_trans_current,255,255,255);		//prints Maximum Current Transient to screen will have to calibrate and multiply by 100
			printf("TransON.val=%d%c%c%c",1,255,255,255);					//prints Maximum Current Transient is reading to screen.
		}
	}
	else if(ResetOn < ResetOff){									//if button has been pressed twice
		printf("Trans.val=%d%c%c%c",max_trans_current,255,255,255);			//prints Maximum Current Transient to screen will have to calibrate and multiply by 100
		printf("TransON.val=%d%c%c%c",0,255,255,255);						//prints Maximum Current Transient is not reading to screen.
	}
}

void PIDsetBuckPWM(void){
	arm_pid_instance_f32 PID;	//ARM PID instance float 32b

	PID.Kp = PID_Kp;
	PID.Ki = PID_Ki;
	PID.Kd = PID_Kd;

	arm_pid_init_f32(&PID, 1);

	if(cvcc_flag == 1){	//if in CV mode
		pid_error = v_lim - v_sense_avg;
	}else{			//in CC mode
		pid_error = i_lim - i_sense_avg;
	}
	pwm_val = arm_pid_f32(&PID, pid_error);

	if(pwm_val > 46)
		pwm_val = 46;

	if(pwm_val < 0)
		pwm_val = 0;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
