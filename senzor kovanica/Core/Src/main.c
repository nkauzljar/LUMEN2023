/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <i2c-lcd.h>
#include "flashFunction.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EFreqWeight 1.f
#define MFreqWeight 1.f
#define MAmplWeight 1.f
#define criticalCertainty 0.9f
#define MA_rdy ADC1->SR || 0x10
#define moving_average_size 8
#define SLAVE_ADDRESS_LCD 0x4E
#define servo1_neutral()
#define servo1_return()
#define servo1_passtrough()
#define FLASH_MEMORY_BEGIN 0x08060000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
volatile uint32_t MA_min_val = 65535;
volatile uint32_t MA_new_val;
volatile uint32_t MF_max_val = 0;
volatile uint32_t MF_new_val;
volatile uint8_t MF_rdy;

volatile uint32_t EF_min_val = 65535;
volatile uint32_t EF_new_val;
volatile uint8_t EF_rdy;

volatile uint32_t MA_mov_avg[moving_average_size];
volatile uint32_t MF_mov_avg[moving_average_size];
volatile uint32_t EF_mov_avg[moving_average_size];

uint8_t MA_mov_avg_index;
uint8_t MF_mov_avg_index;
uint8_t EF_mov_avg_index;

float EF_avg;
float MF_avg;
float MA_avg;

volatile uint8_t MF_sensor_output_stable = 0;
volatile uint8_t EF_sensor_output_stable = 0;
volatile uint8_t MA_sensor_output_stable = 0;

uint8_t i, j, k;
float sum_DV, sum_DF, sum_DA, sum_total, sum = 0;

struct coinSave{
	uint8_t coinID; //coin ID
	char currencyName[3]; //currency name
	float value; //coin value
	float df; //frequency change of M sensor
	float da; //amplitude change of M sensor
	float dv; //frequency change of E sensor
};
struct coinSave savedCoins[8] = {0};
volatile float coin_values[8] = {0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2};

volatile float EFWeight;
volatile float MFWeight;
volatile float MAWeight;

volatile uint8_t coin_inserted = 0;
volatile uint8_t coin_still_present = 0;
volatile uint8_t counting_mode;
volatile uint8_t cal_mode;
volatile uint8_t debug_mode = 1;

volatile uint16_t servo1_timer = 18240;
volatile uint8_t servo1_state = 0;
volatile uint16_t servo2_timer = 18240;
volatile uint8_t servo2_state = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM9_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
static void reset_data();
static float probability_match(struct coinSave *coin1, struct coinSave *coin2);
static float moving_avg(uint8_t *index, uint32_t *array_pointer, uint32_t new_value);
void servo1_angle(int ang);
void servo2_angle(int ang);
static float deviation(float a, float b);

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
	struct coinSave current_coin;
	float DF, DA, DV;
	uint8_t flashCheck;




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
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  lcd_init();

  reset_data();
  MF_mov_avg_index = 0;
  MA_mov_avg_index = 0;
  EF_mov_avg_index = 0;
  EFWeight = EFreqWeight / (EFreqWeight + MFreqWeight + MAmplWeight);
  MFWeight = MFreqWeight / (EFreqWeight + MFreqWeight + MAmplWeight);
  MAWeight = MAmplWeight / (EFreqWeight + MFreqWeight + MAmplWeight);

  flashCheck = read_data(FLASH_MEMORY_BEGIN);
  if(flashCheck == 8){
	//Coins are already stored, store data from flash into RAM
	for(i=0;i<8;i++){
		Read_coin(&savedCoins[i], i);
	}
  }else{
  	void coin_init();
  	//init RAM stored coins
  	for(i = 0;i<8;i++){
  		savedCoins[i].currencyName[0] = 'E';
  		savedCoins[i].currencyName[1] = 'U';
  		savedCoins[i].currencyName[2] = 'R';
  		savedCoins[i].coinID = i;
  		savedCoins[i].value = coin_values[i];
  	}
  }
  current_coin.coinID = 0;

  if(debug_mode){
	  counting_mode = 1;
	  cal_mode = 0;
  }else{
	  cal_mode = 1;
	  counting_mode = 0;
  }


  //HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
  ADC1->CR2 |= 0x01;
  display_total_init();
  display_total_update(0);

  servo1_neutral();
  servo2_angle(90);

  while(!MF_sensor_output_stable && !MA_sensor_output_stable && !EF_sensor_output_stable ){
	  if(MF_rdy == 1){
		  MF_rdy = 0;
		  if(deviation(MF_avg, MF_new_val) < 0.01) {
			  MF_sensor_output_stable = 1;
		  }
		  MF_avg = moving_avg(&MF_mov_avg_index, MF_mov_avg, MF_new_val);
	  }
	  if(EF_rdy == 1){
		  EF_rdy = 0;
		  if(deviation(EF_avg, EF_new_val) < 0.01) {
			  EF_sensor_output_stable = 1;
		  }
		  EF_avg = moving_avg(&EF_mov_avg_index, EF_mov_avg, EF_new_val);
	  }
	  if(MA_rdy){
		  MA_new_val = ADC1->DR;
		  if(deviation(MA_avg, MA_new_val) < 0.01) {
			  MA_sensor_output_stable = 1;
		  }
		  MA_avg = moving_avg(&MA_mov_avg_index, MA_mov_avg, MA_new_val);
	  }
  }
  reset_data();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //self calibrate until coin is inserted
	  while(!coin_inserted){
		  if(MF_rdy == 1)
	  	  {
	  		  MF_rdy = 0;
	  		  MF_avg = moving_avg(&MF_mov_avg_index, MF_mov_avg, MF_new_val);
	  	  }
	  	  if(EF_rdy == 1)
	  	  {
	  		  EF_rdy = 0;
	  		  EF_avg = moving_avg(&EF_mov_avg_index, EF_mov_avg, EF_new_val);
	  	  }
	  	  if(MA_rdy){
	  		  MA_new_val = ADC1->DR;
	  		  MA_avg = moving_avg(&MA_mov_avg_index, MA_mov_avg, MA_new_val);
	  	  }
	  }
	  //coin is inserted, capture peaks/valleys of measurments
	  while(coin_still_present){
		  coin_inserted = 0;
		  if(MF_rdy == 1)
	  	  {
	  		  MF_rdy = 0;
	  		  if(MF_new_val < MF_max_val)
	  		  {
	  			  MF_max_val = MF_new_val;
	  		  }
	  	  }
	  	  if(EF_rdy == 1)
	  	  {
	  		  EF_rdy = 0;
	  		  if(EF_new_val > EF_min_val)
	  		  {
	  			  EF_min_val = MF_new_val;
	  		  }
	  	  }
	  	  //ADC conversion finished
	  	  if(MA_rdy){
	  		  MA_new_val = ADC1->DR;
	  		  if(MA_new_val < MA_min_val){
	  			  MA_min_val = MA_new_val;
	  		  }
	  	  }
	  }
	  //coin left the tunnel
	  HAL_TIM_Base_Stop_IT(&htim10);
	  HAL_TIM_Base_Stop_IT(&htim11);
	  DA = MA_avg/MA_min_val;
  	  DF = MF_max_val - MF_avg;
  	  DV = EF_avg/EF_min_val;
  	  if(debug_mode){
  		  //ispisati na display vrijednosti DA, DF i DV
  	  }else if(counting_mode){
	  	  current_coin.da = DA;
	  	  current_coin.df = DF;
	  	  current_coin.dv = DV;
	  	  j = 0;
	  	  for(i = 0;i<8;i++){
	  		if(probability_match(&savedCoins[i], &current_coin) >= criticalCertainty){
	  			j++;
	  			current_coin.value = savedCoins[i].value;
	  		}
	  		if(j > 1)break;
	  	  }
	  	  if(j == 1){
	  		 //coin is recognized
	  		sum += current_coin.value;
	  		display_total_update(sum);
	  		servo2_angle(current_coin.coinID * 22);
	  		HAL_Delay(1000);
	  		servo1_passtrough();
	  		HAL_Delay(1000);
	  		servo1_neutral();
	  		servo2_angle(90);
	  	  }else{
	  		//coin is not recognized
	  		servo1_return();
	  		HAL_Delay(1000);
	  		servo1_neutral();
	  	  }
	  	  reset_data();

	  }else if(cal_mode){
		  if(j == 0){
			  sum = 0;
		  }else if(j < 5){
			  sum_DV += DV;
			  sum_DF += DF;
			  sum_DA += DA;
		  }else{
			  sum_DV /= 5;
			  sum_DF /= 5;
			  sum_DA/= 5;
			  savedCoins[i].da = sum_DA;
			  savedCoins[i].df = sum_DF;
			  savedCoins[i].dv = sum_DV;
			  savedCoins[i].value = coin_values[i];
			  Write_coin((uint8_t *)(&savedCoins[i]), savedCoins[i].coinID);
			  j = 0;
			  i++;
		  }
		  if(i == 9){
			  counting_mode = 1;
			  cal_mode = 0;
		  }
	  }
  	HAL_TIM_Base_Start_IT(&htim10);
  	HAL_TIM_Base_Start_IT(&htim11);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 240000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 240000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 192;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback
	if (htim == &htim9){
		//neuspjelo :(
	  }
	  if(htim == &htim10){
		  //ovo se ne bi trebalo desiti :/
	  }
	  if(htim == &htim11){
		  //ovo se ne bi trebalo desiti :/
	  }

	  if (htim == &htim2) {

	    if(servo1_state == 1){
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	      servo1_state = 0;
	      TIM2->ARR = servo1_timer;
	    }
	    else{
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	      servo1_state = 1;
	      TIM2->ARR = 240000;
	    }

	  }
	  if(htim == &htim5){
	    if(servo2_state == 1){
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	      servo2_state = 0;
	      TIM5->ARR = servo2_timer;
	    }
	    else{
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	      servo2_state = 1;
	      TIM5->ARR = 240000;
	    }
	  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  //MF update and start ADC conversion  pin PE6 i PA1
  if(GPIO_Pin == GPIO_PIN_6) {
	  MF_rdy = 1;
	  MF_new_val = TIM10->CNT;
  	  TIM10->CNT &= 0x00; //reset timer counter
  	  ADC1->CR2 |= 1 << 30; //start ADC conversion
  }
  //EF update PIN PB3
  if(GPIO_Pin == GPIO_PIN_3) {
	  EF_new_val = TIM11->CNT;
	  TIM11->CNT &= 0x00;
	  EF_rdy = 1;
  }
  //USER gumb na disco boardu
  if(GPIO_Pin == GPIO_PIN_0){
	  if(!debug_mode){
		  cal_mode = 1;
		  counting_mode = 0;
	  }
  }
  //LED detection on coin insertin on PC7
  if(GPIO_Pin == GPIO_PIN_7){
  	  coin_inserted = 1;
  	  coin_still_present = 1;
  }
  //LED detection on coin leaving sensors on PA9
  if(GPIO_Pin == GPIO_PIN_9){
	  coin_still_present = 0;
  }

}
//nazalost ne radi dobro :(
/*
static void set_local_osc(float freq){
	TIM9->ARR = (uint16_t)(48e3/freq+1);
}
*/
static void reset_data(){
	MA_min_val = 4095;
	MF_max_val = 0;
	EF_min_val = 65535;
	MF_rdy = 0;
	EF_rdy = 0;
	ADC1->SR &= ~0x10;
}
static float probability_match(struct coinSave *coin1, struct coinSave *coin2){
	float da_probability, df_probability, dv_probability;
	da_probability = 1 - deviation(coin1->da, coin2->da);
	df_probability = 1 - deviation(coin1->df, coin2->df);
	dv_probability = 1 - deviation(coin1->dv, coin2->dv);
	if(da_probability <= 0 || df_probability <= 0  || dv_probability <= 0) return 0;
	return da_probability * MAWeight + df_probability * MFWeight + dv_probability * EFWeight;
}

static float moving_avg(uint8_t *index, uint32_t *array_pointer, uint32_t new_value){
	float sum = 0;
	uint8_t sum_index;
	array_pointer[*(index)] = new_value;
	*(index) = *(index) + 1;
	*(index) %= moving_average_size;
	for(sum_index=0;sum_index<moving_average_size;sum_index++){
		sum += array_pointer[sum_index];
	}
	sum /= (float)moving_average_size;
	return sum;
}

static float deviation(float a, float b){
	float result = (a - b)/a;
	if(result < 0) result *= -1;
	return result;
}

void servo1_angle(int ang){
	servo1_timer = (ang*120)+7440;
}

void servo2_angle(int ang){
	servo2_timer = (ang*120)+7440;
}
/*
static float abs(float a){
	if(a < 0) return -1*a;
	return a;
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
