/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "track_data_decoder.h"
#include "ws2812b_start_lights.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	RACE_STATE_STARTUP,
	RACE_STATE_RED_FLAG,
	RACE_STATE_YELLOW_FLAG,
	RACE_STATE_GREEN_FLAG,
	RACE_STATE_START_PROC,
	RACE_STATE_JUMP_START,
	RACE_STATE_OPEN
} RaceState;

typedef struct {
    RaceState curr;
    RaceState prev;
} RaceStateHistory;

typedef struct {
	uint8_t red_count; 			// 0 - 5
	uint8_t red_count_prev;
	uint8_t yellow_flag; 		// 0 - 1
	uint8_t yellow_flag_prev;
	uint8_t green_flag; 		// 0 - 1
	uint8_t green_flag_prev;
	uint8_t update_flag;
} StartLightValue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACK_INT_BUF_LEN 16
#define TRACK_INT_DEBOUNCE 10U // debounce of track interrupts in us
#define TRACK_DATA_CPLT_WAIT 220U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim1_up;

/* USER CODE BEGIN PV */
volatile uint32_t millis = 0;

// track data
volatile int32_t track_data_falling[TRACK_INT_BUF_LEN];
volatile uint16_t track_data_completed_ts;
volatile uint8_t track_data_buff_idx = 0;
volatile uint16_t track_data_available = 0;
uint16_t track_data_previous_counter_value = 0;

// state and transition handling
RaceStateHistory race_state = {RACE_STATE_STARTUP, RACE_STATE_STARTUP};
StartLightValue start_light_value = {0};
uint32_t start_light_change_time = 0;

uint8_t safetycar_debounce = 0; // neccessary because of ghost car messages
uint32_t green_flag_triggered_time = 0;
uint32_t safetycar_flash_interval = 0;

uint32_t green_flag_millis_flash = 0;

uint32_t jump_start_millis_flash = 0;


HAL_StatusTypeDef dma_status_pwm = HAL_OK;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void update_race_state(RaceStateHistory *stateHistory, RaceState newState) {
    stateHistory->prev = stateHistory->curr;
    stateHistory->curr = newState;
}

static inline void update_start_light_value_red(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->red_count = newValue;
	startLightValue->red_count_prev = startLightValue->red_count;
	startLightValue->update_flag = 1;
}
static inline void update_start_light_value_green(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->green_flag = newValue;
	startLightValue->green_flag_prev = startLightValue->green_flag;
	startLightValue->update_flag = 1;
}
static inline void update_start_light_value_yellow(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->yellow_flag = newValue;
	startLightValue->yellow_flag_prev = startLightValue->yellow_flag;
	startLightValue->update_flag = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  start_lights_init(&htim1, TIM_CHANNEL_1, TIM_DIER_CC1DE);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  // init manchester detect buffer
  for (int i = 0; i < TRACK_INT_BUF_LEN; i++)
	  track_data_falling[i] = -1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  // process data

	  // process data
	  uint16_t tim3_cnt = TIM3->CNT;
	  if ((track_data_available == 1) && (tim3_cnt - track_data_completed_ts > 220)){

		  // copy buffer
		  int32_t buffer_copy[TRACK_INT_BUF_LEN];
		  for (int i = 0; i < TRACK_INT_BUF_LEN; i++)
			  buffer_copy[i] = track_data_falling[i];

		  // clear buffer
		  for (int i = 0; i < TRACK_INT_BUF_LEN; i++)
			  track_data_falling[i] = -1;

		  // reset buffer variables
		  track_data_buff_idx = 0;
		  track_data_available = 0;

		  // decode cu message
		  CuMessage decoded_data = decodeManchester(buffer_copy, TRACK_INT_BUF_LEN);

		  // set start light instantly
		  if (CU_START_LIGHT == decoded_data.type){
			  update_start_light_value_red(&start_light_value, decoded_data.value);

			  if (decoded_data.value == 5 ){
				  if (race_state.curr != RACE_STATE_START_PROC) // if start proc then it relates to it
					  update_race_state(&race_state, RACE_STATE_RED_FLAG);
			  } else if (decoded_data.value == 1) //
				  update_race_state(&race_state, RACE_STATE_START_PROC);
			  else if (decoded_data.value == 0) {
				  if  ((race_state.curr == RACE_STATE_START_PROC) && race_state.prev == RACE_STATE_RED_FLAG){
					  start_light_change_time = millis;
				  }
				  else if (race_state.curr == RACE_STATE_RED_FLAG){
					  update_race_state(&race_state, RACE_STATE_GREEN_FLAG);
					  green_flag_triggered_time = millis;
				  }
			  }
		  }

		  if (RACE_STATE_START_PROC == race_state.curr){
			  if (!start_light_value.red_count && (millis - start_light_change_time > 400))
				  update_race_state(&race_state, RACE_STATE_OPEN);
		  }

		  if (CU_CONTROLLER_JUMPSTART == decoded_data.type){
			  update_race_state(&race_state, RACE_STATE_JUMP_START);
		  }


		  // check for yellow flag
		  if (CU_CONTROLLER_SAFETY_CAR == decoded_data.type){
			  if (decoded_data.value && (race_state.curr != RACE_STATE_YELLOW_FLAG))
				  safetycar_debounce++;

			  if (!decoded_data.value && (race_state.curr == RACE_STATE_YELLOW_FLAG))
				  safetycar_debounce--;

			  if ((1 < safetycar_debounce) && (race_state.curr != RACE_STATE_YELLOW_FLAG))
				  update_race_state(&race_state, RACE_STATE_YELLOW_FLAG);

			  if ((safetycar_debounce == 0) && (race_state.curr == RACE_STATE_YELLOW_FLAG)){
				  update_race_state(&race_state, RACE_STATE_GREEN_FLAG);
				  green_flag_triggered_time = millis;
			  }
		  }
	  }



	  // start light value changed
	  if (start_light_value.red_count != start_light_value.red_count_prev)
		  update_start_light_value_red(&start_light_value, start_light_value.red_count);

	  // SAFETY CAR
	  if (RACE_STATE_YELLOW_FLAG == race_state.curr){
		  if (millis - safetycar_flash_interval > 200){
			  safetycar_flash_interval = millis;
			  if (0 == start_light_value.yellow_flag)
				  update_start_light_value_yellow(&start_light_value, 1);
			  else
				  update_start_light_value_yellow(&start_light_value, 0);
		  }
	  } else {
		  if (start_light_value.yellow_flag && (RACE_STATE_JUMP_START != race_state.curr)){
			  update_start_light_value_yellow(&start_light_value, 0);
			  update_start_light_value_yellow(&start_light_value, 0);
		  }
	  }

	  // green flag
	  if (RACE_STATE_GREEN_FLAG == race_state.curr){
		  if (millis - green_flag_millis_flash > 500){
			  if (!start_light_value.green_flag){
				  green_flag_millis_flash = millis;
				  update_start_light_value_green(&start_light_value, 1);
			  }else {
				  green_flag_millis_flash = millis;
				  update_start_light_value_green(&start_light_value, 0);
			  }
		  }

		  if (millis - green_flag_triggered_time > 10000)
			  update_race_state(&race_state, RACE_STATE_OPEN);
	  }else{
		  if (1 == start_light_value.green_flag)
			  update_start_light_value_green(&start_light_value, 0);
	  }


	  // JUMP START
	  if (RACE_STATE_JUMP_START == race_state.curr){
		  if (millis - jump_start_millis_flash > 500){
			  jump_start_millis_flash = millis;
			  update_start_light_value_red(&start_light_value, 5);
			  if (!start_light_value.yellow_flag)
				  update_start_light_value_yellow(&start_light_value, 1);
			  else
				  update_start_light_value_yellow(&start_light_value, 0);
		  }
	  }


	  if (start_light_value.update_flag){
	  		  start_light_value.update_flag = 0;
	  		  if (start_light_value.yellow_flag)
	  			  for (uint8_t ylight = 0; ylight < 10; ylight++)
	  				  start_lights_set_colour(ylight, 80, 25 , 0);
	  		  else
	  			  for (uint8_t ylight = 0; ylight < 10; ylight++)
	  				  start_lights_set_colour(ylight, 0, 0 , 0);

	  		  if (start_light_value.green_flag)
	  			  for (uint8_t glight = 10; glight < 20; glight++)
	  				  start_lights_set_colour(glight, 0, 100 , 0);
	  		  else
	  			  for (uint8_t glight = 10; glight < 20; glight++)
	  				  start_lights_set_colour(glight, 0, 0 , 0);

	  		  if (start_light_value.red_count){
	  			  for (uint8_t light = 20; light < 40; light++)
	  				  start_lights_set_colour(light, 0, 0, 0);

	  			  for (uint8_t light = 0; light < start_light_value.red_count; light++){
	  				  start_lights_set_colour(20+light, 200, 0, 0);
	  				  start_lights_set_colour(25+light, 200, 0, 0);
	  				  start_lights_set_colour(30+light, 200, 0, 0);
	  				  start_lights_set_colour(35+light, 200, 0, 0);
	  			  }
	  		  } else {
	  			  for (uint8_t light = 20; light < 40; light++){
	  				  start_lights_set_colour(light, 0, 0, 0);
	  			  }
	  		  }

	  		dma_status_pwm = start_lights_update();
	  	  }

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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 48000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRACK_LIGHTS_R_Pin|TRACK_LIGHTS_G_Pin|IO_DATA_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRACK_LIGHTS_R_Pin TRACK_LIGHTS_G_Pin IO_DATA_INT_Pin */
  GPIO_InitStruct.Pin = TRACK_LIGHTS_R_Pin|TRACK_LIGHTS_G_Pin|IO_DATA_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		uint16_t counterValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		uint16_t debounceCheck = counterValue - track_data_previous_counter_value;
		if (debounceCheck > TRACK_INT_DEBOUNCE){
			track_data_previous_counter_value = counterValue;
			track_data_available = 1;
			track_data_completed_ts = counterValue;
			track_data_falling[track_data_buff_idx] = counterValue;
			track_data_buff_idx++;
		}
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if (htim == START_LIGHTS_TIM)
		start_lights_dma_callback();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM17){
		millis+=100;
	}
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
