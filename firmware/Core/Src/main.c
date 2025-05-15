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
	START_LIGHT_MODE,
	PIT_LANE_LIGHT_MODE
} LightMode;

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

typedef struct {
	uint16_t message_started;
	uint16_t start_time;
	uint16_t gap;
	uint16_t last_rising;
	uint16_t last_falling;
	uint16_t first_rising_occured;
	uint16_t first_rising;
	uint16_t first_falling;
	uint16_t edges_total;
	uint16_t data;
} ManchesterDecoder;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CIRC_BUF_LEN       	16U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim1_up;

/* USER CODE BEGIN PV */
volatile uint32_t millis = 0;

// manchester decoder
volatile ManchesterDecoder data_manchester[CIRC_BUF_LEN];
volatile uint16_t data_manchester_circular_index = 0;
volatile uint16_t data_manchester_circular_read_index = 0;
uint16_t difference_rw = 0;
volatile uint16_t track_data_available = 0;
uint32_t odd_value = 0;
uint16_t message_to_proceed = 0;

// race state
RaceStateHistory race_state = {RACE_STATE_STARTUP, RACE_STATE_STARTUP};
StartLightValue start_light_value = {0};
uint8_t safetycar_debounce = 0; // neccessary because of ghost car messages
uint32_t green_flag_triggered_time = 0;
uint32_t safetycar_flash_interval = 0;
uint32_t green_flag_millis_flash = 0;
uint32_t jump_start_millis_flash = 0;

LightMode light_mode = START_LIGHT_MODE;

HAL_StatusTypeDef dma_status_pwm = HAL_OK;
uint8_t startupState = 0;
uint32_t start_light_change_time = 0;


volatile uint16_t difference_channel2 = 0;
volatile uint16_t last_rising = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
CuMessage decode_manchester(uint16_t message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline uint16_t circ_index(uint16_t i) {
	return i % CIRC_BUF_LEN;
}

static inline uint16_t circ_next(uint16_t i) {
	return (i + 1) % CIRC_BUF_LEN;
}

static inline uint16_t circ_prev(uint16_t i) {
	return (i + CIRC_BUF_LEN - 1) % CIRC_BUF_LEN;
}

static inline uint16_t circ_distance(uint16_t from, uint16_t to) {
	return (to >= from) ? (to - from) : (CIRC_BUF_LEN - from + to);
}

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
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  start_lights_init(&htim1, TIM_CHANNEL_1, TIM_DIER_CC1DE);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // green
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // red

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3); //
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4); //

  light_mode = HAL_GPIO_ReadPin(GPIOA, LIGHT_MODE_SJ_Pin) == 1 ? START_LIGHT_MODE : PIT_LANE_LIGHT_MODE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		difference_rw = circ_distance(data_manchester_circular_read_index, data_manchester_circular_index);
		if (3 < difference_rw){
			uint16_t read_to = data_manchester_circular_index;
			uint16_t read_index = data_manchester_circular_read_index;

			while (read_index != read_to){

				/*
				 * Process
				 *
				 * Filter the message of interest here
				 *
				 * For the lights we need safety car and status messages.
				 *
				 * If the safety car is released, choose this one. Otherwise we only
				 * need the status messages.
				 *
				 * Status messages
				 * 	lights should be indicated with 0x17 (and)
				 * 	jump start indicated with (((0x000F << 8) & message) == (0x001 << 11)) && ((0x000D << 4) & message)
				 *
				 * Safety car message
				 * 	indicated with 0b0000001111000000;
				 */

				if (( data_manchester[read_index].data> 512-1 ) && (data_manchester[read_index].data < 1024)){
					// car message
					if ( (data_manchester[read_index].data & (0x0007 << 6)) == (0x0007 << 6)){
						// safety car message
						if (data_manchester[read_index].data & (0x01 << 1)){
							// safety car active - high priority message
							message_to_proceed = data_manchester[read_index].data;
						} else if (message_to_proceed == 0){
							// safety car message  - low priority message
							// necessary to detect if release was withdrawn
							message_to_proceed = data_manchester[read_index].data;
						}
					}
				} else if ((data_manchester[read_index].data > 4096-1) && (data_manchester[read_index].data < 8192)){
					// cu message

					if ((((0x000F << 8) & data_manchester[read_index].data) == (0x001 << 11)) && ((0x000D << 4) & data_manchester[read_index].data)){
						// jump start
						message_to_proceed = data_manchester[read_index].data;
					} else if (((0x0001 << 3) & data_manchester[read_index].data) && (0x0007 & data_manchester[read_index].data)){
						// start light
						message_to_proceed = data_manchester[read_index].data;
					}

				}

				if (message_to_proceed > 0)
					track_data_available = 1;

				read_index = circ_next(read_index); // ends with actual write index
			}
			data_manchester_circular_read_index = read_index;
		}


		/*
		 * Process the messages
		 *
		 * Assumption is, that the execution interval is smaller than 75ms.
		 * This is the time until the next message block will be sent by the control unit.
		 *
		 * If this is the case, only one message contains data of interest.
		 * A really unlikely case is, when the safety car button will be pressed in the same
		 * message block the starting lights went off. But this is almost impossible due to
		 * physical limits. Try it, but you have round about 6.5ms to engage safety car key
		 * after the start lights went off ;)
		 *
		 */
		if (track_data_available == 1){
			track_data_available = 0;

			// decode cu message
			CuMessage decoded_data = decode_manchester(message_to_proceed);
			message_to_proceed = 0;

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
				  if (!start_light_value.red_count && (millis - start_light_change_time > 400)){
					  update_race_state(&race_state, RACE_STATE_OPEN);
					  if (PIT_LANE_LIGHT_MODE == light_mode)
						  start_light_value.update_flag = 1;
				  }
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


		/*
		 * passive handling of state changes
		 */

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

		/*
		 * state changes come with changes to the traffic lights
		 */
		if (start_light_value.update_flag){
			start_light_value.update_flag = 0;

			/*
			 * SET TRACK LIGHTS
			 */
			uint8_t track_light_green = 0;
			uint8_t track_light_red = 0;

			if (race_state.curr == RACE_STATE_RED_FLAG)
				track_light_red = 100;

			if (race_state.curr == RACE_STATE_YELLOW_FLAG)
				if (start_light_value.yellow_flag){
					track_light_green = 100;
					track_light_red = 66;
				}

			if (start_light_value.green_flag)
				track_light_green = 100;

			TIM3->CCR1 = track_light_green;
			TIM3->CCR2 = track_light_red;

			  /*
			   * SET PIT LANE LIGHT
			   */
			  if (PIT_LANE_LIGHT_MODE == light_mode){
				  if ((RACE_STATE_RED_FLAG == race_state.curr) || (RACE_STATE_JUMP_START == race_state.curr) ||
						  ((RACE_STATE_START_PROC == race_state.curr))){
					  for (uint16_t lights = 0; lights < 3; lights++){
						  if (0 == lights)
							  start_lights_set_colour(lights, 0, 0, 0);
						  else {
							  start_lights_set_colour(lights, 200, 0, 0);
						  }
					  }
				  } else {
					  for (uint16_t lights = 0; lights < 3; lights++){
						  if (0 == lights)
							  start_lights_set_colour(lights, 0, 100, 0);
						  else {
							  start_lights_set_colour(lights, 0, 0, 0);
						  }
					  }
				  }
			  }

			  /*
			   * SET START LIGHT
			   */
			  if (START_LIGHT_MODE == light_mode){
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
			  }

			  dma_status_pwm = start_lights_refresh();
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
  hi2c1.Init.Timing = 0x10805D88;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 75-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IO_POWER_RAIL_INT_GPIO_Port, IO_POWER_RAIL_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IO_POWER_RAIL_INT_Pin */
  GPIO_InitStruct.Pin = IO_POWER_RAIL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IO_POWER_RAIL_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIGHT_MODE_SJ_Pin */
  GPIO_InitStruct.Pin = LIGHT_MODE_SJ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIGHT_MODE_SJ_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3) {
		/*
		 * 32 ticks are 50 us. This shortens the isr execution time by day and night.
		 * This is because we can replace the division with a bit shift.
		 */

		//interrupts_total++;

		// falling edge 1
		if (HAL_TIM_GetActiveChannel(htim) == HAL_TIM_ACTIVE_CHANNEL_3){
			uint16_t cnt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			difference_channel2 = cnt - data_manchester[data_manchester_circular_index].last_falling;
			last_rising = (uint16_t)(cnt - data_manchester[data_manchester_circular_index].last_rising);

			if (last_rising > 132){ // 206.25 us - maximum should be 200 us for 010 pattern
				data_manchester_circular_index = circ_next(data_manchester_circular_index);
				data_manchester[data_manchester_circular_index].gap = (uint16_t)(cnt - data_manchester[data_manchester_circular_index].last_rising);
				data_manchester[data_manchester_circular_index].message_started = 0;
				data_manchester[data_manchester_circular_index].data = 0;
				data_manchester[data_manchester_circular_index].edges_total = 0;
				data_manchester[data_manchester_circular_index].first_rising_occured = 0;
			}

			if (difference_channel2 > 10){ // 15.625 us
				data_manchester[data_manchester_circular_index].last_falling = cnt;
				data_manchester[data_manchester_circular_index].edges_total++;

				//interrupts_total_falling++;

				// manchester decoding
				if (data_manchester[data_manchester_circular_index].message_started == 0){
					data_manchester[data_manchester_circular_index].start_time = cnt;
					data_manchester[data_manchester_circular_index].message_started = 1;
					data_manchester[data_manchester_circular_index].data = 0x01;
					data_manchester[data_manchester_circular_index].first_falling = cnt;
				} else {
					if ( (((cnt - data_manchester[data_manchester_circular_index].start_time + 16) >> 5) & 0x0001) == 0) { // if first bit 0 then even then shift and set first bit
						data_manchester[data_manchester_circular_index].data <<= 1;
						data_manchester[data_manchester_circular_index].data |= 0x0001;
					}
				}
			}

		}


		// rising edge
		if (HAL_TIM_GetActiveChannel(htim) == HAL_TIM_ACTIVE_CHANNEL_4){
			uint16_t cnt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			uint16_t difference_channel1 = cnt - data_manchester[data_manchester_circular_index].last_rising;
			// rising edge 0
			if (difference_channel1 > 10){
				data_manchester[data_manchester_circular_index].last_rising = cnt;
				data_manchester[data_manchester_circular_index].edges_total++;

				//interrupts_total_rising++;

				if (data_manchester[data_manchester_circular_index].message_started == 0){
					//false_rising_edges++;
				}

				if (data_manchester[data_manchester_circular_index].first_rising_occured == 0){
					data_manchester[data_manchester_circular_index].first_rising_occured = 1;
					data_manchester[data_manchester_circular_index].first_rising = cnt;
					//data_manchester[data_manchester_circular_index].first_rising_CNT = TIM1->CNT;
				}

				if ( (((cnt - data_manchester[data_manchester_circular_index].start_time + 16) >> 5) & 0x0001) == 0) { // if first bit 0 then even then shift and set first bit
					data_manchester[data_manchester_circular_index].data <<= 1;
				}
			}
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

CuMessage decode_manchester(uint16_t message){
	CuMessage msg;
	msg.value = 0;
	msg.chunk = message;
	if ((STATUS_DATA & message) == STATUS_DATA){
		if ((STARTING_LIGHT_MASK & message) == STARTING_LIGHT_MASK){
			// start light
			msg.type = CU_START_LIGHT;
			uint16_t valueReversed = (message >> 9);
			msg.value = ((valueReversed & 0b001) << 2) | (valueReversed & 0b010) | ((valueReversed & 0b100) >> 2);
		} else if ((((0x000F << 8) & message) == (0x001 << 11)) && ((0x000D << 4) & message)){
			// jump start
			msg.type = CU_CONTROLLER_JUMPSTART;
			msg.value = 1;
		} else {
			msg.type = CU_UNKNOWN;
		}

	} else if ((CAR_DATA & message) == CAR_DATA){
		if ((SAFETY_CAR_MASK & message) == SAFETY_CAR_MASK){
			msg.type = CU_CONTROLLER_SAFETY_CAR;
			msg.value = message & 0x01 << 1;
		} else {
			msg.type = CU_CONTROLLER_UNKNOWN;
		}
	} else {
		msg.type = CU_UNKNOWN;
	}
	return msg;
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
