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
	ALIGNMENT_VERTICAL,
	ALIGNMENT_HORIZONTAL
}StartLightAlignment;

typedef enum {
	RACE,
	QUALIFYING_TRAINING
}SessionType;

typedef enum {
	RACE_STATE_STARTUP,
	RACE_STATE_RED_FLAG,
	RACE_STATE_YELLOW_FLAG,
	RACE_STATE_GREEN_FLAG,
	RACE_STATE_START_PROC,
	RACE_STATE_JUMP_START,
	RACE_STATE_OPEN
} RaceState;

typedef enum {
	UART_TX_BUFFER_FULL,
	UART_TX_BUFFER_TIMEOUT,
	UART_TX_FORCE
} UartTxTransmitReason;

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
	uint16_t first_rising;
	uint16_t data;
} ManchesterDecoder;

typedef struct {
	uint8_t fuel;
	uint8_t position;
	uint8_t finished;
	uint8_t laps;
} DriverData;

typedef struct {
	DriverData 	driver_data[6];
	uint8_t		fastest_lap_driver;
	uint8_t		start_lights;
	uint8_t		lap_count;
	uint8_t		lap_count_nibble_completed;
	uint8_t		reset_event;
	RaceState	race_state;
	SessionType	session_type;
} RaceInfo;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CIRC_BUF_LEN       	8U
#define UART_TX_BUF_LEN		(32U) //32U

#define HALF_BIT_TICKS   32
#define FULL_BIT_TICKS   64
#define GAP_THRESHOLD    132   // consider 160 as safer midpoint
#define GLITCH_FILTER    10    // ignore pulses shorter than ~16 µs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim16_ch1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint32_t 			track_counter = 0;
uint32_t 			millis_sync_prev = 0;
uint16_t 			millis_sync_state_change_flag = 0;

// manchester decoder
volatile ManchesterDecoder 	data_manchester[CIRC_BUF_LEN] = {0};
volatile uint16_t 			data_manchester_circular_index = 0;
volatile uint16_t 			data_manchester_circular_read_index = 0;
uint16_t 					difference_rw = 0;
volatile uint16_t 			track_data_available = 0;
uint16_t 					message_to_proceed = 0;
uint16_t					last_edge = 0;

// race state
RaceStateHistory 	race_state = {RACE_STATE_STARTUP, RACE_STATE_STARTUP};
StartLightValue 	start_light_value = {0};
uint8_t 			safetycar_debounce = 0; // neccessary because of ghost car messages
uint32_t 			green_flag_triggered_time = 0;
uint32_t 			safetycar_flash_interval = 0;
uint32_t 			green_flag_millis_flash = 0;
uint32_t 			jump_start_millis_flash = 0;

LightMode 			light_mode = START_LIGHT_MODE;
StartLightAlignment start_light_alignment = ALIGNMENT_HORIZONTAL;
SessionType 		session_type = RACE;
//uint8_t 			startupState = 0;
uint32_t 			start_light_change_time = 0;
HAL_StatusTypeDef 	dma_status_pwm = HAL_OK;

// driver data
DriverData			driver_data[6]								= {0};

// uart
static 	 uint8_t	uart_tx_buffer[UART_TX_BUF_LEN] 			;
uint32_t 			uart_tx_last_transmit						= 0;
uint8_t				uart_tx_counter								= 0;
HAL_StatusTypeDef 	uart_status_pwm 							= HAL_OK;
/*
 * 0x01 - race state
 * 0x02 - driver data (fuel, laps, position)
 * 0x04 - start lights
 * 0x08 - reset
 * 0x10 - race finished
 * 0x20 - new fastest lap
 * 0x40 - session type (race, practice)
 */

volatile RaceInfo	uart_race_info = {
		.driver_data = {
				{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
				{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
				{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
				{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
				{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
				{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 }
		},
		.fastest_lap_driver = 0xFF,       // 0xFF no fastest lap yet
		.start_lights = 0,
		.lap_count = 0,
		.lap_count_nibble_completed = 1,
		.reset_event = 0,
		.race_state = RACE_STATE_STARTUP,
		.session_type = RACE
};

volatile uint16_t 	difference_channel2 = 0;
volatile uint16_t 	last_rising = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
CuMessage decode_manchester(uint16_t message);
void HAL_TIM_IC_CaptureCallback_EXTENDED(uint16_t capture, uint8_t direction);
void (*ptr_man_capture_isr)(uint16_t capture, uint8_t direction) = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * circular buffer methods - manchester decoder
 */

/*static inline uint16_t circ_index(uint16_t i) {
	return i % CIRC_BUF_LEN;
}*/

static inline uint16_t circ_next(uint16_t i) {
	return (i + 1) & 0x0007; //% CIRC_BUF_LEN;
}

/*static inline uint16_t circ_prev(uint16_t i) {
	return (i + CIRC_BUF_LEN - 1) % CIRC_BUF_LEN;
}*/

static inline uint16_t circ_distance(uint16_t from, uint16_t to) {
	return (to >= from) ? (to - from) : (CIRC_BUF_LEN - from + to);
}

static inline void update_race_state(RaceStateHistory *stateHistory, RaceState newState) {
	stateHistory->prev = stateHistory->curr;
	stateHistory->curr = newState;
	uart_race_info.race_state = newState;
}

/*
 * update functions
 */

static inline void update_start_light_value_red(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->red_count_prev = startLightValue->red_count;
	startLightValue->red_count = newValue;
	startLightValue->update_flag = 1;
	uart_race_info.start_lights = newValue;
}
static inline void update_start_light_value_green(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->green_flag_prev = startLightValue->green_flag;
	startLightValue->green_flag = newValue;
	startLightValue->update_flag = 1;
}
static inline void update_start_light_value_yellow(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->yellow_flag_prev = startLightValue->yellow_flag;
	startLightValue->yellow_flag = newValue;
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
	ptr_man_capture_isr = HAL_TIM_IC_CaptureCallback_EXTENDED;

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
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	// init ws2812b
	HAL_TIM_PWM_MspInit(&htim16);
	dma_status_pwm = start_lights_init(&htim16, TIM_CHANNEL_1, TIM_DIER_CC1DE);

	// init track interrupt
	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableIT_CC4(TIM3);
	LL_TIM_EnableCounter(TIM3);

	// init light behavior
	light_mode = HAL_GPIO_ReadPin(GPIOA, LIGHT_MODE_SJ_Pin) == 1 ? START_LIGHT_MODE : PIT_LANE_LIGHT_MODE;
	start_light_alignment = HAL_GPIO_ReadPin(START_LIGHT_ALIGNMENT_GPIO_Port, START_LIGHT_ALIGNMENT_Pin) == 1 ?
			ALIGNMENT_HORIZONTAL : ALIGNMENT_VERTICAL;
	session_type = HAL_GPIO_ReadPin(RACE_TRAINING_SESSION_SW_GPIO_Port, RACE_TRAINING_SESSION_SW_Pin) == 0 ? RACE : QUALIFYING_TRAINING;
	uart_race_info.session_type = session_type;

	// init track lights
	//HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); // red
	// Enable output
	LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);
	// Enable counter
	LL_TIM_EnableCounter(TIM14);
	// Force update generation to apply settings
	//LL_TIM_GenerateEvent_UPDATE(TIM14);


	TIM14->CCR1 = 0;//80;
	HAL_GPIO_WritePin(TRACK_GREEN_GPIO_Port, TRACK_GREEN_Pin, GPIO_PIN_RESET);

	//HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, UART_TX_BUF_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// BLOCK 1 - DECODE MANCHESTER DATA
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
						if (data_manchester[read_index].data & (0x0001 << 1)){
							// safety car active - high priority message
							message_to_proceed = data_manchester[read_index].data;
						} else if (message_to_proceed == 0){
							// safety car message  - low priority message
							// necessary to detect if release was withdrawn
							message_to_proceed = data_manchester[read_index].data;
						}
					}
				} else if ((data_manchester[read_index].data > 4096-1) && (data_manchester[read_index].data < 8192)){
					// control unit message
					track_counter++; // message block repeats every 75ms

					uint16_t encode_signal = data_manchester[read_index].data;
					uint8_t controller =
							((encode_signal >> 0) & 0x01) << 2 |
							((encode_signal >> 1) & 0x01) << 1 |
							((encode_signal >> 2) & 0x01) << 0;

					uint8_t command =
							((encode_signal >> 3) & 0x01) << 4 |
							((encode_signal >> 4) & 0x01) << 3 |
							((encode_signal >> 5) & 0x01) << 2 |
							((encode_signal >> 6) & 0x01) << 1 |
							((encode_signal >> 7) & 0x01) << 0;

					uint8_t value =
							((encode_signal >> 8) & 0x01) << 3 |
							((encode_signal >> 9) & 0x01) << 2 |
							((encode_signal >> 10) & 0x01) << 1 |
							((encode_signal >> 11) & 0x01) << 0;

					if ((11 == command) && (1 == value)){
						// jump start
						message_to_proceed = data_manchester[read_index].data;
					} else if ((16 == command) && (7 == controller)){
						// start light
						message_to_proceed = data_manchester[read_index].data;
					} else {
						// catch unprocessed cu data for uart

						// unfortunately we have to go through various if statements.
						if ((6 > controller) && !(0 == command)){
							// things we never ever want to proceed
							if (4 == command){
								if (8 > value){
									// fuel level of car - 7 is full
									uart_race_info.driver_data[controller].fuel = value;
								} else if (16 > value){
									// not really neccessary. Called at jump start
									uart_race_info.driver_data[controller].fuel = value - 8;
								}
							}else if (6 == command){
								if (value < 9){
									// postition
									value--; // position -1 to fit in 4 bits
									uart_race_info.driver_data[controller].position = value;
								} else if (9 == value) {
									// reset lap counter and positions
									uart_race_info.reset_event = 1;
									uart_race_info.lap_count = 0;
									for (uint8_t _cntrl = 0; _cntrl < 6; _cntrl++){
										uart_race_info.driver_data[controller].finished = 0;
										uart_race_info.driver_data[controller].position = 0;
										uart_race_info.driver_data[controller].fuel = 0;
										uart_race_info.driver_data[controller].laps = 0;
									}
								}
							} else if (7 == command){
								// driver finishes race
								uart_race_info.driver_data[controller].finished = 1;
							} else if (8 == command){
								// driver finishes lap / race with new fastest lap
								uart_race_info.fastest_lap_driver = controller;
								uart_race_info.driver_data[controller].laps++;
							} else if (9 == command){
								// driver finishes lap / race wiout new fastest lap
								uart_race_info.driver_data[controller].laps++;
							} else if ((10 == command) && (8 > value)){
								// fuel level of car - 7 is full
								uart_race_info.driver_data[controller].fuel = value;
							}
						} else if ((17 == command) && (7 == controller)){
							// first nibble lap count leader
							uart_race_info.lap_count = value << 4;
							uart_race_info.lap_count_nibble_completed = 0;
						} else if ((18 == command) && (7 == controller)){
							// first nibble lap count leader
							uart_race_info.lap_count |= value;
							uart_race_info.lap_count_nibble_completed = 1;
						}
					} // catch unprocessed cu data for uart end
				} // control unit message end

				if (message_to_proceed > 0)
					track_data_available = 1;

				read_index = circ_next(read_index); // ends with actual write index
			}
			data_manchester_circular_read_index = read_index;
		}

		// BLOCK 2 - PROCESS ALTERNATING PROCESSES

		/*
		 * Use control unit message interval as clock (kind of synchronization pulse)
		 * for time based alterations at the lights. This is necessary when
		 * more then one controller controls different lights on the track
		 * to avoid drift in blinks.
		 *
		 * The increment frequency is 13.3 Hz (75ms)
		 */
		millis_sync_state_change_flag = millis_sync_prev != track_counter ? 1 : 0;
		millis_sync_prev = track_counter;

		// do some stuff with 1.3 Hz (750ms)
		if ((9 < track_counter - uart_tx_last_transmit) && (1 == millis_sync_state_change_flag)){
			/*
			 * Read Race State
			 */
			session_type = HAL_GPIO_ReadPin(RACE_TRAINING_SESSION_SW_GPIO_Port, RACE_TRAINING_SESSION_SW_Pin) == 0 ? RACE : QUALIFYING_TRAINING;
			uart_race_info.session_type = session_type;


			/*
			 * Transmit race state info
			 */
			uint8_t write_index = 0;
			for (uint8_t _driver = 0; _driver < 6; _driver++){
				uart_tx_buffer[write_index++] = 0x00 | uart_race_info.driver_data[_driver].position;
				uart_tx_buffer[write_index++] = 0x00 | uart_race_info.driver_data[_driver].fuel;
				uart_tx_buffer[write_index++] = 0x00 | uart_race_info.driver_data[_driver].laps;
				uart_tx_buffer[write_index++] = 0x00 | uart_race_info.driver_data[_driver].finished;
			}

			uart_tx_buffer[write_index++] = 0x00 | uart_race_info.fastest_lap_driver;
			uart_tx_buffer[write_index++] = 0x00 | uart_race_info.start_lights;
			uart_tx_buffer[write_index++] = 0x00 | uart_race_info.lap_count;
			uart_tx_buffer[write_index++] = 0x00 | uart_race_info.reset_event;
			uart_tx_buffer[write_index++] = 0x00 | uart_race_info.race_state;
			uart_tx_buffer[write_index++] = 0x00 | uart_race_info.session_type;

			uart_tx_buffer[write_index++] = (0xFF00 & track_counter) >> 8;
			uart_tx_buffer[write_index++] = (0x00FF & track_counter);


			// set usart clearance because we dont use usart isr
			huart1.gState = HAL_UART_STATE_READY;
			uart_status_pwm = HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, UART_TX_BUF_LEN);

			// reset track reset event
			uart_race_info.reset_event = 0;
		}

		// BLOCK 3 - PROCESS THE MESSAGES THAT CONTROLS THE LIGHTS

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
						start_light_change_time = track_counter;
					}
					else if (race_state.curr == RACE_STATE_RED_FLAG){
						update_race_state(&race_state, RACE_STATE_GREEN_FLAG);
						green_flag_triggered_time = track_counter;
					}
				}
			}

			if (RACE_STATE_START_PROC == race_state.curr){
				if (!start_light_value.red_count && (track_counter - start_light_change_time > 6)){ // (!start_light_value.red_count && (millis - start_light_change_time > 400))
					update_race_state(&race_state, RACE_STATE_OPEN);
					if ((PIT_LANE_LIGHT_MODE == light_mode) || (QUALIFYING_TRAINING == session_type))
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
					green_flag_triggered_time = track_counter;
				}
			}
		}


		/*
		 * Update the lights and update race state if required (green flag)
		 */

		// start light value changed
		if (start_light_value.red_count != start_light_value.red_count_prev)
			update_start_light_value_red(&start_light_value, start_light_value.red_count);

		// SAFETY CAR
		if (RACE_STATE_YELLOW_FLAG == race_state.curr){
			if (track_counter - safetycar_flash_interval > 3){ // (millis - safetycar_flash_interval > 200){
				safetycar_flash_interval = track_counter;
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
			if (track_counter - green_flag_millis_flash > 7){ // (millis - green_flag_millis_flash > 500)
				if (!start_light_value.green_flag){
					green_flag_millis_flash = track_counter;
					update_start_light_value_green(&start_light_value, 1);
				}else {
					green_flag_millis_flash = track_counter;
					update_start_light_value_green(&start_light_value, 0);
				}
			}

			if (track_counter - green_flag_triggered_time > 142) //millis - green_flag_triggered_time > 10000
				update_race_state(&race_state, RACE_STATE_OPEN);
		}else{
			if ((1 == start_light_value.green_flag) && (RACE == session_type))
				update_start_light_value_green(&start_light_value, 0);
			else if ((0 == start_light_value.green_flag) && (QUALIFYING_TRAINING == session_type) && (RACE_STATE_OPEN == race_state.curr))
				update_start_light_value_green(&start_light_value, 1);
			else if ((1 == start_light_value.green_flag) && (QUALIFYING_TRAINING == session_type) && (RACE_STATE_OPEN != race_state.curr))
				update_start_light_value_green(&start_light_value, 0);
		}


		// JUMP START
		if (RACE_STATE_JUMP_START == race_state.curr){
			if (track_counter - jump_start_millis_flash > 7){ //(millis - jump_start_millis_flash > 500)
				jump_start_millis_flash = track_counter;
				update_start_light_value_red(&start_light_value, 5);
				if (!start_light_value.yellow_flag)
					update_start_light_value_yellow(&start_light_value, 1);
				else
					update_start_light_value_yellow(&start_light_value, 0);
			}
		}

		// BLOCK 4 - SET THE LIGHS

		/*
		 * state changes come with changes to the traffic lights
		 */
		if (start_light_value.update_flag){
			start_light_value.update_flag = 0;

			/*
			 * SET TRACK LIGHTS
			 */
			uint16_t track_light_green = 0;
			uint16_t track_light_red = 0;

			if (race_state.curr == RACE_STATE_RED_FLAG)
				track_light_red = 100;

			if (race_state.curr == RACE_STATE_YELLOW_FLAG)
				if (start_light_value.yellow_flag){
					track_light_green = 1;
					track_light_red = 66;
				}

			if (start_light_value.green_flag)
				track_light_green = 1;

			if (track_light_green)
				HAL_GPIO_WritePin(TRACK_GREEN_GPIO_Port, TRACK_GREEN_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(TRACK_GREEN_GPIO_Port, TRACK_GREEN_Pin, GPIO_PIN_RESET);
			TIM14->CCR1 = track_light_red;

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
				// it must be soldered from left to right
				if (ALIGNMENT_HORIZONTAL == start_light_alignment){
					// horizontal alignment
					if (start_light_value.yellow_flag){
						for (uint8_t ylight = 0; ylight < 5; ylight++){
							start_lights_set_colour(ylight, 80, 25 , 0); 		// front
							start_lights_set_colour(ylight+35, 80, 25 , 0); 	// back
						}
					} else {
						for (uint8_t ylight = 0; ylight < 5; ylight++){
							start_lights_set_colour(ylight, 0, 0 , 0);
							start_lights_set_colour(ylight+35, 0, 0 , 0);
						}
					}

					if (start_light_value.green_flag){
						for (uint8_t glight = 0; glight < 5; glight++){
							start_lights_set_colour(glight+5, 0, 200 , 0);		// front
							start_lights_set_colour(glight+30, 0, 200 , 0);	// back
						}
					} else {
						for (uint8_t glight = 0; glight < 5; glight++){
							start_lights_set_colour(glight+5, 0, 0 , 0);
							start_lights_set_colour(glight+30, 0, 0 , 0);
						}
					}

					// zero all red lights
					for (uint8_t light = 0; light < 20; light++){
						start_lights_set_colour(light+10, 0, 0, 0);
					}
					if (start_light_value.red_count){
						for (uint8_t light = 0; light < start_light_value.red_count; light++){
							start_lights_set_colour(light+10, 200, 0, 0);
							start_lights_set_colour(19-light, 200, 0, 0);
							start_lights_set_colour(24-light, 200, 0, 0);
							start_lights_set_colour(light+25, 200, 0, 0);
						}
					}
				} else {
					// vertical alignment of start lights
					if (start_light_value.yellow_flag){
						for (uint8_t ylight = 0; ylight < 5; ylight++){
							start_lights_set_colour(ylight*8, 80, 25 , 0); 	// front
							start_lights_set_colour(ylight*8+7, 80, 25 , 0); 	// back
						}
					} else {
						for (uint8_t ylight = 0; ylight < 5; ylight++){
							start_lights_set_colour(ylight*8, 0, 0 , 0);
							start_lights_set_colour(ylight*8+7, 0, 0 , 0);
						}
					}

					if (start_light_value.green_flag){
						for (uint8_t glight = 0; glight < 5; glight++){
							start_lights_set_colour(glight*8+1, 0, 200 , 0);		// front
							start_lights_set_colour(glight*8+1+5, 0, 200 , 0);	// back
						}
					} else {
						for (uint8_t glight = 0; glight < 5; glight++){
							start_lights_set_colour(glight*8+1, 0, 0 , 0);
							start_lights_set_colour(glight*8+1+5, 0, 0 , 0);
						}
					}

					// zero all red lights
					for (uint8_t light = 0; light < 5; light++){
						start_lights_set_colour(light*8+2, 0, 0, 0);
						start_lights_set_colour(light*8+3, 0, 0, 0);
						start_lights_set_colour(light*8+4, 0, 0, 0);
						start_lights_set_colour(light*8+5, 0, 0, 0);
					}
					if (start_light_value.red_count){
						// it must be soldered from left to right
						for (uint8_t light = 0; light < start_light_value.red_count; light++){
							start_lights_set_colour(light*8+2, 200, 0, 0);
							start_lights_set_colour(light*8+3, 200, 0, 0);
							start_lights_set_colour(light*8+4, 200, 0, 0);
							start_lights_set_colour(light*8+5, 200, 0, 0);
						}
					}
				}
			}

			dma_status_pwm = start_lights_refresh(&htim16);
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**TIM3 GPIO Configuration
  PA8   ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, 0);
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 74;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_IC_POLARITY_BOTHEDGE);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM14);

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  TIM_InitStruct.Prescaler = 47;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 100;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM14);
  LL_TIM_OC_EnablePreload(TIM14, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM14, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM14, LL_TIM_CHANNEL_CH1);
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**TIM14 GPIO Configuration
  PA7   ------> TIM14_CH1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 20-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  HAL_GPIO_WritePin(TRACK_GREEN_GPIO_Port, TRACK_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_LIGHT_ALIGNMENT_Pin RACE_TRAINING_SESSION_SW_Pin LIGHT_MODE_SJ_Pin */
  GPIO_InitStruct.Pin = START_LIGHT_ALIGNMENT_Pin|RACE_TRAINING_SESSION_SW_Pin|LIGHT_MODE_SJ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRACK_GREEN_Pin */
  GPIO_InitStruct.Pin = TRACK_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRACK_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback_EXTENDED(uint16_t capture, uint8_t direction){
	/*
	 * 32 ticks are 50 us. This shortens the isr execution time by day and night.
	 * This is because we can replace the division with a bit shift.
	 */
	uint16_t diff = capture - last_edge;
	if (diff <= GLITCH_FILTER) {
		return; // glitch: nothing to do
	}

	if (GAP_THRESHOLD < diff){
		// if last edge occurs more then 400us ago
		data_manchester_circular_index = circ_next(data_manchester_circular_index);
		last_edge = capture;
		data_manchester[data_manchester_circular_index].data = 0x0001;
	}
	else{
		if ((48 < diff) && (diff < 80)){
			last_edge = capture;
			data_manchester[data_manchester_circular_index].data <<= 1;
			if (0 == direction)
				data_manchester[data_manchester_circular_index].data |= 0x0001;
		}
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM16)
		start_lights_dma_callback();
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
#ifdef USE_FULL_ASSERT
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
