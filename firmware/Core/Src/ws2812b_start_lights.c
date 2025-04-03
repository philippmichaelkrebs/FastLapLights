#include <ws2812b_start_lights.h>

uint16_t 			START_LIGHTS_DMA_BUF[START_LIGHTS_DMA_BUF_LEN];
WS2812B_LED 		START_LIGHTS_DATA[START_LIGHTS];
volatile uint8_t 	START_LIGHTS_DMA_CPLT_FLAG;
TIM_HandleTypeDef	*START_LIGHTS_TIM;
uint32_t			START_LIGHTS_TIM_CHANNEL;
uint32_t			START_LIGHTS_TIM_DIER_CCxDE;

HAL_StatusTypeDef START_LIGHTS_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t ccxde){
	if (htim == NULL)
		return HAL_ERROR;  // Return an error if htim is NULL

	START_LIGHTS_TIM = htim;
	START_LIGHTS_TIM_CHANNEL = channel;
	START_LIGHTS_TIM_DIER_CCxDE = ccxde;

	HAL_StatusTypeDef halStatus = HAL_TIM_PWM_Init(START_LIGHTS_TIM);

	for (uint16_t index = 0; index < START_LIGHTS_DMA_BUF_LEN; index++)
		START_LIGHTS_DMA_BUF[index] = 0;

	for (uint16_t index = 0; index < START_LIGHTS; index++){
		START_LIGHTS_DATA[index].r = 0;
		START_LIGHTS_DATA[index].g = 0;
		START_LIGHTS_DATA[index].b = 0;
	}

	// set dma clearance
	START_LIGHTS_DMA_CPLT_FLAG = 1;

	return halStatus;
}

void START_LIGHTS_SetColour(uint8_t index, uint8_t r, uint8_t g, uint8_t b){
	if ((index < START_LIGHTS) && (START_LIGHTS_DMA_CPLT_FLAG == 1)){
		START_LIGHTS_DATA[index].r = r;
		START_LIGHTS_DATA[index].g = g;
		START_LIGHTS_DATA[index].b = b;
	}
}

HAL_StatusTypeDef START_LIGHTS_Update(){
	// previous transfer completed

	if (1 != START_LIGHTS_DMA_CPLT_FLAG){
		return HAL_BUSY;
	}

	uint16_t _bufIndex = 0;
	for (uint16_t led = 0; led < START_LIGHTS; led++){

		for (uint8_t greenIndex = 0; greenIndex < 8; greenIndex++){
			if (START_LIGHTS_DATA[led].g & (0x01 << (7-greenIndex)))
				START_LIGHTS_DMA_BUF[_bufIndex] = WS2812B_T1H;
			else
				START_LIGHTS_DMA_BUF[_bufIndex] = WS2812B_T0H;
			_bufIndex++;
		}

		for (uint8_t redIndex = 0; redIndex < 8; redIndex++){
			if (START_LIGHTS_DATA[led].r & (0x01 << (7-redIndex)))
				START_LIGHTS_DMA_BUF[_bufIndex] = WS2812B_T1H;
			else
				START_LIGHTS_DMA_BUF[_bufIndex] = WS2812B_T0H;
			_bufIndex++;
		}

		for (uint8_t blueIndex = 0; blueIndex < 8; blueIndex++){
			if (START_LIGHTS_DATA[led].b & (0x01 << (7-blueIndex)))
				START_LIGHTS_DMA_BUF[_bufIndex] = WS2812B_T1H;
			else
				START_LIGHTS_DMA_BUF[_bufIndex] = WS2812B_T0H;
			_bufIndex++;
		}
	}

	HAL_StatusTypeDef halStatus = HAL_TIM_PWM_Start_DMA(START_LIGHTS_TIM, START_LIGHTS_TIM_CHANNEL, (uint32_t *) START_LIGHTS_DMA_BUF, START_LIGHTS_DMA_BUF_LEN);

	if (HAL_OK == halStatus)
		START_LIGHTS_DMA_CPLT_FLAG = 0;

	return halStatus;
}

void START_LIGHTS_DMA_Callback(){
	// leads to dma error
	//HAL_StatusTypeDef status = HAL_TIM_PWM_Stop_DMA(START_LIGHTS_TIM, START_LIGHTS_TIM_CHANNEL);

	// reset dma because of internal hal dma handling
	// https://community.st.com/t5/stm32-mcus-embedded-software/dma-pwm-outputs-2-pulses-per-buffer-value-stm32l072xx/td-p/262220


	// TODO Work out
	// - why DMA throws error by stopping via HAL
	// - which flags reset by HAL after dma stops

	// Enable the update request DMA event
	SET_BIT(START_LIGHTS_TIM->Instance->DIER, TIM_DIER_UDE);

	// Disable capture/compare DMA event
	CLEAR_BIT(START_LIGHTS_TIM->Instance->DIER, START_LIGHTS_TIM_DIER_CCxDE);

	// set clearance for dma
	START_LIGHTS_DMA_CPLT_FLAG = 1;
}


