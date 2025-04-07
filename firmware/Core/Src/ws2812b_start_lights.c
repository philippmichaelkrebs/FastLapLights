#include <ws2812b_start_lights.h>

uint16_t 			START_LIGHTS_DMA_BUF[START_LIGHTS_DMA_BUF_LEN];
WS2812B_LED 		START_LIGHTS_DATA[START_LIGHTS];
volatile uint8_t 	START_LIGHTS_DMA_CPLT_FLAG;
TIM_HandleTypeDef	*START_LIGHTS_TIM;
uint32_t			START_LIGHTS_TIM_CHANNEL;
uint32_t			START_LIGHTS_TIM_DIER_CCxDE;

HAL_StatusTypeDef start_lights_init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t ccxde){
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

void start_lights_set_colour(uint8_t index, uint8_t r, uint8_t g, uint8_t b){
	if ((index < START_LIGHTS) && (START_LIGHTS_DMA_CPLT_FLAG == 1)){
		START_LIGHTS_DATA[index].r = r;
		START_LIGHTS_DATA[index].g = g;
		START_LIGHTS_DATA[index].b = b;
	}
}

HAL_StatusTypeDef start_lights_refresh(){
	// previous transfer completed

	if ((START_LIGHTS_TIM->hdma[START_LIGHTS_TIM_CHANNEL]->State == HAL_DMA_STATE_READY) && !START_LIGHTS_DMA_CPLT_FLAG){
		return START_LIGHTS_TIM->hdma[START_LIGHTS_TIM_CHANNEL]->State;
	}

	uint16_t bufIdx = 1; // start by one to let dma start properly
	for (uint16_t led = 0; led < START_LIGHTS; led++) {
			uint8_t *lds = (uint8_t *)&START_LIGHTS_DATA[led];
			// g, r, b - MSB
			for (uint8_t byte = 0; byte < 3; byte++)
				for (int8_t bit = 7; bit >= 0; bit--)
					START_LIGHTS_DMA_BUF[bufIdx++] =
							(lds[byte] & (1 << bit)) ? WS2812B_T1H : WS2812B_T0H;
		}

	HAL_StatusTypeDef halStatus = HAL_TIM_PWM_Start_DMA(START_LIGHTS_TIM, START_LIGHTS_TIM_CHANNEL, (uint32_t *) START_LIGHTS_DMA_BUF, START_LIGHTS_DMA_BUF_LEN);

	if (HAL_OK == halStatus)
		START_LIGHTS_DMA_CPLT_FLAG = 0;

	return halStatus;
}

void start_lights_dma_callback(){
	// leads to dma error
	//HAL_StatusTypeDef status = HAL_TIM_PWM_Stop_DMA(START_LIGHTS_TIM, START_LIGHTS_TIM_CHANNEL);

	// reset dma because of internal hal dma handling
	// https://community.st.com/t5/stm32-mcus-embedded-software/dma-pwm-outputs-2-pulses-per-buffer-value-stm32l072xx/td-p/262220


	// TODO Work out
	// - why DMA throws error by stopping via HAL
	// - which flags reset by HAL after dma stops
	// - how to set HAL_TIM_PWM_Start_DMA to update event. Almost impossible
	//   change TIM_DMA_CC1 to TIM_DMA_UPDATE / TIM_DMA_ID_UPDATE leads to dma reset during
	//   invoke of HAL_DMA_Start_IT.
	//   That drives me bonkers
	/*
	 * does not work but its helpful:
	 * https://community.st.com/t5/stm32cubemx-mcus/advanced-timer-1-with-dma-hangs-if-compare-register-equals-0/td-p/246877
	 *
	 * does not work as well
	 * https://electronics.stackexchange.com/questions/471148/stm32f1xx-how-to-use-dma-to-write-to-timx-ccr
	 *
	 * lets test this next:
	 * https://community.st.com/t5/stm32-mcus-products/tim1-update-does-not-request-dma/td-p/747451
	 */

	// Enable the update request DMA event
	SET_BIT(START_LIGHTS_TIM->Instance->DIER, TIM_DIER_UDE);

	// Disable capture/compare DMA event
	CLEAR_BIT(START_LIGHTS_TIM->Instance->DIER, START_LIGHTS_TIM_DIER_CCxDE);

	// set clearance for dma
	START_LIGHTS_DMA_CPLT_FLAG = 1;
}


