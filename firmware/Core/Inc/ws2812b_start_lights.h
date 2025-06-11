/*
 * manchester.h
 *
 *  Created on: Mar 16, 2025
 *      Author: philipp
 */

#ifndef INC_WS2812B_START_LIGHTS_H_
#define INC_WS2812B_START_LIGHTS_H_

#include "stm32c0xx_hal.h"
#include "ws2812b.h"

#define START_LIGHTS				40  // (5 * 4) * 2
#define START_LIGHTS_DMA_BUF_LEN	WS2812B_RES_PERIOD + (START_LIGHTS * WS2812B_LED_BITS)

extern 			WS2812B_LED			START_LIGHTS_DATA[START_LIGHTS];
extern 			uint16_t			START_LIGHTS_DMA_BUF[START_LIGHTS_DMA_BUF_LEN];
extern volatile	uint8_t				START_LIGHTS_DMA_CPLT_FLAG;
extern			TIM_HandleTypeDef	*START_LIGHTS_TIM;
extern			uint32_t			START_LIGHTS_TIM_CHANNEL;
extern			uint32_t			START_LIGHTS_TIM_DIER_CCxDE;

HAL_StatusTypeDef 	start_lights_init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t ccxde);
void				start_lights_set_colour(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
HAL_StatusTypeDef 	start_lights_refresh(TIM_HandleTypeDef *htim);
void				start_lights_dma_callback();

#endif /* INC_WS2812B_START_LIGHTS_H_ */
