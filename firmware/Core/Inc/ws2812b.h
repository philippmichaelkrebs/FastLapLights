/*
 * ws2812b.h
 *
 *  Created on: Apr 2, 2025
 *      Author: philipp
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#define WS2812B_T0H 			1 	// 0.4us
#define WS2812B_T1H				2	// 0.8us
#define WS2812B_RES_PERIOD		40 // gt 40 cycls or above 50us
#define WS2812B_LED_BITS		24U // Composition: G7, G6, .. , G0, R7, ... , R0, B7, ..., B0

typedef struct
{
	uint8_t g;
	uint8_t r;
	uint8_t b;
} WS2812B_LED;

#endif /* INC_WS2812B_H_ */
