/*
 * manchester.h
 *
 *  Created on: Mar 16, 2025
 *      Author: philipp
 */

#ifndef INC_TRACK_DATA_DECODER_H_
#define INC_TRACK_DATA_DECODER_H_


typedef enum {
	CU_UNKNOWN,
	CU_SYSTEM_CALL,
	CU_SYSTEM_UNKNOWN,
	CU_SYSTEM_RESET,
	CU_START_LIGHT,
	CU_CONTROLLER_SAFETY_CAR,
	CU_CONTROLLER_DRIVER,
	CU_CONTROLLER_UNKNOWN,
	CU_CONTROLLER_SETTINGS,
	CU_CONTROLLER_JUMPSTART
} CuDataType;


typedef struct
{
	uint16_t chunk;
	CuDataType type;
	uint16_t value;
} CuMessage;

static const uint16_t STATUS_DATA = 		0b0001000000000000;
static const uint16_t CAR_DATA = 			0b0000001000000000;
static const uint16_t STARTING_LIGHT_MASK = 0b0001000000001111;
static const uint16_t SAFETY_CAR_MASK = 	0b0000001111000000;

#endif /* INC_MANCHESTER_H_ */
