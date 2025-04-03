/*
 * manchester.cpp
 *
 *  Created on: Mar 16, 2025
 *      Author: philipp
 */

#include <track_data_decoder.h>

static uint32_t shiftTimestampArray(int32_t *interrupts, uint32_t size){
	// shift interrupts
	uint32_t shiftIndex = size -1;
	while (shiftIndex >= 0 && interrupts[shiftIndex] < 0) {
		shiftIndex--;
	}
	if (shiftIndex < 0) {
		return size;
	}

	for (int i = shiftIndex; i >= 0; i--)
		interrupts[size -1 - shiftIndex + i] = interrupts[i];
	for (uint32_t i = 0; i < size - shiftIndex - 1; i++)
		interrupts[i] = 0;

	return size - 1 - shiftIndex;
}

/*
 * convert timestamps to index. Even numbers are the middle of an cycle, odd numbers represents edges.
 */
static void convertTimestamps(uint16_t *interrupts, uint32_t size, uint32_t firstValue){

	// convert timestamps to indexes
	uint16_t temp;
	uint16_t firstTimeStamp = interrupts[firstValue];
	for (uint16_t i = firstValue; i < size; i++){
		if (i == firstValue){
			interrupts[i] = 0;
		}
		else{
			temp = interrupts[i] - firstTimeStamp + 25; // 50us clock
			interrupts[i] = temp / 50;
		}
	}
}

static uint16_t decodeBits(uint16_t *interrupts, uint32_t size){
	uint16_t bits = 0;
	// Find first nonzero index
	uint16_t arrayPosition = 0;
	while ((arrayPosition < size) && (interrupts[arrayPosition] == 0)){
		arrayPosition++;
	}
	arrayPosition--; // start at 0

	uint16_t whileIndex = 0;
	while (whileIndex <= (interrupts[size-1]+1)){

		bits = bits << 1;
		if (interrupts[arrayPosition] == whileIndex)
			bits |= 1;

		if (interrupts[arrayPosition] <= whileIndex)
			if (arrayPosition < (size - 1))
				arrayPosition++;

		whileIndex += 2;
	}

	return bits;
}

static uint8_t messageSize(uint16_t message) {
    if (message == 0) return 0;

    for (int i = 15; i >= 0; --i) { // Start MSB
        if (message & (1 << i)) {
            return i; //
        }
    }
    return 0;
}

CuMessage decodeManchester(const int32_t *interrupts, uint32_t size){
	int32_t _interrupts[size];
	uint32_t firstValue = size;
	for (uint32_t i = 0; i < size; i++)
		_interrupts[i] = interrupts[i];
	firstValue = shiftTimestampArray(_interrupts, size);

	uint16_t _dataToConvert[size];
	for (uint16_t i = 0; i < size; i++)
		_dataToConvert[i] = _interrupts[i];
	convertTimestamps(_dataToConvert, size, firstValue);

	uint16_t _dataToDecode[size];
	for (uint16_t i = 0; i < size; i++)
		_dataToDecode[i] = _dataToConvert[i];
	uint16_t decodedMessage = decodeBits(_dataToDecode, size);

	// shift if last bits are 10. The 0 gets lost because of detecting falling edges only.
	uint8_t _messageSize = messageSize(decodedMessage) + 1;
	if ((_messageSize == 12) || (_messageSize == 9) || (_messageSize == 7)){
		size += 1;
		decodedMessage <<= 1;
	}


	CuMessage msg;
	msg.value = 0;
	msg.chunk = decodedMessage;
	msg.size = _messageSize;
	if ((STATUS_DATA & decodedMessage) == STATUS_DATA){
		if ((STARTING_LIGHT_MASK & decodedMessage) == STARTING_LIGHT_MASK){
			// start light
			msg.type = CU_START_LIGHT;
			uint16_t valueReversed = (decodedMessage >> 9);
			msg.value = ((valueReversed & 0b001) << 2) | (valueReversed & 0b010) | ((valueReversed & 0b100) >> 2);
		} else if ((((0x000F << 8) & decodedMessage) == (0x001 << 11)) && ((0x000D << 4) & decodedMessage)){
			// jump start
			msg.type = CU_CONTROLLER_JUMPSTART;
			msg.value = 1;
		} else {
			msg.type = CU_UNKNOWN;
		}

	} else if ((CAR_DATA & decodedMessage) == CAR_DATA){
		if ((SAFETY_CAR_MASK & decodedMessage) == SAFETY_CAR_MASK){
			msg.type = CU_CONTROLLER_SAFETY_CAR;
			msg.value = decodedMessage & 0x01 << 1;
		} else {
			msg.type = CU_CONTROLLER_UNKNOWN;
		}
	} else {
		msg.type = CU_UNKNOWN;
	}
	return msg;
}

