#ifndef __HEART_RATE_H__
#define __HEART_RATE_H__
#include <stdint.h>
#include <stdlib.h>
#include "stm32wbxx_hal.h"
#include <string.h>

uint8_t checkForBeat(int32_t sample);
int16_t averageDCEstimator(int32_t *p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);

#endif
