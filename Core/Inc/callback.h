/*
 * callback.h
 *
 *  Created on: Mar 14, 2024
 *      Author: philiplind
 */

#ifndef INC_CALLBACK_H_
#define INC_CALLBACK_H_
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef testSDCWatchdog(void);
void startTimeoutCounter(uint16_t timeout);
void stopTimeoutCounter(void);
//void timeoutCallback(void);
void startSystemMonitorTimers(void);
void stopSystemMonitorTimers(void);

uint8_t brakePressureOverThreshold(float front_pressure_bar, float rear_pressure_bar);
uint8_t brakePressureOverThresholdFront(float front_pressure_bar);
uint8_t brakePressureOverThresholdRear(float rear_pressure_bar);
uint8_t SDC_CAN_MESSAGE(int CAN_SEND, int SDC_STATUS_CAN);
uint8_t brakePressureSensorError(float front_pressure_bar, float rear_pressure_bar);


#endif /* INC_CALLBACK_H_ */
