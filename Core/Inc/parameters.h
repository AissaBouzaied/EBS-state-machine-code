/*
 * parameters.h
 *
 *  Created on: Jan 28, 2025
 *      Author: Aissa Bouzaied
 */

#ifndef INC_PARAMETERS_H_
#define INC_PARAMETERS_H_
#include "main.h"

extern const float BRAKE_PRESSURE_HYSTERESIS;
extern const float BRAKE_PRESSURE_THRESHOLD;
extern const float BRAKE_PRESSURE_UPPERLIMIT;
extern const float BRAKE_PRESSURE_LOWERLIMIT_FRONT;
extern const float BRAKE_PRESSURE_LOWERLIMIT_REAR;



extern const float ADC_TO_BAR_REAR_BP;
extern const float ADC_TO_BAR_FRONT_BP;
extern const float ADC_TO_BAR_EBS;


extern const float LSB_TO_VOLT[4];

extern const uint32_t CANMESSAGETIMEOUT_MS;


#endif /* INC_PARAMETERS_H_ */
