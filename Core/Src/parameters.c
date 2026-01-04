/*
 * parameters.c
 *
 *  Created on: Jan 28, 2025
 *      Author: Aissa Bouzaied
 */

#include "parameters.h"

const float BRAKE_PRESSURE_HYSTERESIS=5.0f; // in bar
const float BRAKE_PRESSURE_THRESHOLD=17.0f; //in bar
const float BRAKE_PRESSURE_UPPERLIMIT=4000.0f; // above this value -> faulty sensor
const float BRAKE_PRESSURE_LOWERLIMIT_FRONT=100.0f; //below this value -> faulty sensor
const float BRAKE_PRESSURE_LOWERLIMIT_REAR=100.0f; //below this value -> faulty sensor



const float ADC_TO_BAR_REAR_BP = 0.059082;  // prev 0.059082;

const float ADC_TO_BAR_FRONT_BP = 0.059082; //  prev  0.059082;

const float ADC_TO_BAR_EBS = 0.0036926;    // prev 0.0036926;







//const float LSB_TO_VOLT = 0.000806f; // converts adc value 0-4096 to voltage 0-3.3V

//it is now adjusted with the individual tolerances in mind at 5.1V
const float LSB_TO_VOLT[4] = {
    0.001044f,  // Channel 0 (Front brake) BP1 3162.0 max adc
    0.001118f,  // Channel 1 (Rear brake)  BP2 2951.0 max adc
    0.001042f,  // Channel 2 (EBS Pressure 1)  PS1 3166.0 max adc
    0.001044f   // Channel 3 (EBS Pressure 2)  PS2 3160.0 max adc
};

const uint32_t CANMESSAGETIMEOUT_MS = 200;

