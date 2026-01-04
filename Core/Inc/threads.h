/*
 * threads.h
 *
 *  Created on: Mar 14, 2024
 *      Author: philiplind
 */

#ifndef INC_THREADS_H_
#define INC_THREADS_H_


#define ANGLE_BUFFER_SIZE 120
#define PRESSURE_BUFFER_SIZE 80

#define ANGLE_ADC_CHANEL_COUNT 		6
#define PRESSURE_ADC_CHANEL_COUNT	4
#define SAMPLING_SIZE 10




/* -------- CAN MSG FLAGS ----------- */
#define NEW_ECU_MSG_FLAG		1 << 0
#define NEW_DBU_MSG_FLAG		1 << 1
#define NEW_AMS_MSG_FLAG		1 << 2
#define NEW_RES_MSG_FLAG		1 << 3
#define NEW_AS_STATUS_MSG_FLAG	1 << 4
#define NEW_VESC_MSG_FLAG		1 << 5
#define NEW_DSPACE_MSG_FLAG			1 << 6
#define NEW_DV_STATUS_MSG		1 << 7

/* -------- Angle Sensor Thread Flags ----------- */
#define NEW_ANGLE_DATA_READY_FLAG		1 << 0
#define NEW_EBS_DATA_READY_FLAG			1 << 1
#define APPS_ERROR 						1 << 2
#define BPPS_ERROR 						1 << 3

#include "main.h"

typedef struct {

	uint32_t apps_1;
	uint32_t apps_2;
	uint32_t bpps_1;
	uint32_t bpps_2;
	uint32_t steering_Angle_1;
	uint32_t steering_Angle_2;

}angleData_t;

extern  uint16_t adcAngleResultDma[ANGLE_ADC_CHANEL_COUNT];
extern  uint16_t adcPressureResultDma[PRESSURE_ADC_CHANEL_COUNT];
extern angleData_t * summed_angle_data_wr_ptr;
extern angleData_t summedAngleData[2];

extern float Global_EBS_pressure1_bar;
extern float Global_EBS_pressure2_bar;
extern _Bool Global_BP_over_threshold;
extern _Bool vescglobal;




#endif /* INC_THREADS_H_ */
