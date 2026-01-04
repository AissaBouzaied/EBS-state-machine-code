/*
 * callback.c
 *
 *  Created on: 2025
 *      Author: Aissa
 */

#include "stm32f4xx_hal.h"
#include "../CAN/can1.h"
#include "../CAN/can_messages.h"
#include "EBS_state_machine.h"
#include "main.h"
#include "threads.h"
#include "string.h"
#include "parameters.h"
#include <stdbool.h>

uint32_t summedAPPS = 0;
uint32_t summedBPPS = 0;


#define MAXsize 4096*2*0.95
#define MINsize 4096*2*0.85


extern osThreadId_t can_rx_Handle;

static void canRxCallback(CAN_HandleTypeDef *hcan, uint32_t RxFifo);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	static uint32_t angleSamplingCount = 0;

	if (hadc->Instance == ADC1) {

		summed_angle_data_wr_ptr->apps_1+=adcAngleResultDma[0];
		summed_angle_data_wr_ptr->apps_2+=adcAngleResultDma[1];
		summed_angle_data_wr_ptr->bpps_1+=adcAngleResultDma[2];
		summed_angle_data_wr_ptr->bpps_2+=adcAngleResultDma[3];
		summed_angle_data_wr_ptr->steering_Angle_1+=adcAngleResultDma[4];
		summed_angle_data_wr_ptr->steering_Angle_2+=adcAngleResultDma[5];

		summedAPPS = (adcAngleResultDma[0] + adcAngleResultDma[1]);
		summedBPPS = (adcAngleResultDma[2] + adcAngleResultDma[3]);


		if (summedAPPS > MAXsize || summedAPPS < MINsize){
			osThreadFlagsSet(angle_sensors_Handle, APPS_ERROR);
		}else{
			osThreadFlagsClear(APPS_ERROR);

		}

		if (summedBPPS > MAXsize || summedAPPS < MINsize){
					osThreadFlagsSet(angle_sensors_Handle, APPS_ERROR);
				}else{
					osThreadFlagsClear(BPPS_ERROR);

				}

		if(++angleSamplingCount >= SAMPLING_SIZE){

			angleSamplingCount = 0;
			osSemaphoreRelease(angleDataAvailableSemaphoreHandle);
			//Switch the write pointer if task is ready
			if(osSemaphoreAcquire(angleBufferAvailableSemaphoreHandle, 0) == osOK){
				summed_angle_data_wr_ptr = (summed_angle_data_wr_ptr == &summedAngleData[0])? &summedAngleData[1]:&summedAngleData[0];
				memset(summed_angle_data_wr_ptr, 0, sizeof(angleData_t));
			}else{
				Error_Handler();
			}


		}

		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcAngleResultDma, ANGLE_ADC_CHANEL_COUNT);
	}

	if (hadc->Instance == ADC2) {
		osSemaphoreRelease(pressureDmaCpltSemaphoreHandle);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcPressureResultDma, PRESSURE_ADC_CHANEL_COUNT);
	}

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	for (uint32_t i = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); i > 0; i--) {
		canRxCallback(hcan, CAN_RX_FIFO0);
	}

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	for (uint32_t i = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1); i > 0; i--) {
		canRxCallback(hcan, CAN_RX_FIFO1);
	}
}

static void canRxCallback(CAN_HandleTypeDef *hcan, uint32_t RxFifo) {

	CAN_RxHeaderTypeDef RxHeader = { 0 };
	uint8_t RxData[8] = { 0 };

	if (HAL_CAN_GetRxMessage(hcan, RxFifo, &RxHeader, RxData) != HAL_OK) {
		Error_Handler();
	}

	//FIXME: Handle STDID and EXTID better
	if (RxHeader.IDE == CAN_ID_STD) {
		switch(RxHeader.StdId){
		case CAN1_DBU_STATUS_1_FRAME_ID:
			can1_dbu_status_1_unpack(&dbu_status_1_can_msg, RxData, CAN1_DBU_STATUS_1_LENGTH);
			osThreadFlagsSet(can_rx_Handle, NEW_DBU_MSG_FLAG);
			break;

		case CAN1_AMS_STATUS_1_FRAME_ID:
			can1_ams_status_1_unpack(&ams_status_can_msg, RxData, CAN1_AMS_STATUS_1_LENGTH);
			osThreadFlagsSet(can_rx_Handle, NEW_AMS_MSG_FLAG);
			break;

		case CAN1_ECU_STATUS_FRAME_ID:
			can1_ecu_status_unpack(&ecu_status_can_msg, RxData, CAN1_ECU_STATUS_LENGTH);
			osThreadFlagsSet(can_rx_Handle, NEW_ECU_MSG_FLAG);
			break;


		case CAN1_DV_CONTROL_TARGET_TV_FRAME_ID:
			can1_dv_control_target_tv_unpack(&as_system_can_msg, RxData, CAN1_DV_CONTROL_TARGET_TV_LENGTH);
			osThreadFlagsSet(can_rx_Handle, NEW_AS_STATUS_MSG_FLAG);
			break;

		case CAN1_DV_SYSTEM_STATUS_FRAME_ID:
			can1_dv_system_status_unpack(&dv_system_status_can_msg, RxData, CAN1_DV_SYSTEM_STATUS_LENGTH);
			osThreadFlagsSet(can_rx_Handle, NEW_DV_STATUS_MSG);
			break;
		case CAN1_RES_STATUS_FRAME_ID:
			can1_res_status_unpack(&res_status_can_msg, RxData, CAN1_RES_STATUS_LENGTH);
			osThreadFlagsSet(can_rx_Handle, NEW_RES_MSG_FLAG);
			break;
		case CAN1_VEHICLE_STATUS_FRAME_ID:
			can1_vehicle_status_unpack(&vehicle_status_can_msg, RxData, CAN1_VEHICLE_STATUS_LENGTH);
			osThreadFlagsSet(can_rx_Handle, NEW_DSPACE_MSG_FLAG);
			break;


		}



	}
	if (RxHeader.IDE ==CAN_ID_EXT){
		switch(RxHeader.ExtId){

		case CAN1_VESC_STATUS_FRAME_ID:
				can1_vesc_status_unpack(&vesc_status_can_msg, RxData, CAN1_VESC_STATUS_LENGTH);
				osThreadFlagsSet(can_rx_Handle, NEW_VESC_MSG_FLAG);
				break;

		}

	}



}

HAL_StatusTypeDef testSDCWatchdog(void){
	HAL_StatusTypeDef status;
	/* Toggle  Watchdog until active*/
	do{
		HAL_GPIO_TogglePin(WATCHDOG_GPIO_Port, WATCHDOG_Pin);
		osDelay(50);
	}while(HAL_GPIO_ReadPin(WATCHDOG_READY_GPIO_Port, WATCHDOG_READY_Pin) == GPIO_PIN_RESET);

	osDelay(500);
	
	/* Check that the watchdog is disbled when not toggled */
	if(HAL_GPIO_ReadPin(WATCHDOG_READY_GPIO_Port, WATCHDOG_READY_Pin) == GPIO_PIN_RESET){
		status = HAL_OK;
	}else{
		status = HAL_ERROR;
	}

	return status;
}

void startSystemMonitorTimers(void){
	//TIM10->CNT = 1;
	HAL_TIM_Base_Start_IT(&htim10);

	__HAL_TIM_SET_COUNTER(&htim11, 0);
	HAL_TIM_Base_Start_IT(&htim11);

	__HAL_TIM_SET_COUNTER(&htim12, 0);
	HAL_TIM_Base_Start_IT(&htim12);

	__HAL_TIM_SET_COUNTER(&htim13, 0);
	HAL_TIM_Base_Start_IT(&htim13);
}

void stopSystemMonitorTimers(void){
	HAL_TIM_Base_Stop_IT(&htim10);
	HAL_TIM_Base_Stop_IT(&htim11);
	HAL_TIM_Base_Stop_IT(&htim12);
	HAL_TIM_Base_Stop_IT(&htim13);
}

void startTimeoutCounter(uint16_t timeout){
//	__HAL_TIM_SET_COUNTER(&htim14, 0);
//	__HAL_TIM_SET_AUTORELOAD(&htim14, timeout);
//	HAL_TIM_Base_Start_IT(&htim14);

	osTimerStart(timeoutTimerHandle, timeout);
}

void stopTimeoutCounter(void){
//	HAL_TIM_Base_Stop_IT(&htim14);
	osTimerStop(timeoutTimerHandle);
}

void timeoutCallback(void *argument) {
	assert_param(argument == NULL);

	Event_t event = { 0 };
	/* USER CODE BEGIN timeoutCallback */
	event.eEventType = eTimeoutEvent;
	osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);
	stopTimeoutCounter();
	/* USER CODE END timeoutCallback */
}
// we have seperate check aswell as a check for both.
uint8_t brakePressureOverThresholdRear(float rear_pressure_bar){
	bool EBS_rear_status = false;
	bool BP_over_threshold_rear = false;

	if (rear_pressure_bar <= BRAKE_PRESSURE_THRESHOLD - BRAKE_PRESSURE_HYSTERESIS)
	{
		EBS_rear_status=false;
	}
		else if(rear_pressure_bar >= BRAKE_PRESSURE_THRESHOLD + BRAKE_PRESSURE_HYSTERESIS)
		{
			EBS_rear_status=true;
		}
			else
			{
				EBS_rear_status=EBS_rear_status;
			}

	BP_over_threshold_rear=EBS_rear_status;

	return BP_over_threshold_rear;
}
uint8_t brakePressureOverThresholdFront(float front_pressure_bar){
	bool EBS_front_status = false;
	bool BP_over_threshold_front = false;

	if (front_pressure_bar <= BRAKE_PRESSURE_THRESHOLD - BRAKE_PRESSURE_HYSTERESIS)
	{
		EBS_front_status=false;
	}
		else if(front_pressure_bar >= BRAKE_PRESSURE_THRESHOLD + BRAKE_PRESSURE_HYSTERESIS)
		{
			EBS_front_status=true;
		}
			else
			{
				EBS_front_status=EBS_front_status;
			}

	BP_over_threshold_front=EBS_front_status;

	return BP_over_threshold_front;
}

uint8_t brakePressureOverThreshold(float front_pressure_bar, float rear_pressure_bar){
	bool EBS_front_status = false;
	bool EBS_rear_status = false;
	bool BP_over_threshold_logic = false;


	if (front_pressure_bar <= BRAKE_PRESSURE_THRESHOLD - BRAKE_PRESSURE_HYSTERESIS)
			{
			EBS_front_status=false;
			}
			else if(front_pressure_bar >= BRAKE_PRESSURE_THRESHOLD + BRAKE_PRESSURE_HYSTERESIS)
			{
			EBS_front_status=true;
			}
			else
			{
			EBS_front_status=EBS_front_status;
			}

			if (rear_pressure_bar <= BRAKE_PRESSURE_THRESHOLD - BRAKE_PRESSURE_HYSTERESIS)
			{
			EBS_rear_status=false;
			}
				else if(rear_pressure_bar >= BRAKE_PRESSURE_THRESHOLD + BRAKE_PRESSURE_HYSTERESIS)
				{
				EBS_rear_status=true;
				}
					else
					{
					EBS_front_status=EBS_front_status;
					}
			if(EBS_front_status && EBS_rear_status)
			{
			BP_over_threshold_logic=true;
			}
			else
			{
			BP_over_threshold_logic=false;
			}

			return BP_over_threshold_logic;

}
/*We check if the pressure values are in error zone 0-0.5 volts or 4.5-5 volts */
uint8_t brakePressureSensorError(float front_pressure_raw, float rear_pressure_raw){
	bool SensorError=false;
if (front_pressure_raw>BRAKE_PRESSURE_UPPERLIMIT|| rear_pressure_raw>BRAKE_PRESSURE_UPPERLIMIT) //maybe change values too low margin of error
{
	SensorError=true;
}
else if(front_pressure_raw<BRAKE_PRESSURE_LOWERLIMIT_FRONT || rear_pressure_raw<BRAKE_PRESSURE_LOWERLIMIT_REAR)//maybe change values too low margin of error
{
	SensorError=true;
}
else
{
	SensorError=false;
}
return SensorError;
}
/*We check if open (0), to front box(2) or closed(3) and send that message on CAN every second */

uint8_t SDC_CAN_MESSAGE(int CAN_SEND, int SDC_STATUS_CAN){
if(CAN_SEND){
	switch(SDC_STATUS_CAN){

	case 0:
		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
						== osOK) {
					ebs_status_can_msg.ebs_state_machine =
					CAN1_EBS_STATUS_SDC_STATUS_OPEN_CHOICE;
					osMutexRelease(ebs_status_can_message_mutex_Handle);
				} else
					Error_Handler();
		break;
	case 2:
		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
						== osOK) {
					ebs_status_can_msg.ebs_state_machine =
					CAN1_EBS_STATUS_SDC_STATUS_TO_FRONTBOX_CHOICE;
					osMutexRelease(ebs_status_can_message_mutex_Handle);
				} else
					Error_Handler();
		break;
	case 3:
		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
						== osOK) {
					ebs_status_can_msg.ebs_state_machine =
					CAN1_EBS_STATUS_SDC_STATUS_CLOSED_CHOICE;
					osMutexRelease(ebs_status_can_message_mutex_Handle);
				} else
					Error_Handler();
		break;
	}
}
}
