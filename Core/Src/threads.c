/*
 * threads.c
 *
 *
 *      Author: Aissa
 */
#include "threads.h"
#include "stm32f4xx_hal.h"
#include "../CAN/can1.h"
#include "../CAN/can_messages.h"
#include "EBS_state_machine.h"
#include "state_machine_common.h"
#include "main.h"
#include "callback.h"
#include "parameters.h"
#include <stdbool.h>
#include "EBS_state_machine.h"



static void setUpCanFilter(void);

uint16_t adcAngleResultDma[ANGLE_ADC_CHANEL_COUNT] = { 0 };
uint16_t adcPressureResultDma[PRESSURE_ADC_CHANEL_COUNT] = { 0 };

angleData_t summedAngleData[2] = { 0 };
angleData_t *summed_angle_data_rd_ptr = &summedAngleData[1];
angleData_t *summed_angle_data_wr_ptr = summedAngleData;

float Global_EBS_pressure1_bar=0;
float Global_EBS_pressure2_bar=0;
_Bool Global_BP_over_threshold=false;
_Bool vescglobal=true;

/* USER CODE BEGIN Header_taskEbsStateMachine */
/**
 * @brief  Function implementing the ebs_sm_ thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskEbsStateMachine */
static StateMachine_t ebs_sm = { EbsInit };
void taskEbsStateMachine(void *argument) {
	static Event_t event;
	static State_t status;
	static StateHandler prevState;

	static Event_t const onEntryEvent = { onEntry };
	static Event_t const onExitEvent = { onExit };
	event = onEntryEvent;
	for (;;) {
		prevState = ebs_sm.state;
		status = (ebs_sm.state)(&ebs_sm, &event); // Execute the current state

		if (status == TRANSITION) { // If a transition is required, run the onExit and onEntry functions of the respective states.
			(void) (*prevState)(&ebs_sm, &onExitEvent);
			status = (*ebs_sm.state)(&ebs_sm, &onEntryEvent);
		}

		osMessageQueueGet(ebs_sm_event_queue_Handle, &event, osPriorityNone,
		osWaitForever); // Wait incoming events
	}
}

/* USER CODE BEGIN Header_taskCanRx */
/**
 * @brief Function implementing the can_rx_ thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskCanRx */
void taskCanRx(void *argument) {
	uint32_t flags = 0;

	setUpCanFilter();

	for (;;) {
		flags = osThreadFlagsWait(
				NEW_ECU_MSG_FLAG | NEW_DBU_MSG_FLAG | NEW_AMS_MSG_FLAG
						| NEW_RES_MSG_FLAG | NEW_AS_STATUS_MSG_FLAG
						| NEW_VESC_MSG_FLAG | NEW_DSPACE_MSG_FLAG,
				osFlagsWaitAny, osWaitForever);

		if (flags == osErrorTimeout) {
			Error_Handler();
		} else {

			if (flags & NEW_ECU_MSG_FLAG) {
				handle_new_ecu_msg();
			}

			if (flags & NEW_DBU_MSG_FLAG) {
				handle_new_dbu_msg();
			}

			if (flags & NEW_AMS_MSG_FLAG) {
				handle_new_ams_msg();
			}

			if (flags & NEW_RES_MSG_FLAG) {
				handle_new_res_msg();
			}

			if (flags & NEW_AS_STATUS_MSG_FLAG) {
				handle_new_as_system_msg();
			}

			if (flags & NEW_VESC_MSG_FLAG) {
				handle_new_vesc_msg();
			}
			if (flags & NEW_DSPACE_MSG_FLAG) {
				handle_new_dspace_msg();

			}
			if (flags & NEW_DV_STATUS_MSG) {
				handle_new_dv_system_status_msg();
			}

		}

	}
	/* USER CODE END taskCanRx */
}

/* USER CODE BEGIN Header_taskCanTx */
/**
 * @brief Function implementing the can_tx_ thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskCanTx */
void taskCanTx(void *argument) {

	uint32_t flags;
	osStatus_t status;

	uint8_t txSteering[8] = {0};
	uint8_t txEbs[8] = {0};
	uint8_t txPedals[8] = {0};

	uint32_t txMailbox;

	/* Start the CAN bus */
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

	for (;;) {
		flags = osThreadFlagsWait(
		NEW_ANGLE_DATA_READY_FLAG | NEW_EBS_DATA_READY_FLAG, osFlagsWaitAny,
		osWaitForever);

		if (flags == osErrorTimeout) {
			Error_Handler();

		}

		if (flags & NEW_ANGLE_DATA_READY_FLAG) {
			status = osMutexAcquire(can_message_mutex_Handle, osWaitForever);
			if (status != osOK) {
				Error_Handler();
				}

			can1_fb_pedals_pack(txPedals, &pedals_can_msg, CAN1_FB_PEDALS_LENGTH);
			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
				osDelay(1);
			}
			HAL_CAN_AddTxMessage(&hcan1, &pedals_can_msg_header, txPedals,
			&txMailbox);

			can1_steering_pack(txSteering, &steering_can_msg, CAN1_STEERING_LENGTH);
			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
				osDelay(1);
			}
			HAL_CAN_AddTxMessage(&hcan1, &steering_can_msg_header, txSteering,
			&txMailbox);

			osMutexRelease(can_message_mutex_Handle);
			}

		if (flags & NEW_EBS_DATA_READY_FLAG) {
			status = osMutexAcquire(ebs_status_can_message_mutex_Handle, 100);
			if (status != osOK) {
				Error_Handler();
			}

			can1_ebs_status_pack(txEbs, &ebs_status_can_msg,
			CAN1_EBS_STATUS_LENGTH);

			uint32_t i = 0;
			while ((HAL_CAN_AddTxMessage(&hcan1, &ebc_status_can_msg_header,
					txEbs, &txMailbox) != HAL_OK) && (i < MAX_ITERATOR)) {
				i++;
			}


			osMutexRelease(ebs_status_can_message_mutex_Handle);

		}

	}
	/* USER CODE END taskCanTx */
}

/* USER CODE BEGIN Header_taskAngleSensor */
/**
 * @brief Function implementing the angle_sensors_ thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskAngleSensor */

/*void taskAngleSensor2(void )
 xSemaphoreCreateBinary();
 */

void taskAngleSensor(void *argument) {

	osStatus_t statusAngle;

	/* ADC Is triggered by TIM2 Update Event */
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcAngleResultDma,
			ANGLE_ADC_CHANEL_COUNT);

	while (1) {

		statusAngle = osSemaphoreAcquire(angleDataAvailableSemaphoreHandle,
				osWaitForever);

		if (statusAngle != osOK) {
			Error_Handler();
		}

		statusAngle = osMutexAcquire(can_message_mutex_Handle, osWaitForever);

		if (statusAngle != osOK) {
			Error_Handler();
		}

		pedals_can_msg.apps_1 = (summed_angle_data_rd_ptr->apps_1)
				/ SAMPLING_SIZE;
		pedals_can_msg.apps_2 = (summed_angle_data_rd_ptr->apps_2)
				/ SAMPLING_SIZE;
		pedals_can_msg.bpps_1 = (summed_angle_data_rd_ptr->bpps_1)
				/ SAMPLING_SIZE;
		pedals_can_msg.bpps_2 = (summed_angle_data_rd_ptr->bpps_2)
				/ SAMPLING_SIZE;
		steering_can_msg.steering_angle_1 =
				(summed_angle_data_rd_ptr->steering_Angle_1) / SAMPLING_SIZE;
		steering_can_msg.steering_angle_2 =
				(summed_angle_data_rd_ptr->steering_Angle_2) / SAMPLING_SIZE;

		summed_angle_data_rd_ptr =
				(summed_angle_data_rd_ptr == &summedAngleData[0]) ?
						&summedAngleData[1] : &summedAngleData[0];

		if (osThreadFlagsGet() & APPS_ERROR) {
			pedals_can_msg.apps_error = 1;

		} else {
			pedals_can_msg.apps_error = 0;

		}

		osMutexRelease(can_message_mutex_Handle);

		osThreadFlagsSet(can_tx_Handle, NEW_ANGLE_DATA_READY_FLAG);

		osSemaphoreRelease(angleBufferAvailableSemaphoreHandle);

	}
}

/* USER CODE END taskAngleSensor */

/* USER CODE BEGIN Header_taskPressureSensor */
/**
 * @brief Function implementing the pressure_sensor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskPressureSensor */
void taskPressureSensor(void *argument) {
	/* USER CODE BEGIN taskPressureSensor */
	/* Infinite loop */

	osStatus_t status;
	Event_t event;

	bool BP_over_threshold = false;
	bool BP_over_threshold_rear = false;
	bool BP_over_threshold_front = false;
	bool prevBP_over_threshold = false;
	bool prevBP_over_threshold_rear = false;
	bool prevBP_over_threshold_front = false;

	float BP_front_bar;
	float BP_rear_bar;
	float EBS_pressure1_bar;
	float EBS_pressure2_bar;
	float BP_front_raw;
	float BP_rear_raw;
	float BP_front_bar_CAN;
	float BP_rear_bar_CAN;

	uint8_t BP_error_counter=0;




	/* ADC Is triggered by TIM8 Update Event */
	HAL_TIM_Base_Start(&htim8);

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adcPressureResultDma,
	PRESSURE_ADC_CHANEL_COUNT);
	while (1) {
		status = osSemaphoreAcquire(pressureDmaCpltSemaphoreHandle,
				osWaitForever);

		if (status != osOK) {
			Error_Handler();
		}

		BP_front_bar=adcPressureResultDma[0]*ADC_TO_BAR_FRONT_BP-13.5; // prev -12.5
		BP_rear_bar=adcPressureResultDma[1]*ADC_TO_BAR_REAR_BP-16; //prev -20

		EBS_pressure1_bar=ADC_TO_BAR_EBS*adcPressureResultDma[2] - 2.0f; //prev -2.5 front
		EBS_pressure2_bar=ADC_TO_BAR_EBS*adcPressureResultDma[3] - 2.0f; // prev -2.5 rear

		Global_EBS_pressure1_bar=EBS_pressure1_bar;
		Global_EBS_pressure2_bar=EBS_pressure2_bar;

		BP_front_bar_CAN = BP_front_bar;
		BP_rear_bar_CAN = BP_rear_bar;
		if (BP_front_bar>=10) {BP_front_bar_CAN = BP_front_bar;}
		else{BP_front_bar_CAN =0;}

		if (BP_rear_bar>=10) {BP_rear_bar_CAN = BP_rear_bar;}
		else{BP_rear_bar_CAN =0;}


		BP_front_raw=adcPressureResultDma[0];
		BP_rear_raw=adcPressureResultDma[1];

		status = osMutexAcquire(ebs_status_can_message_mutex_Handle,
						osWaitForever);

		if (status != osOK) {
		Error_Handler();
		}

		ebs_status_can_msg.brake_pressure_front = can1_ebs_status_brake_pressure_front_encode(BP_front_bar_CAN);
		ebs_status_can_msg.brake_pressure_rear = can1_ebs_status_brake_pressure_rear_encode(BP_rear_bar_CAN);
		ebs_status_can_msg.ebs_pressure_1 = can1_ebs_status_ebs_pressure_1_encode(EBS_pressure1_bar);
		ebs_status_can_msg.ebs_pressure_2 = can1_ebs_status_ebs_pressure_2_encode(EBS_pressure2_bar);

		osMutexRelease(ebs_status_can_message_mutex_Handle);

		osThreadFlagsSet(can_tx_Handle, NEW_EBS_DATA_READY_FLAG);

		BP_over_threshold = brakePressureOverThreshold(BP_front_bar, BP_rear_bar);
		BP_over_threshold_rear = brakePressureOverThresholdRear(BP_rear_bar);
		BP_over_threshold_front = brakePressureOverThresholdRear(BP_front_bar);

		Global_BP_over_threshold = BP_over_threshold;

		if (BP_over_threshold_rear != prevBP_over_threshold_rear)
		{

			event.eEventType = BP_over_threshold_rear ? eBP_RearHighEvent : eBP_RearLowEvent;

			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
			osWaitForever);
		}



		if (BP_over_threshold_front != prevBP_over_threshold_front)
		{

			event.eEventType = BP_over_threshold_front ? eBP_FrontHighEvent : eBP_FrontLowEvent;

			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
			osWaitForever);
		}



		if (BP_over_threshold != prevBP_over_threshold) {

			event.eEventType = BP_over_threshold ? eBP_HighEvent : eBP_LowEvent;

			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
			osWaitForever);
		}
		prevBP_over_threshold=BP_over_threshold;
		prevBP_over_threshold_front=BP_over_threshold_front;
		prevBP_over_threshold_rear=BP_over_threshold_rear;




		/*if bp front or bp rear gets sensor values outside their range, then send fault message*/
		if(brakePressureSensorError(BP_front_raw,BP_rear_raw)){BP_error_counter++;} else{BP_error_counter=0;}

		if ( BP_error_counter>=10 )
		{

			event.eEventType = eBP_SensorErrorEvent;

			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
			osWaitForever);
		}

		/*We check if the actual pressure(from sensors) aligns with what we want activate/release EBS.
		 * Semaphore =1 Released, =0 Activated. BP_over_threshold =1 activated, =0 released.
		if( osSemaphoreGetCount(BrakeNotActiveSemaphoreHandle) == BP_over_threshold )
		{
			counter=counter+1;
		}
		else
		{
			counter=0;
		}

		if (counter>=2)
		{
			//event.eEventType = eBP_DiscrepancyEvent;

			//osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
			//osWaitForever);

			counter=0;
		}
		Not in use very annoying to test /implement
		*/

	}

}
/* USER CODE END taskPressureSensor */

/* USER CODE BEGIN Header_taskDigitalInputs */
/**
 * @brief Function implementing the digital_inputs_ thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskDigitalInputs */
void taskDigitalInputs(void *argument) {
	uint8_t SDC_status = 0;

	bool PS1_status = false;
	bool prevPS1_status = false;
	bool prevPS2_status = false;
	bool PS2_status = false;

	int SEND_CAN = 0;
	int SDC_STATUS_CAN=0;

	uint8_t PS1_counter = 0;
	uint8_t PS2_counter = 0;

	uint32_t now_time = 0;
	uint32_t prev_time = 0;

	const uint8_t PS_maxcount = 15; /*150ms */

	Event_t event = { 0 };
	/* Infinite loop */
	for (;;) {

		 now_time = HAL_GetTick();
		    if ((now_time % 1000 == 0) && (prev_time != now_time))
		    {
		        SEND_CAN = 1;   // one-shot pulse
		    }
		    else
		    {
		        SEND_CAN = 0;
		    }

		    prev_time = now_time;

		SDC_status = (HAL_GPIO_ReadPin(SDC_FROM_REAR_GPIO_Port,
		SDC_FROM_REAR_Pin) << 1)
				| (HAL_GPIO_ReadPin(SDC_TO_REAR_GPIO_Port, SDC_TO_REAR_Pin));

		switch (SDC_status & 0x03) {
		case 0x00: // SDC Open in rear
			/* Change timer 1 IDLE State (OIS Bit) to LOW  and stop it */
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			SDC_STATUS_CAN=0;
			SDC_CAN_MESSAGE(SEND_CAN,SDC_STATUS_CAN);
			break;
		case 0x01: // Illegal, SDC closed in front (not detectable)
			break;
		case 0x02: // SDC Closed in rear
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 31250);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			SDC_STATUS_CAN=2;
			SDC_CAN_MESSAGE(SEND_CAN,SDC_STATUS_CAN);
			break;
		case 0x03: // SDC Closed in front and rear
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 62499);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			SDC_STATUS_CAN=3;
			SDC_CAN_MESSAGE(SEND_CAN,SDC_STATUS_CAN);
			break;
		}
		//Simple CAN_timer for SDC messages;

		/* Check if primary tank state has changed and update the status,
		 * send event after 150ms if the state remains the same
		 * IMPORTANT NOTE ENABLE THE CHECK IN STATEMACHINE(IT IS DISABLED)*/
		if (Global_EBS_pressure1_bar>=8) {PS1_status =true;}
		else
		{
			PS1_status=false;
		}

		//PS1_status = HAL_GPIO_ReadPin(PS1_GPIO_Port, PS1_Pin);
		if (PS1_status != prevPS1_status)
		{
			PS1_counter=0;
		}
		else
		{
			if (PS1_counter<PS_maxcount)
			{
				PS1_counter++;
			}
		}
		if (PS1_counter>=PS_maxcount)
		{
			event.eEventType = PS1_status ? ePS1_HighEvent : ePS1_LowEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
					10);
		}
		prevPS1_status= PS1_status;

		/* Check if secondary tank state has changed and update the status */
		PS2_status = HAL_GPIO_ReadPin(PS2_GPIO_Port, PS2_Pin);
		if (PS2_status != prevPS2_status)
		{
			PS2_counter=0;
		}
		else
		{
			if (PS2_counter<PS_maxcount)
			{
				PS2_counter++;
			}
		}
		if (PS2_counter>=PS_maxcount)
		{
			event.eEventType = PS2_status ? ePS2_HighEvent : ePS2_LowEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
					10);
		}
		prevPS2_status= PS2_status;

		osDelay(10);
	}
	/* USER CODE END taskDigitalInputs */
}

void taskHeartbeat(void *argument) {
	/* USER CODE BEGIN taskHeartbeat */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(GENERAL_LED_GPIO_Port, GENERAL_LED_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(GENERAL_LED_GPIO_Port, GENERAL_LED_Pin,
				GPIO_PIN_RESET);
		osDelay(900);
	}
	/* USER CODE END taskHeartbeat */
}

void taskCANNodeWatchDog(void *argument)
{	Event_t event = { 0 };
	static int32_t SystemTime_ms;
	while(1){
		SystemTime_ms=HAL_GetTick();
		vescglobal=true;
		if ((SystemTime_ms - last_vesc_time_ms)>= CANMESSAGETIMEOUT_MS){
			event.eEventType = eVESCNotOkEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 10);
			vescglobal=false;
		}
		osDelay(10);
	}
/*
	while( ((last_res_time_ms>-1) && (last_as_time_ms>-1) && (last_vesc_time_ms>-1) && (last_dspace_time_ms>-1) ) == false)
	{
		osDelay(100);
	}
	event.eEventType = eAllCANNodesOk;
	osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);
	while(1) {
		SystemTime_ms=HAL_GetTick();
		if ( ((SystemTime_ms - last_res_time_ms) >= CANMESSAGETIMEOUT_MS)||(res_status_can_msg.radio_state != 1) )
		{
					event.eEventType = eRESNotOkEvent;
					osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 10);
		}
		if ( (SystemTime_ms - last_as_time_ms) >= CANMESSAGETIMEOUT_MS)
		{
			event.eEventType = eASSystemNotOkEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 10);
		}
		if ( (SystemTime_ms - last_vesc_time_ms) >= CANMESSAGETIMEOUT_MS)
		{
			event.eEventType = eVESCNotOkEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 10);
		}

		if ( (SystemTime_ms - last_dspace_time_ms) >= CANMESSAGETIMEOUT_MS)
		{
			event.eEventType = eDSPACENotOkEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 10);
		}
		osDelay(10);

	}
*/


}

static void setUpCanFilter(void) {
	CAN_FilterTypeDef sFilterConfig;

	/* Set up CAN filer */
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMaskIdHigh = CAN1_DBU_STATUS_1_FRAME_ID<<5;
	sFilterConfig.FilterMaskIdLow = CAN1_AMS_STATUS_1_FRAME_ID<<5;
	sFilterConfig.FilterIdHigh = CAN1_ECU_STATUS_FRAME_ID<<5;
	sFilterConfig.FilterIdLow = CAN1_DV_CONTROL_TARGET_TV_FRAME_ID<<5;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMaskIdHigh = CAN1_DV_SYSTEM_STATUS_FRAME_ID<<5;
	sFilterConfig.FilterMaskIdLow = CAN1_RES_STATUS_FRAME_ID<<5;
	sFilterConfig.FilterIdHigh = CAN1_VEHICLE_STATUS_FRAME_ID<<5;
	sFilterConfig.FilterIdLow =0; //not used

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	// for extended can id
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 2;
	sFilterConfig.FilterMaskIdHigh = CAN1_VESC_STATUS_FRAME_ID>>13;
	sFilterConfig.FilterMaskIdLow = (CAN1_VESC_STATUS_FRAME_ID<<3)|0x04;
	sFilterConfig.FilterIdHigh =0;//not in use
	sFilterConfig.FilterIdLow = 0;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
			Error_Handler();
		}

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
			Error_Handler();
		}


}

