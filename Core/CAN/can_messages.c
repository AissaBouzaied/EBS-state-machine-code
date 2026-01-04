/*
 * can_messages.c
 *
 *  Created on: Mar 14, 2024
 *      Author: tore
 */
#include "can_messages.h"
#include "state_machine_common.h"
#include "parameters.h"
#include "main.h"
#include "EBS_state_machine.h"

/* RX CAN Messagse */
struct can1_dbu_status_1_t dbu_status_1_can_msg = { 0 };
struct can1_ams_status_1_t ams_status_can_msg = { 0 };
struct can1_ecu_status_t ecu_status_can_msg = { 0 };
struct can1_res_status_t res_status_can_msg = { 0 };
struct can1_dv_control_target_tv_t as_system_can_msg = { 0 };
struct can1_vesc_status_t vesc_status_can_msg = { 0 };
struct can1_dv_system_status_t dv_system_status_can_msg = { 0 };
struct can1_vehicle_status_t vehicle_status_can_msg = { 0 };


/* TX CAN Messages */
struct can1_ebs_status_t ebs_status_can_msg = { 0 };
struct can1_fb_pedals_t pedals_can_msg = { 0 };
struct can1_steering_t steering_can_msg = { 0 };

const CAN_TxHeaderTypeDef pedals_can_msg_header = { .StdId = CAN1_FB_PEDALS_FRAME_ID,
		.IDE = CAN1_FB_PEDALS_IS_EXTENDED, .DLC = CAN1_FB_PEDALS_LENGTH };


const CAN_TxHeaderTypeDef steering_can_msg_header = { .StdId =
		CAN1_STEERING_FRAME_ID, .IDE = CAN1_STEERING_IS_EXTENDED, .DLC =
		CAN1_STEERING_LENGTH };

const CAN_TxHeaderTypeDef ebc_status_can_msg_header = { .StdId =
CAN1_EBS_STATUS_FRAME_ID, .IDE =
CAN1_EBS_STATUS_IS_EXTENDED, .DLC =
CAN1_EBS_STATUS_LENGTH };

//global variable
int32_t last_res_time_ms=0;
int32_t last_as_time_ms=0;
int32_t last_vesc_time_ms=0;
int32_t last_dspace_time_ms=0;

uint8_t global_dbu_mission=0;


void handle_new_dbu_msg(void) {
	Event_t event = { 0 };
	static struct can1_dbu_status_1_t old_dbu_status_1_can_msg = { 0 };

	if (dbu_status_1_can_msg.selected_mission
			!= old_dbu_status_1_can_msg.selected_mission) {
		switch (dbu_status_1_can_msg.selected_mission) {
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_NO_MISSION_SELECTED_CHOICE:
			event.eEventType = eNoMissionSelectedEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
					0);
			global_dbu_mission=0;

			break;
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_MANUAL_DRIVING_SELECTED_CHOICE:
			event.eEventType = eManualMissionSelectedEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
					0);
			global_dbu_mission=1;
			break;

		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_ACCELERATION_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_AUTOCROSS_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_BRAKETEST_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_INSPECTION_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_SKIDPAD_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_TRACKDRIVE_SELECTED_CHOICE:
			event.eEventType = eAutonomousMissionSelectedEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
					0);
			global_dbu_mission=2;
			break;
		}

	}

	old_dbu_status_1_can_msg = dbu_status_1_can_msg;

}
void handle_new_ams_msg(void) {
	Event_t event = { 0 };
	static struct can1_ams_status_1_t old_ams_status_can_msg = { 0 };

	if (ams_status_can_msg.sc_closed != old_ams_status_can_msg.sc_closed) {
		event.eEventType =
				ams_status_can_msg.sc_closed ? eSDC_CloseEvent : eSDC_OpenEvent;

		if (osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
				0) != osOK) {
			//TODO: Handle error
		}
	}

	old_ams_status_can_msg = ams_status_can_msg;
}
void handle_new_ecu_msg(void) {
	Event_t event = { 0 };
	static struct can1_ecu_status_t old_ecu_status_can_msg = { 0 };

	if (ecu_status_can_msg.ts_off != old_ecu_status_can_msg.ts_off) {
		event.eEventType =
				ecu_status_can_msg.ts_off ? eTS_OffEvent : eTS_OnEvent;

		if (osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
				0) != osOK) {
			//TODO: Handle error
		}
	}

	if (ecu_status_can_msg.rst_button != old_ecu_status_can_msg.rst_button) {
		if (ecu_status_can_msg.rst_button) {
			event.eEventType = eResetEvent;

			if (osMessageQueuePut(ebs_sm_event_queue_Handle, &event,
					osPriorityNone, 0) != osOK) {
				//TODO: Handle error
			}
		}

	}

	old_ecu_status_can_msg = ecu_status_can_msg;
}


//Testing new way to handle messages using time since system started
// minus time since last message received and this should not be over 200ms otherwise failure

void handle_new_as_system_msg(void) {
	last_as_time_ms=HAL_GetTick();
}

void handle_new_res_msg(void) {
	last_res_time_ms=HAL_GetTick();
}

void handle_new_vesc_msg(void) {
	last_vesc_time_ms = HAL_GetTick();
}
void handle_new_dspace_msg(void) {
	last_dspace_time_ms= HAL_GetTick();
}

void handle_new_dv_system_status_msg(void){
	Event_t event = { 0 };
		static struct can1_dv_system_status_t old_dv_system_status_can_msg = { 0 };

		if (dv_system_status_can_msg.as_state
				!= old_dv_system_status_can_msg.as_state) {
			switch (dv_system_status_can_msg.as_state) {
			case CAN1_DV_SYSTEM_STATUS_AS_STATE_DRIVING_CHOICE:
				event.eEventType = eAsDrivingEvent;
				osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
						0);
				break;
			case CAN1_DV_SYSTEM_STATUS_AS_STATE_EMERGENCY_BRAKE_CHOICE:
				event.eEventType = eAsEmergencyEvent;
				osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
						0);
				break;

			case CAN1_DV_SYSTEM_STATUS_AS_STATE_FINISH_CHOICE:
				event.eEventType = eAsFinishEvent;
				osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone,
						0);
				break;

			}
		}

		old_dv_system_status_can_msg = dv_system_status_can_msg;
}

