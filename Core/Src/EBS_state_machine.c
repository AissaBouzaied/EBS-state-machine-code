/*
 * states.c
 *
 *  Created on: Feb 2025
 *      Author: Aissa
 */
#include "main.h"
#include "cmsis_os.h"
#include "EBS_state_machine.h"
#include "../CAN/can_messages.h"
#include "callback.h"
#include "threads.h"

static void ActivateEBS();
static void ReleaseEBS();

static void ReleaseRearEBS();
static void ReleaseFrontEBS();
static void ActivateRearEBS();
static void ActivateFrontEBS();

extern float Global_EBS_pressure1_bar;
extern float Global_EBS_pressure2_bar;
extern _Bool Global_BP_over_threshold;
extern uint8_t global_dbu_mission;
/* State machine variables */
eFault_t fault = eNoFault;

State_t EbsInit(StateMachine_t *const StateMachine, Event_t const *const pEvent) {
	State_t status = IGNORED;

	switch (pEvent->eEventType) {
	case onEntry:
		/* Activate EBS */
		ActivateEBS();

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_INIT_CHOICE;
			ebs_status_can_msg.ebs_status =
			CAN1_EBS_STATUS_EBS_STATUS_EBS_UNAVAILABLE_CHOICE;
			ebs_status_can_msg.ebs_error = 0;

			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();

		/* Test Watchdog */
		if (testSDCWatchdog() == HAL_OK) {
			StateMachine->state = EbsWaitMission;
			status = TRANSITION;
		} else {
			StateMachine->state = EbsFault;
			fault = eWatchdogFault;
			status = TRANSITION;
		}

		break;

	case onExit:
		status = HANDLED;
		break;

	case eSDC_OpenEvent:
		status = IGNORED;
		break;

	case eNoMissionSelectedEvent:
	case eManualMissionSelectedEvent:
	case eAutonomousMissionSelectedEvent:
		status = IGNORED;
		break;

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;
	}
	return status;
}

State_t EbsWaitMission(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {

	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:
		/* Activate EBS */
		ActivateEBS();

		/* Start toggling watchdog */
		HAL_TIM_Base_Start_IT(&htim7);

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_WAIT_MISSION_CHOICE;
			ebs_status_can_msg.ebs_status =
			CAN1_EBS_STATUS_EBS_STATUS_EBS_UNAVAILABLE_CHOICE;
			ebs_status_can_msg.ebs_error = 0;

			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();

		status = HANDLED;

		break;

	case eSDC_OpenEvent:
		status = IGNORED;
		break;

	case eNoMissionSelectedEvent:
		status = IGNORED;
		break;

	case eManualMissionSelectedEvent:
		StateMachine->state = EbsUnavailable;
		status = TRANSITION;
		break;

	case eAutonomousMissionSelectedEvent:
		StateMachine->state = EbsWaitTanks;
		status = TRANSITION;
		break;
	case eVESCNotOkEvent:
		status = IGNORED;
		break;

	default:// 0 none selected 1 manual 2 auto
		if (global_dbu_mission==2){//dv
		StateMachine->state = EbsWaitTanks;
		status = TRANSITION;
		break;
		}
		else if (global_dbu_mission==1){//manual
		StateMachine->state = EbsUnavailable;
		status = TRANSITION;
		}
		else if (global_dbu_mission==0){//nomission
		status = IGNORED;
		}
		else {
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;
		}

	}

	return status;
}

State_t EbsWaitTanks(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {
	State_t status;
	Event_t event = { 0 };

	switch (pEvent->eEventType) {
	case onEntry:

		/* Check if the tanks are already pumped */
		event.eEventType =
				HAL_GPIO_ReadPin(PS1_GPIO_Port, PS1_Pin) ?
						ePS1_HighEvent : ePS1_LowEvent;
		osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);

		event.eEventType =
				HAL_GPIO_ReadPin(PS2_GPIO_Port, PS2_Pin) ?
						ePS2_HighEvent : ePS2_LowEvent;
		osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_WAIT_TANK_CHOICE;
			ebs_status_can_msg.ebs_status =
			CAN1_EBS_STATUS_EBS_STATUS_EBS_UNAVAILABLE_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();

		status = HANDLED;
		break;

	case eSDC_OpenEvent:
		status = IGNORED;
		break;

	case ePS1_HighEvent://very suspect solution check if this works, didnt want to global variable

		if (HAL_GPIO_ReadPin(PS2_GPIO_Port, PS2_Pin) && Global_BP_over_threshold){


			StateMachine->state = EbsReleaseFront;
			status = TRANSITION;
		} else {
			status = IGNORED;
		}
		break;

	case ePS1_LowEvent:
		status = IGNORED;
		break;

	case ePS2_HighEvent:

		if ( (Global_EBS_pressure1_bar>=8) && Global_BP_over_threshold) {

			StateMachine->state = EbsReleaseFront;
			status = TRANSITION;
			}
		else {
			status = IGNORED;
		}
		break;
	case eBP_HighEvent:
		status = IGNORED;
		break;

	case eBP_LowEvent:
		status = IGNORED;
		break;


	case ePS2_LowEvent:
		status = IGNORED;
		break;

	case onExit:
			status = HANDLED;
			break;


	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;

	}

	return status;
}

State_t EbsReleaseFront(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {
	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:

		/* Close SDC */
		HAL_GPIO_WritePin(CLOSE_SDC_GPIO_Port, CLOSE_SDC_Pin, GPIO_PIN_SET);
		/* Release EBS */
		ReleaseFrontEBS();

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_WAIT_RELEASE_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();

		startSystemMonitorTimers();
		status = HANDLED;
		break;

	case eBP_FrontLowEvent:
		/* Primary and Secondary brakes released */
		startTimeoutCounter(1000); // Wait 1000 ms between release and test
		break;

	case eTimeoutEvent:
		StateMachine->state = EbsTestFront;
		status = TRANSITION;
		break;

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;

	}

	return status;
}
State_t EbsTestFront(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {
	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:
		/* Activate Primary EBS */
		ActivateFrontEBS();

		startTimeoutCounter(200); // Start timeout timer, give brakes 200 ms to react

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_TEST_PRIMARY_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();
		status = HANDLED;
		break;

	case onExit:
		stopTimeoutCounter(); // Stop timeout timer;
		status = HANDLED;
		break;

	case eBP_FrontHighEvent:
		/* Both brakes activated */
		StateMachine->state = EbsReleaseRear;
		status = TRANSITION;
		break;

	case eTimeoutEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eBrakeTookToLong;
		break;

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;

	}

	return status;
}
State_t EbsReleaseRear(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {
	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:
		/* Release EBS */
		ReleaseRearEBS();

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_WAIT_RELEASE_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();

		startSystemMonitorTimers();
		status = HANDLED;
		break;

	case eBP_RearLowEvent:
		/* Primary and Secondary brakes released */
		startTimeoutCounter(1000); // Wait 1000 ms between release and test
		break;

	case eTimeoutEvent:
		StateMachine->state = EbsTestRear;
		status = TRANSITION;
		break;

	case eBP_HighEvent:
		status=HANDLED;
		break;
	case onExit:
		stopTimeoutCounter(); // Stop timeout timer;
		status = HANDLED;
		break;

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;

	}

	return status;
}
State_t EbsTestRear(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {
	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:
		/* Activate Primary EBS */
		/* Activate Secondary EBS */
		ActivateRearEBS();

		startTimeoutCounter(200); // Start timeout timer, give brakes 200 ms to react

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_TEST_PRIMARY_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();
		status = HANDLED;
		break;

	case onExit:
		stopTimeoutCounter(); // Stop timeout timer;
		status = HANDLED;
		break;

	case eBP_RearHighEvent:
		/* Both brakes activated */
		StateMachine->state = EbsArmed;
		status = TRANSITION;
		break;

	case eTimeoutEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eBrakeTookToLong;
		break;

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;

	}

	return status;
}

State_t EbsArmed(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {
	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_ARMED_CHOICE;
			ebs_status_can_msg.ebs_status =
			CAN1_EBS_STATUS_EBS_STATUS_EBS_ARMED_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else{
			Error_Handler();
		}
		HAL_GPIO_WritePin(SYSTEM_OK_LED_GPIO_Port, SYSTEM_OK_LED_Pin,
				GPIO_PIN_SET);
		status = HANDLED;
		break;



	case eAsDrivingEvent:
		/* Release EBS */
		ReleaseEBS();
		startTimeoutCounter(500);
		status = HANDLED;
		break;

	case eTimeoutEvent:
		StateMachine->state = EbsDriving;
		status = TRANSITION;
		break;

	case ePS1_LowEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault= eLowTankPressure;
		break;

	case ePS2_LowEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault =eLowTankPressure;
		break;

	case eBP_SensorErrorEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eSensorNotOk;
		break;



	case onExit:
		stopTimeoutCounter();
		status = HANDLED;
		break;
/*
	case eASSystemNotOkEvent:
		fault = eASSystemNotOk;
		StateMachine->state = EbsFault;
		status = TRANSITION;

		break;

	case eRESNotOkEvent:
		fault = eRESNotOk;
		StateMachine->state = EbsFault;
		status = TRANSITION;

		break;
	case eVESCNotOkEvent:
		fault = eVESCNotOk;
		StateMachine->state = EbsFault;
		status = TRANSITION;

		break;
	case eDSPACENotOkEvent:
		fault = eDSPACENotOk;
		StateMachine->state = EbsFault;
		status = TRANSITION;

		break;
*/

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;

	}

	return status;
}

State_t EbsDriving(StateMachine_t *const StateMachine,
			Event_t const *const pEvent) {
	State_t status;
		switch (pEvent->eEventType) {

	case onEntry:

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)== osOK) {

		ebs_status_can_msg.ebs_state_machine =
		CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_ARMED_CHOICE;
		ebs_status_can_msg.ebs_status =
		CAN1_EBS_STATUS_EBS_STATUS_EBS_ARMED_CHOICE;
		osMutexRelease(ebs_status_can_message_mutex_Handle);
		}
		else {
			Error_Handler();
		}

		HAL_GPIO_WritePin(SYSTEM_OK_LED_GPIO_Port, SYSTEM_OK_LED_Pin,
		GPIO_PIN_SET);
		status = HANDLED;
		break;


	case ePS1_LowEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eLowTankPressure;
		break;

	case ePS2_LowEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault =eLowTankPressure;
		break;

	case eAsFinishEvent:
		StateMachine->state = EbsFinished;
		status = TRANSITION;
		break;

	case eVESCNotOkEvent:
		if (vescglobal==false){
		fault = eVESCNotOk;
		StateMachine->state = EbsFault;
		status = TRANSITION;
		}
		else{
		status=HANDLED;
		}

	case eBP_HighEvent:
		if (Global_BP_over_threshold==true)
		{
		StateMachine->state = EbsFault;
		status = TRANSITION;

		}
		else{
		status=HANDLED;
		}
		break;

	case eBP_SensorErrorEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eSensorNotOk;
		break;


	case onExit:
		status = HANDLED;
		break;
	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;
			}
	return status;
}

State_t EbsFinished(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {
	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:

		/* Activate EBS */
		ActivateEBS();

		HAL_GPIO_WritePin(CLOSE_SDC_GPIO_Port, CLOSE_SDC_Pin, GPIO_PIN_RESET);


		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_ARMED_CHOICE; // FIXME should basically say finished
			ebs_status_can_msg.ebs_status =
			CAN1_EBS_STATUS_EBS_STATUS_EBS_ACTIVATED_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else {
			Error_Handler();
		}
		startTimeoutCounter(500);
		status = HANDLED;
		break;

	case eSDC_OpenEvent:
		status = HANDLED;
		break;

	/* AS Finished but brakes are not fully engaged */
	case eTimeoutEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eBrakeTookToLong;
		break;

	//Basically will happen when ASMS is off or thats the intention w potential downsides
	case eVESCNotOkEvent:
		if( (5>=Global_EBS_pressure1_bar) && (5>=Global_EBS_pressure2_bar) && vescglobal==false) {
		StateMachine->state = EbsWaitTanks;
		status = TRANSITION;
		}
		else{
		status = HANDLED;
		}
		break;

	/* AS Finished and brakes engaged */
	case eBP_HighEvent:
		stopTimeoutCounter();
		status = HANDLED;
		break;

	case onExit:

		status = HANDLED;
		break;

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;

	}

	return status;
}

State_t EbsUnavailable(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {

	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:
		/* Release EBS */
		ReleaseEBS();

		/* Close SDC */
		HAL_GPIO_WritePin(CLOSE_SDC_GPIO_Port, CLOSE_SDC_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(SYSTEM_OK_LED_GPIO_Port, SYSTEM_OK_LED_Pin,
				GPIO_PIN_SET);

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_UNAVAILABLE_CHOICE;
			ebs_status_can_msg.ebs_status =
			CAN1_EBS_STATUS_EBS_STATUS_EBS_UNAVAILABLE_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();

		//TODO: ASSERT THERE IS NO PRESSURE IN THE TANKS!!!

		status = HANDLED;
		break;

	case onExit:
		status = HANDLED;
		break;

	case eSDC_OpenEvent:
		status = IGNORED;
		break;
//TODO: check if these should be brought back for PS1/2
	case ePS1_HighEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eHighTankPressure;
		break;

	case ePS2_HighEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eHighTankPressure;
		break;

	case eAutonomousMissionSelectedEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eMissionChangedDuringOperaton;
		break;

	default:
		status = EbsDefaultStateHandler(StateMachine, pEvent);
		break;
	}

	return status;
}

State_t EbsFault(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {

	State_t status;

	switch (pEvent->eEventType) {
	case onEntry:
		/* Activate EBS */
		ActivateEBS();

		/* Open SDC */
		HAL_GPIO_WritePin(CLOSE_SDC_GPIO_Port, CLOSE_SDC_Pin, GPIO_PIN_RESET);

		/* Indicate Fault */
		HAL_GPIO_WritePin(SYSTEM_FAULT_LED_GPIO_Port, SYSTEM_FAULT_LED_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(SYSTEM_OK_LED_GPIO_Port, SYSTEM_OK_LED_Pin,
				GPIO_PIN_RESET);

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_state_machine =
			CAN1_EBS_STATUS_EBS_STATE_MACHINE_EBS_FAULT_CHOICE;
			ebs_status_can_msg.ebs_status =
			CAN1_EBS_STATUS_EBS_STATUS_EBS_ACTIVATED_CHOICE;

			switch (fault) {
			case eLowTankPressure:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_LOW_TANK_PRESSURE_CHOICE;
				break;
			case eHighTankPressure:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_LOW_TANK_PRESSURE_CHOICE;
				break;
			case eSDCOpenUnexpectedly:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_SDC_OPEN_UNEXPECTEDLY_CHOICE;
				break;
			case eWatchdogFault:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_WATCHDOG_FAULT_CHOICE;
				break;
			case eBrakeTookToLong:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_BRAKE_TOOK_TO_LONG_CHOICE;
				break;
			case eAsEmergencyOccurred:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_AS_EMERGENCY_ERROR_CHOICE;
				break;
			case eMissionChangedDuringOperaton:
				ebs_status_can_msg.ebs_error =
						CAN1_EBS_STATUS_EBS_ERROR_MISSION_CHANGED_DURING_OPERATION_CHOICE;
				break;
			case eASSystemNotOk:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_AS_SYSTEM_NOT_OK_CHOICE;
				break;

			case eRESNotOk:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_RES_NOT_OK_CHOICE;
				break;
			case eVESCNotOk:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_VESC_NOT_OK_CHOICE;
				break;
			case eDSPACENotOk:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_DSPACE_NOT_OK_CHOICE;
				break;
			case eSensorNotOk:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_SENSOR_NOT_OK_CHOICE;
				break;
			case eBrakeUnexpectedly:
			 	 ebs_status_can_msg.ebs_error =
			 	 CAN1_EBS_STATUS_EBS_ERROR_BRAKE_UNEXPECTEDLY_CHOICE;
			 	 break;
			case eUnknownError:
			default:
				ebs_status_can_msg.ebs_error =
				CAN1_EBS_STATUS_EBS_ERROR_GENERAL_FAULT_CHOICE;
				break;
			}

			osMutexRelease(ebs_status_can_message_mutex_Handle);
		} else
			Error_Handler();
		break;
	case eResetEvent:
		/*Prepare transition from fault state (TODO: Check that all faults are gone) */
		switch (dbu_status_1_can_msg.selected_mission) {
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_NO_MISSION_SELECTED_CHOICE:
			StateMachine->state = EbsWaitMission;
			break;

		case CAN1_DBU_STATUS_1_SELECTED_MISSION_MANUAL_DRIVING_SELECTED_CHOICE:
			StateMachine->state = EbsUnavailable;
			break;

		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_ACCELERATION_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_AUTOCROSS_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_BRAKETEST_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_INSPECTION_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_SKIDPAD_SELECTED_CHOICE:
		case CAN1_DBU_STATUS_1_SELECTED_MISSION_AS_TRACKDRIVE_SELECTED_CHOICE:
			StateMachine->state = EbsWaitTanks;
			break;
		}

		status = TRANSITION;

		break;
	case eVESCNotOkEvent:
		if( (5>=Global_EBS_pressure1_bar) && (5>=Global_EBS_pressure2_bar) && vescglobal==false) {
		StateMachine->state = EbsWaitMission;
		status = TRANSITION;
		}
		else {
		status = HANDLED;
		}
		break;


	case onExit:
		/* Dont indicate Fault */
		HAL_GPIO_WritePin(SYSTEM_FAULT_LED_GPIO_Port, SYSTEM_FAULT_LED_Pin,
				GPIO_PIN_RESET);

		/* Clear fault */
		fault = eNoFault;

		if (osMutexAcquire(ebs_status_can_message_mutex_Handle, osWaitForever)
				== osOK) {
			ebs_status_can_msg.ebs_error =
			CAN1_EBS_STATUS_EBS_ERROR_NO_ERROR_CHOICE;
			osMutexRelease(ebs_status_can_message_mutex_Handle);
		}

		status = HANDLED;
		break;
	default:
		status = IGNORED;
		break;
	}
	return status;
}

State_t EbsDefaultStateHandler(StateMachine_t *const StateMachine,
		Event_t const *const pEvent) {

	State_t status;

	switch (pEvent->eEventType) {
	case eSDC_OpenEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eSDCOpenUnexpectedly;
		break;

	case eSDC_CloseEvent:
		status = HANDLED;
		break;

	case ePS1_HighEvent:
	case ePS2_HighEvent:
		status = HANDLED;
		break;

	case ePS1_LowEvent:
	case ePS2_LowEvent:
		status = HANDLED;
		break;


	case eWatchdogEvent:
		HAL_GPIO_TogglePin(WATCHDOG_GPIO_Port, WATCHDOG_Pin);
		status = HANDLED;
		break;

	case eAsEmergencyEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eAsEmergencyOccurred;
		break;

	case eNoMissionSelectedEvent:
	case eManualMissionSelectedEvent:
	case eAutonomousMissionSelectedEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eMissionChangedDuringOperaton;
		break;

	case unknownEvent:
		StateMachine->state = EbsFault;
		status = TRANSITION;
		fault = eUnknownError;
		break;
	default:
		status = IGNORED;
		break;
	}
	return status;

}

static void ActivateEBS(void)
{
	/* Activate EBS */
			HAL_GPIO_WritePin(RELEASE_BRAKE_1_GPIO_Port, RELEASE_BRAKE_1_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RELEASE_BRAKE_2_GPIO_Port, RELEASE_BRAKE_2_Pin,
					GPIO_PIN_RESET);

}
static void ReleaseEBS(void)
{
	/* Release EBS */
	HAL_GPIO_WritePin(RELEASE_BRAKE_1_GPIO_Port, RELEASE_BRAKE_1_Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELEASE_BRAKE_2_GPIO_Port, RELEASE_BRAKE_2_Pin,
			GPIO_PIN_SET);
}


static void ReleaseRearEBS(void)
{
	HAL_GPIO_WritePin(RELEASE_BRAKE_2_GPIO_Port, RELEASE_BRAKE_2_Pin,
				GPIO_PIN_SET);
}

static void ReleaseFrontEBS(void)
{
	HAL_GPIO_WritePin(RELEASE_BRAKE_1_GPIO_Port, RELEASE_BRAKE_1_Pin,
				GPIO_PIN_SET);
}

static void ActivateRearEBS(void)
{

	HAL_GPIO_WritePin(RELEASE_BRAKE_2_GPIO_Port, RELEASE_BRAKE_2_Pin,
						GPIO_PIN_RESET);
}
static void ActivateFrontEBS(void)
{
	HAL_GPIO_WritePin(RELEASE_BRAKE_1_GPIO_Port, RELEASE_BRAKE_1_Pin,
						GPIO_PIN_RESET);
}


