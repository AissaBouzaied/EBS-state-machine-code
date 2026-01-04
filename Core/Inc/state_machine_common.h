/*
 * state_machine.h
 *
 *  Created on: Oct 9, 2023
 *      Author: tore
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "main.h"
#include <stdbool.h>

//TODO: Add ePrimaryTankPressureLow, ePrimaryTankHighPressuerHigh, ePrimaryTankPressureOk etc.
typedef enum {
	onExit = -3,
	onEntry = -2,
	unknownEvent = -1,
	eSDC_OpenEvent = 0,
	eSDC_CloseEvent,
	eTS_OnEvent,
	eTS_OffEvent,
	ePS1_HighEvent,
	ePS1_LowEvent,
	ePS2_HighEvent,
	ePS2_LowEvent,
	eBP_FrontHighEvent,
	eBP_RearHighEvent,
	eBP_FrontLowEvent,
	eBP_RearLowEvent,
	eBP_LowEvent,
	eBP_HighEvent,
	eTimeoutEvent,
	eAsEmergencyEvent,
	eAsDrivingEvent,
	eAsFinishEvent,
	eRESEvent,
	eNoMissionSelectedEvent,
	eManualMissionSelectedEvent,
	eAutonomousMissionSelectedEvent,
	eResetEvent,
	eWatchdogEvent,
	eASSystemNotOkEvent,
	eRESNotOkEvent,
	eVESCNotOkEvent,
	eDSPACENotOkEvent,
	eAllCANNodesOk,
	eBP_SensorErrorEvent,
	eBP_DiscrepancyEvent,
	reservedEvent = 0x7FFFFFFF ///< Prevents enum down-size compiler optimization.
} eEvent_t;

typedef struct {
	eEvent_t eEventType;
	//EventData_u data;
} Event_t;


typedef struct StateMachine_t StateMachine_t;

typedef enum {
	TRANSITION, HANDLED, IGNORED, INIT
} State_t;

typedef State_t (*StateHandler)(StateMachine_t *const StateMachine,
		Event_t const *const EventData);

struct StateMachine_t {
	StateHandler state; // State variable
	bool BP_High;
};

typedef enum{
	eNoFault = 0,
	eLowTankPressure = 1,
	eHighTankPressure,
	eSDCOpenUnexpectedly,
	eWatchdogFault,
	eBrakeTookToLong,
	eAsEmergencyOccurred,
	eMissionChangedDuringOperaton,
	eASSystemNotOk,
	eRESNotOk,
	eVESCNotOk,
	eDSPACENotOk,
	eUnknownError,
	eSensorNotOk,
	eBrakePressureDiscrepancyError,
	eBrakeUnexpectedly,
}eFault_t;

void updateTankStatus(uint32_t data, uint8_t * pTankStatus);
#endif /* INC_STATE_MACHINE_H_ */
