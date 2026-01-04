/*
 * ebs_state_machine.h
 *
 *  Created on: Nov 16, 2023
 *      Author: tore
 */

#ifndef INC_EBS_STATE_MACHINE_H_
#define INC_EBS_STATE_MACHINE_H_
/* Includes ------------------------------------------------------------------*/
#include "state_machine_common.h"
#include "../CAN/can1.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
State_t EbsDefaultStateHandler(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsInit(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsWaitMission(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsWaitTanks(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsReleaseFront(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsTestFront(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsReleaseRear(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsTestRear(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsUnavailable(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsArmed(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsDriving(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsFinished(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);
State_t EbsFault(StateMachine_t *const StateMachine,
		Event_t const *const pEvent);


/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


#endif /* INC_EBS_STATE_MACHINE_H_ */
