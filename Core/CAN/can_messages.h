/*
 * can_messages.h
 *
 *  Created on: Mar 14, 2024
 *      Author: tore
 */

#ifndef CAN_CAN_MESSAGES_H_
#define CAN_CAN_MESSAGES_H_

#include "can1.h"
#include "stm32f4xx_hal.h"

/* RX CAN Messagse */
extern struct can1_dbu_status_1_t dbu_status_1_can_msg;
extern struct can1_ams_status_1_t ams_status_can_msg;
extern struct can1_ecu_status_t ecu_status_can_msg;
extern struct can1_dv_control_target_tv_t as_system_can_msg;
extern struct can1_vesc_status_t vesc_status_can_msg;
extern struct can1_dv_system_status_t dv_system_status_can_msg;
extern struct can1_res_status_t res_status_can_msg;
extern struct can1_vehicle_status_t vehicle_status_can_msg;


/* TX CAN Messages */
extern struct can1_ebs_status_t ebs_status_can_msg;
extern struct can1_fb_pedals_t pedals_can_msg;
extern struct can1_steering_t steering_can_msg;

extern const CAN_TxHeaderTypeDef pedals_can_msg_header;
extern const CAN_TxHeaderTypeDef steering_can_msg_header;
extern const CAN_TxHeaderTypeDef ebc_status_can_msg_header;

//Timeout values
extern int32_t last_dspace_time_ms;
extern int32_t last_vesc_time_ms;
extern int32_t last_res_time_ms;
extern int32_t last_as_time_ms;
extern uint8_t global_dbu_mission;


void handle_new_dbu_msg(void);
void handle_new_ams_msg(void);
void handle_new_ecu_msg(void);
void handle_new_res_msg(void);
void handle_new_as_system_msg(void);
void handle_new_dspace_msg(void);
void handle_new_vesc_msg(void);
void handle_new_dv_system_status_msg(void);


#endif /* CAN_CAN_MESSAGES_H_ */
