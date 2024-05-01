/*
 * USR_RC.h
 *
 *  Created on: Apr 24, 2024
 *      Author: byte
 */

#ifndef INC_USR_RC_H_
#define INC_USR_RC_H_

#include "main.h"
extern UART_HandleTypeDef huart7;

#define iBUS_UART				(&huart7)
#define IBUS_UART_INSTANCE		(UART7)
#define iBUS_CHANNEL_COUNT		10		// Use 6 channels

void iBUS_Start_RxIT();
void IBUS_ResolveData(uint8_t user_channels);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern uint16_t iBUS_Channel[];

#endif /* INC_USR_RC_H_ */
