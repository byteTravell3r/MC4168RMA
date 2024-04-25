#include <USR_CAN.h>
#include <USR_RC.h>
#include <stdio.h>
#include "main.h"

extern CAN_HandleTypeDef hcan1;

MotorDataTypeDef Motor[2] = { 0 };

static uint32_t TxMailbox = CAN_TX_MAILBOX0;
static CAN_TxHeaderTypeDef TxHeader;
static uint8_t TxData[8];

void CAN1_START_IRQ() {
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL) != HAL_OK) {
		Error_Handler();
	}

	TxHeader.StdId = ADDR_3508_SET_ID0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
	HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);
	static uint8_t RxData[8];
	static CAN_RxHeaderTypeDef RxHeader;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	uint32_t ID_NUM;
	ID_NUM = RxHeader.StdId - ADDR_3508_GET_ID1;
	if (ID_NUM == MotorX || ID_NUM == MotorY) {
		if (Motor[ID_NUM].MSG_COUNT++ <= 50) {
			Motor_GetInitPosition(&Motor[ID_NUM], RxData);
		} else
			Motor_ResolveFeedbackData(&Motor[ID_NUM], RxData);
	}
	HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);
}

void Motor_ResolveFeedbackData(MotorDataTypeDef *MOTOR_STR, uint8_t DATA[]) {
	MOTOR_STR->LAST_ANGLE = MOTOR_STR->CURR_ANGLE;
	MOTOR_STR->CURR_ANGLE = (uint16_t) (DATA[0] << 8 | DATA[1]);
	MOTOR_STR->RPM = (int16_t) (DATA[2] << 8 | DATA[3]);
	MOTOR_STR->I_REAL = (int16_t) (DATA[4] << 8 | DATA[5]) / -5;
	MOTOR_STR->TEMPERATURE = DATA[6];

	if (MOTOR_STR->CURR_ANGLE - MOTOR_STR->LAST_ANGLE > 4096)
		MOTOR_STR->ROUNDS--;
	else if (MOTOR_STR->CURR_ANGLE - MOTOR_STR->LAST_ANGLE < -4096)
		MOTOR_STR->ROUNDS++;

	MOTOR_STR->TOTAL_ANGLE = MOTOR_STR->ROUNDS * 8192 + MOTOR_STR->CURR_ANGLE
			- MOTOR_STR->OFFSET_ANGLE;
}

void Motor_GetInitPosition(MotorDataTypeDef *MOTOR_STR, uint8_t DATA[]) {
	MOTOR_STR->CURR_ANGLE = (uint16_t) (DATA[0] << 8 | DATA[1]);
	MOTOR_STR->OFFSET_ANGLE = MOTOR_STR->CURR_ANGLE;
}

void Motor_SendCmd(int16_t I1, int16_t I2) {
	TxData[0] = I1 >> 8;
	TxData[1] = I1;
	TxData[2] = I2 >> 8;
	TxData[3] = I2;
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
//	HAL_UART_Receive_IT(IBUS_UART, rx_buffer, 32);
}
