#include "USR_RC.h"
#include <stdio.h>

uint8_t iBUS_UART_RxBuffer[30], iBUS_UART_RxHead_A, iBUS_UART_RxHead_B;
uint16_t iBUS_Channel[6] = { 0 };

void iBUS_Start_RxIT() {
	HAL_UART_Receive_IT(iBUS_UART, &iBUS_UART_RxHead_A, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == IBUS_UART_INSTANCE) {
		if (iBUS_UART_RxHead_A == 0x20) {
			HAL_UART_Receive(iBUS_UART, &iBUS_UART_RxHead_B, 1, 10);
			if (iBUS_UART_RxHead_B == 0x40) {
				HAL_UART_Receive(iBUS_UART, iBUS_UART_RxBuffer, 30, 10);
				IBUS_ResolveData(6);
			}
		}
		iBUS_Start_RxIT();
	}
}

void IBUS_ResolveData(uint8_t user_channels) {
	uint16_t iBUS_Channel_Temp[14], CheckSum_Calc, iBus_RxCheckSum;

	CheckSum_Calc = 0xFF9F;

	for (uint8_t i = 0; i < 14; i++) {
		iBUS_Channel_Temp[i] = (uint16_t) (iBUS_UART_RxBuffer[i * 2 + 1] << 8
				| iBUS_UART_RxBuffer[i * 2]);
		CheckSum_Calc = CheckSum_Calc - iBUS_UART_RxBuffer[i * 2 + 1]
				- iBUS_UART_RxBuffer[i * 2];
	}

	iBus_RxCheckSum = iBUS_UART_RxBuffer[29] << 8 | iBUS_UART_RxBuffer[28];

	if (CheckSum_Calc == iBus_RxCheckSum) {
		for (uint8_t j = 0; j < user_channels; j++) {
			iBUS_Channel[j] = iBUS_Channel_Temp[j];
		}
	}
}

