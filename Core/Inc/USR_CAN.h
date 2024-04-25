#ifndef __USR_CAN
#define __USR_CAN

#include "stm32f4xx_hal.h"

typedef enum {
	ADDR_3508_SET_ID0 = 0x200,
	ADDR_3508_GET_ID1 = 0x201,
	ADDR_3508_GET_ID2 = 0x202,
	ADDR_3508_GET_ID3 = 0x203,
	ADDR_3508_GET_ID4 = 0x204,
	MotorX = 0,
	MotorY = 1,
} CAN_Message_ID;

typedef struct {
	int16_t RPM;
	int16_t I_REAL, I_GIVEN;
	uint16_t CURR_ANGLE, LAST_ANGLE, OFFSET_ANGLE;; //Angle Range: [0, 8191]
	uint8_t TEMPERATURE;
	int32_t ROUNDS, TOTAL_ANGLE;
	uint32_t MSG_COUNT;
} MotorDataTypeDef;

extern MotorDataTypeDef Motor[];

void CAN1_START_IRQ();
void Motor_ResolveFeedbackData(MotorDataTypeDef *ptr, uint8_t data[]);
void Motor_GetInitPosition(MotorDataTypeDef *ptr, uint8_t data[]);
void Motor_SendCmd(int16_t I1, int16_t I2);

#endif
