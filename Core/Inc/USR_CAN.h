#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif
#include "USR_TYPE.h"

/*CAN发送或是接收的ID*/
typedef enum {

	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,
	CAN_DriverPower_ID = 0x80,

	CAN_HeartBeat_ID = 0x156,

} CAN_Message_ID;

#define FILTER_BUF_LEN		5
/*接收到的云台电机的参数结构体*/
typedef struct {
	int16_t speed_rpm;
	int16_t real_current;
	int16_t given_current;
	uint8_t hall;
	uint16_t angle;				//abs angle range:[0,8191]
	uint16_t last_angle;	//abs angle range:[0,8191]
	uint16_t offset_angle;
	int32_t round_cnt;
	int32_t total_angle;
	u8 buf_idx;
	u16 angle_buf[FILTER_BUF_LEN];
	u16 fited_angle;
	u32 msg_cnt;
} moto_measure_t;

/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_yaw, moto_pit, moto_poke, moto_info;
extern float real_current_from_judgesys; //unit :mA
extern float dynamic_limit_current;	//unit :mA,;	//from judge_sys
extern float ZGyroModuleAngle, yaw_zgyro_angle;

void my_can_filter_init(CAN_HandleTypeDef *hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan);
void can_filter_recv_special(CAN_HandleTypeDef *hcan, uint8_t filter_number,
		uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef *hcan);
void can_receive_onetime(CAN_HandleTypeDef *_hcan, int time);
void set_moto_current(CAN_HandleTypeDef *hcan, s16 iq1, s16 iq2, s16 iq3,
		s16 iq4);
#endif
