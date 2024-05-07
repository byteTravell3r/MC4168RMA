#ifndef __USR_PID_H
#define __USR_PID_H

#include "stm32f4xx_hal.h"

enum {
	PREV2 = 0, PREV1 = 1, CURR = 2
};

typedef struct pid_t {
	float p, i, d, pout, iout, dout;
	float set[3], get[3], err[3];		//目标值 测量值 误差
	float Output, Output_Prev;
	float MaxErr, DeadBand;			//err < deadband return
	uint32_t MaxOutput, IntegralLimit;	//输出限幅 积分限幅

	void (*f_param_init)(
			struct pid_t *pid,  //PID参数初始化
			uint32_t maxOutput, uint32_t integralLimit, float p, float i,
			float d);
	void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);

} PID_TypeDef;

void PID_StructInit(PID_TypeDef *pid, uint32_t maxout, uint32_t I_Limit,
		float kp, float ki, float kd, float deadband);

float PID_Calculate(PID_TypeDef *pid, float get, float set);

extern PID_TypeDef PID_MotorSpeed[2], PID_MotorPosition[2];

#endif

