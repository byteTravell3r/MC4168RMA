#ifndef __USR_PID_H
#define __USR_PID_H

#include "stm32f4xx_hal.h"

enum {
	PREV2 = 0, PREV1 = 1, CURR = 2
};

typedef struct pid_t {
	double p, i, d, pout, iout, dout;
	double set[3], get[3], err[3];		//目标值 测量值 误差
	double Output, Output_Prev;
	double MaxErr, DeadBand;			//err < deadband return
	uint32_t MaxOutput, IntegralLimit;	//输出限幅 积分限幅

	void (*f_param_init)(
			struct pid_t *pid,  //PID参数初始化
			uint32_t maxOutput, uint32_t integralLimit, double p, double i,
			double d);
	void (*f_pid_reset)(struct pid_t *pid, double p, double i, double d);

} PID_TypeDef;

void PID_StructInit(PID_TypeDef *pid, uint32_t maxout, uint32_t I_Limit,
		double kp, double ki, double kd, double deadband);

double PID_Calculate(PID_TypeDef *pid, double get, double set);

extern PID_TypeDef PID_MotorSpeed[2], PID_MotorPosition[2];

#endif

