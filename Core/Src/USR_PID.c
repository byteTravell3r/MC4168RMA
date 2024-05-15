// 反馈/测量-get/measure/real/fdb, 期望输入一般叫set/target/ref

#include "USR_PID.h"
#include <math.h>

#define ABS(x) ( (x>0) ? (x) : (-x) )

void abs_limit(double *a, double ABS_MAX) {
	if (*a > ABS_MAX)
		*a = ABS_MAX;
	if (*a < -ABS_MAX)
		*a = -ABS_MAX;
}

static void pid_param_init(PID_TypeDef *pid, uint32_t maxout,
		uint32_t intergral_limit, double kp, double ki, double kd) {

	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
}

static void pid_reset(PID_TypeDef *pid, double kp, double ki, double kd) {
	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
}

double PID_Calculate(PID_TypeDef *pid, double get, double set) {
	pid->get[CURR] = get;
	pid->set[CURR] = set;
	pid->err[CURR] = set - get;	//set - measure
	if (pid->MaxErr != 0 && ABS(pid->err[CURR]) > pid->MaxErr)
		return 0;
	if (pid->DeadBand != 0 && ABS(pid->err[CURR]) < pid->DeadBand)
		return 0;

	pid->pout = pid->p * pid->err[CURR];
	pid->iout += pid->i * pid->err[CURR];
	pid->dout = pid->d * (pid->err[CURR] - pid->err[PREV1]);
	abs_limit(&(pid->iout), pid->IntegralLimit);
	pid->Output = pid->pout + pid->iout + pid->dout;
	abs_limit(&(pid->Output), pid->MaxOutput);
	pid->Output_Prev = pid->Output;	//update last time

	pid->err[PREV2] = pid->err[PREV1];
	pid->err[PREV1] = pid->err[CURR];
	pid->get[PREV2] = pid->get[PREV1];
	pid->get[PREV1] = pid->get[CURR];
	pid->set[PREV2] = pid->set[PREV1];
	pid->set[PREV1] = pid->set[CURR];
	return pid->Output;
}

void PID_StructInit(PID_TypeDef *pid, uint32_t maxout,
		uint32_t I_Limit, double kp, double ki, double kd, double deadband) {

	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_param_init(pid, maxout, I_Limit, kp, ki, kd);
	pid->DeadBand = deadband;
}

PID_TypeDef PID_MotorSpeed[2], PID_MotorPosition[2];
