#ifndef __FOURMOTOR_COORDINATION_H__
#define __FOURMOTOR_COORDINATION_H__

#include "main.h"
#include "DM_BLDC_Motor.h"

/*前进*/
void FMC_FORWARD(float vel, float dist);
/*后退*/
void FMC_BACKWARD(float vel, float dist);
/*左横移*/
void FMC_STRAFE_LEFT(float vel, float dist);
/*右横移*/
void FMC_STRAFE_RIGHT(float vel, float dist);
/*左转掉头*/
void FMC_TURN_LEFT_IN_PLACE(float vel, float dist);
/*右转掉头*/
void FMC_TURN_RIGHT_IN_PLACE(float vel, float dist);
/*左前斜*/
void FMC_FORWARD_LEFT(float vel, float dist);
/*右前斜*/
void FMC_FORWARD_RIGHT(float vel, float dist);
/*左后斜*/
void FMC_BACKWARD_LEFT(float vel, float dist);
/*右后斜*/
void FMC_BACKWARD_RIGHT(float vel, float dist);
/*绕前轴中心左转*/
void FMC_PIVOT_FRONT_LEFT(float vel, float dist);
/*绕前轴中心右转*/
void FMC_PIVOT_FRONT_RIGHT(float vel, float dist);
/*绕后轴中心左转*/
void FMC_PIVOT_REAR_LEFT(float vel, float dist);
/*绕后轴中心右转*/
void FMC_PIVOT_REAR_RIGHT(float vel, float dist);

#endif

