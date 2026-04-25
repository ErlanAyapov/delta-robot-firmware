#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
 extern "C" {
#endif

void Motor_StartPWM(void);
void Motor_StopPWM(void);
void Motor_UpdateCompare(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
