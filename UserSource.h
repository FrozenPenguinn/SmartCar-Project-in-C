/*
 * UserSource.h
 *
 *  Created on: 2019/1/7
 *      Author: ZHANGZH
 */

#ifndef _USERSOURCE_H_
#define _USERSOURCE_H_

#include "ServeSource.h"
IFX_EXTERN void Motor_Duty(int duty);
IFX_EXTERN void Steer_Angle(int duty);
IFX_EXTERN double PID_Sim(int mag);
IFX_EXTERN int Speed_Control(int speed_want);
IFX_EXTERN int* Avg_Filter(void);
IFX_EXTERN void Run(void);
IFX_EXTERN void UserCpu0Main(void);
IFX_EXTERN void UserCpu1Main(void);
IFX_EXTERN uint32 UserInterupt10ms(void);
IFX_EXTERN uint32 UserInterupt100ms(void);
IFX_EXTERN uint32 UserInterupt1000ms(void);
IFX_EXTERN void UserInteruptIO(void);


#endif /* 0_SRC_APPSW_TRICORE_MAIN_USERSOURCE_H_ */
