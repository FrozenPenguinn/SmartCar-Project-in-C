/*
 * VadcApp.h
 *
 *  Created on: 2017��12��8��
 *      Author: Administrator
 */

#ifndef VADCAPP_H_
#define VADCAPP_H_
#include "Vadc/Adc/IfxVadc_Adc.h"
#include "IfxVadc_regdef.h"
void VADC_Init(void);
extern void VADC_Result_Run(void);//��ȡAD���
extern unsigned int  VADC_result[9];//AD����洢λ�� 0.0---0.8 AN0....AN8
#endif /* 0_SRC_0_APPSW_TRICORE_API_VADC_VADCAPP_H_ */
