/*
 * VadcApp.c
 *
 *  Created on: 2017��12��8��
 *      Author: Administrator
 */

#include "VadcApp.h"
// VADC handle
IfxVadc_Adc vadc;
IfxVadc_Adc_Group adcGroup;
IfxVadc_Adc_Config adcConfig;
IfxVadc_Adc_GroupConfig adcGroupConfig;
// create channel config
IfxVadc_Adc_ChannelConfig adcChannelConfig[9];
IfxVadc_Adc_Channel adcChannel[9];
unsigned int  VADC_result[9];

/*****************************电磁采集*********************************/
//磁场数据采样

void VADC_Init(void)
{
	// create configuration
	IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);
	// initialize module
	// IfxVadc_Adc vadc; // declared globally
	IfxVadc_Adc_initModule(&vadc, &adcConfig);

	// create group config
	IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &vadc);

	adcGroupConfig.groupId = IfxVadc_GroupId_0;
	// IMPORTANT: usually we use the same group as master!
	adcGroupConfig.master = adcGroupConfig.groupId;
	// enable all arbiter request sources
	adcGroupConfig.arbiter.requestSlotQueueEnabled          = TRUE; // enable Queue mode
	adcGroupConfig.arbiter.requestSlotScanEnabled           = TRUE; // enable Scan mode
	adcGroupConfig.arbiter.requestSlotBackgroundScanEnabled = TRUE; // enable Background scan
	// enable all gates in "always" mode (no edge detection)
	adcGroupConfig.queueRequest.triggerConfig.gatingMode          = IfxVadc_GatingMode_always;
	adcGroupConfig.scanRequest.triggerConfig.gatingMode           = IfxVadc_GatingMode_always;
	adcGroupConfig.backgroundScanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
	// enable auto scan
	adcGroupConfig.scanRequest.autoscanEnabled = TRUE;
	adcGroupConfig.backgroundScanRequest.autoBackgroundScanEnabled = TRUE;
	// initialize the group
	IfxVadc_Adc_initGroup(&adcGroup, &adcGroupConfig);

	int chnIx;
    for(chnIx=0; chnIx<9; chnIx++) {
        IfxVadc_Adc_initChannelConfig(&adcChannelConfig[chnIx], &adcGroup);
        adcChannelConfig[chnIx].channelId = (IfxVadc_ChannelId)(chnIx);
        adcChannelConfig[chnIx].resultRegister = (IfxVadc_ChannelResult)(chnIx); // use dedicated result register
        // initialize the channela
        IfxVadc_Adc_initChannel(&adcChannel[chnIx], &adcChannelConfig[chnIx]);
        // add to scan
        unsigned channels = (1 << adcChannelConfig[chnIx].channelId);
        unsigned mask = channels;
        IfxVadc_Adc_setScan(&adcGroup, channels, mask);
    }
}


void VADC_Result_Run(void)
{
    // wait for valid result
    Ifx_VADC_RES conversionResult;
    // start scan
    IfxVadc_Adc_startScan(&adcGroup);
    // check results
    int chnIx;
    for(chnIx = 0; chnIx < 9; chnIx++) {
        do {
            conversionResult = IfxVadc_Adc_getResult(&adcChannel[chnIx]);
        } while( !conversionResult.B.VF );
        VADC_result[chnIx] = conversionResult.B.RESULT;
    }
}








