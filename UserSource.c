// 主函数文件
/*****************************引用*********************************/
#include "UserSource.h"
#include "ServeSource.h"
#include "VadcApp.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*****************************全局变量*********************************/
uint8 ctldata = 0;
uint8 ReadWord;
int CodePerid;
int i, j;
int UStop;
int arr_sum[8];
int my_angle_x, my_angle_I, my_duty_x;
int timecounter10 = 0;
float distance;
double myangleOr, anglex, angle_mag, length_err;

/*****************************底层控制*********************************/
//电机速度
void motor_duty(int duty) 
{

    if (duty > 0)
    {
        SetMotor(FORWARD, duty);
    }
    else
    {
        SetMotor(BACKWARD, -duty);
    }
}

// 舵机角度
void steer_angle(int duty) 
{
    duty += 12;
    if (duty > 0) 
    {
        SetSteer(LEFT, duty);
    } 
    else if (duty < 0) 
    {
        SetSteer(RIGHT, -duty);
    } 
    else
    {
        SetSteer(MIDDLE, duty);
    }
}

// PID拟合曲线
double PIDsim(int mag) 
{
    angle_mag = 20 * (-mag / (267.6)) / 784;
    anglex = asin(angle_mag);
    return anglex;
}

/*****************************滤波算法*********************************/
//对于电磁传感器的误差可以采取的滤波算法
int *avg_filter(void) // 5次测量取平均值
{
    // Read multiple set of data to get average values
    int arr_data[8] = {0, 0, 0, 0, 0, 0, 0, 0}, i, *p;
    for (i = 0; i < 8; i++) 
    {
        arr_sum[i] = 0;
    }
    for (i = 0; i < 12; i++) 
    {
        VADCresult_run();
        for (j = 0; j < 8; j++) 
        {
            arr_sum[j] += VADCresult[j + 1];
        }
    }
    for (i = 0; i < 8; i++) {
        arr_sum[i] = arr_sum[i] / 12;
    }
    return arr_sum;
}

/*****************************电机驱动*********************************/
// 行驶函数
void run(void) 
{
    avg_filter();
    // 正常行驶
    if (UStop) 
    {
        myangleOr = PIDsim(arr_sum[3] - arr_sum[2]) * (450);
        myangleI = (int) myangleOr;
        Bluetooth_Send_Data(myangleI);
        mydutyx = 45 - (myangleI / 3);
        motor_duty(-mydutyx);
        steer_angle(myangleI);
        for (i = 0; i < 3; i++) 
        {
            UserInterupt10ms(); //单次执行时间为30ms
        }
    } 
    else
        motor_duty(0);
}

// 入大圆弯道
int circle_count = 0; // global
if ()

/*****************************主函数***********************************/
//CPU0主函数，置于循环中用户主要逻辑计算区
void UserCpu0Main(void) 
{
    VADC_init();
    while (1) 
    {
        ReadWord = Bluetooth_Read_Data();
        if (ReadWord != 0)
        {
            ctldata = ReadWord;
        }
        if (ctldata == 'O')
        {
            UStop = 1;
        }
        else
        {
            UStop = 0;
        }
        run();
    }
}

//CPU1主函数，置于循环中，摄像头读写由此核处理，建议用于摄像头相关计算：
//不要写成死循环，后面有AD相关处理
void UserCpu1Main(void) 
{

}
/**************************************中断调用函数****************************************/
//该函数每10ms执行一次，请在该函数中书写程序，中断时间有限，不要太长
uint32 UserInterupt10ms(void) 
{
    return 0;
}

//该函数每100ms执行一次，请在该函数中书写程序，中断时间有限，不要太长
//样例，获取编码器输出频率与超声举例
uint32 UserInterupt100ms(void)
{
    distance = get_echo_length();
    CodePerid = GetCodePerid();
    return 0;
}

//该函数每1000ms执行一次，请在该函数中书写程序，中断时间有限，不要太长
uint32 UserInterupt1000ms(void) 
{
    return 0;
}

void UserInteruptIO(void) 
{
    IfxPort_togglePin(LED1);
}
