// 主函数文件
/*****************************引用*********************************/
#include "UserSource.h"
#include "ServeSource.h"
#include "VadcApp.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*****************************全局变量*********************************/
uint8 bt_command = 0;
uint8 read_word = 0; 
int code_period = 0;
int i, j, k; // 计数变量
int go_stop = 0;
int filtered_arr[8] = {0,0,0,0,0,0,0,0};
int my_angel_int = 0, my_duty = 0, motor_rotation = 0;
float distance = 0.0; //not used yet, for ultrasonic sensor
double my_angle_double = 0.0, PID_angle_mag_strength_ratio = 0.0, PID_mag_angle_sin = 0.0;
int circle_entry = 0;
int out_mag_count = 0;
int in_out_status= 0;

/*****************************底层控制*********************************/
//电机速度
void Motor_Duty(int duty)
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
void Steer_Angle(int duty) 
{
    duty -= 12; // 对于初始舵机误差的修正
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
double PID_Sim(int mag)
{
    PID_mag_angle_sin = 20 * (-mag / (267.6)) / 784;
    PID_angle_mag_strength_ratio = asin(PID_mag_angle_sin);
    return PID_angle_mag_strength_ratio;
}

// 编码器速度控制
int Speed_Control(int speed_want)
{
    int motor_adjusted;
    motor_adjusted = (speed_want - code_period) / 1000;
    return motor_adjusted;
}

/*****************************滤波算法*********************************/
//对于电磁传感器的误差可以采取的滤波算法
int *Avg_Filter(void)
{
    // 12次测量取平均值
    for (i = 0; i < 12; i++)  
    {
        VADC_Result_Run();
        for (j = 0; j < 8; j++)
        {
            filtered_arr[j] += VADC_result[j + 1];
        }
    }
    for (i = 0; i < 8; i++) 
    {
        filtered_arr[i] = filtered_arr[i] / 12;
    }
    return filtered_arr; // 数据位为0-5
}

/*****************************电机驱动*********************************/
// 行驶函数
void Run(void)
{
    Avg_Filter();
    // 正常行驶
    if (go_stop)
    {
        if(in_out_status == 1)
        {
            my_angel_int = -110;
        }
        else if(in_out_status == 2)
        {
            my_angle_double = PID_Sim(filtered_arr[3] - filtered_arr[2]) * (450);
            my_angel_int = (int) my_angle_double;
        }
        motor_rotation = 6000 - (abs(my_angel_int) * 20);
        my_duty += Speed_Control(motor_rotation);
        if(my_duty > 60) 
        {
            my_duty = 60;
        }
        else if(my_duty < 25) 
        {
            my_duty = 25;
        }
        Motor_Duty(-my_duty);
        Steer_Angle(my_angel_int);

        // 入大圆弯道
        if ((filtered_arr[4] > filtered_arr[3] + 500) && (filtered_arr[4] > filtered_arr[1] + 2000))
        {
            circle_entry ++;
            Bluetooth_Send_Data(circle_entry);
            if (circle_entry == 2)
            {
                my_angel_int = -60;
                Steer_Angle(my_angel_int);
                UserInterupt100ms();

            }
            if (circle_entry == 4)
                circle_entry = 0;
            UserInterupt100ms();
        }
    }
    else
    {
        Motor_Duty(0);
        Steer_Angle(0);
    }
}

/*****************************主函数***********************************/
//CPU0主函数，置于循环中用户主要逻辑计算区
void UserCpu0Main(void)
{
    VADC_Init();
    go_stop = 0;
    while (1)
    {
        // 蓝牙起停指令
        read_word = Bluetooth_Read_Data();
        if (read_word != 0)
        {
            bt_command = read_word;
            Bluetooth_Send_Data(bt_command);
        }
        if (bt_command == 'O')
        {
            if (go_stop == 0)
            {
                delay_ms(2000);
                go_stop = 1;
                out_mag_count = 0;
            }
        }
        else
        {
            k = 0;
            go_stop = 0;
        }
        Run();
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
    code_period = GetCodePerid();
    return 0;
}

//该函数每1000ms执行一次，请在该函数中书写程序，中断时间有限，不要太长
uint32 UserInterupt1000ms(void)
{
    return 0;
}

void UserInteruptIO(void)
{
    out_mag_count ++;
    if(out_mag_count == 1)
    {
        motor_rotation = -4000;
        my_duty += Speed_Control(motor_rotation);
        if(my_duty < -60) 
        {
            my_duty = -60;
        }
        else if(my_duty > -25) 
        {
            my_duty = -25;
        }
        Motor_Duty(-my_duty);
        delay_ms(500);
        in_out_status = 1;
        Run();
        delay_ms(500);
        in_out_status = 2;
        out_mag_count = 2 ;
    }
}
