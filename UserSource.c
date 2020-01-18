// 主函数文件
/*****************************引用*********************************/
#include "UserSource.h"
#include "ServeSource.h"
#include "VadcApp.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*****************************全局变量*********************************/
char bt_command = 0; // 蓝牙指令
int code_period = 0; // 编码器转速
int i, j, k; // 计数变量
int go_stop = 0; // 车辆起停 - 蓝牙控制
int filtered_arr[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 滤波处理后数据
double my_angle_double = 0.0; // 行驶角度
int my_angle_int = 0; // 行驶角度
int my_duty = 0; // 电机功率
int my_speed = 0; // 电机转速
int top_power = 0; // 预设最大功率
int low_power = 0; // 预设最小功率
float distance = 0.0; // 超声探测距离
int PID_angle_mag_strength_ratio = 0.0; // PID电磁循线调整比例
int PID_mag_angle_sin = 0.0;
int circle_entry = 0; // 环路进出判断，0为无环路磁场，1为探测到一次但无视，2为再次测到转向，3为环岛内行驶，4为驶出并忽视之后的环岛磁场
int in_out_status = 0; // 是否车库判断，0在车库中，1不在车库中，2已返回车库

int zebra_strip_black_pixel = 0; // 斑马线上黑色条纹数
int out_turn = 0; // 0为出库转弯前，1为出库转弯中，2为出库转弯后
int in_turn = 0; // 0为入库转弯前，1为入库转弯中，2为入库转弯后
int ver_zebra_strip_white_pixel = 0; // 纵向斑马线白色像素数
int black_strip_status = 0; // 灰度
int num_black_strip = num_white_strip = 0;
int strip_to_black_counter = strip_to_white_counter = 0; // 白到黑，黑到白

int entry_delay_status = 0; // boolean 环岛入弯检测 可更改为1000ms
int entry_delay_time = 0; // int 环岛入弯检测 测到电磁线的数量
int crossroad_left_pixel = 0;
int crossroad_left_status = 0;
int crossroad_right_pixel = 0; // turn xr = pixel width
int crossroad_right_status = 0; // turn xr time = status

// in_turn 入库转向
// out turn 出库转向

/*****************************底层控制*********************************/
//电机功率
void Motor_Duty(int duty) {
    if (duty > 0) {
        SetMotor(FORWARD, duty);
    } else {
        SetMotor(BACKWARD, -duty);
    }
}

// 舵机角度
void Steer_Angle(int duty) {
    duty -= 12; // 对于初始舵机误差的修正
    if (duty > 0) {
        SetSteer(LEFT, duty);
    } else if (duty < 0) {
        SetSteer(RIGHT, -duty);
    } else {
        SetSteer(MIDDLE, duty);
    }
}

// PID拟合曲线
double PID_Sim(int mag) {
    PID_mag_angle_sin = 20 * (-mag / (267.6)) / 784;
    PID_angle_mag_strength_ratio = asin(PID_mag_angle_sin);
    return PID_angle_mag_strength_ratio;
}

// 编码器速度控制
int Speed_Control(int speed_want) {
    int motor_adjusted = 0;
    motor_adjusted = (speed_want - code_period) / 500;
    my_duty += motor_adjusted;
    if (my_duty > top_power) {
        my_duty = top_power;
    } else if (my_duty < low_power) {
        my_duty = low_power;
    }
    if (go_stop) {
        Motor_Duty(-my_duty);
    } else {
        Motor_Duty(0);
    }
    return 0;
}

/*****************************滤波算法*********************************/
//对于电磁传感器的随机误差采取的滤波
int *Avg_Mag_Filter(void) {
    // 12次测量取平均值
    for (i = 0; i < 12; i++) {
        VADC_Result_Run();
        for (j = 0; j < 8; j++) {
            filtered_arr[j] += VADC_result[j + 1];
        }
    }
    for (i = 0; i < 8; i++) {
        filtered_arr[i] = filtered_arr[i] / 12;
    }
    return filtered_arr; // 数据位为0-5
}

// 超声波距离探测数据滤波
void Avg_Sonic_Filter(void) {
    // 8次测量取平均值
    int ultrasonic_filter = 0;
    for (i = 0; i < 8; i++) {
        distance = get_echo_length();
        ultrasonic_filter += distance;
    }
    distance = ultrasonic_filter / 8;
}

/*****************************电机驱动*********************************/
// 避障函数
void Around_Obstacle(void) {
    if (distance < 700) {
        my_angle_int = -60; // 右转避开障碍物
        Steer_Angle(my_angle_int);
        delay_ms(2000);
        my_angle_int = 100; // 左转返回赛道
        Steer_Angle(my_angle_int);
        delay_ms(1000);
        for (k = 0; k < 100; k++) { // 电磁线捕获（返回赛道）
            delay_ms(10);
            my_angle_double = PID_Sim(filtered_arr[3] - filtered_arr[2]) * 450;
            my_angle_int = (int) my_angle_double;
            if (abs(my_angle_int) >= 110 && k > 10) {
                break;
            } else {
                my_angle_int = -80;
            }
            Steer_Angle(my_angle_int);
        }
    }
}

// 行驶函数
void Run(void) {
    // 正常行驶
    if (go_stop == 1) {
        if (in_out_status == 0) { // 在车库内
            circle_entry = 0;
        }
        if (out_turn == 1) { // 出库转弯
            my_angle_int = -90;
            my_speed = 3000;
            top_power = 100;
            low_power = 30;
        } else if (in_out_status == 0) { // 出库启动
            Motor_Duty(-100);
            my_angle_int = 0;
            my_speed = 500;
            top_power = 100;
            low_power = 20;
        } else { // 常规行驶时捕捉电磁线
            my_angle_double = PID_Sim(filtered_arr[3] - filtered_arr[2]) * (450);
            my_angle_int = (int) my_angle_double;
            my_speed = 5000 - (abs(my_angle_int) * 10);
            top_power = 60;
            low_power = 10;
            if (entry_delay_status == 1) { // 捕捉到环岛电磁线后匀速巡航
                my_speed = 2000;
                top_power = 100;
                low_power = 0;
            }
        }
        if (in_turn == 1) { // 入库转弯
            my_angle_int = -110;
            my_speed = 4000;
            top_power = 100;
            low_power = 30;
        }
        if (crossroad_left_status == 2) { // 左菱形转弯
            my_speed = 2000;
            top_power = 100;
            low_power = 10;
        } else if (crossroad_right_status == 2) { // 右菱形转弯
            my_speed = 2000;
            top_power = 100;
            low_power = 10;
        }

        // 环岛行驶
        if (filtered_arr[5] > 2500 && filtered_arr[0] < 1500 && (filtered_arr[5] - filtered_arr[0] > 700) && (abs(my_angle_int <= 70)) && in_out_status == 1) { // 右环岛
            if (abs(my_angle_int) <= 70 && circle_entry == 0)
            {
                circle_entry++;
            }
            entry_delay_status = 1; // 第一次测到环岛电磁后停止此函数900ms，减少运算量
            entry_delay_time = 0;
            if (circle_entry == 2) { // 入环岛
                my_speed = 2000;
                top_power = 100;
                low_power = 30;
                my_angle_int = -80;

            }
            if (circle_entry == 5) { // 重置环岛计数，可应用于多环岛线路
                circle_entry = 0;
            }
        } else if (filtered_arr[0] > 2500 && filtered_arr[5] < 1500 && (filtered_arr[0] - filtered_arr[5] > 700) &&
                   in_out_status == 1 && (abs(my_angle_int) <= 70)) { // 左环岛
            if (abs(my_angle_int) <= 70 && circle_entry == 0)
            {
                circle_entry++;
            }
            entry_delay_status = 1; // 第一次测到环岛电磁后停止此函数900ms，减少运算量
            entry_delay_time = 0;
            if (circle_entry == 2) { // 入环岛
                my_speed = 2000;
                top_power = 100;
                low_power = 20;
                my_angle_int = 80;
            }
            if (circle_entry == 5) { // 重置环岛计数，可应用于多环岛线路
                circle_entry = 0;
            }
        }
        Steer_Angle(my_angle_int);

        if (out_turn == 1) { // 出库成功，尝试捕获电磁线
            for (k = 0; k < 130; k++) {
                my_angle_double = PID_Sim(filtered_arr[3] - filtered_arr[2]) * 500;
                my_angle_int = (int) my_angle_double;
                if (abs(my_angle_int) >= 80 && k >= 20) {
                    break;
                } else {
                    my_angle_int = -110;
                }
                Steer_Angle(my_angle_int);
                delay_ms(10);
            }
            out_turn = 2; // 完成出库
            in_out_status = 1; // 车辆标注为不在车库中
            circle_entry = 0; // 环岛状态清零
        }
        if (in_turn == 1) { // 入库转弯
            delay_ms(700);
            in_out_status = 2; // 车辆标注为返回车库中
            circle_entry = 0; // 环岛状态清零
        }
        if (circle_entry == 2 || circle_entry == 4) { // 2是捕获环路电磁线，4时捕捉赛道电磁线
            for (k = 0; k <= 100; k++) {
                delay_ms(10);
                my_angle_double = PID_Sim(filtered_arr[3] - filtered_arr[2]) * 500;
                my_angle_int = (int) my_angle_double;
                if (abs(my_angle_int) >= 60 && k > 10) {
                    break;
                } else {
                    my_angle_int = 80;
                }
                Steer_Angle(my_angle_int);
            }
            circle_entry++;
        }
        if (crossroad_left_status == 2) { // 菱形左转后捕获电磁线
            for (k = 0; k < 100; k++) {
                delay_ms(10);
                my_angle_double = PID_Sim(filtered_arr[3] - filtered_arr[2]) * 450;
                my_angle_int = (int) my_angle_double;
                if (abs(my_angle_int) >= 60 && k > 5) {
                    break;
                } else {
                    my_angle_int = 70;
                }
                Steer_Angle(my_angle_int);
            }
            crossroad_left_status = 0;
            circle_entry = 0;
        } else if (crossroad_right_status == 2) { // 菱形右转后捕获电磁线
            for (k = 0; k < 100; k++) {
                delay_ms(10);
                my_angle_double = PID_Sim(filtered_arr[3] - filtered_arr[2]) * 450;
                my_angle_int = (int) my_angle_double;
                if (abs(my_angle_int) >= 110 && k > 5) {
                    break;
                } else {
                    my_angle_int = -70;
                }
                Steer_Angle(my_angle_int);
            }
            crossroad_right_status = 0;
            circle_entry = 0
        }

        // 避障
        Around_Obstacle();

    } else { // 蓝牙指令为否定时，停止活动
        my_speed = 0;
        Steer_Angle(0);
        in_out_status = 0;
    }
}


/*****************************主函数***********************************/
//CPU0主函数，置于循环中用户主要逻辑计算区
void UserCpu0Main(void) {
    VADC_Init(); // 初始化电磁数据储存数组
    go_stop = 0; // 车辆起停默认为停止
    while (1) {
        bt_command = Bluetooth_Read_Data(); // 蓝牙起停指令
        if (bt_command != 0) {
            Bluetooth_Send_Data(bt_command);
        }
        if (bt_command == 'W') { // 为特定指令时，2秒后出发
            if (go_stop == 0) {
                delay_ms(2000);
                go_stop = 1;
            }
        }
        Run();
    }
}

//CPU1主函数，置于循环中，摄像头读写由此核处理，建议用于摄像头相关计算：
//不要写成死循环，后面有AD相关处理
void UserCpu1Main(void) {
    // 摄像头探测斑马线出库
    strip_to_black_counter = 0;
    strip_to_white_counter = 0;
    for (i = 100; i >= 60; i--) {
        zebra_strip_black_pixel = 0;
        for (j = 20; j < 90; j++) {
            if (pic[i][j] <= 120) { // 灰度判断
                zebra_strip_black_pixel++;
            }
        }
        if (zebra_strip_black_pixel >= 20) { // 若黑色像素大于20
            if (black_strip_status == 0) { // 之前为白，则现在为黑，白至黑转化数++
                black_strip_status = 1;
                strip_to_black_counter++;
            }
        } else {
            if (black_strip_status == 1) { // 相反则同理
                black_strip_status = 0;
                strip_to_white_counter++;
            }
        }
        if ((strip_to_black_counter >= 3 && strip_to_white_counter >= 3) && out_turn == 0) { // 连续斑马线式变化则开始出库转弯
            out_turn = 1;
            break;
        }
    }
    // 摄像头交叉路口转弯
    num_black_strip = 0;
    num_white_strip = 0;
    crossroad_left_pixel = 0;
    crossroad_right_pixel = 0;
    // 左转符号
    for (i = 10; i <= 60; i++) {
        if (pic[110][i] >= 50 && pic[110][i] <= 80) { // 扫描预定转弯符号出现位置
            crossroad_left_pixel++;
        }
    }
    if (crossroad_left_pixel >= 20 && crossroad_left_pixel <= 30 && crossroad_left_status != 1) {
        crossroad_left_status = 1;
    }
    if (crossroad_left_pixel == 0 && crossroad_left_status == 1) { // 直到菱形顶端，开始转向
        crossroad_left_status = 2;
    }
    // 右转符号
    for (i = 45; i <= 95; i++) {
        if (pic[110][i] <= 50 && pic[110][i] <= 80) {
            crossroad_right_pixel++;
        }
    }
    if (crossroad_right_pixel >= 20 && crossroad_right_pixel <= 30 && crossroad_right_status != 1) {
        crossroad_right_status = 1;
    }
    if (crossroad_right_pixel == 0 && crossroad_right_status == 1) { // 直到菱形顶端，开始转向
        crossroad_right_status = 2;
    }
    if (crossroad_right_status == crossroad_left_status || in_out_status != 1) { // 若左右皆有转向符号，或者车辆不在赛道上，则不转弯
        crossroad_left_status = 0;
        crossroad_right_status = 0;
    }
    // 摄像头探测斑马线入库
    for (j = 20; j <= 100; j++) {
        ver_zebra_strip_white_pixel = 0;
        for (i = 90; i >= 70; i--) {
            if (pic[i][j] <= 120) {
                ver_zebra_strip_white_pixel++;
            }
        }
        if (ver_zebra_strip_white_pixel >= 19) {
            if (black_strip_status == 0) {
                black_strip_status = 1;
                num_black_strip++;
            }
        } else {
            if (black_strip_status == 1) {
                black_strip_status = 0;
                num_white_strip++;
            }
        }
        if (num_black_strip >= 5 && num_white_strip >= 5 && out_turn == 2 && in_turn == 0) { // 入库前，摄像头纵向斑马线识别
            in_turn = 1;
        }
    }
}

/**************************************中断调用函数****************************************/
uint32 UserInterupt10ms(void) {
    Avg_Mag_Filter(); // 刷新电磁数据
    if (entry_delay_status == 1) {
        entry_delay_time++;
    }
    if (entry_delay_time == 90) { // 环岛入弯检测
        entry_delay_status = 0;
        entry_delay_time = 0;
    }
    return 0;
}

uint32 UserInterupt100ms(void) {
    Avg_Sonic_Filter(); // 刷新距离数据
    code_period = GetCodePerid(); // 编码器读数
    Speed_Control(my_speed); // 编码器速度控制
    return 0;
}

//该函数每1000ms执行一次，请在该函数中书写程序，中断时间有限，不要太长
uint32 UserInterupt1000ms(void) {
    return 0;
}
