//
// Created by Z on 26-1-5.
//

#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "sfdmctrl.h"
#include "INS_task.h"
#define FRAME_LENGTH 6
#define PI 3.1415927
#define PI_2 1.5707963

typedef struct
{
    Wheel_Motor_t wheel_motor[2];
    Joint_Motor_t joint_motor[2];

    float target_v;
    float v_set;//期望速度，单位是m/s
    float x_set;//期望位置，单位是m
    float v;//实际的速度,单位是m/s
    float v_act;
    float x;//实际的位移，单位是m

    float phi_set;
    float d_phi_set;

    float turn_set;//期望yaw轴弧度
    float roll_set;	//期望roll轴弧度

    float leg_set;//期望腿长，单位是m
    float last_leg_set;

    float v_filter;//滤波后的车体速度，单位是m/s
    float x_filter;//滤波后的车体位置，单位是m

    float myPithR;
    float myPithGyroR;
    float myRoll;

    float total_yaw;

    uint8_t start_flag;//启动标志
    uint8_t front_flag;
    uint8_t last_front_flag;
    uint8_t turn_flag;
    uint8_t last_turn_flag;
    uint8_t prejump_flag;
    uint8_t jump_flag;//跳跃标志
    uint8_t recover_flag;//一种情况下的倒地自起标志
    uint8_t leg_flag;
    uint8_t autoleg_flag;
    uint8_t autoturn_flag;
    uint8_t fastturn_flag;
    uint8_t movejump_flag;
}chassis_t;

typedef struct
{
    /*左右两腿的公共参数，固定不变*/
    float right_l1;
    float left_l1;

    float left_T1;
    float right_T1;

    float left_F0;
    float right_F0;
    float roll_F0;

    float left_len;
    float right_len;

    //腿长变化率
    float left_len_dot;
    float right_len_dot;
} vmc_leg_t;

extern chassis_t chassis_move;
void Chassis_init(chassis_t *chassis);
void Chassis_task(void);
void mySaturate(float *in,float min,float max);
void chassis_feedback_update(chassis_t *chassis,INS_t *ins,vmc_leg_t *vmc_leg);



#endif //CHASSIS_TASK_H
