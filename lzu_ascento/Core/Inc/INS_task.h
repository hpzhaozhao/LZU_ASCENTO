//
// Created by Z on 26-1-5.
//

#ifndef INS_TASK_H
#define INS_TASK_H
/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "stdint.h"
#include "BMI088driver.h"
// #include "QuaternionEKF.h"

#define X1 0
#define Y1 1
#define Z1 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    float YawAngleLast;
    float YawRoundCount;

    float v_n;//绝对系沿着水平运动方向的速度
    float x_n;//绝对系沿着水平运动方向的位移

    uint8_t ins_flag;
} INS_t;


/**
 * @brief 用于修正安装误差的参数,demo中可无视
 *
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern void INS_Init(void);
extern void INS_task(void);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);



#endif //INS_TASK_H
