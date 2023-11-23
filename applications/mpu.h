/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-20     16795       the first version
 */
#ifndef APPLICATIONS_MPU_H_
#define APPLICATIONS_MPU_H_

#include "mpu6xxx.h"


//卡尔曼滤波参数结构体
struct KalmanFilter{
    float LastP;        //上一次协方差
    float NewP;     //最新的协方差
    float Out;          //卡尔曼输出
    float Kg;               //卡尔曼增益
    float Q;                //过程噪声的协方差
    float R;                //观测噪声的协方差
};


struct angle
{
    float roll;
    float pitch;
    float yaw;
};

/* 3-axis data structure */
struct mpu_data
{
    struct mpu6xxx_3axes gyro;
    struct mpu6xxx_3axes accel;
};

#define squa( Sq )        (((float)Sq)*((float)Sq))
//#define YAW_GYRO
#endif /* APPLICATIONS_MPU_H_ */
