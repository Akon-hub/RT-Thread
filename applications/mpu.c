/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-20     16795       the first version
 */

#include <rtdevice.h>
#include "board.h"
#include "mpu6xxx.h"
#include <rtdbg.h>
#include <math.h>
#include <mpu.h>

static rt_thread_t posture_thread = RT_NULL;

#define MPU6050_I2C_BUS_NAME          "i2c1"      //i2c总线设备名
#define MPU6050_ADDR                  0x68        //MPU6050地址，既定的
struct mpu6xxx_device *i2c_bus;     //6050控制句柄

struct mpu_data mpu;
struct mpu_data mpu_raw;
struct angle angle;

//const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
const float Gyro_G = 0.03051756f*2;     //陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2
const float Gyro_Gr = 0.0005326f*2;     //面计算度每秒,转换弧度每秒则 2*0.03051756    * 0.0174533f = 0.0005326*2

volatile struct mpu6xxx_3axes GyroIntegError = {0};

static float NormAcc;

/* 四元数系数 */
typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;
Quaternion NumQ = {1, 0, 0, 0};

//一维卡尔曼滤波
void kalmanfiter(struct KalmanFilter *EKF,float input)
{
    EKF->NewP = EKF->LastP + EKF->Q;
//    EKF->NewP = EKF->LastP + EKF->Q;
    EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
    EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
    EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
}

//一维卡尔曼滤波
void kalman_fiter(struct KalmanFilter *EKF, rt_int16_t size,struct mpu6xxx_3axes *axes)
{
//    EKF->NewP = EKF->LastP + EKF->Q;
//    EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
//    EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
//    EKF->LastP = (1 - EKF->Kg) * EKF->NewP;

//    (EKf + size) -> NewP = 1;
//     (EKF+1)->NewP = *(EKF+1)->LastP;

    float x[3];
    x[0] = axes->x;
    x[1] = axes->y;
    x[2] = axes->z;
    for (int8_t var = 0; var < size; ++var) {
        (EKF+var)->NewP = (EKF+var)->LastP + (EKF+var)->Q;
        (EKF+var)->Kg = (EKF+var)->NewP / ((EKF+var)->NewP + (EKF+var)->R);
        (EKF+var)->Out = (EKF+var)->Out + (EKF+var)->Kg * (x[var] - (EKF+var)->Out);
        (EKF+var)->LastP = (1 - (EKF+var)->Kg) * (EKF+var)->NewP;
    }
}

rt_uint64_t cou=0;

void GetAngle(struct mpu_data *mpu,struct angle *pAngE, float dt)
{
    volatile struct mpu6xxx_3axes Gravity,Acc,Gyro,AccGravity;

    static  float KpDef = 0.5f ;
    static  float KiDef = 0.0001f;
//static  float KiDef = 0.00001f;

    float q0_t,q1_t,q2_t,q3_t;
  //float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    //提取等效旋转矩阵中的重力分量
    Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // 加速度归一化
    //printf("accX:%d\r\n",MPU6050.accX);
    NormAcc = 1/sqrt(squa(mpu_raw.accel.x)+ squa(mpu_raw.accel.y) +squa(mpu_raw.accel.z));
    //printf("NorAcc%f\r\n",NormAcc);
    //  NormAcc = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));

    Acc.x = mpu->accel.x * NormAcc;
    Acc.y = mpu->accel.y * NormAcc;
    Acc.z = mpu->accel.z * NormAcc;

    //向量差乘得出的值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    //再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    //角速度融合加速度积分补偿值
    Gyro.x = mpu->accel.x * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
    Gyro.y = mpu->accel.y * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = mpu->accel.z * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;

    // 一阶龙格库塔法, 更新四元数
    q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    // 四元数归一化
    NormQuat = 1/sqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // 四元数转欧拉角
    {

#ifdef  YAW_GYRO
        *(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
#else
        float yaw_G = mpu->gyro.z * Gyro_G;
        if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //数据太小可以认为是干扰，不是偏航动作
        {
            pAngE->yaw  += yaw_G * dt;
//            printf("Yaw:%f\r\n",pAngE->yaw);
        }
#endif
        pAngE->pitch  =  asin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;

        pAngE->roll = atan2(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;  //PITCH

        cou++;
        if (cou % 10 == 0) {
            printf("Yaw:%f, %f, %f\n",pAngE->yaw,pAngE->pitch,pAngE->roll);
        }

//        printf("Pitch:%f\n",pAngE->pitch);
//        printf("Roll:%f\n",pAngE->roll);

//        rt_kprintf("Pitch:%f;\r\n",pAngE->pitch);
//        rt_kprintf("Roll:%f;\r\n",pAngE->roll);
    }
}
/* 读取MPU6050数据并加滤波 */
void MpuGetData(void)
{
    uint8_t i;
  uint8_t buffer[12];

//    HAL_I2C_Mem_Read(&hi2c1, MPU_READ, 0X3B, I2C_MEMADD_SIZE_8BIT, buffer, 6, 0xfff);               /* 读取角加速度 */
//    HAL_I2C_Mem_Read(&hi2c1, MPU_READ, 0x43, I2C_MEMADD_SIZE_8BIT, &buffer[6], 6, 0xfff);       /* 读取角速度 */

    mpu6xxx_get_gyro(i2c_bus, &mpu.gyro);
    mpu6xxx_get_accel(i2c_bus, &mpu.accel);

//    rt_kprintf("accel1.x: %d, y: %d, z: %d\n",accel1.x,accel1.y,accel1.z);
    static struct KalmanFilter EKF[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};
//  printf("buff:%d\r\n",buffer[1]);
    kalman_fiter(&EKF, 3, &mpu.accel);

//    rt_kprintf("gyro.x: %d, y: %d, z: %d accel1.x: %d, y: %d, z: %d\n",gyro1.x,gyro1.y,gyro1.z,accel1.x,accel1.y,accel1.z);
//
//    rt_kprintf("%d, %d\n", gyro1.x, gyro1.y);


//    kalman_fiter(&EKF[1], (float)accel1.y);
//    kalman_fiter(&EKF[2], (float)accel1.z);

//    for(i=0;i<6;i++)
//    {
//        pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1])-MpuOffset[i];                           /* 将数据整为16bit，并减去水平校准值 */
//        if(i < 3)       /* 角加速度卡尔曼滤波 */
//        {
//            {
//                static struct KalmanFilter EKF[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};
//                kalmanfiter(&EKF[i],(float)pMpu[i]);
//                pMpu[i] = (int16_t)EKF[i].Out;
////              printf("EKF:%f\r\n",EKF[i].Out);
//            }
//        }
//        if(i > 2)       /* 角速度一阶互补滤波 */
//        {
//            uint8_t k=i-3;
//            const float factor = 0.15f;
//            static float tBuff[3];
//
//            pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;
//        }
//    }
}



/* 线程 1 的入口函数 */
static void posture_thread_entry(void *parameter)
{
    rt_uint16_t count = 0;
    float temp = 0;
    while (1)
    {

        MpuGetData();
        GetAngle(&mpu, &angle, 0.005f);

        rt_thread_mdelay(5);
    }
}

/* 线程示例 */
int posture_thread_init(void)
{
    i2c_bus = (struct mpu6xxx_device *)mpu6xxx_init(MPU6050_I2C_BUS_NAME, MPU6050_ADDR);   //初始化MPU6050，测量单位为角速度，加速度

    mpu6xxx_set_param(i2c_bus, MPU6XXX_ACCEL_RANGE, MPU6XXX_GYRO_RANGE_250DPS);  //陀螺仪范围配置
    mpu6xxx_set_param(i2c_bus, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);     //加速度计
    mpu6xxx_set_param(i2c_bus, MPU6XXX_DLPF_CONFIG, MPU6XXX_DLPF_188HZ);         //低通滤波
    mpu6xxx_set_param(i2c_bus, MPU6XXX_SAMPLE_RATE, 500);                       //采样频率

    mpu6xxx_get_accel_offset(i2c_bus, &mpu.accel);
    mpu6xxx_get_gyro_offset(i2c_bus, &mpu.gyro);

    mpu6xxx_set_accel_offset(i2c_bus, &mpu.accel);
    mpu6xxx_set_gyro_offset(i2c_bus, &mpu.gyro);

    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    posture_thread = rt_thread_create("posture_thread",
                            posture_thread_entry, RT_NULL,
                            4096,
                            4, 3);

    /* 如果获得线程控制块，启动这个线程 */
    if (posture_thread != RT_NULL)
        rt_thread_startup(posture_thread);

    return 0;
}
