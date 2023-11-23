/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-19     16795       the first version
 */
#include <board.h>
#include <rtdevice.h>
#include <math.h>

#define PWM_DEV_NAME        "pwm1"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL_A     1       /* PWM通道 */
#define PWM_DEV_CHANNEL_B     2       /* PWM通道 */
#define PWM_DEV_CHANNEL_C     3       /* PWM通道 */

struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */

extern void MX_TIM1_Init(void);

static rt_thread_t motor_thread = RT_NULL;


#define CONSTANT_AMPLITUDE_TRANSFORMATION
void clark_transform(float_t I_a,float_t I_b, float_t I_c,float_t *I_alpha,float_t *I_beta)
{
    //等幅值变换
#ifdef CONSTANT_AMPLITUDE_TRANSFORMATION
    *I_alpha = I_a;
    *I_beta  = (I_a + (2*I_b))/sqrtf(3);
#else//等功率变换
    *I_alpha = sqrtf(2/3) * (I_a - I_b/2 + I_c/2);
    *I_beta  = sqrtf(2/3) * (I_b * sqrtf(3)/2 - I_c * sqrtf(3)/2);
#endif
}

void park_transform(float_t I_alpha,float_t I_beta,float_t theta, float_t *I_d, float_t *I_q)
{
    *I_d = I_alpha * cosf(theta) + I_beta * sinf(theta);
    *I_q =-I_alpha * sinf(theta) + I_beta * cosf(theta);
}

void park_retransform(float_t U_d, float_t U_q, float_t theta, float_t *U_alpha, float_t *U_beta)
{
    *U_alpha = U_d * cos(theta) - U_q * sin(theta);
    *U_beta  = U_d * sin(theta) + U_q * cos(theta);
}

void clark_retransform(float_t U_alpha, float_t U_beta,float_t *U_r1,float_t *U_r2,float_t *U_r3)
{
    *U_r1 = U_beta;
    *U_r2 = (-U_beta + sqrtf(3)*U_alpha)/2;
    *U_r3 = (-U_beta - sqrtf(3)*U_alpha)/2;
}

void set_phase_voltage(float_t U_q, float_t U_d, float_t theta)
{
    float_t U_alpha,U_beta;
    theta = theta;
    park_retransform(U_d, U_q, theta, &U_alpha, &U_beta);

}

void speed_PID()
{

}

uint8_t sector(float_t U_alpha,float_t U_beta)
{
    rt_bool_t A,B,C;
    A = U_beta > 0;
    B = ( sqrtf(3)*U_alpha - U_beta) > 0;
    C = (-sqrtf(3)*U_alpha - U_beta) > 0;

    uint8_t N;
    N = A + 2*B + 4*C;
    return N;
}

void FOC_function(int16_t voltage,float_t theta,float_t target_speed, float_t I_a,float_t I_b,float_t I_c,float_t now_speed)
{
    float_t Kp,Ki;

    float_t error_speed = target_speed - now_speed;
    float_t I_q_ref = Kp*error_speed;


    float_t I_alpha,I_beta;
    clark_transform(I_a, I_b, I_c, &I_alpha, &I_beta);

    float_t I_d,I_q;
    park_transform(I_alpha, I_beta, theta, &I_d, &I_q);

    float_t U_d,U_q;
    //随便写的，好像开环就是这样
    U_d = I_d;
    U_d = I_q;

    float_t U_alpha,U_beta;

    park_retransform(U_d, U_q, theta, &U_alpha, &U_beta);

    //SVPWM

    //sector
    uint8_t N = sector(U_alpha, U_beta);


    float_t X,Y,Z;
    uint16_t T;
    X = (sqrtf(3) * U_beta * T)/voltage;
    Y = ((sqrtf(3)*U_beta + (3/2)*U_alpha)*T)/voltage;
    Z = (((sqrtf(3)/2)*U_beta - (3/2)*U_alpha)*T)/voltage;

    uint16_t T_1,T_2;
    switch(N)
    {
    case 1:
    {
        T_1 = Z;
        T_2 = Y;
        break;
    }
    case 2:
    {
        T_1 = Y;
        T_2 = -X;
        break;
    }
    case 3:
    {
        T_1 = -Z;
        T_2 = X;
        break;
    }
    case 4:
    {
        T_1 = -X;
        T_2 = Z;
        break;
    }
    case 5:
    {
        T_1 = X;
        T_2 = -Y;
        break;
    }
    case 6:
    {
        T_1 = -Y;
        T_2 = -Z;
        break;
    }
    }

    uint16_t T_a,T_b,T_c;
    T_a = (T - T_1 - T_2)/4;
    T_b = T_a + T_1/2;
    T_c = T_b + T_2/2;
}

void function()
{
    float_t theta;

}

/* 线程 1 的入口函数 */
static void motor_thread_entry(void *parameter)
{
    rt_uint32_t count = 0;

    while (1)
    {
        /* 线程 1 采用低优先级运行，一直打印计数值 */
        rt_kprintf("thread1 count: %d\n", count ++);
        rt_thread_mdelay(500);
    }
}

/* 线程示例 */
int motor_thread_init(void)
{
    MX_TIM1_Init();

    rt_uint32_t period, pulse;

    period = 100000;    /* 周期为5ns，单位为纳秒ns */
    pulse = 50000;          /* PWM脉冲宽度值，单位为纳秒ns */

    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return RT_ERROR;
    }

    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_A, period, pulse);
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_B, period, pulse);
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_C, period, pulse);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL_A);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL_B);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL_C);

    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    motor_thread = rt_thread_create("motor_thread",
                            motor_thread_entry, RT_NULL,
                            512,
                            5, 1);

    /* 如果获得线程控制块，启动这个线程 */
    if (motor_thread != RT_NULL)
        rt_thread_startup(motor_thread);

    return 0;
}
