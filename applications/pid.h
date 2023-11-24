/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-24     16795       the first version
 */
#ifndef APPLICATIONS_PID_H_
#define APPLICATIONS_PID_H_

struct PID
{
    float Kp;
    float Ki;
    float Kd;

    float Sp;
    float Si;
    float Sd;

    float target;
    float now;
    float error;
    float error_last;

    float out;
};

extern void Init_pid(struct PID *pid);

extern void Set_Pid(struct PID *pid, float kp, float ki, float kd);

extern float Position_pid(struct PID *pid,float target, float now);

#endif /* APPLICATIONS_PID_H_ */
