/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-24     16795       the first version
 */

#include "pid.h"

void Init_pid(struct PID *pid)
{
    pid->error = 0;
    pid->Si = 0;
    pid->out = 0;
}

float Position_pid(struct PID *pid,float target, float now)
{
    pid->target = target;
    pid->now = now;

    pid->error_last = pid->error;

    pid->error = pid->target - pid->now;
    pid->Sp = pid->Kp * pid->error;
    pid->Si += pid->Ki * pid->error;
    pid->Sd = pid->Kd * (pid->error - pid->error_last);

    return pid->out = pid->Sp + pid->Si + pid->Sd;
}
