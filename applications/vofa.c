/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-24     16795       the first version
 */

#include "vofa.h"
#include <board.h>
#include <rtdevice.h>


#define UART_NAME       "uart1"    /* 串口设备名称 */
static rt_device_t serial;                /* 串口设备句柄 */

struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 配置参数 */

#define CH_COUNT 6

float data[CH_COUNT];
uint8_t tailData[4];                    //定义的传输Buffer

static rt_thread_t vofa_thread = RT_NULL;

/* 线程 1 的入口函数 */
static void vofa_thread_entry(void *parameter)
{


    while (1)
    {

//        data[0] = (float)frame.position_y_target;    //转成浮点数
//        data[1] = (float)frame.position_y;
//        data[2] = (float)num;
//        data[3] = (float)10.f;
//        data[4] = (float)(frame.position_y - pos_y_last);

        rt_device_write(serial, 0, (uint8_t *)data, sizeof(data));


        tailData[0] = 0x00;                    //写如结尾数据
        tailData[1] = 0x00;
        tailData[2] = 0x80;
        tailData[3] = 0x7f;
        rt_device_write(serial, 0, (uint8_t *)tailData, sizeof(tailData));

        rt_thread_mdelay(5);
    }
}

/* 线程示例 */
int vofa_thread_init(void)
{
    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    vofa_thread = rt_thread_create("vofa_thread",
                            vofa_thread_entry, RT_NULL,
                            2048,
                            20, 1);

    /* 如果获得线程控制块，启动这个线程 */
    if (vofa_thread != RT_NULL)
        rt_thread_startup(vofa_thread);


    serial = rt_device_find(UART_NAME);

    /* 以非阻塞接收和阻塞发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);

    return 0;
}

