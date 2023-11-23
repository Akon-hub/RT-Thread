/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-18     16795       the first version
 */
#include <agile_led.h>
#include "board.h"
#include <button.h>

#define DBG_TAG "led"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


static rt_thread_t led_thread = RT_NULL;

struct agile_led *led = RT_NULL;

/* 邮箱控制块 */
struct rt_mailbox led_mail;
/* 用于放邮件的内存池 */
static char led_mail_pool[512];

extern struct rt_event button_event;
/* 线程 1 的入口函数 */
static void led_thread_entry(void *parameter)
{
    while (1)
    {

//        char *str;
//        if (rt_mb_recv(&led_mail, (rt_uint32_t *)&str, RT_WAITING_FOREVER) == RT_EOK)
//        {
//            agile_led_dynamic_change_light_mode(led, str, 2);
//            agile_led_start(led);
//            LOG_D(str);
//            /* 延时 100ms */
//            rt_thread_mdelay(1000);
//        }

        rt_uint32_t e;

        /* 第一次接收事件，事件 3 或事件 5 任意一个可以触发线程 1，接收完后清除事件标志 */
        if (rt_event_recv(&button_event, BUTTON_CLICK_EVENT_FLAG,
                          RT_EVENT_FLAG_AND |RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &e) == RT_EOK)
        {
            const char * click_light_mode = "100,150";
            agile_led_dynamic_change_light_mode(led, click_light_mode, 2);
            agile_led_start(led);
            LOG_I("Receive Button Event");
        }
        rt_thread_mdelay(1000);
    }
}

/* 线程示例 */
int led_thread_init(void)
{
    rt_err_t result;

    /* 初始化一个 mailbox */
    result = rt_mb_init(&led_mail,
                        "led_mail",                      /* 名称是 mbt */
                        &led_mail_pool[0],                /* 邮箱用到的内存池是 mb_pool */
                        sizeof(led_mail_pool) / 4,        /* 邮箱中的邮件数目，因为一封邮件占 4 字节 */
                        RT_IPC_FLAG_FIFO);          /* 采用 FIFO 方式进行线程等待 */
    if (result != RT_EOK)
    {
        LOG_E("init Led Mail Fail!");
        return -1;
    }

    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    led_thread = rt_thread_create("led_thread",
                            led_thread_entry, RT_NULL,
                            2048,
                            20, 1);

    /* 如果获得线程控制块，启动这个线程 */
    if (led_thread != RT_NULL)
        rt_thread_startup(led_thread);

    const char * init_light_mode = "100,100";
    led = agile_led_create(GET_PIN(D,12), PIN_LOW, init_light_mode, 2);
    agile_led_start(led);


    return 0;
}

