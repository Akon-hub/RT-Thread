/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-18     16795       the first version
 */
#include <agile_button.h>
#include <button.h>

#define LOG_TAG "thread_running_time"
#define LOG_LVL LOG_LVL_INFO
#include <rtdbg.h>

static rt_thread_t button_thread = RT_NULL;

struct agile_btn *button = RT_NULL;

static void btn_press_down_event_cb(agile_btn_t *btn)
{
    rt_kprintf("thread2: send BUTTON_DOWN_CLICK_FLAG\n");
}

static void btn_hold_event_cb(agile_btn_t *btn)
{
    rt_kprintf("thread2: send BUTTON_HOLD_EVENT_FLAG\n");
}

static void btn_press_up_event_cb(agile_btn_t *btn)
{
    rt_kprintf("thread2: send BUTTON_UP_EVENT_FLAG\n");

}

extern struct rt_mailbox led_mail;
extern struct rt_mailbox mail_screen;

rt_int32_t num = 5;
static void btn_click_event_cb(agile_btn_t *btn)
{

    rt_mb_send(&led_mail, (rt_uint32_t)&"100,120");
    rt_mb_send(&mail_screen, (rt_int32_t *)&num);
}


/* 线程 1 的入口函数 */
static void button_thread_entry(void *parameter)
{
    rt_uint32_t thread_tick_start,thread_tick_stop;
    rt_uint32_t thread_tick_count = 0;
    while (1)
    {
        thread_tick_start = rt_tick_get();
        agile_btn_process();
        thread_tick_stop = rt_tick_get();
        thread_tick_count = thread_tick_stop - thread_tick_start;
        LOG_I("123");
//        LOG_I("button running");
        rt_thread_mdelay(5);
    }
}

/* 线程示例 */
int button_thread_init(void)
{

    agile_btn_env_init();

    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    button_thread = rt_thread_create("button_thread",
                            button_thread_entry, RT_NULL,
                            3072,
                            5, 1);

    /* 如果获得线程控制块，启动这个线程 */
    if (button_thread != RT_NULL)
        rt_thread_startup(button_thread);

    button = agile_btn_create(0,PIN_HIGH,PIN_MODE_INPUT_PULLDOWN);
    if (button == RT_NULL) {
        LOG_D("button NULL");
    }


    agile_btn_set_event_cb(button,BTN_PRESS_DOWN_EVENT, btn_press_down_event_cb);
    agile_btn_set_event_cb(button,BTN_HOLD_EVENT, btn_hold_event_cb);
    agile_btn_set_event_cb(button,BTN_PRESS_UP_EVENT, btn_press_up_event_cb);
    agile_btn_set_event_cb(button,BTN_CLICK_EVENT, btn_click_event_cb);
    agile_btn_start(button);

    return 0;
}
