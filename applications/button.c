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
#include <rtdbg.h>
#include <button.h>

static rt_thread_t button_thread = RT_NULL;

struct agile_btn *button = RT_NULL;

/* 事件控制块 */
struct rt_event button_event;

static void btn_press_down_event_cb(agile_btn_t *btn)
{
    rt_kprintf("thread2: send BUTTON_DOWN_CLICK_FLAG\n");
    rt_event_send(&button_event, BUTTON_PRESS_DOWN_EVENT_FLAG);
}

static void btn_hold_event_cb(agile_btn_t *btn)
{
    rt_kprintf("thread2: send BUTTON_HOLD_EVENT_FLAG\n");
    rt_event_send(&button_event, BUTTON_HOLD_EVENT_FLAG);
}

static void btn_press_up_event_cb(agile_btn_t *btn)
{
    rt_kprintf("thread2: send BUTTON_UP_EVENT_FLAG\n");
    rt_event_send(&button_event, BUTTON_PRESS_UP_EVENT_FLAG);
}

static void btn_click_event_cb(agile_btn_t *btn)
{
//    click_count++;
    rt_kprintf("thread2: send BUTTON_EVENT_CLICK_FLAG\n");
    rt_event_send(&button_event, BUTTON_CLICK_EVENT_FLAG);

//    char str[] = "70,70";
//    rt_mb_send(&led_mail, (rt_uint32_t)&str);

//    agile_led_start(led);
//    LOG_D("button btn_click_event_cb");
//    LOG_I("click count num %d",click_count);
}


/* 线程 1 的入口函数 */
static void button_thread_entry(void *parameter)
{
    while (1)
    {

        LOG_I("button running");
        rt_thread_mdelay(10000);
    }
}

/* 线程示例 */
int button_thread_init(void)
{

    rt_err_t result;

    /* 初始化事件对象 */
    result = rt_event_init(&button_event, "button_event", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        rt_kprintf("init button_event failed.\n");
        return -1;
    }

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
//    agile_btn_set_event_cb(button,BTN_PRESS_DOWN_EVENT, btn_press_down_event_cb);
//    agile_btn_set_event_cb(button,BTN_HOLD_EVENT, btn_hold_event_cb);
//    agile_btn_set_event_cb(button,BTN_PRESS_UP_EVENT, btn_press_up_event_cb);
//    agile_btn_set_event_cb(button,BTN_CLICK_EVENT, btn_click_event_cb);
    agile_btn_start(button);

    return 0;
}
