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

#include "screen.h"

static rt_thread_t button_thread = RT_NULL;

struct agile_btn *button_board = RT_NULL;
struct agile_btn *button_encoder = RT_NULL;


extern struct rt_mailbox led_mail;

struct move_screen move_board={0,0,Up};
static void button_board_click_event_cb(agile_btn_t *btn)
{

    rt_mb_send(&led_mail, (rt_uint32_t)&"100,120");
    rt_mb_send(&move_screen_mail, (rt_base_t)&move_board);
}

struct move_screen move_encoder={0,0,Down};
static void button_encoder_click_event_cb(agile_btn_t *btn)
{

    rt_mb_send(&move_screen_mail, (rt_base_t)&move_encoder);
    LOG_W("Button Encoder");
}


/* 线程 1 的入口函数 */
static void button_thread_entry(void *parameter)
{
    while (1)
    {
        agile_btn_process();

//        LOG_W("Now pin: %d",rt_pin_read(50));

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

    button_board = agile_btn_create(0,PIN_HIGH,PIN_MODE_INPUT_PULLDOWN);
    if (button_board == RT_NULL) {
        LOG_D("button NULL");
    }
    agile_btn_set_event_cb(button_board,BTN_CLICK_EVENT, button_board_click_event_cb);
    agile_btn_start(button_board);

    //编码器按键
    button_encoder = agile_btn_create(50,PIN_LOW,PIN_MODE_INPUT_PULLDOWN);
    if (button_encoder == RT_NULL) {
        LOG_D("button NULL");
    }
    agile_btn_set_event_cb(button_encoder,BTN_CLICK_EVENT, button_encoder_click_event_cb);
    agile_btn_start(button_encoder);

    return 0;
}
