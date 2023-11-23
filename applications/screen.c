/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-21     16795       the first version
 */
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <u8g2_port.h>
#include <stdio.h>

#define DBG_TAG "screen"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_thread_t screen_thread = RT_NULL;

#define OLED_I2C_PIN_SCL                    42  // PB6
#define OLED_I2C_PIN_SDA                    43  // PB7

/* 邮箱控制块 */
struct rt_mailbox mail_screen;
/* 用于放邮件的内存池 */
static char mail_screen_pool[512];


u8g2_t u8g2;

struct Seting_list
{
    char *str;
};
struct Seting_list screen_list[]=
{
        {"Hello World"},
        {"No Thanks"},
        {"Why not?"},
        {"I can't forgive you"}
};
rt_uint8_t frame_length,frame_y,frame_legth_target,frame_y_target;

rt_int16_t show_x = 10,show_y = 15,show_x_target,show_y_target;

void ui_run(rt_int16_t *a,rt_int16_t *a_tar)
{
    if (*a < *a_tar) {
        *a += 1;
    }
    else if (*a > *a_tar) {
        *a -= 1;
    }
    else {

    }

}

void ui_show()
{
    rt_uint8_t list_length = sizeof(screen_list)/sizeof(struct Seting_list);
    u8g2_ClearBuffer(&u8g2);

//    u8g2_SetFont(&u8g2, u8g2_font_amstrad_cpc_extended_8f);
    for (rt_uint8_t var = 0; var < list_length; ++var) {
        u8g2_DrawUTF8(&u8g2, show_x, show_y+15*var, screen_list[var].str);
    }
    u8g2_SendBuffer(&u8g2);
}

/* 线程 1 的入口函数 */
static void screen_thread_entry(void *parameter)
{
    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    // Draw Graphics
    u8g2_ClearBuffer(&u8g2);

    rt_int16_t x = 60,x_target = 100;

    u8g2_SetFont(&u8g2, u8g2_font_8x13B_tr);

    ui_show();
    while (1)
    {
        rt_int32_t *str;
        /* 从邮箱中收取邮件 */
        if (rt_mb_recv(&mail_screen, (rt_int32_t *)&str, RT_WAITING_FOREVER) == RT_EOK)
        {
            LOG_I("Screen %d",*str);
            show_x += (*str)*2.5f;
            ui_show();
            /* 延时 100ms */
            rt_thread_mdelay(5);
        }
    }
}

/* 线程示例 */
int screen_thread_init(void)
{
    rt_err_t result;

    /* 初始化一个 mailbox */
    result = rt_mb_init(&mail_screen,
                        "mbt",                      /* 名称是 mbt */
                        &mail_screen_pool[0],                /* 邮箱用到的内存池是 mb_pool */
                        sizeof(mail_screen_pool) / 4,        /* 邮箱中的邮件数目，因为一封邮件占 4 字节 */
                        RT_IPC_FLAG_FIFO);          /* 采用 FIFO 方式进行线程等待 */
    if (result != RT_EOK)
    {
        rt_kprintf("init mailbox failed.\n");
        return -1;
    }

    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    screen_thread = rt_thread_create("screen_thread",
                            screen_thread_entry, RT_NULL,
                            2048,
                            18, 5);

    /* 如果获得线程控制块，启动这个线程 */
    if (screen_thread != RT_NULL)
        rt_thread_startup(screen_thread);

    return 0;
}
