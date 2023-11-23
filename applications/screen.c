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

#include "screen.h"

#define DBG_TAG "screen"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_thread_t screen_thread = RT_NULL;

#define OLED_I2C_PIN_SCL                    42  // PB6
#define OLED_I2C_PIN_SDA                    43  // PB7

/* 邮箱控制块 */
struct rt_mailbox move_screen_mail;
/* 用于放邮件的内存池 */
static char move_screen_mail_pool[512];


u8g2_t u8g2;

struct Seting_list
{
    char *str;
    rt_uint8_t length;
};
struct Seting_list screen_list[]=
{
        {"Hello World",11},
        {"No Thanks",9},
        {"Why not?",8},
        {"I can't",7},
        {"Ayou",4}
};
//rt_uint8_t frame_length,frame_y,frame_legth_target,frame_y_target;

rt_int16_t show_x = 6,show_y = 12,show_x_target,show_y_target;

rt_uint16_t frame_length,frame_length_target;
rt_uint16_t frame_hight,frame_hight_target;
rt_uint16_t frame_position_x,frame_position_y;
rt_uint16_t frame_position_x_target,frame_position_y_target;
rt_uint8_t frame_now_line = 0;
const rt_uint8_t frame_words_distance = 3;
const rt_uint8_t frame_single_words_width = 6;
const rt_uint8_t frame_r = 3;

const rt_uint16_t show_y_distance = 14;//每行间距
const rt_uint16_t show_line_distance = 6;//每行间距，必须为偶数，尽量是frame_words_distance的两倍
const float move_x_speed_mutiple = 2.5;
const float move_y_speed_mutiple = 3;
const rt_uint8_t show_word_pixel = 8;

void ui_move(rt_int16_t *y, rt_int16_t y_target)
{
    uint8_t speed;
    if (abs(y_target - *y) < move_y_speed_mutiple) {
        speed = 1;
    }else {
        speed = move_y_speed_mutiple;
    }
    if (*y < y_target) {
        *y += speed;
    }
    else if (*y > y_target) {
        *y -= speed;
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
        u8g2_DrawUTF8(&u8g2, show_x, show_y+show_y_distance*var, screen_list[var].str);
    }


    frame_position_x = show_x - frame_words_distance;
    frame_position_y_target = show_y - show_word_pixel + (show_word_pixel + show_line_distance)*frame_now_line - frame_words_distance;
    frame_length_target = screen_list[frame_now_line].length*frame_single_words_width + frame_words_distance*2;
    frame_hight_target = show_word_pixel + frame_words_distance*2;
    u8g2_DrawRFrame(&u8g2, frame_position_x, frame_position_y, frame_length, frame_hight, frame_r);

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

    u8g2_SetFont(&u8g2, u8g2_font_profont12_mf );//8像素


    show_x_target = show_x;
    show_y_target = show_y;

    frame_position_x = show_x - frame_words_distance;
    frame_position_y = show_y - show_word_pixel + show_y_distance*frame_now_line - frame_words_distance;
    frame_length = screen_list[frame_now_line].length*frame_single_words_width + frame_words_distance*2;
    frame_hight = show_word_pixel + frame_words_distance*2;

    ui_show();
    while (1)
    {
        struct move_screen *str;
        /* 从邮箱中收取邮件 */
        if (rt_mb_recv(&move_screen_mail, (rt_ubase_t *)&str, 0) == RT_EOK)
        {
            LOG_I("Screen move x: %d, Screen move y: %d",str->x,str->y);
//            show_x_target += str->x*move_x_speed_mutiple;
//            show_y_target += str->y*show_y_distance;
            if (frame_now_line < sizeof(screen_list)/sizeof(struct Seting_list) - 1) {
                frame_now_line += 1;
            }
        }
//        ui_move(&show_y, show_y_target);
        ui_move(&frame_length, frame_length_target);
        ui_move(&frame_hight, frame_hight_target);
        ui_move(&frame_position_y, frame_position_y_target);
        ui_show();
        rt_thread_mdelay(2);
    }
}

/* 线程示例 */
int screen_thread_init(void)
{
    rt_err_t result;

    /* 初始化一个 mailbox */
    result = rt_mb_init(&move_screen_mail,
                        "move_screen_mail",                      /* 名称是 mbt */
                        &move_screen_mail_pool[0],                /* 邮箱用到的内存池是 mb_pool */
                        sizeof(move_screen_mail_pool) / 4,        /* 邮箱中的邮件数目，因为一封邮件占 4 字节 */
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
                            5, 5);

    /* 如果获得线程控制块，启动这个线程 */
    if (screen_thread != RT_NULL)
        rt_thread_startup(screen_thread);

    return 0;
}
