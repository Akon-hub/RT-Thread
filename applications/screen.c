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

#include "pid.h"

static rt_thread_t screen_thread = RT_NULL;

#define OLED_I2C_PIN_SCL                    42  // PB6
#define OLED_I2C_PIN_SDA                    43  // PB7

#define START_POSITION_X 6
#define START_POSITION_Y 12

/* 邮箱控制块 */
struct rt_mailbox move_screen_mail;
/* 用于放邮件的内存池 */
static char move_screen_mail_pool[512];

u8g2_t u8g2;

const rt_uint16_t show_y_distance = 14;//每行间距
const rt_uint16_t show_line_distance = 6;//每行间距，必须为偶数，尽量是frame_words_distance的两倍
const float move_x_speed_mutiple = 2.5;
float move_y_speed_mutiple = 0.7;

const rt_uint8_t show_word_pixel = 8;

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
        {"I can't",7}
};

rt_int16_t show_x = START_POSITION_X,show_y = START_POSITION_Y,show_x_target,show_y_target;

struct Frame
{
    float length;
    float height;
    float position_x,position_y;
    float r;//3

    float position_y_last,position_x_last,length_last,height_last;

    rt_int16_t length_target,height_target,position_x_target,position_y_target,r_target;

    rt_uint8_t now_line;

    const rt_uint8_t words_distance;//3
    const rt_uint8_t single_words_width;//6

    rt_uint8_t max_line;
};
struct Frame frame =
{
        .words_distance = 4,
        .single_words_width = 6,
        .r = 3,
        .now_line = 0,
};
void frame_init()
{
    frame.max_line = sizeof(screen_list)/sizeof(struct Seting_list);
    frame.position_x = show_x - frame.words_distance;
    frame.position_y = show_y - show_word_pixel + show_y_distance*frame.now_line-frame.words_distance;
    frame.height = show_word_pixel + frame.words_distance*2;
    frame.length = screen_list[frame.now_line].length*frame.single_words_width + frame.words_distance*2;

    frame.position_x_target = frame.position_x;
    frame.position_y_target = frame.position_y;
    frame.height_target = frame.height;
    frame.length_target = frame.length;
}

void ui_init()
{
    frame_init();

    show_x_target = show_x;
    show_y_target = show_y;
}


void ui_frame_move(enum Direct direct,rt_int16_t show_x, rt_int16_t show_y)
{
    switch (direct) {
        case Up:
            if (frame.now_line == 0) {
                break;
            }
            frame.now_line -= 1;
            break;
        case Down:
            if (frame.now_line+1 >= frame.max_line) {
                break;
            }
            frame.now_line += 1;
            break;
        default:
            break;
    }

    frame.position_x_target = show_x - frame.words_distance;
    frame.position_y_target = show_y - show_word_pixel + (show_word_pixel + show_line_distance)*frame.now_line - frame.words_distance;
    frame.length_target = screen_list[frame.now_line].length*frame.single_words_width + frame.words_distance*2;
    frame.height_target = show_word_pixel + frame.words_distance*2;
    frame.r_target = (frame.length_target - frame.length)/frame.length*10 + (frame.position_y_target - frame.position_y)/frame.position_y*10 + 3;

}
struct PID screen_pid_y;
struct PID screen_pid_l;
struct PID screen_pid_h;
struct PID screen_pid_x;
struct PID screen_pid_r;

void Init_screen_pid()
{
    Init_pid(&screen_pid_x);
    Init_pid(&screen_pid_y);
    Init_pid(&screen_pid_h);
    Init_pid(&screen_pid_l);
    Init_pid(&screen_pid_r);

    float screen_kp = 0.6,screen_ki = 0.03,screen_kd = 0.01;

    Set_Pid(&screen_pid_x, screen_kp, screen_ki, screen_kd);
    Set_Pid(&screen_pid_y, screen_kp, screen_ki, screen_kd);
    Set_Pid(&screen_pid_l, screen_kp, screen_ki, screen_kd);
    Set_Pid(&screen_pid_h, screen_kp, screen_ki, screen_kd);
    Set_Pid(&screen_pid_r, screen_kp, screen_ki, screen_kd);
}

void ui_show()
{
    rt_uint8_t list_length = sizeof(screen_list)/sizeof(struct Seting_list);
    u8g2_ClearBuffer(&u8g2);

    u8g2_SetDrawColor(&u8g2, 1);
    for (rt_uint8_t var = 0; var < list_length; ++var) {
        if (var == frame.now_line) {
            continue;
        }
        u8g2_DrawUTF8(&u8g2, show_x, show_y+show_y_distance*var, screen_list[var].str);
    }
    u8g2_DrawRBox(&u8g2, frame.position_x, frame.position_y, frame.length, frame.height, frame.r);

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawUTF8(&u8g2, show_x, show_y+show_y_distance*frame.now_line, screen_list[frame.now_line].str);

    u8g2_SendBuffer(&u8g2);

    frame.position_x_last = frame.position_x;
    frame.position_y_last = frame.position_y;
    frame.length_last = frame.length;
    frame.height_last = frame.height;

    frame.length += Position_pid(&screen_pid_l, frame.length_target, frame.length);
    frame.height += Position_pid(&screen_pid_h, frame.height_target, frame.height);
    frame.position_x += Position_pid(&screen_pid_x, frame.position_x_target, frame.position_x);
    frame.position_y += Position_pid(&screen_pid_y, frame.position_y_target, frame.position_y);
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

    ui_init();

    Init_screen_pid();
    ui_show();
    while (1)
    {

        struct move_screen *str;
        /* 从邮箱中收取邮件 */
        if (rt_mb_recv(&move_screen_mail, (rt_ubase_t *)&str, 0) == RT_EOK)
        {
            show_x_target += str->x*move_x_speed_mutiple;
            show_y_target += str->y*show_y_distance;
            ui_frame_move(str->direct, show_x, show_y);
        }

        ui_show();

        rt_thread_mdelay(1);
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
