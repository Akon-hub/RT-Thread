/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-17     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include <rtdevice.h>
#include "board.h"

#include <led.h>
#include <button.h>

#include "easyflash.h"
#include "spi_flash.h"


rt_spi_flash_device_t w25q64;


extern int encoder_thread_init(void);
extern int screen_thread_init(void);

int main(void)
{
    int count = 1;

    rt_hw_spi_device_attach("spi5","SPI5",GPIOF,GPIO_PIN_6);

    if (w25q64 = rt_sfud_flash_probe("W25Q64", "SPI5")) {
        rt_kprintf("W25Q64 Not Found!");
    }

    easyflash_init();

    led_thread_init();

    button_thread_init();

    screen_thread_init();
    encoder_thread_init();

    while (count++)
    {
        LOG_D("Hello RT-Thread!");
        rt_mb_send(&led_mail, (rt_ubase_t *)&"50,100");
        rt_thread_mdelay(5000);
    }

    return RT_EOK;
}
