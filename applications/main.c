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

#define LOG_TAG "main"
#define LOG_LVL LOG_LVL_INFO
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

extern rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, GPIO_TypeDef *cs_gpiox, uint16_t cs_gpio_pin);
extern rt_spi_flash_device_t rt_sfud_flash_probe(const char *spi_flash_dev_name, const char *spi_dev_name);

int main(void)
{
    int count = 1;

    rt_hw_spi_device_attach("spi5","SPI5",GPIOF,GPIO_PIN_6);

    w25q64 = rt_sfud_flash_probe("W25Q64", "SPI5");
    if (w25q64 == RT_NULL) {
        LOG_E("W25Q64 Not Found!");
    }

    easyflash_init();

    led_thread_init();

    button_thread_init();

    screen_thread_init();
    encoder_thread_init();

    while (count++)
    {
        LOG_I("system is still running");
        rt_mb_send(&led_mail, (rt_ubase_t)&"50,100");
        rt_thread_mdelay(5000);
    }

    return RT_EOK;
}
