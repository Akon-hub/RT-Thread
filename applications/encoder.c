/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-20     16795       the first version
 */

#include <board.h>
#include <rtdevice.h>

#include <screen.h>

#define DBG_TAG "encoder_speed"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define AS5600_I2C_BUS_NAME      "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define AS5600_ADDR               0x0C   /* 从机地址 */
struct rt_i2c_bus_device *i2c_bus;      /* I2C总线设备句柄 */

#define AS5600_RAW_ANGLE_REGISTER  0x0C           //0x0C寄存器


static rt_bool_t initialized = RT_FALSE;                /* 传感器初始化状态 */

static rt_thread_t encoder_thread = RT_NULL;

rt_device_t pulse_encoder_dev = RT_NULL;   /* 脉冲编码器设备句柄 */

#define PULSE_ENCODER_DEV_NAME    "pulse2"    /* 脉冲编码器名称 */

extern struct rt_mailbox move_screen_mail;


/* 写传感器寄存器
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t *data)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;
    rt_uint32_t buf_size = 1;

    buf[0] = reg; //cmd
    if (data != RT_NULL)
    {
        buf[1] = data[0];
        buf[2] = data[1];
        buf_size = 3;
    }

    msgs.addr = AHT10_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = buf_size;

     调用I2C设备接口传输数据
    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

 读传感器寄存器数据
static rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs;

    msgs.addr = AHT10_ADDR;
    msgs.flags = RT_I2C_RD;
    msgs.buf = buf;
    msgs.len = len;

     调用I2C设备接口传输数据
    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static void read_temp_humi(float *cur_temp, float *cur_humi)
{
    rt_uint8_t temp[6];

    write_reg(i2c_bus, AHT10_GET_DATA, RT_NULL);       发送命令
    rt_thread_mdelay(400);
    read_regs(i2c_bus, 6, temp);                 获取传感器数据

     湿度数据转换
    *cur_humi = (temp[1] << 12 | temp[2] << 4 | (temp[3] & 0xf0) >> 4) * 100.0 / (1 << 20);
     温度数据转换
    *cur_temp = ((temp[3] & 0xf) << 16 | temp[4] << 8 | temp[5]) * 200.0 / (1 << 20) - 50;
}

static void aht10_init(const char *name)
{
    rt_uint8_t temp[2] = {0, 0};

     查找I2C总线设备，获取I2C总线设备句柄
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(name);

    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", name);
    }
    else
    {
        write_reg(i2c_bus, AHT10_NORMAL_CMD, temp);
        rt_thread_mdelay(400);

        temp[0] = 0x08;
        temp[1] = 0x00;
        write_reg(i2c_bus, AHT10_CALIBRATION_CMD, temp);
        rt_thread_mdelay(400);
        initialized = RT_TRUE;
    }
}

static void i2c_aht10_sample(int argc, char *argv[])
{
    float humidity, temperature;
    char name[RT_NAME_MAX];

    humidity = 0.0;
    temperature = 0.0;

    if (argc == 2)
    {
        rt_strncpy(name, argv[1], RT_NAME_MAX);
    }
    else
    {
        rt_strncpy(name, AHT10_I2C_BUS_NAME, RT_NAME_MAX);
    }

    if (!initialized)
    {
         传感器初始化
        aht10_init(name);
    }
    if (initialized)
    {
         读取温湿度数据
        read_temp_humi(&temperature, &humidity);

        rt_kprintf("read aht10 sensor humidity   : %d.%d %%\n", (int)humidity, (int)(humidity * 10) % 10);
        if( temperature >= 0 )
        {
            rt_kprintf("read aht10 sensor temperature: %d.%d°C\n", (int)temperature, (int)(temperature * 10) % 10);
        }
        else
        {
            rt_kprintf("read aht10 sensor temperature: %d.%d°C\n", (int)temperature, (int)(-temperature * 10) % 10);
        }
    }
    else
    {
        rt_kprintf("initialize sensor failed!\n");
    }
}*/


void get_raw_angle()
{

}

void get_angle()
{

}


/* 线程 1 的入口函数 */
static void encoder_thread_entry(void *parameter)
{
    rt_int32_t count;
    struct move_screen move;
    while (1)
    {
        rt_thread_mdelay(10);

        /* 读取脉冲编码器计数值 */
        rt_device_read(pulse_encoder_dev, 0, &count, 1);
        rt_device_control(pulse_encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
        if (abs(count) < 100 && count != 0) {
//            rt_kprintf("get_count: %d\n",count);
            move.x = count;
            rt_mb_send(&move_screen_mail, (rt_ubase_t)&move);
        }
    }
}

extern void MX_TIM2_Init(void);

/* 线程示例 */
int encoder_thread_init(void)
{
    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    encoder_thread = rt_thread_create("encoder_thread",
                            encoder_thread_entry, RT_NULL,
                            1024,
                            20, 1);

    /* 如果获得线程控制块，启动这个线程 */
    if (encoder_thread != RT_NULL)
        rt_thread_startup(encoder_thread);

    MX_TIM2_Init();

    /* 查找脉冲编码器设备 */
    pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);
    if (pulse_encoder_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_DEV_NAME);
        return RT_ERROR;
    }

    rt_err_t ret = RT_EOK;

    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME);
        return ret;
    }

    return 0;
}

