/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-21     16795       the first version
 */
#ifndef APPLICATIONS_SCREEN_H_
#define APPLICATIONS_SCREEN_H_

extern struct rt_mailbox move_screen_mail;

struct move_screen
{
    rt_int32_t x;
    rt_int32_t y;
};

extern int screen_thread_init(void);

#endif /* APPLICATIONS_SCREEN_H_ */
