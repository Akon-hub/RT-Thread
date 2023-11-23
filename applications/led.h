/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-18     16795       the first version
 */
#ifndef APPLICATIONS_LED_H_
#define APPLICATIONS_LED_H_

extern int led_thread_init(void);

extern struct rt_mailbox led_mail;

#endif /* APPLICATIONS_LED_H_ */
