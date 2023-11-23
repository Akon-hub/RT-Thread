/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-18     16795       the first version
 */
#ifndef APPLICATIONS_BUTTON_H_
#define APPLICATIONS_BUTTON_H_

extern int button_thread_init(void);

#define BUTTON_PRESS_DOWN_EVENT_FLAG (1 << 1)
#define BUTTON_HOLD_EVENT_FLAG (1 << 2)
#define BUTTON_PRESS_UP_EVENT_FLAG (1 << 3)
#define BUTTON_CLICK_EVENT_FLAG (1 << 4)

#endif /* APPLICATIONS_BUTTON_H_ */
