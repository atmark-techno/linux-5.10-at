/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 NXP.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * @file linux/imx_rpmsg.h
 *
 * @brief Global header file for iMX RPMSG
 *
 * @ingroup RPMSG
 */
#ifndef __LINUX_IMX_RPMSG_H__
#define __LINUX_IMX_RPMSG_H__

/* Category define */
#define IMX_RMPSG_LIFECYCLE	1
#define IMX_RPMSG_PMIC		2
#define IMX_RPMSG_AUDIO		3
#define IMX_RPMSG_KEY		4
#define IMX_RPMSG_GPIO		5
#define IMX_RPMSG_RTC		6
#define IMX_RPMSG_SENSOR	7
#define IMX_RPMSG_I2C		9
#define IMX_RPMSG_PWM		0xa
#define IMX_RPMSG_ADC		0xb
#define IMX_RPMSG_TTY		0xf1
#define IMX_RPMSG_WDOG		0xf2
#define IMX_RPMSG_CAN		0xf3
#define IMX_RPMSG_SPI		0xf4
#define IMX_RPMSG_DAC		0xf5
/* rpmsg version */
#define IMX_RMPSG_MAJOR		1
#define IMX_RMPSG_MINOR		0

#define CIRC_ADD(idx, size, value)	(((idx) + (value)) & ((size) - 1))

struct imx_rpmsg_head {
	u8 cate;
	u8 major;
	u8 minor;
	u8 type;
	u8 cmd;
	u8 reserved[5];
} __packed;

/* RPMSG_BUF_SIZE - sizeof(struct rpmsg_hdr), max length we can use
 * before writing imx_rpmsg_head. head is not excluded as it usually is
 * part of each message struct. */
#define RPMSG_MAX_PAYLOAD_SIZE (496)
#define IMX_RPMSG_HEAD_SIZE (sizeof(struct imx_rpmsg_head))

#endif /* __LINUX_IMX_RPMSG_H__ */
