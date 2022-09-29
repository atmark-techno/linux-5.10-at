// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * THCV242A-P and THCV241A-P relay driver for OX01F10
 *
 * Copyright (C) 2022 Atmark Techno, Inc. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

static const struct i2c_device_id thcv24xa_ox01f10_id[] = {
	{ "thcv24xa_ox01f10", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, thcv24xa_ox01f10_id);

static const struct of_device_id thcv24xa_ox01f10_of_match[] = {
	{ .compatible = "thcv24xa_ox01f10" },
	{ }
};
MODULE_DEVICE_TABLE(of, thcv24xa_ox01f10_of_match);

struct thcv24xa_ox01f10 {
	struct i2c_client *rx; /* THCV242A-P */
	struct i2c_client *tx; /* THCV241A-P */
};

#define I2C_THCV241A_ADDR 0x51

static int thcv24xa_ox01f10_writeb_addr16(struct i2c_client *client, u16 addr, u8 val)
{
	u8 buf[3];
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = 3,
		.buf = buf
	};
	int ret;

	buf[0] = (addr >> 8) & 0xff;
	buf[1] = addr & 0xff;
	buf[2] = val;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "%s: addr 0x%04x val 0x%02x error %d\n",
			__func__, addr, val, ret);
		if (ret < 0)
			return ret;
		else
			return -EIO;
	}

	return 0;
}

static int thcv24xa_ox01f10_writeb_addr8(struct i2c_client *client, u8 addr, u8 val)
{
	u8 buf[2];
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = 2,
		.buf = buf
	};
	int ret;

	buf[0] = addr;
	buf[1] = val;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "%s: addr 0x%02x val 0x%02x error %d\n",
			__func__, addr, val, ret);
		if (ret < 0)
			return ret;
		else
			return -EIO;
	}

	return 0;
}

static void thcv24xa_ox01f10_setup(struct thcv24xa_ox01f10 *thcv24xa_ox01f10)
{
	// [Settings for Load Function in Version 056 ]
	//  [Welcome Screen] Execution Mode : Detail Mode
	//  [Target Selection] Target Host Environment : CSV Output
	//  [System Configuration] Configuration : 1Camera - 1Host
	//  [Sub-Link Use Setting] Use of Sub-Link : YES
	//  [Sub-Link Use Setting] Sub-Link Protocol : Pass Through
	//  [Tx Selection] Tx : THCV241A-P (Device Address = 0x51
	//                 [7bit address & hexadecimal])
	//  [Rx Selection] Rx : THCV242A-P (Device Address = 0x65
	//                 [7bit address & hexadecimal])
	//  [MIPI CSI-2 Setting for Tx] MIPI bitrate/lane : 500 Mbps
	//   Information : If you have changed MIPI parameters in "MIPI
	//                 Parameter Setting for Rx (MIPI Tx)", you have to set
	//                 them again after Load this file.
	//  [MIPI CSI-2 Setting for Tx] Number of lanes for MIPI from camera
	//                              module to Tx : 2 lane(s)
	//  [MIPI CSI-2 Setting for Tx] Line Start / Line End Code :  Non-Use
	//  [Reference Clock Input Setting] Reference Clock Input Frequency
	//                                  for Tx : 37.125 MHz
	//  [Reference Clock Input Setting] CKO Output : Disable
	//  [V-by-One(R) HS Setting] Number of lanes for V-by-One(R) HS :
	//                           1 lane(s)
	//  [V-by-One(R) HS Setting] Lane of Rx : Lane 0
	//  [V-by-One(R) HS Setting] V-by-One(R) HS format name : MPRF
	//   Information : V-by-One(R) HS rate : 1249.99999851597 Mbps
	//  [MIPI CSI-2 Setting for Rx] Number of lanes for MIPI from Rx to
	//                              Host : 2 lane(s)
	//  [I2C Clock Setting] 2-wire Slave Device Support : YES
	//  [I2C Clock Setting] I2C Clock : 100kHz
	//  [I2C Clock Setting] Remote I2C (Device #0) of (Sub-Link Lane0) :
	//                      7bit Address = 0x36 : Name = "Imager"
	//  [GPIO Setting] Tx GPIO0 : Register GPIO / Output from Tx /
	//                 CMOS (High)
	//  [GPIO Setting] Tx GPIO1 : No Use
	//  [GPIO Setting] Tx GPIO2 : No Use
	//  [GPIO Setting] Tx GPIO3 : No Use
	// [End of Load Settings]

	// [Rx Register Settings]
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0050, 0x51);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0004, 0x03);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0010, 0x10);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1704, 0x01);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0102, 0x02);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0103, 0x02);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0104, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0105, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0100, 0x03);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x010F, 0x25);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x010A, 0x15);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0031, 0x02);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0032, 0x10);

	// [Tx Register Setting]
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->tx, 0x00FE, 0x11);

	// [Rx Register Setting only for Pass Through]
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0032, 0x00);

	// [Tx Register Settings]
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xF3, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xF2, 0x22);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xF0, 0x03);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xFF, 0x19);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xF6, 0x15);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xC9, 0x15);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xCA, 0x15);

	// [Tx Register Settings]
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0xFE, 0x21);

	// [PLL Settings for Pass Through mode]
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x0F, 0x01);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x11, 0x1E);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x12, 0x4D);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x13, 0x93);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x14, 0x64);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x15, 0x66);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x16, 0x01);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x00, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x01, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x02, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x55, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x04, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x2B, 0x05);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x27, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x07, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x08, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x09, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x0C, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x0D, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x2D, 0x11);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x2C, 0x01);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x05, 0x01);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x06, 0x01);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x3D, 0x00);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x3E, 0x10);
	thcv24xa_ox01f10_writeb_addr8(thcv24xa_ox01f10->tx, 0x3F, 0x01);

	// [ Rx Register Settings]
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0010, 0x11);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1010, 0xA1);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1011, 0x05);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1012, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1021, 0x30);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1022, 0x01);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1023, 0x31);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1024, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1025, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1026, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1027, 0x07);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1028, 0x02);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1030, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1100, 0x01);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1101, 0x01);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1102, 0x01);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1600, 0x1A);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1605, 0x29);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1606, 0x44);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x161F, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1609, 0x04);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x160A, 0x07);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x160B, 0x05);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x160D, 0x08);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x160E, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x160F, 0x03);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1610, 0x02);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1611, 0x03);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1612, 0x06);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1013, 0x01);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1703, 0x01);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1704, 0x11);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1003, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x1004, 0x00);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x001B, 0x18);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0032, 0x10);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0040, 0x36);
	thcv24xa_ox01f10_writeb_addr16(thcv24xa_ox01f10->rx, 0x0041, 0x36);
}

static int thcv24xa_ox01f10_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct thcv24xa_ox01f10 *thcv24xa_ox01f10;
	struct device *dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"Doesn't support required functionality\n");
		return -EOPNOTSUPP;
	}

	thcv24xa_ox01f10 = devm_kzalloc(dev, sizeof(struct thcv24xa_ox01f10), GFP_KERNEL);
	if (!thcv24xa_ox01f10)
		return -ENOMEM;

	thcv24xa_ox01f10->rx = client;
	i2c_set_clientdata(client, thcv24xa_ox01f10);

	thcv24xa_ox01f10->tx = devm_i2c_new_dummy_device(dev, client->adapter,
							 I2C_THCV241A_ADDR);
	if (IS_ERR(thcv24xa_ox01f10->tx)) {
		dev_err(&client->dev, "Failed to allocate I2C device\n");
		return PTR_ERR(thcv24xa_ox01f10->tx);
	}

	/*
	 * FIXME: There is no basis for this wait. But i2c access
	 * fails without wait.
	 */
	msleep(200);

	thcv24xa_ox01f10_setup(thcv24xa_ox01f10);

	return 0;
}

static struct i2c_driver thcv24xa_ox01f10_driver = {
	.driver = {
		.name		= "thcv24xa-ox01f10",
		.of_match_table	= of_match_ptr(thcv24xa_ox01f10_of_match),
	},
	.probe		= thcv24xa_ox01f10_probe,
	.id_table	= thcv24xa_ox01f10_id,
};

module_i2c_driver(thcv24xa_ox01f10_driver);

MODULE_AUTHOR("Daisuke Mizobuchi <mizo@atmark-techno.com>");
MODULE_DESCRIPTION("THCV242A-P and THCV241A-P relay driver for OX01F10");
MODULE_LICENSE("GPL");
