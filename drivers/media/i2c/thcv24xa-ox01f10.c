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

struct thcv24xa_ox01f10_setting {
	struct i2c_client *client;
	unsigned char addr_len;
	union {
		u8 addr8;
		u16 addr16;
	};
	u8 val;
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

static int thcv24xa_ox01f10_writes(struct thcv24xa_ox01f10_setting *settings,
				   int nr_settings)
{
	int i;
	int ret;

	for (i = 0; i < nr_settings; i++) {
		switch (settings[i].addr_len) {
		case 8:
			ret = thcv24xa_ox01f10_writeb_addr8(settings[i].client,
							    settings[i].addr8,
							    settings[i].val);
			if (ret)
				return ret;
			break;
		case 16:
			ret = thcv24xa_ox01f10_writeb_addr16(settings[i].client,
							     settings[i].addr16,
							     settings[i].val);
			if (ret)
				return ret;
			break;
		default:
			BUG();
		}
	}

	return 0;
}

static int thcv24xa_ox01f10_setup(struct thcv24xa_ox01f10 *thcv24xa_ox01f10)
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
	struct thcv24xa_ox01f10_setting settings[] = {
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0050, .val = 0x51},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0004, .val = 0x03},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0010, .val = 0x10},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1704, .val = 0x01},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0102, .val = 0x02},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0103, .val = 0x02},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0104, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0105, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0100, .val = 0x03},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x010F, .val = 0x25},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x010A, .val = 0x15},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0031, .val = 0x02},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0032, .val = 0x10},

		// [Tx Register Setting]
		{.client = thcv24xa_ox01f10->tx, .addr_len = 16, .addr16 = 0x00FE, .val = 0x11},

		// [Rx Register Setting only for Pass Through]
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0032, .val = 0x00},

		// [Tx Register Settings]
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xF3, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xF2, .val = 0x22},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xF0, .val = 0x03},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xFF, .val = 0x19},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xF6, .val = 0x15},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xC9, .val = 0x15},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xCA, .val = 0x15},

		// [Tx Register Settings]
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0xFE, .val = 0x21},

		// [PLL Settings for Pass Through mode]
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x0F, .val = 0x01},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x11, .val = 0x1E},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x12, .val = 0x4D},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x13, .val = 0x93},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x14, .val = 0x64},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x15, .val = 0x66},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x16, .val = 0x01},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x00, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x01, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x02, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x55, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x04, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x2B, .val = 0x05},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x27, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x07, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x08, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x09, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x0C, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x0D, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x2D, .val = 0x11},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x2C, .val = 0x01},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x05, .val = 0x01},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x06, .val = 0x01},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x3D, .val = 0x00},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x3E, .val = 0x10},
		{.client = thcv24xa_ox01f10->tx, .addr_len = 8, .addr8 = 0x3F, .val = 0x01},

		// [ Rx Register Settings]
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0010, .val = 0x11},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1010, .val = 0xA1},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1011, .val = 0x05},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1012, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1021, .val = 0x30},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1022, .val = 0x01},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1023, .val = 0x31},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1024, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1025, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1026, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1027, .val = 0x07},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1028, .val = 0x02},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1030, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1100, .val = 0x01},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1101, .val = 0x01},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1102, .val = 0x01},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1600, .val = 0x1A},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1605, .val = 0x29},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1606, .val = 0x44},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x161F, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1609, .val = 0x04},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x160A, .val = 0x07},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x160B, .val = 0x05},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x160D, .val = 0x08},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x160E, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x160F, .val = 0x03},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1610, .val = 0x02},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1611, .val = 0x03},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1612, .val = 0x06},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1013, .val = 0x01},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1703, .val = 0x01},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1704, .val = 0x11},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1003, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x1004, .val = 0x00},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x001B, .val = 0x18},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0032, .val = 0x10},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0040, .val = 0x36},
		{.client = thcv24xa_ox01f10->rx, .addr_len = 16, .addr16 = 0x0041, .val = 0x36},
	};

	return thcv24xa_ox01f10_writes(settings, ARRAY_SIZE(settings));
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

	return thcv24xa_ox01f10_setup(thcv24xa_ox01f10);
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
