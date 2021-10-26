/**
 * @file       ox01f10.c
 * @brief      OX01F10 sensor driver
 * @date       2021/07/26
 * @par        Copyright
 *             2021- Shikino High-Tech Co.,LTD. All rights reserved.
 *
 *             This program is free software; you can redistribute it and/or modify it
 *             under the terms and conditions of the GNU General Public License,
 *             version 2, as published by the Free Software Foundation.
 *
 *             This program is distributed in the hope it will be useful, but WITHOUT
 *             ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *             FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *             more details.
 *
 *             You should have received a copy of the GNU General Public License
 *             along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/camera_common.h>
#include "./../../../nvidia/drivers/media/platform/tegra/camera/camera_gpio.h"
#include "ox01f10_mode_tbls.h"

#define DEBUG_PRINTK
#ifndef DEBUG_PRINTK
#define debug_printk(s , ... )
#else
#define debug_printk printk
#endif

#define CAMERA_CID_BASE	(V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define CAMERA_CID_USER_HDR_EN (CAMERA_CID_BASE + 81)
#define CAMERA_CID_AECAGC_EN (CAMERA_CID_BASE + 82)
#define CAMERA_CID_AECAGC_TARGET (CAMERA_CID_BASE + 83)
#define CAMERA_CID_EXP_LONG (CAMERA_CID_BASE + 84)
#define CAMERA_CID_EXP_SHORT (CAMERA_CID_BASE + 85)
#define CAMERA_CID_EXP_VERY_SHORT (CAMERA_CID_BASE + 86)
#define CAMERA_CID_USER_GAIN (CAMERA_CID_BASE + 87)
#define CAMERA_CID_AWB_EN (CAMERA_CID_BASE + 88)
#define CAMERA_CID_AWB_GAIN_R (CAMERA_CID_BASE + 89)
#define CAMERA_CID_AWB_GAIN_B (CAMERA_CID_BASE + 90)
#define CAMERA_CID_Y_OFFSET (CAMERA_CID_BASE + 91)
#define CAMERA_CID_Y_GAIN (CAMERA_CID_BASE + 92)
#define CAMERA_CID_USER_SATURATION (CAMERA_CID_BASE + 93)
#define CAMERA_CID_BAND_CONTROL (CAMERA_CID_BASE + 94)
#define CAMERA_CID_USER_MIRROR (CAMERA_CID_BASE + 95)
#define CAMERA_CID_USER_FLIP (CAMERA_CID_BASE + 96)

#define OX01F10_MIN_AEAGC_TARGET		0
#define OX01F10_MAX_AEAGC_TARGET		64
#define OX01F10_DEFAULT_AEAGC_TARGET	24

#define OX01F10_MIN_EXP					1
#define OX01F10_MAX_EXP					1069
#define OX01F10_DEFAULT_EXP_L			184
#define OX01F10_DEFAULT_EXP_S			156
#define OX01F10_DEFAULT_EXP_VS			7

#define OX01F10_MIN_GAIN				16
#define OX01F10_MAX_GAIN				248
#define OX01F10_DEFAULT_GAIN			16

#define OX01F10_MIN_WB_GAIN				0
#define OX01F10_MAX_WB_GAIN				4095
#define OX01F10_DEFAULT_WB_GAIN_R		612
#define OX01F10_DEFAULT_WB_GAIN_B		374

#define OX01F10_MIN_Y_OFFSET			0
#define OX01F10_MAX_Y_OFFSET			255
#define OX01F10_CENTER_Y_OFFSET			128
#define OX01F10_DEFAULT_Y_OFFSET		OX01F10_CENTER_Y_OFFSET

#define OX01F10_MIN_Y_GAIN				0
#define OX01F10_MAX_Y_GAIN				511
#define OX01F10_CENTER_Y_GAIN			256
#define OX01F10_DEFAULT_Y_GAIN			OX01F10_CENTER_Y_GAIN

#define OX01F10_MIN_SATURATION			0
#define OX01F10_MAX_SATURATION			255
#define OX01F10_DEFAULT_SATURATION		48

#define REG_ADDR_AEAGC_L_TARGET		0xB219
#define REG_ADDR_AEAGC_S_TARGET		0xB21A
#define REG_ADDR_AEC_L_EXP_LO		0xB149
#define REG_ADDR_AEC_L_EXP_HI		0xB148
#define REG_ADDR_AEC_L_EXP_LO		0xB149
#define REG_ADDR_AEC_S_EXP_HI		0xB14A
#define REG_ADDR_AEC_S_EXP_LO		0xB14B
#define REG_ADDR_AEC_VS_EXP_HI		0xB14C
#define REG_ADDR_AEC_VS_EXP_LO		0xB14D
#define REG_ADDR_AEC_L_GAIN_HI		0xB154
#define REG_ADDR_AEC_L_GAIN_LO		0xB155
#define REG_ADDR_AEC_S_GAIN_HI		0xB156
#define REG_ADDR_AEC_S_GAIN_LO		0xB157
#define REG_ADDR_AEC_VS_GAIN_HI		0xB158
#define REG_ADDR_AEC_VS_GAIN_LO		0xB159
#define REG_ADDR_AEAGC_ENABLE		0xB140
#define REG_ADDR_ENABLE_MANUAL_GAIN	0xB141
#define REG_ADDR_ISP_SETTING_9		0x5009
#define REG_ADDR_ENABLE_MANUAL_WB	0xB36D
#define REG_ADDR_WB_L_GAIN_R_HI	 	0xB276
#define REG_ADDR_WB_L_GAIN_R_LO	 	0xB277
#define REG_ADDR_WB_M_GAIN_R_HI	 	0xB27C
#define REG_ADDR_WB_M_GAIN_R_LO	 	0xB27D
#define REG_ADDR_WB_S_GAIN_R_HI	 	0xB282
#define REG_ADDR_WB_S_GAIN_R_LO	 	0xB283
#define REG_ADDR_WB_L_GAIN_B_HI	 	0xB272
#define REG_ADDR_WB_L_GAIN_B_LO	 	0xB273
#define REG_ADDR_WB_M_GAIN_B_HI	 	0xB278
#define REG_ADDR_WB_M_GAIN_B_LO	 	0xB279
#define REG_ADDR_WB_S_GAIN_B_HI	 	0xB27E
#define REG_ADDR_WB_S_GAIN_B_LO	 	0xB27F
#define REG_ADDR_Y_GAIN_HI			0xA800
#define REG_ADDR_Y_GAIN_LO			0xA801
#define REG_ADDR_SATURATION_1		0xB6D4
#define REG_ADDR_SATURATION_2		0xB6D5
#define REG_ADDR_SATURATION_3		0xB6D6
#define REG_ADDR_SATURATION_4		0xB6D7
#define REG_ADDR_SATURATION_5		0xB6D8
#define REG_ADDR_SATURATION_6		0xB6D9
#define REG_ADDR_SATURATION_7		0xB6DA
#define REG_ADDR_SATURATION_8		0xB6DB
#define REG_ADDR_Y_OFFSET_HI		0xA808
#define REG_ADDR_Y_OFFSET_LO		0xA809
#define REG_ADDR_BAND_ENABLE		0xB210
#define REG_ADDR_BAND_MODE			0xB211
#define REG_ADDR_RAW_TARGET_L		0xB218
#define REG_ADDR_TIMING_CTRL		0x3820

#define MIRROR_ENABLE_BIT			0x02
#define FLIP_ENABLE_BIT				0x0C
#define HDR_MODE_LSVS				0x00
#define HDR_MODE_LS					0x01
#define HDR_MODE_OFF				0x05
#define BAND_60						0x01
#define DISABLE_AEAGC				0x01
#define ENABLE_AEAGC				0x00
#define DISABLE_MANUAL_GAIN			0x00
#define ENABLE_MANUAL_GAIN			0x01
#define DISABLE_MANUAL_WB			0x00
#define ENABLE_MANUAL_WB			0x01
#define BAND_DISABLE				0x00
#define BAND_ENABLE					0x03
#define BAND_AUTO_DISABLE			0x00
#define BAND_AUTO_ENABLE			0x01
#define BAND_50						0x02
#define BAND_60						0x01

struct ox01f10 {
	struct camera_common_power_rail power;
	int numctrls;
	struct v4l2_ctrl_handler ctrl_handler;
	struct i2c_client *i2c_client;
	struct v4l2_subdev *subdev;
	struct media_pad pad;

	struct regmap *regmap;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;
	struct camera_common_frmfmt *cam_frmfmt;

	struct v4l2_ctrl *ctrls[];
};

enum hdr_state {
	HDR_OFF,
	HDR_ON_1,
	HDR_ON_2
};

static const s64 hdr_ctrl_qmenu[] = {
	HDR_OFF,
	HDR_ON_1,
	HDR_ON_2
};

enum band_state {
	BAND_OFF,
	BAND_ON_50,
	BAND_ON_60,
	BAND_ON_AUTO
};

static const s64 band_ctrl_qmenu[] = {
	BAND_OFF,
	BAND_ON_50,
	BAND_ON_60,
	BAND_ON_AUTO
};

static int ox01f10_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int ox01f10_s_ctrl(struct v4l2_ctrl *ctrl);
static int ox01f10_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param);
static void ox01f10_reset_gpio(unsigned int gpio, int value);
static int ox01f10_set_aeagc_target(struct ox01f10 *priv, s32 val);
static int ox01f10_set_exp_long(struct ox01f10 *priv, s32 val);
static int ox01f10_set_exp_short(struct ox01f10 *priv, s32 val);
static int ox01f10_set_exp_very_short(struct ox01f10 *priv, s32 val);
static int ox01f10_set_gain(struct ox01f10 *priv, s32 val);
static int ox01f10_set_aeagc(struct ox01f10 *priv, s32 val);
static int ox01f10_set_hdr(struct ox01f10 *priv, s32 val);
static int ox01f10_set_awb_en(struct ox01f10 *priv, s32 val);
static int ox01f10_set_awb_gain_r(struct ox01f10 *priv, s32 val);
static int ox01f10_set_awb_gain_b(struct ox01f10 *priv, s32 val);
static int ox01f10_set_y_gain(struct ox01f10 *priv, s32 val);
static int ox01f10_set_saturation(struct ox01f10 *priv, s32 val);
static int ox01f10_set_y_offset(struct ox01f10 *priv, s32 val);
static int ox01f10_set_band(struct ox01f10 *priv, s32 val);
static int ox01f10_set_mirror(struct ox01f10 *priv, s32 val);
static int ox01f10_set_flip(struct ox01f10 *priv, s32 val);
static inline int ox01f10_read_reg(struct ox01f10 *priv, u16 addr, u8 *val);
static int ox01f10_write_reg(struct ox01f10 *priv, u16 addr, u8 val);
static int ox01f10_write_table(struct ox01f10 *priv, const ox01f10_reg table[]);
static int ox01f10_regmap_util_write_table_8(struct regmap *regmap,
											 const struct reg_8 table[],
											 const struct reg_8 override_list[],
											 int num_override_regs, u16 wait_ms_addr, u16 end_addr);


static const struct v4l2_ctrl_ops ox01f10_ctrl_ops = {
	.g_volatile_ctrl = ox01f10_g_volatile_ctrl,
	.s_ctrl = ox01f10_s_ctrl,
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_USER_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(hdr_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = HDR_ON_2,
		.qmenu_int = hdr_ctrl_qmenu,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_AECAGC_EN,
		.name = "AE/AGC enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = SWITCH_ON,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_AECAGC_TARGET,
		.name = "AE/AGC target",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_AEAGC_TARGET,
		.max = OX01F10_MAX_AEAGC_TARGET,
		.def = OX01F10_DEFAULT_AEAGC_TARGET,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_EXP_LONG,
		.name = "Exposure(LONG)",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_EXP,
		.max = OX01F10_MAX_EXP,
		.def = OX01F10_DEFAULT_EXP_L,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_EXP_SHORT,
		.name = "Exposure(SHORT)",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_EXP,
		.max = OX01F10_MAX_EXP,
		.def = OX01F10_DEFAULT_EXP_S,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_EXP_VERY_SHORT,
		.name = "Exposure(VERY SHORT)",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_EXP,
		.max = OX01F10_MAX_EXP,
		.def = OX01F10_DEFAULT_EXP_VS,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_USER_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_GAIN,
		.max = OX01F10_MAX_GAIN,
		.def = OX01F10_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_AWB_EN,
		.name = "AWB enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = SWITCH_ON,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_AWB_GAIN_R,
		.name = "WB Gain R",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_WB_GAIN,
		.max = OX01F10_MAX_WB_GAIN,
		.def = OX01F10_DEFAULT_WB_GAIN_R,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_AWB_GAIN_B,
		.name = "WB Gain B",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_WB_GAIN,
		.max = OX01F10_MAX_WB_GAIN,
		.def = OX01F10_DEFAULT_WB_GAIN_B,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_Y_OFFSET,
		.name = "Brightness",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_Y_OFFSET,
		.max = OX01F10_MAX_Y_OFFSET,
		.def = OX01F10_DEFAULT_Y_OFFSET,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_Y_GAIN,
		.name = "Contrast",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_Y_GAIN,
		.max = OX01F10_MAX_Y_GAIN,
		.def = OX01F10_DEFAULT_Y_GAIN,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_USER_SATURATION,
		.name = "Saturation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OX01F10_MIN_SATURATION,
		.max = OX01F10_MAX_SATURATION,
		.def = OX01F10_DEFAULT_SATURATION,
		.step = 1,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_BAND_CONTROL,
		.name = "Flicker",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(band_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = BAND_OFF,
		.qmenu_int = band_ctrl_qmenu,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_USER_MIRROR,
		.name = "Mirror",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = SWITCH_OFF,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ox01f10_ctrl_ops,
		.id = CAMERA_CID_USER_FLIP,
		.name = "Flip",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = SWITCH_OFF,
		.qmenu_int = switch_ctrl_qmenu,
	},
};

/**
 * @brief       Set AE/AGC target
 * @param[in]   *priv    : ox01f10ドライバ情報
 * @param[in]   val      : 設定値
 * @return      err      : エラーコード
 */
static int ox01f10_set_aeagc_target(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned char w_val = (unsigned char)val + 32;
	
	err = ox01f10_write_reg(priv, REG_ADDR_AEAGC_L_TARGET, w_val);
	if (err)
		goto fail;
	err = ox01f10_write_reg(priv, REG_ADDR_AEAGC_S_TARGET, w_val);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_aeagc_target w_val=0x%02X\n", w_val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_aeagc_target write reg error\n");
	return err;
}

/**
 * @brief       Set exposure(LONG)
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_exp_long(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned short w_val = (unsigned short)val;
	static ox01f10_reg exp_long_reg[3] = {
		{REG_ADDR_AEC_L_EXP_HI, 0x00},
		{REG_ADDR_AEC_L_EXP_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	exp_long_reg[0].val = (unsigned char)((w_val >> 8) & 0x00FF);
	exp_long_reg[1].val = (unsigned char)(w_val & 0x00FF);
	err = ox01f10_write_table(priv, exp_long_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_exp_long w_val=0x%04X\n", w_val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_exp_long write reg error\n");
	return err;
}

/**
 * @brief       Set exposure(SHORT)
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_exp_short(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned short w_val = (unsigned short)val;
	static ox01f10_reg exp_short_reg[3] = {
		{REG_ADDR_AEC_S_EXP_HI, 0x00},
		{REG_ADDR_AEC_S_EXP_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	exp_short_reg[0].val = (unsigned char)((w_val >> 8) & 0x00FF);
	exp_short_reg[1].val = (unsigned char)(w_val & 0x00FF);
	err = ox01f10_write_table(priv, exp_short_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_exp_short w_val=0x%04X\n", w_val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_exp_short write reg error\n");
	return err;
}

/**
 * @brief       Set exposure(VERY SHORT)
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_exp_very_short(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned short w_val = (unsigned short)val;
	static ox01f10_reg exp_very_short_reg[3] = {
		{REG_ADDR_AEC_VS_EXP_HI, 0x00},
		{REG_ADDR_AEC_VS_EXP_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	exp_very_short_reg[0].val = (unsigned char)((w_val >> 8) & 0x00FF);
	exp_very_short_reg[1].val = (unsigned char)(w_val & 0x00FF);
	err = ox01f10_write_table(priv, exp_very_short_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_exp_very_short w_val=0x%04X\n", w_val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_exp_very_short write reg error\n");
	return err;
}

/**
 * @brief       Set gain
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_gain(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned char w_val = (unsigned char)val;
	static ox01f10_reg gain_reg[7] = {
		{REG_ADDR_AEC_L_GAIN_HI, 0x00},
		{REG_ADDR_AEC_L_GAIN_LO, 0x00},
		{REG_ADDR_AEC_S_GAIN_HI, 0x00},
		{REG_ADDR_AEC_S_GAIN_LO, 0x00},
		{REG_ADDR_AEC_VS_GAIN_HI, 0x00},
		{REG_ADDR_AEC_VS_GAIN_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	gain_reg[1].val = w_val;
	gain_reg[3].val = w_val;
	gain_reg[5].val = w_val;
	err = ox01f10_write_table(priv, gain_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_gain w_val=%02X\n", w_val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_gain write reg error\n");
	return err;
}

/**
 * @brief       Set AE/AGC
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_aeagc(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned char w_val1 = 0;
	unsigned char w_val2 = 0;
	
	if(val == SWITCH_ON)
	{
		w_val1 = ENABLE_AEAGC;
		w_val2 = DISABLE_MANUAL_GAIN;
	}
	else
	{
		w_val1 = DISABLE_AEAGC;
		w_val2 = ENABLE_MANUAL_GAIN;
	}
	
	err = ox01f10_write_reg(priv, REG_ADDR_AEAGC_ENABLE, w_val1);
	if (err)
		goto fail;
	err = ox01f10_write_reg(priv, REG_ADDR_ENABLE_MANUAL_GAIN, w_val2);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_aeagc \n");
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_aeagc write reg error\n");
	return err;
}

/**
 * @brief       Set HDR
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_hdr(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned char w_val = 0;
	
	if(val == HDR_ON_2)
	{
		w_val = HDR_MODE_LSVS;
	}
	else if(val == HDR_ON_1)
	{
		w_val = HDR_MODE_LS;
	}
	else
	{
		w_val = HDR_MODE_OFF;
	}
	
	err = ox01f10_write_reg(priv, REG_ADDR_ISP_SETTING_9, w_val);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_hdr \n");
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_hdr write reg error\n");
	return err;
}

/**
 * @brief       Set AWB
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_awb_en(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned char w_val = 0;
	
	if(val == SWITCH_ON)
	{
		w_val = DISABLE_MANUAL_WB;
	}
	else
	{
		w_val = ENABLE_MANUAL_WB;
	}
	
	err = ox01f10_write_reg(priv, REG_ADDR_ENABLE_MANUAL_WB, w_val);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_awb_en \n");
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_awb_en write reg error\n");
	return err;
}

/**
 * @brief       Set AWB red gain
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_awb_gain_r(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned short w_val = (unsigned short)val;
	static ox01f10_reg awb_gain_r_reg[7] = {
		{REG_ADDR_WB_L_GAIN_R_HI, 0x00},
		{REG_ADDR_WB_L_GAIN_R_LO, 0x00},
		{REG_ADDR_WB_M_GAIN_R_HI, 0x00},
		{REG_ADDR_WB_M_GAIN_R_LO, 0x00},
		{REG_ADDR_WB_S_GAIN_R_HI, 0x00},
		{REG_ADDR_WB_S_GAIN_R_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	awb_gain_r_reg[0].val = (unsigned char)((w_val >> 8) & 0x00FF);
	awb_gain_r_reg[1].val = (unsigned char)(w_val & 0x00FF);
	awb_gain_r_reg[2].val = (unsigned char)((w_val >> 8) & 0x00FF);
	awb_gain_r_reg[3].val = (unsigned char)(w_val & 0x00FF);
	awb_gain_r_reg[4].val = (unsigned char)((w_val >> 8) & 0x00FF);
	awb_gain_r_reg[5].val = (unsigned char)(w_val & 0x00FF);
	err = ox01f10_write_table(priv, awb_gain_r_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_awb_gain_r w_val=%04X\n", w_val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_awb_gain_r write reg error\n");
	return err;
}

/**
 * @brief       Set AWB blue gain
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_awb_gain_b(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned short w_val = (unsigned short)val;
	static ox01f10_reg awb_gain_b_reg[7] = {
		{REG_ADDR_WB_L_GAIN_B_HI, 0x00},
		{REG_ADDR_WB_L_GAIN_B_LO, 0x00},
		{REG_ADDR_WB_M_GAIN_B_HI, 0x00},
		{REG_ADDR_WB_M_GAIN_B_LO, 0x00},
		{REG_ADDR_WB_S_GAIN_B_HI, 0x00},
		{REG_ADDR_WB_S_GAIN_B_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	awb_gain_b_reg[0].val = (unsigned char)((w_val >> 8) & 0x00FF);
	awb_gain_b_reg[1].val = (unsigned char)(w_val & 0x00FF);
	awb_gain_b_reg[2].val = (unsigned char)((w_val >> 8) & 0x00FF);
	awb_gain_b_reg[3].val = (unsigned char)(w_val & 0x00FF);
	awb_gain_b_reg[4].val = (unsigned char)((w_val >> 8) & 0x00FF);
	awb_gain_b_reg[5].val = (unsigned char)(w_val & 0x00FF);
	err = ox01f10_write_table(priv, awb_gain_b_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_awb_gain_b w_val=%04X\n", w_val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_awb_gain_b write reg error\n");
	return err;
}

/**
 * @brief       Set contrast(Y Gain)
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_y_gain(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	static ox01f10_reg y_gain_reg[3] = {
		{REG_ADDR_Y_GAIN_HI, 0x00},
		{REG_ADDR_Y_GAIN_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	y_gain_reg[0].val = (unsigned char)((val >> 8) & 0x0001);
	y_gain_reg[1].val = (unsigned char)(val & 0x00FF);
	err = ox01f10_write_table(priv, y_gain_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_y_gain val=%d\n", val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_y_gain write reg error\n");
	return err;
}

/**
 * @brief       Set saturation
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_saturation(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	short w_val = 0;
	static ox01f10_reg saturation_reg[9] = {
		{REG_ADDR_SATURATION_1, 0x00},
		{REG_ADDR_SATURATION_2, 0x00},
		{REG_ADDR_SATURATION_3, 0x00},
		{REG_ADDR_SATURATION_4, 0x00},
		{REG_ADDR_SATURATION_5, 0x00},
		{REG_ADDR_SATURATION_6, 0x00},
		{REG_ADDR_SATURATION_7, 0x00},
		{REG_ADDR_SATURATION_8, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};
	
	w_val = (short)(val & 0xFF);
	saturation_reg[0].val = (unsigned char)(w_val & 0x00FF);
	saturation_reg[1].val = (unsigned char)(w_val & 0x00FF);
	w_val = (short)(val & 0xFF) - 8;
	if (w_val < 0) w_val = 0;
	saturation_reg[2].val = (unsigned char)(w_val & 0x00FF);
	saturation_reg[3].val = (unsigned char)(w_val & 0x00FF);
	w_val = (short)(val & 0xFF) - 22;
	if (w_val < 0) w_val = 0;
	saturation_reg[4].val = (unsigned char)(w_val & 0x00FF);
	saturation_reg[5].val = (unsigned char)(w_val & 0x00FF);
	w_val = (short)(val & 0xFF) - 32;
	if (w_val < 0) w_val = 0;
	saturation_reg[6].val = (unsigned char)(w_val & 0x00FF);
	saturation_reg[7].val = (unsigned char)(w_val & 0x00FF);
	err = ox01f10_write_table(priv, saturation_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_saturation val=%d\n", val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_saturation write reg error\n");
	return err;
}

/**
 * @brief       Set brightness(Y Offset)
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_y_offset(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	static ox01f10_reg y_offset_reg[3] = {
		{REG_ADDR_Y_OFFSET_HI, 0x00},
		{REG_ADDR_Y_OFFSET_LO, 0x00},
		{OX01F10_TABLE_END, 0x00}
	};

	if(val >= OX01F10_CENTER_Y_OFFSET)
	{
		val -= OX01F10_CENTER_Y_OFFSET;
	}
	else
	{
		val += OX01F10_CENTER_Y_OFFSET;
	}
	
	y_offset_reg[0].val = (unsigned char)((val >> 4) & 0x0F);
	y_offset_reg[1].val = (unsigned char)((val << 4) & 0xF0);
	err = ox01f10_write_table(priv, y_offset_reg);
	if (err)
		goto fail;
	
	dev_info(dev, "ox01f10_set_y_offset val=%d\n", val);
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_y_offset write reg error\n");
	return err;
}

/**
 * @brief       Set flicker correction
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_band(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	
	if(val == BAND_ON_50)
	{
		err = ox01f10_write_reg(priv, REG_ADDR_RAW_TARGET_L, BAND_AUTO_DISABLE);
		if (err)
			goto fail;
		err = ox01f10_write_reg(priv, REG_ADDR_BAND_ENABLE, BAND_ENABLE);
		if (err)
			goto fail;
		err = ox01f10_write_reg(priv, REG_ADDR_BAND_MODE, BAND_50);
		if (err)
			goto fail;
	}
	else if(val == BAND_ON_60)
	{
		err = ox01f10_write_reg(priv, REG_ADDR_RAW_TARGET_L, BAND_AUTO_DISABLE);
		if (err)
			goto fail;
		err = ox01f10_write_reg(priv, REG_ADDR_BAND_ENABLE, BAND_ENABLE);
		if (err)
			goto fail;
		err = ox01f10_write_reg(priv, REG_ADDR_BAND_MODE, BAND_60);
		if (err)
			goto fail;
	}
	else if(val == BAND_ON_AUTO)
	{
		err = ox01f10_write_reg(priv, REG_ADDR_RAW_TARGET_L, BAND_AUTO_ENABLE);
		if (err)
			goto fail;
		err = ox01f10_write_reg(priv, REG_ADDR_BAND_ENABLE, BAND_ENABLE);
		if (err)
			goto fail;
	}
	else
	{
		err = ox01f10_write_reg(priv, REG_ADDR_BAND_ENABLE, BAND_DISABLE);
		if (err)
			goto fail;
	}
	
	dev_info(dev, "ox01f10_set_band \n");
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_band write reg error\n");
	return err;
}

/**
 * @brief       Set mirror
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_mirror(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned char w_val = 0;
	
	ox01f10_read_reg(priv, REG_ADDR_TIMING_CTRL, &w_val);
	
	if(val == SWITCH_ON)
	{
		err = ox01f10_write_reg(priv, REG_ADDR_TIMING_CTRL, (w_val | MIRROR_ENABLE_BIT));
		if (err)
			goto fail;
	}
	else
	{
		err = ox01f10_write_reg(priv, REG_ADDR_TIMING_CTRL, (w_val & ~(MIRROR_ENABLE_BIT)));
		if (err)
			goto fail;
	}
	
	dev_info(dev, "ox01f10_set_mirror \n");
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_mirror write reg error\n");
	return err;
}

/**
 * @brief       Set flip
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   val      : Set value
 * @return      err      : Error code
 */
static int ox01f10_set_flip(struct ox01f10 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;
	unsigned char w_val = 0;
	
	ox01f10_read_reg(priv, REG_ADDR_TIMING_CTRL, &w_val);
	
	if(val == SWITCH_ON)
	{
		err = ox01f10_write_reg(priv, REG_ADDR_TIMING_CTRL, (w_val | FLIP_ENABLE_BIT));
		if (err)
			goto fail;
	}
	else
	{
		err = ox01f10_write_reg(priv, REG_ADDR_TIMING_CTRL, (w_val & ~(FLIP_ENABLE_BIT)));
		if (err)
			goto fail;
	}
	
	dev_info(dev, "ox01f10_set_flip \n");
	return 0;
	
fail:
	dev_err(dev, "ox01f10_set_flip write reg error\n");
	return err;
}

/**
 * @brief       Get parameter
 * @param[in]   *ctrl    : V4L2 control info
 * @return      err      : Error code
 */
static int ox01f10_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ox01f10 *priv =
		container_of(ctrl->handler, struct ox01f10, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	return err;
}

/**
 * @brief       Set parameter
 * @param[in]   *ctrl    : V4L2 control info
 * @return      err      : Error code
 */
static int ox01f10_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ox01f10 *priv =
		container_of(ctrl->handler, struct ox01f10, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case CAMERA_CID_AECAGC_TARGET:
		err = ox01f10_set_aeagc_target(priv, ctrl->val);
		break;
	case CAMERA_CID_EXP_LONG:
		err = ox01f10_set_exp_long(priv, ctrl->val);
		break;
	case CAMERA_CID_EXP_SHORT:
		err = ox01f10_set_exp_short(priv, ctrl->val);
		break;
	case CAMERA_CID_EXP_VERY_SHORT:
		err = ox01f10_set_exp_very_short(priv, ctrl->val);
		break;
	case CAMERA_CID_USER_GAIN:
		err = ox01f10_set_gain(priv, ctrl->val);
		break;
	case CAMERA_CID_AECAGC_EN:
		err = ox01f10_set_aeagc(priv, ctrl->val);
		break;
	case CAMERA_CID_USER_HDR_EN:
		err = ox01f10_set_hdr(priv, ctrl->val);
		break;
	case CAMERA_CID_AWB_EN:
		err = ox01f10_set_awb_en(priv, ctrl->val);
		break;
	case CAMERA_CID_AWB_GAIN_R:
		err = ox01f10_set_awb_gain_r(priv, ctrl->val);
		break;
	case CAMERA_CID_AWB_GAIN_B:
		err = ox01f10_set_awb_gain_b(priv, ctrl->val);
		break;
	case CAMERA_CID_Y_GAIN:
		err = ox01f10_set_y_gain(priv, ctrl->val);
		break;
	case CAMERA_CID_USER_SATURATION:
		err = ox01f10_set_saturation(priv, ctrl->val);
		break;
	case CAMERA_CID_Y_OFFSET:
		err = ox01f10_set_y_offset(priv, ctrl->val);
		break;
	case CAMERA_CID_BAND_CONTROL:
		err = ox01f10_set_band(priv, ctrl->val);
		break;
	case CAMERA_CID_USER_MIRROR:
		err = ox01f10_set_mirror(priv, ctrl->val);
		break;
	case CAMERA_CID_USER_FLIP:
		err = ox01f10_set_flip(priv, ctrl->val);
		break;
	default:
		dev_err(dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

/**
 * @brief       Read register
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   addr     : Address
 * @param[out]  *val     : Read value
 * @return      err      : Error code
 */
static inline int ox01f10_read_reg(struct ox01f10 *priv, u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

/**
 * @brief       Write register
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   addr     : Address
 * @param[in]   val      : Write value
 * @return      err      : Error code
 */
static int ox01f10_write_reg(struct ox01f10 *priv, u16 addr, u8 val)
{
	int err;
	struct device *dev = &priv->i2c_client->dev;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

/**
* @brief        Write register(for Register table)
 * @param[in]   *priv    : ox01f10 driver info
 * @param[in]   table[]  : Register table
 * @return      err      : Error code
 */
static int ox01f10_write_table(struct ox01f10 *priv, const ox01f10_reg table[])
{
	return ox01f10_regmap_util_write_table_8(priv->regmap,
											 table,
											 NULL, 0,
											 OX01F10_TABLE_WAIT_MS,
											 OX01F10_TABLE_END);
}


/**
* @brief        Write register table
 * @param[in]   *regmap              : Register map info
 * @param[in]   table[]              : Register table
 * @param[in]   override_list[]      : Register table over write list
 * @param[in]   num_override_regs    : Register table over write list number
 * @param[in]   wait_ms_addr         : Wait Address
 * @param[in]   end_addr             : Termination Address
 * @return      err                  : Error code
 */
static int ox01f10_regmap_util_write_table_8(struct regmap *regmap,
									  const struct reg_8 table[],
									  const struct reg_8 override_list[],
									  int num_override_regs, u16 wait_ms_addr, u16 end_addr)
{
	int err = 0;
	const struct reg_8 *next;
	int i;
	u8 val;

	int range_start = -1;
	int range_count = 0;
	/* bug 200048392 -
	 * the vi i2c cannot take a FIFO buffer bigger than 16 bytes
	 */
	u8 range_vals[16];
	int max_range_vals = ARRAY_SIZE(range_vals);

	for (next = table;; next++) {
		/* If we have a range open and */
		/* either the address doesn't match */
		/* or the temporary storage is full, flush */
		if  ((next->addr != range_start + range_count) ||
			 (next->addr == end_addr) ||
			 (next->addr == wait_ms_addr) ||
			 (range_count == max_range_vals)) {

			if (range_count == 1) {
				err =
					regmap_write(regmap, range_start,
						 range_vals[0]);
			} else if (range_count > 1) {
				err =
					regmap_bulk_write(regmap, range_start,
							  &range_vals[0],
							  range_count);
			} else {
				err = 0;
			}

			if (err) {
				pr_err("%s:regmap_util_write_table:%d",
					   __func__, err);
				return err;
			}

			range_start = -1;
			range_count = 0;

			/* Handle special address values */
			if (next->addr == end_addr)
				break;

			if (next->addr == wait_ms_addr) {
				usleep_range(next->val * 1000, next->val * 1000 + 500);
				continue;
			}
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list			   */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (range_start == -1)
			range_start = next->addr;

		range_vals[range_count++] = val;
	}
	return 0;
}

/**
 * @brief       Power On
 * @param[in]   *s_data  : Camera control info
 * @return      err      : Error code
 */
static int ox01f10_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct ox01f10 *priv = (struct ox01f10 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	if (!priv || !priv->pdata)
		return -EINVAL;
	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			dev_err(&priv->i2c_client->dev,"%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}
	
	/* XSHUTDOWN HI */
	ox01f10_reset_gpio(priv->pdata->pwdn_gpio, 1);	

	pw->state = SWITCH_ON;
	return 0;
}

/**
 * @brief       Power Off
 * @param[in]   *priv    : ox01f10 driver info
 * @return      err      : Error code
 */
static int ox01f10_power_off(struct ox01f10 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	if (!priv || !priv->pdata)
		return -EINVAL;
	
	if (!pw)
		return -EFAULT;
	
	if (unlikely(!pw))
		return -EFAULT;

	if (priv->pdata->use_cam_gpio)
		cam_gpio_deregister(&priv->i2c_client->dev, pw->pwdn_gpio);
	else {
		gpio_free(pw->pwdn_gpio);
	}

	return 0;
}

/**
 * @brief       Get power info
 * @param[in]   *priv    : ox01f10 driver info
 * @return      err      : Error code
 */
static int ox01f10_power_get(struct ox01f10 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	const char *mclk_name = NULL;
	const char *parentclk_name = NULL;
	struct clk *parent;
	int err = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;

	if(priv->pdata->mclk_name) {
		pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(&priv->i2c_client->dev, "unable to get clock %s\n",
				mclk_name);
			return PTR_ERR(pw->mclk);
		}

		parentclk_name = priv->pdata->parentclk_name;
			if (parentclk_name) {
			parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
			if (IS_ERR(parent))
				dev_err(&priv->i2c_client->dev,
					"unable to get parent clock %s",
					parentclk_name);
			else
				clk_set_parent(pw->mclk, parent);
		}
	}
	
	pw->state = SWITCH_OFF;
	return err;
}

/**
 * @brief       Stream control
 * @param[in]   *sd     : V4L2 sub device info
 * @param[in]   enable  : Stream enable/disable
 * @return      err     : Error code
 */
static int ox01f10_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ox01f10 *priv = (struct ox01f10 *)s_data->priv;
	struct v4l2_control control;
	int err = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;

	// Increment the refs count when streaming and decrement when streaming is disabled
	if (enable) {
		if (!try_module_get(s_data->owner))
			return -ENODEV;
	} else {
		module_put(s_data->owner);
	}
	
	printk("ox01f10_s_stream enable=%d\n", enable);
	
	if(enable)
	{
		control.id = CAMERA_CID_USER_HDR_EN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_hdr(priv, control.value);
		
		control.id = CAMERA_CID_AECAGC_EN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_aeagc(priv, control.value);
		
		if(control.value == 0)
		{
			control.id = CAMERA_CID_EXP_LONG;
			err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
			if (err)
				return err;
			ox01f10_set_exp_long(priv, control.value);
			
			control.id = CAMERA_CID_EXP_SHORT;
			err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
			if (err)
				return err;
			ox01f10_set_exp_short(priv, control.value);
			
			control.id = CAMERA_CID_EXP_VERY_SHORT;
			err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
			if (err)
				return err;
			ox01f10_set_exp_very_short(priv, control.value);
			
			control.id = CAMERA_CID_USER_GAIN;
			err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
			if (err)
				return err;
			ox01f10_set_gain(priv, control.value);
		}
		
		control.id = CAMERA_CID_AWB_EN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_awb_en(priv, control.value);
		
		if(control.value == 0)
		{
			control.id = CAMERA_CID_AWB_GAIN_R;
			err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
			if (err)
				return err;
			ox01f10_set_awb_gain_r(priv, control.value);
			
			control.id = CAMERA_CID_AWB_GAIN_B;
			err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
			if (err)
				return err;
			ox01f10_set_awb_gain_b(priv, control.value);
		}
		
		control.id = CAMERA_CID_Y_OFFSET;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_y_offset(priv, control.value);
		
		control.id = CAMERA_CID_Y_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_y_gain(priv, control.value);
		
		control.id = CAMERA_CID_USER_SATURATION;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_saturation(priv, control.value);
		
		control.id = CAMERA_CID_BAND_CONTROL;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_band(priv, control.value);
		
		control.id = CAMERA_CID_USER_MIRROR;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_mirror(priv, control.value);
		
		control.id = CAMERA_CID_USER_FLIP;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		if (err)
			return err;
		ox01f10_set_flip(priv, control.value);
		
		err = ox01f10_write_table(priv, ox01f10_start);
		if (err)
			return err;
	}
	else
	{
		err = ox01f10_write_table(priv, ox01f10_stop);
		if (err)
			return err;
	}

	return 0;
}

/**
 * @brief       Get parameter
 * @param[in]   *sd     : V4L2 sub device info
 * @param[in]   *param  : Stream parameter
 * @return      err     : Error code
 */
static int ox01f10_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ox01f10 *priv = (struct ox01f10 *)s_data->priv;

	if (!priv || !priv->pdata) {
		return -ENOTTY;
	}

	return 0;
}

/**
 * @brief       Set parameter
 * @param[in]   *sd     : V4L2 sub device info
 * @param[in]   *param  : Stream parameter
 * @return      err     : Error code
 */
static int ox01f10_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ox01f10 *priv = (struct ox01f10 *)s_data->priv;

	if (!priv || !priv->pdata) {
		return -EINVAL;
	}

	return 0;
}

static struct v4l2_subdev_video_ops ox01f10_subdev_video_ops = {
	.s_stream = ox01f10_s_stream,
	.g_mbus_config = camera_common_g_mbus_config,
	.g_parm = ox01f10_g_parm,
	.s_parm = ox01f10_s_parm,
};

static struct v4l2_subdev_core_ops ox01f10_subdev_core_ops = {
	.s_power = camera_common_s_power,
};

/**
 * @brief       Get format
 * @param[in]   *sd     : V4L2 sub device info
 * @param[in]   *cfg    : V4L2 sub device pad config info
 * @param[out]  *format : V4L2 sub device format info
 * @return      err     : Error code
 */
static int ox01f10_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

/**
 * @brief       Set format
 * @param[in]   *sd     : V4L2 sub device info
 * @param[in]   *cfg    : V4L2 sub device pad config info
 * @param[in]   *format : V4L2 sub device format info
 * @return      err     : Error code
 */
static int ox01f10_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static struct v4l2_subdev_pad_ops ox01f10_subdev_pad_ops = {
	.enum_mbus_code = camera_common_enum_mbus_code,
	.set_fmt = ox01f10_set_fmt,
	.get_fmt = ox01f10_get_fmt,
	.enum_frame_size = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops ox01f10_subdev_ops = {
	.core = &ox01f10_subdev_core_ops,
	.video = &ox01f10_subdev_video_ops,
	.pad = &ox01f10_subdev_pad_ops,
};

static struct of_device_id ox01f10_of_match[] = {
	{.compatible = "nvidia,ox01f10",},
	{},
};

/**
 * @brief       Initialize control info
 * @param[in]   *priv    : ox01f10 driver info
 * @return      err      : Error code
 */
static int ox01f10_ctrls_init(struct ox01f10 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct device *dev = &client->dev;
	struct v4l2_ctrl *ctrl;
	int numctrls;
	int err;
	int i;

	dev_dbg(dev, "%s++\n", __func__);

	numctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

	for (i = 0; i < numctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
				printk("%d. Initialized Custom Ctrl %s \n",
				 i, ctrl_config_list[i].name);
		priv->ctrls[i] = ctrl;
	}

	priv->numctrls = numctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, ox01f10_of_match);

/**
 * @brief       Parse device tree
 * @param[in]   *client  : I2C client info
 * @return      err      : Error code
 */
static struct camera_common_pdata *ox01f10_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio;

	if (!node)
		return NULL;

	match = of_match_device(ox01f10_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata =
		devm_kzalloc(&client->dev, sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
	if (gpio < 0) {
		dev_err(&client->dev, "pwdn gpios not in DT\n");
		goto error;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static const struct media_entity_operations ox01f10_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/**
 * @brief      Reset gpio control
 * @param[in]   gpio  : GPIO info
 * @param[in]   val   : Set value
 * @return      err   : Error code
 */
static void ox01f10_reset_gpio(unsigned int gpio, int val)
{
	if (gpio_cansleep(gpio)){
		gpio_direction_output(gpio,val);
		gpio_set_value_cansleep(gpio, val);
	} else{
		gpio_direction_output(gpio,val);
		gpio_set_value(gpio, val);
	}
	
	if(1 == val) msleep(31);
}

/**
 * @brief       Driver initialize/detection
 * @param[in]   *client  : I2C client info
 * @param[in]   *id      : I2C device id
 * @return      err      : Error code
 */
static int ox01f10_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct ox01f10 *priv;

	unsigned char read_reg;

	int err = 0;
	if (!IS_ENABLED(CONFIG_OF) || !node) {
		dev_err(&client->dev, "not enable CONFIG_OF or node ptr null\n");
		return -EINVAL;
	}

	common_data =
		devm_kzalloc(&client->dev,
			 sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "memory alloc failed(struct camera_common_data)\n");
		return -ENOMEM;
	}

	priv =
		devm_kzalloc(&client->dev,
			 sizeof(struct ox01f10) +
			 sizeof(struct v4l2_ctrl *) * ARRAY_SIZE(ctrl_config_list),
			 GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "memory alloc failed(struct ox01f10)\n");
		return -ENOMEM;
	}
	
	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = ox01f10_parse_dt(client);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}
	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	common_data->priv = (void *)priv;

	err = ox01f10_power_get(priv);
	if (err) {
		dev_err(&client->dev, "power get failed\n");
		return err;
	}

	err = ox01f10_power_on(common_data);
	if (err) {
		dev_err(&client->dev, "power on failed\n");
		return err;
	}
	
	/* Check I2C connection */
	err = ox01f10_read_reg(priv, 0x0100, &read_reg);
	if (err) {
		dev_err(&client->dev, "reg read failed\n");
		return -EFAULT;
	}
	
	common_data->ops = NULL;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->frmfmt = ox01f10_frmfmt;
	common_data->colorfmt = camera_common_find_datafmt(OX01F10_DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts = ARRAY_SIZE(ox01f10_frmfmt);
	common_data->def_mode = OX01F10_DEFAULT_MODE;
	common_data->def_width = OX01F10_DEFAULT_WIDTH;
	common_data->def_height = OX01F10_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = 24000000;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;

	err = camera_common_initialize(common_data, "ox01f10");
	if (err) {
		dev_err(&client->dev, "Failed to initialize ox01f10.\n");
		return err;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &ox01f10_subdev_ops);

	/* Enumerate Ctrls */
	err = ox01f10_ctrls_init(priv);
	if (err)
	{
		dev_err(&client->dev, "ctrls init failed\n");
		return err;
	}

	priv->subdev->flags |=
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	if (priv->subdev->owner == THIS_MODULE) {
		common_data->owner = priv->subdev->owner;
		priv->subdev->owner = NULL;
	} else {
		// It shouldn't come here in probe();
		;
	}

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &ox01f10_media_ops;
	err = tegra_media_entity_init(&priv->subdev->entity, 1, &priv->pad, true, true);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		goto  err_ctrls;
	}

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
	{
		dev_err(&client->dev, "v4l2_async_register_subdev failed\n");
		goto err_me;
	}
	
	err = ox01f10_write_table(priv, mode_table[priv->s_data->mode]);
	if (err)
		return err;
		
	
	return 0;
	
err_me:
	media_entity_cleanup(&priv->subdev->entity);
	
err_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	
	return err;
}

#define FREE_SAFE(dev, ptr) \
	if(ptr) { \
		devm_kfree(dev, ptr); \
	}


/**
 * @brief       Remove driver
 * @param[in]   *client  : I2C client info
 * @return      err      : Error code
 */
static int ox01f10_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ox01f10 *priv = (struct ox01f10 *)s_data->priv;
	struct device_node *node = client->dev.of_node;
	int loop = 0;
	uint16_t pwdn_gpio = 0;
		
	if (!priv || !priv->pdata)
		return -1;

	/* pwdn the Gpios */
	pwdn_gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
	if(pwdn_gpio < 0) {
		dev_err(&client->dev, "Unable to get power GPIO\n");
		return -EINVAL;
	}
	gpio_free(pwdn_gpio);
	
	v4l2_async_unregister_subdev(priv->subdev);
	media_entity_cleanup(&priv->subdev->entity);

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	ox01f10_power_off(priv);
	camera_common_remove_debugfs(s_data);

	for(loop = 0; loop < s_data->numfmts; loop++ ) {
		FREE_SAFE(&client->dev, (void *)priv->cam_frmfmt[loop].framerates);
	}

	FREE_SAFE(&client->dev, priv->cam_frmfmt);

	FREE_SAFE(&client->dev, priv->pdata);
	FREE_SAFE(&client->dev, priv->s_data);
	FREE_SAFE(&client->dev, priv);
	return 0;
}

static const struct i2c_device_id ox01f10_id[] = {
	{"ox01f10", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ox01f10_id);

static struct i2c_driver ox01f10_i2c_driver = {
	.driver = {
		   .name = "ox01f10",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ox01f10_of_match),
		   },
	.probe = ox01f10_probe,
	.remove = ox01f10_remove,
	.id_table = ox01f10_id,
};

module_i2c_driver(ox01f10_i2c_driver);

MODULE_DESCRIPTION("V4L2 driver for OX01F10");
MODULE_AUTHOR("Shikino High-Tech");
MODULE_LICENSE("GPL v2");
