// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Renesas RZ/G2L CRU
 *
 * Copyright (C) 2022 Renesas Electronics Corp.
 */

#include <linux/delay.h>
#include "rzg2l-cru.h"

struct rzg2l_cru_ip_format {
	u32 code;
	unsigned int datatype;
	unsigned int bpp;
};

static const struct rzg2l_cru_ip_format rzg2l_cru_ip_formats[] = {
	{ .code = MEDIA_BUS_FMT_UYVY8_2X8,	.datatype = 0x1e, .bpp = 16 },
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16,	.datatype = 0x1e, .bpp = 16 },
};

enum rzg2l_csi2_pads {
	RZG2L_CRU_IP_SINK = 0,
	RZG2L_CRU_IP_SOURCE,
};

static inline struct rzg2l_cru_ip *sd_to_ip(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rzg2l_cru_ip, subdev);
}

static const struct rzg2l_cru_ip_format *rzg2l_cru_ip_code_to_fmt(unsigned int code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rzg2l_cru_ip_formats); i++)
		if (rzg2l_cru_ip_formats[i].code == code)
			return &rzg2l_cru_ip_formats[i];

	return NULL;
}

struct v4l2_mbus_framefmt *rzg2l_cru_ip_get_src_fmt(struct rzg2l_cru_dev *cru)
{
	return &cru->ip.mf;
}

static int rzg2l_cru_ip_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rzg2l_cru_dev *cru;
	int s_stream_ret = 0;
	int ret;

	cru = v4l2_get_subdevdata(sd);

	if (!enable) {
		ret = v4l2_subdev_call(cru->ip.remote, video, s_stream, enable);
		if (ret)
			s_stream_ret = ret;

		ret = v4l2_subdev_call(cru->ip.remote, video, post_streamoff);
		if (ret == -ENOIOCTLCMD)
			ret = 0;
		if (ret && !s_stream_ret)
			s_stream_ret = ret;
		rzg2l_cru_stop_image_processing(cru);
	} else {
		ret = v4l2_subdev_call(cru->ip.remote, video, pre_streamon, 0);
		if (ret == -ENOIOCTLCMD)
			ret = 0;
		if (ret)
			return ret;

		fsleep(1000);

		ret = rzg2l_cru_start_image_processing(cru);
		if (ret) {
			v4l2_subdev_call(cru->ip.remote, video, post_streamoff);
			return ret;
		}

		rzg2l_cru_vclk_unprepare(cru);

		ret = v4l2_subdev_call(cru->ip.remote, video, s_stream, enable);
		if (ret == -ENOIOCTLCMD)
			ret = 0;
		if (!ret) {
			ret = rzg2l_cru_vclk_prepare(cru);
			if (!ret)
				return 0;
		} else {
			/* enable back vclk so that s_stream in error path disables it */
			if (rzg2l_cru_vclk_prepare(cru))
				dev_err(cru->dev, "Failed to enable vclk\n");
		}

		s_stream_ret = ret;

		v4l2_subdev_call(cru->ip.remote, video, post_streamoff);
		rzg2l_cru_stop_image_processing(cru);
	}

	return s_stream_ret;
}

static int rzg2l_cru_ip_get_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *fmt)
{
	struct rzg2l_cru_ip *ip = sd_to_ip(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	else
		fmt->format = ip->mf;

	return 0;
}

static int rzg2l_cru_ip_set_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *fmt)
{
	struct rzg2l_cru_ip *ip = sd_to_ip(sd);
	struct v4l2_mbus_framefmt *src_format;
	struct v4l2_mbus_framefmt *sink_format;

	src_format = v4l2_subdev_get_try_format(sd, cfg, RZG2L_CRU_IP_SOURCE);
	if (fmt->pad == RZG2L_CRU_IP_SOURCE) {
		fmt->format = *src_format;
		return 0;
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ip->mf = fmt->format;
		return 0;
	}

	sink_format = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);

	if (!rzg2l_cru_ip_code_to_fmt(fmt->format.code))
		sink_format->code = rzg2l_cru_ip_formats[0].code;
	else
		sink_format->code = fmt->format.code;

	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.width = clamp_t(u32, fmt->format.width,
				    RZG2L_CRU_MIN_INPUT_WIDTH, RZG2L_CRU_MAX_INPUT_WIDTH);
	fmt->format.height = clamp_t(u32, fmt->format.height,
				     RZG2L_CRU_MIN_INPUT_HEIGHT, RZG2L_CRU_MAX_INPUT_HEIGHT);

	*sink_format = fmt->format;

	/* propagate format to source pad */
	*src_format = *sink_format;

	return 0;
}

static int rzg2l_cru_ip_enum_mbus_code(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(rzg2l_cru_ip_formats))
		return -EINVAL;

	code->code = rzg2l_cru_ip_formats[code->index].code;
	return 0;
}

static int rzg2l_cru_ip_enum_frame_size(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index != 0)
		return -EINVAL;

	if (!rzg2l_cru_ip_code_to_fmt(fse->code))
		return -EINVAL;

	fse->min_width = RZG2L_CRU_MIN_INPUT_WIDTH;
	fse->min_height = RZG2L_CRU_MIN_INPUT_HEIGHT;
	fse->max_width = RZG2L_CRU_MAX_INPUT_WIDTH;
	fse->max_height = RZG2L_CRU_MAX_INPUT_HEIGHT;

	return 0;
}

static int rzg2l_cru_ip_init_config(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { .pad = RZG2L_CRU_IP_SINK, };

	fmt.format.width = RZG2L_CRU_MIN_INPUT_WIDTH;
	fmt.format.height = RZG2L_CRU_MIN_INPUT_HEIGHT;
	fmt.format.field = V4L2_FIELD_NONE;
	fmt.format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt.format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt.format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt.format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt.format.xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return rzg2l_cru_ip_set_format(sd, cfg, &fmt);
}

static const struct v4l2_subdev_video_ops rzg2l_cru_ip_video_ops = {
	.s_stream = rzg2l_cru_ip_s_stream,
};

static const struct v4l2_subdev_pad_ops rzg2l_cru_ip_pad_ops = {
	.enum_mbus_code = rzg2l_cru_ip_enum_mbus_code,
	.enum_frame_size = rzg2l_cru_ip_enum_frame_size,
	.init_cfg = rzg2l_cru_ip_init_config,
	.get_fmt = rzg2l_cru_ip_get_format,
	.set_fmt = rzg2l_cru_ip_set_format,
};

static const struct v4l2_subdev_ops rzg2l_cru_ip_subdev_ops = {
	.video = &rzg2l_cru_ip_video_ops,
	.pad = &rzg2l_cru_ip_pad_ops,
};

static const struct media_entity_operations rzg2l_cru_ip_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

int rzg2l_cru_ip_subdev_register(struct rzg2l_cru_dev *cru)
{
	struct rzg2l_cru_ip *ip = &cru->ip;
	int ret;

	ip->subdev.dev = cru->dev;
	v4l2_subdev_init(&ip->subdev, &rzg2l_cru_ip_subdev_ops);
	v4l2_set_subdevdata(&ip->subdev, cru);
	snprintf(ip->subdev.name, sizeof(ip->subdev.name),
		 "cru-ip-%s", dev_name(cru->dev));
	ip->subdev.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	ip->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	ip->subdev.entity.ops = &rzg2l_cru_ip_entity_ops;

	ip->pads[0].flags = MEDIA_PAD_FL_SINK;
	ip->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ip->subdev.entity, 2, ip->pads);
	if (ret)
		return ret;

	ret = v4l2_device_register_subdev(&cru->v4l2_dev, &ip->subdev);
	if (ret < 0)
		goto entity_cleanup;

	return 0;
entity_cleanup:
	media_entity_cleanup(&ip->subdev.entity);

	return ret;
}

void rzg2l_cru_ip_subdev_unregister(struct rzg2l_cru_dev *cru)
{
	struct rzg2l_cru_ip *ip = &cru->ip;

	media_entity_cleanup(&ip->subdev.entity);
	v4l2_device_unregister_subdev(&ip->subdev);
}
