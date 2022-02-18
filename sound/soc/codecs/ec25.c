// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EC25 audio driver
 *
 * Copyright (c) 2022 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: wm8782.c
 *   Copyright:	Analog Devices Inc.
 *   Author:	Cliff Cai <cliff.cai@analog.com>
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

static const struct snd_soc_dapm_widget ec25_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("VIN"),
	SND_SOC_DAPM_OUTPUT("VOUT"),
};

static const struct snd_soc_dapm_route ec25_dapm_routes[] = {
	{ "Capture", NULL, "VIN" },
	{ "VOUT", NULL, "Playback" },
};

static struct snd_soc_dai_driver ec25_dai = {
	.name = "ec25",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
};

static const struct snd_soc_component_driver soc_component_dev_ec25 = {
	.dapm_widgets		= ec25_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ec25_dapm_widgets),
	.dapm_routes		= ec25_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ec25_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int ec25_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
			&soc_component_dev_ec25, &ec25_dai, 1);
}

static const struct of_device_id ec25_codec_of_match[] = {
	{ .compatible = "quectel,ec25" },
	{},
};
MODULE_DEVICE_TABLE(of, ec25_codec_of_match);

static struct platform_driver ec25_codec_driver = {
	.driver = {
		.name = "ec25",
		.of_match_table = of_match_ptr(ec25_codec_of_match),
	},
	.probe = ec25_probe,
};

module_platform_driver(ec25_codec_driver);

MODULE_DESCRIPTION("Quectel EC25 audio driver");
MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_LICENSE("GPL v2");
