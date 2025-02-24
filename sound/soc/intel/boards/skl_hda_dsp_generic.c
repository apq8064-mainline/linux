// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2015-18 Intel Corporation.

/*
 * Machine Driver for SKL+ platforms with DSP and iDisp, HDA Codecs
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-acpi.h>
#include "skl_hda_dsp_common.h"

static const struct snd_soc_dapm_widget skl_hda_widgets[] = {
	SND_SOC_DAPM_HP("Analog Out", NULL),
	SND_SOC_DAPM_MIC("Analog In", NULL),
	SND_SOC_DAPM_HP("Alt Analog Out", NULL),
	SND_SOC_DAPM_MIC("Alt Analog In", NULL),
	SND_SOC_DAPM_SPK("Digital Out", NULL),
	SND_SOC_DAPM_MIC("Digital In", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
};

static const struct snd_soc_dapm_route skl_hda_map[] = {
	{ "hifi3", NULL, "iDisp3 Tx"},
	{ "iDisp3 Tx", NULL, "iDisp3_out"},
	{ "hifi2", NULL, "iDisp2 Tx"},
	{ "iDisp2 Tx", NULL, "iDisp2_out"},
	{ "hifi1", NULL, "iDisp1 Tx"},
	{ "iDisp1 Tx", NULL, "iDisp1_out"},

	{ "Analog Out", NULL, "Codec Output Pin1" },
	{ "Digital Out", NULL, "Codec Output Pin2" },
	{ "Alt Analog Out", NULL, "Codec Output Pin3" },

	{ "Codec Input Pin1", NULL, "Analog In" },
	{ "Codec Input Pin2", NULL, "Digital In" },
	{ "Codec Input Pin3", NULL, "Alt Analog In" },

	/* digital mics */
	{"DMic", NULL, "SoC DMIC"},

	/* CODEC BE connections */
	{ "Analog Codec Playback", NULL, "Analog CPU Playback" },
	{ "Analog CPU Playback", NULL, "codec0_out" },
	{ "Digital Codec Playback", NULL, "Digital CPU Playback" },
	{ "Digital CPU Playback", NULL, "codec1_out" },
	{ "Alt Analog Codec Playback", NULL, "Alt Analog CPU Playback" },
	{ "Alt Analog CPU Playback", NULL, "codec2_out" },

	{ "codec0_in", NULL, "Analog CPU Capture" },
	{ "Analog CPU Capture", NULL, "Analog Codec Capture" },
	{ "codec1_in", NULL, "Digital CPU Capture" },
	{ "Digital CPU Capture", NULL, "Digital Codec Capture" },
	{ "codec2_in", NULL, "Alt Analog CPU Capture" },
	{ "Alt Analog CPU Capture", NULL, "Alt Analog Codec Capture" },
};

static int skl_hda_card_late_probe(struct snd_soc_card *card)
{
	return skl_hda_hdmi_jack_init(card);
}

static int
skl_hda_add_dai_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
	struct skl_hda_private *ctx = snd_soc_card_get_drvdata(card);
	int ret = 0;

	dev_dbg(card->dev, "dai link name - %s\n", link->name);
	link->platforms->name = ctx->platform_name;
	link->nonatomic = 1;

	if (!ctx->hdmi.idisp_codec)
		return 0;

	if (strstr(link->name, "HDMI")) {
		ret = skl_hda_hdmi_add_pcm(card, ctx->pcm_count);

		if (ret < 0)
			return ret;

		ctx->dai_index++;
	}

	ctx->pcm_count++;
	return ret;
}

#define IDISP_DAI_COUNT		3
#define HDAC_DAI_COUNT		2
#define DMIC_DAI_COUNT		2
#define BT_DAI_COUNT		1

/* there are two routes per iDisp output */
#define IDISP_ROUTE_COUNT	(IDISP_DAI_COUNT * 2)

#define HDA_CODEC_AUTOSUSPEND_DELAY_MS 1000

static int skl_hda_fill_card_info(struct device *dev, struct snd_soc_card *card,
				  struct snd_soc_acpi_mach_params *mach_params)
{
	struct skl_hda_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai_link *dai_link;
	struct snd_soc_dai_link *bt_link;
	u32 codec_count, codec_mask;
	int i, num_links, num_route;

	codec_mask = mach_params->codec_mask;
	codec_count = hweight_long(codec_mask);

	if (!codec_count || codec_count > 2 ||
	    (codec_count == 2 && !ctx->hdmi.idisp_codec))
		return -EINVAL;

	if (codec_mask == IDISP_CODEC_MASK) {
		/* topology with iDisp as the only HDA codec */
		num_links = IDISP_DAI_COUNT + DMIC_DAI_COUNT + BT_DAI_COUNT;
		num_route = IDISP_ROUTE_COUNT;

		/*
		 * rearrange the dai link array and make the
		 * dmic dai links follow idsp dai links for only
		 * num_links of dai links need to be registered
		 * to ASoC.
		 */
		for (i = 0; i < (DMIC_DAI_COUNT + BT_DAI_COUNT); i++) {
			skl_hda_be_dai_links[IDISP_DAI_COUNT + i] =
				skl_hda_be_dai_links[IDISP_DAI_COUNT +
					HDAC_DAI_COUNT + i];
		}
	} else {
		/* topology with external and iDisp HDA codecs */
		num_links = ARRAY_SIZE(skl_hda_be_dai_links);
		num_route = ARRAY_SIZE(skl_hda_map);
		card->dapm_widgets = skl_hda_widgets;
		card->num_dapm_widgets = ARRAY_SIZE(skl_hda_widgets);
		if (!ctx->hdmi.idisp_codec) {
			card->dapm_routes = &skl_hda_map[IDISP_ROUTE_COUNT];
			num_route -= IDISP_ROUTE_COUNT;
			for (i = 0; i < IDISP_DAI_COUNT; i++) {
				skl_hda_be_dai_links[i].codecs = &snd_soc_dummy_dlc;
				skl_hda_be_dai_links[i].num_codecs = 1;
			}
		}
	}

	if (!ctx->bt_offload_present) {
		/* remove last link since bt audio offload is not supported */
		num_links -= BT_DAI_COUNT;
	} else {
		if (codec_mask == IDISP_CODEC_MASK)
			bt_link = &skl_hda_be_dai_links[IDISP_DAI_COUNT + DMIC_DAI_COUNT];
		else
			bt_link = &skl_hda_be_dai_links[IDISP_DAI_COUNT + HDAC_DAI_COUNT + DMIC_DAI_COUNT];

		/* complete the link name and dai name with SSP port number */
		bt_link->name = devm_kasprintf(dev, GFP_KERNEL, "SSP%d-BT",
					       ctx->ssp_bt);
		if (!bt_link->name)
			return -ENOMEM;

		bt_link->cpus->dai_name = devm_kasprintf(dev, GFP_KERNEL,
							 "SSP%d Pin",
							 ctx->ssp_bt);
		if (!bt_link->cpus->dai_name)
			return -ENOMEM;
	}

	card->num_links = num_links;
	card->num_dapm_routes = num_route;

	for_each_card_prelinks(card, i, dai_link)
		dai_link->platforms->name = mach_params->platform;

	return 0;
}

static void skl_set_hda_codec_autosuspend_delay(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	struct hdac_hda_priv *hda_pvt;
	struct snd_soc_dai *dai;

	for_each_card_rtds(card, rtd) {
		if (!strstr(rtd->dai_link->codecs->name, "ehdaudio0D0"))
			continue;
		dai = snd_soc_rtd_to_codec(rtd, 0);
		hda_pvt = snd_soc_component_get_drvdata(dai->component);
		if (hda_pvt) {
			/*
			 * all codecs are on the same bus, so it's sufficient
			 * to look up only the first one
			 */
			snd_hda_set_power_save(hda_pvt->codec->bus,
					       HDA_CODEC_AUTOSUSPEND_DELAY_MS);
			break;
		}
	}
}

static int skl_hda_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_acpi_mach *mach = pdev->dev.platform_data;
	struct skl_hda_private *ctx;
	struct snd_soc_card *card;
	int ret;

	dev_dbg(&pdev->dev, "entry\n");

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	card = &ctx->card;
	card->name = "hda-dsp";
	card->owner = THIS_MODULE;
	card->dai_link = skl_hda_be_dai_links;
	card->dapm_widgets = skl_hda_widgets;
	card->dapm_routes = skl_hda_map;
	card->add_dai_link = skl_hda_add_dai_link;
	card->fully_routed = true;
	card->late_probe = skl_hda_card_late_probe;

	snd_soc_card_set_drvdata(card, ctx);

	if (mach->mach_params.codec_mask & IDISP_CODEC_MASK)
		ctx->hdmi.idisp_codec = true;

	if (hweight_long(mach->mach_params.bt_link_mask) == 1) {
		ctx->bt_offload_present = true;
		ctx->ssp_bt = fls(mach->mach_params.bt_link_mask) - 1;
	}

	ret = skl_hda_fill_card_info(&pdev->dev, card, &mach->mach_params);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unsupported HDAudio/iDisp configuration found\n");
		return ret;
	}

	ctx->pcm_count = card->num_links;
	ctx->dai_index = 1; /* hdmi codec dai name starts from index 1 */
	ctx->platform_name = mach->mach_params.platform;

	card->dev = &pdev->dev;
	if (!snd_soc_acpi_sof_parent(&pdev->dev))
		card->disable_route_checks = true;

	if (mach->mach_params.dmic_num > 0) {
		card->components = devm_kasprintf(card->dev, GFP_KERNEL,
						  "cfg-dmics:%d",
						  mach->mach_params.dmic_num);
		if (!card->components)
			return -ENOMEM;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (!ret)
		skl_set_hda_codec_autosuspend_delay(card);

	return ret;
}

static struct platform_driver skl_hda_audio = {
	.probe = skl_hda_audio_probe,
	.driver = {
		.name = "skl_hda_dsp_generic",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(skl_hda_audio)

/* Module information */
MODULE_DESCRIPTION("SKL/KBL/BXT/APL HDA Generic Machine driver");
MODULE_AUTHOR("Rakesh Ughreja <rakesh.a.ughreja@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:skl_hda_dsp_generic");
MODULE_IMPORT_NS(SND_SOC_INTEL_HDA_DSP_COMMON);
