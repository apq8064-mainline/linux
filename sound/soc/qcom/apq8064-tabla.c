// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm APQ8064 with WCD9310, using Apps-managed SLIMbus playback. */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <sound/soc.h>

#include "apq8064-slim.h"

struct apq8064_tabla {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codec;
	struct snd_soc_dai_link_component platform;
	struct gpio_desc *spk_en;
	struct gpio_desc *spk_en2;
};

/*
 * The flo has no external headphone-jack GPIO: the original hardware relied
 * on the WCD9310 codec's internal MBHC for detection, which this SLIMbus
 * driver does not wire up. Instead, the active output route is selected
 * manually via the "Audio Output" control: 0 = Speaker, 1 = Headphone.
 *
 * Picking Headphone disables the Speaker pin, so DAPM powers down the
 * external speaker amp (its event deasserts the amp enable GPIOs); picking
 * Speaker does the reverse.
 */
static int audio_out_mode;

static const char *const audio_out_texts[] = { "Speaker", "Headphone" };

static const struct soc_enum audio_out_enum = {
	.items = ARRAY_SIZE(audio_out_texts),
	.mask = 1,
	.texts = audio_out_texts,
};

static int audio_out_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = audio_out_mode;
	return 0;
}

static int audio_out_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);
	int mode, ret;

	if (ucontrol->value.enumerated.item[0] >= ARRAY_SIZE(audio_out_texts))
		return -EINVAL;

	mode = ucontrol->value.enumerated.item[0];
	if (mode == audio_out_mode)
		return 1;

	/*
	 * Switch the active output pin under the DAPM lock so the pin change
	 * and its power sequencing happen atomically.
	 */
	snd_soc_dapm_mutex_lock(dapm);
	if (mode) {
		ret = snd_soc_dapm_disable_pin_unlocked(dapm, "Speaker");
		if (!ret)
			ret = snd_soc_dapm_enable_pin_unlocked(dapm,
							       "Headphone");
	} else {
		ret = snd_soc_dapm_disable_pin_unlocked(dapm, "Headphone");
		if (!ret)
			ret = snd_soc_dapm_enable_pin_unlocked(dapm,
							       "Speaker");
	}
	if (!ret)
		ret = snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);
	if (ret)
		return ret;

	dev_info(card->dev, "audio output: %s\n", audio_out_texts[mode]);
	audio_out_mode = mode;
	return 1;
}

static int apq8064_tabla_spk_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(w->dapm);
	struct apq8064_tabla *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = gpiod_set_value_cansleep(priv->spk_en,
				      event == SND_SOC_DAPM_PRE_PMU);
	if (ret)
		return ret;
	if (priv->spk_en2) {
		ret = gpiod_set_value_cansleep(priv->spk_en2,
					      event == SND_SOC_DAPM_PRE_PMU);
		if (ret)
			return ret;
	}
	usleep_range(4000, 4500);
	return 0;
}

static const struct snd_kcontrol_new apq8064_tabla_controls[] = {
	SOC_ENUM_EXT("Audio Output", audio_out_enum, audio_out_get,
		     audio_out_put),
};

static const struct snd_soc_dapm_widget apq8064_tabla_widgets[] = {
	/* OUT_DRV sequences the external amp after both codec lineout PAs. */
	SND_SOC_DAPM_OUT_DRV_E("Speaker Amplifier", SND_SOC_NOPM, 0, 0,
			       NULL, 0, apq8064_tabla_spk_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("Speaker"),
	SND_SOC_DAPM_OUTPUT("Earpiece"),
	SND_SOC_DAPM_OUTPUT("Headphone"),
};

static const struct snd_soc_dapm_route apq8064_tabla_routes[] = {
	/* Top amp: LINEOUT2/4 bridge.  Bottom amp: LINEOUT1/3 bridge. */
	{ "Speaker Amplifier", NULL, "LINEOUT2" },
	{ "Speaker Amplifier", NULL, "LINEOUT4" },
	{ "Speaker Amplifier", NULL, "LINEOUT1" },
	{ "Speaker Amplifier", NULL, "LINEOUT3" },
	{ "Speaker", NULL, "Speaker Amplifier" },
	{ "Earpiece", NULL, "EAR" },
	{ "Headphone", NULL, "HPHL" },
	{ "Headphone", NULL, "HPHR" },
};

static int apq8064_tabla_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(rtd->card);
	const unsigned int rx_slot = 138;
	int ret;

	ret = snd_soc_dai_set_channel_map(snd_soc_rtd_to_codec(rtd, 0),
					 0, NULL, 1, &rx_slot);
	if (ret)
		return ret;

	/* Speaker is the default output; the "Audio Output" control switches
	 * to Headphone.  Keep the unused Earpiece off. */
	ret = snd_soc_dapm_disable_pin(dapm, "Earpiece");
	if (ret)
		return ret;
	return snd_soc_dapm_disable_pin(dapm, "Headphone");
}

static void apq8064_tabla_put_codec_node(void *data)
{
	of_node_put(data);
}

static int apq8064_tabla_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apq8064_tabla *priv;
	struct snd_soc_card *card;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spk_en = devm_gpiod_get(dev, "speaker-en", GPIOD_OUT_LOW);
	if (IS_ERR(priv->spk_en))
		return dev_err_probe(dev, PTR_ERR(priv->spk_en),
				     "failed to get speaker enable GPIO\n");

	/* Second (bottom) amp, present on dual-speaker boards. */
	priv->spk_en2 = devm_gpiod_get_optional(dev, "speaker-bottom-en",
						 GPIOD_OUT_LOW);

	priv->codec.of_node = of_parse_phandle(dev->of_node, "codec", 0);
	if (!priv->codec.of_node)
		return -EINVAL;
	ret = devm_add_action_or_reset(dev, apq8064_tabla_put_codec_node,
				       priv->codec.of_node);
	if (ret)
		return ret;

	priv->cpu.of_node = dev->of_node;
	priv->cpu.dai_name = "apq8064-slim";
	priv->codec.dai_name = "tabla_rx1";
	priv->platform.of_node = dev->of_node;
	priv->link.name = "Tabla";
	priv->link.stream_name = "Tabla";
	priv->link.init = apq8064_tabla_link_init;
	priv->link.cpus = &priv->cpu;
	priv->link.num_cpus = 1;
	priv->link.codecs = &priv->codec;
	priv->link.num_codecs = 1;
	priv->link.platforms = &priv->platform;
	priv->link.num_platforms = 1;
	/* Codec port setup precedes host activation; shutdown reverses this. */
	priv->link.trigger_start = SND_SOC_TRIGGER_ORDER_LDC;
	priv->link.trigger_stop = SND_SOC_TRIGGER_ORDER_LDC;
	priv->link.nonatomic = true;

	card = &priv->card;
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->dai_link = &priv->link;
	card->num_links = 1;
	card->controls = apq8064_tabla_controls;
	card->num_controls = ARRAY_SIZE(apq8064_tabla_controls);
	card->dapm_widgets = apq8064_tabla_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(apq8064_tabla_widgets);
	card->dapm_routes = apq8064_tabla_routes;
	card->num_dapm_routes = ARRAY_SIZE(apq8064_tabla_routes);
	snd_soc_card_set_drvdata(card, priv);

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		return ret;
	ret = devm_snd_soc_register_component(dev, &apq8064_slim_component,
					      &apq8064_slim_dai, 1);
	if (ret)
		return ret;
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id apq8064_tabla_of_match[] = {
	{ .compatible = "qcom,apq8064-tabla-snd-card" },
	{ }
};
MODULE_DEVICE_TABLE(of, apq8064_tabla_of_match);

static struct platform_driver apq8064_tabla_driver = {
	.probe = apq8064_tabla_probe,
	.driver = {
		.name = "apq8064-tabla-snd-card",
		.of_match_table = apq8064_tabla_of_match,
	},
};
module_platform_driver(apq8064_tabla_driver);

MODULE_DESCRIPTION("Qualcomm APQ8064 with WCD9310 audio");
MODULE_LICENSE("GPL");
