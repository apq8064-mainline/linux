// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm WCD9310 "tabla" SLIMbus audio codec driver
 *
 * Playback driver for lineout, earpiece and headphone paths, with mono
 * SLIM RX1 feeding the RX1/RX2 and RX5/RX6 interpolators. Targets MSM8960/APQ8064
 * era devices such as the Samsung Galaxy S4 (jflte). Register protocol and power-up
 * recipes are ported from the downstream wcd9310 driver.
 *
 * The codec appears on the bus as two SLIMbus devices (product 0x60,
 * "tabla2x" silicon): the PGD device (dev_index 1) carries all codec
 * register access, the interface device (dev_index 0) carries the
 * SLIMbus port configuration registers.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slimbus.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#define TABLA_MANF_ID			0x217
#define TABLA_PROD_CODE			0x60

/* Codec registers are accessed as value elements at this base offset */
#define TABLA_REG_OFFSET		0x800
#define TABLA_MAX_REGISTER		0x3ff

/* SLIMbus channel/port layout for the codec RX (playback) ports */
#define TABLA_RX_PORT_START		10	/* RX slave ports 10-16 */

/*
 * Interface device (IFD) port configuration registers.  idx is the
 * zero-based RX port index (port - TABLA_RX_PORT_START).  Tabla's RX
 * ports start at slave port 10, so the register bases must skip the
 * first 10 ports to land on the RX-port block (matches downstream
 * wcd9xxx-slimslave.c for TABLA, not TAIKO).
 */
#define TABLA_IFC_PORT_CFG_BASE		(0x040 + TABLA_RX_PORT_START)
#define TABLA_IFC_RX_MCH_BASE		(0x180 + (TABLA_RX_PORT_START * 4))
#define TABLA_IFC_PORT_WM_ENABLE	((2 << 1) | 1)	/* wm 12 bytes + enable */

/* IFC SLIM port interrupt registers (downstream TABLA_SLIM_PGD_PORT_INT_*) */
#define TABLA_SLIM_PGD_PORT_INT_EN0	0x30
#define TABLA_SLIM_PGD_PORT_INT_STATUS0	0x34
#define TABLA_SLIM_PGD_PORT_INT_SOURCE0	0x60

/* Chip identification / bring-up */
#define TABLA_A_CHIP_ID_BYTE_0		0x004
#define TABLA_A_QFUSE_CTL		0x048
#define TABLA_A_CDC_CTL			0x080
#define TABLA_A_LEAKAGE_CTL		0x088

/* Clock and bias */
#define TABLA_A_BIAS_REF_CTL		0x100
#define TABLA_A_BIAS_CENTRAL_BG_CTL	0x101
#define TABLA_A_BIAS_BG_STATUS		0x106
#define TABLA_A_CP_EN			0x192
#define TABLA_A_CP_STATIC		0x194
#define TABLA_A_CLK_BUFF_EN1		0x108
#define TABLA_A_CLK_BUFF_EN2		0x109
#define TABLA_A_CDC_CLK_RX_RESET_CTL	0x301
#define TABLA_A_CDC_CLK_OTHR_CTL	0x30a
#define TABLA_A_CDC_CLK_OTHR_RESET_CTL	0x307
#define TABLA_A_CDC_CLK_RX_B1_CTL	0x30d
#define TABLA_A_CDC_CLK_MCLK_CTL	0x30f
#define TABLA_A_CDC_CLSG_CTL		0x327
#define TABLA_A_CDC_CLSG_FREQ_THRESH_B1_CTL	0x320
#define TABLA_A_CDC_CLSG_FREQ_THRESH_B2_CTL	0x321
#define TABLA_A_CDC_CLSG_FREQ_THRESH_B3_CTL	0x322
#define TABLA_A_CDC_CLSG_FREQ_THRESH_B4_CTL	0x323
#define TABLA_A_CDC_CLSG_GAIN_THRESH_CTL	0x324
#define TABLA_A_CDC_CONN_CLSG_CTL	0x3b0

/* Compander block registers (per-compander stride = 8) */
#define TABLA_A_CDC_COMP2_SHUT_DOWN_STATUS	0x37e

/* RX path */
#define TABLA_A_RX_COM_BIAS		0x1a2
#define TABLA_A_RX_COM_OCP_COUNT	0x1a0
#define TABLA_A_RX_HPH_OCP_CTL		0x1aa
#define TABLA_A_RX_HPH_CNP_EN		0x1ab
#define TABLA_A_RX_HPH_L_GAIN		0x1ae
#define TABLA_A_RX_HPH_L_DAC_CTL	0x1b1
#define TABLA_A_RX_HPH_R_GAIN		0x1b4
#define TABLA_A_RX_HPH_R_DAC_CTL	0x1b7
#define TABLA_A_MICB_1_INT_RBIAS	0x12c
#define TABLA_A_MICB_2_MBHC		0x133
#define TABLA_A_MICB_2_INT_RBIAS	0x132
#define TABLA_A_MICB_3_INT_RBIAS	0x138
#define TABLA_A_TX_5_6_EN		0x167
#define TABLA_2_A_MICB_4_INT_RBIAS	0x13e
#define TABLA_A_RX_LINE_CNP_EN		0x1ca
#define TABLA_A_RX_LINE_1_GAIN		0x1cd
#define TABLA_A_RX_LINE_1_DAC_CTL	0x1cf
#define TABLA_A_RX_LINE_1_STATUS	0x1d0
#define TABLA_A_RX_LINE_2_GAIN		0x1d1
#define TABLA_A_RX_LINE_2_DAC_CTL	0x1d3
#define TABLA_A_RX_LINE_2_STATUS	0x1d4
#define TABLA_A_RX_LINE_3_GAIN		0x1d5
#define TABLA_A_RX_LINE_3_DAC_CTL	0x1d7
#define TABLA_A_RX_LINE_3_STATUS	0x1d8
#define TABLA_A_RX_LINE_4_GAIN		0x1d9
#define TABLA_A_RX_LINE_4_DAC_CTL	0x1db
#define TABLA_A_RX_LINE_4_STATUS	0x1dc
#define TABLA_A_RX_HPH_BIAS_PA		0x1a6
#define TABLA_A_RX_HPH_BIAS_LDO		0x1a7
#define TABLA_A_RX_EAR_BIAS_PA		0x1ba
#define TABLA_A_RX_EAR_GAIN		0x1bd
#define TABLA_A_RX_EAR_VCM		0x1c1
#define TABLA_A_RX_LINE_5_GAIN		0x1dd
#define TABLA_A_RX_EAR_EN		0x1bc
#define TABLA_A_CDC_RX1_B5_CTL		0x2b4
#define TABLA_A_CDC_RX1_B6_CTL		0x2b5
#define TABLA_A_CDC_RX1_VOL_CTL_B2_CTL	0x2b7
#define TABLA_A_CDC_RX2_B5_CTL		0x2bc
#define TABLA_A_CDC_RX2_B6_CTL		0x2bd
#define TABLA_A_CDC_RX2_VOL_CTL_B2_CTL	0x2bf
#define TABLA_A_CDC_RX3_B5_CTL		0x2c4
#define TABLA_A_CDC_RX3_B6_CTL		0x2c5
#define TABLA_A_CDC_RX3_VOL_CTL_B2_CTL	0x2c7
#define TABLA_A_CDC_RX4_B5_CTL		0x2cc
#define TABLA_A_CDC_RX4_B6_CTL		0x2cd
#define TABLA_A_CDC_RX4_VOL_CTL_B2_CTL	0x2cf
#define TABLA_A_CDC_RX5_B5_CTL		0x2d4
#define TABLA_A_CDC_RX5_B6_CTL		0x2d5
#define TABLA_A_CDC_RX5_VOL_CTL_B2_CTL	0x2d7
#define TABLA_A_CDC_RX6_B5_CTL		0x2dc
#define TABLA_A_CDC_RX6_B6_CTL		0x2dd
#define TABLA_A_CDC_RX6_VOL_CTL_B2_CTL	0x2df
#define TABLA_A_CDC_CONN_RX1_B1_CTL	0x380
#define TABLA_A_CDC_CONN_RX2_B1_CTL	0x383
#define TABLA_A_CDC_CONN_RX3_B1_CTL	0x386
#define TABLA_A_CDC_CONN_RX5_B1_CTL	0x38b
#define TABLA_A_CDC_CONN_RX_SB_B1_CTL	0x3ae
#define TABLA_A_CDC_CONN_RX_SB_B2_CTL	0x3af

#define TABLA_MCLK_RATE			12288000

static int tabla_slim_status(struct slim_device *sdev,
			     enum slim_device_status status);

struct tabla {
	struct slim_device *ifc_sdev;
	struct regmap *regmap;
	struct regmap *ifc_regmap;
	struct clk *mclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[6];
	struct mutex init_lock;
	bool registered;
	bool powered;
	bool mclk_enabled;
	bool ifc_port_enabled;
};

static int tabla_ifc_port_enable(struct tabla *tabla, bool enable);

/*
 * Register access over SLIMbus value elements. Downstream uses
 * byte-wide transactions with a 3x retry @5ms; keep that behavior.
 */
static int tabla_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct slim_device *sdev = context;
	int v, retry = 3;

	do {
		v = slim_readb(sdev, TABLA_REG_OFFSET + reg);
		if (v >= 0)
			break;
		usleep_range(5000, 5500);
	} while (--retry);

	if (v < 0)
		return v;

	*val = v;
	return 0;
}

static int tabla_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct slim_device *sdev = context;
	int ret, retry = 3;

	do {
		ret = slim_writeb(sdev, TABLA_REG_OFFSET + reg, val);
		if (!ret)
			break;
		usleep_range(5000, 5500);
	} while (--retry);

	return ret;
}

/*
 * Status registers change without a driver write; never serve them from
 * the cache.  Includes the IFC port interrupt status/source range (the
 * same regmap config is shared by the PGD and IFC regmaps).
 */
static bool tabla_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TABLA_A_RX_LINE_1_STATUS:
	case TABLA_A_RX_LINE_2_STATUS:
	case TABLA_A_RX_LINE_3_STATUS:
	case TABLA_A_RX_LINE_4_STATUS:
	case TABLA_A_BIAS_BG_STATUS:
	case TABLA_A_CDC_COMP2_SHUT_DOWN_STATUS:
		return true;
	default:
		break;
	}

	if ((reg >= TABLA_SLIM_PGD_PORT_INT_STATUS0 &&
	     reg <= TABLA_SLIM_PGD_PORT_INT_STATUS0 + 3) ||
	    (reg >= TABLA_SLIM_PGD_PORT_INT_SOURCE0 &&
	     reg <= TABLA_SLIM_PGD_PORT_INT_SOURCE0 + 23))
		return true;

	return false;
}

static const struct regmap_config tabla_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = TABLA_MAX_REGISTER,
	.reg_read = tabla_reg_read,
	.reg_write = tabla_reg_write,
	.volatile_reg = tabla_volatile_register,
	.cache_type = REGCACHE_RBTREE,
};

static void tabla_codec_disable_clock_block(struct snd_soc_component *component)
{
	snd_soc_component_update_bits(component, TABLA_A_CLK_BUFF_EN2,
				      0x04, 0x00);
	usleep_range(50, 55);
	snd_soc_component_update_bits(component, TABLA_A_CLK_BUFF_EN2,
				      0x02, 0x02);
	snd_soc_component_update_bits(component, TABLA_A_CLK_BUFF_EN1,
				      0x05, 0x00);
	usleep_range(50, 55);
}

static void tabla_codec_enable_clock_block(struct snd_soc_component *component)
{
	/* MCLK only: make sure the RCO/config-mode path is off. */
	snd_soc_component_update_bits(component, TABLA_A_CLK_BUFF_EN1,
				      0x08, 0x00);

	snd_soc_component_update_bits(component, TABLA_A_CLK_BUFF_EN1,
				      0x01, 0x01);
	snd_soc_component_update_bits(component, TABLA_A_CLK_BUFF_EN2,
				      0x02, 0x00);
	snd_soc_component_update_bits(component, TABLA_A_CLK_BUFF_EN2,
				      0x04, 0x04);
	snd_soc_component_update_bits(component, TABLA_A_CDC_CLK_MCLK_CTL,
				      0x01, 0x01);
	usleep_range(50, 55);
}

static int tabla_event_cdc_clk(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		tabla_codec_disable_clock_block(component);
		snd_soc_component_update_bits(component,
					      TABLA_A_BIAS_CENTRAL_BG_CTL,
					      0x80, 0x80);
		snd_soc_component_update_bits(component,
					      TABLA_A_BIAS_CENTRAL_BG_CTL,
					      0x04, 0x04);
		snd_soc_component_update_bits(component,
					      TABLA_A_BIAS_CENTRAL_BG_CTL,
					      0x01, 0x01);
		usleep_range(1000, 1100);
		snd_soc_component_update_bits(component,
					      TABLA_A_BIAS_CENTRAL_BG_CTL,
					      0x80, 0x00);
		tabla_codec_enable_clock_block(component);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tabla_codec_disable_clock_block(component);
		snd_soc_component_update_bits(component,
					      TABLA_A_BIAS_CENTRAL_BG_CTL,
					      0x05, 0x00);
		break;
	}
	return 0;
}

static int tabla_event_reset_interpolator(struct snd_soc_dapm_widget *w,
					  struct snd_kcontrol *kcontrol,
					  int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	static const u16 rx_vol_reg[] = {
		[0] = TABLA_A_CDC_RX1_VOL_CTL_B2_CTL,
		[1] = TABLA_A_CDC_RX2_VOL_CTL_B2_CTL,
		[2] = TABLA_A_CDC_RX3_VOL_CTL_B2_CTL,
		[4] = TABLA_A_CDC_RX5_VOL_CTL_B2_CTL,
		[5] = TABLA_A_CDC_RX6_VOL_CTL_B2_CTL,
	};

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component,
			TABLA_A_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 1 << w->shift);
		snd_soc_component_update_bits(component,
			TABLA_A_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* re-apply the digital gain after the interpolator is on */
		if (w->shift < ARRAY_SIZE(rx_vol_reg) && rx_vol_reg[w->shift]) {
			unsigned int val;

			val = snd_soc_component_read(component,
						     rx_vol_reg[w->shift]);
			snd_soc_component_write(component, rx_vol_reg[w->shift],
						val);
		}
		break;
	}

	return 0;
}

static int tabla_event_rx_dac(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, w->reg, 0x40, 0x40);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, w->reg, 0x40, 0x00);
		break;
	}

	return 0;
}

static int tabla_event_lineout_pa(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	unsigned int gain_reg;

	if (w->shift > 3)
		return -EINVAL;
	gain_reg = TABLA_A_RX_LINE_1_GAIN + 4 * w->shift;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Prepare the PA before DAPM switches the lineout enable bits. */
		snd_soc_component_update_bits(component, gain_reg, 0x40, 0x40);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* Allow the lineout PA to settle. */
		usleep_range(16, 17);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, gain_reg, 0x40, 0x00);
		break;
	}

	return 0;
}

static int tabla_event_rx_bias(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	unsigned int val;

	val = snd_soc_component_read(component, TABLA_A_RX_COM_BIAS);
	return 0;
}

static int tabla_event_cp(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * Downstream waits for the charge-pump supply to settle, then
		 * enables the CLSG clock and releases the CP static clamp.
		 */
		usleep_range(15000, 16000);
		snd_soc_component_update_bits(component,
					      TABLA_A_CDC_CLK_OTHR_CTL,
					      0x01, 0x01);
		snd_soc_component_update_bits(component,
					      TABLA_A_CDC_CLSG_CTL,
					      0x08, 0x08);
		usleep_range(200, 220);
		snd_soc_component_update_bits(component,
					      TABLA_A_CP_STATIC,
					      0x10, 0x00);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component,
					      TABLA_A_CDC_CLK_OTHR_RESET_CTL,
					      0x10, 0x10);
		usleep_range(20, 22);
		snd_soc_component_update_bits(component,
					      TABLA_A_CP_STATIC,
					      0x08, 0x08);
		snd_soc_component_update_bits(component,
					      TABLA_A_CP_STATIC,
					      0x10, 0x10);
		snd_soc_component_update_bits(component,
					      TABLA_A_CDC_CLSG_CTL,
					      0x08, 0x00);
		snd_soc_component_update_bits(component,
					      TABLA_A_CDC_CLK_OTHR_CTL,
					      0x01, 0x00);
		snd_soc_component_update_bits(component,
					      TABLA_A_CDC_CLK_OTHR_RESET_CTL,
					      0x10, 0x00);
		snd_soc_component_update_bits(component,
					      TABLA_A_CP_STATIC,
					      0x08, 0x00);
		break;
	}

	return 0;
}

/*
 * Earpiece DAC/PA event, mirroring downstream tabla_ear_pa_event: the EAR
 * DAC and PA share TABLA_A_RX_EAR_EN. The widget controls the DAC1 switch
 * (bit 5); downstream also enables PA bits 4 and 6 on PRE_PMU.
 */
static int tabla_event_ear_dac(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, TABLA_A_RX_EAR_EN,
					      0x50, 0x50);
		usleep_range(16, 17);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, TABLA_A_RX_EAR_EN,
					      0x50, 0x00);
		break;
	}

	return 0;
}

/* Select the interpolator output rather than the disabled compander. */
static int tabla_event_hph_chain(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	unsigned int reg = w->shift ? TABLA_A_CDC_RX2_B6_CTL :
				     TABLA_A_CDC_RX1_B6_CTL;

	return snd_soc_component_update_bits(component, reg, BIT(5),
			event == SND_SOC_DAPM_POST_PMU ? BIT(5) : 0);
}

static int tabla_event_hph_pa(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	/* Allow the headphone outputs to settle before disabling their DACs. */
	usleep_range(10000, 11000);

	return 0;
}

static const char * const rx_mix1_text[] = { "ZERO", "RX1", "RX2" };
static const unsigned int rx_mix1_values[] = { 0, 5, 6 };

static const char * const rx_dsm_text[] = {
	"CIC_OUT", "DSM_INV"
};

static SOC_VALUE_ENUM_SINGLE_DECL(rx1_mix1_inp1_enum, TABLA_A_CDC_CONN_RX1_B1_CTL,
					0, 0xf, rx_mix1_text, rx_mix1_values);

static SOC_VALUE_ENUM_SINGLE_DECL(rx2_mix1_inp1_enum, TABLA_A_CDC_CONN_RX2_B1_CTL,
					0, 0xf, rx_mix1_text, rx_mix1_values);

static SOC_VALUE_ENUM_SINGLE_DECL(rx3_mix1_inp1_enum, TABLA_A_CDC_CONN_RX3_B1_CTL,
					0, 0xf, rx_mix1_text, rx_mix1_values);

static SOC_VALUE_ENUM_SINGLE_DECL(rx5_mix1_inp1_enum, TABLA_A_CDC_CONN_RX5_B1_CTL,
					0, 0xf, rx_mix1_text, rx_mix1_values);

static SOC_ENUM_SINGLE_DECL(rx6_dsm_enum, TABLA_A_CDC_RX6_B6_CTL, 4,
			    rx_dsm_text);

static const struct snd_kcontrol_new rx1_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx1_mix1_inp1_enum);
static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_enum);
static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp1_enum);
static const struct snd_kcontrol_new rx5_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX5 MIX1 INP1 Mux", rx5_mix1_inp1_enum);
static const struct snd_kcontrol_new rx6_dsm_mux =
	SOC_DAPM_ENUM("RX6 DSM MUX", rx6_dsm_enum);

static DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static DECLARE_TLV_DB_SCALE(digital_gain, -8400, 100, 1);

static const struct snd_kcontrol_new tabla_snd_controls[] = {
	/* Downstream uses inverted gain: higher control value -> lower raw */

	SOC_SINGLE("HPHL Volume", TABLA_A_RX_HPH_L_GAIN, 0, 12, 1),
	SOC_SINGLE("HPHR Volume", TABLA_A_RX_HPH_R_GAIN, 0, 12, 1),

	SOC_SINGLE_TLV("LINEOUT1 Volume", TABLA_A_RX_LINE_1_GAIN, 0, 12, 1,
		       line_gain),

	SOC_SINGLE_TLV("LINEOUT2 Volume", TABLA_A_RX_LINE_2_GAIN, 0, 12, 1,
		       line_gain),

	SOC_SINGLE_TLV("LINEOUT3 Volume", TABLA_A_RX_LINE_3_GAIN, 0, 12, 1,
		       line_gain),

	SOC_SINGLE_TLV("LINEOUT4 Volume", TABLA_A_RX_LINE_4_GAIN, 0, 12, 1,
		       line_gain),
	SOC_SINGLE_S8_TLV("RX1 Digital Volume",
			  TABLA_A_CDC_RX1_VOL_CTL_B2_CTL, -84, 40,
			  digital_gain),

	SOC_SINGLE_S8_TLV("RX2 Digital Volume",
			  TABLA_A_CDC_RX2_VOL_CTL_B2_CTL, -84, 40,
			  digital_gain),

	SOC_SINGLE_S8_TLV("RX3 Digital Volume",
			  TABLA_A_CDC_RX3_VOL_CTL_B2_CTL, -84, 40,
			  digital_gain),

	SOC_SINGLE_S8_TLV("RX5 Digital Volume",
			  TABLA_A_CDC_RX5_VOL_CTL_B2_CTL, -84, 40,
			  digital_gain),
	SOC_SINGLE_S8_TLV("RX6 Digital Volume",
			  TABLA_A_CDC_RX6_VOL_CTL_B2_CTL, -84, 40,
			  digital_gain),
};

static const struct snd_soc_dapm_widget tabla_dapm_widgets[] = {
	/*
	 * The core creates a playback DAI widget named after the DAI stream
	 * name ("AIF1 Playback").  Add an explicit AIF_IN widget for the codec's
	 * SLIMbus playback port so DAPM sees the SLIMbus RX stage as a real node in
	 * the path, matching downstream's topology.
	 */
	SND_SOC_DAPM_AIF_IN("SLIM RX", "AIF1 Playback", 0, SND_SOC_NOPM, 0,
			    0),

	SND_SOC_DAPM_SUPPLY("CDC CLK", SND_SOC_NOPM, 0, 0,
			    tabla_event_cdc_clk,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("RX_BIAS", TABLA_A_RX_COM_BIAS, 7, 0,
			    tabla_event_rx_bias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("CP", TABLA_A_CP_EN, 0, 0,
			    tabla_event_cp,
			    SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MUX("RX1 MIX1 INP1 Mux", SND_SOC_NOPM, 0, 0,
			 &rx1_mix1_inp1_mux),
	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER_E("RX1 MIX2", TABLA_A_CDC_CLK_RX_B1_CTL, 0, 0,
			     NULL, 0, tabla_event_reset_interpolator,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_DAC_E("EAR DAC", NULL, TABLA_A_RX_EAR_EN, 5, 0,
			   tabla_event_ear_dac,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MUX("RX2 MIX1 INP1 Mux", SND_SOC_NOPM, 0, 0,
			 &rx2_mix1_inp1_mux),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER_E("RX2 MIX2", TABLA_A_CDC_CLK_RX_B1_CTL, 1, 0,
			     NULL, 0, tabla_event_reset_interpolator,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_E("RX1 CHAIN", SND_SOC_NOPM, 0, 0, NULL, 0,
			   tabla_event_hph_chain,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("RX2 CHAIN", SND_SOC_NOPM, 1, 0, NULL, 0,
			   tabla_event_hph_chain,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HPHL DAC", NULL, TABLA_A_RX_HPH_L_DAC_CTL, 7, 0,
			   tabla_event_rx_dac,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HPHR DAC", NULL, TABLA_A_RX_HPH_R_DAC_CTL, 7, 0,
			   tabla_event_rx_dac,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_S("HPHL PA", 1, TABLA_A_RX_HPH_CNP_EN, 5, 0,
			   tabla_event_hph_pa, SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_S("HPHR PA", 1, TABLA_A_RX_HPH_CNP_EN, 4, 0,
			   tabla_event_hph_pa, SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("RX5 MIX1 INP1 Mux", SND_SOC_NOPM, 0, 0,
			 &rx5_mix1_inp1_mux),

	SND_SOC_DAPM_MIXER_E("RX5 MIX1", TABLA_A_CDC_CLK_RX_B1_CTL, 4, 0,
			     NULL, 0, tabla_event_reset_interpolator,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX6 DSM MUX", TABLA_A_CDC_CLK_RX_B1_CTL, 5, 0,
			   &rx6_dsm_mux, tabla_event_reset_interpolator,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_DAC_E("LINEOUT2 DAC", NULL, TABLA_A_RX_LINE_2_DAC_CTL,
			   7, 0, tabla_event_rx_dac,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("LINEOUT4 DAC", NULL, TABLA_A_RX_LINE_4_DAC_CTL,
			   7, 0, tabla_event_rx_dac,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("RX3 MIX1 INP1 Mux", SND_SOC_NOPM, 0, 0,
			 &rx3_mix1_inp1_mux),
	SND_SOC_DAPM_MIXER_E("RX3 MIX1", TABLA_A_CDC_CLK_RX_B1_CTL, 2, 0,
			     NULL, 0, tabla_event_reset_interpolator,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_DAC_E("LINEOUT1 DAC", NULL, TABLA_A_RX_LINE_1_DAC_CTL,
			   7, 0, tabla_event_rx_dac,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("LINEOUT3 DAC", NULL, TABLA_A_RX_LINE_3_DAC_CTL,
			   7, 0, tabla_event_rx_dac,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("LINEOUT2 PA", TABLA_A_RX_LINE_CNP_EN, 1, 0,
			   NULL, 0, tabla_event_lineout_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("LINEOUT4 PA", TABLA_A_RX_LINE_CNP_EN, 3, 0,
			   NULL, 0, tabla_event_lineout_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("LINEOUT1 PA", TABLA_A_RX_LINE_CNP_EN, 0, 0,
			   NULL, 0, tabla_event_lineout_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("LINEOUT3 PA", TABLA_A_RX_LINE_CNP_EN, 2, 0,
			   NULL, 0, tabla_event_lineout_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_OUTPUT("LINEOUT1"),

	SND_SOC_DAPM_OUTPUT("LINEOUT2"),

	SND_SOC_DAPM_OUTPUT("LINEOUT3"),

	SND_SOC_DAPM_OUTPUT("LINEOUT4"),
	SND_SOC_DAPM_OUTPUT("EAR"),
	SND_SOC_DAPM_OUTPUT("HPHL"),
	SND_SOC_DAPM_OUTPUT("HPHR"),
};

static const struct snd_soc_dapm_route tabla_dapm_routes[] = {
	/* Select the input matching the mono channel map on the codec DAI. */
	{ "RX5 MIX1 INP1 Mux", "RX1", "SLIM RX" },
	{ "RX5 MIX1", NULL, "RX5 MIX1 INP1 Mux" },
	{ "RX6 DSM MUX", "DSM_INV", "RX5 MIX1" },

	/* Differential lineout pair through RX5/RX6 (top speaker amp). */
	{ "LINEOUT2 DAC", NULL, "RX5 MIX1" },
	{ "LINEOUT4 DAC", NULL, "RX6 DSM MUX" },

	/*
	 * Bottom amplifier: a stereo pair, LINEOUT1 carries the right channel
	 * through the RX3 chain (RX3 MIX1 INP1 = RX2) and LINEOUT3 the left
	 * channel through RX5 (INP1 = RX1). The flo downstream mixer paths
	 * set RX3 MIX1 INP1 = RX2, RX4 MIX1 INP1 = RX1 and leave both DSM
	 * muxes on CIC_OUT, so LINEOUT3 is the left channel and not an
	 * inverted copy of LINEOUT1.
	 */
	{ "RX3 MIX1 INP1 Mux", "RX1", "SLIM RX" },
	{ "RX3 MIX1 INP1 Mux", "RX2", "SLIM RX" },
	{ "RX3 MIX1", NULL, "RX3 MIX1 INP1 Mux" },
	{ "LINEOUT1 DAC", NULL, "RX3 MIX1" },
	{ "LINEOUT3 DAC", NULL, "RX5 MIX1" },

	/*
	 * Earpiece: the selected mono SLIMbus playback port into the RX1
	 * chain, mirroring downstream RX1 MIX1 -> RX1 MIX2 -> DAC1 -> EAR PA.
	 */
	{ "RX1 MIX1 INP1 Mux", "RX1", "SLIM RX" },
	{ "RX1 MIX1", NULL, "RX1 MIX1 INP1 Mux" },
	{ "RX1 MIX2", NULL, "RX1 MIX1" },
	{ "EAR DAC", NULL, "RX1 MIX2" },
	{ "EAR", NULL, "EAR DAC" },

	/*
	 * The CPU DAI transmits two SLIM slots: slot 0 (RX1, left) and
	 * slot 1 (RX2, right). HPHL takes slot 0 (see RX1 MIX1 INP1 Mux
	 * above); HPHR takes slot 1 so the two headphone channels are
	 * independent.
	 */
	{ "RX2 MIX1 INP1 Mux", "RX1", "SLIM RX" },
	{ "RX2 MIX1 INP1 Mux", "RX2", "SLIM RX" },
	{ "RX2 MIX1", NULL, "RX2 MIX1 INP1 Mux" },
	{ "RX2 MIX2", NULL, "RX2 MIX1" },
	{ "RX1 CHAIN", NULL, "RX1 MIX2" },
	{ "RX2 CHAIN", NULL, "RX2 MIX2" },
	{ "HPHL DAC", NULL, "RX1 CHAIN" },
	{ "HPHR DAC", NULL, "RX2 CHAIN" },
	{ "HPHL PA", NULL, "HPHL DAC" },
	{ "HPHR PA", NULL, "HPHR DAC" },
	{ "HPHL", NULL, "HPHL PA" },
	{ "HPHR", NULL, "HPHR PA" },
	{ "HPHL DAC", NULL, "CP" },
	{ "HPHR DAC", NULL, "CP" },
	{ "RX2 MIX2", NULL, "RX_BIAS" },
	{ "RX2 MIX2", NULL, "CDC CLK" },

	{ "LINEOUT2 PA", NULL, "LINEOUT2 DAC" },
	{ "LINEOUT4 PA", NULL, "LINEOUT4 DAC" },
	{ "LINEOUT1 PA", NULL, "LINEOUT1 DAC" },
	{ "LINEOUT3 PA", NULL, "LINEOUT3 DAC" },

	{ "LINEOUT2", NULL, "LINEOUT2 PA" },
	{ "LINEOUT4", NULL, "LINEOUT4 PA" },
	{ "LINEOUT1", NULL, "LINEOUT1 PA" },
	{ "LINEOUT3", NULL, "LINEOUT3 PA" },

	{ "RX1 MIX2", NULL, "RX_BIAS" },
	{ "RX1 MIX2", NULL, "CDC CLK" },
	{ "RX5 MIX1", NULL, "RX_BIAS" },
	{ "RX5 MIX1", NULL, "CDC CLK" },
	{ "RX3 MIX1", NULL, "RX_BIAS" },
	{ "RX3 MIX1", NULL, "CDC CLK" },
	{ "RX6 DSM MUX", NULL, "CDC CLK" },

	{ "CP", NULL, "RX_BIAS" },
	/* Downstream routes the earpiece DAC through the charge pump. */
	{ "EAR DAC", NULL, "CP" },
	{ "EAR DAC", NULL, "RX_BIAS" },
	/*
	 * Downstream does not route LINEOUT DACs through CP; CP/CLSG is for
	 * headphone/earpiece negative rail.  Keep LINEOUT DACs on RX_BIAS only.
	 */
	{ "LINEOUT2 DAC", NULL, "RX_BIAS" },
	{ "LINEOUT4 DAC", NULL, "RX_BIAS" },
	{ "LINEOUT1 DAC", NULL, "RX_BIAS" },
	{ "LINEOUT3 DAC", NULL, "RX_BIAS" },

};

static int tabla_component_probe(struct snd_soc_component *component)
{
	struct tabla *tabla = dev_get_drvdata(component->dev);

	/* Codec registers belong to the PGD, not the interface device. */
	snd_soc_component_init_regmap(component, tabla->regmap);

	return 0;
}

static const struct snd_soc_component_driver tabla_component_driver = {
	.name			= "tabla",
	.probe			= tabla_component_probe,
	.controls		= tabla_snd_controls,
	.num_controls		= ARRAY_SIZE(tabla_snd_controls),
	.dapm_widgets		= tabla_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tabla_dapm_widgets),
	.dapm_routes		= tabla_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tabla_dapm_routes),
};

/* Number of RX slave ports used for the (up to) 2-channel headphone path. */
#define TABLA_IFC_RX_PORT_COUNT		2

static int tabla_ifc_port_enable(struct tabla *tabla, bool enable)
{
	int ret, i;

	/*
	 * Each RX channel lands on its own SLIM slave port: channel 0 on
	 * port 10 (RX1), channel 1 on port 11 (RX2). Both ports must be
	 * programmed, or the other channel stays silent.
	 *
	 * Make every port a group of its own (mask relative to the first RX
	 * port, as downstream does for single-port playback). Grouping both
	 * ports together makes port 10 stall unless port 11 is also being
	 * drained by an interpolator, which silences outputs that only use
	 * RX1, such as the rear speaker and the earpiece.
	 */
	for (i = 0; i < TABLA_IFC_RX_PORT_COUNT; i++) {
		if (enable) {
			ret = regmap_write(tabla->ifc_regmap,
					   TABLA_IFC_RX_MCH_BASE + 4 * i,
					   BIT(i));
			if (ret)
				return ret;
		}
		ret = regmap_write(tabla->ifc_regmap,
				TABLA_IFC_PORT_CFG_BASE + i,
					    enable ? TABLA_IFC_PORT_WM_ENABLE : 0);
		if (ret)
			return ret;
	}
	return 0;
}

static int tabla_dai_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct tabla *tabla = dev_get_drvdata(dai->dev);
	int ret;

	if (params_rate(params) != 48000 ||
	    (params_channels(params) != 1 && params_channels(params) != 2) ||
	    params_format(params) != SNDRV_PCM_FORMAT_S16_LE)
		return -EINVAL;
	ret = regmap_update_bits(tabla->regmap, TABLA_A_CDC_RX1_B5_CTL,
				 0xe0, 0x60);
	if (ret)
		return ret;
	ret = regmap_update_bits(tabla->regmap, TABLA_A_CDC_RX3_B5_CTL,
				 0xe0, 0x60);
	if (ret)
		return ret;
	ret = regmap_update_bits(tabla->regmap, TABLA_A_CDC_RX2_B5_CTL,
				 0xe0, 0x60);
	if (ret)
		return ret;
	ret = regmap_update_bits(tabla->regmap, TABLA_A_CDC_RX5_B5_CTL,
				 0xe0, 0x60);
	if (ret)
		return ret;
	return regmap_update_bits(tabla->regmap, TABLA_A_CDC_RX6_B5_CTL,
				  0xe0, 0x60);
}

static int tabla_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct tabla *tabla = dev_get_drvdata(dai->dev);
	bool enable;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		enable = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		enable = false;
		break;
	default:
		return -EINVAL;
	}

	if (tabla->ifc_port_enabled == enable)
		return 0;
	/* Host trigger order keeps DMA priming before channel activation. */
	ret = tabla_ifc_port_enable(tabla, enable);
	if (!ret)
		tabla->ifc_port_enabled = enable;
	return ret;
}

static int tabla_dai_set_channel_map(struct snd_soc_dai *dai,
				     unsigned int tx_num,
				     const unsigned int *tx_slot,
				     unsigned int rx_num,
				     const unsigned int *rx_slot)
{
	/* The playback interface exposes one or two SLIM RX channels. */
	if (tx_num || !rx_slot)
		return -EINVAL;
	if (rx_num == 1)
		return rx_slot[0] == 138 ? 0 : -EINVAL;
	if (rx_num == 2)
		return (rx_slot[0] == 138 && rx_slot[1] == 139) ? 0 : -EINVAL;
	return -EINVAL;
}

static const struct snd_soc_dai_ops tabla_dai_ops = {
	.hw_params	= tabla_dai_hw_params,
	.trigger	= tabla_dai_trigger,
	.set_channel_map = tabla_dai_set_channel_map,
};

static struct snd_soc_dai_driver tabla_dai = {
	.name = "tabla_rx1",
	.playback = {
		.stream_name = "AIF1 Playback",
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
	},
	.ops = &tabla_dai_ops,
};
static int tabla_bring_up(struct regmap *regmap)
{
	int ret;

	/* downstream wcd9xxx_bring_up sequence */
	ret = regmap_write(regmap, TABLA_A_LEAKAGE_CTL, 0x04);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_CTL, 0x00);
	if (ret)
		return ret;
	usleep_range(5000, 5500);
	ret = regmap_write(regmap, TABLA_A_CDC_CTL, 0x03);
	if (ret)
		return ret;

	return regmap_write(regmap, TABLA_A_LEAKAGE_CTL, 0x03);
}

static int tabla_check_chip_id(struct device *dev, struct regmap *regmap)
{
	unsigned int id[4];
	int ret, i;

	for (i = 0; i < 4; i++) {
		ret = regmap_read(regmap, TABLA_A_CHIP_ID_BYTE_0 + i, &id[i]);
		if (ret)
			return ret;
	}
	/* This interface supports the tabla2x device. */
	if (id[1] || id[2] || id[3] != 1 || id[0] != 2) {
		dev_err(dev, "unrecognized codec id\n");
		return -ENODEV;
	}

	return 0;
}

/* Static register init subset from downstream tabla_codec_probe */
static int tabla_init_regs(struct regmap *regmap)
{
	int ret;

	/* Use the tested mid-range analog gains for generic mixer paths. */
	ret = regmap_update_bits(regmap, TABLA_A_RX_HPH_L_GAIN, 0x1f, 0x16);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_RX_HPH_R_GAIN, 0x1f, 0x16);
	if (ret)
		return ret;

	/* use register gain for lineouts */
	ret = regmap_update_bits(regmap, TABLA_A_RX_LINE_1_GAIN, 0x10, 0x10);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_RX_LINE_2_GAIN, 0x1f, 0x16);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_RX_LINE_3_GAIN, 0x10, 0x10);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_RX_LINE_4_GAIN, 0x1f, 0x16);
	if (ret)
		return ret;

	/*
	 * Over-current protection settings from downstream
	 * tabla_codec_reg_init_val.  Without these the lineout PAs can stay
	 * shut down.
	 */
	ret = regmap_update_bits(regmap, TABLA_A_RX_HPH_OCP_CTL, 0xe0, 0x60);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_RX_COM_OCP_COUNT, 0xff, 0xff);
	if (ret)
		return ret;

	/* RX interpolator defaults */
	ret = regmap_write(regmap, TABLA_A_CDC_RX2_B5_CTL, 0x78);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX2_B6_CTL, 0x80);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX3_B5_CTL, 0x78);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX3_B6_CTL, 0x80);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX4_B5_CTL, 0x78);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX4_B6_CTL, 0x80);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX5_B5_CTL, 0x78);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX5_B6_CTL, 0x80);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX6_B5_CTL, 0x78);
	if (ret)
		return ret;
	/* LINEOUT2/4 drive the speaker amplifier as a differential pair. */
	ret = regmap_write(regmap, TABLA_A_CDC_RX6_B6_CTL, 0x90);
	if (ret)
		return ret;

	/* RX1/2 start at -6 dB; the other interpolators pass through at 0 dB. */
	ret = regmap_write(regmap, TABLA_A_CDC_RX1_VOL_CTL_B2_CTL, 0xfa);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX2_VOL_CTL_B2_CTL, 0xfa);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX3_VOL_CTL_B2_CTL, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX4_VOL_CTL_B2_CTL, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX5_VOL_CTL_B2_CTL, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_RX6_VOL_CTL_B2_CTL, 0x00);
	if (ret)
		return ret;

	/*
	 * Route the SLIM RX slots to the output chains before DAPM creates the
	 * mux paths.  The mux registers must agree with the DAPM routes: DAPM
	 * only powers a chain when the register selects the source named in the
	 * route table.  HPHL/top speaker/EAR use slot 0 ("RX1"); HPHR and the bottom
	 * speaker use slot 1
	 * ("RX2") so the two headphone channels are independent.
	 */
	ret = regmap_update_bits(regmap, TABLA_A_CDC_CONN_RX1_B1_CTL, 0x0f, 0x05);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_CDC_CONN_RX2_B1_CTL, 0x0f, 0x06);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_CDC_CONN_RX3_B1_CTL, 0x0f, 0x06);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, TABLA_A_CDC_CONN_RX5_B1_CTL, 0x0f, 0x05);
	if (ret)
		return ret;

	/* 16-bit RX sample size on the SLIMbus interface */
	ret = regmap_write(regmap, TABLA_A_CDC_CONN_RX_SB_B1_CTL, 0xaa);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_CONN_RX_SB_B2_CTL, 0xaa);
	if (ret)
		return ret;

	/* Bias calibration for a 2.2 V analog supply. */
	ret = regmap_write(regmap, TABLA_A_QFUSE_CTL, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(regmap, TABLA_A_BIAS_REF_CTL, 0x1e);
	if (ret)
		return ret;

	/* connection-classifier gain setting from downstream init table */
	ret = regmap_update_bits(regmap, TABLA_A_CDC_CONN_CLSG_CTL, 0x3c, 0x14);
	if (ret)
		return ret;

	/* Class-G switching thresholds from the downstream init table. */
	ret = regmap_write(regmap, TABLA_A_CDC_CLSG_FREQ_THRESH_B1_CTL, 0x02);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_CLSG_FREQ_THRESH_B2_CTL, 0x05);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_CLSG_FREQ_THRESH_B3_CTL, 0x06);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_CLSG_FREQ_THRESH_B4_CTL, 0x0c);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_CDC_CLSG_GAIN_THRESH_CTL, 0x0d);
	if (ret)
		return ret;

	/* Shared bias defaults, also required on Tabla 2.x. */
	ret = regmap_write(regmap, TABLA_A_MICB_1_INT_RBIAS, 0x24);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_MICB_2_INT_RBIAS, 0x24);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_MICB_3_INT_RBIAS, 0x24);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_RX_HPH_BIAS_PA, 0x57);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_RX_HPH_BIAS_LDO, 0x56);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_RX_EAR_BIAS_PA, 0xa6);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_RX_EAR_GAIN, 0x02);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_RX_EAR_VCM, 0x03);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_RX_LINE_5_GAIN, 0x10);
	if (ret)
		return ret;

	/*
	 * Tabla 2.0-specific defaults from downstream tabla_2_0_reg_defaults.
	 * These are applied unconditionally on 2.x silicon.
	 */
	ret = regmap_write(regmap, TABLA_A_MICB_2_MBHC, 0x02);
	if (ret)
		return ret;
	ret = regmap_write(regmap, TABLA_A_TX_5_6_EN, 0x81);
	if (ret)
		return ret;
	return regmap_write(regmap, TABLA_2_A_MICB_4_INT_RBIAS, 0x24);
}

static void tabla_put_ifc(void *data)
{
	struct tabla *tabla = data;

	if (tabla->ifc_sdev)
		put_device(&tabla->ifc_sdev->dev);
}

static int tabla_slim_init(struct slim_device *sdev,
			     enum slim_device_status status)
{
	struct tabla *tabla = slim_get_devicedata(sdev);
	struct device *dev = &sdev->dev;
	int ret, i;

	if (status != SLIM_DEVICE_STATUS_UP || tabla->registered)
		return 0;
	if (sdev->status != SLIM_DEVICE_STATUS_UP || !sdev->is_laddr_valid)
		return -ETIMEDOUT;

	/*
	 * The interface device may not have reported present yet when the
	 * PGD device comes up.  Retry while keeping the controller awake so
	 * the bus is active for the interface device to report present and
	 * receive a logical address.  Match/extend downstream's
	 * SLIMBUS_PRESENT_TIMEOUT behaviour.
	 */
	ret = pm_runtime_resume_and_get(sdev->dev.parent);
	if (ret < 0) {
		dev_err_probe(dev, ret,
			      "failed to resume slimbus controller for IFD\n");
		return ret;
	}
	{
		int retry = 500;

		do {
			ret = slim_get_logical_addr(tabla->ifc_sdev);
			if (!ret)
				break;
			usleep_range(1000, 1100);
		} while (--retry);
	}
	pm_runtime_put_sync(sdev->dev.parent);
	if (ret) {
		/*
		 * slim_get_logical_addr() reports every assignment failure
		 * as -EINVAL. The only reason it fails here is that the
		 * interface device did not answer on the bus, which is the
		 * "codec unreachable" case, not an invalid argument.
		 */
		dev_dbg(dev, "interface device has no logical address (%d)\n",
			ret);
		return -ETIMEDOUT;
	}

	/*
	 * init can run twice in one probe (a stale "already present" status
	 * leads to a failed init and a retry after the reset cycle); reuse
	 * the regmaps instead of re-creating them.
	 */
	if (IS_ERR_OR_NULL(tabla->regmap)) {
		tabla->regmap = devm_regmap_init(dev, NULL, sdev,
						 &tabla_regmap_config);
		if (IS_ERR(tabla->regmap))
			return dev_err_probe(dev, PTR_ERR(tabla->regmap),
					     "regmap init failed\n");
	}

	if (IS_ERR_OR_NULL(tabla->ifc_regmap)) {
		tabla->ifc_regmap = devm_regmap_init(dev, NULL, tabla->ifc_sdev,
						 &tabla_regmap_config);
		if (IS_ERR(tabla->ifc_regmap))
			return dev_err_probe(dev, PTR_ERR(tabla->ifc_regmap),
					     "interface device regmap init failed\n");
	}

	/* Mask port interrupts until a handler is available to clear them. */
	for (i = 0; i < 3; i++) {
		ret = regmap_write(tabla->ifc_regmap,
				   TABLA_SLIM_PGD_PORT_INT_EN0 + i, 0x00);
		if (ret)
			return ret;
	}

	ret = tabla_bring_up(tabla->regmap);
	if (ret)
		return dev_err_probe(dev, ret, "codec bring-up failed\n");

	ret = tabla_check_chip_id(dev, tabla->regmap);
	if (ret)
		return ret;

	ret = tabla_init_regs(tabla->regmap);
	if (ret)
		return dev_err_probe(dev, ret, "codec init failed\n");

	ret = devm_snd_soc_register_component(dev, &tabla_component_driver,
					      &tabla_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "component register failed\n");

	tabla->registered = true;

	return 0;
}

static int tabla_slim_status(struct slim_device *sdev,
			     enum slim_device_status status)
{
	struct tabla *tabla = slim_get_devicedata(sdev);
	int ret;

	if (status != SLIM_DEVICE_STATUS_UP || !tabla || !tabla->mclk_enabled)
		return 0;
	mutex_lock(&tabla->init_lock);
	ret = tabla_slim_init(sdev, status);
	mutex_unlock(&tabla->init_lock);
	return ret;
}

static const char * const tabla_supply_names[] = {
	"cdc-vdd-cp",
	"cdc-vdda-rx",
	"cdc-vdda-tx",
	"vddio-cdc",
	"vddd-cdc-d",
	"cdc-vdda-a-1p2v",
};

static void tabla_slim_disable_hw(void *data)
{
	struct tabla *tabla = data;

	if (tabla->mclk_enabled) {
		clk_disable_unprepare(tabla->mclk);
		tabla->mclk_enabled = false;
	}
	if (tabla->powered) {
		regulator_bulk_disable(ARRAY_SIZE(tabla->supplies), tabla->supplies);
		tabla->powered = false;
	}
}

/*
 * Errors that mean "the codec did not answer" (as opposed to a real
 * software fault).  Those are transient: the reset-cycle recovery
 * below, or a later deferred probe, can fix them.
 */
static bool tabla_slim_codec_unreachable(int ret)
{
	return ret == -ETIMEDOUT || ret == -ENOTCONN || ret == -EIO ||
	       ret == -EACCES || ret == -EBUSY;
}

/* Called under init_lock so enumeration cannot initialise a half-reset codec. */
static int tabla_slim_reset_cycle(struct slim_device *sdev,
				  struct tabla *tabla, unsigned int duration_ms)
{
	int ret;

	if (!tabla->reset)
		return -ENODEV;

	ret = gpiod_set_value_cansleep(tabla->reset, 1);
	if (ret)
		return ret;
	msleep(duration_ms);

	/* Both functions lose their logical addresses during hardware reset. */
	if (sdev->is_laddr_valid)
		slim_report_absent(sdev);
	if (tabla->ifc_sdev->is_laddr_valid)
		slim_report_absent(tabla->ifc_sdev);

	ret = gpiod_set_value_cansleep(tabla->reset, 0);
	if (ret)
		return ret;
	msleep(20);
	return 0;
}

static int tabla_slim_probe(struct slim_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct device_node *ifc_np;
	struct tabla *tabla;
	int i, ret;

	tabla = devm_kzalloc(dev, sizeof(*tabla), GFP_KERNEL);
	if (!tabla)
		return -ENOMEM;

	slim_set_devicedata(sdev, tabla);
	mutex_init(&tabla->init_lock);
	ret = devm_add_action_or_reset(dev, tabla_put_ifc, tabla);
	if (ret)
		return ret;

	/* Resolve both functions before resetting their shared hardware. */
	ifc_np = of_parse_phandle(dev->of_node, "slim-ifc-dev", 0);
	if (!ifc_np)
		return -ENODEV;
	tabla->ifc_sdev = of_slim_get_device(sdev->ctrl, ifc_np);
	of_node_put(ifc_np);
	if (!tabla->ifc_sdev)
		return -EPROBE_DEFER;

	for (i = 0; i < ARRAY_SIZE(tabla->supplies); i++)
		tabla->supplies[i].supply = tabla_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(tabla->supplies),
				      tabla->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	/*
	 * RESET_B is active low.  Drive it to the deassert level from
	 * the start: that makes the boot-time pin state deterministic
	 * (a floating low would hold the codec in reset) and lets the
	 * codec enumerate on its own before the first register access.
	 */
	tabla->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tabla->reset))
		return dev_err_probe(dev, PTR_ERR(tabla->reset),
				     "failed to get reset gpio\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(tabla->supplies),
				    tabla->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");
	tabla->powered = true;

	/* MCLK is the audio master clock; enable it up front. */
	tabla->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(tabla->mclk)) {
		ret = dev_err_probe(dev, PTR_ERR(tabla->mclk),
				     "failed to get mclk\n");
		goto err_disable;
	}
	ret = clk_set_rate(tabla->mclk, TABLA_MCLK_RATE);
	if (ret) {
		dev_err_probe(dev, ret, "failed to set mclk rate\n");
		goto err_disable;
	}
	ret = clk_prepare_enable(tabla->mclk);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enable mclk\n");
		goto err_disable;
	}
	/*
	 * Keep the bus active before the codec is reset: it reports present
	 * on the bus as soon as RESET_B is released, and the controller has to
	 * be running to receive that report and assign the logical addresses.
	 * With the controller runtime-suspended the assignment is rejected
	 * (-ENOTCONN) and the codec comes up without addresses, which then
	 * needs a second, 1 s reset cycle.
	 */
	ret = pm_runtime_resume_and_get(sdev->dev.parent);
	if (ret < 0) {
		/*
		 * -EACCES means the controller's runtime PM is not ready
		 * yet (e.g. its own probe is still in progress); retry
		 * later.
		 */
		dev_err_probe(dev, ret, "failed to resume slimbus controller\n");
		ret = (ret == -EACCES || ret == -EBUSY) ? -EPROBE_DEFER : ret;
		/* mclk_enabled is not set yet, so release the clock here. */
		clk_disable_unprepare(tabla->mclk);
		goto err_disable;
	}

	/* Both jflte and flo downstream reset for 20 ms at initial probe. */
	mutex_lock(&tabla->init_lock);
	tabla->mclk_enabled = true;
	ret = tabla->reset ? tabla_slim_reset_cycle(sdev, tabla, 20) : 0;
	mutex_unlock(&tabla->init_lock);
	if (ret) {
		pm_runtime_put_sync(sdev->dev.parent);
		goto err_disable;
	}
	ret = devm_add_action_or_reset(dev, tabla_slim_disable_hw, tabla);
	if (ret) {
		pm_runtime_put_sync(sdev->dev.parent);
		goto err_disable;
	}

	for (i = 0; i < 50; i++) {
		if (sdev->status == SLIM_DEVICE_STATUS_UP)
			break;
		usleep_range(10000, 11000);
	}

	/*
	 * If the codec reports present, try to bind it right away.  That
	 * can still fail if the status is stale: the slim core keeps a
	 * device UP with its logical address after a driver unbind, so a
	 * rebind after the codec was reset sees status UP while the codec
	 * is actually dead and transactions to it time out.  In that case
	 * fall through and cycle the reset line below.
	 */
	if (sdev->status == SLIM_DEVICE_STATUS_UP) {
		dev_info(dev, "codec enumerated, initialising\n");
		ret = tabla_slim_status(sdev, SLIM_DEVICE_STATUS_UP);
		if (!ret) {
			pm_runtime_put_sync(sdev->dev.parent);
			return 0;
		}
		if (!tabla_slim_codec_unreachable(ret)) {
			dev_err(dev, "codec init failed (%d)\n", ret);
			pm_runtime_put_sync(sdev->dev.parent);
			goto err_disable;
		}
		dev_warn(dev, "codec present but unresponsive (%d), cycling reset\n",
			 ret);
	} else {
		dev_info(dev, "codec not present, cycling reset\n");
	}

	/*
	 * Either the codec did not report present or it does not answer
	 * its registers: in both cases the cure is a real reset cycle
	 * on RESET_B.  A freshly reset chip takes a few seconds to
	 * re-enumerate its SLIM functions and accept register writes
	 * again, so the bring-up below is retried until the codec
	 * answers.  MCLK (and the controller) stays active until probe
	 * ends.
	 */
	mutex_lock(&tabla->init_lock);
	ret = tabla->registered ? 0 : tabla_slim_reset_cycle(sdev, tabla, 1000);
	mutex_unlock(&tabla->init_lock);
	if (ret && ret != -ENODEV) {
		pm_runtime_put_sync(sdev->dev.parent);
		goto err_disable;
	}

	{
		unsigned long deadline = jiffies + msecs_to_jiffies(15000);

		do {
			ret = sdev->status == SLIM_DEVICE_STATUS_UP ?
				tabla_slim_status(sdev, SLIM_DEVICE_STATUS_UP) :
				-ETIMEDOUT;
			if (!ret || !tabla_slim_codec_unreachable(ret))
				break;
			usleep_range(200000, 210000);
		} while (time_before(jiffies, deadline));
	}
	pm_runtime_put_sync(sdev->dev.parent);

	if (ret) {
		dev_err(dev, "codec unresponsive after reset cycle (%d)\n", ret);
		/*
		 * If the codec simply does not answer, retry later; a
		 * different failure is permanent.
		 */
		ret = tabla_slim_codec_unreachable(ret) ? -EPROBE_DEFER : ret;
		goto err_disable;
	}
	return 0;

err_disable:
	tabla_slim_disable_hw(tabla);
	return ret;
}

static const struct slim_device_id tabla_slim_ids[] = {
	{ TABLA_MANF_ID, TABLA_PROD_CODE, 1, 0 },
	{}
};
MODULE_DEVICE_TABLE(slim, tabla_slim_ids);

static struct slim_driver tabla_slim_driver = {
	.driver = {
		.name = "wcd9310-tabla",
	},
	.probe = tabla_slim_probe,
	.device_status = tabla_slim_status,
	.id_table = tabla_slim_ids,
};
module_slim_driver(tabla_slim_driver);

MODULE_DESCRIPTION("Qualcomm WCD9310 tabla SLIMbus codec driver");
MODULE_LICENSE("GPL");
