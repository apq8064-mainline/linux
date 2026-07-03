// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung OCTA FHD panel driver - Galaxy S4.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include "panel-samsung-magna-octa-smart-dim.h"

#define MAGNA_OCTA_ID3_EARLY_REVB	0x85
#define MAGNA_OCTA_DEFAULT_ELVSS	0x2b
#define MAGNA_OCTA_ELVSS_LSL		2

enum magna_octa_revision {
	MAGNA_OCTA_REV_UNKNOWN,
	MAGNA_OCTA_REV_EARLY_REVB,
	MAGNA_OCTA_REV_DEFAULT_REVF,
};

struct magna_octa {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	u32 panel_id;
	struct backlight_device *backlight;
	u16 brightness;
	unsigned int candela;
	u8 mtp_orig[33];
	u8 b1_reg[6];
	u8 b6_reg_magna[23];
	struct SMART_DIM smart_dim;
	bool prepared;
	bool smart_dim_ready;
	bool sleeping;
	bool panel_id_valid;
	enum magna_octa_revision revision;
};

static inline struct magna_octa *to_magna_octa(struct drm_panel *panel)
{
	return container_of(panel, struct magna_octa, panel);
}

static enum magna_octa_revision magna_octa_classify_revision(u32 panel_id)
{
	u8 id3 = panel_id & 0xff;

	if (id3 == MAGNA_OCTA_ID3_EARLY_REVB)
		return MAGNA_OCTA_REV_EARLY_REVB;

	return MAGNA_OCTA_REV_DEFAULT_REVF;
}

static const char *magna_octa_revision_name(enum magna_octa_revision revision)
{
	switch (revision) {
	case MAGNA_OCTA_REV_EARLY_REVB:
		return "early-revb";
	case MAGNA_OCTA_REV_DEFAULT_REVF:
		return "default-revf";
	case MAGNA_OCTA_REV_UNKNOWN:
	default:
		return "unknown";
	}
}

static const int magna_octa_lux_table[] = {
	10, 11, 12, 13, 14,
	15, 16, 17, 19, 20,
	21, 22, 24, 25, 27,
	29, 30, 32, 34, 37,
	39, 41, 44, 47, 50,
	53, 56, 60, 64, 68,
	72, 77, 82, 87, 93,
	98, 105, 111, 119, 126,
	134, 143, 152, 162, 172,
	183, 195, 207, 220, 234,
	249, 265, 282, 300,
};

struct magna_octa_elvss_entry {
	unsigned int max_candela;
	u8 level;
};

static const struct magna_octa_elvss_entry magna_octa_elvss_table[] = {
	{ 105, 0x0f },
	{ 111, 0x0b },
	{ 119, 0x0a },
	{ 126, 0x09 },
	{ 134, 0x08 },
	{ 143, 0x07 },
	{ 152, 0x06 },
	{ 172, 0x05 },
	{ 183, 0x09 },
	{ 195, 0x08 },
	{ 207, 0x07 },
	{ 220, 0x06 },
	{ 234, 0x05 },
	{ 249, 0x04 },
	{ 265, 0x03 },
	{ 282, 0x01 },
};

static const u8 magna_octa_brightness_thresholds[] = {
	10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 21, 23, 24, 26,
	28, 29, 31, 32, 36, 38, 40, 43, 46, 49, 52, 55, 59, 63,
	67, 71, 76, 81, 86, 92, 97, 104, 110, 118, 125, 133, 142,
	149, 161, 171, 182, 194, 206, 219, 232, 248, 249, 251, 253,
	255,
};

static unsigned int magna_octa_get_candela_index(unsigned int brightness)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(magna_octa_brightness_thresholds); i++) {
		if (brightness <= magna_octa_brightness_thresholds[i])
			return i;
	}

	return ARRAY_SIZE(magna_octa_lux_table) - 1;
}

static unsigned int magna_octa_get_candela(unsigned int brightness)
{
	return magna_octa_lux_table[magna_octa_get_candela_index(brightness)];
}

static u8 magna_octa_get_elvss_value(unsigned int candela)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(magna_octa_elvss_table); i++) {
		if (candela <= magna_octa_elvss_table[i].max_candela)
			return MAGNA_OCTA_DEFAULT_ELVSS +
			       (magna_octa_elvss_table[i].level << MAGNA_OCTA_ELVSS_LSL);
	}

	return MAGNA_OCTA_DEFAULT_ELVSS;
}

static const u8 magna_test_key_on1[] = {
	0xF0, 0x5A, 0x5A,
};

static const u8 magna_test_key_on2[] = {
	0xF1, 0x5A, 0x5A,
};

static const u8 magna_test_key_on3[] = {
	0xFC, 0x5A, 0x5A,
};

static const u8 magna_test_key_off1[] = {
	0xF0, 0xA5, 0xA5,
};

static const u8 magna_test_key_off2[] = {
	0xF1, 0xA5, 0xA5,
};

static const u8 magna_ready_on_revf_first[] = {
	0xD2, 0xDD, 0x21, 0x24, 0xFF, 0x11,
	0x0B, 0x73, 0x00, 0x00, 0x26, 0x20,
};

static const u8 magna_manual_bootsing[] = {
	0xF3, 0x00, 0x01,
};

static const u8 magna_soft_reset[] = { 0x01 };

struct magna_octa_aor_entry {
	unsigned int min_candela;
	u8 value[5];
};

static const struct magna_octa_aor_entry magna_octa_aor_table[] = {
	{ 183, { 0xB2, 0x00, 0x00, 0x00, 0x08 } },
	{ 111, { 0xB2, 0x00, 0x00, 0x03, 0x06 } },
	{ 105, { 0xB2, 0x00, 0x00, 0x00, 0x20 } },
	{ 98, { 0xB2, 0x00, 0x00, 0x00, 0xA5 } },
	{ 93, { 0xB2, 0x00, 0x00, 0x01, 0x03 } },
	{ 87, { 0xB2, 0x00, 0x00, 0x01, 0x72 } },
	{ 82, { 0xB2, 0x00, 0x00, 0x01, 0xCC } },
	{ 77, { 0xB2, 0x00, 0x00, 0x02, 0x27 } },
	{ 72, { 0xB2, 0x00, 0x00, 0x02, 0x83 } },
	{ 68, { 0xB2, 0x00, 0x00, 0x02, 0xCE } },
	{ 64, { 0xB2, 0x00, 0x00, 0x03, 0x1B } },
	{ 60, { 0xB2, 0x00, 0x00, 0x03, 0x69 } },
	{ 56, { 0xB2, 0x00, 0x00, 0x03, 0xAE } },
	{ 53, { 0xB2, 0x00, 0x00, 0x03, 0xE2 } },
	{ 50, { 0xB2, 0x00, 0x00, 0x04, 0x16 } },
	{ 47, { 0xB2, 0x00, 0x00, 0x04, 0x4E } },
	{ 44, { 0xB2, 0x00, 0x00, 0x04, 0x86 } },
	{ 41, { 0xB2, 0x00, 0x00, 0x04, 0xBF } },
	{ 39, { 0xB2, 0x00, 0x00, 0x04, 0xE3 } },
	{ 37, { 0xB2, 0x00, 0x00, 0x05, 0x06 } },
	{ 34, { 0xB2, 0x00, 0x00, 0x05, 0x3B } },
	{ 32, { 0xB2, 0x00, 0x00, 0x05, 0x5E } },
	{ 30, { 0xB2, 0x00, 0x00, 0x05, 0x82 } },
	{ 29, { 0xB2, 0x00, 0x00, 0x05, 0x92 } },
	{ 27, { 0xB2, 0x00, 0x00, 0x05, 0xB3 } },
	{ 25, { 0xB2, 0x00, 0x00, 0x05, 0xD4 } },
	{ 24, { 0xB2, 0x00, 0x00, 0x05, 0xE5 } },
	{ 22, { 0xB2, 0x00, 0x00, 0x06, 0x06 } },
	{ 21, { 0xB2, 0x00, 0x00, 0x06, 0x16 } },
	{ 20, { 0xB2, 0x00, 0x00, 0x06, 0x27 } },
	{ 19, { 0xB2, 0x00, 0x00, 0x06, 0x38 } },
	{ 17, { 0xB2, 0x00, 0x00, 0x06, 0x5B } },
	{ 16, { 0xB2, 0x00, 0x00, 0x06, 0x6C } },
	{ 15, { 0xB2, 0x00, 0x00, 0x06, 0x7D } },
	{ 14, { 0xB2, 0x00, 0x00, 0x06, 0x8F } },
	{ 13, { 0xB2, 0x00, 0x00, 0x06, 0xA0 } },
	{ 12, { 0xB2, 0x00, 0x00, 0x06, 0xB2 } },
	{ 11, { 0xB2, 0x00, 0x00, 0x06, 0xC3 } },
	{ 0, { 0xB2, 0x00, 0x00, 0x06, 0xD5 } },
};

static const u8 *magna_octa_get_aor_value(unsigned int candela)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(magna_octa_aor_table); i++) {
		if (candela >= magna_octa_aor_table[i].min_candela)
			return magna_octa_aor_table[i].value;
	}

	return magna_octa_aor_table[ARRAY_SIZE(magna_octa_aor_table) - 1].value;
}

static const u8 magna_gamma_update[] = {
	0xF7, 0x01,
};

static const u8 magna_brightness_gamma[] = {
	0xCA,
	0x01, 0x00, 0x01, 0x00,
	0x01, 0x00, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x00, 0x00,
	0x00,
};

static int magna_octa_read_panel_id(struct magna_octa *ctx, u8 *id1, u8 *id2,
				    u8 *id3)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	ret = mipi_dsi_dcs_read(dsi, 0xda, id1, 1);
	if (ret < 0)
		return dev_err_probe(&dsi->dev, ret, "failed to read panel ID1\n");

	ret = mipi_dsi_dcs_read(dsi, 0xdb, id2, 1);
	if (ret < 0)
		return dev_err_probe(&dsi->dev, ret, "failed to read panel ID2\n");

	ret = mipi_dsi_dcs_read(dsi, 0xdc, id3, 1);
	if (ret < 0)
		return dev_err_probe(&dsi->dev, ret, "failed to read panel ID3\n");

	ctx->panel_id = (*id1 << 16) | (*id2 << 8) | *id3;
	ctx->revision = magna_octa_classify_revision(ctx->panel_id);

	dev_info(&dsi->dev, "panel ID %02x %02x %02x rev=%s\n",
		 *id1, *id2, *id3, magna_octa_revision_name(ctx->revision));

	return 0;
}

static int magna_octa_set_maximum_return_packet_size(struct magna_octa *ctx,
						     u16 size)
{
	int ret;

	ret = mipi_dsi_set_maximum_return_packet_size(ctx->dsi, size);
	if (ret < 0)
		dev_err(&ctx->dsi->dev,
			"failed to set max return packet size %u: %d\n",
			size, ret);

	return ret;
}

static int magna_octa_write_buffer(struct magna_octa *ctx, const void *data,
				   size_t len, const char *what)
{
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, data, len);
	if (ret < 0)
		return dev_err_probe(&ctx->dsi->dev, ret, "failed %s\n", what);

	return 0;
}

static int magna_octa_read_reg_chunked(struct magna_octa *ctx, u8 reg,
				       u8 read_from, void *buf, size_t len)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	u8 *data = buf;
	u8 read_pos_buf[] = { 0xB0, 0x00 };
	size_t offset = 0;
	int ret;

	ret = magna_octa_set_maximum_return_packet_size(ctx, 6);
	if (ret < 0)
		return ret;

	while (offset < len) {
		size_t chunk_len = min_t(size_t, len - offset, 6);

		read_pos_buf[1] = read_from + offset;
		ret = mipi_dsi_dcs_write_buffer(dsi, read_pos_buf,
						sizeof(read_pos_buf));
		if (ret < 0)
			return dev_err_probe(&dsi->dev, ret,
					     "failed to set read position\n");

		ret = mipi_dsi_dcs_read(dsi, reg, &data[offset], chunk_len);
		if (ret < 0)
			return dev_err_probe(&dsi->dev, ret,
					     "failed register read\n");
		if (ret != chunk_len)
			return dev_err_probe(&dsi->dev, -EIO,
					     "short register read\n");

		offset += chunk_len;
	}

	return 0;
}

static int magna_octa_read_magna_mtp(struct magna_octa *ctx)
{
	int ret;

	ret = magna_octa_write_buffer(ctx, magna_test_key_on1,
				      sizeof(magna_test_key_on1),
				      "mtp key on1");
	if (ret < 0)
		return ret;

	ret = magna_octa_write_buffer(ctx, magna_test_key_on3,
				      sizeof(magna_test_key_on3),
				      "mtp key on3");
	if (ret < 0)
		return ret;

	ret = magna_octa_read_reg_chunked(ctx, 0xC8, 0, ctx->mtp_orig,
					  sizeof(ctx->mtp_orig));
	if (ret < 0)
		return ret;

	ret = magna_octa_read_reg_chunked(ctx, 0xB1, 9, ctx->b1_reg,
					  sizeof(ctx->b1_reg));
	if (ret < 0)
		return ret;

	ret = magna_octa_read_reg_chunked(ctx, 0xB6, 3, ctx->b6_reg_magna,
					  sizeof(ctx->b6_reg_magna));
	if (ret < 0)
		return ret;

	return 0;
}

static int magna_octa_smart_dim_load_magna(struct magna_octa *ctx)
{
	struct SMART_DIM *smart_dim = &ctx->smart_dim;
	int ret;

	memset(smart_dim, 0, sizeof(*smart_dim));

	memcpy(&smart_dim->MTP_ORIGN, ctx->mtp_orig, sizeof(ctx->mtp_orig));
	memcpy(smart_dim->hbm_reg.b1_reg, ctx->b1_reg,
	       sizeof(smart_dim->hbm_reg.b1_reg));
	memcpy(smart_dim->hbm_reg.b6_reg_magna, ctx->b6_reg_magna,
	       sizeof(smart_dim->hbm_reg.b6_reg_magna));

	smart_dim->lux_table_max = ARRAY_SIZE(magna_octa_lux_table);
	smart_dim->plux_table = (int *)magna_octa_lux_table;
	smart_dim->brightness_level = ctx->candela;
	smart_dim->ldi_revision = ctx->panel_id;

	ret = smart_dimming_init(smart_dim);
	if (ret < 0) {
		ctx->smart_dim_ready = false;
		return ret;
	}

	ctx->smart_dim_ready = true;

	return 0;
}

static int magna_octa_prepare_revf_smart_dim(struct magna_octa *ctx)
{
	int ret;

	if (ctx->revision != MAGNA_OCTA_REV_DEFAULT_REVF)
		return 0;

	if (ctx->smart_dim_ready)
		return 0;

	ret = magna_octa_read_magna_mtp(ctx);
	if (ret < 0)
		return ret;

	return magna_octa_smart_dim_load_magna(ctx);
}

static int magna_octa_write_revf_brightness(struct magna_octa *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	const u8 *aor;
	u8 elvss[] = { 0xB6, 0x00 };
	u8 gamma[GAMMA_SET_MAX + 1] = { 0xCA };
	u8 ctrl = ctx->brightness >= 128 ? 0xA0 : 0x00;
	int ret;

	aor = magna_octa_get_aor_value(ctx->candela);
	elvss[1] = magna_octa_get_elvss_value(ctx->candela);

	ret = magna_octa_write_buffer(ctx, magna_test_key_on1,
				      sizeof(magna_test_key_on1),
				      "brightness key on1");
	if (ret < 0)
		return ret;

	ret = magna_octa_write_buffer(ctx, magna_test_key_on2,
				      sizeof(magna_test_key_on2),
				      "brightness key on2");
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, &ctrl, 1);
	if (ret < 0)
		return ret;

	ret = magna_octa_write_buffer(ctx, aor, 5, "AOR value");
	if (ret < 0)
		return ret;

	ret = magna_octa_write_buffer(ctx, elvss, sizeof(elvss), "ELVSS value");
	if (ret < 0)
		return ret;

	if (ctx->smart_dim_ready) {
		ctx->smart_dim.brightness_level = ctx->candela;
		generate_gamma(&ctx->smart_dim, &gamma[1], GAMMA_SET_MAX);
		ret = magna_octa_write_buffer(ctx, gamma, sizeof(gamma),
					      "generated gamma");
	} else {
		ret = magna_octa_write_buffer(ctx, magna_brightness_gamma,
					      sizeof(magna_brightness_gamma),
					      "default gamma");
	}
	if (ret < 0)
		return ret;

	ret = magna_octa_write_buffer(ctx, magna_gamma_update,
				      sizeof(magna_gamma_update),
				      "gamma update");
	if (ret < 0)
		return ret;

	ret = magna_octa_write_buffer(ctx, magna_test_key_off1,
				      sizeof(magna_test_key_off1),
				      "brightness key off1");
	if (ret < 0)
		return ret;

	ret = magna_octa_write_buffer(ctx, magna_test_key_off2,
				      sizeof(magna_test_key_off2),
				      "brightness key off2");
	if (ret < 0)
		return ret;

	return 0;
}

static int magna_octa_push_brightness(struct magna_octa *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	if (!ctx->prepared)
		return 0;

	if (ctx->revision == MAGNA_OCTA_REV_DEFAULT_REVF)
		return magna_octa_write_revf_brightness(ctx);

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				 (u8[]){ 0x20 }, 1);
	if (ret < 0)
		return dev_err_probe(&dsi->dev, ret,
				     "failed to enable brightness control\n");

	ret = mipi_dsi_dcs_set_display_brightness(dsi, ctx->brightness);
	if (ret < 0)
		return dev_err_probe(&dsi->dev, ret,
				     "failed to set display brightness\n");

	return 0;
}

static int magna_octa_bl_update_status(struct backlight_device *bl)
{
	struct magna_octa *ctx = bl_get_data(bl);

	ctx->brightness = backlight_get_brightness(bl);
	ctx->candela = magna_octa_get_candela(ctx->brightness);

	return magna_octa_push_brightness(ctx);
}

static int magna_octa_bl_get_brightness(struct backlight_device *bl)
{
	struct magna_octa *ctx = bl_get_data(bl);

	return ctx->brightness;
}

static const struct backlight_ops magna_octa_bl_ops = {
	.update_status = magna_octa_bl_update_status,
	.get_brightness = magna_octa_bl_get_brightness,
};

static const struct drm_display_mode magna_octa_mode = {
	.clock = 149000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 145,
	.hsync_end = 1080 + 145 + 10,
	.htotal = 1080 + 145 + 10 + 50,
	.vdisplay = 1920,
	.vsync_start = 1920 + 10,
	.vsync_end = 1920 + 10 + 2,
	.vtotal = 1920 + 10 + 2 + 4,
	.width_mm = 62,
	.height_mm = 111,
};

static void magna_octa_reset(struct magna_octa *ctx)
{
	if (!ctx->reset_gpio)
		return;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(7000, 8000);
}

static int magna_octa_prepare(struct drm_panel *panel)
{
	struct magna_octa *ctx = to_magna_octa(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	u8 id1, id2, id3;
	int ret;

	if (ctx->prepared)
		return 0;

	magna_octa_reset(ctx);
	msleep(20);

	if (!ctx->panel_id_valid) {
		ret = magna_octa_read_panel_id(ctx, &id1, &id2, &id3);
		if (ret < 0)
			return ret;

		ctx->panel_id_valid = true;
	}

	ret = magna_octa_prepare_revf_smart_dim(ctx);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return dev_err_probe(&dsi->dev, ret, "failed to exit sleep mode\n");

	ctx->sleeping = false;
	ctx->prepared = true;
	msleep(120);

	return 0;
}

static int magna_octa_enable(struct drm_panel *panel)
{
	struct magna_octa *ctx = to_magna_octa(panel);
	int ret;

	if (ctx->sleeping) {
		ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
		if (ret < 0)
			return dev_err_probe(&ctx->dsi->dev, ret,
					     "failed to exit sleep mode\n");

		ctx->sleeping = false;
		msleep(120);
	}

	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	if (ret < 0)
		return dev_err_probe(&ctx->dsi->dev, ret, "failed to set display on\n");


	return 0;
}

static int magna_octa_disable(struct drm_panel *panel)
{
	struct magna_octa *ctx = to_magna_octa(panel);
	int ret;

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		dev_err(&ctx->dsi->dev, "failed to set display off: %d\n", ret);

	msleep(40);

	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	if (ret < 0)
		dev_err(&ctx->dsi->dev, "failed to enter sleep mode: %d\n", ret);
	else
		ctx->sleeping = true;

	msleep(120);

	return 0;
}

static int magna_octa_unprepare(struct drm_panel *panel)
{
	struct magna_octa *ctx = to_magna_octa(panel);

	if (!ctx->prepared)
		return 0;

	ctx->sleeping = false;
	ctx->prepared = false;
	if (ctx->reset_gpio)
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static int magna_octa_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &magna_octa_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs magna_octa_panel_funcs = {
	.prepare = magna_octa_prepare,
	.enable = magna_octa_enable,
	.disable = magna_octa_disable,
	.unprepare = magna_octa_unprepare,
	.get_modes = magna_octa_get_modes,
};

static int magna_octa_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct magna_octa *ctx;
	struct backlight_properties bl_props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct magna_octa, panel,
				   &magna_octa_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->brightness = 255;
	ctx->candela = magna_octa_get_candela(ctx->brightness);
	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get optional reset GPIO\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_NO_HFP;
	dsi->hs_rate = 898000000;

	ctx->panel.prepare_prev_first = true;

	ctx->backlight = devm_backlight_device_register(dev, dev_name(dev), dev,
							ctx, &magna_octa_bl_ops,
							&bl_props);
	if (IS_ERR(ctx->backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->backlight),
				     "failed to register backlight\n");

	ctx->backlight->props.brightness = ctx->brightness;
	ctx->panel.backlight = ctx->backlight;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void magna_octa_shutdown_panel(struct magna_octa *ctx)
{
	if (ctx->panel.enabled)
		drm_panel_disable(&ctx->panel);
	if (ctx->panel.prepared)
		drm_panel_unprepare(&ctx->panel);
}

static void magna_octa_remove(struct mipi_dsi_device *dsi)
{
	struct magna_octa *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	magna_octa_shutdown_panel(ctx);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static void magna_octa_shutdown(struct mipi_dsi_device *dsi)
{
	struct magna_octa *ctx = mipi_dsi_get_drvdata(dsi);

	magna_octa_shutdown_panel(ctx);
}

static const struct of_device_id magna_octa_of_match[] = {
	{ .compatible = "samsung,magna-octa-fhd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, magna_octa_of_match);

static struct mipi_dsi_driver magna_octa_driver = {
	.probe = magna_octa_probe,
	.remove = magna_octa_remove,
	.shutdown = magna_octa_shutdown,
	.driver = {
		.name = "panel-samsung-magna-octa",
		.of_match_table = magna_octa_of_match,
	},
};
module_mipi_dsi_driver(magna_octa_driver);

MODULE_AUTHOR("Alexandre MINETTE <contact@alex-min.fr>");
MODULE_DESCRIPTION("DRM driver for Samsung jflte OCTA FHD panel");
MODULE_LICENSE("GPL");
