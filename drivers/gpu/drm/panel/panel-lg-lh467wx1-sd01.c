// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2021, Ivan Belokobylskiy <belokobylskij@gmail.com>

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>


char lcd_mirror [2] = {0x36, 0x02};

// values of DSV setting START
static char panel_setting_1 [6] = {0xB0, 0x43, 0x00, 0x00, 0x00, 0x00};
static char panel_setting_2 [3] = {0xB3, 0x0A, 0x9F};

static char display_mode1 [6] = {0xB5, 0x50, 0x20, 0x40, 0x00, 0x20};
static char display_mode2 [8] = {0xB6, 0x00, 0x14, 0x0F, 0x16, 0x13, 0x05, 0x05};

static char p_gamma_r_setting[10] = {0xD0, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char n_gamma_r_setting[10] = {0xD1, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char p_gamma_g_setting[10] = {0xD2, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char n_gamma_g_setting[10] = {0xD3, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char p_gamma_b_setting[10] = {0xD4, 0x20, 0x23, 0x74, 0x00, 0x1F, 0x10, 0x50, 0x33, 0x03};
static char n_gamma_b_setting[10] = {0xD5, 0x20, 0x23, 0x74, 0x00, 0x1F, 0x10, 0x50, 0x33, 0x03};

static char ief_on_set0[2] = {0xE0, 0x00};
static char ief_on_set4[4] = {0xE4, 0x00, 0x00, 0x00};
static char ief_on_set5[4] = {0xE5, 0x00, 0x00, 0x00};
static char ief_on_set6[4] = {0xE6, 0x00, 0x00, 0x00};

static char ief_set1[5] = {0xE1, 0x00, 0x00, 0x01, 0x01};
static char ief_set2[3] = {0xE2, 0x01, 0x00};
static char ief_set3[6] = {0xE3, 0x00, 0x00, 0x42, 0x35, 0x00};
static char ief_set7[9] = {0xE7, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40};
static char ief_set8[9] = {0xE8, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D};
static char ief_set9[9] = {0xE9, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B};
static char ief_setA[9] = {0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char ief_setB[9] = {0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char ief_setC[9] = {0xEC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static char osc_setting[4] =     {0xC0, 0x00, 0x0A, 0x10};
static char power_setting3[13] = {0xC3, 0x00, 0x88, 0x03, 0x20, 0x01, 0x57, 0x4F, 0x33,0x02,0x38,0x38,0x00};
static char power_setting4[6] =  {0xC4, 0x31, 0x24, 0x11, 0x11, 0x3D};
static char power_setting5[4] =  {0xC5, 0x3B, 0x3B, 0x03};


struct lh467wx1_sd01 {
    struct drm_panel panel;
    struct mipi_dsi_device *dsi;
    struct regulator_bulk_data supplies[2];
    struct gpio_desc *reset_gpio;

    bool prepared;
};

static inline struct
        lh467wx1_sd01 *to_lh467wx1_sd01(struct drm_panel *panel)
{
    return container_of(panel, struct lh467wx1_sd01, panel);
}

#define dsi_dcs_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static void lh467wx1_sd01_reset(struct lh467wx1_sd01 *ctx)
{
    gpiod_set_value_cansleep(ctx->reset_gpio, 1);
    usleep_range(5000, 6000);
    gpiod_set_value_cansleep(ctx->reset_gpio, 0);
    usleep_range(1000, 2000);
    gpiod_set_value_cansleep(ctx->reset_gpio, 1);
    usleep_range(10000, 11000);
}

static int lh467wx1_sd01_on(struct lh467wx1_sd01 *ctx)
{
    struct mipi_dsi_device *dsi = ctx->dsi;
    struct device *dev = &dsi->dev;
    int ret;

    dsi->mode_flags |= MIPI_DSI_MODE_LPM;

    // Display Initial Set
    dsi_dcs_write_seq(dsi, 0x36, 0x02); // lcd_mirror
    dsi_dcs_write_seq(dsi, 0xB0, 0x43, 0x00, 0x00, 0x00, 0x00); // panel_setting_1
    dsi_dcs_write_seq(dsi, 0xB3, 0x0A, 0x9F); // panel_setting_2
    dsi_dcs_write_seq(dsi, 0xB5, 0x50, 0x20, 0x40, 0x00, 0x20); // display_mode1
    dsi_dcs_write_seq(dsi, 0xB6, 0x00, 0x14, 0x0F, 0x16, 0x13, 0x05, 0x05); // display_mode2

    ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
    if (ret < 0) {
        dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
        return ret;
    }
    msleep(120);

    // Gamma Set
    dsi_dcs_write_seq(dsi, 0xD0, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01); // p_gamma_r_setting
    dsi_dcs_write_seq(dsi, 0xD1, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01); // p_gamma_r_setting
    dsi_dcs_write_seq(dsi, 0xD2, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01); // p_gamma_g_setting
    dsi_dcs_write_seq(dsi, 0xD3, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01); // p_gamma_g_setting
    dsi_dcs_write_seq(dsi, 0xD4, 0x20, 0x23, 0x74, 0x00, 0x1F, 0x10, 0x50, 0x33, 0x03); // p_gamma_b_setting
    dsi_dcs_write_seq(dsi, 0xD5, 0x20, 0x23, 0x74, 0x00, 0x1F, 0x10, 0x50, 0x33, 0x03); // p_gamma_b_setting
    dsi_dcs_write_seq(dsi, 0xD5, 0x20, 0x23, 0x74, 0x00, 0x1F, 0x10, 0x50, 0x33, 0x03); // p_gamma_b_setting

    // IEF set
    dsi_dcs_write_seq(dsi, 0xE0, 0x00); // ief_on_set0
    dsi_dcs_write_seq(dsi, 0xE4, 0x00, 0x00, 0x00); // ief_on_set4
    dsi_dcs_write_seq(dsi, 0xE5, 0x00, 0x00, 0x00); // ief_on_set5
    dsi_dcs_write_seq(dsi, 0xE6, 0x00, 0x00, 0x00); // ief_on_set6
    dsi_dcs_write_seq(dsi, 0xE1, 0x00, 0x00, 0x01, 0x01); // ief_set1
    dsi_dcs_write_seq(dsi, 0xE2, 0x01, 0x00); // ief_set2
    dsi_dcs_write_seq(dsi, 0xE3, 0x00, 0x00, 0x42, 0x35, 0x00); // ief_set3
    dsi_dcs_write_seq(dsi, 0xE7, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40); // ief_set7
    dsi_dcs_write_seq(dsi, 0xE8, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D); // ief_set8
    dsi_dcs_write_seq(dsi, 0xE9, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B); // ief_set9
    dsi_dcs_write_seq(dsi, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); // ief_setA
    dsi_dcs_write_seq(dsi, 0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); // ief_setB
    dsi_dcs_write_seq(dsi, 0xEC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); // ief_setC

    // Power Supply Set
    dsi_dcs_write_seq(dsi, 0xC0, 0x00, 0x0A, 0x10); // osc_setting
    dsi_dcs_write_seq(dsi, 0xC3, 0x00, 0x88, 0x03, 0x20, 0x01, 0x57, 0x4F, 0x33,0x02,0x38,0x38,0x00); // power_setting3
    dsi_dcs_write_seq(dsi, 0xC4, 0x31, 0x24, 0x11, 0x11, 0x3D); // power_setting4
    dsi_dcs_write_seq(dsi, 0xC5, 0x3B, 0x3B, 0x03); // power_setting5



    // power_on_set_2
    dsi_dcs_write_seq(dsi, 0xC2,0x063); // exit_sleep_power_control_2
    msleep(10);
    dsi_dcs_write_seq(dsi, 0xC2,0x0E); // exit_sleep_power_control_3
    msleep(1);

    // TODO: enable ext_dsv_load

    dsi_dcs_write_seq(dsi, 0xF1,0x10,0x00); // otp_protection
    dsi_dcs_write_seq(dsi, 0x11,0x00); // sleep_out_for_cabc
    dsi_dcs_write_seq(dsi, 0xC1,0x08); // gate_output_enabled_by_manual

    ret = mipi_dsi_dcs_set_display_on(dsi);
    if (ret < 0) {
        dev_err(dev, "Failed to set display on: %d\n", ret);
        return ret;
    }

    return 0;
}

static int lh467wx1_sd01_off(struct lh467wx1_sd01 *ctx)
{
    struct mipi_dsi_device *dsi = ctx->dsi;
    struct device *dev = &dsi->dev;
    int ret;

    dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

    ret = mipi_dsi_dcs_set_display_off(dsi);
    if (ret < 0) {
        dev_err(dev, "Failed to set display off: %d\n", ret);
        return ret;
    }
    msleep(20);

    ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
    if (ret < 0) {
        dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
        return ret;
    }
    msleep(5);

    // TODO: disable ext_dsv

    dsi_dcs_write_seq(dsi, 0xC2,0x00); // analog_boosting_power_control
    msleep(10);
    dsi_dcs_write_seq(dsi, 0xC2,0x01); // enter_sleep_power_control_3
    msleep(10);
    dsi_dcs_write_seq(dsi, 0xC2,0x00); // enter_sleep_power_control_2
    dsi_dcs_write_seq(dsi, 0xC1,0x02); // deep_standby
    msleep(10);

    return 0;
}

static int lh467wx1_sd01_prepare(struct drm_panel *panel)
{
    struct lh467wx1_sd01 *ctx = to_lh467wx1_sd01(panel);
    struct device *dev = &ctx->dsi->dev;
    int ret;

    if (ctx->prepared)
        return 0;

    ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
    if (ret < 0) {
        dev_err(dev, "Failed to enable regulators: %d\n", ret);
        return ret;
    }

    lh467wx1_sd01_reset(ctx);

    ret = lh467wx1_sd01_on(ctx);
    if (ret < 0) {
        dev_err(dev, "Failed to initialize panel: %d\n", ret);
        gpiod_set_value_cansleep(ctx->reset_gpio, 0);
        regulator_bulk_disable(ARRAY_SIZE(ctx->supplies),
                               ctx->supplies);
        return ret;
    }

    ctx->prepared = true;
    return 0;
}

static int lh467wx1_sd01_unprepare(struct drm_panel *panel)
{
    struct lh467wx1_sd01 *ctx = to_lh467wx1_sd01(panel);
    struct device *dev = &ctx->dsi->dev;
    int ret;

    if (!ctx->prepared)
        return 0;

    ret = lh467wx1_sd01_off(ctx);
    if (ret < 0)
        dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

    gpiod_set_value_cansleep(ctx->reset_gpio, 0);
    regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

    ctx->prepared = false;
    return 0;
}

static const struct drm_display_mode lh467wx1_sd01_mode = {
        .clock = (768 + 8 + 4 + 180) * (1280 + 8 + 2 + 22) * 60 / 1000,
        .hdisplay = 768,
        .hsync_start = 768 + 8,
        .hsync_end = 768 + 8 + 4,
        .htotal = 768 + 8 + 4 + 180,
        .vdisplay = 1280,
        .vsync_start = 1280 + 8,
        .vsync_end = 1280 + 8 + 2,
        .vtotal = 1280 + 8 + 2 + 22,
        .width_mm = 61,
        .height_mm = 102,
};

static int lh467wx1_sd01_get_modes(struct drm_panel *panel,
                                        struct drm_connector *connector)
{
    struct drm_display_mode *mode;

    mode = drm_mode_duplicate(connector->dev, &lh467wx1_sd01_mode);
    if (!mode)
        return -ENOMEM;

    drm_mode_set_name(mode);

    mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    connector->display_info.width_mm = mode->width_mm;
    connector->display_info.height_mm = mode->height_mm;
    drm_mode_probed_add(connector, mode);

    return 1;
}

static const struct drm_panel_funcs lh467wx1_sd01_panel_funcs = {
        .unprepare = lh467wx1_sd01_unprepare,
        .prepare = lh467wx1_sd01_prepare,
        .get_modes = lh467wx1_sd01_get_modes,
};

static int lh467wx1_sd01_probe(struct mipi_dsi_device *dsi)
{
    struct device *dev = &dsi->dev;
    struct lh467wx1_sd01 *ctx;
    int ret;

    ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    ctx->supplies[0].supply = "vci";
    ctx->supplies[1].supply = "iovcc";
    ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
                                  ctx->supplies);
    if (ret < 0) {
        dev_err(dev, "Failed to get regulators: %d\n", ret);
        return ret;
    }

    ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(ctx->reset_gpio)) {
        ret = PTR_ERR(ctx->reset_gpio);
        dev_err(dev, "Failed to get reset-gpios: %d\n", ret);
        return ret;
    }

    ctx->dsi = dsi;
    mipi_dsi_set_drvdata(dsi, ctx);

    dsi->lanes = 4;
    dsi->format = MIPI_DSI_FMT_RGB888;
    dsi->mode_flags = MIPI_DSI_MODE_VIDEO; // TODO ???

    drm_panel_init(&ctx->panel, dev, &lh467wx1_sd01_panel_funcs,
                   DRM_MODE_CONNECTOR_DSI);

    drm_panel_add(&ctx->panel);

    ret = mipi_dsi_attach(dsi);
    if (ret < 0) {
        dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
        return ret;
    }

    return 0;
}

static int lh467wx1_sd01_remove(struct mipi_dsi_device *dsi)
{
    struct lh467wx1_sd01 *ctx = mipi_dsi_get_drvdata(dsi);
    int ret;

    ret = mipi_dsi_detach(dsi);
    if (ret < 0)
        dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

    drm_panel_remove(&ctx->panel);

    return 0;
}

static const struct of_device_id lh467wx1_sd01_of_match[] = {
        { .compatible = "lg,lh467wx1-sd01" },
        { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, lh467wx1_sd01_of_match);

static struct mipi_dsi_driver lh467wx1_sd01_driver = {
        .probe = lh467wx1_sd01_probe,
        .remove = lh467wx1_sd01_remove,
        .driver = {
                .name = "panel-lh467wx1-sd01",
                .of_match_table = lh467wx1_sd01_of_match,
        },
};
module_mipi_dsi_driver(lh467wx1_sd01_driver);

MODULE_AUTHOR("Ivan Belokobylskiy <belokobylskij@gmail.com>");
MODULE_DESCRIPTION("MIPI-DSI based Panel Driver for lh467wx1-sd01 LCD");
MODULE_LICENSE("GPL v2");
