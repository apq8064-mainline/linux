// SPDX-License-Identifier: GPL-2.0-only
/*
 * Author: Wouter Franken <wouter.franken@gmail.com>
 *
 * From internet archives, the panel for Sony Xperia Z, 2013 model is a
 * jdc model MDY7.
 */

//#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

static const char * const regulator_names[] = {
	"vddp",
	"iovcc"
};

struct jdc_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct regulator_bulk_data supplies[ARRAY_SIZE(regulator_names)];

	//struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *dcdc_en_gpio;
	//struct backlight_device *backlight;

	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;
};

static inline struct jdc_panel *to_jdi_panel(struct drm_panel *panel)
{
	return container_of(panel, struct jdc_panel, base);
}

static int jdc_panel_init(struct jdc_panel *jdc)
{
	struct mipi_dsi_device *dsi = jdc->dsi;
	struct device *dev = &jdc->dsi->dev;
	int ret;

	// dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0)
		return ret;

        msleep(5000);

        /*
         * TODO: Should these stay here?

	ret = mipi_dsi_dcs_set_pixel_format(dsi, MIPI_DCS_PIXEL_FMT_24BIT << 4);
	if (ret < 0) {
		dev_err(dev, "failed to set pixel format: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_column_address(dsi, 0, jdc->mode->hdisplay - 1);
	if (ret < 0) {
		dev_err(dev, "failed to set column address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_page_address(dsi, 0, jdc->mode->vdisplay - 1);
	if (ret < 0) {
		dev_err(dev, "failed to set page address: %d\n", ret);
		return ret;
	}
        */

        /* mcap */
	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x00}, 2);
	if (ret < 0) {
		dev_err(dev, "failed to set default values for mcap: %d\n"
			, ret);
		return ret;
	}

        /* sequencer test control */
	ret = mipi_dsi_generic_write(dsi, (u8[]){0xD6, 0x01}, 2);
	if (ret < 0) {
		dev_err(dev, "failed to set sequencer test control: %d\n"
			, ret);
		return ret;
	}

	/* Interface setting */
	ret = mipi_dsi_generic_write(dsi, (u8[])
				     {0xB3, 0x14, 0x00, 0x00, 0x20, 0x00, 0x00}, 7);
	if (ret < 0) {
		dev_err(dev, "failed to set display interface setting: %d\n"
			, ret);
		return ret;
	}

	/* Interface id setting */
	ret = mipi_dsi_generic_write(dsi, (u8[])
				     {0xB4, 0x0C, 0x00}, 3);
	if (ret < 0) {
		dev_err(dev, "failed to set display interface id setting: %d\n"
			, ret);
		return ret;
	}

	/* DSI control */
	ret = mipi_dsi_generic_write(dsi, (u8[])
				     {0xB6, 0x3A, 0xD3}, 3);
	if (ret < 0) {
		dev_err(dev, "failed to set dsi control: %d\n"
			, ret);
		return ret;
	}

	return 0;
}

static int jdc_panel_on(struct jdc_panel *jdc)
{
	struct mipi_dsi_device *dsi = jdc->dsi;
	struct device *dev = &jdc->dsi->dev;
	int ret;

	// dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to set exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		dev_err(dev, "failed to set display on: %d\n", ret);

	return ret;
}

static void jdc_panel_off(struct jdc_panel *jdc)
{
	struct mipi_dsi_device *dsi = jdc->dsi;
	struct device *dev = &jdc->dsi->dev;
	int ret;

	// dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		dev_err(dev, "failed to set display off: %d\n", ret);

	msleep(20);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		dev_err(dev, "failed to enter sleep mode: %d\n", ret);

	msleep(80);
}

static int jdc_panel_disable(struct drm_panel *panel)
{
	struct jdc_panel *jdc = to_jdi_panel(panel);

	if (!jdc->enabled)
		return 0;

	//backlight_disable(jdc->backlight);

	jdc->enabled = false;

	return 0;
}

static int jdc_panel_unprepare(struct drm_panel *panel)
{
	struct jdc_panel *jdc = to_jdi_panel(panel);
	struct device *dev = &jdc->dsi->dev;
	int ret;

	if (!jdc->prepared)
		return 0;

	jdc_panel_off(jdc);

	gpiod_set_value(jdc->reset_gpio, 0);
        usleep_range(11000, 12000);		/* Spec says > 10 ms */

	gpiod_set_value(jdc->dcdc_en_gpio, 0);
        usleep_range(25000, 26000);

	ret = regulator_bulk_disable(ARRAY_SIZE(jdc->supplies), jdc->supplies);
	if (ret < 0)
		dev_err(dev, "regulator disable failed, %d\n", ret);

	jdc->prepared = false;

	return 0;
}

static int jdc_panel_prepare(struct drm_panel *panel)
{
	struct jdc_panel *jdc = to_jdi_panel(panel);
	struct device *dev = &jdc->dsi->dev;
	int ret;

	if (jdc->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(jdc->supplies), jdc->supplies);
	if (ret < 0) {
		dev_err(dev, "regulator enable failed, %d\n", ret);
		return ret;
	}

        usleep_range(25000, 26000);

	gpiod_set_value(jdc->dcdc_en_gpio, 1);
	gpiod_set_value(jdc->reset_gpio, 0);
	usleep_range(25000, 26000);

	gpiod_set_value(jdc->reset_gpio, 1);
	usleep_range(11000, 12000); // Spec says > 10 ms

	ret = jdc_panel_init(jdc);
	if (ret < 0) {
		dev_err(dev, "failed to init panel: %d\n", ret);
		goto poweroff;
	}
        msleep(120);

	ret = jdc_panel_on(jdc);
	if (ret < 0) {
		dev_err(dev, "failed to set panel on: %d\n", ret);
		goto poweroff;
	}

	jdc->prepared = true;

	return 0;

poweroff:

	gpiod_set_value(jdc->reset_gpio, 0);
        usleep_range(11000, 12000);		/* Spec says > 10 ms */

	gpiod_set_value(jdc->dcdc_en_gpio, 0);
        usleep_range(25000, 26000);

	ret = regulator_bulk_disable(ARRAY_SIZE(jdc->supplies), jdc->supplies);
	if (ret < 0)
		dev_err(dev, "regulator disable failed, %d\n", ret);

	return ret;
}

static int jdc_panel_enable(struct drm_panel *panel)
{
	struct jdc_panel *jdc = to_jdi_panel(panel);

	if (jdc->enabled)
		return 0;

	//backlight_enable(jdc->backlight);

	jdc->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
		.clock = 149730,
		.hdisplay = 1080,
		.hsync_start = 1080 + 129,
		.hsync_end = 1080 + 129 + 5,
		.htotal = 1080 + 129 + 5 + 75,
		.vdisplay = 1920,
		.vsync_start = 1920 + 8,
		.vsync_end = 1920 + 8 + 4,
		.vtotal = 1920 + 8 + 4 + 4,
                //.vrefresh = 60,
		.flags = 0,
};

static int jdc_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct jdc_panel *jdc = to_jdi_panel(panel);
	struct device *dev = &jdc->dsi->dev;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 62;
	connector->display_info.height_mm = 110;

	return 1;
}

/*
static int dsi_dcs_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int ret;
	u16 brightness = bl->props.brightness;

	// dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	// dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness & 0xff;
}

static int dsi_dcs_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int ret;

	// dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	// dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops dsi_bl_ops = {
	.update_status = dsi_dcs_bl_update_status,
	.get_brightness = dsi_dcs_bl_get_brightness,
};

static struct backlight_device *
drm_panel_create_dsi_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.brightness = 255;
	props.max_brightness = 255;

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &dsi_bl_ops, &props);
}
*/

static const struct drm_panel_funcs jdc_panel_funcs = {
	.disable = jdc_panel_disable,
	.unprepare = jdc_panel_unprepare,
	.prepare = jdc_panel_prepare,
	.enable = jdc_panel_enable,
	.get_modes = jdc_panel_get_modes,
};

static const struct of_device_id jdc_of_match[] = {
	{ .compatible = "jdc,mdy70", },
	{ }
};
MODULE_DEVICE_TABLE(of, jdc_of_match);

static int jdc_panel_add(struct jdc_panel *jdc)
{
	struct device *dev = &jdc->dsi->dev;
	int ret;
	unsigned int i;

	jdc->mode = &default_mode;

	for (i = 0; i < ARRAY_SIZE(jdc->supplies); i++)
		jdc->supplies[i].supply = regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(jdc->supplies),
				      jdc->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to init regulator, ret=%d\n", ret);
		return ret;
	}

        /*
	jdc->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(jdc->enable_gpio)) {
		ret = PTR_ERR(jdc->enable_gpio);
		dev_err(dev, "cannot get enable-gpio %d\n", ret);
		return ret;
	}
        */

	jdc->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(jdc->reset_gpio)) {
		ret = PTR_ERR(jdc->reset_gpio);
		dev_err(dev, "cannot get reset-gpios %d\n", ret);
		return ret;
	}

	jdc->dcdc_en_gpio = devm_gpiod_get(dev, "dcdc-en", GPIOD_OUT_LOW);
	if (IS_ERR(jdc->dcdc_en_gpio)) {
		ret = PTR_ERR(jdc->dcdc_en_gpio);
		dev_err(dev, "cannot get dcdc-en-gpio %d\n", ret);
		return ret;
	}

        /*
	jdc->backlight = drm_panel_create_dsi_backlight(jdc->dsi);
	if (IS_ERR(jdc->backlight)) {
		ret = PTR_ERR(jdc->backlight);
		dev_err(dev, "failed to register backlight %d\n", ret);
		return ret;
	}
        */

	drm_panel_init(&jdc->base, &jdc->dsi->dev, &jdc_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&jdc->base);

	return 0;
}

static void jdc_panel_del(struct jdc_panel *jdc)
{
	if (jdc->base.dev)
		drm_panel_remove(&jdc->base);
}

static int jdc_panel_probe(struct mipi_dsi_device *dsi)
{
	struct jdc_panel *jdc;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_CLOCK_NON_CONTINUOUS;

	jdc = devm_kzalloc(&dsi->dev, sizeof(*jdc), GFP_KERNEL);
	if (!jdc)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, jdc);

	jdc->dsi = dsi;

	ret = jdc_panel_add(jdc);
	if (ret < 0)
		return ret;

	return mipi_dsi_attach(dsi);
}

static int jdc_panel_remove(struct mipi_dsi_device *dsi)
{
	struct jdc_panel *jdc = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = jdc_panel_disable(&jdc->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n",
			ret);

	jdc_panel_del(jdc);

	return 0;
}

static void jdc_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct jdc_panel *jdc = mipi_dsi_get_drvdata(dsi);

	jdc_panel_disable(&jdc->base);
}

static struct mipi_dsi_driver jdc_panel_driver = {
	.driver = {
		.name = "panel-jdc-mdy70",
		.of_match_table = jdc_of_match,
	},
	.probe = jdc_panel_probe,
	.remove = jdc_panel_remove,
	.shutdown = jdc_panel_shutdown,
};
module_mipi_dsi_driver(jdc_panel_driver);

MODULE_AUTHOR("Wouter Franken <wouter.franken@gmail.com>");
MODULE_DESCRIPTION("JDC MDY70 FWVGA");
MODULE_LICENSE("GPL v2");
