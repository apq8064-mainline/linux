// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung ICE4 FPGA GPIO expander
 *
 * Some Samsung APQ8064 devices use a Lattice ICE4 FPGA for board control
 * GPIOs.  The GPIO register interface is exposed after the FPGA bitstream is
 * loaded over two GPIO lines that are then reused as an I2C-like bus.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#define ICE4_I2C_ADDR		0x6c
#define ICE4_NGPIO		16
#define ICE4_WLAN_EN_GPIO	3
#define ICE4_FW_RETRIES		2

struct samsung_ice4 {
	struct device *dev;
	struct gpio_chip gc;
	struct gpio_desc *scl;
	struct gpio_desc *sda;
	struct gpio_desc *creset;
	struct gpio_desc *reset;
	struct gpio_desc *cdone;
	struct regulator_bulk_data supplies[3];
	struct clk *clk;
	struct mutex lock;
	struct i2c_algo_bit_data bit_data;
	struct i2c_adapter adapter;
	struct i2c_client *client;
	u16 state;
	unsigned int bus_warns;
	bool reg_high_base;
};

static int ice4_drive_output(struct gpio_desc *desc, int value)
{
	int ret;

	ret = gpiod_direction_output(desc, 0);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(desc, value);

	return 0;
}

static int ice4_setscl(struct samsung_ice4 *ice4, int value)
{
	return ice4_drive_output(ice4->scl, value);
}

static int ice4_setsda(struct samsung_ice4 *ice4, int value)
{
	if (value)
		return gpiod_direction_input(ice4->sda);

	return ice4_drive_output(ice4->sda, 0);
}

static int ice4_set_program_sda(struct samsung_ice4 *ice4, int value)
{
	return ice4_drive_output(ice4->sda, value);
}

static int ice4_getsda(struct samsung_ice4 *ice4)
{
	return gpiod_get_value_cansleep(ice4->sda);
}

static int ice4_getscl(struct samsung_ice4 *ice4)
{
	return gpiod_get_value_cansleep(ice4->scl);
}

static void ice4_log_bus(struct samsung_ice4 *ice4, const char *op,
				u8 reg, int value, int ret)
{
	dev_info(ice4->dev,
		 "%s: reg=0x%02x value=0x%02x ret=%d reset=%d cdone=%d scl=%d sda=%d state=0x%04x\n",
		 op, reg, value & 0xff, ret,
		 gpiod_get_value_cansleep(ice4->reset),
		 gpiod_get_value_cansleep(ice4->cdone), ice4_getscl(ice4),
		 ice4_getsda(ice4), ice4->state);
}

static void ice4_warn_if_bus_stuck(struct samsung_ice4 *ice4, const char *op)
{
	int scl = ice4_getscl(ice4);
	int sda = ice4_getsda(ice4);

	if (scl > 0 && sda > 0)
		return;

	if (ice4->bus_warns++ >= 8)
		return;

	dev_err(ice4->dev,
		 "%s: bus not idle after release: reset=%d cdone=%d scl=%d sda=%d warn=%u\n",
		 op, gpiod_get_value_cansleep(ice4->reset),
		 gpiod_get_value_cansleep(ice4->cdone), scl, sda,
		 ice4->bus_warns);
}

static void ice4_bit_setsda(void *data, int value)
{
	struct samsung_ice4 *ice4 = data;
	int ret;

	ret = ice4_setsda(ice4, value);
	if (ret)
		dev_err(ice4->dev, "failed to drive SDA=%d: %d\n", value, ret);
}

static void ice4_bit_setscl(void *data, int value)
{
	struct samsung_ice4 *ice4 = data;
	int ret;

	ret = ice4_setscl(ice4, value);
	if (ret)
		dev_err(ice4->dev, "failed to drive SCL=%d: %d\n", value, ret);
}

static int ice4_bit_getsda(void *data)
{
	return ice4_getsda(data);
}

static int ice4_bit_getscl(void *data)
{
	return ice4_getscl(data);
}

static int ice4_enable_access(struct samsung_ice4 *ice4)
{
	int ret_scl, ret_sda;
	int ret;

	if (ice4->clk) {
		ret = clk_prepare_enable(ice4->clk);
		if (ret)
			return ret;
	}

	gpiod_set_value_cansleep(ice4->reset, 1);
	usleep_range(1000, 2000);
	ret_sda = ice4_setsda(ice4, 1);
	ret_scl = ice4_setscl(ice4, 1);
	usleep_range(1000, 2000);

	dev_info(ice4->dev,
		 "ICE4 access enabled: reset=%d cdone=%d scl=%d sda=%d set_scl=%d set_sda=%d clk=%lu\n",
		 gpiod_get_value_cansleep(ice4->reset),
		 gpiod_get_value_cansleep(ice4->cdone),
		 ice4_getscl(ice4), ice4_getsda(ice4), ret_scl, ret_sda,
		 ice4->clk ? clk_get_rate(ice4->clk) : 0);
	ice4_warn_if_bus_stuck(ice4, "enable_access");

	if (ret_sda || ret_scl) {
		gpiod_set_value_cansleep(ice4->reset, 0);
		if (ice4->clk)
			clk_disable_unprepare(ice4->clk);
		return ret_sda ?: ret_scl;
	}

	return 0;
}

static void ice4_disable_access(struct samsung_ice4 *ice4)
{
	usleep_range(20000, 25000);
	gpiod_set_value_cansleep(ice4->reset, 0);

	if (ice4->clk)
		clk_disable_unprepare(ice4->clk);

	dev_info(ice4->dev, "ICE4 access disabled: reset=%d cdone=%d\n",
		 gpiod_get_value_cansleep(ice4->reset),
		 gpiod_get_value_cansleep(ice4->cdone));
}

static int ice4_retry_access(struct samsung_ice4 *ice4)
{
	ice4_disable_access(ice4);

	return ice4_enable_access(ice4);
}

static int ice4_write_reg(struct samsung_ice4 *ice4, u8 reg, u8 value)
{
	int ret;

	ret = ice4_enable_access(ice4);
	if (ret)
		return ret;

	ice4_log_bus(ice4, "before write", reg, value, 0);
	ret = i2c_smbus_write_byte_data(ice4->client, reg, value);
	ice4_log_bus(ice4, ret ? "after failed write" : "after write", reg, value, ret);
	if (ret) {
		dev_info(ice4->dev,
			 "write reg 0x%02x = 0x%02x failed %d, retrying\n",
			 reg, value, ret);
		ret = ice4_retry_access(ice4);
		if (ret)
			return ret;
		ice4_log_bus(ice4, "before write retry", reg, value, 0);
		ret = i2c_smbus_write_byte_data(ice4->client, reg, value);
		ice4_log_bus(ice4, ret ? "after failed write retry" : "after write retry", reg, value, ret);
	}

	ice4_disable_access(ice4);

	return ret;
}

static int ice4_read_reg(struct samsung_ice4 *ice4, u8 reg)
{
	int ret;

	ret = ice4_enable_access(ice4);
	if (ret)
		return ret;

	ice4_log_bus(ice4, "before read", reg, 0, 0);
	ret = i2c_smbus_read_byte_data(ice4->client, reg);
	ice4_log_bus(ice4, ret < 0 ? "after failed read" : "after read", reg, ret, ret);
	if (ret < 0) {
		dev_info(ice4->dev, "read reg 0x%02x failed %d, retrying\n",
			 reg, ret);
		ret = ice4_retry_access(ice4);
		if (ret)
			return ret;
		ice4_log_bus(ice4, "before read retry", reg, 0, 0);
		ret = i2c_smbus_read_byte_data(ice4->client, reg);
		ice4_log_bus(ice4, ret < 0 ? "after failed read retry" : "after read retry", reg, ret, ret);
	}

	ice4_disable_access(ice4);

	return ret;
}

static u8 ice4_gpio_reg(struct samsung_ice4 *ice4, unsigned int offset)
{
	if (ice4->reg_high_base)
		return 0xfc | (offset >> 3);

	return offset >> 3;
}

static int ice4_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct samsung_ice4 *ice4 = gpiochip_get_data(gc);
	int ret;

	mutex_lock(&ice4->lock);
	ret = ice4_read_reg(ice4, ice4_gpio_reg(ice4, offset));
	mutex_unlock(&ice4->lock);

	if (ret < 0)
		return ret;

	return !!(ret & BIT(offset & 7));
}

static int ice4_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct samsung_ice4 *ice4 = gpiochip_get_data(gc);
	u8 reg, regval;
	int readback, ret;

	mutex_lock(&ice4->lock);

	if (offset == ICE4_WLAN_EN_GPIO)
		dev_info(ice4->dev,
			 "GPIO%u access pre-state: reset=%d cdone=%d scl=%d sda=%d\n",
			 offset, gpiod_get_value_cansleep(ice4->reset),
			 gpiod_get_value_cansleep(ice4->cdone),
			 ice4_getscl(ice4), ice4_getsda(ice4));

	if (value)
		ice4->state |= BIT(offset);
	else
		ice4->state &= ~BIT(offset);

	reg = ice4_gpio_reg(ice4, offset);
	regval = offset >= 8 ? ice4->state >> 8 : ice4->state;
	ret = ice4_write_reg(ice4, reg, regval);
	if (offset == ICE4_WLAN_EN_GPIO) {
		readback = ret ? ret : ice4_read_reg(ice4, reg);
		dev_info(ice4->dev,
			 "GPIO%u -> %d (reg 0x%02x = 0x%02x): %d, readback 0x%02x bit %d\n",
			 offset, value, reg, regval, ret,
			 readback < 0 ? 0xff : readback,
			 readback < 0 ? -1 : !!(readback & BIT(offset & 7)));
	}
	if (ret)
		dev_err(ice4->dev, "failed to set GPIO%u: %d\n", offset, ret);

	mutex_unlock(&ice4->lock);

	return ret;
}

static int ice4_program(struct samsung_ice4 *ice4, const u8 *data, size_t len)
{
	size_t i;
	int bit, ret;

	if (ice4->clk) {
		ret = clk_prepare_enable(ice4->clk);
		if (ret)
			return ret;
	}

	gpiod_set_value_cansleep(ice4->reset, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ice4->reset, 0);
	gpiod_set_value_cansleep(ice4->creset, 0);
	usleep_range(30, 50);

	gpiod_set_value_cansleep(ice4->creset, 1);
	usleep_range(1000, 1300);

	for (i = 0; i < len; i++) {
		for (bit = 7; bit >= 0; bit--) {
			ret = ice4_setscl(ice4, 0);
			if (ret)
				goto out_disable;

			ret = ice4_set_program_sda(ice4, !!(data[i] & BIT(bit)));
			if (ret)
				goto out_disable;

			ret = ice4_setscl(ice4, 1);
			if (ret)
				goto out_disable;
		}
	}

	for (i = 0; i < 200; i++) {
		ret = ice4_setscl(ice4, 0);
		if (ret)
			goto out_disable;

		ret = ice4_setscl(ice4, 1);
		if (ret)
			goto out_disable;
	}

	usleep_range(50, 70);

	dev_info(ice4->dev, "CDONE after firmware load: %d\n",
		 gpiod_get_value_cansleep(ice4->cdone));
	if (!gpiod_get_value_cansleep(ice4->cdone)) {
		ret = -EIO;
		goto out_disable;
	}

	ret = 0;

out_disable:
	usleep_range(20000, 25000);
	gpiod_set_value_cansleep(ice4->reset, 0);
	if (ice4->clk)
		clk_disable_unprepare(ice4->clk);
	dev_info(ice4->dev, "firmware access disabled after load: ret=%d reset=%d cdone=%d\n",
		 ret, gpiod_get_value_cansleep(ice4->reset),
		 gpiod_get_value_cansleep(ice4->cdone));

	return ret;
}

static int ice4_load_firmware(struct samsung_ice4 *ice4)
{
	const struct firmware *fw;
	const char *fw_name;
	int retry, ret;

	ret = device_property_read_string(ice4->dev, "firmware-name", &fw_name);
	if (ret)
		fw_name = "samsung/ice4.bin";

	dev_info(ice4->dev, "loading firmware %s\n", fw_name);

	ret = request_firmware(&fw, fw_name, ice4->dev);
	if (ret)
		return dev_err_probe(ice4->dev, ret,
				     "failed to load %s\n", fw_name);

	dev_info(ice4->dev, "firmware %s loaded, %zu bytes\n",
		 fw_name, fw->size);

	for (retry = 0; retry < ICE4_FW_RETRIES; retry++) {
		dev_info(ice4->dev, "programming attempt %d/%d\n",
			 retry + 1, ICE4_FW_RETRIES);
		ret = ice4_program(ice4, fw->data, fw->size);
		if (!ret)
			break;
	}

	release_firmware(fw);

	if (ret)
		return dev_err_probe(ice4->dev, ret,
				     "failed to program ICE4 FPGA\n");

	dev_info(ice4->dev, "firmware programmed successfully\n");

	return 0;
}

static void samsung_ice4_disable_regulators(void *data)
{
	struct samsung_ice4 *ice4 = data;

	regulator_bulk_disable(ARRAY_SIZE(ice4->supplies), ice4->supplies);
}

static void samsung_ice4_unregister_client(void *data)
{
	i2c_unregister_device(data);
}

static void samsung_ice4_del_adapter(void *data)
{
	i2c_del_adapter(data);
}

static void ice4_scan_i2c(struct samsung_ice4 *ice4)
{
	union i2c_smbus_data data;
	int addr, found = 0;
	int ret;

	ret = ice4_enable_access(ice4);
	if (ret) {
		dev_err(ice4->dev, "ICE4 scan: failed to enable access: %d\n", ret);
		return;
	}

	for (addr = 0x60; addr <= 0x77; addr++) {
		memset(&data, 0, sizeof(data));
		ret = i2c_smbus_xfer(&ice4->adapter, addr, 0, I2C_SMBUS_READ,
				     0, I2C_SMBUS_BYTE, &data);
		if (!ret) {
			found++;
			dev_err(ice4->dev,
				"ICE4 scan: address 0x%02x ACK byte=0x%02x\n",
				addr, data.byte);
		} else if (addr == ICE4_I2C_ADDR) {
			dev_err(ice4->dev,
				"ICE4 scan: expected address 0x%02x ret=%d\n",
				addr, ret);
		}
	}

	dev_err(ice4->dev, "ICE4 scan: found %d responding address(es)\n",
		found);
	ice4_disable_access(ice4);
}

static int samsung_ice4_register_i2c(struct samsung_ice4 *ice4)
{
	int ret;

	ice4->bit_data.data = ice4;
	ice4->bit_data.setsda = ice4_bit_setsda;
	ice4->bit_data.setscl = ice4_bit_setscl;
	ice4->bit_data.getsda = ice4_bit_getsda;
	ice4->bit_data.getscl = ice4_bit_getscl;
	ice4->bit_data.udelay = 5;
	ice4->bit_data.timeout = msecs_to_jiffies(100);

	ice4->adapter.owner = THIS_MODULE;
	ice4->adapter.algo_data = &ice4->bit_data;
	ice4->adapter.dev.parent = ice4->dev;
	strscpy(ice4->adapter.name, "samsung-ice4-gpio",
		sizeof(ice4->adapter.name));

	ret = i2c_bit_add_bus(&ice4->adapter);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(ice4->dev, samsung_ice4_del_adapter,
		&ice4->adapter);
	if (ret)
		return ret;

	ice4_scan_i2c(ice4);

	ice4->client = i2c_new_dummy_device(&ice4->adapter, ICE4_I2C_ADDR);
	if (IS_ERR(ice4->client))
		return PTR_ERR(ice4->client);

	return devm_add_action_or_reset(ice4->dev,
		samsung_ice4_unregister_client, ice4->client);
}

static int samsung_ice4_probe(struct platform_device *pdev)
{
	struct samsung_ice4 *ice4;
	struct gpio_chip *gc;
	int ret;

	ice4 = devm_kzalloc(&pdev->dev, sizeof(*ice4), GFP_KERNEL);
	if (!ice4)
		return -ENOMEM;

	ice4->dev = &pdev->dev;
	mutex_init(&ice4->lock);
	ice4->reg_high_base = device_property_read_bool(&pdev->dev,
							"samsung,high-gpio-regs");
	dev_info(&pdev->dev, "probe start, gpio register base %s\n",
		 ice4->reg_high_base ? "high" : "low");

	ice4->supplies[0].supply = "vdd";
	ice4->supplies[1].supply = "vio";
	ice4->supplies[2].supply = "vcore";
	ret = devm_regulator_bulk_get(&pdev->dev, ARRAY_SIZE(ice4->supplies),
					 ice4->supplies);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(ice4->supplies), ice4->supplies);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to enable regulators\n");
	dev_info(&pdev->dev, "regulators enabled\n");

	ret = devm_add_action_or_reset(&pdev->dev, samsung_ice4_disable_regulators,
				       ice4);
	if (ret)
		return ret;

	ice4->scl = devm_gpiod_get(&pdev->dev, "scl", GPIOD_OUT_LOW);
	if (IS_ERR(ice4->scl))
		return PTR_ERR(ice4->scl);

	ice4->sda = devm_gpiod_get(&pdev->dev, "sda", GPIOD_OUT_LOW);
	if (IS_ERR(ice4->sda))
		return PTR_ERR(ice4->sda);

	ice4->creset = devm_gpiod_get(&pdev->dev, "creset", GPIOD_OUT_LOW);
	if (IS_ERR(ice4->creset))
		return PTR_ERR(ice4->creset);

	ice4->reset = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ice4->reset))
		return PTR_ERR(ice4->reset);

	ice4->cdone = devm_gpiod_get(&pdev->dev, "cdone", GPIOD_IN);
	if (IS_ERR(ice4->cdone))
		return PTR_ERR(ice4->cdone);
	dev_info(&pdev->dev, "GPIO lines acquired\n");

	ice4->clk = devm_clk_get_optional(&pdev->dev, "core");
	if (IS_ERR(ice4->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(ice4->clk),
				     "failed to get core clock\n");

	if (ice4->clk) {
		ret = clk_set_rate(ice4->clk, 24000000);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to set core clock rate\n");

		dev_info(&pdev->dev, "core clock set to %lu Hz\n",
			 clk_get_rate(ice4->clk));
	} else {
		dev_info(&pdev->dev, "no core clock provided\n");
	}

	ret = ice4_load_firmware(ice4);
	if (ret)
		return ret;

	ret = samsung_ice4_register_i2c(ice4);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
			"failed to register ICE4 I2C adapter\n");

	gc = &ice4->gc;
	gc->label = dev_name(&pdev->dev);
	gc->parent = &pdev->dev;
	gc->owner = THIS_MODULE;
	gc->get = ice4_gpio_get;
	gc->set = ice4_gpio_set;
	gc->base = -1;
	gc->ngpio = ICE4_NGPIO;
	gc->can_sleep = true;

	ret = devm_gpiochip_add_data(&pdev->dev, gc, ice4);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "gpiochip registered with %u GPIOs\n", ICE4_NGPIO);

	return 0;
}

static const struct of_device_id samsung_ice4_of_match[] = {
	{ .compatible = "samsung,ice4-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, samsung_ice4_of_match);

static struct platform_driver samsung_ice4_driver = {
	.probe = samsung_ice4_probe,
	.driver = {
		.name = "gpio-samsung-ice4",
		.of_match_table = samsung_ice4_of_match,
	},
};
module_platform_driver(samsung_ice4_driver);

MODULE_DESCRIPTION("Samsung ICE4 FPGA GPIO expander");
MODULE_LICENSE("GPL");
