// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define PM8917_L33_CTRL			0x0c6
#define PM8917_L33_TEST			0x0c7
#define PM8917_L34_CTRL			0x0d2
#define PM8917_L34_TEST			0x0d3

#define PM8917_LDO_ENABLE		BIT(7)
#define PM8917_LDO_PULL_DOWN		BIT(6)
#define PM8917_LDO_LOW_POWER		BIT(5)
#define PM8917_LDO_VPROG_MASK		GENMASK(4, 0)

/*
 * Program 1.2 V in the PLDO low range:
 * 750 mV + 36 * 12.5 mV, with VPROG[5:1] = 18.
 */
#define PM8917_L33_1P2_VPROG		0x12
#define PM8917_L33_TEST_BANK2_1P2	0xac

static const struct regulator_ops pm8917_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc pm8917_l33_desc = {
	.name = "8917_l33",
	.of_match = "regulator-l33",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &pm8917_ldo_ops,
	.fixed_uV = 1200000,
	.n_voltages = 1,
	.enable_reg = PM8917_L33_CTRL,
	.enable_mask = PM8917_LDO_ENABLE,
	.enable_time = 200,
};

static const struct regulator_desc pm8917_l34_desc = {
	.name = "8917_l34",
	.of_match = "regulator-l34",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &pm8917_ldo_ops,
	.fixed_uV = 2200000,
	.n_voltages = 1,
	.enable_reg = PM8917_L34_CTRL,
	.enable_mask = PM8917_LDO_ENABLE,
	.enable_time = 200,
};

static int pm8917_l34_init(struct regmap *regmap)
{
	unsigned int val;
	int ret;

	/* Select test bank 2 without writing its contents. */
	ret = regmap_write(regmap, PM8917_L34_TEST, 0x20);
	if (ret)
		return ret;
	ret = regmap_read(regmap, PM8917_L34_TEST, &val);
	if (ret)
		return ret;
	/* Normal range, no fine step, voltage-update latch set. */
	ret = regmap_write(regmap, PM8917_L34_TEST, (val & 0x01) | 0xa8);
	if (ret)
		return ret;

	/* Clear range extension in bank 4, preserving its other bits. */
	ret = regmap_write(regmap, PM8917_L34_TEST, 0x40);
	if (ret)
		return ret;
	ret = regmap_read(regmap, PM8917_L34_TEST, &val);
	if (ret)
		return ret;
	ret = regmap_write(regmap, PM8917_L34_TEST, (val & 0x0e) | 0xc0);
	if (ret)
		return ret;

	ret = regmap_read(regmap, PM8917_L34_CTRL, &val);
	if (ret)
		return ret;
	/*
	 * 1.5 V + (VPROG * 2) * 25 mV = 2.2 V for VPROG=14.
	 * Use high-power mode for the codec analog load, with pull-down.
	 * Force the CTRL write to latch the test-bank voltage settings.
	 */
	val &= ~(PM8917_LDO_LOW_POWER | PM8917_LDO_VPROG_MASK);
	val |= PM8917_LDO_PULL_DOWN | 14;
	return regmap_write(regmap, PM8917_L34_CTRL, val);
}

static int pm8917_ldo_probe(struct platform_device *pdev)
{
	const struct regulator_desc *desc = device_get_match_data(&pdev->dev);
	struct regulator_config config = {
		.dev = &pdev->dev,
		.of_node = pdev->dev.of_node,
	};
	struct regulator_dev *rdev;
	int ret;

	config.regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!config.regmap)
		return -ENODEV;

	if (desc == &pm8917_l34_desc) {
		ret = pm8917_l34_init(config.regmap);
		if (ret)
			return ret;
		goto register_ldo;
	}

	ret = regmap_write(config.regmap, PM8917_L33_TEST,
			   PM8917_L33_TEST_BANK2_1P2);
	if (ret)
		return ret;

	ret = regmap_update_bits(config.regmap, PM8917_L33_CTRL,
				 PM8917_LDO_PULL_DOWN |
				 PM8917_LDO_VPROG_MASK,
				 PM8917_LDO_PULL_DOWN |
				 PM8917_L33_1P2_VPROG);
	if (ret)
		return ret;

register_ldo:
	rdev = devm_regulator_register(&pdev->dev, desc, &config);

	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id pm8917_ldo_match[] = {
	{ .compatible = "qcom,pm8917-l33", .data = &pm8917_l33_desc },
	{ .compatible = "qcom,pm8917-l34", .data = &pm8917_l34_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8917_ldo_match);

static struct platform_driver pm8917_ldo_driver = {
	.probe = pm8917_ldo_probe,
	.driver = {
		.name = "pm8917-ldo",
		.of_match_table = pm8917_ldo_match,
	},
};
module_platform_driver(pm8917_ldo_driver);

MODULE_DESCRIPTION("Qualcomm PM8917 L33/L34 regulators");
MODULE_LICENSE("GPL");
