// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define PM8917_L33_CTRL			0x0c6
#define PM8917_L33_TEST			0x0c7

#define PM8917_LDO_ENABLE		BIT(7)
#define PM8917_LDO_PULL_DOWN		BIT(6)
#define PM8917_LDO_VPROG_MASK		GENMASK(4, 0)

/*
 * Program 1.2 V in the PLDO low range:
 * 750 mV + 36 * 12.5 mV, with VPROG[5:1] = 18.
 */
#define PM8917_L33_1P2_VPROG		0x12
#define PM8917_L33_TEST_BANK2_1P2	0xac

static int pm8917_l33_enable(struct regulator_dev *rdev)
{
	unsigned int before = 0;
	unsigned int after;
	int read_ret;
	int ret;

	read_ret = regmap_read(rdev->regmap, PM8917_L33_CTRL, &before);
	ret = regulator_enable_regmap(rdev);
	if (ret)
		return ret;

	regmap_read(rdev->regmap, PM8917_L33_CTRL, &after);

	return 0;
}

static const struct regulator_ops pm8917_l33_ops = {
	.enable = pm8917_l33_enable,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc pm8917_l33_desc = {
	.name = "8917_l33",
	.of_match = "regulator-l33",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &pm8917_l33_ops,
	.fixed_uV = 1200000,
	.n_voltages = 1,
	.enable_reg = PM8917_L33_CTRL,
	.enable_mask = PM8917_LDO_ENABLE,
	.enable_time = 200,
};

static int pm8917_l33_probe(struct platform_device *pdev)
{
	struct regulator_config config = {
		.dev = &pdev->dev,
		.of_node = pdev->dev.of_node,
	};
	struct regulator_dev *rdev;
	int ret;

	config.regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!config.regmap)
		return -ENODEV;

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

	rdev = devm_regulator_register(&pdev->dev, &pm8917_l33_desc,
				       &config);

	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id pm8917_l33_match[] = {
	{ .compatible = "qcom,pm8917-l33" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8917_l33_match);

static struct platform_driver pm8917_l33_driver = {
	.probe = pm8917_l33_probe,
	.driver = {
		.name = "pm8917-l33",
		.of_match_table = pm8917_l33_match,
	},
};
module_platform_driver(pm8917_l33_driver);

MODULE_DESCRIPTION("Qualcomm PM8917 L33 regulator");
MODULE_LICENSE("GPL");
