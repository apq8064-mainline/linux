// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2018, 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#include <linux/clk/clk-conf.h>

#include "msm_drv.h"

/*
 * Util/helpers:
 */

struct clk *msm_clk_bulk_get_clock(struct clk_bulk_data *bulk, int count,
		const char *name)
{
	int i;
	char n[32];

	snprintf(n, sizeof(n), "%s_clk", name);

	for (i = 0; bulk && i < count; i++) {
		if (!strcmp(bulk[i].id, name) || !strcmp(bulk[i].id, n))
			return bulk[i].clk;
	}


	return NULL;
}

struct clk *msm_clk_get(struct platform_device *pdev, const char *name)
{
	struct clk *clk;
	char name2[32];

	clk = devm_clk_get(&pdev->dev, name);
	if (!IS_ERR(clk) || PTR_ERR(clk) == -EPROBE_DEFER)
		return clk;

	snprintf(name2, sizeof(name2), "%s_clk", name);

	clk = devm_clk_get(&pdev->dev, name2);
	if (!IS_ERR(clk))
		dev_warn(&pdev->dev, "Using legacy clk name binding.  Use "
				"\"%s\" instead of \"%s\"\n", name, name2);

	return clk;
}

int msm_parse_clock(struct platform_device *pdev, struct clk_bulk_data **clocks)
{
	u32 i, rc = 0;
	const char *clock_name;
	struct clk_bulk_data *bulk;
	int num_clk = 0;

	if (!pdev)
		return -EINVAL;

	num_clk = of_property_count_strings(pdev->dev.of_node, "clock-names");
	if (num_clk <= 0) {
		pr_debug("clocks are not defined\n");
		return 0;
	}

	bulk = devm_kcalloc(&pdev->dev, num_clk, sizeof(struct clk_bulk_data), GFP_KERNEL);
	if (!bulk)
		return -ENOMEM;

	for (i = 0; i < num_clk; i++) {
		rc = of_property_read_string_index(pdev->dev.of_node,
						   "clock-names", i,
						   &clock_name);
		if (rc) {
			DRM_DEV_ERROR(&pdev->dev, "Failed to get clock name for %d\n", i);
			return rc;
		}
		bulk[i].id = devm_kstrdup(&pdev->dev, clock_name, GFP_KERNEL);
	}

	rc = devm_clk_bulk_get(&pdev->dev, num_clk, bulk);
	if (rc) {
		DRM_DEV_ERROR(&pdev->dev, "Failed to get clock refs %d\n", rc);
		return rc;
	}

	rc = of_clk_set_defaults(pdev->dev.of_node, false);
	if (rc) {
		DRM_DEV_ERROR(&pdev->dev, "Failed to set clock defaults %d\n", rc);
		return rc;
	}

	*clocks = bulk;

	return num_clk;
}

static void __iomem *_msm_ioremap(struct platform_device *pdev, const char *name,
				  bool quiet, phys_addr_t *psize)
{
	struct resource *res;
	unsigned long size;
	void __iomem *ptr;

	if (name)
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	else
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		if (!quiet)
			DRM_DEV_ERROR(&pdev->dev, "failed to get memory resource: %s\n", name);
		return ERR_PTR(-EINVAL);
	}

	size = resource_size(res);

	ptr = devm_ioremap(&pdev->dev, res->start, size);
	if (!ptr) {
		if (!quiet)
			DRM_DEV_ERROR(&pdev->dev, "failed to ioremap: %s\n", name);
		return ERR_PTR(-ENOMEM);
	}

	if (psize)
		*psize = size;

	return ptr;
}

void __iomem *msm_ioremap(struct platform_device *pdev, const char *name)
{
	return _msm_ioremap(pdev, name, false, NULL);
}

void __iomem *msm_ioremap_quiet(struct platform_device *pdev, const char *name)
{
	return _msm_ioremap(pdev, name, true, NULL);
}

void __iomem *msm_ioremap_size(struct platform_device *pdev, const char *name,
			  phys_addr_t *psize)
{
	return _msm_ioremap(pdev, name, false, psize);
}

static enum hrtimer_restart msm_hrtimer_worktimer(struct hrtimer *t)
{
	struct msm_hrtimer_work *work = container_of(t,
			struct msm_hrtimer_work, timer);

	kthread_queue_work(work->worker, &work->work);

	return HRTIMER_NORESTART;
}

void msm_hrtimer_queue_work(struct msm_hrtimer_work *work,
			    ktime_t wakeup_time,
			    enum hrtimer_mode mode)
{
	hrtimer_start(&work->timer, wakeup_time, mode);
}

void msm_hrtimer_work_init(struct msm_hrtimer_work *work,
			   struct kthread_worker *worker,
			   kthread_work_func_t fn,
			   clockid_t clock_id,
			   enum hrtimer_mode mode)
{
	hrtimer_init(&work->timer, clock_id, mode);
	work->timer.function = msm_hrtimer_worktimer;
	work->worker = worker;
	kthread_init_work(&work->work, fn);
}
