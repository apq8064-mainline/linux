/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SLIMBUS_QCOM_H
#define _LINUX_SLIMBUS_QCOM_H

#include <linux/errno.h>
#include <linux/scatterlist.h>
#include <linux/slimbus.h>

#if IS_ENABLED(CONFIG_SLIM_QCOM_CTRL)
int qcom_slim_port_xfer(struct slim_controller *ctrl, u8 pn,
			struct scatterlist *sgl, unsigned int sg_len,
			void (*callback)(void *), void *cookie);
void qcom_slim_port_xfer_stop(struct slim_controller *ctrl, u8 pn);
int qcom_slim_port_enable_hw(struct slim_controller *ctrl, u8 pn);
void qcom_slim_port_disable_hw(struct slim_controller *ctrl, u8 pn);
#else
static inline int qcom_slim_port_xfer(struct slim_controller *ctrl, u8 pn,
				      struct scatterlist *sgl,
				      unsigned int sg_len,
				      void (*callback)(void *),
				      void *cookie)
{
	return -ENODEV;
}

static inline void qcom_slim_port_xfer_stop(struct slim_controller *ctrl,
					    u8 pn)
{
}

static inline int qcom_slim_port_enable_hw(struct slim_controller *ctrl, u8 pn)
{
	return -ENODEV;
}

static inline void qcom_slim_port_disable_hw(struct slim_controller *ctrl,
					     u8 pn)
{
}

#endif

#endif
