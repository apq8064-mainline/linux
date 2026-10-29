// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2011-2017, The Linux Foundation
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "slimbus.h"
#include <linux/slimbus/qcom.h>

/* Manager registers */
#define	MGR_CFG		0x200
#define	MGR_STATUS	0x204
#define	MGR_INT_EN	0x210
#define	MGR_INT_STAT	0x214
#define	MGR_INT_CLR	0x218
#define	MGR_TX_MSG	0x230
#define	MGR_RX_MSG	0x270
#define	MGR_IE_STAT	0x2F0
#define	MGR_VE_STAT	0x300
#define	MGR_CFG_ENABLE	1

/* Framer registers */
#define	FRM_CFG		0x400
#define	FRM_STAT	0x404
#define	FRM_INT_EN	0x410
#define	FRM_INT_STAT	0x414
#define	FRM_INT_CLR	0x418
#define	FRM_WAKEUP	0x41C
#define	FRM_CLKCTL_DONE	0x420
#define	FRM_IE_STAT	0x430
#define	FRM_VE_STAT	0x440

/* Interface registers */
#define	INTF_CFG	0x600
#define	INTF_STAT	0x604
#define	INTF_INT_EN	0x610
#define	INTF_INT_STAT	0x614
#define	INTF_INT_CLR	0x618
#define	INTF_IE_STAT	0x630
#define	INTF_VE_STAT	0x640

/* Interrupt status bits */
#define	MGR_INT_TX_NACKED_2	BIT(25)
#define	MGR_INT_MSG_BUF_CONTE	BIT(26)
#define	MGR_INT_RX_MSG_RCVD	BIT(30)
#define	MGR_INT_TX_MSG_SENT	BIT(31)

/* Framer config register settings */
#define	FRM_ACTIVE	1
#define	CLK_GEAR	7
#define	ROOT_FREQ	11
#define	REF_CLK_GEAR	15
#define	INTR_WAKE	19

#define SLIM_MSG_ASM_FIRST_WORD(l, mt, mc, dt, ad) \
		((l) | ((mt) << 5) | ((mc) << 8) | ((dt) << 15) | ((ad) << 16))

#define SLIM_ROOT_FREQ 24576000
#define QCOM_SLIM_AUTOSUSPEND 1000

/* MAX message size over control channel */
#define SLIM_MSGQ_BUF_LEN	40
#define QCOM_TX_MSGS 2
#define QCOM_RX_MSGS	8
#define QCOM_BUF_ALLOC_RETRIES	10

#define CFG_PORT(r, v) ((v) ? CFG_PORT_V2(r) : CFG_PORT_V1(r))

/* V2 Component registers */
#define CFG_PORT_V2(r) ((r ## _V2))
#define	COMP_CFG_V2		4
#define	COMP_TRUST_CFG_V2	0x3000
#define	PGD_CFG_V2		0x800
#define	PGD_OWN_EE_V2		0x300C

/* V1 Component registers */
#define CFG_PORT_V1(r) ((r ## _V1))
#define	COMP_CFG_V1		0
#define	COMP_TRUST_CFG_V1	0x14
#define	PGD_CFG_V1		0x1000
#define	PGD_OWN_EE_V1		0x1020

/* Resource group info for manager, and non-ported generic device-components */
#define EE_MGR_RSC_GRP	(1 << 10)
#define EE_NGD_2	(2 << 6)
#define EE_NGD_1	0

/*
 * Manager data ports (V1 hardware): logical ports 0-6 map to hardware
 * ports 17-23, paired 1:1 with pipes of the slimbus BAM. Per-port
 * register blocks are 32 bytes, per-EE interrupt registers stride 16.
 * The CPU (apps) is execution environment 1.
 */
/* Device-index field of the Qualcomm manager PGD enumeration address. */
#define QCOM_PGD_DEV_INDEX	5

#define SLIM_PORT_START		17
#define SLIM_NMGR_PORTS		7

#define PGD_PORT_INT_EN_EE_V1	0x1030
#define PGD_PORT_INT_ST_EE_V1	0x1034
#define PGD_PORT_INT_CL_EE_V1	0x1038
#define PGD_PORT_CFG_V1		0x1080
#define PGD_PORT_STAT_V1	0x1084
#define PGD_PORT_BLK_V1		0x108c
#define PGD_PORT_TRAN_V1	0x1090

#define PGD_EE			1
#define PGD_PORT_EE(r)		((r) + PGD_EE * 16)
#define PGD_PORT(r, p)		((r) + (p) * 32)

/* port config: watermark 8 bytes, packed, enable */
#define PORT_CFG_VAL		((8 << 1) | BIT(6) | 1)

/* port STAT bits */
#define PORT_STAT_OVERFLOW	BIT(2)
#define PORT_STAT_UNDERFLOW	BIT(3)
#define PORT_STAT_DISCONNECT	BIT(19)

#define MGR_INT_RECFG_DONE	BIT(24)

struct slim_ctrl_buf {
	void		*base;
	spinlock_t	lock;
	int		head;
	int		tail;
	int		sl_sz;
	int		n;
};

struct slim_tx_completion {
	struct completion done;
	int err;
};

struct qcom_slim_ctrl {
	struct slim_controller  ctrl;
	struct slim_framer	framer;
	struct device		*dev;
	void __iomem		*base;
	void __iomem		*slew_reg;

	struct slim_ctrl_buf	rx;
	struct slim_ctrl_buf	tx;

	struct mutex		msg_lock;
	struct slim_tx_completion **wr_comp;
	struct completion	reconf;
	int			irq;
	struct workqueue_struct *rxwq;
	struct work_struct	wd;
	struct clk		*rclk;
	struct clk		*hclk;
	struct dma_chan		*dma_chans[SLIM_NMGR_PORTS];
	u8			port_ch[SLIM_NMGR_PORTS];
	bool			port_connected[SLIM_NMGR_PORTS];
	bool			port_enabled[SLIM_NMGR_PORTS];
	u8			pgdla;
	bool			pgdla_valid;
};

static void qcom_slim_queue_tx(struct qcom_slim_ctrl *ctrl, void *buf,
			       u8 len, u32 tx_reg)
{
	int count = (len + 3) >> 2;

	__iowrite32_copy(ctrl->base + tx_reg, buf, count);

	/* Ensure ordering of subsequent writes */
	mb();
}

static void *slim_alloc_rxbuf(struct qcom_slim_ctrl *ctrl)
{
	unsigned long flags;
	int idx;

	spin_lock_irqsave(&ctrl->rx.lock, flags);
	if ((ctrl->rx.tail + 1) % ctrl->rx.n == ctrl->rx.head) {
		spin_unlock_irqrestore(&ctrl->rx.lock, flags);
		dev_err(ctrl->dev, "RX QUEUE full!");
		return NULL;
	}
	idx = ctrl->rx.tail;
	ctrl->rx.tail = (ctrl->rx.tail + 1) % ctrl->rx.n;
	spin_unlock_irqrestore(&ctrl->rx.lock, flags);

	return ctrl->rx.base + (idx * ctrl->rx.sl_sz);
}

static void slim_ack_txn(struct qcom_slim_ctrl *ctrl, int err)
{
	struct slim_tx_completion *comp;
	unsigned long flags;
	int idx;

	spin_lock_irqsave(&ctrl->tx.lock, flags);
	idx = ctrl->tx.head;
	comp = ctrl->wr_comp[idx];
	if (!comp) {
		spin_unlock_irqrestore(&ctrl->tx.lock, flags);
		return;
	}
	ctrl->wr_comp[idx] = NULL;
	ctrl->tx.head = (ctrl->tx.head + 1) % ctrl->tx.n;
	comp->err = err;
	/* Serialize completion with timeout cleanup of this stack object. */
	complete(&comp->done);
	spin_unlock_irqrestore(&ctrl->tx.lock, flags);
}

static int qcom_slim_check_clock_rates(struct qcom_slim_ctrl *ctrl)
{
	unsigned long rate = clk_get_rate(ctrl->rclk);

	if (rate != SLIM_ROOT_FREQ)
		return -EINVAL;

	return 0;
}

static void qcom_slim_config_port(struct qcom_slim_ctrl *ctrl, u8 pn)
{
	u32 int_en;

	writel_relaxed(PORT_CFG_VAL, ctrl->base + PGD_PORT(PGD_PORT_CFG_V1, pn));
	writel_relaxed(0, ctrl->base + PGD_PORT(PGD_PORT_BLK_V1, pn));
	writel_relaxed(0, ctrl->base + PGD_PORT(PGD_PORT_TRAN_V1, pn));

	int_en = readl_relaxed(ctrl->base + PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
	writel_relaxed(int_en | BIT(pn),
		       ctrl->base + PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
	/* Make sure port registers are updated before returning */
	mb();
}

static int qcom_slim_connect_port(struct qcom_slim_ctrl *ctrl, u8 pn)
{
	struct dma_slave_config cfg = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		/* BAM interprets maxburst as the descriptor-count threshold. */
		.dst_maxburst = 1,
	};
	struct dma_chan *chan;
	char name[8];
	int ret;

	if (pn >= SLIM_NMGR_PORTS)
		return -EINVAL;
	if (ctrl->port_connected[pn])
		return 0;

	ret = pm_runtime_resume_and_get(ctrl->dev);
	if (ret < 0)
		return ret;
	ret = qcom_slim_check_clock_rates(ctrl);
	if (ret)
		goto put_ctrl;

	snprintf(name, sizeof(name), "tx%u", pn);
	chan = dma_request_chan(ctrl->dev, name);
	if (IS_ERR(chan)) {
		ret = PTR_ERR(chan);
		goto put_ctrl;
	}
	ret = pm_runtime_resume_and_get(chan->device->dev);
	if (ret < 0)
		goto release_chan;
	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		goto put_bam;

	/* The client must prime DMA before enabling the hardware port. */
	ctrl->dma_chans[pn] = chan;
	ctrl->port_connected[pn] = true;
	ctrl->port_enabled[pn] = false;
	return 0;

put_bam:
	pm_runtime_put_autosuspend(chan->device->dev);
release_chan:
	dma_release_channel(chan);
put_ctrl:
	pm_runtime_put_autosuspend(ctrl->dev);
	return ret;
}

int qcom_slim_port_enable_hw(struct slim_controller *sctrl, u8 pn)
{
	struct qcom_slim_ctrl *ctrl = container_of(sctrl, struct qcom_slim_ctrl,
						 ctrl);
	u8 port = SLIM_PORT_START + pn;
	u8 wbuf[2];
	struct slim_val_inf msg = {0, sizeof(wbuf), NULL, wbuf, NULL};
	DEFINE_SLIM_LDEST_TXN(txn, SLIM_MSG_MC_CONNECT_SOURCE, 6, 0, &msg);
	int ret;

	if (pn >= SLIM_NMGR_PORTS)
		return -EINVAL;
	if (!ctrl->port_connected[pn] || !ctrl->dma_chans[pn])
		return -ENOTCONN;
	if (!ctrl->pgdla_valid)
		return -ENODEV;
	if (ctrl->port_enabled[pn])
		return 0;

	qcom_slim_config_port(ctrl, port);
	ctrl->port_enabled[pn] = true;

	/* The PGD accepts CONNECT after its hardware port has been enabled. */
	wbuf[0] = port;
	wbuf[1] = ctrl->port_ch[pn];
	txn.la = ctrl->pgdla;
	ret = slim_do_transfer(sctrl, &txn);
	if (ret)
		qcom_slim_port_disable_hw(sctrl, pn);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_slim_port_enable_hw);

static int qcom_slim_disconnect_port(struct qcom_slim_ctrl *ctrl, u8 pn)
{
	u32 int_en;
	u8 idx = pn - SLIM_PORT_START;
	u8 wbuf[1] = { pn };
	struct slim_val_inf msg = {0, sizeof(wbuf), NULL, wbuf, NULL};
	DEFINE_SLIM_LDEST_TXN(txn, SLIM_MSG_MC_DISCONNECT_PORT, 5, 0, &msg);
	int ret;

	if (idx >= SLIM_NMGR_PORTS)
		return -EINVAL;
	if (!ctrl->dma_chans[idx])
		return 0;

	writel_relaxed(0, ctrl->base + PGD_PORT(PGD_PORT_CFG_V1, pn));

	int_en = readl_relaxed(ctrl->base + PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
	writel_relaxed(int_en & ~BIT(pn),
		       ctrl->base + PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
	/* Complete MMIO updates before subsequent state transitions. */
	mb();

	ctrl->port_connected[idx] = false;
	ctrl->port_enabled[idx] = false;

	/* Disconnect the PGD before releasing its DMA channel. */
	if (ctrl->pgdla_valid) {
		txn.la = ctrl->pgdla;
		ret = slim_do_transfer(&ctrl->ctrl, &txn);
	} else
		ret = -ENODEV;

	if (ctrl->dma_chans[idx]) {
		struct dma_chan *chan = ctrl->dma_chans[idx];

		dmaengine_terminate_sync(chan);
		pm_runtime_put_autosuspend(chan->device->dev);
		dma_release_channel(chan);
		ctrl->dma_chans[idx] = NULL;
	}

	pm_runtime_put_autosuspend(ctrl->dev);
	return ret;
}

int qcom_slim_port_xfer(struct slim_controller *sctrl, u8 pn,
			struct scatterlist *sgl, unsigned int sg_len,
			void (*callback)(void *), void *cookie)
{
	struct qcom_slim_ctrl *ctrl = container_of(sctrl, struct qcom_slim_ctrl,
					     ctrl);
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t tx_cookie;

	if (pn >= SLIM_NMGR_PORTS)
		return -EINVAL;

	chan = ctrl->dma_chans[pn];
	if (!chan)
		return -EINVAL;

	/* Each period needs EOT, completion notification and ordered writes. */
	desc = dmaengine_prep_slave_sg(chan, sgl, sg_len, DMA_MEM_TO_DEV,
				       DMA_PREP_INTERRUPT | DMA_PREP_FENCE);
	if (!desc)
		return -ENOMEM;

	desc->callback = callback;
	desc->callback_param = cookie;

	tx_cookie = dmaengine_submit(desc);
	if (dma_submit_error(tx_cookie))
		return dma_submit_error(tx_cookie);
	dma_async_issue_pending(chan);

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_slim_port_xfer);

void qcom_slim_port_xfer_stop(struct slim_controller *sctrl, u8 pn)
{
	struct qcom_slim_ctrl *ctrl = container_of(sctrl, struct qcom_slim_ctrl,
					     ctrl);
	struct dma_chan *chan;

	if (pn >= SLIM_NMGR_PORTS)
		return;

	chan = ctrl->dma_chans[pn];
	if (!chan)
		return;

	dmaengine_terminate_sync(chan);
}
EXPORT_SYMBOL_GPL(qcom_slim_port_xfer_stop);

/*
 * Disable the manager-side data port in hardware without releasing its BAM
 * channel or runtime-PM references. Stop the port before terminating DMA so
 * that its FIFO cannot underflow while channel removal is in flight. Full
 * cleanup follows when slim_stream_unprepare() sends DISCONNECT_PORT.
 */
void qcom_slim_port_disable_hw(struct slim_controller *sctrl, u8 pn)
{
	struct qcom_slim_ctrl *ctrl = container_of(sctrl, struct qcom_slim_ctrl,
					     ctrl);
	u32 int_en;
	u8 port, idx;

	if (pn >= SLIM_NMGR_PORTS)
		return;

	port = SLIM_PORT_START + pn;
	idx = pn;

	if (!ctrl->port_enabled[idx])
		return;

	writel_relaxed(0, ctrl->base + PGD_PORT(PGD_PORT_CFG_V1, port));
	int_en = readl_relaxed(ctrl->base + PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
	writel_relaxed(int_en & ~BIT(port),
		       ctrl->base + PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
	/* Complete MMIO updates before subsequent state transitions. */
	mb();

	ctrl->port_enabled[idx] = false;
}
EXPORT_SYMBOL_GPL(qcom_slim_port_disable_hw);

static irqreturn_t qcom_slim_handle_port_irq(struct qcom_slim_ctrl *ctrl)
{
	u32 pstat;
	int i;

	pstat = readl_relaxed(ctrl->base + PGD_PORT_EE(PGD_PORT_INT_ST_EE_V1));
	pstat &= readl_relaxed(ctrl->base + PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
	if (!pstat)
		return IRQ_NONE;

	for (i = SLIM_PORT_START; i < SLIM_PORT_START + SLIM_NMGR_PORTS; i++) {
		u32 val;

		if (!(pstat & BIT(i)))
			continue;

		val = readl_relaxed(ctrl->base +
				    PGD_PORT(PGD_PORT_STAT_V1, i));
		if (val & PORT_STAT_DISCONNECT)
			ctrl->port_connected[i - SLIM_PORT_START] = false;
		else if (val & PORT_STAT_OVERFLOW)
			dev_err_ratelimited(ctrl->dev, "port %d overflow\n", i);
		else if (val & PORT_STAT_UNDERFLOW)
			dev_err_ratelimited(ctrl->dev, "port %d underflow\n", i);

		if (val & (PORT_STAT_DISCONNECT | PORT_STAT_OVERFLOW |
			   PORT_STAT_UNDERFLOW)) {
			u32 int_en;

			/* A persistent FIFO fault can immediately reassert the IRQ. */
			int_en = readl_relaxed(ctrl->base +
					      PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
			writel_relaxed(int_en & ~BIT(i), ctrl->base +
				       PGD_PORT_EE(PGD_PORT_INT_EN_EE_V1));
			dev_err(ctrl->dev, "port %d fault: masking IRQ until port enable, stat:0x%x\n",
				i, val);
		}

		/* Match downstream's EE clear strobe, not the status bitmap. */
		writel_relaxed(1, ctrl->base +
			       PGD_PORT_EE(PGD_PORT_INT_CL_EE_V1));
	}
	/* Complete MMIO updates before subsequent state transitions. */
	mb();

	return IRQ_HANDLED;
}

static irqreturn_t qcom_slim_handle_tx_irq(struct qcom_slim_ctrl *ctrl,
					   u32 stat)
{
	int err = 0;

	if (stat & MGR_INT_TX_MSG_SENT)
		writel_relaxed(MGR_INT_TX_MSG_SENT,
			       ctrl->base + MGR_INT_CLR);

	if (stat & MGR_INT_TX_NACKED_2) {
		writel_relaxed(MGR_INT_TX_NACKED_2, ctrl->base + MGR_INT_CLR);
		dev_err_ratelimited(ctrl->dev, "TX message rejected\n");
		err = -ENOTCONN;
	}

	slim_ack_txn(ctrl, err);

	return IRQ_HANDLED;
}

static irqreturn_t qcom_slim_handle_rx_irq(struct qcom_slim_ctrl *ctrl,
					   u32 stat)
{
	u32 *rx_buf, pkt[10];
	bool q_rx = false;
	u8 mc, mt, len;

	pkt[0] = readl_relaxed(ctrl->base + MGR_RX_MSG);
	mt = SLIM_HEADER_GET_MT(pkt[0]);
	len = SLIM_HEADER_GET_RL(pkt[0]);
	mc = SLIM_HEADER_GET_MC(pkt[0]>>8);

	/*
	 * this message cannot be handled by ISR, so
	 * let work-queue handle it
	 */
	if (mt == SLIM_MSG_MT_CORE && mc == SLIM_MSG_MC_REPORT_PRESENT) {
		rx_buf = (u32 *)slim_alloc_rxbuf(ctrl);
		if (!rx_buf)
			goto rx_ret_irq;
		rx_buf[0] = pkt[0];

	} else
		rx_buf = pkt;

	__ioread32_copy(rx_buf + 1, ctrl->base + MGR_RX_MSG + 4,
			DIV_ROUND_UP(len, 4));

	switch (mc) {

	case SLIM_MSG_MC_REPORT_PRESENT:
		q_rx = true;
		break;
	case SLIM_MSG_MC_REPLY_INFORMATION:
	case SLIM_MSG_MC_REPLY_VALUE:
		slim_msg_response(&ctrl->ctrl, (u8 *)(rx_buf + 1),
				  (u8)(*rx_buf >> 24), (len - 4));
		break;
	case SLIM_MSG_MC_REPORT_INFORMATION:
		/* Notifications do not complete a value-element transaction. */
		break;
	default:
		break;
	}
rx_ret_irq:
	writel(MGR_INT_RX_MSG_RCVD, ctrl->base +
		       MGR_INT_CLR);
	if (q_rx)
		queue_work(ctrl->rxwq, &ctrl->wd);

	return IRQ_HANDLED;
}

static irqreturn_t qcom_slim_interrupt(int irq, void *d)
{
	struct qcom_slim_ctrl *ctrl = d;
	u32 stat = readl_relaxed(ctrl->base + MGR_INT_STAT);
	int ret = IRQ_NONE;

	if (stat & (MGR_INT_TX_MSG_SENT | MGR_INT_TX_NACKED_2))
		ret = qcom_slim_handle_tx_irq(ctrl, stat);

	if (stat & MGR_INT_RX_MSG_RCVD)
		ret = qcom_slim_handle_rx_irq(ctrl, stat);

	if (stat & MGR_INT_RECFG_DONE) {
		writel_relaxed(MGR_INT_RECFG_DONE, ctrl->base + MGR_INT_CLR);
		/* Complete MMIO updates before subsequent state transitions. */
		mb();
		complete(&ctrl->reconf);
		ret = IRQ_HANDLED;
	}

	if (ctrl->ctrl.mgr_data_ports &&
	    qcom_slim_handle_port_irq(ctrl) == IRQ_HANDLED)
		ret = IRQ_HANDLED;

	/*
	 * Clear any other pending status bits (e.g. MGR_INT_MSG_BUF_CONTE)
	 * that we don't otherwise handle.  Leaving them pending can cause
	 * level-triggered spurious interrupts and eventually get the IRQ
	 * disabled by the core.
	 */
	if (stat & ~(MGR_INT_TX_MSG_SENT | MGR_INT_TX_NACKED_2 |
		     MGR_INT_RX_MSG_RCVD | MGR_INT_RECFG_DONE)) {
		writel_relaxed(stat, ctrl->base + MGR_INT_CLR);
		/* Complete MMIO updates before subsequent state transitions. */
		mb();
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int qcom_clk_pause_wakeup(struct slim_controller *sctrl)
{
	struct qcom_slim_ctrl *ctrl = dev_get_drvdata(sctrl->dev);
	int ret;

	ret = clk_prepare_enable(ctrl->hclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(ctrl->rclk);
	if (ret)
		goto err_hclk;
	ret = qcom_slim_check_clock_rates(ctrl);
	if (ret)
		goto err_rclk;
	enable_irq(ctrl->irq);

	writel_relaxed(1, ctrl->base + FRM_WAKEUP);
	/* Make sure framer wakeup write goes through before ISR fires */
	mb();
	/*
	 * HW Workaround: Currently, slave is reporting lost-sync messages
	 * after SLIMbus comes out of clock pause.
	 * Transaction with slave fail before slave reports that message
	 * Give some time for that report to come
	 * SLIMbus wakes up in clock gear 10 at 24.576MHz. With each superframe
	 * being 250 usecs, we wait for 5-10 superframes here to ensure
	 * we get the message
	 */
	usleep_range(1250, 2500);
	return 0;

err_rclk:
	clk_disable_unprepare(ctrl->rclk);
err_hclk:
	clk_disable_unprepare(ctrl->hclk);
	return ret;
}

static void *slim_alloc_txbuf(struct qcom_slim_ctrl *ctrl,
			      struct slim_msg_txn *txn,
			      struct slim_tx_completion *done)
{
	unsigned long flags;
	int idx;

	spin_lock_irqsave(&ctrl->tx.lock, flags);
	if (((ctrl->tx.head + 1) % ctrl->tx.n) == ctrl->tx.tail) {
		spin_unlock_irqrestore(&ctrl->tx.lock, flags);
		return NULL;
	}
	idx = ctrl->tx.tail;
	ctrl->wr_comp[idx] = done;
	ctrl->tx.tail = (ctrl->tx.tail + 1) % ctrl->tx.n;

	spin_unlock_irqrestore(&ctrl->tx.lock, flags);

	return ctrl->tx.base + (idx * ctrl->tx.sl_sz);
}

static int qcom_slim_xfer_msg(struct slim_controller *sctrl,
			 struct slim_msg_txn *txn)
{
	struct qcom_slim_ctrl *ctrl = dev_get_drvdata(sctrl->dev);
	struct slim_tx_completion done;
	void *pbuf;
	u8 *puc;
	int ret = 0, retries = QCOM_BUF_ALLOC_RETRIES;
	unsigned long time_left, flags;
	u8 la = txn->la;
	u32 *head;
	int idx;
	bool is_reconf_now = (txn->mt == SLIM_MSG_MT_CORE &&
			      txn->mc == SLIM_MSG_MC_RECONFIGURE_NOW);

	init_completion(&done.done);
	done.err = 0;

	/* HW expects length field to be excluded */
	txn->rl--;

	/*
	 * RECONFIGURE_NOW may not raise MGR_INT_TX_MSG_SENT.  If we queue it
	 * like a normal message, the hardware TX FIFO can remain blocked on the
	 * reconfig slot and all later messages time out.  Send it directly
	 * without consuming a software queue slot, after making sure the queue
	 * is empty.
	 */
	if (is_reconf_now) {
		u32 head_word;
		int wait_retry = 100;

		while (ctrl->tx.head != ctrl->tx.tail) {
			if (!wait_retry--)
				return -EBUSY;
			usleep_range(1000, 2000);
		}

		if (txn->dt == SLIM_MSG_DEST_LOGICALADDR)
			head_word = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt,
							    txn->mc, 0, la);
		else
			head_word = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt,
							    txn->mc, 1, la);

		reinit_completion(&ctrl->reconf);
		qcom_slim_queue_tx(ctrl, &head_word, txn->rl, MGR_TX_MSG);

		time_left = wait_for_completion_timeout(&ctrl->reconf, HZ);
		if (!time_left) {
			dev_err(ctrl->dev, "reconfigure_now timed out\n");
			return -ETIMEDOUT;
		}

		return 0;
	}

	pbuf = slim_alloc_txbuf(ctrl, txn, &done);

	/* spin till buffer is made available */
	if (!pbuf) {
		while (retries--) {
			usleep_range(10000, 15000);
			pbuf = slim_alloc_txbuf(ctrl, txn, &done);
			if (pbuf)
				break;
		}
	}

	if (retries < 0 && !pbuf)
		return -ENOMEM;

	puc = (u8 *)pbuf;
	head = (u32 *)pbuf;

	if (txn->dt == SLIM_MSG_DEST_LOGICALADDR) {
		*head = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt,
						txn->mc, 0, la);
		puc += 3;
	} else {
		*head = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt,
						txn->mc, 1, la);
		puc += 2;
	}

	if (slim_tid_txn(txn->mt, txn->mc))
		*(puc++) = txn->tid;

	if (slim_ec_txn(txn->mt, txn->mc)) {
		*(puc++) = (txn->ec & 0xFF);
		*(puc++) = (txn->ec >> 8) & 0xFF;
	}

	if (txn->msg && txn->msg->wbuf)
		memcpy(puc, txn->msg->wbuf, txn->msg->num_bytes);

	qcom_slim_queue_tx(ctrl, head, txn->rl, MGR_TX_MSG);

	time_left = wait_for_completion_timeout(&done.done,
				msecs_to_jiffies(txn->rl + HZ));
	if (!time_left) {
		dev_err(ctrl->dev, "TX timed out:MC:0x%x,mt:0x%x", txn->mc,
			 txn->mt);

		ret = -ETIMEDOUT;
	} else
		ret = done.err;

	/*
	 * If the TX completion never arrived, the slot is still held by
	 * the aborted message. Free it so the queue does not leak.
	 */
	if (ret == -ETIMEDOUT) {
		spin_lock_irqsave(&ctrl->tx.lock, flags);
		idx = ctrl->tx.head;
		if (ctrl->wr_comp[idx] == &done) {
			ctrl->wr_comp[idx] = NULL;
			ctrl->tx.head = (ctrl->tx.head + 1) % ctrl->tx.n;
		}
		spin_unlock_irqrestore(&ctrl->tx.lock, flags);
	}

	return ret;

}

static int qcom_xfer_msg(struct slim_controller *sctrl,
			 struct slim_msg_txn *txn)
{
	struct qcom_slim_ctrl *ctrl = dev_get_drvdata(sctrl->dev);
	u8 la = txn->la;
	int ret;

	/*
	 * Prepare manager-side DMA resources locally. Defer the PGD CONNECT
	 * until the client has primed DMA and enabled the hardware port.
	 */
	if (sctrl->mgr_data_ports && txn->mt == SLIM_MSG_MT_CORE &&
	    la == SLIM_LA_MANAGER &&
	    (txn->mc == SLIM_MSG_MC_CONNECT_SOURCE ||
	     txn->mc == SLIM_MSG_MC_CONNECT_SINK ||
	     txn->mc == SLIM_MSG_MC_DISCONNECT_PORT) &&
	    txn->msg && txn->msg->wbuf) {
		u8 pn = txn->msg->wbuf[0];
		int err;

		if (pn >= SLIM_NMGR_PORTS)
			return -EINVAL;
		if (txn->mc == SLIM_MSG_MC_DISCONNECT_PORT)
			return qcom_slim_disconnect_port(ctrl, SLIM_PORT_START + pn);
		if (txn->mc != SLIM_MSG_MC_CONNECT_SOURCE)
			return -EOPNOTSUPP;
		if (txn->msg->num_bytes != 2)
			return -EINVAL;

		err = qcom_slim_connect_port(ctrl, pn);
		if (err)
			return err;

		ctrl->port_ch[pn] = txn->msg->wbuf[1];
		return 0;
	}

	mutex_lock(&ctrl->msg_lock);
	ret = qcom_slim_xfer_msg(sctrl, txn);
	mutex_unlock(&ctrl->msg_lock);
	return ret;
}

static int qcom_set_laddr(struct slim_controller *sctrl,
				struct slim_eaddr *ead, u8 laddr)
{
	struct qcom_slim_ctrl *ctrl = dev_get_drvdata(sctrl->dev);
	struct {
		__be16 manf_id;
		__be16 prod_code;
		u8 dev_index;
		u8 instance;
		u8 laddr;
	} __packed p;
	struct slim_val_inf msg = {0};
	DEFINE_SLIM_EDEST_TXN(txn, SLIM_MSG_MC_ASSIGN_LOGICAL_ADDRESS,
			      10, laddr, &msg);
	int ret;

	p.manf_id = cpu_to_be16(ead->manf_id);
	p.prod_code = cpu_to_be16(ead->prod_code);
	p.dev_index = ead->dev_index;
	p.instance = ead->instance;
	p.laddr = laddr;

	msg.wbuf = (void *)&p;
	msg.num_bytes = 7;
	ret = slim_do_transfer(&ctrl->ctrl, &txn);

	if (ret)
		dev_err(ctrl->dev, "set LA:0x%x failed:ret:%d\n",
				  laddr, ret);

	/*
	 * The manager PGD owns the local data ports. Its device index is 5;
	 * other Qualcomm devices and the codec must not receive its CONNECTs.
	 */
	if (!ret && ead->manf_id == 0x217 && ead->prod_code == 0x50 &&
	    ead->dev_index == QCOM_PGD_DEV_INDEX) {
		ctrl->pgdla = laddr;
		ctrl->pgdla_valid = true;
	}

	return ret;
}

static int slim_get_current_rxbuf(struct qcom_slim_ctrl *ctrl, void *buf)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl->rx.lock, flags);
	if (ctrl->rx.tail == ctrl->rx.head) {
		spin_unlock_irqrestore(&ctrl->rx.lock, flags);
		return -ENODATA;
	}
	memcpy(buf, ctrl->rx.base + (ctrl->rx.head * ctrl->rx.sl_sz),
				ctrl->rx.sl_sz);

	ctrl->rx.head = (ctrl->rx.head + 1) % ctrl->rx.n;
	spin_unlock_irqrestore(&ctrl->rx.lock, flags);

	return 0;
}

static void qcom_slim_rxwq(struct work_struct *work)
{
	u8 buf[SLIM_MSGQ_BUF_LEN];
	u8 mc, mt;
	int ret;
	struct qcom_slim_ctrl *ctrl = container_of(work, struct qcom_slim_ctrl,
						 wd);

	while ((slim_get_current_rxbuf(ctrl, buf)) != -ENODATA) {
		mt = SLIM_HEADER_GET_MT(buf[0]);
		mc = SLIM_HEADER_GET_MC(buf[1]);
		if (mt == SLIM_MSG_MT_CORE &&
			mc == SLIM_MSG_MC_REPORT_PRESENT) {
			struct slim_eaddr ea;
			u8 laddr;

			ea.manf_id = be16_to_cpup((__be16 *)&buf[2]);
			ea.prod_code = be16_to_cpup((__be16 *)&buf[4]);
			ea.dev_index = buf[6];
			ea.instance = buf[7];

			ret = slim_device_report_present(&ctrl->ctrl, &ea,
							 &laddr);
			if (ret < 0)
				dev_err(ctrl->dev, "assign laddr failed:%d\n",
					ret);

		} else
			dev_err(ctrl->dev, "unexpected message:mc:%x, mt:%x\n",
				mc, mt);
	}
}

static void qcom_slim_prg_slew(struct platform_device *pdev,
				struct qcom_slim_ctrl *ctrl)
{
	struct resource *res;

	if (ctrl->slew_reg)
		goto program;

	/*
	 * The slew register lives inside the TLMM region, so requesting it as
	 * an exclusive platform resource conflicts with pinctrl.  Map it
	 * directly without claiming the region.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "slew");
	if (!res)
		return;

	ctrl->slew_reg = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!ctrl->slew_reg) {
		dev_err(&pdev->dev, "failed to map slew register\n");
		return;
	}

program:
	writel_relaxed(1, ctrl->slew_reg);
	/* Make sure SLIMbus-slew rate enabling goes through */
	wmb();
}

static int qcom_slim_probe(struct platform_device *pdev)
{
	struct qcom_slim_ctrl *ctrl;
	struct slim_controller *sctrl;
	int ret, ver;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	ctrl->dev = &pdev->dev;

	ctrl->hclk = devm_clk_get(&pdev->dev, "iface");
	if (IS_ERR(ctrl->hclk))
		return PTR_ERR(ctrl->hclk);

	ctrl->rclk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(ctrl->rclk))
		return PTR_ERR(ctrl->rclk);

	ret = clk_set_rate(ctrl->rclk, SLIM_ROOT_FREQ);
	if (ret) {
		dev_err(&pdev->dev, "ref-clock set-rate failed:%d\n", ret);
		return ret;
	}
	ret = qcom_slim_check_clock_rates(ctrl);
	if (ret)
		return ret;

	ctrl->irq = platform_get_irq(pdev, 0);
	if (ctrl->irq < 0)
		return ctrl->irq;

	sctrl = &ctrl->ctrl;
	sctrl->dev = &pdev->dev;
	platform_set_drvdata(pdev, ctrl);
	dev_set_drvdata(ctrl->dev, ctrl);

	ctrl->base = devm_platform_ioremap_resource_byname(pdev, "ctrl");
	if (IS_ERR(ctrl->base))
		return PTR_ERR(ctrl->base);

	sctrl->set_laddr = qcom_set_laddr;
	sctrl->xfer_msg = qcom_xfer_msg;
	sctrl->wakeup =  qcom_clk_pause_wakeup;
	sctrl->mgr_data_ports = of_device_is_compatible(pdev->dev.of_node,
						      "qcom,apq8064-slim");
	ctrl->tx.n = QCOM_TX_MSGS;
	ctrl->tx.sl_sz = SLIM_MSGQ_BUF_LEN;
	ctrl->rx.n = QCOM_RX_MSGS;
	ctrl->rx.sl_sz = SLIM_MSGQ_BUF_LEN;
	ctrl->wr_comp = devm_kcalloc(&pdev->dev, QCOM_TX_MSGS,
				    sizeof(*ctrl->wr_comp), GFP_KERNEL);
	if (!ctrl->wr_comp)
		return -ENOMEM;

	init_completion(&ctrl->reconf);

	mutex_init(&ctrl->msg_lock);
	spin_lock_init(&ctrl->rx.lock);
	spin_lock_init(&ctrl->tx.lock);
	INIT_WORK(&ctrl->wd, qcom_slim_rxwq);
	ctrl->rxwq = create_singlethread_workqueue("qcom_slim_rx");
	if (!ctrl->rxwq) {
		dev_err(ctrl->dev, "Failed to start Rx WQ\n");
		return -ENOMEM;
	}

	ctrl->framer.rootfreq = SLIM_ROOT_FREQ / 8;
	ctrl->framer.superfreq =
		ctrl->framer.rootfreq / SLIM_CL_PER_SUPERFRAME_DIV8;
	sctrl->a_framer = &ctrl->framer;
	sctrl->sched.msgsl = 30;
	sctrl->clkgear = SLIM_MAX_CLK_GEAR;

	qcom_slim_prg_slew(pdev, ctrl);

	ret = devm_request_irq(&pdev->dev, ctrl->irq, qcom_slim_interrupt,
				IRQF_TRIGGER_HIGH, "qcom_slim_irq", ctrl);
	if (ret) {
		dev_err(&pdev->dev, "request IRQ failed\n");
		goto err_request_irq_failed;
	}

	ret = clk_prepare_enable(ctrl->hclk);
	if (ret)
		goto err_hclk_enable_failed;

	ret = clk_prepare_enable(ctrl->rclk);
	if (ret)
		goto err_rclk_enable_failed;
	ret = qcom_slim_check_clock_rates(ctrl);
	if (ret)
		goto err;

	ctrl->tx.base = devm_kcalloc(&pdev->dev, ctrl->tx.n, ctrl->tx.sl_sz,
				     GFP_KERNEL);
	if (!ctrl->tx.base) {
		ret = -ENOMEM;
		goto err;
	}

	ctrl->rx.base = devm_kcalloc(&pdev->dev, ctrl->rx.n, ctrl->rx.sl_sz,
				     GFP_KERNEL);
	if (!ctrl->rx.base) {
		ret = -ENOMEM;
		goto err;
	}

	/* Register with framework before enabling frame, clock */
	ret = slim_register_controller(&ctrl->ctrl);
	if (ret) {
		dev_err(ctrl->dev, "error adding controller\n");
		goto err;
	}

	ver = readl_relaxed(ctrl->base);
	/* Version info in 16 MSbits */
	ver >>= 16;
	/* Component register initialization */
	writel(1, ctrl->base + CFG_PORT(COMP_CFG, ver));
	writel((EE_MGR_RSC_GRP | EE_NGD_2 | EE_NGD_1),
				ctrl->base + CFG_PORT(COMP_TRUST_CFG, ver));

	writel((MGR_INT_TX_NACKED_2 | MGR_INT_RECFG_DONE |
		MGR_INT_MSG_BUF_CONTE | MGR_INT_RX_MSG_RCVD |
		MGR_INT_TX_MSG_SENT), ctrl->base + MGR_INT_EN);
	writel(1, ctrl->base + MGR_CFG);
	/* Framer register initialization */
	writel((1 << INTR_WAKE) | (0xA << REF_CLK_GEAR) |
		(0xA << CLK_GEAR) | (1 << ROOT_FREQ) | (1 << FRM_ACTIVE) | 1,
		ctrl->base + FRM_CFG);
	writel(MGR_CFG_ENABLE, ctrl->base + MGR_CFG);
	writel(1, ctrl->base + INTF_CFG);

	/*
	 * Enable the ported generic device (PGD) inside the manager and assign
	 * the manager data ports to the Apps EE.  Without this the manager-side
	 * data ports are not connected to the SLIMbus framer even though the
	 * BAM descriptor is submitted, causing playback descriptors to stall and
	 * later control transactions to time out.
	 */
	if (sctrl->mgr_data_ports) {
		writel(1, ctrl->base + CFG_PORT(PGD_CFG, ver));
		writel(GENMASK(SLIM_PORT_START + SLIM_NMGR_PORTS - 1,
			       SLIM_PORT_START),
		       ctrl->base + CFG_PORT(PGD_OWN_EE, ver) + 4 * PGD_EE);
		/* Complete MMIO updates before subsequent state transitions. */
		mb();
	}

	writel(1, ctrl->base + CFG_PORT(COMP_CFG, ver));

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, QCOM_SLIM_AUTOSUSPEND);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;

err:
	clk_disable_unprepare(ctrl->rclk);
err_rclk_enable_failed:
	clk_disable_unprepare(ctrl->hclk);
err_hclk_enable_failed:
err_request_irq_failed:
	destroy_workqueue(ctrl->rxwq);
	return ret;
}

static void qcom_slim_remove(struct platform_device *pdev)
{
	struct qcom_slim_ctrl *ctrl = platform_get_drvdata(pdev);

	int i;

	pm_runtime_resume_and_get(&pdev->dev);
	slim_unregister_controller(&ctrl->ctrl);
	for (i = 0; i < SLIM_NMGR_PORTS; i++) {
		if (ctrl->dma_chans[i])
			qcom_slim_disconnect_port(ctrl, SLIM_PORT_START + i);
	}
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(ctrl->rclk);
	clk_disable_unprepare(ctrl->hclk);
	destroy_workqueue(ctrl->rxwq);
}

/*
 * If PM_RUNTIME is not defined, these 2 functions become helper
 * functions to be called from system suspend/resume.
 */
#ifdef CONFIG_PM
static int qcom_slim_runtime_suspend(struct device *device)
{
	struct qcom_slim_ctrl *ctrl = dev_get_drvdata(device);
	int ret;

	ret = slim_ctrl_clk_pause(&ctrl->ctrl, false, SLIM_CLK_UNSPECIFIED);
	if (ret) {
	} else {
		disable_irq(ctrl->irq);
		clk_disable_unprepare(ctrl->hclk);
		clk_disable_unprepare(ctrl->rclk);
	}
	return ret;
}

static int qcom_slim_runtime_resume(struct device *device)
{
	struct qcom_slim_ctrl *ctrl = dev_get_drvdata(device);
	int ret = 0;

	ret = slim_ctrl_clk_pause(&ctrl->ctrl, true, 0);

	return ret;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int qcom_slim_suspend(struct device *dev)
{
	int ret = 0;

	if (!pm_runtime_enabled(dev) ||
		(!pm_runtime_suspended(dev)))
		ret = qcom_slim_runtime_suspend(dev);

	return ret;
}

static int qcom_slim_resume(struct device *dev)
{
	if (!pm_runtime_enabled(dev) || !pm_runtime_suspended(dev)) {
		int ret;

		ret = qcom_slim_runtime_resume(dev);
		if (!ret) {
			pm_runtime_mark_last_busy(dev);
			pm_request_autosuspend(dev);
		}
		return ret;

	}
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops qcom_slim_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(qcom_slim_suspend, qcom_slim_resume)
	SET_RUNTIME_PM_OPS(
			   qcom_slim_runtime_suspend,
			   qcom_slim_runtime_resume,
			   NULL
	)
};

static const struct of_device_id qcom_slim_dt_match[] = {
	{ .compatible = "qcom,apq8064-slim", },
	{ .compatible = "qcom,slim", },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_slim_dt_match);

static struct platform_driver qcom_slim_driver = {
	.probe = qcom_slim_probe,
	.remove = qcom_slim_remove,
	.driver	= {
		.name = "qcom_slim_ctrl",
		.of_match_table = qcom_slim_dt_match,
		.pm = &qcom_slim_dev_pm_ops,
	},
};
module_platform_driver(qcom_slim_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm SLIMbus Controller");
