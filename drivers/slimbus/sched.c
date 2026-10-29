// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2011-2017, The Linux Foundation
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/slimbus.h>
#include "slimbus.h"

/**
 * slim_ctrl_clk_pause() - Called by slimbus controller to enter/exit
 *			   'clock pause'
 * @ctrl: controller requesting bus to be paused or woken up
 * @wakeup: Wakeup this controller from clock pause.
 * @restart: Restart time value per spec used for clock pause. This value
 *    isn't used when controller is to be woken up.
 *
 * Slimbus specification needs this sequence to turn-off clocks for the bus.
 * The sequence involves sending 3 broadcast messages (reconfiguration
 * sequence) to inform all devices on the bus.
 * To exit clock-pause, controller typically wakes up active framer device.
 * This API executes clock pause reconfiguration sequence if wakeup is false.
 * If wakeup is true, controller's wakeup is called.
 * For entering clock-pause, -EBUSY is returned if a message txn in pending.
 */
int slim_ctrl_clk_pause(struct slim_controller *ctrl, bool wakeup, u8 restart)
{
	int i, ret = 0;
	unsigned long flags;
	struct slim_sched *sched = &ctrl->sched;
	struct slim_val_inf msg = {0, 0, NULL, NULL};

	DEFINE_SLIM_BCAST_TXN(txn, SLIM_MSG_MC_BEGIN_RECONFIGURATION,
				3, SLIM_LA_MANAGER, &msg);

	if (wakeup == false && restart > SLIM_CLK_UNSPECIFIED)
		return -EINVAL;

	mutex_lock(&sched->m_reconf);
	if (wakeup) {
		if (sched->clk_state == SLIM_CLK_ACTIVE) {
			mutex_unlock(&sched->m_reconf);
			return 0;
		}

		/*
		 * Fine-tune calculation based on clock gear,
		 * message-bandwidth after bandwidth management
		 */
		ret = wait_for_completion_timeout(&sched->pause_comp,
				msecs_to_jiffies(100));
		if (!ret) {
			mutex_unlock(&sched->m_reconf);
			return -ETIMEDOUT;
		}
		ret = 0;

		/*
		 * Slimbus framework will call controller wakeup
		 * Controller should make sure that it sets active framer
		 * out of clock pause
		 */
		if (sched->clk_state == SLIM_CLK_PAUSED && ctrl->wakeup)
			ret = ctrl->wakeup(ctrl);
		if (!ret)
			sched->clk_state = SLIM_CLK_ACTIVE;
		mutex_unlock(&sched->m_reconf);

		return ret;
	}

	/* already paused */
	if (ctrl->sched.clk_state == SLIM_CLK_PAUSED) {
		mutex_unlock(&sched->m_reconf);
		return 0;
	}

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	for (i = 0; i < SLIM_MAX_TIDS; i++) {
		/* Pending response for a message */
		if (idr_find(&ctrl->tid_idr, i)) {
			spin_unlock_irqrestore(&ctrl->txn_lock, flags);
			mutex_unlock(&sched->m_reconf);
			return -EBUSY;
		}
	}
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);

	sched->clk_state = SLIM_CLK_ENTERING_PAUSE;

	/* clock pause sequence */
	ret = slim_do_transfer(ctrl, &txn);
	if (ret)
		goto clk_pause_ret;

	txn.mc = SLIM_MSG_MC_NEXT_PAUSE_CLOCK;
	txn.rl = 4;
	msg.num_bytes = 1;
	msg.wbuf = &restart;
	ret = slim_do_transfer(ctrl, &txn);
	if (ret)
		goto clk_pause_ret;

	txn.mc = SLIM_MSG_MC_RECONFIGURE_NOW;
	txn.rl = 3;
	msg.num_bytes = 1;
	msg.wbuf = NULL;
	ret = slim_do_transfer(ctrl, &txn);

clk_pause_ret:
	if (ret)
		sched->clk_state = SLIM_CLK_ACTIVE;
	else {
		sched->clk_state = SLIM_CLK_PAUSED;
		complete(&sched->pause_comp);
	}
	mutex_unlock(&sched->m_reconf);

	return ret;
}
EXPORT_SYMBOL_GPL(slim_ctrl_clk_pause);

/*
 * Compute the Presence Rate Control Field (PRCF) for the NEXT_DEFINE_CONTENT
 * message.  For Isochronous channels this encodes the rate family, coefficient
 * and exponent per the SLIMbus spec.  For Pushed/Pulled channels the field is
 * not used and should be zero.
 */
static u8 slim_calc_prrate(int ratem, enum slim_transport_protocol prot,
			   int superfreq)
{
	int rate = ratem;
	int exp = 0;
	u8 pr = 0;

	if (prot != SLIM_PROTO_ISO)
		return 0;

	/*
	 * APQ8064 runs the bus at a 4 kHz superframe rate.  The 11.025 kHz
	 * family is not used here.
	 */
	if (superfreq != 4000)
		return 0;

	/* Decompose ratem into coefficient * 2^exp, coefficient is 1 or 3. */
	while (!(rate & 1)) {
		rate >>= 1;
		exp++;
	}

	if (rate > 3) {
		/*
		 * Not a valid hard-ISO rate.  Return 0 so the content message
		 * does not advertise a broken rate; the channel setup will fail
		 * later if the caller really needs ISO.
		 */
		return 0;
	}

	if (rate == 1)
		pr = 0x10;
	else
		exp++;

	pr |= exp;
	pr |= 0x80; /* exact rate */

	return pr;
}

int slim_nextdefine_ch(struct slim_stream_runtime *stream, struct slim_port *port)
{
	struct slim_controller *ctrl = stream->dev->ctrl;
	struct slim_channel *slc = &port->ch;
	u32 chrate;
	u32 exp = 0;
	u32 coeff = 0;
	bool exact = true;
	bool done = false;
	int ret = 0;

	slc->prrate = slim_calc_prrate(stream->ratem, stream->prot,
				       ctrl->a_framer->superfreq);

	chrate = stream->rate;
	/* max allowed sample freq = 768 seg/frame */
	if (chrate > 3600000)
		return -EDQUOT;

	if (stream->prot == SLIM_PROTO_ISO &&
	    ctrl->a_framer->superfreq == 4000 &&
	    (chrate % 4000) == 0)
		coeff = stream->ratem;
	else if (stream->prot == SLIM_PROTO_ISO &&
		 ctrl->a_framer->superfreq == 3675 &&
		 (chrate % 11025) == 0)
		coeff = 3 * stream->ratem;
	else {
		u32 tempr = chrate * SLIM_CL_PER_SUPERFRAME_DIV8;

		coeff = tempr / ctrl->a_framer->rootfreq;
		if (coeff * ctrl->a_framer->rootfreq != tempr) {
			coeff++;
			exact = false;
		}
	}

	/* convert coeff to coeff-exponent */
	exp = 0;
	while (!done) {
		while ((coeff & 0x1) != 0x1) {
			coeff >>= 1;
			exp++;
		}
		if (coeff > 3) {
			coeff++;
			exact = false;
		} else
			done = true;
	}

	if (stream->prot == SLIM_PROTO_ISO && !exact)
		return -EPROTONOSUPPORT;

	slc->rootexp = exp;
	slc->seglen = stream->bps / SLIM_SLOT_LEN_BITS;
	if (stream->prot != SLIM_PROTO_ISO)
		slc->seglen++;
	if (stream->prot >= SLIM_PROTO_EXT_SMPLX)
		slc->seglen++;

	/* convert coeff to enum */
	if (coeff == 1) {
		if (exp > 9)
			ret = -EIO;
		slc->coeff = SLIM_COEFF_1;
	} else {
		if (exp > 8)
			ret = -EIO;
		slc->coeff = SLIM_COEFF_3;
	}

	slc->prot = stream->prot;
	return ret;
}

static void slim_add_ch(struct slim_controller *ctrl, struct slim_channel *slc)
{
	struct slim_channel **arr;
	int i, j;
	int *len;
	u32 sl = slc->seglen << slc->rootexp;

	/* Channel is already active and other end is transmitting data */
	if (slc->state >= SLIM_CH_STATE_ACTIVE)
		return;

	if (slc->coeff == SLIM_COEFF_1) {
		arr = ctrl->sched.chc1;
		len = &ctrl->sched.num_cc1;
	} else {
		arr = ctrl->sched.chc3;
		len = &ctrl->sched.num_cc3;
		sl *= 3;
	}

	*len += 1;

	/* Insert the channel based on rootexp and seglen */
	for (i = 0; i < *len - 1; i++) {
		/*
		 * Primary key: exp low to high.
		 * Secondary key: seglen: high to low
		 */
		if ((slc->rootexp > arr[i]->rootexp) ||
		    ((slc->rootexp == arr[i]->rootexp) &&
		     (slc->seglen < arr[i]->seglen)))
			continue;
		else
			break;
	}
	for (j = *len - 1; j > i; j--)
		arr[j] = arr[j - 1];
	arr[i] = slc;
	ctrl->sched.usedslots += sl;
}

static int slim_remove_ch(struct slim_controller *ctrl, struct slim_channel *slc)
{
	struct slim_channel **arr;
	int i;
	int *len;

	if (slc->coeff == SLIM_COEFF_1) {
		arr = ctrl->sched.chc1;
		len = &ctrl->sched.num_cc1;
	} else {
		arr = ctrl->sched.chc3;
		len = &ctrl->sched.num_cc3;
	}

	for (i = 0; i < *len; i++) {
		if (arr[i] == slc)
			break;
	}
	if (i >= *len)
		return -EXFULL;
	for (; i < *len - 1; i++)
		arr[i] = arr[i + 1];
	*len -= 1;
	arr[*len] = NULL;

	slc->state = SLIM_CH_STATE_ALLOCATED;
	slc->newintr = 0;
	slc->newoff = 0;
	slc->def = 0;
	return 0;
}

/* Drop every scheduler reference before stream->ports is freed. */
void slim_sched_unprepare(struct slim_stream_runtime *stream)
{
	struct slim_controller *ctrl = stream->dev->ctrl;
	int i;

	mutex_lock(&ctrl->sched.m_reconf);
	mutex_lock(&ctrl->lock);
	for (i = 0; i < stream->num_ports; i++) {
		struct slim_channel *slc = &stream->ports[i].ch;
		u32 slots = slc->seglen << slc->rootexp;

		if (slc->coeff == SLIM_COEFF_3)
			slots *= 3;
		/* Failed removal rolls back to ACTIVE and retains this pointer. */
		if (!slim_remove_ch(ctrl, slc))
			ctrl->sched.usedslots -= slots;
		if (ctrl->sched.ch_map[slc->id] == slc)
			ctrl->sched.ch_map[slc->id] = NULL;
	}
	mutex_unlock(&ctrl->lock);
	mutex_unlock(&ctrl->sched.m_reconf);
}

static u32 getsubfrmcoding(u32 *ctrlw, u32 *subfrml, u32 *msgsl)
{
	u32 code = 0;

	if (*ctrlw == *subfrml) {
		*ctrlw = 8;
		*subfrml = 8;
		*msgsl = SLIM_SL_PER_SUPERFRAME - SLIM_FRM_SLOTS_PER_SUPERFRAME
			- SLIM_GDE_SLOTS_PER_SUPERFRAME;
		return 0;
	}
	if (*subfrml == 6) {
		code = 0;
		*msgsl = 256;
	} else if (*subfrml == 8) {
		code = 1;
		*msgsl = 192;
	} else if (*subfrml == 24) {
		code = 2;
		*msgsl = 64;
	} else { /* 32 */
		code = 3;
		*msgsl = 48;
	}

	if (*ctrlw < 8) {
		if (*ctrlw >= 6) {
			*ctrlw = 6;
			code |= 0x14;
		} else {
			if (*ctrlw == 5)
				*ctrlw = 4;
			code |= (*ctrlw << 2);
		}
	} else {
		code -= 2;
		if (*ctrlw >= 24) {
			*ctrlw = 24;
			code |= 0x1e;
		} else if (*ctrlw >= 16) {
			*ctrlw = 16;
			code |= 0x1c;
		} else if (*ctrlw >= 12) {
			*ctrlw = 12;
			code |= 0x1a;
		} else {
			*ctrlw = 8;
			code |= 0x18;
		}
	}

	*msgsl = (*msgsl * *ctrlw) - SLIM_FRM_SLOTS_PER_SUPERFRAME -
		SLIM_GDE_SLOTS_PER_SUPERFRAME;
	return code;
}

static void shiftsegoffsets(struct slim_controller *ctrl,
			    struct slim_channel **ach, int sz, u32 shft)
{
	int i;

	for (i = 0; i < sz; i++) {
		struct slim_channel *slc;

		if (ach[i] == NULL)
			continue;
		slc = ach[i];
		if (slc->state == SLIM_CH_STATE_PENDING_REMOVAL)
			continue;
		slc->newoff += shft;
		/* seg. offset must be <= interval */
		if (slc->newoff >= slc->newintr)
			slc->newoff -= slc->newintr;
	}
}

static int slim_update_open_slots(int *open, int *open3, int bucket,
				  int max_slots)
{
	if (open[bucket] >= 0) {
		open3[bucket & 1] = min(open3[bucket & 1], open[bucket]);
		return 0;
	}
	if (!(bucket & 1) || open[bucket - 1] != max_slots)
		return -EXFULL;

	open[bucket - 1] += open[bucket];
	open3[0] = min(open3[0], open[bucket - 1]);
	open3[1] = 0;
	open[bucket] = 0;
	return open[bucket - 1] < 0 ? -EXFULL : 0;
}

static int slim_sched_chans(struct slim_device *sb, u32 clkgear,
			    u32 *ctrlw, u32 *subfrml)
{
	int coeff1, coeff3;
	enum slim_ch_coeff bias;
	struct slim_controller *ctrl = sb->ctrl;
	int last1 = ctrl->sched.num_cc1 - 1;
	int last3 = ctrl->sched.num_cc3 - 1;

	/*
	 * Find first channels with coeff 1 & 3 as starting points for
	 * scheduling
	 */
	for (coeff3 = 0; coeff3 < ctrl->sched.num_cc3; coeff3++) {
		struct slim_channel *slc = ctrl->sched.chc3[coeff3];

		if (slc->state == SLIM_CH_STATE_PENDING_REMOVAL)
			continue;
		else
			break;
	}
	for (coeff1 = 0; coeff1 < ctrl->sched.num_cc1; coeff1++) {
		struct slim_channel *slc = ctrl->sched.chc1[coeff1];

		if (slc->state == SLIM_CH_STATE_PENDING_REMOVAL)
			continue;
		else
			break;
	}
	if (coeff3 == ctrl->sched.num_cc3 && coeff1 == ctrl->sched.num_cc1) {
		*ctrlw = 8;
		*subfrml = 8;
		return 0;
	} else if (coeff3 == ctrl->sched.num_cc3)
		bias = SLIM_COEFF_1;
	else
		bias = SLIM_COEFF_3;

	/*
	 * Find last chan in coeff1, 3 list, we will use to know when we
	 * have done scheduling all coeff1 channels
	 */
	while (last1 >= 0) {
		if (ctrl->sched.chc1[last1] != NULL &&
		    (ctrl->sched.chc1[last1])->state !=
		    SLIM_CH_STATE_PENDING_REMOVAL)
			break;
		last1--;
	}
	while (last3 >= 0) {
		if (ctrl->sched.chc3[last3] != NULL &&
		    (ctrl->sched.chc3[last3])->state !=
		    SLIM_CH_STATE_PENDING_REMOVAL)
			break;
		last3--;
	}

	if (bias == SLIM_COEFF_1) {
		struct slim_channel *slc1 = coeff1 < ctrl->sched.num_cc1 ?
			ctrl->sched.chc1[coeff1] : NULL;
		u32 expshft = SLIM_MAX_CLK_GEAR - clkgear;
		int curexp, finalexp;
		u32 curintr, curmaxsl;
		int opensl1[2];
		int maxctrlw1;

		finalexp = (ctrl->sched.chc1[last1])->rootexp;
		curexp = (int)expshft - 1;

		curintr = (SLIM_MAX_INTR_COEFF_1 * 2) >> (curexp + 1);
		curmaxsl = curintr >> 1;
		opensl1[0] = opensl1[1] = curmaxsl;

		while ((coeff1 < ctrl->sched.num_cc1) || (curintr > 24)) {
			curintr >>= 1;
			curmaxsl >>= 1;

			/* update 4K family open slot records */
			if (opensl1[1] < opensl1[0])
				opensl1[1] -= curmaxsl;
			else
				opensl1[1] = opensl1[0] - curmaxsl;
			opensl1[0] = curmaxsl;
			if (opensl1[1] < 0) {
				opensl1[0] += opensl1[1];
				opensl1[1] = 0;
			}
			if (opensl1[0] <= 0)
				return -EXFULL;
			curexp++;
			/* schedule 4k family channels */
			while ((coeff1 < ctrl->sched.num_cc1) &&
			       (curexp == (int)(slc1->rootexp + expshft))) {
				if (slc1->state ==
				    SLIM_CH_STATE_PENDING_REMOVAL) {
					coeff1++;
					slc1 = coeff1 < ctrl->sched.num_cc1 ?
						ctrl->sched.chc1[coeff1] : NULL;
					continue;
				}
				if (opensl1[1] >= opensl1[0] ||
				    (finalexp == (int)slc1->rootexp &&
				     curintr <= 24 &&
				     opensl1[0] == curmaxsl)) {
					opensl1[1] -= slc1->seglen;
					slc1->newoff = curmaxsl + opensl1[1];
					if (opensl1[1] < 0 &&
					    opensl1[0] == curmaxsl) {
						opensl1[0] += opensl1[1];
						opensl1[1] = 0;
					}
					if (opensl1[0] < 0)
						return -EXFULL;
				} else {
					if (slc1->seglen > opensl1[0])
						return -EXFULL;
					slc1->newoff = opensl1[0] - slc1->seglen;
					opensl1[0] = slc1->newoff;
				}
				slc1->newintr = curintr;
				coeff1++;
				slc1 = coeff1 < ctrl->sched.num_cc1 ?
					ctrl->sched.chc1[coeff1] : NULL;
			}
		}
		/* Leave some slots for messaging space */
		if (opensl1[1] <= 0 && opensl1[0] <= 0)
			return -EXFULL;
		if (opensl1[1] > opensl1[0]) {
			int temp = opensl1[0];

			opensl1[0] = opensl1[1];
			opensl1[1] = temp;
			shiftsegoffsets(ctrl, ctrl->sched.chc1,
					ctrl->sched.num_cc1, curmaxsl);
		}
		/* choose subframe mode to maximize bw */
		maxctrlw1 = opensl1[0];
		if (opensl1[0] == curmaxsl)
			maxctrlw1 += opensl1[1];
		if (curintr >= 24) {
			*subfrml = 24;
			*ctrlw = maxctrlw1;
		} else if (curintr == 12) {
			if (maxctrlw1 > opensl1[1] * 4) {
				*subfrml = 24;
				*ctrlw = maxctrlw1;
			} else {
				*subfrml = 6;
				*ctrlw = opensl1[1];
			}
		} else {
			*subfrml = 6;
			*ctrlw = maxctrlw1;
		}
	} else {
		struct slim_channel *slc1 = NULL;
		struct slim_channel *slc3 = coeff3 < ctrl->sched.num_cc3 ?
			ctrl->sched.chc3[coeff3] : NULL;
		u32 expshft = SLIM_MAX_CLK_GEAR - clkgear;
		int curexp, finalexp, exp1;
		u32 curintr, curmaxsl;
		int opensl3[2];
		int opensl1[6];
		bool opensl1valid = false;
		int maxctrlw1, maxctrlw3, i;

		finalexp = (ctrl->sched.chc3[last3])->rootexp;
		if (last1 >= 0) {
			slc1 = coeff1 < ctrl->sched.num_cc1 ?
				ctrl->sched.chc1[coeff1] : NULL;
			exp1 = (ctrl->sched.chc1[last1])->rootexp;
			if (exp1 > finalexp)
				finalexp = exp1;
		}
		curexp = (int)expshft - 1;

		curintr = (SLIM_MAX_INTR_COEFF_3 * 2) >> (curexp + 1);
		curmaxsl = curintr >> 1;
		opensl3[0] = opensl3[1] = curmaxsl;

		while (coeff1 < ctrl->sched.num_cc1 ||
		       coeff3 < ctrl->sched.num_cc3 ||
		       curintr > 32) {
			curintr >>= 1;
			curmaxsl >>= 1;

			/* update 12k family open slot records */
			if (opensl3[1] < opensl3[0])
				opensl3[1] -= curmaxsl;
			else
				opensl3[1] = opensl3[0] - curmaxsl;
			opensl3[0] = curmaxsl;
			if (opensl3[1] < 0) {
				opensl3[0] += opensl3[1];
				opensl3[1] = 0;
			}
			if (opensl3[0] <= 0)
				return -EXFULL;
			curexp++;

			/* schedule 12k family channels */
			while (coeff3 < ctrl->sched.num_cc3 &&
			       curexp == (int)slc3->rootexp + expshft) {
				if (slc3->state ==
				    SLIM_CH_STATE_PENDING_REMOVAL) {
					coeff3++;
					slc3 = coeff3 < ctrl->sched.num_cc3 ?
						ctrl->sched.chc3[coeff3] : NULL;
					continue;
				}
				opensl1valid = false;
				if (opensl3[1] >= opensl3[0] ||
				    (finalexp == (int)slc3->rootexp &&
				     curintr <= 32 &&
				     opensl3[0] == curmaxsl &&
				     last1 < 0)) {
					opensl3[1] -= slc3->seglen;
					slc3->newoff = curmaxsl + opensl3[1];
					if (opensl3[1] < 0 &&
					    opensl3[0] == curmaxsl) {
						opensl3[0] += opensl3[1];
						opensl3[1] = 0;
					}
					if (opensl3[0] < 0)
						return -EXFULL;
				} else {
					if (slc3->seglen > opensl3[0])
						return -EXFULL;
					slc3->newoff = opensl3[0] - slc3->seglen;
					opensl3[0] = slc3->newoff;
				}
				slc3->newintr = curintr;
				coeff3++;
				slc3 = coeff3 < ctrl->sched.num_cc3 ?
					ctrl->sched.chc3[coeff3] : NULL;
			}
			/* update 4k openslot records */
			if (opensl1valid == false) {
				for (i = 0; i < 3; i++) {
					opensl1[i * 2] = opensl3[0];
					opensl1[(i * 2) + 1] = opensl3[1];
				}
			} else {
				int opensl1p[6];

				memcpy(opensl1p, opensl1, sizeof(opensl1));
				for (i = 0; i < 3; i++) {
					if (opensl1p[i] < opensl1p[i + 3])
						opensl1[(i * 2) + 1] =
							opensl1p[i];
					else
						opensl1[(i * 2) + 1] =
							opensl1p[i + 3];
				}
				for (i = 0; i < 3; i++) {
					opensl1[(i * 2) + 1] -= curmaxsl;
					opensl1[i * 2] = curmaxsl;
					if (opensl1[(i * 2) + 1] < 0) {
						opensl1[i * 2] +=
							opensl1[(i * 2) + 1];
						opensl1[(i * 2) + 1] = 0;
					}
					if (opensl1[i * 2] < 0)
						return -EXFULL;
				}
			}
			/* schedule 4k family channels */
			while (coeff1 < ctrl->sched.num_cc1 &&
			       curexp == (int)slc1->rootexp + expshft) {
				/* searchorder effective when opensl valid */
				static const int srcho[] = { 5, 2, 4, 1, 3, 0 };
				int maxopensl = 0;
				int maxi = 0;

				if (slc1->state ==
				    SLIM_CH_STATE_PENDING_REMOVAL) {
					coeff1++;
					slc1 = coeff1 < ctrl->sched.num_cc1 ?
						ctrl->sched.chc1[coeff1] : NULL;
					continue;
				}
				opensl1valid = true;
				for (i = 0; i < 6; i++) {
					if (opensl1[srcho[i]] > maxopensl) {
						maxopensl = opensl1[srcho[i]];
						maxi = srcho[i];
					}
				}
				opensl1[maxi] -= slc1->seglen;
				slc1->newoff = (curmaxsl * maxi) +
					opensl1[maxi];
				if (slim_update_open_slots(opensl1, opensl3, maxi,
							   curmaxsl))
					return -EXFULL;
				slc1->newintr = curintr * 3;
				coeff1++;
				slc1 = coeff1 < ctrl->sched.num_cc1 ?
					ctrl->sched.chc1[coeff1] : NULL;
			}
		}
		/* Leave some slots for messaging space */
		if (opensl3[1] <= 0 && opensl3[0] <= 0)
			return -EXFULL;
		/* swap 1st and 2nd bucket if 2nd bucket has more open slots */
		if (opensl3[1] > opensl3[0]) {
			int temp = opensl3[0];

			opensl3[0] = opensl3[1];
			opensl3[1] = temp;
			temp = opensl1[5];
			opensl1[5] = opensl1[4];
			opensl1[4] = opensl1[3];
			opensl1[3] = opensl1[2];
			opensl1[2] = opensl1[1];
			opensl1[1] = opensl1[0];
			opensl1[0] = temp;
			shiftsegoffsets(ctrl, ctrl->sched.chc1,
					ctrl->sched.num_cc1, curmaxsl);
			shiftsegoffsets(ctrl, ctrl->sched.chc3,
					ctrl->sched.num_cc3, curmaxsl);
		}
		/* subframe mode to maximize BW */
		maxctrlw3 = opensl3[0];
		maxctrlw1 = opensl1[0];
		if (opensl3[0] == curmaxsl)
			maxctrlw3 += opensl3[1];
		for (i = 0; i < 5 && opensl1[i] == curmaxsl; i++)
			maxctrlw1 += opensl1[i + 1];
		if (curintr >= 32) {
			*subfrml = 32;
			*ctrlw = maxctrlw3;
		} else if (curintr == 16) {
			if (maxctrlw3 > (opensl3[1] * 4)) {
				*subfrml = 32;
				*ctrlw = maxctrlw3;
			} else {
				*subfrml = 8;
				*ctrlw = opensl3[1];
			}
		} else {
			if ((maxctrlw1 * 8) >= (maxctrlw3 * 24)) {
				*subfrml = 24;
				*ctrlw = maxctrlw1;
			} else {
				*subfrml = 8;
				*ctrlw = maxctrlw3;
			}
		}
	}
	return 0;
}

static int slim_allocbw(struct slim_device *sb, u32 *subfrmc, u32 *clkgear)
{
	u32 msgsl = 0;
	u32 ctrlw = 0;
	u32 subfrml = 0;
	int ret = -EIO;
	struct slim_controller *ctrl = sb->ctrl;
	u32 usedsl = ctrl->sched.usedslots + ctrl->sched.msgsl;
	u32 availsl = SLIM_SL_PER_SUPERFRAME - SLIM_FRM_SLOTS_PER_SUPERFRAME -
		SLIM_GDE_SLOTS_PER_SUPERFRAME;

	*clkgear = SLIM_MAX_CLK_GEAR;

	/*
	 * If number of slots are 0, that means channels are inactive.
	 * It is very likely that the manager will call clock pause very soon.
	 * By making sure that bus is in MAX_GEAR, clk pause sequence will take
	 * minimum amount of time.
	 */
	if (ctrl->sched.usedslots != 0) {
		while ((usedsl * 2 <= availsl) && (*clkgear > ctrl->min_cg)) {
			*clkgear -= 1;
			usedsl *= 2;
		}
	}

	/*
	 * Try scheduling data channels at current clock gear, if all channels
	 * can be scheduled, or reserved BW can't be satisfied, increase clock
	 * gear and try again
	 */
	for (; *clkgear <= ctrl->max_cg; (*clkgear)++) {
		ret = slim_sched_chans(sb, *clkgear, &ctrlw, &subfrml);

		if (ret == 0) {
			*subfrmc = getsubfrmcoding(&ctrlw, &subfrml, &msgsl);
			if ((msgsl >> (ctrl->max_cg - *clkgear) <
			     ctrl->sched.msgsl) &&
			    (*clkgear < ctrl->max_cg))
				continue;
			else
				break;
		}
	}

	return ret;
}

static void slim_change_existing_chans(struct slim_controller *ctrl, int coeff)
{
	struct slim_channel **arr;
	int len, i;

	if (coeff == SLIM_COEFF_1) {
		arr = ctrl->sched.chc1;
		len = ctrl->sched.num_cc1;
	} else {
		arr = ctrl->sched.chc3;
		len = ctrl->sched.num_cc3;
	}
	for (i = 0; i < len; i++) {
		struct slim_channel *slc = arr[i];

		if (slc->state == SLIM_CH_STATE_ACTIVE) {
			slc->offset = slc->newoff;
			slc->interval = slc->newintr;
		}
	}
}

static void slim_chan_changes(struct slim_device *sb, bool revert)
{
	struct slim_controller *ctrl = sb->ctrl;

	while (!list_empty(&sb->mark_define)) {
		struct slim_channel *slc;
		struct slim_pending_ch *pch =
			list_entry(sb->mark_define.next,
				   struct slim_pending_ch, pending);

		slc = ctrl->sched.ch_map[pch->chan];
		if (revert) {
			if (slc && slc->state == SLIM_CH_STATE_PENDING_ACTIVE) {
				u32 sl = slc->seglen << slc->rootexp;

				if (slc->coeff == SLIM_COEFF_3)
					sl *= 3;
				ctrl->sched.usedslots -= sl;
				slim_remove_ch(ctrl, slc);
				slc->state = SLIM_CH_STATE_DEFINED;
			}
		} else if (slc) {
			slc->state = SLIM_CH_STATE_ACTIVE;
			slc->def++;
		}
		list_del_init(&pch->pending);
		kfree(pch);
	}

	while (!list_empty(&sb->mark_removal)) {
		struct slim_pending_ch *pch =
			list_entry(sb->mark_removal.next,
				   struct slim_pending_ch, pending);
		struct slim_channel *slc = ctrl->sched.ch_map[pch->chan];

		if (slc) {
			u32 sl = slc->seglen << slc->rootexp;

			if (revert) {
				if (slc->coeff == SLIM_COEFF_3)
					sl *= 3;
				ctrl->sched.usedslots += sl;
				slc->def = 1;
				slc->state = SLIM_CH_STATE_ACTIVE;
			} else
				slim_remove_ch(ctrl, slc);
		}
		list_del_init(&pch->pending);
		kfree(pch);
	}

	/* Change already active channel if reconfig succeeded */
	if (!revert) {
		slim_change_existing_chans(ctrl, SLIM_COEFF_1);
		slim_change_existing_chans(ctrl, SLIM_COEFF_3);
	}
}

int slim_reconfigure_now(struct slim_device *sb)
{
	u8 i;
	u8 wbuf[4];
	u32 clkgear, subframe;
	u32 curexp;
	int ret;
	struct slim_controller *ctrl = sb->ctrl;
	u32 expshft;
	u32 segdist;
	struct slim_pending_ch *pch;
	struct slim_val_inf msg = {0};

	DEFINE_SLIM_BCAST_TXN(txn, SLIM_MSG_MC_BEGIN_RECONFIGURATION,
				3, 0, &msg);

	mutex_lock(&ctrl->sched.m_reconf);
	mutex_lock(&ctrl->lock);
	/*
	 * If there are no pending changes from this client, avoid sending
	 * the reconfiguration sequence
	 */
	if (list_empty(&sb->mark_define)) {
		struct list_head *pos, *next;

		list_for_each_safe(pos, next, &sb->mark_removal) {
			struct slim_channel *slc;

			pch = list_entry(pos, struct slim_pending_ch, pending);
			slc = ctrl->sched.ch_map[pch->chan];
			if (slc && slc->def > 0)
				slc->def--;
			if (slc && slc->def != 0) {
				list_del(&pch->pending);
				kfree(pch);
			}
		}
		if (list_empty(&sb->mark_removal)) {
			mutex_unlock(&ctrl->lock);
			mutex_unlock(&ctrl->sched.m_reconf);
			return 0;
		}
	}

	list_for_each_entry(pch, &sb->mark_define, pending) {
		struct slim_channel *slc = ctrl->sched.ch_map[pch->chan];

		if (!slc)
			continue;
		slim_add_ch(ctrl, slc);
		if (slc->state < SLIM_CH_STATE_ACTIVE)
			slc->state = SLIM_CH_STATE_PENDING_ACTIVE;
	}

	list_for_each_entry(pch, &sb->mark_removal, pending) {
		struct slim_channel *slc = ctrl->sched.ch_map[pch->chan];
		u32 sl;

		if (!slc)
			continue;
		sl = slc->seglen << slc->rootexp;
		if (slc->coeff == SLIM_COEFF_3)
			sl *= 3;
		ctrl->sched.usedslots -= sl;
		slc->state = SLIM_CH_STATE_PENDING_REMOVAL;
	}

	ret = slim_allocbw(sb, &subframe, &clkgear);

	if (!ret)
		ret = slim_do_transfer(ctrl, &txn);

	if (!ret && subframe != ctrl->sched.subfrmcode) {
		wbuf[0] = (u8)(subframe & 0xFF);
		msg.num_bytes = 1;
		msg.wbuf = wbuf;
		txn.mc = SLIM_MSG_MC_NEXT_SUBFRAME_MODE;
		txn.rl = 4;
		ret = slim_do_transfer(ctrl, &txn);
	}
	if (!ret && clkgear != (u32)ctrl->clkgear) {
		wbuf[0] = (u8)(clkgear & 0xFF);
		msg.num_bytes = 1;
		msg.wbuf = wbuf;
		txn.mc = SLIM_MSG_MC_NEXT_CLOCK_GEAR;
		txn.rl = 4;
		ret = slim_do_transfer(ctrl, &txn);
	}
	if (ret)
		goto revert_reconfig;

	expshft = SLIM_MAX_CLK_GEAR - clkgear;
	/* activate/remove channel */
	list_for_each_entry(pch, &sb->mark_define, pending) {
		struct slim_channel *slc = ctrl->sched.ch_map[pch->chan];

		if (!slc)
			continue;
		/* Define content */
		wbuf[0] = slc->id;
		wbuf[1] = slc->prrate;
		wbuf[2] = slc->data_fmt | (slc->aux_fmt << 4);
		wbuf[3] = slc->bps / SLIM_SLOT_LEN_BITS;
		msg.num_bytes = 4;
		msg.wbuf = wbuf;
		txn.mc = SLIM_MSG_MC_NEXT_DEFINE_CONTENT;
		txn.rl = 7;
		ret = slim_do_transfer(ctrl, &txn);
		if (ret)
			goto revert_reconfig;

		wbuf[0] = slc->id;
		msg.num_bytes = 1;
		msg.wbuf = wbuf;
		txn.mc = SLIM_MSG_MC_NEXT_ACTIVATE_CHANNEL;
		txn.rl = 4;
		ret = slim_do_transfer(ctrl, &txn);
		if (ret)
			goto revert_reconfig;
	}

	list_for_each_entry(pch, &sb->mark_removal, pending) {
		struct slim_channel *slc = ctrl->sched.ch_map[pch->chan];

		if (!slc)
			continue;
		wbuf[0] = slc->id;
		msg.num_bytes = 1;
		msg.wbuf = wbuf;
		txn.mc = SLIM_MSG_MC_NEXT_REMOVE_CHANNEL;
		txn.rl = 4;
		ret = slim_do_transfer(ctrl, &txn);
		if (ret)
			goto revert_reconfig;
	}

	/* Define CC1 channel */
	for (i = 0; i < ctrl->sched.num_cc1; i++) {
		struct slim_channel *slc = ctrl->sched.chc1[i];

		if (!slc || slc->state == SLIM_CH_STATE_PENDING_REMOVAL)
			continue;
		curexp = slc->rootexp + expshft;
		segdist = (slc->newoff << curexp) & 0x1FF;
		expshft = SLIM_MAX_CLK_GEAR - clkgear;

		if (slc->state < SLIM_CH_STATE_ACTIVE || !slc->def ||
		    slc->newintr != slc->interval ||
		    slc->newoff != slc->offset) {
			segdist |= 0x200;
			segdist >>= curexp;
			segdist |= (slc->newoff << (curexp + 1)) & 0xC00;
			wbuf[0] = slc->id;
			wbuf[1] = (u8)(segdist & 0xFF);
			wbuf[2] = (u8)((segdist & 0xF00) >> 8) |
				(slc->prot << 4);
			wbuf[3] = slc->seglen;
			msg.num_bytes = 4;
			msg.wbuf = wbuf;
			txn.mc = SLIM_MSG_MC_NEXT_DEFINE_CHANNEL;
			txn.rl = 7;
			ret = slim_do_transfer(ctrl, &txn);
			if (ret)
				goto revert_reconfig;
		}
	}

	/* Define CC3 channels */
	for (i = 0; i < ctrl->sched.num_cc3; i++) {
		struct slim_channel *slc = ctrl->sched.chc3[i];

		if (!slc || slc->state == SLIM_CH_STATE_PENDING_REMOVAL)
			continue;
		curexp = slc->rootexp + expshft;
		segdist = (slc->newoff << curexp) & 0x1FF;
		expshft = SLIM_MAX_CLK_GEAR - clkgear;

		if (slc->state < SLIM_CH_STATE_ACTIVE || !slc->def ||
		    slc->newintr != slc->interval ||
		    slc->newoff != slc->offset) {
			segdist |= 0x200;
			segdist >>= curexp;
			segdist |= 0xC00;
			wbuf[0] = slc->id;
			wbuf[1] = (u8)(segdist & 0xFF);
			wbuf[2] = (u8)((segdist & 0xF00) >> 8) |
				(slc->prot << 4);
			wbuf[3] = (u8)(slc->seglen);
			msg.num_bytes = 4;
			msg.wbuf = wbuf;
			txn.mc = SLIM_MSG_MC_NEXT_DEFINE_CHANNEL;
			txn.rl = 7;
			ret = slim_do_transfer(ctrl, &txn);
			if (ret)
				goto revert_reconfig;
		}
	}

	txn.mc = SLIM_MSG_MC_RECONFIGURE_NOW;
	txn.rl = 3;
	msg.num_bytes = 0;
	msg.wbuf = NULL;
	ret = slim_do_transfer(ctrl, &txn);
	if (!ret) {
		ctrl->sched.subfrmcode = subframe;
		ctrl->clkgear = clkgear;
		slim_chan_changes(sb, false);
		mutex_unlock(&ctrl->lock);
		mutex_unlock(&ctrl->sched.m_reconf);
		return 0;
	}

revert_reconfig:
	/* Revert channel changes */
	slim_chan_changes(sb, true);
	mutex_unlock(&ctrl->lock);
	mutex_unlock(&ctrl->sched.m_reconf);
	return ret;
}

static int add_pending_ch(struct list_head *listh, u8 chan)
{
	struct slim_pending_ch *pch;

	pch = kmalloc_obj(*pch);
	if (!pch)
		return -ENOMEM;
	pch->chan = chan;
	list_add_tail(&pch->pending, listh);
	return 0;
}

int slim_control_ch(struct slim_device *sbdev, u8 chan,
		    enum slim_ch_control chctrl)
{
	struct slim_controller *ctrl = sbdev->ctrl;
	struct slim_channel *slc;
	int ret = 0;

	mutex_lock(&ctrl->lock);
	slc = ctrl->sched.ch_map[chan];
	if (!slc) {
		ret = -EINVAL;
		goto unlock;
	}
	if (slc->state < SLIM_CH_STATE_DEFINED) {
		ret = -ENOTCONN;
		goto unlock;
	}

	if (chctrl == SLIM_CH_ACTIVATE) {
		if (slc->state > SLIM_CH_STATE_ACTIVE) {
			ret = -EISCONN;
			goto unlock;
		}
		ret = add_pending_ch(&sbdev->mark_define, chan);
	} else if (chctrl == SLIM_CH_REMOVE) {
		struct slim_pending_ch *pch, *next;

		if (slc->state < SLIM_CH_STATE_ACTIVE) {
			ret = -ENOTCONN;
			goto unlock;
		}
		/* Cancel a pending definition instead of scheduling its removal. */
		list_for_each_entry_safe(pch, next, &sbdev->mark_define, pending) {
			if (pch->chan == chan) {
				list_del(&pch->pending);
				kfree(pch);
				goto unlock;
			}
		}
		ret = add_pending_ch(&sbdev->mark_removal, chan);
	} else {
		ret = -EINVAL;
	}

unlock:
	mutex_unlock(&ctrl->lock);
	return ret;
}
