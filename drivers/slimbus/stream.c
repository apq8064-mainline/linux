// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018, Linaro Limited

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/slimbus.h>
#include <uapi/sound/asound.h>
#include "slimbus.h"

/**
 * struct segdist_code - Segment Distributions code from
 *	Table 20 of SLIMbus Specs Version 2.0
 *
 * @ratem: Channel Rate Multipler(Segments per Superframe)
 * @seg_interval: Number of slots between the first Slot of Segment
 *		and the first slot of the next  consecutive Segment.
 * @segdist_code: Segment Distribution Code SD[11:0]
 * @seg_offset_mask: Segment offset mask in SD[11:0]
 */
struct segdist_code {
	int ratem;
	int seg_interval;
	int segdist_code;
	u32 seg_offset_mask;

};

/* segdist_codes - List of all possible Segment Distribution codes. */
static const struct segdist_code segdist_codes[] = {
	{1,	1536,	0x200,	 0xdff},
	{2,	768,	0x100,	 0xcff},
	{4,	384,	0x080,	 0xc7f},
	{8,	192,	0x040,	 0xc3f},
	{16,	96,	0x020,	 0xc1f},
	{32,	48,	0x010,	 0xc0f},
	{64,	24,	0x008,	 0xc07},
	{128,	12,	0x004,	 0xc03},
	{256,	6,	0x002,	 0xc01},
	{512,	3,	0x001,	 0xc00},
	{3,	512,	0xe00,	 0x1ff},
	{6,	256,	0xd00,	 0x0ff},
	{12,	128,	0xc80,	 0x07f},
	{24,	64,	0xc40,	 0x03f},
	{48,	32,	0xc20,	 0x01f},
	{96,	16,	0xc10,	 0x00f},
	{192,	8,	0xc08,	 0x007},
	{364,	4,	0xc04,	 0x003},
	{768,	2,	0xc02,	 0x001},
};

/*
 * Presence Rate table for all Natural Frequencies
 * The Presence rate of a constant bitrate stream is mean flow rate of the
 * stream expressed in occupied Segments of that Data Channel per second.
 * Table 66 from SLIMbus 2.0 Specs
 *
 * Index of the table corresponds to Presence rate code for the respective rate
 * in the table.
 */
static const int slim_presence_rate_table[] = {
	0, /* Not Indicated */
	12000,
	24000,
	48000,
	96000,
	192000,
	384000,
	768000,
	0, /* Reserved */
	11025,
	22050,
	44100,
	88200,
	176400,
	352800,
	705600,
	4000,
	8000,
	16000,
	32000,
	64000,
	128000,
	256000,
	512000,
};

/**
 * slim_stream_allocate() - Allocate a new SLIMbus Stream
 * @dev:Slim device to be associated with
 * @name: name of the stream
 *
 * This is very first call for SLIMbus streaming, this API will allocate
 * a new SLIMbus stream and return a valid stream runtime pointer for client
 * to use it in subsequent stream apis. state of stream is set to ALLOCATED
 *
 * Return: valid pointer on success and error code on failure.
 * From ASoC DPCM framework, this state is linked to startup() operation.
 */
struct slim_stream_runtime *slim_stream_allocate(struct slim_device *dev,
						 const char *name)
{
	struct slim_stream_runtime *rt;

	rt = kzalloc_obj(*rt);
	if (!rt)
		return ERR_PTR(-ENOMEM);

	rt->name = kasprintf(GFP_KERNEL, "slim-%s", name);
	if (!rt->name) {
		kfree(rt);
		return ERR_PTR(-ENOMEM);
	}

	rt->dev = dev;
	spin_lock(&dev->stream_list_lock);
	list_add_tail(&rt->node, &dev->stream_list);
	spin_unlock(&dev->stream_list_lock);

	return rt;
}
EXPORT_SYMBOL_GPL(slim_stream_allocate);

static int slim_connect_port_channel(struct slim_stream_runtime *stream,
				     struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[2];
	struct slim_val_inf msg = {0, 2, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_CONNECT_SOURCE;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 6, stream->dev->laddr, &msg);

	if (port->direction == SLIM_PORT_SINK)
		txn.mc = SLIM_MSG_MC_CONNECT_SINK;

	wbuf[0] = port->id;
	wbuf[1] = port->ch.id;
	port->ch.state = SLIM_CH_STATE_ASSOCIATED;
	port->state = SLIM_PORT_UNCONFIGURED;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_disconnect_port(struct slim_stream_runtime *stream,
				struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[1];
	struct slim_val_inf msg = {0, 1, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_DISCONNECT_PORT;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 5, stream->dev->laddr, &msg);

	wbuf[0] = port->id;
	port->ch.state = SLIM_CH_STATE_DISCONNECTED;
	port->state = SLIM_PORT_DISCONNECTED;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_deactivate_remove_channel(struct slim_stream_runtime *stream,
					  struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[1];
	struct slim_val_inf msg = {0, 1, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_NEXT_DEACTIVATE_CHANNEL;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 5, stream->dev->laddr, &msg);
	int ret;

	wbuf[0] = port->ch.id;
	ret = slim_do_transfer(sdev->ctrl, &txn);
	if (ret)
		return ret;

	txn.mc = SLIM_MSG_MC_NEXT_REMOVE_CHANNEL;
	port->ch.state = SLIM_CH_STATE_REMOVED;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_get_prate_code(int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(slim_presence_rate_table); i++) {
		if (rate == slim_presence_rate_table[i])
			return i;
	}

	return -EINVAL;
}

static int slim_mgr_connect_port_channel(struct slim_stream_runtime *stream,
				     struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[2];
	struct slim_val_inf msg = {0, 2, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_CONNECT_SOURCE;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 6, stream->dev->laddr, &msg);

	if (port->direction == SLIM_PORT_SINK)
		txn.mc = SLIM_MSG_MC_CONNECT_SINK;

	wbuf[0] = port->id;
	wbuf[1] = port->ch.id;
	port->state = SLIM_PORT_UNCONFIGURED;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_connect_mgr_port(struct slim_stream_runtime *stream,
				 struct slim_port *port, int idx)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[2];
	struct slim_val_inf msg = {0, 2, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_CONNECT_SOURCE;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 6, SLIM_LA_MANAGER, &msg);

	if (port->direction == SLIM_PORT_SOURCE)
		txn.mc = SLIM_MSG_MC_CONNECT_SINK;

	wbuf[0] = idx;
	wbuf[1] = port->ch.id;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_disconnect_mgr_port(struct slim_stream_runtime *stream,
				    int idx)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[1];
	struct slim_val_inf msg = {0, 1, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_DISCONNECT_PORT;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 5, SLIM_LA_MANAGER, &msg);

	wbuf[0] = idx;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_mgr_stream_prepare(struct slim_stream_runtime *rt,
			struct slim_stream_config *cfg)
{
	struct slim_controller *ctrl = rt->dev->ctrl;
	struct slim_port *port;
	int num_ports, i, port_id, ret, connected = 0;

	if (!cfg->chs || !cfg->ch_count || !cfg->bps ||
	    cfg->bps % SLIM_SLOT_LEN_BITS ||
	    cfg->rate < ctrl->a_framer->superfreq ||
	    (cfg->port_mask & ~GENMASK(SLIM_DEVICE_MAX_PORTS - 1, 0)) ||
	    (cfg->direction != SNDRV_PCM_STREAM_PLAYBACK &&
	     cfg->direction != SNDRV_PCM_STREAM_CAPTURE) ||
	    cfg->ch_count != hweight_long(cfg->port_mask))
		return -EINVAL;

	if (rt->ports) {
		dev_err(&rt->dev->dev, "Stream already Prepared\n");
		return -EINVAL;
	}

	num_ports = hweight32(cfg->port_mask);
	rt->ports = kzalloc_objs(*port, num_ports);
	if (!rt->ports)
		return -ENOMEM;

	rt->num_ports = num_ports;
	rt->rate = cfg->rate;
	rt->bps = cfg->bps;
	rt->direction = cfg->direction;

	if (cfg->rate % ctrl->a_framer->superfreq) {
		/*
		 * data rate not exactly multiple of super frame,
		 * use PUSH/PULL protocol
		 */
		if (cfg->direction == SNDRV_PCM_STREAM_PLAYBACK)
			rt->prot = SLIM_PROTO_PUSH;
		else
			rt->prot = SLIM_PROTO_PULL;
	} else {
		rt->prot = SLIM_PROTO_ISO;
	}

	rt->ratem = cfg->rate / ctrl->a_framer->superfreq;

	i = 0;
	for_each_set_bit(port_id, &cfg->port_mask, SLIM_DEVICE_MAX_PORTS) {
		port = &rt->ports[i];
		port->state = SLIM_PORT_DISCONNECTED;
		port->id = port_id;
		port->ch.id = cfg->chs[i];
		if (cfg->chs[i] >= SLIM_DEVICE_MAX_CHANNELS) {
			ret = -EINVAL;
			goto err_unprepare;
		}
		port->ch.data_fmt = SLIM_CH_DATA_FMT_NOT_DEFINED;
		port->ch.aux_fmt = SLIM_CH_AUX_FMT_NOT_APPLICABLE;
		port->ch.state = SLIM_CH_STATE_ALLOCATED;
		port->ch.bps = rt->bps;

		if (cfg->direction == SNDRV_PCM_STREAM_PLAYBACK)
			port->direction = SLIM_PORT_SINK;
		else
			port->direction = SLIM_PORT_SOURCE;

		ret = slim_nextdefine_ch(rt, port);
		if (ret)
			goto err_unprepare;

		port->ch.state = SLIM_CH_STATE_DEFINED;
		mutex_lock(&ctrl->lock);
		if (ctrl->sched.ch_map[port->ch.id]) {
			mutex_unlock(&ctrl->lock);
			ret = -EBUSY;
			goto err_unprepare;
		}
		ctrl->sched.ch_map[port->ch.id] = &port->ch;
		mutex_unlock(&ctrl->lock);

		i++;
	}

	/*
	 * Connect the manager-side data ports during prepare so that the DMA
	 * channel (e.g. the Qualcomm BAM pipe) is already allocated by the time
	 * the CPU-side trigger tries to queue descriptors.  The codec-side ports
	 * are still connected in slim_stream_enable(), keeping activation close
	 * to the channel reconfiguration.
	 */
	for (i = 0; i < num_ports; i++) {
		ret = slim_connect_mgr_port(rt, &rt->ports[i], i);
		if (ret)
			goto err_unprepare;
		connected++;
	}

	return 0;

err_unprepare:
	while (connected--)
		slim_disconnect_mgr_port(rt, connected);
	mutex_lock(&ctrl->lock);
	for (i = 0; i < num_ports; i++) {
		if (rt->ports[i].ch.id >= 0 &&
		    rt->ports[i].ch.id < SLIM_DEVICE_MAX_CHANNELS &&
		    ctrl->sched.ch_map[rt->ports[i].ch.id] == &rt->ports[i].ch)
			ctrl->sched.ch_map[rt->ports[i].ch.id] = NULL;
	}
	mutex_unlock(&ctrl->lock);
	kfree(rt->ports);
	rt->ports = NULL;
	rt->num_ports = 0;
	return ret;
}

static int slim_mgr_stream_enable(struct slim_stream_runtime *stream)
{
	int ret, i;

	/*
	 * Connect the codec-side ports before the reconfiguration, matching the
	 * downstream Qualcomm sequence.  The manager-side ports were already
	 * connected in slim_stream_prepare() so that their DMA channels are
	 * allocated before the CPU-side trigger queues descriptors.
	 */
	for (i = 0; i < stream->num_ports; i++) {
		ret = slim_mgr_connect_port_channel(stream, &stream->ports[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < stream->num_ports; i++) {
		struct slim_port *port = &stream->ports[i];

		ret = slim_control_ch(stream->dev, port->ch.id, SLIM_CH_ACTIVATE);
		if (ret)
			return ret;
	}

	ret = slim_reconfigure_now(stream->dev);
	if (ret)
		return ret;

	for (i = 0; i < stream->num_ports; i++)
		stream->ports[i].state = SLIM_PORT_CONFIGURED;

	return 0;
}

static int slim_mgr_stream_disable(struct slim_stream_runtime *stream)
{
	int ret, i;

	if (!stream->ports || !stream->num_ports)
		return -EINVAL;

	for (i = 0; i < stream->num_ports; i++) {
		struct slim_port *port = &stream->ports[i];

		ret = slim_control_ch(stream->dev, port->ch.id, SLIM_CH_REMOVE);
		if (ret)
			return ret;
	}

	return slim_reconfigure_now(stream->dev);
}

static int slim_mgr_stream_unprepare(struct slim_stream_runtime *stream)
{
	int i, ret = 0;

	if (!stream->ports || !stream->num_ports)
		return -EINVAL;

	/*
	 * If the stream is still active, remove its channels from the scheduler
	 * before tearing down the ports. Some CPU-DAI trigger paths skip
	 * slim_stream_disable() and call unprepare directly.
	 */
	for (i = 0; i < stream->num_ports; i++) {
		struct slim_port *port = &stream->ports[i];

		if (port->ch.state == SLIM_CH_STATE_ACTIVE) {
			int err = slim_control_ch(stream->dev, port->ch.id,
						  SLIM_CH_REMOVE);
			if (err && err != -ENOTCONN)
				dev_err(&stream->dev->dev,
					"unprepare: remove ch:%d failed:%d\n",
					port->ch.id, err);
		}
	}

	ret = slim_reconfigure_now(stream->dev);
	if (ret)
		dev_err(&stream->dev->dev,
			"unprepare: reconfigure removal failed:%d\n", ret);

	for (i = 0; i < stream->num_ports; i++) {
		int err = slim_disconnect_port(stream, &stream->ports[i]);

		if (err) {
			dev_err(&stream->dev->dev,
				"unprepare: disconnect codec port %d failed:%d\n",
				stream->ports[i].id, err);
			if (!ret)
				ret = err;
		}
		err = slim_disconnect_mgr_port(stream, i);
		if (err && !ret)
			ret = err;
	}

	/* Reconfiguration failure must not leave pointers into freed ports. */
	slim_sched_unprepare(stream);
	kfree(stream->ports);
	stream->ports = NULL;
	stream->num_ports = 0;

	return ret;
}

/**
 * slim_stream_prepare() - Prepare a SLIMbus Stream
 *
 * @rt: instance of slim stream runtime to configure
 * @cfg: new configuration for the stream
 *
 * This API will configure SLIMbus stream with config parameters from cfg.
 * return zero on success and error code on failure. From ASoC DPCM framework,
 * this state is linked to hw_params() operation.
 */
int slim_stream_prepare(struct slim_stream_runtime *rt,
			struct slim_stream_config *cfg)
{
	struct slim_controller *ctrl = rt->dev->ctrl;
	struct slim_port *port;
	int num_ports, i, port_id, prrate;

	if (ctrl->mgr_data_ports)
		return slim_mgr_stream_prepare(rt, cfg);

	if (rt->ports) {
		dev_err(&rt->dev->dev, "Stream already Prepared\n");
		return -EINVAL;
	}

	num_ports = hweight32(cfg->port_mask);
	rt->ports = kzalloc_objs(*port, num_ports);
	if (!rt->ports)
		return -ENOMEM;

	rt->num_ports = num_ports;
	rt->rate = cfg->rate;
	rt->bps = cfg->bps;
	rt->direction = cfg->direction;

	prrate = slim_get_prate_code(cfg->rate);
	if (prrate < 0) {
		dev_err(&rt->dev->dev, "Cannot get presence rate for rate %d Hz\n",
			cfg->rate);
		return prrate;
	}

	if (cfg->rate % ctrl->a_framer->superfreq) {
		/*
		 * data rate not exactly multiple of super frame,
		 * use PUSH/PULL protocol
		 */
		if (cfg->direction == SNDRV_PCM_STREAM_PLAYBACK)
			rt->prot = SLIM_PROTO_PUSH;
		else
			rt->prot = SLIM_PROTO_PULL;
	} else {
		rt->prot = SLIM_PROTO_ISO;
	}

	rt->ratem = cfg->rate/ctrl->a_framer->superfreq;

	i = 0;
	for_each_set_bit(port_id, &cfg->port_mask, SLIM_DEVICE_MAX_PORTS) {
		port = &rt->ports[i];
		port->state = SLIM_PORT_DISCONNECTED;
		port->id = port_id;
		port->ch.prrate = prrate;
		port->ch.id = cfg->chs[i];
		port->ch.data_fmt = SLIM_CH_DATA_FMT_NOT_DEFINED;
		port->ch.aux_fmt = SLIM_CH_AUX_FMT_NOT_APPLICABLE;
		port->ch.state = SLIM_CH_STATE_ALLOCATED;

		if (cfg->direction == SNDRV_PCM_STREAM_PLAYBACK)
			port->direction = SLIM_PORT_SINK;
		else
			port->direction = SLIM_PORT_SOURCE;

		slim_connect_port_channel(rt, port);
		i++;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(slim_stream_prepare);

static int slim_define_channel_content(struct slim_stream_runtime *stream,
				       struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[4];
	struct slim_val_inf msg = {0, 4, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_NEXT_DEFINE_CONTENT;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 8, stream->dev->laddr, &msg);

	wbuf[0] = port->ch.id;
	wbuf[1] = port->ch.prrate;

	/* Frequency Locked for ISO Protocol */
	if (stream->prot != SLIM_PROTO_ISO)
		wbuf[1] |= SLIM_CHANNEL_CONTENT_FL;

	wbuf[2] = port->ch.data_fmt | (port->ch.aux_fmt << 4);
	wbuf[3] = stream->bps/SLIM_SLOT_LEN_BITS;
	port->ch.state = SLIM_CH_STATE_CONTENT_DEFINED;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_get_segdist_code(int ratem)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(segdist_codes); i++) {
		if (segdist_codes[i].ratem == ratem)
			return segdist_codes[i].segdist_code;
	}

	return -EINVAL;
}

static int slim_define_channel(struct slim_stream_runtime *stream,
				       struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[4];
	struct slim_val_inf msg = {0, 4, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_NEXT_DEFINE_CHANNEL;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 8, stream->dev->laddr, &msg);

	port->ch.seg_dist = slim_get_segdist_code(stream->ratem);

	wbuf[0] = port->ch.id;
	wbuf[1] = port->ch.seg_dist & 0xFF;
	wbuf[2] = (stream->prot << 4) | ((port->ch.seg_dist & 0xF00) >> 8);
	if (stream->prot == SLIM_PROTO_ISO)
		wbuf[3] = stream->bps/SLIM_SLOT_LEN_BITS;
	else
		wbuf[3] = stream->bps/SLIM_SLOT_LEN_BITS + 1;

	port->ch.state = SLIM_CH_STATE_DEFINED;

	return slim_do_transfer(sdev->ctrl, &txn);
}

static int slim_activate_channel(struct slim_stream_runtime *stream,
				 struct slim_port *port)
{
	struct slim_device *sdev = stream->dev;
	u8 wbuf[1];
	struct slim_val_inf msg = {0, 1, NULL, wbuf, NULL};
	u8 mc = SLIM_MSG_MC_NEXT_ACTIVATE_CHANNEL;
	DEFINE_SLIM_LDEST_TXN(txn, mc, 5, stream->dev->laddr, &msg);

	txn.msg->num_bytes = 1;
	txn.msg->wbuf = wbuf;
	wbuf[0] = port->ch.id;
	port->ch.state = SLIM_CH_STATE_ACTIVE;

	return slim_do_transfer(sdev->ctrl, &txn);
}

/**
 * slim_stream_enable() - Enable a prepared SLIMbus Stream
 *
 * @stream: instance of slim stream runtime to enable
 *
 * This API will enable all the ports and channels associated with
 * SLIMbus stream
 *
 * Return: zero on success and error code on failure. From ASoC DPCM framework,
 * this state is linked to trigger() start operation.
 */
int slim_stream_enable(struct slim_stream_runtime *stream)
{
	DEFINE_SLIM_BCAST_TXN(txn, SLIM_MSG_MC_BEGIN_RECONFIGURATION,
				3, SLIM_LA_MANAGER, NULL);
	struct slim_controller *ctrl = stream->dev->ctrl;
	int ret, i;

	if (ctrl->mgr_data_ports)
		return slim_mgr_stream_enable(stream);

	if (ctrl->enable_stream) {
		ret = ctrl->enable_stream(stream);
		if (ret)
			return ret;

		for (i = 0; i < stream->num_ports; i++)
			stream->ports[i].ch.state = SLIM_CH_STATE_ACTIVE;

		return ret;
	}

	ret = slim_do_transfer(ctrl, &txn);
	if (ret)
		return ret;

	/* define channels first before activating them */
	for (i = 0; i < stream->num_ports; i++) {
		struct slim_port *port = &stream->ports[i];

		slim_define_channel(stream, port);
		slim_define_channel_content(stream, port);
	}

	for (i = 0; i < stream->num_ports; i++) {
		struct slim_port *port = &stream->ports[i];

		slim_activate_channel(stream, port);
		port->state = SLIM_PORT_CONFIGURED;
	}
	txn.mc = SLIM_MSG_MC_RECONFIGURE_NOW;

	return slim_do_transfer(ctrl, &txn);
}
EXPORT_SYMBOL_GPL(slim_stream_enable);

/**
 * slim_stream_disable() - Disable a SLIMbus Stream
 *
 * @stream: instance of slim stream runtime to disable
 *
 * This API will disable all the ports and channels associated with
 * SLIMbus stream
 *
 * Return: zero on success and error code on failure. From ASoC DPCM framework,
 * this state is linked to trigger() pause operation.
 */
int slim_stream_disable(struct slim_stream_runtime *stream)
{
	DEFINE_SLIM_BCAST_TXN(txn, SLIM_MSG_MC_BEGIN_RECONFIGURATION,
				3, SLIM_LA_MANAGER, NULL);
	struct slim_controller *ctrl = stream->dev->ctrl;
	int ret, i;

	if (ctrl->mgr_data_ports)
		return slim_mgr_stream_disable(stream);

	if (!stream->ports || !stream->num_ports)
		return -EINVAL;

	if (ctrl->disable_stream)
		ctrl->disable_stream(stream);

	ret = slim_do_transfer(ctrl, &txn);
	if (ret)
		return ret;

	for (i = 0; i < stream->num_ports; i++)
		slim_deactivate_remove_channel(stream, &stream->ports[i]);

	txn.mc = SLIM_MSG_MC_RECONFIGURE_NOW;

	return slim_do_transfer(ctrl, &txn);
}
EXPORT_SYMBOL_GPL(slim_stream_disable);

/**
 * slim_stream_unprepare() - Un-prepare a SLIMbus Stream
 *
 * @stream: instance of slim stream runtime to unprepare
 *
 * This API will un allocate all the ports and channels associated with
 * SLIMbus stream
 *
 * Return: zero on success and error code on failure. From ASoC DPCM framework,
 * this state is linked to trigger() stop operation.
 */
int slim_stream_unprepare(struct slim_stream_runtime *stream)
{
	struct slim_controller *ctrl = stream->dev->ctrl;
	int i;

	if (ctrl->mgr_data_ports)
		return slim_mgr_stream_unprepare(stream);

	if (!stream->ports || !stream->num_ports)
		return -EINVAL;

	for (i = 0; i < stream->num_ports; i++)
		slim_disconnect_port(stream, &stream->ports[i]);

	kfree(stream->ports);
	stream->ports = NULL;
	stream->num_ports = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(slim_stream_unprepare);

/**
 * slim_stream_free() - Free a SLIMbus Stream
 *
 * @stream: instance of slim stream runtime to free
 *
 * This API will un allocate all the memory associated with
 * slim stream runtime, user is not allowed to make an dereference
 * to stream after this call.
 *
 * Return: zero on success and error code on failure. From ASoC DPCM framework,
 * this state is linked to shutdown() operation.
 */
int slim_stream_free(struct slim_stream_runtime *stream)
{
	struct slim_device *sdev = stream->dev;

	spin_lock(&sdev->stream_list_lock);
	list_del(&stream->node);
	spin_unlock(&sdev->stream_list_lock);

	kfree(stream->name);
	kfree(stream);

	return 0;
}
EXPORT_SYMBOL_GPL(slim_stream_free);
