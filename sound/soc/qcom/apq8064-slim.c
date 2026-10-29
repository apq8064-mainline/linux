// SPDX-License-Identifier: GPL-2.0-only
/*
 * APQ8064 SLIMbus host PCM support.
 *
 * The Apps processor supplies PCM data through BAM manager port 0 when
 * audio is used without the downstream DSP stack.
 */

#include <linux/atomic.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/slimbus.h>
#include <linux/slimbus/qcom.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "apq8064-slim.h"

struct apq8064_slim_runtime {
	struct snd_pcm_substream *substream;
	struct slim_controller *ctrl;
	struct slim_stream_runtime *stream;
	struct slim_stream_config config;
	struct work_struct period_work;
	struct work_struct elapsed_work;
	atomic_t pending;
	unsigned int channel;
	unsigned int pos;
	unsigned int submit_pos;
	unsigned int period_bytes;
	unsigned int buffer_bytes;
	int error;
	bool running;
	bool prepared;
	bool enabled;
};

static void apq8064_slim_dma_done(void *data)
{
	struct apq8064_slim_runtime *sr = data;

	if (!READ_ONCE(sr->running))
		return;
	atomic_inc(&sr->pending);
	queue_work(system_highpri_wq, &sr->period_work);
}

static int apq8064_slim_submit(struct apq8064_slim_runtime *sr,
			      unsigned int offset)
{
	struct snd_pcm_runtime *runtime = sr->substream->runtime;
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_dma_address(&sg) = runtime->dma_addr + offset;
	sg_dma_len(&sg) = sr->period_bytes;
	return qcom_slim_port_xfer(sr->ctrl, 0, &sg, 1,
				  apq8064_slim_dma_done, sr);
}

static void apq8064_slim_elapsed_work(struct work_struct *work)
{
	struct apq8064_slim_runtime *sr =
		container_of(work, struct apq8064_slim_runtime, elapsed_work);

	if (!READ_ONCE(sr->running))
		return;
	if (READ_ONCE(sr->error))
		snd_pcm_stop_xrun(sr->substream);
	else
		snd_pcm_period_elapsed(sr->substream);
}

static void apq8064_slim_period_work(struct work_struct *work)
{
	struct apq8064_slim_runtime *sr =
		container_of(work, struct apq8064_slim_runtime, period_work);
	unsigned int pos;
	int pending, i, ret;

	if (!READ_ONCE(sr->running))
		return;
	pending = atomic_xchg(&sr->pending, 0);
	if (!pending)
		return;

	pos = (READ_ONCE(sr->pos) + pending * sr->period_bytes) %
		sr->buffer_bytes;
	WRITE_ONCE(sr->pos, pos);

	/* Replenish DMA before an elapsed notification can take the PCM lock. */
	for (i = 0; i < pending; i++) {
		ret = apq8064_slim_submit(sr, sr->submit_pos);
		if (ret) {
			WRITE_ONCE(sr->error, ret);
			break;
		}
		sr->submit_pos = (sr->submit_pos + sr->period_bytes) %
				sr->buffer_bytes;
	}
	queue_work(system_highpri_wq, &sr->elapsed_work);
}

/* Called with the PCM stream lock held: never wait for elapsed_work here. */
static int apq8064_slim_stop(struct apq8064_slim_runtime *sr)
{
	int ret = 0;

	WRITE_ONCE(sr->running, false);
	cancel_work_sync(&sr->period_work);
	qcom_slim_port_disable_hw(sr->ctrl, 0);
	qcom_slim_port_xfer_stop(sr->ctrl, 0);
	/* A callback already in flight may have queued one last work item. */
	cancel_work_sync(&sr->period_work);
	cancel_work(&sr->elapsed_work);
	atomic_set(&sr->pending, 0);

	if (sr->prepared) {
		ret = slim_stream_unprepare(sr->stream);
		sr->prepared = false;
	}
	sr->enabled = false;
	return ret;
}

static int apq8064_slim_sync_stop(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream)
{
	struct apq8064_slim_runtime *sr = substream->runtime->private_data;

	/* ALSA invokes sync_stop outside the stream lock. */
	cancel_work_sync(&sr->period_work);
	cancel_work_sync(&sr->elapsed_work);
	return 0;
}

static const struct snd_pcm_hardware apq8064_slim_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 1,
	/* Two queued periods cover host workqueue scheduling latency. */
	.period_bytes_min = 32768,
	.period_bytes_max = 32768,
	.periods_min = 2,
	.periods_max = 2,
	.buffer_bytes_max = 65536,
};

static int apq8064_slim_open(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec = snd_soc_rtd_to_codec(rtd, 0);
	struct slim_device *sdev = to_slim_device(codec->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct apq8064_slim_runtime *sr;
	int ret;

	sr = kzalloc_obj(*sr);
	if (!sr)
		return -ENOMEM;
	sr->substream = substream;
	sr->ctrl = sdev->ctrl;
	sr->channel = 138;
	INIT_WORK(&sr->period_work, apq8064_slim_period_work);
	INIT_WORK(&sr->elapsed_work, apq8064_slim_elapsed_work);
	atomic_set(&sr->pending, 0);

	runtime->hw = apq8064_slim_hardware;
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		kfree(sr);
		return ret;
	}
	runtime->private_data = sr;
	return 0;
}

static int apq8064_slim_close(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	struct apq8064_slim_runtime *sr = substream->runtime->private_data;

	apq8064_slim_stop(sr);
	apq8064_slim_sync_stop(component, substream);
	if (sr->stream)
		slim_stream_free(sr->stream);
	substream->runtime->private_data = NULL;
	kfree(sr);
	return 0;
}

static int apq8064_slim_hw_params(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct slim_device *sdev = to_slim_device(snd_soc_rtd_to_codec(rtd, 0)->dev);
	struct apq8064_slim_runtime *sr = substream->runtime->private_data;

	if (sr->stream)
		return -EBUSY;
	sr->period_bytes = params_period_bytes(params);
	sr->buffer_bytes = params_buffer_bytes(params);
	sr->config.rate = params_rate(params);
	sr->config.bps = params_width(params);
	sr->config.ch_count = 1;
	sr->config.chs = &sr->channel;
	sr->config.port_mask = BIT(10);
	sr->config.direction = SNDRV_PCM_STREAM_PLAYBACK;

	sr->stream = slim_stream_allocate(sdev, "apq8064-playback");
	if (IS_ERR(sr->stream)) {
		int ret = PTR_ERR(sr->stream);

		sr->stream = NULL;
		return ret;
	}
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int apq8064_slim_hw_free(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct apq8064_slim_runtime *sr = substream->runtime->private_data;
	int ret;

	ret = apq8064_slim_stop(sr);
	apq8064_slim_sync_stop(component, substream);
	if (sr->stream) {
		slim_stream_free(sr->stream);
		sr->stream = NULL;
	}
	snd_pcm_set_runtime_buffer(substream, NULL);
	return ret;
}

static int apq8064_slim_prepare(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct apq8064_slim_runtime *sr = substream->runtime->private_data;
	int ret;

	if (!sr->stream)
		return -EINVAL;
	if (sr->prepared) {
		ret = apq8064_slim_stop(sr);
		if (ret)
			return ret;
	}
	ret = slim_stream_prepare(sr->stream, &sr->config);
	if (ret)
		return ret;
	sr->prepared = true;
	return 0;
}

static int apq8064_slim_trigger(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream, int cmd)
{
	struct apq8064_slim_runtime *sr = substream->runtime->private_data;
	unsigned int offset;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (!sr->prepared)
			return -EINVAL;
		if (sr->enabled)
			return 0;
		WRITE_ONCE(sr->pos, 0);
		WRITE_ONCE(sr->error, 0);
		sr->submit_pos = 0;
		atomic_set(&sr->pending, 0);
		WRITE_ONCE(sr->running, true);

		for (offset = 0; offset < sr->buffer_bytes;
		     offset += sr->period_bytes) {
			ret = apq8064_slim_submit(sr, offset);
			if (ret)
				goto stop;
		}
		ret = qcom_slim_port_enable_hw(sr->ctrl, 0);
		if (ret)
			goto stop;
		ret = slim_stream_enable(sr->stream);
		if (ret)
			goto stop;
		sr->enabled = true;
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return apq8064_slim_stop(sr);
	default:
		return -EINVAL;
	}

stop:
	apq8064_slim_stop(sr);
	return ret;
}

static snd_pcm_uframes_t
apq8064_slim_pointer(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream)
{
	struct apq8064_slim_runtime *sr = substream->runtime->private_data;

	return bytes_to_frames(substream->runtime, READ_ONCE(sr->pos));
}

static int apq8064_slim_mmap(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream,
			    struct vm_area_struct *vma)
{
	return snd_pcm_lib_default_mmap(substream, vma);
}

static int apq8064_slim_pcm_new(struct snd_soc_component *component,
			       struct snd_soc_pcm_runtime *rtd)
{
	struct slim_device *sdev = to_slim_device(snd_soc_rtd_to_codec(rtd, 0)->dev);

	return snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
					      sdev->dev.parent, 65536, 65536);
}

const struct snd_soc_component_driver apq8064_slim_component = {
	.name = "apq8064-slim-pcm",
	.open = apq8064_slim_open,
	.close = apq8064_slim_close,
	.hw_params = apq8064_slim_hw_params,
	.hw_free = apq8064_slim_hw_free,
	.prepare = apq8064_slim_prepare,
	.trigger = apq8064_slim_trigger,
	.sync_stop = apq8064_slim_sync_stop,
	.pointer = apq8064_slim_pointer,
	.mmap = apq8064_slim_mmap,
	.pcm_new = apq8064_slim_pcm_new,
};

struct snd_soc_dai_driver apq8064_slim_dai = {
	.name = "apq8064-slim",
	.playback = {
		.stream_name = "CPU Playback",
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 1,
	},
};
