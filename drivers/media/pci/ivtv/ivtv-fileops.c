// SPDX-License-Identifier: GPL-2.0-or-later
/*
    file operation functions
    Copyright (C) 2003-2004  Kevin Thayer <nufan_wfk at yahoo.com>
    Copyright (C) 2004  Chris Kennedy <c@groovy.org>
    Copyright (C) 2005-2007  Hans Verkuil <hverkuil@xs4all.nl>

 */

#include "ivtv-driver.h"
#include "ivtv-fileops.h"
#include "ivtv-i2c.h"
#include "ivtv-queue.h"
#include "ivtv-udma.h"
#include "ivtv-irq.h"
#include "ivtv-vbi.h"
#include "ivtv-mailbox.h"
#include "ivtv-routing.h"
#include "ivtv-streams.h"
#include "ivtv-yuv.h"
#include "ivtv-ioctl.h"
#include "ivtv-cards.h"
#include "ivtv-firmware.h"
#include <media/v4l2-event.h>
#include <media/i2c/saa7115.h>

/* This function tries to claim the stream for a specific file descriptor.
   If no one else is using this stream then the stream is claimed and
   associated VBI streams are also automatically claimed.
   Possible error returns: -EBUSY if someone else has claimed
   the stream or 0 on success. */
int ivtv_claim_stream(struct ivtv_open_id *id, int type)
{
	struct ivtv *itv = id->itv;
	struct ivtv_stream *s = &itv->streams[type];
	struct ivtv_stream *s_vbi;
	int vbi_type;

	if (test_and_set_bit(IVTV_F_S_CLAIMED, &s->s_flags)) {
		/* someone already claimed this stream */
		if (s->fh == &id->fh) {
			/* yes, this file descriptor did. So that's OK. */
			return 0;
		}
		if (s->fh == NULL && (type == IVTV_DEC_STREAM_TYPE_VBI ||
					 type == IVTV_ENC_STREAM_TYPE_VBI)) {
			/* VBI is handled already internally, now also assign
			   the file descriptor to this stream for external
			   reading of the stream. */
			s->fh = &id->fh;
			IVTV_DEBUG_INFO("Start Read VBI\n");
			return 0;
		}
		/* someone else is using this stream already */
		IVTV_DEBUG_INFO("Stream %d is busy\n", type);
		return -EBUSY;
	}
	s->fh = &id->fh;
	if (type == IVTV_DEC_STREAM_TYPE_VBI) {
		/* Enable reinsertion interrupt */
		ivtv_clear_irq_mask(itv, IVTV_IRQ_DEC_VBI_RE_INSERT);
	}

	/* IVTV_DEC_STREAM_TYPE_MPG needs to claim IVTV_DEC_STREAM_TYPE_VBI,
	   IVTV_ENC_STREAM_TYPE_MPG needs to claim IVTV_ENC_STREAM_TYPE_VBI
	   (provided VBI insertion is on and sliced VBI is selected), for all
	   other streams we're done */
	if (type == IVTV_DEC_STREAM_TYPE_MPG) {
		vbi_type = IVTV_DEC_STREAM_TYPE_VBI;
	} else if (type == IVTV_ENC_STREAM_TYPE_MPG &&
		   itv->vbi.insert_mpeg && !ivtv_raw_vbi(itv)) {
		vbi_type = IVTV_ENC_STREAM_TYPE_VBI;
	} else {
		return 0;
	}
	s_vbi = &itv->streams[vbi_type];

	if (!test_and_set_bit(IVTV_F_S_CLAIMED, &s_vbi->s_flags)) {
		/* Enable reinsertion interrupt */
		if (vbi_type == IVTV_DEC_STREAM_TYPE_VBI)
			ivtv_clear_irq_mask(itv, IVTV_IRQ_DEC_VBI_RE_INSERT);
	}
	/* mark that it is used internally */
	set_bit(IVTV_F_S_INTERNAL_USE, &s_vbi->s_flags);
	return 0;
}
EXPORT_SYMBOL(ivtv_claim_stream);

/* This function releases a previously claimed stream. It will take into
   account associated VBI streams. */
void ivtv_release_stream(struct ivtv_stream *s)
{
	struct ivtv *itv = s->itv;
	struct ivtv_stream *s_vbi;

	s->fh = NULL;
	if ((s->type == IVTV_DEC_STREAM_TYPE_VBI || s->type == IVTV_ENC_STREAM_TYPE_VBI) &&
		test_bit(IVTV_F_S_INTERNAL_USE, &s->s_flags)) {
		/* this stream is still in use internally */
		return;
	}
	if (!test_and_clear_bit(IVTV_F_S_CLAIMED, &s->s_flags)) {
		IVTV_DEBUG_WARN("Release stream %s not in use!\n", s->name);
		return;
	}

	ivtv_flush_queues(s);

	/* disable reinsertion interrupt */
	if (s->type == IVTV_DEC_STREAM_TYPE_VBI)
		ivtv_set_irq_mask(itv, IVTV_IRQ_DEC_VBI_RE_INSERT);

	/* IVTV_DEC_STREAM_TYPE_MPG needs to release IVTV_DEC_STREAM_TYPE_VBI,
	   IVTV_ENC_STREAM_TYPE_MPG needs to release IVTV_ENC_STREAM_TYPE_VBI,
	   for all other streams we're done */
	if (s->type == IVTV_DEC_STREAM_TYPE_MPG)
		s_vbi = &itv->streams[IVTV_DEC_STREAM_TYPE_VBI];
	else if (s->type == IVTV_ENC_STREAM_TYPE_MPG)
		s_vbi = &itv->streams[IVTV_ENC_STREAM_TYPE_VBI];
	else
		return;

	/* clear internal use flag */
	if (!test_and_clear_bit(IVTV_F_S_INTERNAL_USE, &s_vbi->s_flags)) {
		/* was already cleared */
		return;
	}
	if (s_vbi->fh) {
		/* VBI stream still claimed by a file descriptor */
		return;
	}
	/* disable reinsertion interrupt */
	if (s_vbi->type == IVTV_DEC_STREAM_TYPE_VBI)
		ivtv_set_irq_mask(itv, IVTV_IRQ_DEC_VBI_RE_INSERT);
	clear_bit(IVTV_F_S_CLAIMED, &s_vbi->s_flags);
	ivtv_flush_queues(s_vbi);
}
EXPORT_SYMBOL(ivtv_release_stream);

static void ivtv_dualwatch(struct ivtv *itv)
{
	struct v4l2_tuner vt;
	u32 new_stereo_mode;
	const u32 dual = 0x02;

	new_stereo_mode = v4l2_ctrl_g_ctrl(itv->cxhdl.audio_mode);
	memset(&vt, 0, sizeof(vt));
	ivtv_call_all(itv, tuner, g_tuner, &vt);
	if (vt.audmode == V4L2_TUNER_MODE_LANG1_LANG2 && (vt.rxsubchans & V4L2_TUNER_SUB_LANG2))
		new_stereo_mode = dual;

	if (new_stereo_mode == itv->dualwatch_stereo_mode)
		return;

	IVTV_DEBUG_INFO("dualwatch: change stereo flag from 0x%x to 0x%x.\n",
			   itv->dualwatch_stereo_mode, new_stereo_mode);
	if (v4l2_ctrl_s_ctrl(itv->cxhdl.audio_mode, new_stereo_mode))
		IVTV_DEBUG_INFO("dualwatch: changing stereo flag failed\n");
}

static void ivtv_update_pgm_info(struct ivtv *itv)
{
	u32 wr_idx = (read_enc(itv->pgm_info_offset) - itv->pgm_info_offset - 4) / 24;
	int cnt;
	int i = 0;

	if (wr_idx >= itv->pgm_info_num) {
		IVTV_DEBUG_WARN("Invalid PGM index %d (>= %d)\n", wr_idx, itv->pgm_info_num);
		return;
	}
	cnt = (wr_idx + itv->pgm_info_num - itv->pgm_info_write_idx) % itv->pgm_info_num;
	while (i < cnt) {
		int idx = (itv->pgm_info_write_idx + i) % itv->pgm_info_num;
		struct v4l2_enc_idx_entry *e = itv->pgm_info + idx;
		u32 addr = itv->pgm_info_offset + 4 + idx * 24;
		const int mapping[8] = { -1, V4L2_ENC_IDX_FRAME_I, V4L2_ENC_IDX_FRAME_P, -1,
			V4L2_ENC_IDX_FRAME_B, -1, -1, -1 };
					// 1=I, 2=P, 4=B

		e->offset = read_enc(addr + 4) + ((u64)read_enc(addr + 8) << 32);
		if (e->offset > itv->mpg_data_received) {
			break;
		}
		e->offset += itv->vbi_data_inserted;
		e->length = read_enc(addr);
		e->pts = read_enc(addr + 16) + ((u64)(read_enc(addr + 20) & 1) << 32);
		e->flags = mapping[read_enc(addr + 12) & 7];
		i++;
	}
	itv->pgm_info_write_idx = (itv->pgm_info_write_idx + i) % itv->pgm_info_num;
}

static struct ivtv_buffer *ivtv_get_buffer(struct ivtv_stream *s, int non_block, int *err)
{
	struct ivtv *itv = s->itv;
	struct ivtv_stream *s_vbi = &itv->streams[IVTV_ENC_STREAM_TYPE_VBI];
	struct ivtv_buffer *buf;
	DEFINE_WAIT(wait);

	*err = 0;
	while (1) {
		if (s->type == IVTV_ENC_STREAM_TYPE_MPG) {
			/* Process pending program info updates and pending VBI data */
			ivtv_update_pgm_info(itv);

			if (time_after(jiffies,
				       itv->dualwatch_jiffies +
				       msecs_to_jiffies(1000))) {
				itv->dualwatch_jiffies = jiffies;
				ivtv_dualwatch(itv);
			}

			if (test_bit(IVTV_F_S_INTERNAL_USE, &s_vbi->s_flags) &&
			    !test_bit(IVTV_F_S_APPL_IO, &s_vbi->s_flags)) {
				while ((buf = ivtv_dequeue(s_vbi, &s_vbi->q_full))) {
					/* byteswap and process VBI data */
					ivtv_process_vbi_data(itv, buf, s_vbi->dma_pts, s_vbi->type);
					ivtv_enqueue(s_vbi, buf, &s_vbi->q_free);
				}
			}
			buf = &itv->vbi.sliced_mpeg_buf;
			if (buf->readpos != buf->bytesused) {
				return buf;
			}
		}

		/* do we have leftover data? */
		buf = ivtv_dequeue(s, &s->q_io);
		if (buf)
			return buf;

		/* do we have new data? */
		buf = ivtv_dequeue(s, &s->q_full);
		if (buf) {
			if ((buf->b_flags & IVTV_F_B_NEED_BUF_SWAP) == 0)
				return buf;
			buf->b_flags &= ~IVTV_F_B_NEED_BUF_SWAP;
			if (s->type == IVTV_ENC_STREAM_TYPE_MPG)
				/* byteswap MPG data */
				ivtv_buf_swap(buf);
			else if (s->type != IVTV_DEC_STREAM_TYPE_VBI) {
				/* byteswap and process VBI data */
				ivtv_process_vbi_data(itv, buf, s->dma_pts, s->type);
			}
			return buf;
		}

		/* return if end of stream */
		if (s->type != IVTV_DEC_STREAM_TYPE_VBI && !test_bit(IVTV_F_S_STREAMING, &s->s_flags)) {
			IVTV_DEBUG_INFO("EOS %s\n", s->name);
			return NULL;
		}

		/* return if file was opened with O_NONBLOCK */
		if (non_block) {
			*err = -EAGAIN;
			return NULL;
		}

		/* wait for more data to arrive */
		mutex_unlock(&itv->serialize_lock);
		prepare_to_wait(&s->waitq, &wait, TASK_INTERRUPTIBLE);
		/* New buffers might have become available before we were added to the waitqueue */
		if (!s->q_full.buffers)
			schedule();
		finish_wait(&s->waitq, &wait);
		mutex_lock(&itv->serialize_lock);
		if (signal_pending(current)) {
			/* return if a signal was received */
			IVTV_DEBUG_INFO("User stopped %s\n", s->name);
			*err = -EINTR;
			return NULL;
		}
	}
}

static void ivtv_setup_sliced_vbi_buf(struct ivtv *itv)
{
	int idx = itv->vbi.inserted_frame % IVTV_VBI_FRAMES;

	itv->vbi.sliced_mpeg_buf.buf = itv->vbi.sliced_mpeg_data[idx];
	itv->vbi.sliced_mpeg_buf.bytesused = itv->vbi.sliced_mpeg_size[idx];
	itv->vbi.sliced_mpeg_buf.readpos = 0;
}

static size_t ivtv_copy_buf_to_user(struct ivtv_stream *s, struct ivtv_buffer *buf,
		char __user *ubuf, size_t ucount)
{
	struct ivtv *itv = s->itv;
	size_t len = buf->bytesused - buf->readpos;

	if (len > ucount) len = ucount;
	if (itv->vbi.insert_mpeg && s->type == IVTV_ENC_STREAM_TYPE_MPG &&
	    !ivtv_raw_vbi(itv) && buf != &itv->vbi.sliced_mpeg_buf) {
		const char *start = buf->buf + buf->readpos;
		const char *p = start + 1;
		const u8 *q;
		u8 ch = itv->search_pack_header ? 0xba : 0xe0;
		int stuffing, i;

		while (start + len > p && (q = memchr(p, 0, start + len - p))) {
			p = q + 1;
			if ((char *)q + 15 >= buf->buf + buf->bytesused ||
			    q[1] != 0 || q[2] != 1 || q[3] != ch) {
				continue;
			}
			if (!itv->search_pack_header) {
				if ((q[6] & 0xc0) != 0x80)
					continue;
				if (((q[7] & 0xc0) == 0x80 && (q[9] & 0xf0) == 0x20) ||
				    ((q[7] & 0xc0) == 0xc0 && (q[9] & 0xf0) == 0x30)) {
					ch = 0xba;
					itv->search_pack_header = 1;
					p = q + 9;
				}
				continue;
			}
			stuffing = q[13] & 7;
			/* all stuffing bytes must be 0xff */
			for (i = 0; i < stuffing; i++)
				if (q[14 + i] != 0xff)
					break;
			if (i == stuffing && (q[4] & 0xc4) == 0x44 && (q[12] & 3) == 3 &&
					q[14 + stuffing] == 0 && q[15 + stuffing] == 0 &&
					q[16 + stuffing] == 1) {
				itv->search_pack_header = 0;
				len = (char *)q - start;
				ivtv_setup_sliced_vbi_buf(itv);
				break;
			}
		}
	}
	if (copy_to_user(ubuf, (u8 *)buf->buf + buf->readpos, len)) {
		IVTV_DEBUG_WARN("copy %zd bytes to user failed for %s\n", len, s->name);
		return -EFAULT;
	}
	/*IVTV_INFO("copied %lld %d %d %d %d %d vbi %d\n", itv->mpg_data_received, len, ucount,
			buf->readpos, buf->bytesused, buf->bytesused - buf->readpos - len,
			buf == &itv->vbi.sliced_mpeg_buf); */
	buf->readpos += len;
	if (s->type == IVTV_ENC_STREAM_TYPE_MPG && buf != &itv->vbi.sliced_mpeg_buf)
		itv->mpg_data_received += len;
	return len;
}

static ssize_t ivtv_read(struct ivtv_stream *s, char __user *ubuf, size_t tot_count, int non_block)
{
	struct ivtv *itv = s->itv;
	size_t tot_written = 0;
	int single_frame = 0;

	if (atomic_read(&itv->capturing) == 0 && s->fh == NULL) {
		/* shouldn't happen */
		IVTV_DEBUG_WARN("Stream %s not initialized before read\n", s->name);
		return -EIO;
	}

	/* Each VBI buffer is one frame, the v4l2 API says that for VBI the frames should
	   arrive one-by-one, so make sure we never outp? */
		buf = ivtv_dequeudst;
	u32 size;
};

struct ivtv_user_dma {
	struct mutex lock;
	int page_count;
	struct page *map[IVTV_DMA_SG_OSD_ENT];
	/* Needed when dealing with highmem userspace buffers */
	struct page *bouncemap[IVTV_DMA_SG_OSD_ENT];

	/* Base Dev SG Array for cx23415/6 */
	struct ivtv_sg_element SGarray[IVTV_DMA_SG_OSD_ENT];
	dma_addr_t SG_handle;
	int SG_length;

	/* SG List of Buffers */
	struct scatterlist SGlist[IVTV_DMA_SG_OSD_ENT];
};

struct ivtv_dma_page_info {
	unsigned long uaddr;
	unsigned long first;
	unsigned long last;
	unsigned int offset;
	unsigned int tail;
	int page_count;
};

struct ivtv_buffer {
	struct list_head list;
	dma_addr_t dma_handle;
	unsigned short b_flags;
	unsigned short dma_xfer_cnt;
	char *buf;
	u32 bytesused;
	u32 readpos;
};

struct ivtv_queue {
	struct list_head list;          /* the list of buffers in this queue */
	u32 buffers;                    /* number of buffers in this queue */
	u32 length;                     /* total number of bytes of available buffer space */
	u32 bytesused;                  /* total number of bytes used in this queue */
};

struct ivtv;				/* forward reference */

struct ivtv_stream {
	/* These first four fields are always set, even if the stream
	   is not actually created. */
	struct video_device vdev;	/* vdev.v4l2_dev is NULL if there is no device */
	struct ivtv *itv;		/* for ease of use */
	const char *name;		/* name of the stream */
	int type;			/* stream type */

	struct v4l2_fh *fh;		/* pointer to the streaming filehandle */
	spinlock_t qlock;		/* locks access to the queues */
	unsigned long s_flags;		/* status flags, see above */
	int dma;			/* can be PCI_DMA_TODEVICE, PCI_DMA_FROMDEVICE or PCI_DMA_NONE */
	u32 pending_offset;
	u32 pending_backup;
	u64 pending_pts;

	u32 dma_offset;
	u32 dma_backup;
	u64 dma_pts;

	int subtype;
	wait_queue_head_t waitq;
	u32 dma_last_offset;

	/* Buffer Stats */
	u32 buffers;
	u32 buf_size;
	u32 buffers_stolen;

	/* Buffer Queues */
	struct ivtv_queue q_free;	/* free buffers */
	struct ivtv_queue q_full;	/* full buffers */
	struct ivtv_queue q_io;		/* waiting for I/O */
	struct ivtv_queue q_dma;	/* waiting for DMA */
	struct ivtv_queue q_predma;	/* waiting for DMA */

	/* DMA xfer counter, buffers belonging to the same DMA
	   xfer will have the same dma_xfer_cnt. */
	u16 dma_xfer_cnt;

	/* Base Dev SG Array for cx23415/6 */
	struct ivtv_sg_host_element *sg_pending;
	struct ivtv_sg_host_element *sg_processing;
	struct ivtv_sg_element *sg_dma;
	dma_addr_t sg_handle;
	int sg_pending_size;
	int sg_processing_size;
	int sg_processed;

	/* SG List of Buffers */
	struct scatterlist *SGlist;
};

struct ivtv_open_id {
	struct v4l2_fh fh;
	int type;                       /* stream type */
	int yuv_frames;                 /* 1: started OUT_UDMA_YUV output mode */
	struct ivtv *itv;
};

static inline struct ivtv_open_id *fh2id(struct v4l2_fh *fh)
{
	return container_of(fh, struct ivtv_open_id, fh);
}

struct yuv_frame_info
{
	u32 update;
	s32 src_x;
	s32 src_y;
	u32 src_w;
	u32 src_h;
	s32 dst_x;
	s32 dst_y;
	u32 dst_w;
	u32 dst_h;
	s32 pan_x;
	s32 pan_y;
	u32 vis_w;
	u32 vis_h;
	u32 interlaced_y;
	u32 interlaced_uv;
	s32 tru_x;
	u32 tru_w;
	u32 tru_h;
	u32 offset_y;
	s32 lace_mode;
	u32 sync_field;
	u32 delay;
	u32 interlaced;
};

#define IVTV_YUV_MODE_INTERLACED	0x00
#define IVTV_YUV_MODE_PROGRESSIVE	0x01
#define IVTV_YUV_MODE_AUTO		0x02
#define IVTV_YUV_MODE_MASK		0x03

#define IVTV_YUV_SYNC_EVEN		0x00
#define IVTV_YUV_SYNC_ODD		0x04
#define IVTV_YUV_SYNC_MASK		0x04

#define IVTV_YUV_BUFFERS 8

struct yuv_playback_info
{
	u32 reg_2834;
	u32 reg_2838;
	u32 reg_283c;
	u32 reg_2840;
	u32 reg_2844;
	u32 reg_2848;
	u32 reg_2854;
	u32 reg_285c;
	u32 reg_2864;

	u32 reg_2870;
	u32 reg_2874;
	u32 reg_2890;
	u32 reg_2898;
	u32 reg_289c;

	u32 reg_2918;
	u32 reg_291c;
	u32 reg_2920;
	u32 reg_2924;
	u32 reg_2928;
	u32 reg_292c;
	u32 reg_2930;

	u32 reg_2934;

	u32 reg_2938;
	u32 reg_293c;
	u32 reg_2940;
	u32 reg_2944;
	u32 reg_2948;
	u32 reg_294c;
	u32 reg_2950;
	u32 reg_2954;
	u32 reg_2958;
	u32 reg_295c;
	u32 reg_2960;
	u32 reg_2964;
	u32 reg_2968;
	u32 reg_296c;

	u32 reg_2970;

	int v_filter_1;
	int v_filter_2;
	int h_filter;

	u8 track_osd; /* Should yuv output track the OSD size & position */

	u32 osd_x_offset;
	u32 osd_y_offset;

	u32 osd_x_pan;
	u32 osd_y_pan;

	u32 osd_vis_w;
	u32 osd_vis_h;

	u32 osd_full_w;
	u32 osd_full_h;

	int decode_height;

	int lace_mode;
	int lace_threshold;
	int lace_sync_field;

	atomic_t next_dma_frame;
	atomic_t next_fill_frame;

	u32 yuv_forced_update;
	int update_frame;

	u8 fields_lapsed;   /* Counter used when delaying a frame */

	struct yuv_frame_info new_frame_info[IVTV_YUV_BUFFERS];
	struct yuv_frame_info old_frame_info;
	struct yuv_frame_info old_frame_info_args;

	void *blanking_ptr;
	dma_addr_t blanking_dmaptr;

	int stream_size;

	u8 draw_frame; /* PVR350 buffer to draw into */
	u8 max_frames_buffered; /* Maximum number of frames to buffer */

	struct v4l2_rect main_rect;
	u32 v4l2_src_w;
	u32 v4l2_src_h;

	u8 running; /* Have any frames been displayed */
};

#define IVTV_VBI_FRAMES 32

/* VBI data */
struct vbi_cc {
	u8 odd[2];	/* two-byte payload of odd field */
	u8 even[2];	/* two-byte payload of even field */;
};

struct vbi_vps {
	u8 data[5];	/* five-byte VPS payload */
};

struct vbi_info {
	/* VBI general data, does not change during streaming */

	u32 raw_decoder_line_size;              /* raw VBI line size from digitizer */
	u8 raw_decoder_sav_odd_field;           /* raw VBI Start Active Video digitizer code of odd field */
	u8 raw_decoder_sav_even_field;          /* raw VBI Start Active Video digitizer code of even field */
	u32 sliced_decoder_line_size;           /* sliced VBI line size from digitizer */
	u8 sliced_decoder_sav_odd_field;        /* sliced VBI Start Active Video digitizer code of odd field */
	u8 sliced_decoder_sav_even_field;       /* sliced VBI Start Active Video digitizer code of even field */

	u32 start[2];				/* start of first VBI line in the odd/even fields */
	u32 count;				/* number of VBI lines per field */
	u32 raw_size;				/* size of raw VBI line from the digitizer */
	u32 sliced_size;			/* size of sliced VBI line from the digitizer */

	u32 dec_start;				/* start in decoder memory of VBI re-insertion buffers */
	u32 enc_start;				/* start in encoder memory of VBI capture buffers */
	u32 enc_size;				/* size of VBI capture area */
	int fpi;				/* number of VBI frames per interrupt */

	struct v4l2_format in;			/* current VBI capture format */
	struct v4l2_sliced_vbi_format *sliced_in; /* convenience pointer to sliced struct in vbi.in union */
	int insert_mpeg;			/* if non-zero, then embed VBI data in MPEG stream */

	/* Raw VBI compatibility hack */

	u32 frame;				/* frame counter hack needed for backwards compatibility
						   of old VBI software */

	/* Sliced VBI output data */

	struct vbi_cc cc_payload[256];		/* sliced VBI CC payload array: it is an array to
						   prevent dropping CC data if they couldn't be
						   processed fast enough */
	int cc_payload_idx;			/* index in cc_payload */
	u8 cc_missing_cnt;			/* counts number of frames without CC for passthrough mode */
	int wss_payload;			/* sliced VBI WSS payload */
	u8 wss_missing_cnt;			/* counts number of frames without WSS for passthrough mode */
	struct vbi_vps vps_payload;		/* sliced VBI VPS payload */

	/* Sliced VBI capture data */

	struct v4l2_sliced_vbi_data sliced_data[36];	/* sliced VBI storage for VBI encoder stream */
	struct v4l2_sliced_vbi_data sliced_dec_data[36];/* sliced VBI storage for VBI decoder stream */

	/* VBI Embedding data */

	/* Buffer for VBI data inserted into MPEG stream.
	   The first byte is a dummy byte that's never used.
	   The next 16 bytes contain the MPEG header for the VBI data,
	   the remainder is the actual VBI data.
	   The max size accepted by the MPEG VBI reinsertion turns out
	   to be 1552 bytes, which happens to be 4 + (1 + 42) * (2 * 18) bytes,
	   where 4 is a four byte header, 42 is the max sliced VBI payload, 1 is
	   a single line header byte and 2 * 18 is the number of VBI lines per frame.

	   However, it seems that the data must be 1K aligned, so we have to
	   pad the data until the 1 or 2 K boundary.

	   This pointer array will allocate 2049 bytes to store each VBI frame. */
	u8 *sliced_mpeg_data[IVTV_VBI_FRAMES];
	u32 sliced_mpeg_size[IVTV_VBI_FRAMES];
	struct ivtv_buffer sliced_mpeg_buf;	/* temporary buffer holding data from sliced_mpeg_data */
	u32 inserted_frame;			/* index in sliced_mpeg_size of next sliced data
						   to be inserted in the MPEG stream */
};

/* forward declaration of struct defined in ivtv-cards.h */
struct ivtv_card;

/* Struct to hold info about ivtv cards */
struct ivtv {
	/* General fixed card data */
	struct pci_dev *pdev;		/* PCI device */
	const struct ivtv_card *card;	/* card information */
	const char *card_name;          /* full name of the card */
	const struct ivtv_card_tuner_i2c *card_i2c; /* i2c addresses to probe for tuner */
	u8 has_cx23415;			/* 1 if it is a cx23415 based card, 0 for cx23416 */
	u8 pvr150_workaround;           /* 1 if the cx25840 needs to workaround a PVR150 bug */
	u8 nof_inputs;			/* number of video inputs */
	u8 nof_audio_inputs;		/* number of audio inputs */
	u32 v4l2_cap;			/* V4L2 capabilities of card */
	u32 hw_flags;			/* hardware description of the board */
	v4l2_std_id tuner_std;		/* the norm of the card's tuner (fixed) */
	struct v4l2_subdev *sd_video;	/* controlling video decoder subdev */
	bool sd_video_is_streaming;	/* is video already streaming? */
	struct v4l2_subdev *sd_audio;	/* controlling audio subdev */
	struct v4l2_subdev *sd_muxer;	/* controlling audio muxer subdev */
	resource_size_t base_addr;      /* PCI resource base address */
	volatile void __iomem *enc_mem; /* pointer to mapped encoder memory */
	volatile void __iomem *dec_mem; /* pointer to mapped decoder memory */
	volatile void __iomem *reg_mem; /* pointer to mapped registers */
	struct ivtv_options options;	/* user options */

	struct v4l2_device v4l2_dev;
	struct cx2341x_handler cxhdl;
	struct {
		/* PTS/Frame count control cluster */
		struct v4l2_ctrl *ctrl_pts;
		struct v4l2_ctrl *ctrl_frame;
	};
	struct {
		/* Audio Playback control cluster */
		struct v4l2_ctrl *ctrl_audio_playback;
		struct v4l2_ctrl *ctrl_audio_multilingual_playback;
	};
	struct v4l2_ctrl_handler hdl_gpio;
	struct v4l2_subdev sd_gpio;	/* GPIO sub-device */
	u16 instance;

	/* High-level state info */
	unsigned long i_flags;          /* global ivtv flags */
	u8 is_50hz;                     /* 1 if the current capture standard is 50 Hz */
	u8 is_60hz                      /* 1 if the current capture standard is 60 Hz */;
	u8 is_out_50hz                  /* 1 if the current TV output standard is 50 Hz */;
	u8 is_out_60hz                  /* 1 if the current TV output standard is 60 Hz */;
	int output_mode;                /* decoder output mode: NONE, MPG, YUV, UDMA YUV, passthrough */
	u32 audio_input;                /* current audio input */
	u32 active_input;               /* current video input */
	u32 active_output;              /* current video output */
	v4l2_std_id std;                /* current capture TV standard */
	v4l2_std_id std_out;            /* current TV output standard */
	u8 audio_stereo_mode;           /* decoder setting how to handle stereo MPEG audio */
	u8 audio_bilingual_mode;        /* decoder setting how to handle bilingual MPEG audio */

	/* Locking */
	spinlock_t lock;                /* lock access to this struct */
	struct mutex serialize_lock;    /* mutex used to serialize open/close/start/stop/ioctl operations */

	/* Streams */
	int stream_buf_size[IVTV_MAX_STREAMS];          /* stream buffer size */
	struct ivtv_stream streams[IVTV_MAX_STREAMS];	/* stream data */
	atomic_t capturing;		/* count number of active capture streams */
	atomic_t decoding;		/* count number of active decoding streams */

	/* ALSA interface for PCM capture stream */
	struct snd_ivtv_card *alsa;
	void (*pcm_announce_callback)(struct snd_ivtv_card *card, u8 *pcm_data,
				      size_t num_bytes);

	/* Used for ivtv-alsa module loading */
	struct work_struct request_module_wk;

	/* Interrupts & DMA */
	u32 irqmask;                    /* active interrupts */
	u32 irq_rr_idx;                 /* round-robin stream index */
	struct kthread_worker irq_worker;		/* kthread worker for PIO/YUV/VBI actions */
	struct task_struct *irq_worker_task;		/* task for irq_worker */
	struct kthread_work irq_work;	/* kthread work entry */
	spinlock_t dma_reg_lock;        /* lock access to DMA engine registers */
	int cur_dma_stream;		/* index of current stream doing DMA (-1 if none) */
	int cur_pio_stream;		/* index of current stream doing PIO (-1 if none) */
	u32 dma_data_req_offset;        /* store offset in decoder memory of current DMA request */
	u32 dma_data_req_size;          /* store size of current DMA request */
	int dma_retries;                /* current DMA retry attempt */
	struct ivtv_user_dma udma;      /* user based DMA for OSD */
	struct timer_list dma_timer;    /* timer used to catch unfinished DMAs */
	u32 last_vsync_field;           /* last seen vsync field */
	wait_queue_head_t dma_waitq;    /* wake up when the current DMA is finished */
	wait_queue_head_t eos_waitq;    /* wake up when EOS arrives */
	wait_queue_head_t event_waitq;  /* wake up when the next decoder event arrives */
	wait_queue_head_t vsync_waitq;  /* wake up when the next decoder vsync arrives */


	/* Mailbox */
	struct ivtv_mailbox_data enc_mbox;              /* encoder mailboxes */
	struct ivtv_mailbox_data dec_mbox;              /* decoder mailboxes */
	struct ivtv_api_cache api_cache[256];		/* cached API commands */


	/* I2C */
	struct i2c_adapter i2c_adap;
	struct i2c_algo_bit_data i2c_algo;
	struct i2c_client i2c_client;
	int i2c_state;                  /* i2c bit state */
	struct mutex i2c_bus_lock;      /* lock i2c bus */

	struct IR_i2c_init_data ir_i2c_init_data;

	/* Program Index information */
	u32 pgm_info_offset;            /* start of pgm info in encoder memory */
	u32 pgm_info_num;               /* number of elements in the pgm cyclic buffer in encoder memory */
	u32 pgm_info_write_idx;         /* last index written by the card that was transferred to pgm_info[] */
	u32 pgm_info_read_idx;          /* last index in pgm_info read by the application */
	struct v4l2_enc_idx_entry pgm_info[IVTV_MAX_PGM_INDEX]; /* filled from the pgm cyclic buffer on the card */


	/* Miscellaneous */
	u32 open_id;			/* incremented each time an open occurs, is >= 1 */
	int search_pack_header;         /* 1 if ivtv_copy_buf_to_user() is scanning for a pack header (0xba) */
	int speed;                      /* current playback speed setting */
	u8 speed_mute_audio;            /* 1 if audio should be muted when fast forward */
	u64 mpg_data_received;          /* number of bytes received from the MPEG stream */
	u64 vbi_data_inserted;          /* number of VBI bytes inserted into the MPEG stream */
	u32 last_dec_timing[3];         /* cache last retrieved pts/scr/frame values */
	unsigned long dualwatch_jiffies;/* jiffies value of the previous dualwatch check */
	u32 dualwatch_stereo_mode;      /* current detected dualwatch stereo mode */


	/* VBI state info */
	struct vbi_info vbi;            /* VBI-specific data */


	/* YUV playback */
	struct yuv_playback_info yuv_info;              /* YUV playback data */


	/* OSD support */
	unsigned long osd_video_pbase;
	int osd_global_alpha_state;     /* 1 = global alpha is on */
	int osd_local_alpha_state;      /* 1 = local alpha is on */
	int osd_chroma_key_state;       /* 1 = chroma-keying is on */
	u8  osd_global_alpha;           /* current global alpha */
	u32 osd_chroma_key;             /* current chroma key */
	struct v4l2_rect osd_rect;      /* current OSD position and size */
	struct v4l2_rect main_rect;     /* current Main window position and size */
	struct osd_info *osd_info;      /* ivtvfb private OSD info */
	void (*ivtvfb_restore)(struct ivtv *itv); /* Used for a warm start */
};

static inline struct ivtv *to_ivtv(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct ivtv, v4l2_dev);
}

/* ivtv extensions to be loaded */
extern int (*ivtv_ext_init)(struct ivtv *);

/* Globals */
extern int ivtv_first_minor;

/*==============Prototypes==================*/

/* Hardware/IRQ */
void ivtv_set_irq_mask(struct ivtv *itv, u32 mask);
void ivtv_clear_irq_mask(struct ivtv *itv, u32 mask);

/* try to set output mode, return current mode. */
int ivtv_set_output_mode(struct ivtv *itv, int mode);

/* return current output stream based on current mode */
struct ivtv_stream *ivtv_get_output_stream(struct ivtv *itv);

/* Return non-zero if a signal is pending */
int ivtv_msleep_timeout(unsigned int msecs, int intr);

/* Wait on queue, returns -EINTR if interrupted */
int ivtv_waitq(wait_queue_head_t *waitq);

/* Read Hauppauge eeprom */
struct tveeprom; /* forward reference */
void ivtv_read_eeprom(struct ivtv *itv, struct tveeprom *tv);

/* First-open initialization: load firmware, init cx25840, etc. */
int ivtv_init_on_first_open(struct ivtv *itv);

/* Test if the current VBI mode is raw (1) or sliced (0) */
static inline int ivtv_raw_vbi(const struct ivtv *itv)
{
	return itv->vbi.in.type == V4L2_BUF_TYPE_VBI_CAPTURE;
}

/* This is a PCI post thing, where if the pci register is not read, then
   the write doesn't always take effect right away. By reading back the
   register any pending PCI writes will be performed (in order), and so
   you can be sure that the writes are guaranteed to be done.

   Rarely needed, only in some timing sensitive cases.
   Apparently if this is not done some motherboards seem
   to kill the firmware and get into the broken state until computer is
   rebooted. */
#define write_sync(val, reg) \
	do { writel(val, reg); readl(reg); } while (0)

#define read_reg(reg) readl(itv->reg_mem + (reg))
#define write_reg(val, reg) writel(val, itv->reg_mem + (reg))
#define write_reg_sync(val, reg) \
	do { write_reg(val, reg); read_reg(reg); } while (0)

#define read_enc(addr) readl(itv->enc_mem + (u32)(addr))
#define write_enc(val, addr) writel(val, itv->enc_mem + (u32)(addr))
#define write_enc_sync(val, addr) \
	do { write_enc(val, addr); read_enc(addr); } while (0)

#define read_dec(addr) readl(itv->dec_mem + (u32)(addr))
#define write_dec(val, addr) writel(val, itv->dec_mem + (u32)(addr))
#define write_dec_sync(val, addr) \
	do { write_dec(val, addr); read_dec(addr); } while (0)

/* Call the specified callback for all subdevs matching hw (if 0, then
   match them all). Ignore any errors. */
#define ivtv_call_hw(itv, hw, o, f, args...)				\
	v4l2_device_mask_call_all(&(itv)->v4l2_dev, hw, o, f, ##args)

#define ivtv_call_all(itv, o, f, args...) ivtv_call_hw(itv, 0, o, f , ##args)

/* Call the specified callback for all subdevs matching hw (if 0, then
   match them all). If the callback returns an error other than 0 or
   -ENOIOCTLCMD, then return with that error code. */
#define ivtv_call_hw_err(itv, hw, o, f, args...)			\
	v4l2_device_mask_call_until_err(&(itv)->v4l2_dev, hw, o, f, ##args)

#define ivtv_call_all_err(itv, o, f, args...) ivtv_call_hw_err(itv, 0, o, f , ##args)

#endif
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              