/*****************************************************************************
 * x265.c : x265 encoding functions
 *****************************************************************************
 * Copyright (C) 2018 LTN
 *
 * Authors: Steven Toth <stoth@ltnglobal.com>
 * Using the x264 template developed by: Kieran Kunhya <kieran@kunhya.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 ******************************************************************************/

#include "common/common.h"
#include "encoders/video/video.h"
#include <libavutil/mathematics.h>
#include <x265.h>

#define LOCAL_DEBUG 0
#define NAL_DEBUG 0

/* Debug feature, save each 1080i field picture as YUV to disk. */
#define SAVE_FIELDS 0

#define MESSAGE_PREFIX "[x265]:"

int g_x265_monitor_bps = 0;
static int64_t g_frame_duration = 0;

#define SERIALIZE_CODED_FRAMES 0
#if SERIALIZE_CODED_FRAMES
static FILE *sfh = NULL;
static void serialize_coded_frame(obe_coded_frame_t *cf)
{
    if (!sfh)
        sfh = fopen("/storage/dev/x265-out.cf", "wb");
    if (sfh)
        coded_frame_serializer_write(sfh, cf);
}
#endif

struct context_s
{
	obe_vid_enc_params_t *enc_params;
	obe_t *h;
	obe_encoder_t *encoder;

	/* */
	x265_encoder *hevc_encoder;
	x265_param   *hevc_params;
	x265_picture *hevc_picture_in;
	x265_picture *hevc_picture_out;

	uint32_t      i_nal;
	x265_nal     *hevc_nals;

	uint64_t      raw_frame_count;
};

struct userdata_s
{
	struct avfm_s avfm;
};

static struct userdata_s *userdata_calloc()
{
	struct userdata_s *ud = calloc(1, sizeof(*ud));
	return ud;
}

static int userdata_set(struct userdata_s *ud, struct avfm_s *s)
{
	memcpy(&ud->avfm, s, sizeof(struct avfm_s));
	return 0;
}

static void userdata_free(struct userdata_s *ud)
{
	free(ud);
}

#if 0
static const char *sliceTypeLookup(uint32_t type)
{
	switch(type) {
	case X265_TYPE_AUTO: return "X265_TYPE_AUTO";
	case X265_TYPE_IDR:  return "X265_TYPE_IDR";
	case X265_TYPE_I:    return "X265_TYPE_I";
	case X265_TYPE_P:    return "X265_TYPE_P";
	case X265_TYPE_BREF: return "X265_TYPE_BREF";
	case X265_TYPE_B:    return "X265_TYPE_B";
	default:             return "UNKNOWN";
	}
}
#endif

/* TODO: Duplicated from video.c */
typedef struct
{
	int planes;
	float width[4];
	float height[4];
	int mod_width;
	int mod_height;
	int bit_depth;
} obe_cli_csp_t;

const static obe_cli_csp_t obe_cli_csps[] =
{
	[PIX_FMT_YUV420P]   = { 3, { 1, .5, .5 }, { 1, .5, .5 }, 2, 2,  8 },
	[PIX_FMT_NV12]      = { 2, { 1,  1 },     { 1, .5 },     2, 2,  8 },
	[PIX_FMT_YUV420P10] = { 3, { 1, .5, .5 }, { 1, .5, .5 }, 2, 2, 10 },
	[PIX_FMT_YUV422P10] = { 3, { 1, .5, .5 }, { 1, 1, 1 },   2, 2, 10 },
	[PIX_FMT_YUV420P16] = { 3, { 1, .5, .5 }, { 1, .5, .5 }, 2, 2, 16 },
};

static int csp_num_interleaved( int csp, int plane )
{
	return (csp == PIX_FMT_NV12 && plane == 1) ? 2 : 1;
}
/* end -- Duplicated from video.c */

/* Free the shallow copy. */
static void x265_picture_free_all(x265_picture *pic)
{
	/* Originally I was copying the planes also. We
	 * can probably refactor this function away.
	 */
#if 0
	free(pic->planes[0]);
#endif
	x265_picture_free(pic);
}

#if SAVE_FIELDS
static void x265_picture_save(x265_picture *pic)
{
	static int index = 0;

	char fn[64];
	sprintf(fn, "field%06d.yuv", index++);
	FILE *fh = fopen(fn, "wb");

	/* Copy all of the planes. */
	uint32_t plane_len[4] = { 0 };
	for (int i = 2; i > 0; i--) {
		plane_len[i - 1] = pic->planes[i] - pic->planes[i - 1];
	}
	if (3) {
		plane_len[2] = plane_len[1];
	}
	/* TODO: Highly specific to 1080i */
	plane_len[0] = 1036800;
	plane_len[1] = 259200;
	plane_len[2] = 259200;

	for (int i = 0; i < 3; i++) {
		if (pic->planes[i]) {
			fwrite(pic->planes[i], 1, plane_len[i], fh);
		}
	}

	fclose(fh);
}
#endif

/* Shallow copy the object. */
static x265_picture *x265_picture_copy(x265_picture *pic)
{
	x265_picture *p = malloc(sizeof(*p));
	memcpy(p, pic, sizeof(*p));

#if 0
	/* Copy all of the planes. */
	uint32_t plane_len[4] = { 0 };
	for (int i = 2; i > 0; i--) {
		plane_len[i - 1] = pic->planes[i] - pic->planes[i - 1];
	}
	if (3) {
		plane_len[2] = plane_len[1];
	}

	uint32_t alloc_size = 0;
	for (int i = 0; i < 3; i++)
		alloc_size += plane_len[i];

        for (int i = 0; i < 3; i++) {

                if (i == 0 && pic->planes[i]) {
                        p->planes[i] = (uint8_t *)malloc(alloc_size);
                        memcpy(p->planes[i], pic->planes[i], alloc_size);
                }
                if (i > 0) {
                        p->planes[i] = p->planes[i - 1] + plane_len[i - 1];
                }

        }
#endif

	return p;
}

/* Convert an 8bit interleaved pair of fields into TFF top/bottom image. */
static int convert_interleaved_to_topbottom(struct context_s *ctx, obe_raw_frame_t *raw_frame)
{
	/* Create a new image, copy/reformat the source image then discard it. */
	obe_image_t *img = &raw_frame->img;
	obe_image_t tmp_image = {0};
	obe_image_t *out = &tmp_image;

	tmp_image.csp = img->csp == PIX_FMT_YUV422P10 ? PIX_FMT_YUV422P : PIX_FMT_YUV420P;
	tmp_image.width = raw_frame->img.width;
	tmp_image.height = raw_frame->img.height;
	tmp_image.planes = av_pix_fmt_descriptors[tmp_image.csp].nb_components;
	tmp_image.format = raw_frame->img.format;

	if (av_image_alloc(tmp_image.plane, tmp_image.stride, tmp_image.width, tmp_image.height + 1, tmp_image.csp, 16) < 0) {
		syslog(LOG_ERR, "%s() Malloc failed", __func__);
		return -1;
	}

	for (int i = 0; i < img->planes; i++) {

		int num_interleaved = csp_num_interleaved(img->csp, i);
		int height = obe_cli_csps[img->csp].height[i] * img->height;
		int width = obe_cli_csps[img->csp].width[i] * img->width / num_interleaved;
/*
plane0 num_interleaved 1 width 1920 height 1080
plane1 num_interleaved 1 width 960 height 540
plane2 num_interleaved 1 width 960 height 540

plane0 num_interleaved 1 width 720 height 480
plane1 num_interleaved 1 width 360 height 240
plane2 num_interleaved 1 width 360 height 240
printf("plane%d num_interleaved %d width %d height %d\n", i, num_interleaved, width, height);
*/
/*
 * in->stride[0] = 720  out->stride[0] = 720
 * in->stride[1] = 368  out->stride[1] = 368
 * in->stride[2] = 368  out->stride[2] = 368
 * printf("in->stride[%d] = %d  out->stride[%d] = %d\n", i, img->stride[i], i, out->stride[i]);
 */
		/* Lets assume the first row belongs in the first field (TFF). */
		uint8_t *src = img->plane[i];
		uint8_t *dstTop = out->plane[i];
		uint8_t *dstBottom = out->plane[i] + ((out->stride[i] * height) / 2);

		for (int j = 0; j < height / 2; j++) {

			/* First field. */
			memcpy(dstTop, src, width);
			src += img->stride[i];
			dstTop += out->stride[i];

			/* Second field. */
			memcpy(dstBottom, src, width);
			src += img->stride[i];
			dstBottom += out->stride[i];

		}
	}

	raw_frame->release_data(raw_frame);
	memcpy(&raw_frame->alloc_img, &tmp_image, sizeof(obe_image_t));
	memcpy(&raw_frame->img, &raw_frame->alloc_img, sizeof(obe_image_t));

	return 0;
}

/* Convert a obe_raw_frame_t into a x264_picture_t struct.
 * Incoming frame is colorspace YUV420P.
 */
static int convert_obe_to_x265_pic(struct context_s *ctx, x265_picture *p, struct userdata_s *ud, obe_raw_frame_t *rf)
{
	obe_image_t *img = &rf->img;
	int count = 0, idx = 0;

	if (ctx->enc_params->avc_param.b_interlaced) {
		/* Convert from traditional interlaced interleaved lines to a top/bottom image. */
		/* TODO: deal with field dominance. */
		convert_interleaved_to_topbottom(ctx, rf);
	}

#if 0
	/* Save raw image to disk. Its 8bit. Convert for viewing purposes.
	 * Interlaced content will still be interlaced.
	 * Convert to png with:
	 * ffmpeg -y -f rawvideo -pix_fmt yuv420p -s 1920x1080 -i video-frame-00000413.yuv -vframes 1 new.png
	 */
	obe_image_save(&rf->img);
#endif

	/* Dealloc any previous SEI pointers and payload, if we had any. */
	if (p->userSEI.numPayloads) {
		for (int i = 0; i < p->userSEI.numPayloads; i++) {
			free(p->userSEI.payloads[i].payload);
			p->userSEI.payloads[i].payload = NULL;
			p->userSEI.payloads[idx].payloadSize = 0;
		}
		free(p->userSEI.payloads);
		p->userSEI.payloads = NULL;
	}

	x265_picture_init(ctx->hevc_params, p);

	p->sliceType = X265_TYPE_AUTO;
	p->bitDepth = 8;
	p->stride[0] = img->stride[0];
	p->stride[1] = img->stride[1]; // >> x265_cli_csps[p->colorSpace].width[1];
	p->stride[2] = img->stride[2]; // >> x265_cli_csps[p->colorSpace].width[2];

	for (int i = 0; i < 3; i++) {
		p->stride[i] = img->stride[i];
		p->planes[i] = img->plane[i];
	}

	p->colorSpace = img->csp == PIX_FMT_YUV422P || img->csp == PIX_FMT_YUV422P10 ? X265_CSP_I422 : X265_CSP_I420;
#ifdef HIGH_BIT_DEPTH
	p->colorSpace |= X265_CSP_HIGH_DEPTH;
#endif

	for (int i = 0; i < rf->num_user_data; i++) {
		/* Only give correctly formatted data to the encoder */
		if (rf->user_data[i].type == USER_DATA_AVC_REGISTERED_ITU_T35 ||
			rf->user_data[i].type == USER_DATA_AVC_UNREGISTERED) {
			count++;
		}
	}

	if (g_sei_timestamping) {
		/* Create space for unregister data, containing before and after timestamps. */
		count += 1;
	}

	p->userSEI.numPayloads = count;

	if (p->userSEI.numPayloads) {
		p->userSEI.payloads = malloc(p->userSEI.numPayloads * sizeof(*p->userSEI.payloads));
		if (!p->userSEI.payloads)
			return -1;

		for (int i = 0; i < rf->num_user_data; i++) {
			/* Only give correctly formatted data to the encoder */

			if (rf->user_data[i].type == USER_DATA_AVC_REGISTERED_ITU_T35 || rf->user_data[i].type == USER_DATA_AVC_UNREGISTERED) {
				p->userSEI.payloads[idx].payloadType = rf->user_data[i].type;
				p->userSEI.payloads[idx].payloadSize = rf->user_data[i].len;
				p->userSEI.payloads[idx].payload = malloc(p->userSEI.payloads[idx].payloadSize);
				memcpy(p->userSEI.payloads[idx].payload, rf->user_data[i].data, p->userSEI.payloads[idx].payloadSize);
				idx++;
			} else {
				syslog(LOG_WARNING, MESSAGE_PREFIX " Invalid user data presented to encoder - type %i\n", rf->user_data[i].type);
				printf(MESSAGE_PREFIX " (1) Invalid user data presented to encoder - type %i\n", rf->user_data[i].type);
				free(rf->user_data[i].data);
			}
		}
	} else if (rf->num_user_data) {
		for (int i = 0; i < rf->num_user_data; i++) {
			syslog(LOG_WARNING, MESSAGE_PREFIX " Invalid user data presented to encoder - type %i\n", rf->user_data[i].type);
			printf(MESSAGE_PREFIX " (2) Invalid user data presented to encoder - type %i\n", rf->user_data[i].type);
			free(rf->user_data[i].data);
		}
	}

	if (g_sei_timestamping) {
		x265_sei_payload *x;

		/* Start time - Always the last SEI */
		static uint32_t framecount = 0;
		x = &p->userSEI.payloads[count - 1];
		x->payloadType = USER_DATA_AVC_UNREGISTERED;
		x->payloadSize = SEI_TIMESTAMP_PAYLOAD_LENGTH;
		x->payload = set_timestamp_alloc(); /* Freed when we enter the function prior to pic re-init. */

		struct timeval tv;
		gettimeofday(&tv, NULL);

		set_timestamp_field_set(x->payload, 1, framecount);
		set_timestamp_field_set(x->payload, 2, avfm_get_hw_received_tv_sec(&rf->avfm));
		set_timestamp_field_set(x->payload, 3, avfm_get_hw_received_tv_usec(&rf->avfm));
		set_timestamp_field_set(x->payload, 4, tv.tv_sec);
		set_timestamp_field_set(x->payload, 5, tv.tv_usec);
		set_timestamp_field_set(x->payload, 6, 0);
		set_timestamp_field_set(x->payload, 7, 0);
		set_timestamp_field_set(x->payload, 8, 0);
		set_timestamp_field_set(x->payload, 9, 0);

		/* The remaining 8 bytes (time exit from compressor fields)
		 * will be filled when the frame exists the compressor. */
		framecount++;
	}

	return 0;
}

static int dispatch_payload(struct context_s *ctx, const unsigned char *buf, int lengthBytes, int64_t arrival_time)
{
	obe_coded_frame_t *cf = new_coded_frame(ctx->encoder->output_stream_id, lengthBytes);
	if (!cf) {
		fprintf(stderr, MESSAGE_PREFIX " unable to alloc a new coded frame\n");
		return -1;
	}

#if NAL_DEBUG
	printf(MESSAGE_PREFIX " --  acquired %7d nal bytes, pts = %12" PRIi64 " dts = %12" PRIi64 ", ",
		lengthBytes,
		ctx->hevc_picture_out->pts,
		ctx->hevc_picture_out->dts);
	printf("poc %8d  sliceType %d\n",
		ctx->hevc_picture_out->poc, ctx->hevc_picture_out->sliceType);
#endif

	static int64_t last_hw_pts = 0;
	struct userdata_s *out_ud = ctx->hevc_picture_out->userData; 
	if (out_ud) {
		/* Make sure we push the original hardware timing into the new frame. */
		memcpy(&cf->avfm, &out_ud->avfm, sizeof(struct avfm_s));

		cf->pts = out_ud->avfm.audio_pts;
		last_hw_pts = out_ud->avfm.audio_pts;
		userdata_free(out_ud);
		out_ud = NULL;
		ctx->hevc_picture_out->userData = 0;
	} else {
		//fprintf(stderr, MESSAGE_PREFIX " missing pic out userData\n");
		cf->pts = last_hw_pts;
	}

	memcpy(cf->data, buf, lengthBytes);
	cf->len                      = lengthBytes;
	cf->type                     = CF_VIDEO;
#if 0
	/* Rely on the PTS time that comes from the s/w codec. */
	cf->pts                      = ctx->hevc_picture_out->pts + 45000;
	cf->real_pts                 = ctx->hevc_picture_out->pts + 45000;
	cf->real_dts                 = ctx->hevc_picture_out->dts;
#else
	/* Recalculate a new real pts/dts based on the hardware time, not the codec time. */
	cf->real_pts                 = cf->pts;
	cf->real_dts                 = ctx->hevc_picture_out->dts;
#endif

	cf->cpb_initial_arrival_time = cf->real_pts;

	double estimated_final = ((double)cf->len / 0.0810186) + (double)cf->cpb_initial_arrival_time;
	cf->cpb_final_arrival_time   = estimated_final;

	cf->priority = IS_X265_TYPE_I(ctx->hevc_picture_out->sliceType);
	cf->random_access = IS_X265_TYPE_I(ctx->hevc_picture_out->sliceType);

	/* If interlaced is active, and the pts time has repeated, increment clocks
	 * by the field rate (frame_duration / 2). Why do we increment clocks? We can either
	 * take the PTS from the codec (which repeats) or derive the PTS from the actual
	 * capture hardware. The codec PTS is guaranteed sequence, a problem during signal loss.
	 * The hardware clock is non-sequential - a better clock to handle signal loss.
	 */
	static int64_t last_dispatch_pts = 0;
	if (ctx->enc_params->avc_param.b_interlaced) {
		if (cf->pts == last_dispatch_pts) {
			cf->pts += (g_frame_duration / 2);
			/* Recalculate a new real pts/dts based on the hardware time, not the codec time. */
			cf->real_pts = cf->pts;
			cf->real_dts = cf->pts;
		}
		last_dispatch_pts = cf->pts;
	}

#if NAL_DEBUG
	printf(MESSAGE_PREFIX " --    output %7d nal bytes, pts = %12" PRIi64 " dts = %12" PRIi64 "\n",
		lengthBytes,
		cf->pts,
		cf->real_dts);
#endif
	if (ctx->h->obe_system == OBE_SYSTEM_TYPE_LOWEST_LATENCY || ctx->h->obe_system == OBE_SYSTEM_TYPE_LOW_LATENCY) {
		cf->arrival_time = arrival_time;
#if SERIALIZE_CODED_FRAMES
		serialize_coded_frame(cf);
#endif
		add_to_queue(&ctx->h->mux_queue, cf);
		//printf(MESSAGE_PREFIX " Encode Latency %"PRIi64" \n", obe_mdate() - cf->arrival_time);
	} else {
#if SERIALIZE_CODED_FRAMES
		serialize_coded_frame(cf);
#endif
		add_to_queue(&ctx->h->enc_smoothing_queue, cf);
	}

	return 0;
}

static void _monitor_bps(struct context_s *ctx, int lengthBytes)
{
	/* Monitor bps for sanity... */
	static int codec_bps_current = 0;
	static int codec_bps = 0;
	static time_t codec_bps_time = 0;
	time_t now;
	time(&now);
	if (now != codec_bps_time) {
		codec_bps = codec_bps_current;
		codec_bps_current = 0;
		codec_bps_time = now;
		double dbps = (double)codec_bps;
		dbps /= 1e6;
		if (dbps >= ctx->enc_params->avc_param.rc.i_vbv_max_bitrate) {
			fprintf(stderr, MESSAGE_PREFIX " codec output %d bps exceeds vbv_max_bitrate %d @ %s",
				codec_bps,
				ctx->enc_params->avc_param.rc.i_vbv_max_bitrate,
				ctime(&now));
		}
		if (g_x265_monitor_bps) {
			printf(MESSAGE_PREFIX " codec output %.02f (Mb/ps) @ %s", dbps, ctime(&now));
		}
	}
	codec_bps_current += (lengthBytes * 8);
}

static void _process_nals(struct context_s *ctx, int64_t arrival_time)
{
	if (ctx->i_nal == 0)
		return;

	if (g_sei_timestamping) {
		/* Walk through each of the NALS and insert current time into any LTN sei timestamp frames we find. */
		for (int m = 0; m < ctx->i_nal; m++) {
			int offset = ltn_uuid_find(&ctx->hevc_nals[m].payload[0], ctx->hevc_nals[m].sizeBytes);
			if (offset >= 0) {
				struct timeval tv;
				gettimeofday(&tv, NULL);

				/* Add the time exit from compressor seconds/useconds. */
				set_timestamp_field_set(&ctx->hevc_nals[m].payload[offset], 6, tv.tv_sec);
				set_timestamp_field_set(&ctx->hevc_nals[m].payload[offset], 7, tv.tv_usec);
			}
		}
	}

	/* Collapse N nals into a single allocation, so we can submit a single coded_frame, with a single clock. */
	int nalbuffer_size = 0;
	for (int z = 0; z < ctx->i_nal; z++) {
		nalbuffer_size += ctx->hevc_nals[z].sizeBytes;
	}

	unsigned char *nalbuffer = malloc(((nalbuffer_size / getpagesize()) + 1) * getpagesize());
	int offset = 0;
	for (int z = 0; z < ctx->i_nal; z++) {
		memcpy(nalbuffer + offset, ctx->hevc_nals[z].payload, ctx->hevc_nals[z].sizeBytes);
		offset += ctx->hevc_nals[z].sizeBytes;
	}

	dispatch_payload(ctx, nalbuffer, nalbuffer_size, arrival_time);
	free(nalbuffer);

	/* Monitor bps for sanity... */
	_monitor_bps(ctx, nalbuffer_size);
}

/* OBE will pass us a AVC struct initially. Pull out any important pieces
 * and pass those to x265.
 */
static void *x265_start_encoder( void *ptr )
{
	struct context_s ectx, *ctx = &ectx;
	memset(ctx, 0, sizeof(*ctx));

	ctx->enc_params = ptr;
	ctx->h = ctx->enc_params->h;
	ctx->encoder = ctx->enc_params->encoder;

	ctx->hevc_params = x265_param_alloc();
	if (!ctx->hevc_params) {
		fprintf(stderr, MESSAGE_PREFIX " failed to allocate params\n");
		goto out1;
	}

	x265_param_default(ctx->hevc_params);

	int ret = x265_param_default_preset(ctx->hevc_params, "ultrafast", "zerolatency");
	if (ret < 0) {
		fprintf(stderr, MESSAGE_PREFIX " failed to set default params\n");
		goto out1;
	}

//	ctx->hevc_params->fpsDenom = ctx->enc_params->avc_param.i_fps_den;
//	ctx->hevc_params->fpsNum = ctx->enc_params->avc_param.i_fps_num;

#if 0
                avc_param->rc.i_vbv_max_bitrate = obe_otoi( vbv_maxrate, 0 );
                avc_param->rc.i_vbv_buffer_size = obe_otoi( vbv_bufsize, 0 );
                avc_param->rc.i_bitrate         = obe_otoi( bitrate, 0 );
                avc_param->i_keyint_max        = obe_otoi( keyint, avc_param->i_keyint_max );
                avc_param->rc.i_lookahead      = obe_otoi( lookahead, avc_param->rc.i_lookahead );
                avc_param->i_threads           = obe_otoi( threads, avc_param->i_threads );
#endif

	char val[64];
	if (ctx->enc_params->avc_param.b_interlaced) {
		sprintf(val, "%dx%d",
			ctx->enc_params->avc_param.i_width,
			ctx->enc_params->avc_param.i_height / 2);
	} else {
		sprintf(val, "%dx%d", ctx->enc_params->avc_param.i_width, ctx->enc_params->avc_param.i_height);
	}
	x265_param_parse(ctx->hevc_params, "input-res", val);

	if (getenv("OBE_X265_STREAM_INJECT_DETAILS") == NULL)
		x265_param_parse(ctx->hevc_params, "info", "0");

	/* The decklink module sets TFF to always true for interlaced formats, which isn't technically
         * correct. NTSC is BFF. A comment suggests NTSC should be BFF but is packed as TFF.
         * We'll obey the params here, and leave this comment that NTSC could be wrong if the original
         * packing issues was specific to a decklink driver version.
         */
	if (ctx->enc_params->avc_param.b_interlaced) {
		if (ctx->enc_params->avc_param.b_tff)
			x265_param_parse(ctx->hevc_params, "interlace", "tff");
		else
			x265_param_parse(ctx->hevc_params, "interlace", "bff");
	} else {
		x265_param_parse(ctx->hevc_params, "interlace", "0");
	}

	ctx->hevc_params->internalCsp = X265_CSP_I420;
	x265_param_parse(ctx->hevc_params, "repeat-headers", "1");

	if (ctx->enc_params->avc_param.b_interlaced) {
		/* X265 wants the rate in fields per second, instead of (progressive) frames per second. */
		sprintf(&val[0], "%d/%d", ctx->enc_params->avc_param.i_fps_num * 2, ctx->enc_params->avc_param.i_fps_den);
	} else {
		sprintf(&val[0], "%d/%d", ctx->enc_params->avc_param.i_fps_num, ctx->enc_params->avc_param.i_fps_den);
	}
	x265_param_parse(ctx->hevc_params, "fps", val);

	sprintf(&val[0], "%d",ctx->enc_params->avc_param.i_keyint_max);
	x265_param_parse(ctx->hevc_params, "keyint", val);

	if (ctx->h->obe_system == OBE_SYSTEM_TYPE_LOWEST_LATENCY) {
		/* Found that in lowest mode, obe doesn't accept the param, but the codec reports underruns. */
		ctx->enc_params->avc_param.rc.i_vbv_buffer_size = ctx->enc_params->avc_param.rc.i_vbv_max_bitrate;
	}
	sprintf(&val[0], "%d", ctx->enc_params->avc_param.rc.i_vbv_buffer_size);
	x265_param_parse(ctx->hevc_params, "vbv-bufsize", val);

	sprintf(&val[0], "%d", ctx->enc_params->avc_param.rc.i_vbv_max_bitrate);
	x265_param_parse(ctx->hevc_params, "vbv-maxrate", val);

	if (ctx->enc_params->avc_param.i_threads < 8) {
		printf(MESSAGE_PREFIX "configuration threads defined as %d, need a minimum of 8. Adjusting to 8\n",
			ctx->enc_params->avc_param.i_threads);
 
		ctx->enc_params->avc_param.i_threads = 8;
	}
	/* 0 Is preferred, which is 'autodetect' */
	sprintf(&val[0], "%d", ctx->enc_params->avc_param.i_threads);
	x265_param_parse(ctx->hevc_params, "frame-threads", val);

//	x265_param_parse(ctx->hevc_params, "rc-lookahead", "4");
//	x265_param_parse(ctx->hevc_params, "vbv-minrate", "6000");
	sprintf(&val[0], "%d", ctx->enc_params->avc_param.rc.i_bitrate);
	x265_param_parse(ctx->hevc_params, "bitrate", val);

	ctx->hevc_picture_in = x265_picture_alloc();
	if (!ctx->hevc_picture_in) {
		fprintf(stderr, MESSAGE_PREFIX " failed to allocate picture\n");
		goto out2;
	}

	ctx->hevc_picture_out = x265_picture_alloc();
	if (!ctx->hevc_picture_out) {
		fprintf(stderr, MESSAGE_PREFIX " failed to allocate picture\n");
		goto out3;
	}

	/* Fix this? its AVC specific. */
	ctx->encoder->encoder_params = malloc(sizeof(ctx->enc_params->avc_param) );
	if (!ctx->encoder->encoder_params) {
		pthread_mutex_unlock(&ctx->encoder->queue.mutex);
		fprintf(stderr, MESSAGE_PREFIX " failed to allocate encoder params\n");
		goto out4;
	}
	memcpy(ctx->encoder->encoder_params, &ctx->enc_params->avc_param, sizeof(ctx->enc_params->avc_param));

	ctx->hevc_encoder = x265_encoder_open(ctx->hevc_params);
	if (!ctx->hevc_encoder) {
		pthread_mutex_unlock(&ctx->encoder->queue.mutex);
		fprintf(stderr, MESSAGE_PREFIX " failed to open encoder\n");
		goto out5;
	}

	/* Lock the mutex until we verify and fetch new parameters */
	pthread_mutex_lock(&ctx->encoder->queue.mutex);

	ctx->encoder->is_ready = 1;

	g_frame_duration = av_rescale_q( 1, (AVRational){ ctx->enc_params->avc_param.i_fps_den, ctx->enc_params->avc_param.i_fps_num}, (AVRational){ 1, OBE_CLOCK } );
	printf("frame_duration = %" PRIi64 "\n", g_frame_duration);
	//buffer_duration = frame_duration * ctx->enc_params->avc_param.sc.i_buffer_size;

	/* Wake up the muxer */
	pthread_cond_broadcast(&ctx->encoder->queue.in_cv);
	pthread_mutex_unlock(&ctx->encoder->queue.mutex);

	x265_picture_init(ctx->hevc_params, ctx->hevc_picture_in);

	while (1) {
		pthread_mutex_lock(&ctx->encoder->queue.mutex);

		while (!ctx->encoder->queue.size && !ctx->encoder->cancel_thread) {
			pthread_cond_wait(&ctx->encoder->queue.in_cv, &ctx->encoder->queue.mutex);
		}

		if (ctx->encoder->cancel_thread) {
			pthread_mutex_unlock(&ctx->encoder->queue.mutex);
			break;
		}

		/* Reset the speedcontrol buffer if the source has dropped frames. Otherwise speedcontrol
		 * stays in an underflow state and is locked to the fastest preset.
		 */
		pthread_mutex_lock(&ctx->h->drop_mutex);
		if (ctx->h->video_encoder_drop) {
			pthread_mutex_lock(&ctx->h->enc_smoothing_queue.mutex);
			ctx->h->enc_smoothing_buffer_complete = 0;
			pthread_mutex_unlock(&ctx->h->enc_smoothing_queue.mutex);
#if 0
			fprintf(stderr, MESSAGE_PREFIX " Speedcontrol reset\n");
			x264_speedcontrol_sync( s, enc_params->avc_param.sc.i_buffer_size, enc_params->avc_param.sc.f_buffer_init, 0 );
#endif
			ctx->h->video_encoder_drop = 0;
		}
		pthread_mutex_unlock(&ctx->h->drop_mutex);

		obe_raw_frame_t *rf = ctx->encoder->queue.queue[0];
		ctx->raw_frame_count++;
		pthread_mutex_unlock(&ctx->encoder->queue.mutex);

#if LOCAL_DEBUG
		//printf(MESSAGE_PREFIX " popped a raw frame[%" PRIu64 "] -- pts %" PRIi64 "\n", ctx->raw_frame_count, rf->avfm.audio_pts);
#endif

		struct userdata_s *ud = userdata_calloc();

		/* convert obe_frame_t into x264 friendly struct.
		 * Bundle up and incoming SEI etc, into the userdata context
		 */
		if (convert_obe_to_x265_pic(ctx, ctx->hevc_picture_in, ud, rf) < 0) {
			fprintf(stderr, MESSAGE_PREFIX " pic prepare failed\n");
			break;
		}

		ctx->hevc_picture_in->userData = ud;

		/* Cache the upstream timing information in userdata. */
		userdata_set(ud, &rf->avfm);

		/* If the AFD has changed, then change the SAR. x264 will write the SAR at the next keyframe
		 * TODO: allow user to force keyframes in order to be frame accurate.
		 */
		if (rf->sar_width != ctx->enc_params->avc_param.vui.i_sar_width ||
			rf->sar_height != ctx->enc_params->avc_param.vui.i_sar_height) {

			ctx->enc_params->avc_param.vui.i_sar_width  = rf->sar_width;
			ctx->enc_params->avc_param.vui.i_sar_height = rf->sar_height;
//			pic.param = &enc_params->avc_param;

		}

		int leave = 0;
		while (!leave) { 
			/* Compress PIC to NALS. */
			/* Once the pipeline is completely full, x265_encoder_encode() will block until the next output picture is complete. */

			if (rf) {
				ctx->hevc_picture_in->pts = rf->avfm.audio_pts;
				if (ctx->enc_params->avc_param.b_interlaced) {

					/* Complete image copy of the original top/bottom frame. We'll submit this to the encoder as a bottom field
					 * sans any structs we don't specifically want to copy.
					 */
					x265_picture *cpy = x265_picture_copy(ctx->hevc_picture_in);
					cpy->userData = NULL;
					cpy->rcData = NULL;
					cpy->quantOffsets = NULL;
					cpy->userSEI.numPayloads = 0;
					cpy->userSEI.payloads = NULL;

					/* Tamper with plane offets to access the boottom field, adjust start of plane to reflect
  					 * the second field then push this into the codec. */
					uint8_t *origplanes[3] = { 0 };
					for (int n = 0; n < 3; n++) {
						origplanes[n] = cpy->planes[n];
						if (n == 0)
							cpy->planes[n] += (cpy->stride[n] * (ctx->enc_params->avc_param.i_height / 2));
						else
							cpy->planes[n] += (cpy->stride[n] * (ctx->enc_params->avc_param.i_height / 4));
					}

					ctx->i_nal = 0;
					/* TFF or BFF? */
#if SAVE_FIELDS
					x265_picture_save(ctx->hevc_picture_in);
#endif
					ret = x265_encoder_encode(ctx->hevc_encoder, &ctx->hevc_nals, &ctx->i_nal, ctx->hevc_picture_in, ctx->hevc_picture_out);
					if (ret > 0) {
						_process_nals(ctx, rf->arrival_time);
						ret = 0;
						ctx->i_nal = 0;
					}

					ctx->i_nal = 0;
#if SAVE_FIELDS
					x265_picture_save(cpy);
#endif
					ret = x265_encoder_encode(ctx->hevc_encoder, &ctx->hevc_nals, &ctx->i_nal, cpy, ctx->hevc_picture_out);
					if (ret > 0) {
						_process_nals(ctx, rf->arrival_time);
						ret = 0;
						ctx->i_nal = 0;
					}

					/* Restore the original copy plane offsets before we free the image. */
					for (int n = 0; n < 3; n++)
						cpy->planes[n] = origplanes[n];
					x265_picture_free_all(cpy);
				} else {
					ret = x265_encoder_encode(ctx->hevc_encoder, &ctx->hevc_nals, &ctx->i_nal, ctx->hevc_picture_in, ctx->hevc_picture_out);
				}
			}

			int64_t arrival_time = 0;
			if (rf) {
				arrival_time = rf->arrival_time;
				rf->release_data(rf);
				rf->release_frame(rf);
				remove_from_queue(&ctx->encoder->queue);
				rf = 0;
			}

			if (ret < 0) {
				fprintf(stderr, MESSAGE_PREFIX " picture encode failed, ret = %d\n", ret);
				break;
			}

			if (ret == 0) {
				//fprintf(stderr, MESSAGE_PREFIX " ret = %d\n", ret);
				leave = 1;
				continue;
			}

			if (ret > 0) {
				_process_nals(ctx, arrival_time);
			} /* if nal_bytes > 0 */

			leave = 1;
		} /* While ! leave */

	} /* While (1) */

	if (ctx->hevc_encoder)
		x265_encoder_close(ctx->hevc_encoder);

out5:
	free(ctx->enc_params);

out4:
	if (ctx->hevc_picture_out)
		x265_picture_free(ctx->hevc_picture_out);
out3:
	if (ctx->hevc_picture_in)
		x265_picture_free(ctx->hevc_picture_in);
out2:
	if (ctx->hevc_params)
		x265_param_free(ctx->hevc_params);
out1:
	x265_cleanup();

	return NULL;
}

const obe_vid_enc_func_t x265_obe_encoder = { x265_start_encoder };
