/*
 * Copyright (c) 2017 Kernel Labs Inc. All Rights Reserved
 *
 * Address: Kernel Labs Inc., PO Box 745, St James, NY. 11780
 * Contact: sales@kernellabs.com
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <assert.h>
#include "smpte337_detector2.h"
#include "common/common.h"

struct smpte337_detector2_item_s
{
	struct xorg_list list;
	struct avfm_s avfm;

	unsigned char *ptr;
	int maxlen;
	int usedlen;
	int readpos;
};

static struct smpte337_detector2_item_s *_item_alloc(int maxlen)
{
	struct smpte337_detector2_item_s *i = calloc(1, sizeof(*i));
	i->ptr = malloc(maxlen);
	i->maxlen = maxlen;
	i->usedlen = 0;
	i->readpos = 0;
	return i;
}

static void _item_free(struct smpte337_detector2_item_s *i)
{
	if (i->ptr) {
		free(i->ptr);
		i->ptr = NULL;
	}

	i->usedlen = 0;
	i->maxlen = 0;
	i->readpos = 0;

	free(i);
}

static void _list_empty(struct smpte337_detector2_s *ctx)
{
	pthread_mutex_lock(&ctx->itemListMutex);
	while (!xorg_list_is_empty(&ctx->itemList)) {
		struct smpte337_detector2_item_s *i = xorg_list_first_entry(&ctx->itemList, struct smpte337_detector2_item_s, list);

		xorg_list_del(&i->list);

		_item_free(i);
	}
	pthread_mutex_unlock(&ctx->itemListMutex);
}

static int _list_peek(struct smpte337_detector2_s *ctx, unsigned char *buf, int byteCount, int updateCounters, struct avfm_s *avfm)
{
	int copied_timing = 0;
	int len = 0;
	pthread_mutex_lock(&ctx->itemListMutex);

	if (xorg_list_is_empty(&ctx->itemList)) {
		pthread_mutex_unlock(&ctx->itemListMutex);
		return 0;
	}

	struct smpte337_detector2_item_s *e = NULL, *next;
	xorg_list_for_each_entry_safe(e, next, &ctx->itemList, list) {

		/* Satisfy a read from the top item. */
		int rem = byteCount - len;
		if (e->usedlen - e->readpos >= rem) {
			memcpy(buf + len, e->ptr + e->readpos, rem);
			len += rem;
			memcpy(avfm, &e->avfm, sizeof(e->avfm));

			if (updateCounters) {
				e->readpos += rem;
				ctx->itemListTotalBytes -= rem;
				assert(ctx->itemListTotalBytes >= 0);
				if (e->readpos == e->usedlen) {
					xorg_list_del(&e->list);
					_item_free(e);
				}
			}

			pthread_mutex_unlock(&ctx->itemListMutex);
			return len;
		}

		/* We need to handle reads across buffers. */
		rem = e->usedlen - e->readpos;
		memcpy(buf + len, e->ptr + e->readpos, rem);
		len += rem;

		/* Return the earliest timing information along with this buffer. */
		if (copied_timing++ == 0)
			memcpy(avfm, &e->avfm, sizeof(e->avfm));

		if (updateCounters) {
			e->readpos += rem;
			ctx->itemListTotalBytes -= rem;
			assert(ctx->itemListTotalBytes >= 0);
			if (e->readpos == e->usedlen) {
				xorg_list_del(&e->list);
				_item_free(e);
			}
		}
		
		if (len == byteCount)
			break;

	}

	pthread_mutex_unlock(&ctx->itemListMutex);

	return len;
}

static size_t _list_read_alloc(struct smpte337_detector2_s *ctx, unsigned char **payload, int byteCount, struct avfm_s *avfm)
{
	/* Caller is responsible for its destruction. */
	unsigned char *buf = malloc(byteCount);
	if (!buf)
		return 0;

	*payload = buf;

	return _list_peek(ctx, buf, byteCount, 1, avfm);
}

/* Discard byteCount bytes from the list. */
static int _list_discard(struct smpte337_detector2_s *ctx, int byteCount)
{
	unsigned char buf[128];
	struct avfm_s avfm;
	int r = _list_peek(ctx, &buf[0], byteCount, 1, &avfm);

	return r;
}

struct smpte337_detector2_s *smpte337_detector2_alloc(smpte337_detector2_callback cb, void *cbContext)
{
	struct smpte337_detector2_s *ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		return NULL;

	pthread_mutex_init(&ctx->itemListMutex, NULL);
	xorg_list_init(&ctx->itemList);

	ctx->cb = cb;
	ctx->cbContext = cbContext;

	return ctx;
}

void smpte337_detector2_free(struct smpte337_detector2_s *ctx)
{
	_list_empty(ctx);
	free(ctx);
}

static void handleCallback(struct smpte337_detector2_s *ctx, uint8_t datamode, uint8_t datatype,
	uint32_t payload_bitCount, uint8_t *payload, struct avfm_s *avfm)
{
	ctx->cb(ctx->cbContext, ctx, datamode, datatype, payload_bitCount, payload, avfm);
}

static size_t smpte337_detector2_write_32b(struct smpte337_detector2_s *ctx, struct smpte337_detector2_item_s *item, uint8_t *buf,
	uint32_t audioFrames, uint32_t sampleDepth, uint32_t channelsPerFrame,
	uint32_t frameStrideBytes,
	uint32_t spanCount, struct avfm_s *avfm)
{
	size_t consumed = 0;

	unsigned char *dst = item->ptr + item->usedlen;

	uint32_t *p = (uint32_t *)buf;
	for (int i = 0; i < audioFrames; i++) {

		uint32_t *q = p;
		for (int k = 0; k < spanCount; k++) {
			//printf("item %p, usedlen %d maxlen %d\n", item, item->usedlen, item->maxlen);
			assert(item->usedlen <= item->maxlen);

			/* Sample in N words into a byte orientied buffer */
			uint8_t *x = (uint8_t *)q;

			*(dst + 0) = *(((const char *)x) + 3);
			*(dst + 1) = *(((const char *)x) + 2);

			dst += 2;
			item->usedlen += 2;

			q++;

			consumed += 2;
		}

		p += (frameStrideBytes / sizeof(uint32_t));
	}

	return consumed;
}

static void run_detector(struct smpte337_detector2_s *ctx)
{
	int skipped = 0;

#define PEEK_LEN 16
	uint8_t dat[PEEK_LEN];
	struct avfm_s avfm;
	if (_list_peek(ctx, &dat[0], PEEK_LEN, 0, &avfm) < PEEK_LEN)
		return;

	while(1) {
		if (ctx->itemListTotalBytes < PEEK_LEN)
			break;

		if (_list_peek(ctx, &dat[0], PEEK_LEN, 0, &avfm) < PEEK_LEN)
			break;

		/* Find the supported patterns - In this case, AC3 only in 16bit mode */
		/* See SMPTE 337M 2015 spec table 6.
		 * Pa = dat0/1
		 * Pb = dat2/3 ... etc
		 */
		if (dat[0] == 0xF8 && dat[1] == 0x72 && dat[2] == 0x4e && dat[3] == 0x1f) {
			/* Confirmed.... pa = 16bit, pb = 16bit */

			/* Calculate, from the beginning of this audio frame (buffer), the offset of the
			 * word where the sync byte occured.
			 */

			/* Check the burst_info.... */
			if ((dat[5] & 0x1f) == 0x01) {
				/* Bits 0-4 datatype, 1 = AC3 */
				/* Bits 5-6 datamode, 0 = 16bit */
				/* Bits   7 errorflg, 0 = no error */
				uint32_t payload_bitCount = (dat[6] << 8) | dat[7];
				uint32_t payload_byteCount = payload_bitCount / 8;
				
				if (ctx->itemListTotalBytes >= (8 + payload_byteCount)) {
					unsigned char *payload = NULL;
					size_t l = _list_read_alloc(ctx, &payload, 8 + payload_byteCount, &avfm);
					if (l != (8 + payload_byteCount)) {
						fprintf(stderr, "[smpte337_detector2] Warning, list read failure.\n");

						/* Intensionally flush the ring and start acquisition again. */
						_list_empty(ctx);
					} else {
						handleCallback(ctx, (dat[5] >> 5) & 0x03, dat[5] & 0x1f,
							payload_bitCount, (uint8_t *)payload + 8, &avfm);
					}
					if (payload)
						free(payload);
				} else {
					/* Not enough data in the ring buffer, come back next time. */
					break;
				}

			} else {
				_list_discard(ctx, 1); /* Pop a byte, and continue the search */
				skipped++;
			}
		} else {
			_list_discard(ctx, 1); /* Pop a byte, and continue the search */
			skipped++;
		}

	} /* while */
}

size_t smpte337_detector2_write(struct smpte337_detector2_s *ctx, uint8_t *buf,
	uint32_t audioFrames, uint32_t sampleDepth, uint32_t channelsPerFrame,
	uint32_t frameStrideBytes, uint32_t spanCount, struct avfm_s *avfm)
{
	if ((!buf) || (!audioFrames) || (!channelsPerFrame) || (!frameStrideBytes) ||
		((sampleDepth != 16) && (sampleDepth != 32)) ||
		(spanCount == 0) || (spanCount > channelsPerFrame)) {
		return 0;
	}

	struct smpte337_detector2_item_s *item = _item_alloc(16384);
	memcpy(&item->avfm, avfm, sizeof(*avfm));

	size_t ret = 0;
#if 0
	if (sampleDepth == 16) {
		ret = smpte337_detector2_write_16b(ctx, item, buf, audioFrames, sampleDepth,
			channelsPerFrame, frameStrideBytes, spanCount, avfm);
	} else
#endif
	if (sampleDepth == 32) {
		ret = smpte337_detector2_write_32b(ctx, item, buf, audioFrames, sampleDepth,
			channelsPerFrame, frameStrideBytes, spanCount, avfm);
	}

	pthread_mutex_lock(&ctx->itemListMutex);
	xorg_list_append(&item->list, &ctx->itemList);
	ctx->itemListTotalBytes += item->usedlen;
	pthread_mutex_unlock(&ctx->itemListMutex);

	/* Now all the fifo contains byte stream re-ordered data, run the detector. */
	run_detector(ctx);

	return ret;
}
