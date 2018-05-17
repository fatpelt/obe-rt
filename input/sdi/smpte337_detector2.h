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

/* A tool to inspect audio buffers from blackmagic hardware, analyze channels start + span,
 * detect SMPTE337 preable headers and inform the caller. We're a helper framework than
 * can be used to extract AC3 bitstream audio from SDI, or other bitstream codecs for that
 * matter.
 */

#ifndef _SMPTE337_DETECTOR2_H 
#define _SMPTE337_DETECTOR2_H 

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include "input/sdi/v4l2/xorg-list.h"

#ifdef __cplusplus
extern "C" {
#endif

struct smpte337_detector2_s;
struct avfm_s;

typedef void (*smpte337_detector2_callback)(void *user_context,
	struct smpte337_detector2_s *ctx, 
	uint8_t datamode, uint8_t datatype, uint32_t payload_bitCount,
	uint8_t *payload, struct avfm_s *avfm);

struct smpte337_detector2_s
{
	struct xorg_list itemList;
	pthread_mutex_t itemListMutex;
	int itemListTotalBytes;

	smpte337_detector2_callback cb;
	void *cbContext;
};

struct smpte337_detector2_s *smpte337_detector2_alloc(smpte337_detector2_callback cb, void *cbContext);

void smpte337_detector2_free(struct smpte337_detector2_s *ctx);

size_t smpte337_detector2_write(struct smpte337_detector2_s *ctx, uint8_t *buf,
	uint32_t audioFrames, uint32_t sampleDepth, uint32_t channelsPerFrame, uint32_t frameStrideBytes,
	uint32_t spanCount, struct avfm_s *avfm);

#ifdef __cplusplus
};
#endif

#endif /* _SMPTE337_DETECTOR2_H */
