/*****************************************************************************
 * file.c : FILE output functions
 *****************************************************************************
 * Copyright (C) 2018 LTN
 *
 * Authors: Steven Toth <stoth@ltnglobal.com>
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
 *****************************************************************************/

#include <sys/time.h>

#include "common/common.h"
#include "common/network/network.h"
#include "common/network/udp/udp.h"
#include "output/output.h"
#include "common/bitstream.h"

#define LOCAL_DEBUG 1

#define PREFIX "[FILE]: "

struct file_ts_status
{
    obe_output_t *output;
    FILE *fh;
};

static void close_output(void *handle)
{
	struct file_ts_status *status = handle;

	if (status->output->output_dest.type == OUTPUT_FILE_TS) {
		fclose(status->fh);
		status->fh = NULL;
	}

	if (status->output->output_dest.target)
		free(status->output->output_dest.target);

	pthread_mutex_unlock(&status->output->queue.mutex);
	free(status);
}

static void *file_ts_start(void *ptr)
{
#if LOCAL_DEBUG
	printf(PREFIX "Starting\n");
#endif

	obe_output_t *output = ptr;
	obe_output_dest_t *output_dest = &output->output_dest;
	struct file_ts_status *status = calloc(1, sizeof(*status));
	if (!status) {
		fprintf(stderr, PREFIX "Unable to malloc\n");
		return NULL;
	}
	status->output = output;

	if (output_dest->type != OUTPUT_FILE_TS) {
            fprintf(stderr, PREFIX "Output type is not file ts\n");
            return NULL;
	}

	status->fh = fopen(output_dest->target, "wb");
	if (!status->fh) {
            fprintf(stderr, PREFIX "Could not create file output [%s]\n", output_dest->target);
            return NULL;
	}

	pthread_cleanup_push(close_output, (void*)status);

	while (1)
	{
		pthread_mutex_lock(&output->queue.mutex);
		while(!output->queue.size && !output->cancel_thread) {
			/* Often this cond_wait is not because of an underflow */
			pthread_cond_wait(&output->queue.in_cv, &output->queue.mutex);
		}

		if (output->cancel_thread) {
			pthread_mutex_unlock(&output->queue.mutex);
			break;
		}

		int num_muxed_data = output->queue.size;

		AVBufferRef **muxed_data = malloc(num_muxed_data * sizeof(*muxed_data));
		if (!muxed_data) {
			pthread_mutex_unlock(&output->queue.mutex);
			syslog(LOG_ERR, PREFIX "Malloc failed\n");
			return NULL;
		}
		memcpy(muxed_data, output->queue.queue, num_muxed_data * sizeof(*muxed_data));
		pthread_mutex_unlock(&output->queue.mutex);

#if LOCAL_DEBUG
		//printf(PREFIX "writing %d frames\n", num_muxed_data);
#endif
		for (int i = 0; i < num_muxed_data; i++) {
			int len = 188 * 7;
			size_t wlen = fwrite(&muxed_data[i]->data[7 * sizeof(int64_t)], 1, len, status->fh);
			if (wlen <= 0) {
				fprintf(stderr, PREFIX "Failed to write packet\n");
				syslog(LOG_ERR, PREFIX "Failed to write packet\n");
			}

			remove_from_queue(&output->queue);
			av_buffer_unref(&muxed_data[i]);
		}

		free(muxed_data);
		muxed_data = NULL;

	} /* while(1) */

	pthread_cleanup_pop(1);
	return NULL;
}

const obe_output_func_t file_ts_output = { file_ts_start };
