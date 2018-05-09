/*****************************************************************************
 * mux/ts/smoothing.c : Mux output smoothing
 *****************************************************************************
 * Copyright (C) 2012 Open Broadcast Systems Ltd.
 *
 * Authors: Kieran Kunhya <kieran@kunhya.com>
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

#include <libavutil/mathematics.h>
#include <libavutil/intreadwrite.h>
#include <libavutil/fifo.h>
#include <libavutil/buffer.h>
#include "common/common.h"

int64_t g_mux_smoother_last_item_count = 0;
int64_t g_mux_smoother_last_total_item_size = 0;
int64_t g_mux_smoother_fifo_pcr_size = 0;
int64_t g_mux_smoother_fifo_data_size = 0;

static void *mux_start_smoothing( void *ptr )
{
    obe_t *h = ptr;
    int num_muxed_data = 0, buffer_complete = 0;
    int64_t start_clock = -1, start_pcr, end_pcr, temporal_vbv_size = 0, cur_pcr = 0;
    obe_muxed_data_t **muxed_data = NULL, *start_data, *end_data;
    AVFifoBuffer *fifo_data = NULL, *fifo_pcr = NULL;
    AVBufferRef **output_buffers = NULL;

    struct sched_param param = {0};
    param.sched_priority = 99;
    pthread_setschedparam( pthread_self(), SCHED_FIFO, &param );

    /* This thread buffers one VBV worth of frames */
    fifo_data = av_fifo_alloc( TS_PACKETS_SIZE );
    if( !fifo_data )
    {
        fprintf( stderr, "[mux-smoothing] Could not allocate data fifo" );
        return NULL;
    }

    fifo_pcr = av_fifo_alloc( 7 * sizeof(int64_t) );
    if( !fifo_pcr )
    {
        fprintf( stderr, "[mux-smoothing] Could not allocate pcr fifo" );
        return NULL;
    }

    output_buffers = malloc( h->num_outputs * sizeof(*output_buffers) );
    if( !output_buffers )
    {
        fprintf( stderr, "[mux-smoothing] Could not allocate output buffers" );
        return NULL;
    }

    if( h->obe_system != OBE_SYSTEM_TYPE_LOWEST_LATENCY )
    {
        for( int i = 0; i < h->num_encoders; i++ )
        {
            if( h->encoders[i]->is_video )
            {
                pthread_mutex_lock( &h->encoders[i]->queue.mutex );
                while( !h->encoders[i]->is_ready )
                    pthread_cond_wait( &h->encoders[i]->queue.in_cv, &h->encoders[i]->queue.mutex );
                x264_param_t *params = h->encoders[i]->encoder_params;
                temporal_vbv_size = av_rescale_q_rnd(
                (int64_t)params->rc.i_vbv_buffer_size * params->rc.f_vbv_buffer_init,
                (AVRational){1, params->rc.i_vbv_max_bitrate }, (AVRational){ 1, OBE_CLOCK }, AV_ROUND_UP );
                pthread_mutex_unlock( &h->encoders[i]->queue.mutex );
                break;
            }
        }
    }

    while( 1 )
    {
        pthread_mutex_lock( &h->mux_smoothing_queue.mutex );

        while( h->mux_smoothing_queue.size == num_muxed_data && !h->cancel_mux_smoothing_thread )
            pthread_cond_wait( &h->mux_smoothing_queue.in_cv, &h->mux_smoothing_queue.mutex );

        if( h->cancel_mux_smoothing_thread )
        {
            pthread_mutex_unlock( &h->mux_smoothing_queue.mutex );
            break;
        }

        num_muxed_data = h->mux_smoothing_queue.size;
        g_mux_smoother_last_item_count = num_muxed_data;

        /* Refill the buffer after a drop */
        pthread_mutex_lock( &h->drop_mutex );
        if( h->mux_drop )
        {
            syslog( LOG_INFO, "Mux smoothing buffer reset\n" );
            printf("[Mux-Smoother] smoothing buffer reset\n" );
            h->mux_drop = 0;
            av_fifo_reset( fifo_data );
            av_fifo_reset( fifo_pcr );
            buffer_complete = 0;
            start_clock = -1;

            /* Trash the entire input queue to avoid unwanted buildup
             * in noisy signal environments, leading to an eventual
             * OOM kill event.
             */
            for (int i = 0; i < num_muxed_data; i++) {
printf("removing item %d of %d\n", i, num_muxed_data);
                obe_muxed_data_t *md = h->mux_smoothing_queue.queue[0];
                destroy_muxed_data(md);
                remove_from_queue_without_lock(&h->mux_smoothing_queue);
            }
            num_muxed_data = 0;
            pthread_mutex_unlock(&h->mux_smoothing_queue.mutex);
            pthread_mutex_unlock(&h->drop_mutex);
            continue;
        }
        pthread_mutex_unlock( &h->drop_mutex );


        /* If we don't have enough transport data to fill temporal_vbv_size ticks (27MHz) of
         * of output, continue to wait.
         */
        if( !buffer_complete )
        {
printf("num_muxed_data %d\n", num_muxed_data);
            start_data = h->mux_smoothing_queue.queue[0];
            end_data = h->mux_smoothing_queue.queue[num_muxed_data-1];

            start_pcr = start_data->pcr_list[0];
            end_pcr = end_data->pcr_list[(end_data->len / 188)-1];
printf("end_pcr %" PRIi64 " - start_pcr %" PRIi64 " = %" PRIi64 "\n",
  end_pcr,
  start_pcr,
  end_pcr - start_pcr);
            if( end_pcr - start_pcr >= temporal_vbv_size )
            {
                buffer_complete = 1;
                start_clock = -1;
            }
            else
            {
printf("Not yet complete.\n");
                pthread_mutex_unlock( &h->mux_smoothing_queue.mutex );
                continue;
            }
        }

        const char *ts = obe_ascii_datetime();
        printf("[Mux-Smoother] smoothing %i frames @ %s\n", num_muxed_data, ts);
        free((void *)ts);

        /* Copy all queued frames into a new allocation. Is this credible if we have a massie queue? */
        /* Allocate an array of obe_muxed_data_t objects (muxed_data), clone the entire queue... */
        muxed_data = malloc( num_muxed_data * sizeof(*muxed_data) );
        if( !muxed_data )
        {
            pthread_mutex_unlock( &h->mux_smoothing_queue.mutex );
            syslog( LOG_ERR, "Malloc failed\n" );
            return NULL;
        }
        memcpy( muxed_data, h->mux_smoothing_queue.queue, num_muxed_data * sizeof(*muxed_data) );
        pthread_mutex_unlock( &h->mux_smoothing_queue.mutex );

        printf("[Mux-Smoother] fifo_data size %d, num_muxed_data %d\n", av_fifo_size(fifo_data), num_muxed_data);
        printf("[Mux-Smoother] start_pcr %" PRIi64 "  end_pcr %" PRIi64 "  cur_pcr %" PRIi64 " t_vbv_size %" PRIi64 "\n",
            start_pcr, end_pcr, cur_pcr, temporal_vbv_size);

        g_mux_smoother_fifo_pcr_size = av_fifo_size(fifo_pcr);
        g_mux_smoother_fifo_data_size = av_fifo_size(fifo_data);

        /* For every object we cloned... */
        /* Write the transport packets to the fifo_data fifo. */
        /* Write the associated PCR list to the fifo_pcr fifo. */
        /* Destroy the cloned copy, and the original on the queue. */
        g_mux_smoother_last_total_item_size = 0;
        for( int i = 0; i < num_muxed_data; i++ )
        {
            g_mux_smoother_last_total_item_size += muxed_data[i]->len;

            int len = av_fifo_size( fifo_data ) + muxed_data[i]->len;

            if (len > 4096000 || num_muxed_data > 80)
                printf("i = %d, len %d\n", i, len);

            if( av_fifo_realloc2( fifo_data, len ) < 0 )
            {
                syslog( LOG_ERR, "Malloc failed\n" );
                return NULL;
            }

            av_fifo_generic_write( fifo_data, muxed_data[i]->data, muxed_data[i]->len, NULL );

            if( av_fifo_realloc2( fifo_pcr, av_fifo_size( fifo_pcr ) + ((muxed_data[i]->len * sizeof(int64_t)) / 188) ) < 0 )
            {
                syslog( LOG_ERR, "Malloc failed\n" );
                return NULL;
            }

            av_fifo_generic_write( fifo_pcr, muxed_data[i]->pcr_list, (muxed_data[i]->len * sizeof(int64_t)) / 188, NULL );

            remove_from_queue( &h->mux_smoothing_queue );
            destroy_muxed_data( muxed_data[i] );
        }

        /* Drop the entire clone f the queue allocation. */
        free( muxed_data );
        muxed_data = NULL;
        num_muxed_data = 0;

        /* While we have atleast 7 transport packets in the TS packet fifo.... */
        while(!h->mux_drop && av_fifo_size( fifo_data ) >= TS_PACKETS_SIZE )
        {
            if (av_fifo_size( fifo_data ) > 10000000) {
                /* We won't want this much buffered content, lose it. */
                h->mux_drop = 1;
                continue;
            }
            /* allocate a buffer, of exactly 7 PCRs followed by 7 transport packets, drain the relevant fifos. */
            output_buffers[0] = av_buffer_alloc( TS_PACKETS_SIZE + 7 * sizeof(int64_t) );
            av_fifo_generic_read( fifo_pcr, output_buffers[0]->data, 7 * sizeof(int64_t), NULL );
            av_fifo_generic_read( fifo_data, &output_buffers[0]->data[7 * sizeof(int64_t)], TS_PACKETS_SIZE, NULL );

            /* Generally, we only ever have a single (IP transmitter) output, take a reference for each. */
            for( int i = 1; i < h->num_outputs; i++ )
            {
                output_buffers[i] = av_buffer_ref( output_buffers[0] );
                if( !output_buffers[i] )
                {
                    syslog( LOG_ERR, "Malloc failed\n" );
                    return NULL;
                }
            }

            /* Gather the most recent PCR. */
            cur_pcr = AV_RN64( output_buffers[0]->data );

            /* We never sleep after the upstream signal was lost
             * or once enough queue data has been gathered to fill vbv ticks.
             * We're sleeping for N 27Mhz ticks, essentially 'smoothing' the
             * IP output (and delaying this thread).
             */
            if( start_clock != -1 )
            {
                struct timeval then, now;
                gettimeofday(&then, NULL);
                sleep_input_clock( h, cur_pcr - start_pcr + start_clock );
                gettimeofday(&now, NULL);

                int64_t duration = (now.tv_sec - then.tv_sec) * 1000000;
                if (duration > 1000000) {
                    printf("duration %" PRIi64 "\n", duration);
                }
            }

            /* If we have a LOS upstream, or we've just received enough to fill a vbv period.... */
            if( start_clock == -1 )
            {
                start_clock = get_input_clock_in_mpeg_ticks( h );
                start_pcr = cur_pcr;
            }

            /* put the new output buffer(s) on all of the output interfaces. Typically only one. */
            for( int i = 0; i < h->num_outputs; i++ )
            {
                if( add_to_queue( &h->outputs[i]->queue, output_buffers[i] ) < 0 )
                    return NULL;
                output_buffers[i] = NULL;
            }
        }
    }

    av_fifo_free( fifo_data );
    av_fifo_free( fifo_pcr );
    free( output_buffers );

    return NULL;
}

const obe_smoothing_func_t mux_smoothing = { mux_start_smoothing };
