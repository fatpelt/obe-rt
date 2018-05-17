/*****************************************************************************
 * twolame.c : twolame encoding functions
 *****************************************************************************
 * Copyright (C) 2010 Open Broadcast Systems Ltd.
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
 ******************************************************************************/

#include "common/common.h"
#include "encoders/audio/audio.h"
#include <twolame.h>
#include <libavutil/fifo.h>
#include <libavresample/avresample.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>

#define MP2_AUDIO_BUFFER_SIZE 50000

#define LOCAL_DEBUG 0

static void *start_encoder_mp2( void *ptr )
{
#if LOCAL_DEBUG
    printf("%s()\n", __func__);
#endif

    obe_aud_enc_params_t *enc_params = ptr;
    obe_t *h = enc_params->h;
    obe_encoder_t *encoder = enc_params->encoder;
    obe_output_stream_t *stream = enc_params->stream;
    obe_raw_frame_t *raw_frame;
    obe_coded_frame_t *coded_frame;

#if LOCAL_DEBUG
    printf("%s() output_stream_id = %d\n", __func__, encoder->output_stream_id);
#endif

    twolame_options *tl_opts = NULL;
    int output_size, frame_size, linesize; /* Linesize in libavresample terminology is the entire buffer size for packed formats */
    float *audio_buf = NULL;
    uint8_t *output_buf = NULL;
    AVAudioResampleContext *avr = NULL;
    AVFifoBuffer *fifo = NULL;

    /* Lock the mutex until we verify parameters */
    pthread_mutex_lock( &encoder->queue.mutex );

    tl_opts = twolame_init();
    if( !tl_opts )
    {
        fprintf( stderr, "[twolame] could not load options" );
        pthread_mutex_unlock( &encoder->queue.mutex );
        goto end;
    }

    /* TODO: setup bitrate reconfig, errors */
    twolame_set_bitrate( tl_opts, stream->bitrate );
    twolame_set_in_samplerate( tl_opts, enc_params->sample_rate );
    twolame_set_out_samplerate( tl_opts, enc_params->sample_rate );
    twolame_set_copyright( tl_opts, 1 );
    twolame_set_original( tl_opts, 1 );
    twolame_set_num_channels( tl_opts, av_get_channel_layout_nb_channels( stream->channel_layout ) );
    twolame_set_error_protection( tl_opts, 1 );
    if( stream->channel_layout == AV_CH_LAYOUT_STEREO )
        twolame_set_mode( tl_opts, stream->mp2_mode-1 );

    twolame_init_params( tl_opts );

    frame_size = twolame_get_framelength( tl_opts ) * enc_params->frames_per_pes;

    encoder->is_ready = 1;
    /* Broadcast because input and muxer can be stuck waiting for encoder */
    pthread_cond_broadcast( &encoder->queue.in_cv );
    pthread_mutex_unlock( &encoder->queue.mutex );

    output_buf = malloc( MP2_AUDIO_BUFFER_SIZE );
    if( !output_buf )
    {
        fprintf( stderr, "Malloc failed\n" );
        goto end;
    }

    avr = avresample_alloc_context();
    if( !avr )
    {
        fprintf( stderr, "Malloc failed\n" );
        goto end;
    }

    av_opt_set_int( avr, "in_channel_layout",   stream->channel_layout,  0 );
    av_opt_set_int( avr, "in_sample_fmt",       enc_params->input_sample_format, 0 );
    av_opt_set_int( avr, "in_sample_rate",      enc_params->sample_rate, 0 );
    av_opt_set_int( avr, "out_channel_layout",  stream->channel_layout, 0 );
    av_opt_set_int( avr, "out_sample_fmt",      AV_SAMPLE_FMT_FLT,   0 );
    av_opt_set_int( avr, "dither_method",       AV_RESAMPLE_DITHER_TRIANGULAR_NS, 0 );

    if( avresample_open( avr ) < 0 )
    {
        fprintf( stderr, "Could not open AVResample\n" );
        goto end;
    }

    /* Setup the output FIFO */
    fifo = av_fifo_alloc( frame_size );
    if( !fifo )
    {
        fprintf( stderr, "Malloc failed\n" );
        goto end;
    }

    while( 1 )
    {
        pthread_mutex_lock( &encoder->queue.mutex );

        while( !encoder->queue.size && !encoder->cancel_thread )
            pthread_cond_wait( &encoder->queue.in_cv, &encoder->queue.mutex );

        if( encoder->cancel_thread )
        {
            pthread_mutex_unlock( &encoder->queue.mutex );
            break;
        }

        raw_frame = encoder->queue.queue[0];
        pthread_mutex_unlock( &encoder->queue.mutex );

#if LOCAL_DEBUG
        /* Send any audio to the AC3 frame slicer.
         * Push the buffer starting at the channel containing bitstream, and span 2 channels,
         * we'll get called back with a completely aligned, crc'd and valid AC3 frame.
         */
        printf("%s() output_stream_id = %d linesize = %d, num_samples = %d, num_channels = %d, sample_fmt = %d, raw_frame->input_stream_id = %d\n",
                __func__,
                encoder->output_stream_id,
                raw_frame->audio_frame.linesize,
                raw_frame->audio_frame.num_samples,
                raw_frame->audio_frame.num_channels,
                raw_frame->audio_frame.sample_fmt,
                raw_frame->input_stream_id);
#endif

        /* Allocate the output buffer */
        if( av_samples_alloc( (uint8_t**)&audio_buf, &linesize, av_get_channel_layout_nb_channels( raw_frame->audio_frame.channel_layout ),
                              raw_frame->audio_frame.linesize, AV_SAMPLE_FMT_FLT, 0 ) < 0 )
        {
            syslog( LOG_ERR, "Malloc failed\n" );
            goto end;
        }

        if( avresample_convert( avr, NULL, 0, raw_frame->audio_frame.num_samples, raw_frame->audio_frame.audio_data,
                                raw_frame->audio_frame.linesize, raw_frame->audio_frame.num_samples ) < 0 )
        {
            syslog( LOG_ERR, "[twolame] Sample format conversion failed\n" );
            break;
        }

        avresample_read( avr, (uint8_t**)&audio_buf, avresample_available( avr ) );

        output_size = twolame_encode_buffer_float32_interleaved( tl_opts, audio_buf, raw_frame->audio_frame.num_samples, output_buf, MP2_AUDIO_BUFFER_SIZE );

        if( output_size < 0 )
        {
            syslog( LOG_ERR, "[twolame] Encode failed\n" );
            break;
        }

        free( audio_buf );
        audio_buf = NULL;

        struct avfm_s avfm;
        memcpy(&avfm, &raw_frame->avfm, sizeof(avfm));

        raw_frame->release_data( raw_frame );
        raw_frame->release_frame( raw_frame );
        remove_from_queue( &encoder->queue );

        if( av_fifo_realloc2( fifo, av_fifo_size( fifo ) + output_size ) < 0 )
        {
            syslog( LOG_ERR, "Malloc failed\n" );
            break;
        }

        av_fifo_generic_write( fifo, output_buf, output_size, NULL );

        while( av_fifo_size( fifo ) >= frame_size )
        {
            coded_frame = new_coded_frame( encoder->output_stream_id, frame_size );
            if( !coded_frame )
            {
                syslog( LOG_ERR, "Malloc failed\n" );
                goto end;
            }
            av_fifo_generic_read( fifo, coded_frame->data, frame_size, NULL );
            memcpy(&coded_frame->avfm, &avfm, sizeof(avfm));

            /* a = (27000000 / 1000) * 96 - Calculate 96ms in units of a 27MHz clock.
             * a = 2592000
             * Output PTS packets (27MHz) are rounded to nearest 96ms interval on a 27MHz clock.
             */
            /* The outgoing PTS is rounded and based on recent h/w clock, so massive leaps in
             * the h/w clock are automatically compensated for.
             */
            int64_t rounded_pts = av_rescale(coded_frame->avfm.audio_pts, OBE_CLOCK, 2592000);
            rounded_pts /= OBE_CLOCK;
            rounded_pts *= 2592000;
            coded_frame->pts = rounded_pts;
            coded_frame->pts += get_param_audio_offset_ticks(h);

            coded_frame->random_access = 1; /* Every frame output is a random access point */
            coded_frame->type = CF_AUDIO;

            add_to_queue( &h->mux_queue, coded_frame );
        }
    }

end:
    if( output_buf )
        free( output_buf );

    if( audio_buf )
        free( audio_buf );

    if( avr )
        avresample_free( &avr );

    if( fifo )
        av_fifo_free( fifo );

    if( tl_opts )
        twolame_close( &tl_opts );
    free( enc_params );

    return NULL;
}

const obe_aud_enc_func_t twolame_encoder = { start_encoder_mp2 };
