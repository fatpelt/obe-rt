/*****************************************************************************
 * lavc.c: libavcodec audio encoding functions
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
#include "common/lavc.h"
#include "encoders/audio/audio.h"
#include <libavutil/fifo.h>
#include <libavresample/avresample.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>

#define HWCLK 1
#define MODULE "[lavc]: "

int g_aac_cf_debug = 0;

typedef struct
{
    int obe_name;
    int lavc_name;
} lavc_encoder_t;

static const lavc_encoder_t lavc_encoders[] =
{
    { AUDIO_AC_3,   AV_CODEC_ID_AC3 },
    { AUDIO_E_AC_3, AV_CODEC_ID_EAC3 },
    { AUDIO_AAC,    AV_CODEC_ID_AAC },
    { -1, -1 },
};

static void *aac_start_encoder( void *ptr )
{
    obe_aud_enc_params_t *enc_params = ptr;
    obe_t *h = enc_params->h;
    obe_encoder_t *encoder = enc_params->encoder;
    obe_output_stream_t *stream = enc_params->stream;
    obe_raw_frame_t *raw_frame;
    obe_coded_frame_t *coded_frame;
    int64_t cur_pts = -1, pts_increment;
    int i, frame_size, ret, got_pkt, num_frames = 0, total_size = 0;
    AVFifoBuffer *out_fifo = NULL;
    AVAudioResampleContext *avr = NULL;
    AVPacket pkt;
    AVCodecContext *codec = NULL;
    AVFrame *frame = NULL;
    AVDictionary *opts = NULL;
    char is_latm[2];
    uint8_t *audio_planes[8] = { NULL };

#if HWCLK
    printf(MODULE "h/w clocking is enabled.\n");
    struct avfm_s avfm;
#endif
    int64_t lastOutputFramePTS = 0; /* Last pts we output, we'll comare against future version to warn for discontinuities. */

enc_params->use_fifo_head_timing = 1;
    if (enc_params->use_fifo_head_timing) {
        fprintf(stderr, MODULE "Warning: LAVC encoder does not support the use_fifo_head_timing mode\n");
    }

    avcodec_register_all();

    codec = avcodec_alloc_context3( NULL );
    if( !codec )
    {
        fprintf(stderr, MODULE "Malloc failed\n");
        goto finish;
    }

    for( i = 0; lavc_encoders[i].obe_name != -1; i++ )
    {
        if( lavc_encoders[i].obe_name == stream->stream_format )
            break;
    }

    if( lavc_encoders[i].obe_name == -1 )
    {
        fprintf(stderr, MODULE "Could not find encoder1\n");
        goto finish;
    }

    AVCodec *enc = avcodec_find_encoder( lavc_encoders[i].lavc_name );
    if( !enc )
    {
        fprintf(stderr, MODULE "Could not find encoder2\n");
        goto finish;
    }

    if( enc->sample_fmts[0] == -1 )
    {
        fprintf(stderr, MODULE "No valid sample formats\n");
        goto finish;
    }

    codec->sample_rate = enc_params->sample_rate;
    codec->bit_rate = stream->bitrate * 1000;
    codec->sample_fmt = enc->sample_fmts[0];
    codec->channels = av_get_channel_layout_nb_channels( stream->channel_layout );
    codec->channel_layout = stream->channel_layout;
    codec->time_base.num = 1;
    codec->time_base.den = OBE_CLOCK;
    codec->profile = stream->aac_opts.aac_profile == AAC_HE_V2 ? FF_PROFILE_AAC_HE_V2 :
                     stream->aac_opts.aac_profile == AAC_HE_V1 ? FF_PROFILE_AAC_HE :
                     FF_PROFILE_AAC_LOW;

    snprintf( is_latm, sizeof(is_latm), "%i", stream->aac_opts.latm_output );
    av_dict_set( &opts, "latm", is_latm, 0 );
    av_dict_set( &opts, "header_period", "2", 0 );

    if( avcodec_open2( codec, enc, &opts ) < 0 )
    {
        fprintf(stderr, MODULE "Could not open encoder\n");
        goto finish;
    }

    avr = avresample_alloc_context();
    if( !avr )
    {
        fprintf(stderr, MODULE "Malloc failed\n");
        goto finish;
    }

    av_opt_set_int( avr, "in_channel_layout",   codec->channel_layout, 0 );
    av_opt_set_int( avr, "in_sample_fmt",       enc_params->input_sample_format, 0 );
    av_opt_set_int( avr, "in_sample_rate",      enc_params->sample_rate, 0 );
    av_opt_set_int( avr, "out_channel_layout",  codec->channel_layout, 0 );
    av_opt_set_int( avr, "out_sample_fmt",      codec->sample_fmt,     0 );
    av_opt_set_int( avr, "dither_method",       AV_RESAMPLE_DITHER_TRIANGULAR_NS, 0 );

    if( avresample_open( avr ) < 0 )
    {
        fprintf(stderr, MODULE "Could not open AVResample\n");
        goto finish;
    }

    /* The number of samples per E-AC3 frame is unknown until the encoder is ready */
    if( stream->stream_format == AUDIO_E_AC_3 || stream->stream_format == AUDIO_AAC )
    {
        pthread_mutex_lock( &encoder->queue.mutex );
        encoder->is_ready = 1;
        encoder->num_samples = codec->frame_size;
        /* Broadcast because input and muxer can be stuck waiting for encoder */
        pthread_cond_broadcast( &encoder->queue.in_cv );
        pthread_mutex_unlock( &encoder->queue.mutex );
    }

    frame_size = (double)codec->frame_size * 125 * stream->bitrate *
                 enc_params->frames_per_pes / enc_params->sample_rate;
    /* NB: libfdk-aac already doubles the frame size appropriately */
    pts_increment = (double)codec->frame_size * OBE_CLOCK * enc_params->frames_per_pes / enc_params->sample_rate;

    out_fifo = av_fifo_alloc( frame_size );
    if( !out_fifo )
    {
        fprintf(stderr, MODULE "Malloc failed\n");
        goto finish;
    }

    frame = avcodec_alloc_frame();
    if( !frame )
    {
        fprintf(stderr, MODULE "Could not allocate frame\n");
        goto finish;
    }

    if( av_samples_alloc( audio_planes, NULL, codec->channels, codec->frame_size, codec->sample_fmt, 0 ) < 0 )
    {
        fprintf(stderr, MODULE "Could not allocate audio samples\n");
        goto finish;
    }

/* AAC has 1 frame per pes in lowest latency mode, frame size 1024. */
/* AAC has 6 frame per pes in normal latency mode, frame size 2048. */
printf(MODULE "frames per pes %d\n", enc_params->frames_per_pes);
printf(MODULE "codec frame size %d\n", codec->frame_size);

    while( 1 )
    {
        /* TODO: detect bitrate or channel reconfig */
        pthread_mutex_lock( &encoder->queue.mutex );

        while( !encoder->queue.size && !encoder->cancel_thread )
            pthread_cond_wait( &encoder->queue.in_cv, &encoder->queue.mutex );

        if( encoder->cancel_thread )
        {
            pthread_mutex_unlock( &encoder->queue.mutex );
            goto finish;
        }

        raw_frame = encoder->queue.queue[0];
#if HWCLK
        if (raw_frame->avfm.audio_pts - avfm.audio_pts >= (2 * 576000)) {
            cur_pts = -1; /* Reset the audio timebase from the hardware. */
        }
        memcpy(&avfm, &raw_frame->avfm, sizeof(avfm));
#endif

        pthread_mutex_unlock( &encoder->queue.mutex );

        if( cur_pts == -1 ) {
#if HWCLK
            /* Drain any fifos and zero our processing latency, the clock has been
             * reset so we're rebasing time from the audio hardward clock.
             */
            cur_pts = avfm.audio_pts;

            printf(MODULE "strm %d audio pts reset to %" PRIi64 "\n",
                encoder->output_stream_id,
                cur_pts);

            /* Drain the conversion fifos else we induce drift. */
            av_fifo_drain(out_fifo, av_fifo_size(out_fifo));
            avresample_read(avr, NULL, avresample_available(avr));
#else
            cur_pts = raw_frame->pts;
#endif
        }

        if( avresample_convert( avr, NULL, 0, raw_frame->audio_frame.num_samples, raw_frame->audio_frame.audio_data,
                                raw_frame->audio_frame.linesize, raw_frame->audio_frame.num_samples ) < 0 )
        {
            syslog(LOG_ERR, MODULE "Sample format conversion failed\n");
            break;
        }

        raw_frame->release_data( raw_frame );
        raw_frame->release_frame( raw_frame );
        remove_from_queue( &encoder->queue );

        /* While we have enough pcm samples to pass to the compressor... */
        while( avresample_available( avr ) >= codec->frame_size )
        {
            got_pkt = 0;
            avcodec_get_frame_defaults( frame );
            frame->nb_samples = codec->frame_size;
            memcpy( frame->data, audio_planes, sizeof(frame->data) );
            avresample_read( avr, frame->data, codec->frame_size );

            av_init_packet( &pkt );
            pkt.data = NULL;
            pkt.size = 0;

            /* Compress some PCM into the codec of choice... */
            ret = avcodec_encode_audio2( codec, &pkt, frame, &got_pkt );
            if( ret < 0 )
            {
                syslog(LOG_ERR, MODULE "Audio encoding failed\n");
                goto finish;
            }

            /* Continue until we have enough ooutput coded data for an entire downstream packet. */
            if( !got_pkt )
                continue;

            total_size += pkt.size;
            num_frames++;

            if( av_fifo_realloc2( out_fifo, av_fifo_size( out_fifo ) + pkt.size ) < 0 )
            {
                syslog(LOG_ERR, MODULE "Malloc failed\n");
                break;
            }

            /* Write the output codec data into a fifo, because we want to make downstream packets
             * of exactly N frames (frames_per_pes).
             */
            av_fifo_generic_write( out_fifo, pkt.data, pkt.size, NULL );
            obe_free_packet( &pkt );

            /* When we've written enough output frames to the fifo, process a complete downstream packet. */
            if( num_frames == enc_params->frames_per_pes )
            {
                coded_frame = new_coded_frame( encoder->output_stream_id, total_size );
                if( !coded_frame )
                {
                    syslog(LOG_ERR, MODULE "Malloc failed\n");
                    goto finish;
                }
                av_fifo_generic_read( out_fifo, coded_frame->data, total_size, NULL );
                coded_frame->pts = cur_pts;
#if HWCLK
                static int64_t ptsfixup = 0;
                if (getenv("HALF_DUPLEX") && ptsfixup == 0) {
                    /* Fixup the clock for 10.11.2, due to an audio clocking bug. */
                    ptsfixup  = 900900;
                    ptsfixup += (avfm.audio_pts - avfm.video_pts);
printf("HALF_DUPLEX=1(1) ... ptsfixup %" PRIi64 "\n", ptsfixup);
                    ptsfixup  = ptsfixup % 900900;
printf("HALF_DUPLEX=1(2) ... ptsfixup %" PRIi64 "\n", ptsfixup);

                    if ((avfm.audio_pts - avfm.video_pts) < 0 &&
                        ((avfm.audio_pts - avfm.video_pts) > -181000)) {
                        /* Round down, to avoid a one frame offset from video. */
                        ptsfixup = ptsfixup + - 900900;
                    }

                }
                /* We seem to be 33.2ms latent for 1080i, adjust it. Does this vary for low vs normal latency? */
                coded_frame->pts += (-33 * 27000LL);
                coded_frame->pts += (-2 * 2700LL);
                if (h->obe_system == OBE_SYSTEM_TYPE_GENERIC) {
                    coded_frame->pts += (8 * 2700LL);
                }
                coded_frame->pts += (ptsfixup * -1);
#endif
                coded_frame->pts += ((int64_t)stream->audio_offset_ms * 27000);
                coded_frame->random_access = 1; /* Every frame output is a random access point */
                coded_frame->type = CF_AUDIO;

                if (g_aac_cf_debug && encoder->output_stream_id == 1) {
                    double interval = coded_frame->pts - lastOutputFramePTS;
                    printf(MODULE "strm %d output pts %13" PRIi64 " size %6d bytes, pts-interval %6.0fticks/%6.2fms\n",
                        encoder->output_stream_id,
                        coded_frame->pts,
                        total_size,
                        interval,
                        interval / 27000.0);
                }
                if (lastOutputFramePTS + (576000 * enc_params->frames_per_pes) != coded_frame->pts) {
                    if (encoder->output_stream_id == 1) {
                    printf(MODULE "strm %d Output PTS discontinuity\n\tShould be %" PRIi64 " was %" PRIi64 " diff %9" PRIi64 " frames_per_pes %d\n",
                        encoder->output_stream_id,
                        lastOutputFramePTS + (576000 * enc_params->frames_per_pes),
                        coded_frame->pts,
                        coded_frame->pts - (lastOutputFramePTS + (576000 * enc_params->frames_per_pes)),
                        enc_params->frames_per_pes);
                    }
                }

                lastOutputFramePTS = coded_frame->pts;
                add_to_queue( &h->mux_queue, coded_frame );

                /* We need to generate PTS because frame sizes have changed */
                cur_pts += pts_increment;
                total_size = num_frames = 0;
            }
        }
    }

finish:
    if( frame )
       avcodec_free_frame( &frame );

    if( audio_planes[0] )
        av_free( audio_planes[0] );

    if( out_fifo )
        av_fifo_free( out_fifo );

    if( avr )
        avresample_free( &avr );

    if( codec )
    {
        avcodec_close( codec );
        av_free( codec );
    }

    free( enc_params );

    return NULL;
}

const obe_aud_enc_func_t lavc_encoder = { aac_start_encoder };
