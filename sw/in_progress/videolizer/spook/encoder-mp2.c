/*
 * Copyright (C) 2004 Nathan Lutchansky <lutchann@litech.org>
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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>

#include <mpegaudio.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <encoders.h>
#include <conf_parse.h>

struct mp2_encoder {
	struct stream *output;
	struct stream_destination *input;
	MpegAudioContext ctx;
	int samprate;
	int channels;
	int bitrate;
	struct soft_queue *inq;
	struct soft_queue *outq;
	pthread_t thread;
};

static const int bitrateII_tab[] = { -1, 32000, 48000, 56000, 64000,
				80000, 96000, 112000, 128000, 160000,
				192000, 224000, 256000, 320000, 384000, -1 };
static const int samrate_tab[] = { 44100, 48000, 32000, -1 };

int MPA_encode_init(MpegAudioContext *s, int freq, int bitrate, int channels);
int MPA_encode_frame(MpegAudioContext *s, unsigned char *frame, int buf_size,
				unsigned char *sampbuf, int step);

static void *mp2_loop( void *data )
{
	struct mp2_encoder *en = (struct mp2_encoder *)data;
	struct frame *mp2, *in;
	unsigned char *pcm, *pcmbuf;
	int maxlen, off = 0, s, d, c;

	pcmbuf = (unsigned char *)malloc( 2 * en->channels * 1152 );
	maxlen = 1152 / 8 *
		bitrateII_tab[en->bitrate] / samrate_tab[en->samprate] + 1;

	in = get_next_event( en->inq );

	for(;;)
	{
		if( ( in->length - off ) < 1152 * in->step )
		{
			int split = ( in->length - off ) / in->step;

			/* Copy the samples from off to length into pcmbuf */
			for( c = 0; c < en->channels; ++c )
				for( s = off + ( c << 1 ), d = c << 1;
						s < in->length;
						s += in->step,
							d += en->channels << 1 )
				{
					pcmbuf[d] = in->d[s];
					pcmbuf[d + 1] = in->d[s + 1];
				}
			/* Get rid of old frame and get new frame */
			unref_frame( in );
			in = get_next_event( en->inq );
			/* Set off to be the offset *after* we get the partial
			 * frame from the beginning */
			off = ( 1152 - split ) * in->step;
			/* Copy the samples from 0 to off into pcmbuf */
			for( c = 0; c < en->channels; ++c )
				for( s = c << 1, d = ( ( split *
							en->channels ) << 1 ) +
							( c << 1 );
						s < off;
						s += in->step,
							d += en->channels << 1 )
				{
					pcmbuf[d] = in->d[s];
					pcmbuf[d + 1] = in->d[s + 1];
				}
			pcm = pcmbuf;
		} else
		{
			pcm = in->d + off;
			off += 1152 * in->step;
		}

		if( ! ( mp2 = new_frame() ) ) continue;
		mp2->format = FORMAT_MPA;
		mp2->width = mp2->height = 0;
		mp2->key = 1;
		mp2->length = MPA_encode_frame( &en->ctx, mp2->d, maxlen, pcm,
			pcm == pcmbuf ? ( en->channels << 1 ) : in->step );
		if( soft_queue_add( en->outq, mp2 ) < 0 ) unref_frame( mp2 );
	}
	return NULL;
}

static void get_back_frame( struct event_info *ei, void *d )
{
	struct mp2_encoder *en = (struct mp2_encoder *)d;
	struct frame *f = (struct frame *)ei->data;

	deliver_frame_to_stream( f, en->output );
}

static void mp2_encode( struct frame *input, void *d )
{
	struct mp2_encoder *en = (struct mp2_encoder *)d;

	if( soft_queue_add( en->inq, input ) < 0 ) unref_frame( input );
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct mp2_encoder *en = (struct mp2_encoder *)s->private;

	en->input->stream->get_framerate( en->input->stream, fincr, fbase );
}

static void set_running( struct stream *s, int running )
{
	struct mp2_encoder *en = (struct mp2_encoder *)s->private;

	set_waiting( en->input, running );
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct mp2_encoder *en;

	en = (struct mp2_encoder *)malloc( sizeof( struct mp2_encoder ) );
	en->output = NULL;
	en->bitrate = 0;

	return en;
}

static int end_block( void *d )
{
	struct mp2_encoder *en = (struct mp2_encoder *)d;
	int fincr, fbase;

	if( ! en->input )
	{
		spook_log( SL_ERR, "mp2: missing input stream name" );
		return -1;
	}
	if( ! en->output )
	{
		spook_log( SL_ERR, "mp2: missing output stream name" );
		return -1;
	}

	en->input->stream->get_framerate( en->input->stream, &fincr, &fbase );

	en->channels = fincr;

	switch( fbase / fincr )
	{
	case 44100:
		en->samprate = 0;
		break;
	case 48000:
		en->samprate = 1;
		break;
	case 32000:
		en->samprate = 2;
		break;
	default:
		spook_log( SL_ERR,
			"mp2: sample rate %d cannot be encoded to MP2",
			fbase / fincr );
		return -1;
	}

	if( bitrateII_tab[en->bitrate] < 0 )
	{
		spook_log( SL_ERR, "mp2: no bitrate specified!" );
		return -1;
	}

	if( MPA_encode_init( &en->ctx, samrate_tab[en->samprate],
			bitrateII_tab[en->bitrate], en->channels ) < 0 )
	{
		spook_log( SL_ERR, "mp2: unable to initialize MPEG encoder" );
		return -1;
	}

	en->inq = new_soft_queue( 16 );
	en->outq = new_soft_queue( 16 );
	add_softqueue_event( en->outq, 0, get_back_frame, en );
	pthread_create( &en->thread, NULL, mp2_loop, en );

	return 0;
}

static int set_input( int num_tokens, struct token *tokens, void *d )
{
	struct mp2_encoder *en = (struct mp2_encoder *)d;
	int format = FORMAT_PCM;

	if( ! ( en->input = connect_to_stream( tokens[1].v.str, mp2_encode,
						en, &format, 1 ) ) )
	{
		spook_log( SL_ERR, "mp2: unable to connect to stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct mp2_encoder *en = (struct mp2_encoder *)d;

	en->output = new_stream( tokens[1].v.str, FORMAT_MPA, en );
	if( ! en->output )
	{
		spook_log( SL_ERR, "mp2: unable to create stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	en->output->get_framerate = get_framerate;
	en->output->set_running = set_running;
	return 0;
}

static int set_bitrate( int num_tokens, struct token *tokens, void *d )
{
	struct mp2_encoder *en = (struct mp2_encoder *)d;

	if( tokens[1].v.num < 1000 ) tokens[1].v.num *= 1000;

	for( en->bitrate = 1; bitrateII_tab[en->bitrate] > 0; ++en->bitrate )
		if( bitrateII_tab[en->bitrate] == tokens[1].v.num )
			return 0;

	spook_log( SL_ERR, "mp2: invalid bitrate %d", tokens[1].v.num );
	return -1;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "input", set_input, 1, 1, { TOKEN_STR } },
	{ "output", set_output, 1, 1, { TOKEN_STR } },
	{ "bitrate", set_bitrate, 1, 1, { TOKEN_NUM } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

int mp2_init(void)
{
	register_config_context( "encoder", "mp2", start_block, end_block,
					config_statements );
	return 0;
}
