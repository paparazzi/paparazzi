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

#include <xvid.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
// #include <decoders.h>
#include <conf_parse.h>

struct mpeg4_decoder {
	struct stream *output;
	struct stream_destination *input;
	struct frame_exchanger *ex;
	pthread_t decoding_thread;
	int running; /* only used by the main thread */

	int width;
	int height;

	/* reset_pending is used to indicate that there may have been a long
	 * break since the last frame, presumably because nobody was listening
	 * and the source stopped sending us frames, so the decoding thread
	 * should reset the decoder.  It doesn't really need synchronization
	 * because if we reset in the middle of a bunch of contiguous frames
	 * it's not the end of the world. */
	int reset_pending;

	/* All the parameters after the handle indicate the settings used
	 * to set up the current decoder.  Therefore, they are invalid
	 * when xvid_handle == NULL. */
	void *xvid_handle;
};

static void mpeg4_start( struct mpeg4_decoder *en, struct frame *f )
{
	xvid_dec_create_t xvid_dec_create;

	en->reset_pending = 0;

	memset( &xvid_dec_create, 0, sizeof( xvid_dec_create ) );
	xvid_dec_create.version = XVID_VERSION;
	xvid_dec_create.width = 0;
	xvid_dec_create.height = 0;

	if( xvid_decore( NULL, XVID_DEC_CREATE,
				&xvid_dec_create, NULL ) )
	{
		spook_log( SL_ERR, "mpeg4: unable to start XviD!" );
		return;
	}

	en->xvid_handle = xvid_dec_create.handle;
}

static void mpeg4_stop( struct mpeg4_decoder *en )
{
	spook_log( SL_DEBUG, "mpeg4: destroying mpeg4 decoder" );

	xvid_decore( en->xvid_handle, XVID_DEC_DESTROY, NULL, NULL );
	en->xvid_handle = NULL;
}

static void *mpeg4_loop( void *d )
{
	struct mpeg4_decoder *en = (struct mpeg4_decoder *)d;
	xvid_dec_frame_t xvid_dec_frame;
	xvid_dec_stats_t xvid_dec_stats;
	struct frame *out, *input;
	int used, pos;

	for(;;)
	{
		input = get_next_frame( en->ex, 1 );

		if( en->reset_pending && en->xvid_handle ) mpeg4_stop( en );
		if( ! en->xvid_handle ) mpeg4_start( en, input );

		out = new_frame();
		out->width = en->width;
		out->height = en->height;

		pos = 0;

		while( input->length - pos > 0 )
		{
			memset( &xvid_dec_frame, 0, sizeof( xvid_dec_frame ) );
			xvid_dec_frame.version = XVID_VERSION;
			xvid_dec_frame.general = 0;
			xvid_dec_frame.bitstream = input->d + pos;
			xvid_dec_frame.length = input->length - pos;
			xvid_dec_frame.output.plane[0] = out->d;
			xvid_dec_frame.output.stride[0] = 2 * out->width;
			xvid_dec_frame.output.csp = XVID_CSP_UYVY;
			xvid_dec_stats.version = XVID_VERSION;

			used = xvid_decore( en->xvid_handle, XVID_DEC_DECODE,
					&xvid_dec_frame, &xvid_dec_stats );
			if( used < 0 )
			{
				out->length = 0;
				spook_log( SL_WARN, "mpeg4: XviD decoding failed!" );
			}
			if( xvid_dec_stats.type == XVID_TYPE_VOL )
			{
				out->width = en->width = xvid_dec_stats.data.vol.width;
				out->height = en->height = xvid_dec_stats.data.vol.height;
			}
			pos += used;
		}

		out->format = FORMAT_RAW_UYVY;
		out->length = 2 * out->width * out->height;
		out->key = 1;

		deliver_frame( en->ex, out );

		unref_frame( input );
	}

	return NULL;
}

static void mpeg4_decode( struct frame *input, void *d )
{
	struct mpeg4_decoder *en = (struct mpeg4_decoder *)d;

	exchange_frame( en->ex, input );
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct mpeg4_decoder *en = (struct mpeg4_decoder *)s->private;

	en->input->stream->get_framerate( en->input->stream, fincr, fbase );
}

static void set_running( struct stream *s, int running )
{
	struct mpeg4_decoder *en = (struct mpeg4_decoder *)s->private;

	spook_log( SL_DEBUG,
		"mpeg4 decoder is told to set running to %d", running );

	if( ! en->running && running ) en->reset_pending = 1;
	set_waiting( en->input, running );
	en->running = running;
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct mpeg4_decoder *en;

	en = (struct mpeg4_decoder *)malloc( sizeof( struct mpeg4_decoder ) );
	en->output = NULL;
	en->xvid_handle = NULL;
	en->width = en->height = 0;

	return en;
}

static int end_block( void *d )
{
	struct mpeg4_decoder *en = (struct mpeg4_decoder *)d;

	if( ! en->output )
	{
		spook_log( SL_ERR, "mpeg4: missing output stream name" );
		return -1;
	}
	if( ! en->input )
	{
		spook_log( SL_ERR, "mpeg4: missing input stream name" );
		return -1;
	}

	en->ex = new_exchanger( 8, deliver_frame_to_stream, en->output );
	pthread_create( &en->decoding_thread, NULL, mpeg4_loop, en );

	return 0;
}

static int set_input( int num_tokens, struct token *tokens, void *d )
{
	struct mpeg4_decoder *en = (struct mpeg4_decoder *)d;
	int formats[1] = { FORMAT_MPEG4 };

	if( ! ( en->input = connect_to_stream( tokens[1].v.str, mpeg4_decode,
						en, formats, 1 ) ) )
	{
		spook_log( SL_ERR, "mpeg4: unable to connect to stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct mpeg4_decoder *en = (struct mpeg4_decoder *)d;

	en->output = new_stream( tokens[1].v.str, FORMAT_RAW_UYVY, en );
	if( ! en->output )
	{
		spook_log( SL_ERR, "mpeg4: unable to create stream \"%s\"",
			tokens[1].v.str );
		return -1;
	}
	en->output->get_framerate = get_framerate;
	en->output->set_running = set_running;
	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "input", set_input, 1, 1, { TOKEN_STR } },
	{ "output", set_output, 1, 1, { TOKEN_STR } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

int mpeg4_dec_init(void)
{
	xvid_gbl_init_t xvid_gbl_init;

	memset( &xvid_gbl_init, 0, sizeof( xvid_gbl_init ) );
	xvid_gbl_init.version = XVID_VERSION;
	xvid_global( NULL, XVID_GBL_INIT, &xvid_gbl_init, NULL );
	register_config_context( "decoder", "mpeg4", start_block, end_block,
					config_statements );
	return 0;
}
