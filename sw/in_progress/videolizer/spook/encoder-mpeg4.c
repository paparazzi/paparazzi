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
#include <encoders.h>
#include <conf_parse.h>

struct mpeg4_encoder {
	struct stream *output;
	struct stream_destination *input;
	struct frame_exchanger *ex;
	pthread_t encoding_thread;
	int running; /* only used by the main thread */

	/* parameters from the config file */
	int bitrate;

	/* reset_pending is used to indicate that there may have been a long
	 * break since the last frame, presumably because nobody was listening
	 * and the source stopped sending us frames, so the encoding thread
	 * should reset the encoder.  It doesn't really need synchronization
	 * because if we reset in the middle of a bunch of contiguous frames
	 * it's not the end of the world. */
	int reset_pending;

	/* All the parameters after the handle indicate the settings used
	 * to set up the current encoder.  Therefore, they are invalid
	 * when xvid_handle == NULL. */
	void *xvid_handle;
	int width;
	int height;
};

static void mpeg4_start( struct mpeg4_encoder *en, struct frame *f )
{
	xvid_enc_create_t xvid_enc_create;
	xvid_enc_plugin_t plugins[1];
	xvid_plugin_single_t single;

	en->reset_pending = 0;
	en->width = f->width;
	en->height = f->height;

	memset( &xvid_enc_create, 0, sizeof( xvid_enc_create ) );
	xvid_enc_create.version = XVID_VERSION;
	xvid_enc_create.width = en->width;
	xvid_enc_create.height = en->height;
	xvid_enc_create.profile = XVID_PROFILE_ARTS_L4;
	en->input->stream->get_framerate( en->input->stream,
						&xvid_enc_create.fincr,
						&xvid_enc_create.fbase );
	spook_log( SL_DEBUG, "creating mpeg4 encoder with fincr=%d fbase=%d",
			xvid_enc_create.fincr, xvid_enc_create.fbase );
	xvid_enc_create.zones = NULL;
	xvid_enc_create.num_zones = 0;
	xvid_enc_create.plugins = plugins;
	xvid_enc_create.num_plugins = 1;
	xvid_enc_create.num_threads = 0;
	xvid_enc_create.max_key_interval = 300;
	xvid_enc_create.max_bframes = 0;
	xvid_enc_create.bquant_ratio = 150;
	xvid_enc_create.bquant_offset = 100;
	xvid_enc_create.frame_drop_ratio = 0;
	xvid_enc_create.global = 0;

	memset( &single, 0, sizeof( single ) );
	single.version = XVID_VERSION;
	single.bitrate = en->bitrate * 1000;
	plugins[0].func = xvid_plugin_single;
	plugins[0].param = &single;

	if( xvid_encore( NULL, XVID_ENC_CREATE,
				&xvid_enc_create, NULL ) )
	{
		spook_log( SL_ERR, "mpeg4: unable to start XviD!" );
		return;
	}

	en->xvid_handle = xvid_enc_create.handle;
}

static void mpeg4_stop( struct mpeg4_encoder *en )
{
	spook_log( SL_DEBUG, "mpeg4: destroying mpeg4 encoder" );

	xvid_encore( en->xvid_handle, XVID_ENC_DESTROY, NULL, NULL );
	en->xvid_handle = NULL;
}

static void *mpeg4_loop( void *d )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)d;
	xvid_enc_frame_t xvid_enc_frame;
	struct frame *mpeg, *input;

	for(;;)
	{
		input = get_next_frame( en->ex, 1 );

		if( en->reset_pending && en->xvid_handle ) mpeg4_stop( en );
		if( ! en->xvid_handle ) mpeg4_start( en, input );

		if( input->width != en->width || input->height != en->height )
		{
			spook_log( SL_WARN,
				"mpeg4: image size changed midstream!" );
			unref_frame( input );
			continue;
		}

		mpeg = new_frame();

		memset( &xvid_enc_frame, 0, sizeof( xvid_enc_frame ) );
		xvid_enc_frame.version = XVID_VERSION;
		xvid_enc_frame.bitstream = mpeg->d;
		xvid_enc_frame.length = -1;
		xvid_enc_frame.input.plane[0] = input->d;
		switch( input->format )
		{
		case FORMAT_RAW_BGR24:
			xvid_enc_frame.input.csp = XVID_CSP_BGR;
			xvid_enc_frame.input.stride[0] = en->width * 3;
			break;
		case FORMAT_RAW_UYVY:
			xvid_enc_frame.input.csp = XVID_CSP_UYVY;
			xvid_enc_frame.input.stride[0] = en->width * 2;
			break;
		}
		xvid_enc_frame.vol_flags = 0;
		xvid_enc_frame.vop_flags = 0;
		xvid_enc_frame.type = XVID_TYPE_AUTO;
		xvid_enc_frame.quant = 0;
		xvid_enc_frame.motion = XVID_ME_ADVANCEDDIAMOND16;
		xvid_enc_frame.quant_intra_matrix = NULL;
		xvid_enc_frame.quant_inter_matrix = NULL;

		mpeg->length = xvid_encore( en->xvid_handle, XVID_ENC_ENCODE,
					&xvid_enc_frame, NULL );
		if( mpeg->length < 0 )
		{
			mpeg->length = 0;
			spook_log( SL_WARN, "mpeg4: XviD encoding failed!" );
		}

		mpeg->format = FORMAT_MPEG4;
		mpeg->width = en->width;
		mpeg->height = en->height;
		mpeg->key = xvid_enc_frame.out_flags & XVID_KEYFRAME;

		deliver_frame( en->ex, mpeg );

		unref_frame( input );
	}

	return NULL;
}

static void mpeg4_encode( struct frame *input, void *d )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)d;

	exchange_frame( en->ex, input );
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)s->private;

	en->input->stream->get_framerate( en->input->stream, fincr, fbase );
}

static void set_running( struct stream *s, int running )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)s->private;

	spook_log( SL_DEBUG,
		"mpeg4 encoder is told to set running to %d", running );

	if( ! en->running && running ) en->reset_pending = 1;
	set_waiting( en->input, running );
	en->running = running;
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct mpeg4_encoder *en;

	en = (struct mpeg4_encoder *)malloc( sizeof( struct mpeg4_encoder ) );
	en->output = NULL;
	en->xvid_handle = NULL;
	en->bitrate = -1;

	return en;
}

static int end_block( void *d )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)d;

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
	if( en->bitrate < 0 )
	{
		spook_log( SL_ERR, "mpeg4: bitrate must be specified" );
		return -1;
	}

	en->ex = new_exchanger( 8, deliver_frame_to_stream, en->output );
	pthread_create( &en->encoding_thread, NULL, mpeg4_loop, en );

	return 0;
}

static int set_input( int num_tokens, struct token *tokens, void *d )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)d;
	int formats[2] = { FORMAT_RAW_UYVY, FORMAT_RAW_BGR24 };

	if( ! ( en->input = connect_to_stream( tokens[1].v.str, mpeg4_encode,
						en, formats, 2 ) ) )
	{
		spook_log( SL_ERR, "mpeg4: unable to connect to stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)d;

	en->output = new_stream( tokens[1].v.str, FORMAT_MPEG4, en );
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

static int set_bitrate( int num_tokens, struct token *tokens, void *d )
{
	struct mpeg4_encoder *en = (struct mpeg4_encoder *)d;

	if( tokens[1].v.num < 10 || tokens[1].v.num > 4000 )
	{
		spook_log( SL_ERR,
			"mpeg4: bitrate must be between 10 and 4000" );
		return -1;
	}
	en->bitrate = tokens[1].v.num;
	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "input", set_input, 1, 1, { TOKEN_STR } },
	{ "output", set_output, 1, 1, { TOKEN_STR } },
	{ "bitrate", set_bitrate, 1, 1, { TOKEN_NUM } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

int mpeg4_init(void)
{
	xvid_gbl_init_t xvid_gbl_init;

	memset( &xvid_gbl_init, 0, sizeof( xvid_gbl_init ) );
	xvid_gbl_init.version = XVID_VERSION;
	xvid_global( NULL, XVID_GBL_INIT, &xvid_gbl_init, NULL );
	register_config_context( "encoder", "mpeg4", start_block, end_block,
					config_statements );
	return 0;
}
