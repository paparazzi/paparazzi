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

#include <jpeglib.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <encoders.h>
#include <conf_parse.h>

struct jpeg_encoder {
	struct stream *output;
	struct stream_destination *input;
	int format;
	struct frame_exchanger *ex;
	pthread_t thread;
};

static void init_destination( j_compress_ptr cinfo )
{
}

static boolean empty_output_buffer( j_compress_ptr cinfo )
{
	return TRUE;
}

static void term_destination( j_compress_ptr cinfo )
{
}

static void *jpeg_loop( void *d )
{
	struct jpeg_encoder *en = (struct jpeg_encoder *)d;
	struct frame *jpeg, *input;
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	struct jpeg_destination_mgr destmgr;
	JSAMPROW row_ptr[1];

	for(;;)
	{
		input = get_next_frame( en->ex, 1 );
		jpeg = new_frame();

		destmgr.next_output_byte = jpeg->d;
		destmgr.free_in_buffer = get_max_frame_size();
		destmgr.init_destination = init_destination;
		destmgr.empty_output_buffer = empty_output_buffer;
		destmgr.term_destination = term_destination;

		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_compress( &cinfo );
		cinfo.dest = &destmgr;
		cinfo.image_width = input->width;
		cinfo.image_height = input->height;
		cinfo.input_components = 3;
		cinfo.in_color_space = JCS_RGB;
		jpeg_set_defaults( &cinfo );
		jpeg_start_compress( &cinfo, TRUE );
		while( cinfo.next_scanline < cinfo.image_height )
		{
			row_ptr[0] = input->d +
					cinfo.next_scanline * input->width * 3;
			jpeg_write_scanlines( &cinfo, row_ptr, 1 );
		}
		jpeg_finish_compress( &cinfo );
		jpeg_destroy_compress( &cinfo );

		jpeg->format = FORMAT_JPEG;
		jpeg->width = input->width;
		jpeg->height = input->height;
		jpeg->key = 1;
		jpeg->length = get_max_frame_size() - destmgr.free_in_buffer;

		deliver_frame( en->ex, jpeg );
		unref_frame( input );
	}
	return NULL;
}

static void jpeg_encode( struct frame *input, void *d )
{
	struct jpeg_encoder *en = (struct jpeg_encoder *)d;

	exchange_frame( en->ex, input );
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct jpeg_encoder *en = (struct jpeg_encoder *)s->private;

	en->input->stream->get_framerate( en->input->stream, fincr, fbase );
}

static void set_running( struct stream *s, int running )
{
	struct jpeg_encoder *en = (struct jpeg_encoder *)s->private;

	set_waiting( en->input, running );
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct jpeg_encoder *en;

	en = (struct jpeg_encoder *)malloc( sizeof( struct jpeg_encoder ) );
	en->output = NULL;

	return en;
}

static int end_block( void *d )
{
	struct jpeg_encoder *en = (struct jpeg_encoder *)d;

	if( ! en->input )
	{
		spook_log( SL_ERR, "jpeg: missing input stream name" );
		return -1;
	}
	if( ! en->output )
	{
		spook_log( SL_ERR, "jpeg: missing output stream name" );
		return -1;
	}

	en->ex = new_exchanger( 8, deliver_frame_to_stream, en->output );
	pthread_create( &en->thread, NULL, jpeg_loop, en );

	return 0;
}

static int set_input( int num_tokens, struct token *tokens, void *d )
{
	struct jpeg_encoder *en = (struct jpeg_encoder *)d;
	int format = FORMAT_RAW_RGB24;

	if( ! ( en->input = connect_to_stream( tokens[1].v.str, jpeg_encode,
						en, &format, 1 ) ) )
	{
		spook_log( SL_ERR, "jpeg: unable to connect to stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct jpeg_encoder *en = (struct jpeg_encoder *)d;

	en->output = new_stream( tokens[1].v.str, FORMAT_JPEG, en );
	if( ! en->output )
	{
		spook_log( SL_ERR, "jpeg: unable to create stream \"%s\"",
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

int jpeg_init(void)
{
	register_config_context( "encoder", "jpeg", start_block, end_block,
					config_statements );
	return 0;
}
