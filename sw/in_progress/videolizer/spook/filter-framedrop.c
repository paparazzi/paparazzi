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
#include <pthread.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <filters.h>
#include <conf_parse.h>

struct framedropper {
	struct stream *output;
	struct stream_destination *input;
	int scale;
	int count;
};

static void do_framedrop( struct frame *input, void *d )
{
	struct framedropper *drop = (struct framedropper *)d;

	if( drop->count == 0 ) deliver_frame_to_stream( input, drop->output );
	else unref_frame( input );

	if( ++drop->count == drop->scale ) drop->count = 0;
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct framedropper *drop = (struct framedropper *)s->private;

	drop->input->stream->get_framerate( drop->input->stream, fincr, fbase );
	*fincr *= drop->scale;
}

static void set_running( struct stream *s, int running )
{
	struct framedropper *drop = (struct framedropper *)s->private;

	set_waiting( drop->input, running );
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct framedropper *drop;

	drop = (struct framedropper *)malloc( sizeof( struct framedropper ) );
	drop->output = 0;
	drop->scale = 0;
	drop->count = 0;

	return drop;
}

static int end_block( void *d )
{
	struct framedropper *drop = (struct framedropper *)d;

	if( ! drop->input )
	{
		spook_log( SL_ERR,
			"framedrop: missing input stream name" );
		return -1;
	}
	if( ! drop->output )
	{
		spook_log( SL_ERR,
			"framedrop: missing output stream name" );
		return -1;
	}
	if( drop->scale < 1 )
	{
		spook_log( SL_ERR,
			"framedrop: missing scale factor" );
		return -1;
	}

	return 0;
}

static int set_input( int num_tokens, struct token *tokens, void *d )
{
	struct framedropper *drop = (struct framedropper *)d;

	if( ! ( drop->input = connect_to_stream( tokens[1].v.str, do_framedrop,
						drop, NULL, 0 ) ) )
	{
		spook_log( SL_ERR,
			"framedrop: unable to connect to stream \"%s\"\n",
				tokens[1].v.str );
		return -1;
	}
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct framedropper *drop = (struct framedropper *)d;

	if( ! drop->input )
	{
		spook_log( SL_ERR,
			"framedrop: input must be specified before output" );
		return -1;
	}
	drop->output = new_stream( tokens[1].v.str,
					drop->input->stream->format, drop );
	if( ! drop->output )
	{
		spook_log( SL_ERR,
			"framedrop: unable to create stream \"%s\"",
			tokens[1].v.str );
		return -1;
	}
	drop->output->get_framerate = get_framerate;
	drop->output->set_running = set_running;
	return 0;
}

static int set_scale( int num_tokens, struct token *tokens, void *d )
{
	struct framedropper *drop = (struct framedropper *)d;

	if( tokens[1].v.num < 1 )
	{
		spook_log( SL_ERR,
			"framedrop: scale factor cannot be less than 1!" );
		return -1;
	}
	drop->scale = tokens[1].v.num;
	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "input", set_input, 1, 1, { TOKEN_STR } },
	{ "output", set_output, 1, 1, { TOKEN_STR } },
	{ "scale", set_scale, 1, 1, { TOKEN_NUM } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

int framedrop_init(void)
{
	register_config_context( "filter", "framedrop", start_block, end_block,
					config_statements );
	return 0;
}
