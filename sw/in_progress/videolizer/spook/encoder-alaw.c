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

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <encoders.h>
#include <conf_parse.h>

struct alaw_encoder {
	struct stream *output;
	struct stream_destination *input;
};

static void alaw_encode( struct frame *pcm, void *d )
{
	struct alaw_encoder *en = (struct alaw_encoder *)d;
	struct frame *alaw;

	//alaw = new_frame( pcm->length / 2 );
	alaw = new_frame();
	alaw->format = FORMAT_ALAW;
	alaw->width = 0;
	alaw->height = 0;
	alaw->length = pcm->length / 2;
	alaw->key = 1;
	alaw_enc( pcm->d, alaw->d, pcm->length );
	unref_frame( pcm );
	deliver_frame_to_stream( alaw, en->output );
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct alaw_encoder *en = (struct alaw_encoder *)s->private;

	en->input->stream->get_framerate( en->input->stream, fincr, fbase );
}

static void set_running( struct stream *s, int running )
{
	struct alaw_encoder *en = (struct alaw_encoder *)s->private;

	set_waiting( en->input, running );
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct alaw_encoder *en;

	en = (struct alaw_encoder *)malloc( sizeof( struct alaw_encoder ) );
	en->input = NULL;
	en->output = NULL;

	return en;
}

static int end_block( void *d )
{
	struct alaw_encoder *en = (struct alaw_encoder *)d;

	if( ! en->input )
	{
		spook_log( SL_ERR, "alaw: missing input stream name" );
		return -1;
	}
	if( ! en->output )
	{
		spook_log( SL_ERR, "alaw: missing output stream name" );
		return -1;
	}

	return 0;
}

static int set_input( int num_tokens, struct token *tokens, void *d )
{
	struct alaw_encoder *en = (struct alaw_encoder *)d;
	int format = FORMAT_PCM;

	if( ! ( en->input = connect_to_stream( tokens[1].v.str, alaw_encode,
						en, &format, 1 ) ) )
	{
		spook_log( SL_ERR, "alaw: unable to connect to stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct alaw_encoder *en = (struct alaw_encoder *)d;

	en->output = new_stream( tokens[1].v.str, FORMAT_ALAW, en );
	if( ! en->output )
	{
		spook_log( SL_ERR, "alaw: unable to create stream \"%s\"",
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

int alaw_init(void)
{
	register_config_context( "encoder", "alaw", start_block, end_block,
					config_statements );
	return 0;
}
