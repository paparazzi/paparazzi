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

#include <config.h>

#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <pthread.h>

#include <linux/soundcard.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <inputs.h>
#include <conf_parse.h>

struct oss_input {
	struct stream *output;
	char device[256];
	struct soft_queue *queue;
	struct audio_ring *ring;
	int fd;
	int rate;
	int channels;
	pthread_t thread;
	int running;
};

static void *capture_loop( void *d )
{
	struct oss_input *conf = (struct oss_input *)d;
	unsigned char *buf;
	int len = 0, ret, blocksize;

	blocksize = conf->rate * conf->channels * 2 / 50;

	spook_log( SL_VERBOSE, "input-oss: blocksize is %d", blocksize );
	buf = malloc( blocksize );

	for(;;)
	{
		ret = read( conf->fd, buf + len, blocksize - len );
		if( ret < 0 )
		{
			perror( "read" );
			break;
		}
		len += ret;
		if( len == blocksize )
		{
			audio_ring_input( conf->ring, buf, len );
			len = 0;
		}
	}
	return NULL;
}

static void get_back_frame( struct event_info *ei, void *d )
{
	struct oss_input *conf = (struct oss_input *)d;
	struct frame *f = (struct frame *)ei->data;

	deliver_frame_to_stream( f, conf->output );
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct oss_input *conf = (struct oss_input *)s->private;

	*fincr = conf->channels;
	*fbase = conf->rate * conf->channels;
}

static void set_running( struct stream *s, int running )
{
	struct oss_input *conf = (struct oss_input *)s->private;

	conf->running = running;
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct oss_input *conf;

	conf = (struct oss_input *)malloc( sizeof( struct oss_input ) );
	conf->output = NULL;
	conf->device[0] = 0;
	conf->rate = 0;
	conf->channels = 2;
	conf->running = 0;

	return conf;
}

static int end_block( void *d )
{
	struct oss_input *conf = (struct oss_input *)d;
	int i;

	if( ! conf->output )
	{
		spook_log( SL_ERR, "oss: missing output stream name" );
		return -1;
	}
	if( ! conf->device[0] )
	{
		spook_log( SL_ERR, "oss: missing DSP device name" );
		return -1;
	}
	if( conf->rate == 0 )
	{
		spook_log( SL_ERR, "oss: sample rate not specified" );
		return -1;
	}
	if( ( conf->fd = open( conf->device, O_RDONLY ) ) < 0 )
	{
		spook_log( SL_ERR, "oss: unable to open %s: %s", conf->device,
				strerror( errno ) );
		return -1;
	}
	if( ioctl( conf->fd, SNDCTL_DSP_GETFMTS, &i ) < 0 )
	{
		spook_log( SL_ERR,
			"oss: unable to query available sample formats" );
		return -1;
	}
	if( ! ( i & AFMT_S16_BE ) )
	{
		spook_log( SL_ERR,
			"oss: device does not support any usable sample formats" );
		return -1;
	}
	i = AFMT_S16_BE;
	if( ioctl( conf->fd, SNDCTL_DSP_SETFMT, &i ) < 0 )
	{
		spook_log( SL_ERR, "oss: unable to set sample format" );
		return -1;
	}
	i = conf->rate;
	if( ioctl( conf->fd, SNDCTL_DSP_SPEED, &i ) < 0 )
	{
		spook_log( SL_ERR, "oss: unable to set sample rate" );
		return -1;
	}
	i = 1;
	if( ioctl( conf->fd, SNDCTL_DSP_STEREO, &i ) < 0 )
	{
		spook_log( SL_ERR, "oss: unable to set channel count" );
		return -1;
	}
	/* make sure channel count is right!!! */
	if( ioctl( conf->fd, SOUND_PCM_READ_CHANNELS, &i ) < 0 )
	{
		spook_log( SL_ERR, "oss: unable to set channel count" );
		return -1;
	}
	conf->queue = new_soft_queue( 16 );
	add_softqueue_event( conf->queue, 0, get_back_frame, conf );
	/* Set frame length to 4608, which is the size of the blocks that
	 * the MP2 encoder will need.  This is just temporary... */
	conf->ring = new_audio_ring( 2 * conf->channels, conf->rate,
					4608, conf->queue );
	pthread_create( &conf->thread, NULL, capture_loop, conf );

	return 0;
}

static int set_device( int num_tokens, struct token *tokens, void *d )
{
	struct oss_input *conf = (struct oss_input *)d;

	strcpy( conf->device, tokens[1].v.str );
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct oss_input *conf = (struct oss_input *)d;

	conf->output = new_stream( tokens[1].v.str, FORMAT_PCM, conf );
	if( ! conf->output )
	{
		spook_log( SL_ERR, "oss: unable to create stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	conf->output->get_framerate = get_framerate;
	conf->output->set_running = set_running;
	return 0;
}

static int set_samplerate( int num_tokens, struct token *tokens, void *d )
{
	struct oss_input *conf = (struct oss_input *)d;

	if( conf->rate > 0 )
	{
		spook_log( SL_ERR, "oss: sample rate has already been set!" );
		return -1;
	}
	conf->rate = tokens[1].v.num;

	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "output", set_output, 1, 1, { TOKEN_STR } },
	{ "device", set_device, 1, 1, { TOKEN_STR } },
	{ "samplerate", set_samplerate, 1, 1, { TOKEN_NUM } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

void oss_init(void)
{
	register_config_context( "input", "oss", start_block, end_block,
					config_statements );
}
