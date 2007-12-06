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
#include <pthread.h>

#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <inputs.h>
#include <conf_parse.h>

#define WIDTH	320
#define HEIGHT	240

struct dc1394_cam {
	raw1394handle_t handle;
	int running;
	struct soft_queue *outq;
	struct stream *output;
};

struct dc1394_input {
	struct dc1394_cam cam[16];
	dc1394_cameracapture camera[16];
	int cam_count;
	pthread_t thread;
};

static void *capture_loop( void *d )
{
	struct dc1394_input *conf = (struct dc1394_input *)d;
	struct frame *f;
	int cam;

	for(;;)
	{
		dc1394_dma_multi_capture( conf->camera, conf->cam_count );
		for( cam = 0; cam < conf->cam_count; ++cam )
		{
			if( conf->cam[cam].running && ( f = new_frame() ) )
			{
				f->length = HEIGHT * WIDTH * 2;
				f->format = FORMAT_RAW_UYVY;
				f->width = WIDTH;
				f->height = HEIGHT;
				f->key = 1;

				memcpy( f->d, conf->camera[cam].capture_buffer,
						f->length );

				if( soft_queue_add( conf->cam[cam].outq, f ) < 0 )
					unref_frame( f );
			}
			dc1394_dma_done_with_buffer( &conf->camera[cam] );
		}
	}

	return NULL;
}

static void get_back_frame( struct event_info *ei, void *d )
{
	struct dc1394_cam *cam = (struct dc1394_cam *)d;
	struct frame *f = (struct frame *)ei->data;

	deliver_frame_to_stream( f, cam->output );
}

static int cam_setup( struct dc1394_input *conf, int cam, int port,
				nodeid_t node, int dma_chan )
{
	unsigned int channel;
	unsigned int speed;

	conf->cam[cam].running = 0;
	conf->camera[cam].node = node;
	conf->cam[cam].handle = dc1394_create_handle( port );
	if( ! conf->cam[cam].handle )
	{
		spook_log( SL_ERR, "dc1394: unable to create a camera handle" );
		return -1;
	}
	if( dc1394_get_iso_channel_and_speed( conf->cam[cam].handle, node,
				&channel, &speed ) != DC1394_SUCCESS )
	{
		spook_log( SL_ERR, "dc1394: unable to create an ISO channel" );
		return -1;
	}

	if( dc1394_dma_setup_capture( conf->cam[cam].handle, node, dma_chan,
				FORMAT_VGA_NONCOMPRESSED, MODE_320x240_YUV422,
				SPEED_400, FRAMERATE_30, 8 /* num buffers */,
#ifdef DC1394_EXTRA_BUFFERING_FLAG
				0 /* do_extra_buffering */,
#endif
				1 /* drop frames */, NULL /* device name */,
				&conf->camera[cam] ) != DC1394_SUCCESS )
	{
		spook_log( SL_ERR, "dc1394: unable to set up DMA for camera" );
		return -1;
	}
	if( dc1394_start_iso_transmission( conf->cam[cam].handle,
				conf->camera[cam].node ) != DC1394_SUCCESS )
	{
		spook_log( SL_ERR,
			"dc1394: unable to start ISO transfer from camera" );
		return -1;
	}
	conf->cam[cam].outq = new_soft_queue( 16 );
	add_softqueue_event( conf->cam[cam].outq, 0,
			get_back_frame, &conf->cam[cam] );
	return 0;
}

static int dc1394_setup( struct dc1394_input *conf )
{
	raw1394handle_t raw_handle;
	nodeid_t *camera_nodes = NULL;
	int numPorts;
	int actual_count = 0, c, i;
	struct raw1394_portinfo ports[16];

	raw_handle = raw1394_new_handle();
	if( ! raw_handle )
	{
		spook_log( SL_ERR,
			"dc1394: unable to acquire a raw1394 handle" );
		return -1;
	}

	numPorts = raw1394_get_port_info( raw_handle, ports, 16 );
	raw1394_destroy_handle( raw_handle );
	for( i = 0; i < numPorts; ++i )
	{
		int camCount = 0;

		raw_handle = raw1394_new_handle();
		raw1394_set_port( raw_handle, i );
		camera_nodes = dc1394_get_camera_nodes( raw_handle,
							&camCount, 1 );
		raw1394_destroy_handle( raw_handle );
		spook_log( SL_INFO, "dc1394: found %d cams on port %d",
				camCount, i );
		for( c = 0; c < camCount; ++c )
		{
			if( cam_setup( conf, actual_count, i,
						camera_nodes[c], c ) < 0 )
				return -1;
			if( ++actual_count == conf->cam_count ) return 0;
		}
		// this doesn't work, fix later (just a li'l memory leak...)
		//dc1394_free_camera_nodes( camera_nodes );
	}

	if( actual_count == 0 )
	{
		spook_log( SL_ERR, "dc1394: did not find any IIDC cameras" );
		return -1;
	}

	if( actual_count < conf->cam_count )
		spook_log( SL_WARN,
			"dc1394: only found %d cameras, some outputs will be inactive",
			actual_count );

	return 0;
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct dc1394_cam *cam = (struct dc1394_cam *)s->private;

	*fincr = 1;
	*fbase = 30;
}

static void set_running( struct stream *s, int running )
{
	struct dc1394_cam *cam = (struct dc1394_cam *)s->private;

	cam->running = running;
}

#if 0
void dc1394_close(void)
{
	fprintf( stderr, "closing down..." );
	dc1394_dma_unlisten( handle, &camera );
	dc1394_dma_release_camera( handle, &camera );
	dc1394_destroy_handle( handle );
	fprintf( stderr, "done!\n" );
}
#endif

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct dc1394_input *conf;

	conf = (struct dc1394_input *)malloc( sizeof( struct dc1394_input ) );
	conf->cam_count = 0;

	return conf;
}

static int end_block( void *d )
{
	struct dc1394_input *conf = (struct dc1394_input *)d;

	if( conf->cam_count == 0 )
	{
		spook_log( SL_ERR, "dc1394: missing output stream name" );
		return -1;
	}
	if( dc1394_setup( conf ) < 0 )
	{
		spook_log( SL_ERR, "dc1394: unable to initialize video input" );
		return -1;
	}
	pthread_create( &conf->thread, NULL, capture_loop, conf );

	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct dc1394_input *conf = (struct dc1394_input *)d;

	conf->cam[conf->cam_count].output = new_stream( tokens[1].v.str,
				FORMAT_RAW_UYVY, &conf->cam[conf->cam_count] );
	if( ! conf->cam[conf->cam_count].output )
	{
		spook_log( SL_ERR, "dc1394: unable to create stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	conf->cam[conf->cam_count].output->get_framerate = get_framerate;
	conf->cam[conf->cam_count].output->set_running = set_running;
	++conf->cam_count;
	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "output", set_output, 1, 1, { TOKEN_STR } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

void dc1394_init(void)
{
	register_config_context( "input", "dc1394", start_block, end_block,
					config_statements );
}
