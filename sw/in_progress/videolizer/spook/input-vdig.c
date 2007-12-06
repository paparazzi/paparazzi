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

#include <QuickTime/Movies.h>
#include <QuickTime/QuickTimeComponents.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <inputs.h>
#include <conf_parse.h>

#define WIDTH	320
#define HEIGHT	240

struct vdig_input {
	struct stream *output;
	struct frame_exchanger *ex;
	VideoDigitizerComponent ci;
	pthread_t thread;
	int running;
};

static void *capture_loop( void *d )
{
	struct vdig_input *conf = (struct vdig_input *)d;
	struct frame *f;
	int frames = 0;
	UInt8 queuedFrameCount;
	Ptr theData;
	long dataSize;
	UInt8 similarity;
	TimeRecord t;
	OSErr err;

	for(;;)
	{
		if( conf->running )
		{
			++frames;
			f = get_next_frame( conf->ex, 0 );
		} else
		{
			frames = 0;
			f = NULL;
		}
		if( ! f )
		{
			if( conf->running )
				spook_log( SL_WARN, "vdig: dropping frame" );
			err = VDCompressOneFrameAsync( conf->ci );
			if( err != noErr )
			{
				spook_log( SL_ERR, "vdig: error compressing frame: %d", err );
				exit( 1 );
			}
			for(;;)
			{
				err = VDCompressDone( conf->ci, &queuedFrameCount, &theData, &dataSize,
							&similarity, &t );
				if( err != noErr )
				{
					spook_log( SL_ERR, "vdig: error waiting for frame: %d", err );
					exit( 1 );
				}
				if( queuedFrameCount > 0 ) break;
	//			printf( "waiting...\n" );
	//			ReceiveNextEvent( 0, NULL, kEventDurationForever, true, &theEvent );
				usleep(10000);
	//			printf( "got event!\n" );
			}
			VDReleaseCompressBuffer( conf->ci, theData );
			continue;
		}

		f->format = FORMAT_RAW_UYVY;
		f->width = WIDTH;
		f->height = HEIGHT;
		f->key = 1;
		frames = 0;

		err = VDCompressOneFrameAsync( conf->ci );
		if( err != noErr )
		{
			spook_log( SL_ERR, "vdig: error compressing frame: %d", err );
			exit( 1 );
		}
		for(;;)
		{
			err = VDCompressDone( conf->ci, &queuedFrameCount, &theData, &dataSize,
						&similarity, &t );
			if( err != noErr )
			{
				spook_log( SL_ERR, "vdig: error waiting for frame: %d", err );
				exit( 1 );
			}
			if( queuedFrameCount > 0 ) break;
//			printf( "waiting...\n" );
//			ReceiveNextEvent( 0, NULL, kEventDurationForever, true, &theEvent );
			usleep(10000);
//			printf( "got event!\n" );
		}
//		printf( "pointer: %08x  data size: %ld  queued frame count: %d\n",
//			(unsigned int)theData, dataSize, queuedFrameCount );

		f->length = dataSize;
		memcpy( f->d, theData, f->length );

		VDReleaseCompressBuffer( conf->ci, theData );

		deliver_frame( conf->ex, f );
	}
	return NULL;
}

static void get_back_frame( struct frame *f, void *d )
{
	struct vdig_input *conf = (struct vdig_input *)d;

	exchange_frame( conf->ex, new_frame() );
	deliver_frame_to_stream( f, conf->output );
}

static int vdig_setup( struct vdig_input *conf )
{
	ImageDescriptionHandle desc;
	DigitizerInfo info;
	Str255 outName;
	UInt32 outNameFlags;
	long inputCurrentFlag, outputCurrentFlag;
	OSErr err;
	short inputs;
	Rect bounds;

//	theTarget = GetEventDispatcherTarget();

	conf->ci = OpenDefaultComponent( FOUR_CHAR_CODE('vdig'), 0 );
	if( ! conf->ci )
	{
		spook_log( SL_ERR, "vdig: unable to find any video digitizers" );
		return -1;
	}
	err = VDGetDeviceNameAndFlags( conf->ci, outName, &outNameFlags );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: error retrieving digitizer name: %d", err );
		return -1;
	}
	spook_log( SL_INFO, "vdig: using digitizer \"%s\"", outName );
	err = VDGetCurrentFlags( conf->ci, &inputCurrentFlag, &outputCurrentFlag );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: error querying digitizer capabilities: %d", err );
		return -1;
	}
	if( ! ( outputCurrentFlag & digiOutDoesCompress ) )
	{
		spook_log( SL_ERR, "vdig: digitizer does not support required video format" );
		return -1;
	}
	err = VDSetTimeBase( conf->ci, NewTimeBase() );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: error setting time base: %s", err );
		return -1;
	}
	err = VDGetNumberOfInputs( conf->ci, &inputs );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: error querying input ports: %s", err );
		return -1;
	}
	if( inputs > 0 )
	{
		spook_log( SL_INFO, "vdig: digitizer has %d inputs", inputs + 1 );
	}
	err = VDSetInput( conf->ci, 0 );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: unable to set input port: %d", err );
		return -1;
	}
	err = VDSetInputStandard( conf->ci, ntscIn );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: unable to set video mode: %d", err );
		return -1;
	}
	memset( &info, 0, sizeof( info ) );
	err = VDGetDigitizerInfo( conf->ci, &info );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: error querying digitizer: %d", err );
		return -1;
	}
//	printf( "max width: %d  max height: %d\n", info.maxDestWidth, info.maxDestHeight );
	desc = (ImageDescriptionHandle)NewHandle( sizeof( ImageDescription ) );
	err = VDGetImageDescription( conf->ci, desc );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: error querying image description: %d", err );
		return -1;
	}
	err = VDSetCompressionOnOff( conf->ci, TRUE );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: unable to select non-RGB formats: %d", err ); 
		return -1;
	}
	bounds.left = bounds.top = 0;
	bounds.right = 320;
	bounds.bottom = 240;
	err = VDSetCompression( conf->ci, k422YpCbCr8CodecType, 16, &bounds,
		codecNormalQuality, codecNormalQuality, 0 );
	if( err != noErr )
	{
		spook_log( SL_ERR, "vdig: unable to select UYVY compression: %d", err );
		return -1;
	}
	return 0;
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct vdig_input *conf = (struct vdig_input *)s->private;

	*fincr = 1;
	*fbase = 30;
}

static void set_running( struct stream *s, int running )
{
	struct vdig_input *conf = (struct vdig_input *)s->private;

	conf->running = running;
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct vdig_input *conf;

	conf = (struct vdig_input *)malloc( sizeof( struct vdig_input ) );
	conf->output = NULL;
	conf->running = 0;

	return conf;
}

static int end_block( void *d )
{
	struct vdig_input *conf = (struct vdig_input *)d;
	int i;

	if( ! conf->output )
	{
		spook_log( SL_ERR, "vdig: missing output stream name" );
		return -1;
	}
	if( vdig_setup( conf ) < 0 ) return -1;
	conf->ex = new_exchanger( 16, get_back_frame, conf );
	for( i = 0; i < 16; ++i ) exchange_frame( conf->ex, new_frame() );
	pthread_create( &conf->thread, NULL, capture_loop, conf );

	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct vdig_input *conf = (struct vdig_input *)d;

	conf->output = new_stream( tokens[1].v.str, FORMAT_RAW_UYVY, conf );
	if( ! conf->output )
	{
		spook_log( SL_ERR, "vdig: unable to create stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	conf->output->get_framerate = get_framerate;
	conf->output->set_running = set_running;
	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "output", set_output, 1, 1, { TOKEN_STR } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

void vdig_init(void)
{
	register_config_context( "input", "vdig", start_block, end_block,
					config_statements );
}
