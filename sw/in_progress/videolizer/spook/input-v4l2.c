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
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#ifdef HAVE_ASM_TYPES_H
#include <asm/types.h>
#endif
#ifdef HAVE_LINUX_COMPILER_H
#include <linux/compiler.h>
#endif
#include <linux/videodev2.h>
#ifdef HAVE_GO7007_H
#include <linux/go7007.h>
#endif

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <inputs.h>
#include <conf_parse.h>

#define	INPUTTYPE_WEBCAM	1
#define INPUTTYPE_NTSC		2
#define INPUTTYPE_PAL		3
#define INPUTTYPE_PAL_BG	4

struct v4l2_spook_input {
	struct stream *output;
	struct frame_exchanger *ex;
	char device[256];
	int format;
	int bitrate;
	int width;
	int height;
	int fincr;
	int fbase;
	int inputport;
	int inputtype;
	int fps;
	int fd;
	void *bufaddr[2];
	pthread_t thread;
	int running;
};

static void *capture_loop( void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;
	struct v4l2_buffer buf;
	struct frame *f;
	struct timeval now, start;
	int frames = 0, i;
	fd_set rfds;

	start.tv_sec = 0;

	for( i = 0; i < 2; ++i )
	{
		memset( &buf, 0, sizeof( buf ) );
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if( ioctl( conf->fd, VIDIOC_QBUF, &buf ) < 0 )
		{
			spook_log( SL_ERR,
				"v4l2: aborting on error in VIDIOC_QBUF: %s",
				strerror( errno ) );
			exit( 1 );
		}
	}

	i = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if( ioctl( conf->fd, VIDIOC_STREAMON, &i ) < 0 )
	{
		spook_log( SL_ERR, "v4l2: aborting on error in VIDIOC_STREAMON: %s",
				strerror( errno ) );
		exit( 1 );
	}

	for(;;)
	{
		FD_ZERO( &rfds );
		FD_SET( conf->fd, &rfds );
		if( select( conf->fd + 1, &rfds, NULL, NULL, NULL ) < 0 )
		{
			spook_log( SL_ERR, "v4l2: select returned an error: %s",
					strerror( errno ) );
			exit( 1 );
		}

		memset( &buf, 0, sizeof( buf ) );
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;

		if( ioctl( conf->fd, VIDIOC_DQBUF, &buf ) < 0 )
		{
			if( errno == EIO )
			{
				spook_log( SL_VERBOSE, "v4l2: video capture device reports bad signal, dropping frame" );
			} else
			{
				spook_log( SL_ERR,
					"v4l2: aborting on error in VIDIOC_DQBUF: %s",
					strerror( errno ) );
				exit( 1 );
			}
		}

		if( conf->fincr == 0 )
		{
			if( start.tv_sec > 0 )
			{
				gettimeofday( &now, NULL );
				++frames;
				if( now.tv_sec >= start.tv_sec + 8 &&
						now.tv_usec >= start.tv_usec )
				{
					i = ( now.tv_sec - start.tv_sec ) * 1000000 +
						now.tv_usec - start.tv_usec;
					i = ( (double)1000000 * (double)frames
						/ (double)i + 0.005 ) * (double)100;
					if( i % 100 == 0 )
					{
						conf->fincr = 1;
						conf->fbase = i / 100;
					} else if( i % 10 == 0 )
					{
						conf->fincr = 10;
						conf->fbase = i / 10;
					} else
					{
						conf->fincr = 100;
						conf->fbase = i;
					}
					spook_log( SL_INFO, "guessed frame rate at %.2f",
						(double)conf->fbase/(double)conf->fincr );
					spook_log( SL_DEBUG, "fincr = %d, fbase = %d",
							conf->fbase, conf->fincr );
					frames = 0;
				}
			} else gettimeofday( &start, NULL );
		} else if( ! conf->running )
		{
			frames = 0;
			f = NULL;
		} else if( ( f = get_next_frame( conf->ex, 0 ) ) )
		{
			f->length = buf.bytesused;
			f->format = conf->format;
			f->width = conf->width;
			f->height = conf->height;
			f->key = 1;
			frames = 0;

			memcpy( f->d, conf->bufaddr[buf.index], f->length );

			deliver_frame( conf->ex, f );
		} else
		{
			spook_log( SL_WARN, "v4l2: dropping frame" );
			++frames;
		}

		if( ioctl( conf->fd, VIDIOC_QBUF, &buf ) < 0 )
		{
			spook_log( SL_ERR,
				"v4l2: aborting on error in VIDIOC_QBUF: %s",
				strerror( errno ) );
			exit( 1 );
		}
	}
	return NULL;
}

static void get_back_frame( struct frame *f, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	exchange_frame( conf->ex, new_frame() );
	deliver_frame_to_stream( f, conf->output );
}

static int v4l2_setup( struct v4l2_spook_input *conf )
{
	struct v4l2_capability vc;
	struct v4l2_input vi;
	v4l2_std_id std;
	struct v4l2_cropcap cc;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	struct v4l2_requestbuffers req;
	struct v4l2_buffer buf;
	int i;

	if( ( conf->fd = open( conf->device, O_RDWR | O_NONBLOCK ) ) < 0 )
	{
		spook_log( SL_ERR, "v4l2: unable to open %s: %s",
				conf->device, strerror( errno ) );
		return -1;
	}
	if( ioctl( conf->fd, VIDIOC_QUERYCAP, &vc ) < 0 )
	{
		spook_log( SL_ERR, "v4l2: error determining device capabilities" );
		return -1;
	}
	if( ! ( vc.capabilities & V4L2_CAP_VIDEO_CAPTURE ) )
	{
		spook_log( SL_ERR, "v4l2: device has no capture capability" );
		return -1;
	}
	if( ! ( vc.capabilities & V4L2_CAP_STREAMING ) )
	{
		spook_log( SL_ERR, "v4l2: device does not support streaming capture" );
		return -1;
	}

	spook_log( SL_INFO, "v4l2: using capture device \"%s\"", vc.card );

	memset( &vi, 0, sizeof( vi ) );
	vi.index = conf->inputport;
	if( conf->inputport < 0 ||
			ioctl( conf->fd, VIDIOC_ENUMINPUT, &vi ) < 0 )
	{
		spook_log( SL_ERR,
			"v4l2: input port is invalid!  Valid input ports:" );

		for( vi.index = 0;
			ioctl( conf->fd, VIDIOC_ENUMINPUT, &vi ) >= 0;
			++vi.index )
		{
			spook_log( SL_INFO, "v4l2: input port %d: %s",
					vi.index, vi.name );
		}
		return -1;
	}

	if( ioctl( conf->fd, VIDIOC_S_INPUT, &conf->inputport ) < 0 )
	{
		spook_log( SL_ERR,
			"v4l2: input port seems valid but unable to select: %s",
			strerror( errno ) );
		return -1;
	}

	switch( conf->inputtype )
	{
	case INPUTTYPE_NTSC:
		std = V4L2_STD_NTSC;
		break;
	case INPUTTYPE_PAL:
		std = V4L2_STD_PAL;
		break;
	case INPUTTYPE_PAL_BG:
		std = V4L2_STD_PAL_BG;
		break;
	}

	if( ioctl( conf->fd, VIDIOC_S_STD, &std ) < 0 )
	{
		spook_log( SL_ERR,
		      "v4l2: input port does not support this video standard" );
		return -1;
	}

	cc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if( ioctl( conf->fd, VIDIOC_CROPCAP, &cc ) >= 0 )
	{
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cc.defrect;
		ioctl( conf->fd, VIDIOC_S_CROP, &crop );
	}

	memset( &fmt, 0, sizeof( fmt ) );
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = conf->width;
	fmt.fmt.pix.height = conf->height;
	switch( conf->format )
	{
	case FORMAT_RAW_YUY2:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		break;
	case FORMAT_RAW_UYVY:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
		break;
	case FORMAT_JPEG:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
		break;
	case FORMAT_MPEG4:
#ifdef HAVE_GO7007_H
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MPEG4;
		break;
#else
		spook_log( SL_ERR,
		      "v4l2: don't know how to set MPEG4 on this device" );
		return -1;
#endif
	}
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

	if( ioctl( conf->fd, VIDIOC_S_FMT, &fmt ) < 0 )
	{
		spook_log( SL_ERR, "v4l2: error setting video resolution or "
				"compression/palette format" );
		return -1;
	}
	conf->width = fmt.fmt.pix.width;
	conf->height = fmt.fmt.pix.height;

	if( conf->bitrate > 0 )
	{
#ifdef HAVE_GO7007_H
		i = conf->bitrate * 1000;
		if( ioctl( conf->fd, GO7007IOC_S_BITRATE, &i ) < 0 )
		{
			spook_log( SL_ERR, "v4l2: error setting bitrate; does "
				"this hardware support bitrate setting?" );
			return -1;
		}
#else
		spook_log( SL_ERR,
			"v4l2: don't know how to set bitrate on this device" );
		return -1;
#endif
	}

	memset( &req, 0, sizeof( req ) );
	req.count = 2;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if( ioctl( conf->fd, VIDIOC_REQBUFS, &req ) < 0 )
	{
		spook_log( SL_ERR, "v4l2: device does not support memory mapping" );
		return -1;
	}

	for( i = 0; i < req.count; ++i )
	{
		memset( &buf, 0, sizeof( buf ) );
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if( ioctl( conf->fd, VIDIOC_QUERYBUF, &buf ) < 0 )
		{
			spook_log( SL_ERR, "v4l2: device does not support memory mapping" );
			return -1;
		}
		if( ! ( conf->bufaddr[i] = (unsigned char *)mmap( NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, conf->fd, buf.m.offset ) ) )
		{
			spook_log( SL_ERR, "v4l2: error mapping driver memory" );
			return -1;
		}
	}

	return 0;
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)s->private;

	*fincr = conf->fincr;
	*fbase = conf->fbase;
}

static void set_running( struct stream *s, int running )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)s->private;

	conf->running = running;
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct v4l2_spook_input *conf;

	conf = (struct v4l2_spook_input *)malloc( sizeof( struct v4l2_spook_input ) );
	conf->output = NULL;
	conf->device[0] = 0;
	conf->format = FORMAT_RAW_UYVY;
	conf->bitrate = 0;
	conf->width = 0;
	conf->height = 0;
	conf->inputport = -1;
	conf->inputtype = 0;
	conf->fps = -1;
	conf->fd = -1;
	conf->running = 0;

	return conf;
}

static int end_block( void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;
	int i;

	if( ! conf->output )
	{
		spook_log( SL_ERR, "v4l2: missing output stream name" );
		return -1;
	}
	if( ! conf->device[0] )
	{
		spook_log( SL_ERR, "v4l2: missing V4L2 device name" );
		return -1;
	}
	switch( conf->inputtype )
	{
	case 0:
		spook_log( SL_ERR, "v4l2: input type not specified" );
		return -1;
	case INPUTTYPE_NTSC:
		conf->fincr = 1001;
		conf->fbase = 30000;
		if( conf->width == 0 )
		{
			conf->width = 320;
			conf->height = 240;
		}
		break;
	case INPUTTYPE_PAL:
	case INPUTTYPE_PAL_BG:
		conf->fincr = 1;
		conf->fbase = 25;
		if( conf->width == 0 )
		{
			conf->width = 320;
			conf->height = 288;
		}
		break;
	case INPUTTYPE_WEBCAM:
		if( conf->fps < 0 )
		{
			spook_log( SL_ERR,
				"v4l2: framerate not specified for webcam" );
			return -1;
		} else if( conf->fps > 0 )
		{
			conf->fincr = 1;
			conf->fbase = conf->fps;
		} else spook_log( SL_INFO, "v4l2: must figure out framerate, this will take some time..." );
		if( conf->inputport < 0 ) conf->inputport = 0;
		if( conf->width == 0 )
		{
			conf->width = 352;
			conf->height = 288;
		}
		break;
	}
	if( v4l2_setup( conf ) < 0 ) return -1;
	conf->ex = new_exchanger( 8, get_back_frame, conf );
	for( i = 0; i < 8; ++i ) exchange_frame( conf->ex, new_frame() );
	pthread_create( &conf->thread, NULL, capture_loop, conf );

	return 0;
}

static int set_device( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	strcpy( conf->device, tokens[1].v.str );
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	conf->output = new_stream( tokens[1].v.str, conf->format, conf );
	if( ! conf->output )
	{
		spook_log( SL_ERR, "v4l2: unable to create stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	conf->output->get_framerate = get_framerate;
	conf->output->set_running = set_running;
	return 0;
}

static int set_format( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	if( conf->output )
	{
		spook_log( SL_ERR, "v4l2: output format must be specified "
				"before output stream name" );
		return -1;
	}
	if( ! strcasecmp( tokens[1].v.str, "raw" ) )
		conf->format = FORMAT_RAW_UYVY;
	else if( ! strcasecmp( tokens[1].v.str, "raw2" ) )
		conf->format = FORMAT_RAW_YUY2;
	else if( ! strcasecmp( tokens[1].v.str, "mpeg4" ) )
		conf->format = FORMAT_MPEG4;
	else if( ! strcasecmp( tokens[1].v.str, "mjpeg" ) )
		conf->format = FORMAT_JPEG;
	else
	{
		spook_log( SL_ERR, "v4l2: format \"%s\" is unsupported; try "
				"RAW, MJPEG, or MPEG4", tokens[1].v.str );
		return -1;
	}

	return 0;
}

static int set_bitrate( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	if( tokens[1].v.num < 10 || tokens[1].v.num > 10000 )
	{
		spook_log( SL_ERR,
			"v4l2: bitrate must be between 10 and 10000" );
		return -1;
	}
	conf->bitrate = tokens[1].v.num;
	return 0;
}

static int set_framesize( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	conf->width = tokens[1].v.num;
	conf->height = tokens[2].v.num;

	return 0;
}

static int set_inputport( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	conf->inputport = tokens[1].v.num;

	return 0;
}

static int set_inputtype( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	if( ! strcasecmp( tokens[1].v.str, "webcam" ) )
		conf->inputtype = INPUTTYPE_WEBCAM;
	else if( ! strcasecmp( tokens[1].v.str, "ntsc" ) )
		conf->inputtype = INPUTTYPE_NTSC;
	else if( ! strcasecmp( tokens[1].v.str, "pal" ) )
		conf->inputtype = INPUTTYPE_PAL;
	else if( ! strcasecmp( tokens[1].v.str, "pal_bg" ) )
		conf->inputtype = INPUTTYPE_PAL_BG;
	else
	{
		spook_log( SL_ERR,
			"v4l2: video mode \"%s\" is unsupported; try NTSC, PAL or WEBCAM",
			tokens[1].v.str );
		return -1;
	}

	return 0;
}

static int set_framerate_num( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	if( conf->fps >= 0 )
	{
		spook_log( SL_ERR, "v4l2: frame rate has already been set!" );
		return -1;
	}
	conf->fps = tokens[1].v.num;

	return 0;
}

static int set_framerate_str( int num_tokens, struct token *tokens, void *d )
{
	struct v4l2_spook_input *conf = (struct v4l2_spook_input *)d;

	if( conf->fps >= 0 )
	{
		spook_log( SL_ERR, "v4l2: frame rate has already been set!" );
		return -1;
	}
	if( strcasecmp( tokens[1].v.str, "auto" ) )
	{
		spook_log( SL_ERR,
			"v4l2: frame rate should be a number or \"auto\"" );
		return -1;
	}

	conf->fps = 0;

	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "device", set_device, 1, 1, { TOKEN_STR } },
	{ "output", set_output, 1, 1, { TOKEN_STR } },
	{ "format", set_format, 1, 1, { TOKEN_STR } },
	{ "bitrate", set_bitrate, 1, 1, { TOKEN_NUM } },
	{ "framesize", set_framesize, 2, 2, { TOKEN_NUM, TOKEN_NUM } },
	{ "inputport", set_inputport, 1, 1, { TOKEN_NUM } },
	{ "inputtype", set_inputtype, 1, 1, { TOKEN_STR } },
	{ "framerate", set_framerate_num, 1, 1, { TOKEN_NUM } },
	{ "framerate", set_framerate_str, 1, 1, { TOKEN_STR } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

void v4l2_init(void)
{
	register_config_context( "input", "v4l2", start_block, end_block,
					config_statements );
}
