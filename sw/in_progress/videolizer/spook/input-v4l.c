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
#include <linux/videodev.h>
#ifdef HAVE_PWC_IOCTL_H
#include <pwc-ioctl.h>
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

struct v4l_input {
	struct stream *output;
	struct frame_exchanger *ex;
	char device[256];
	int width;
	int height;
	int fincr;
	int fbase;
	int inputport;
	int inputtype;
#ifdef HAVE_PWC_IOCTL_H
	struct pwc_whitebalance pwc_whitebalance;
#endif
	int fps;
	int fd;
	struct video_mbuf vm;
	unsigned char *mmap_buf;
	int cur_frame;
	pthread_t thread;
	int running;
};

static void copy_yuv420p_to_uyvy( unsigned char *dest, unsigned char *src,
		int width, int height )
{
	unsigned char *p, *y, *u, *v;
	int r, c;

	p = dest;
	y = src;
	u = y + width * height;
	v = u + ( width / 2 ) * ( height / 2 );

	for( r = 0; r < height; ++r )
		for( c = 0; c < width; c += 2 )
		{
			*(p++) = u[( r / 2 ) * ( width / 2 ) + ( c / 2 )];
			*(p++) = y[r * width + c];
			*(p++) = v[( r / 2 ) * ( width / 2 ) + ( c / 2 )];
			*(p++) = y[r * width + c + 1];
		}
}

static void *capture_loop( void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;
	struct video_mmap mm;
	struct frame *f;
	struct timeval now, start;
	int frames = 0, cur_buf, i;

	start.tv_sec = 0;

	mm.frame = 0;
	mm.width = conf->width;
	mm.height = conf->height;
	mm.format = VIDEO_PALETTE_YUV420P;

	if( ioctl( conf->fd, VIDIOCMCAPTURE, &mm ) < 0 )
	{
		spook_log( SL_ERR, "v4l: aborting on error in VIDIOCMCAPTURE: %s",
				strerror( errno ) );
		exit( 1 );
	}

	cur_buf = 1;

	for(;;)
	{
		mm.frame = cur_buf;
		mm.width = conf->width;
		mm.height = conf->height;
		mm.format = VIDEO_PALETTE_YUV420P;

		if( ioctl( conf->fd, VIDIOCMCAPTURE, &mm ) < 0 )
		{
			spook_log( SL_ERR,
				"v4l: aborting on error in VIDIOCMCAPTURE: %s",
				strerror( errno ) );
			exit( 1 );
		}

		cur_buf = cur_buf ^ 1;

		if( ioctl( conf->fd, VIDIOCSYNC, &cur_buf ) < 0 )
		{
			spook_log( SL_ERR,
				"v4l: aborting on error in VIDIOCSYNC: %s",
				strerror( errno ) );
			exit( 1 );
		}

		if( conf->fincr == 0 )
		{
			if( start.tv_sec == 0 )
			{
				gettimeofday( &start, NULL );
				continue;
			}
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
			continue;
		}
		if( ! conf->running )
		{
			frames = 0;
			f = NULL;
			continue;
		}

		++frames;

		if( ! ( f = get_next_frame( conf->ex, 0 ) ) )
		{
			spook_log( SL_WARN, "v4l: dropping frame" );
			continue;
		}

		f->length = conf->width * conf->height * 2;
		f->format = FORMAT_RAW_UYVY;
		f->width = conf->width;
		f->height = conf->height;
		f->key = 1;
		frames = 0;

		copy_yuv420p_to_uyvy( f->d,
				conf->mmap_buf + conf->vm.offsets[cur_buf],
				f->width, f->height );

		deliver_frame( conf->ex, f );
	}
	return NULL;
}

static void get_back_frame( struct frame *f, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	exchange_frame( conf->ex, new_frame() );
	deliver_frame_to_stream( f, conf->output );
}

static int v4l_setup( struct v4l_input *conf )
{
	struct video_capability vc;
	struct video_channel chan;
	struct video_picture pict;
	struct video_window win;
	int i, pwc, real_width, real_height;
#ifdef HAVE_PWC_IOCTL_H
	struct pwc_probe pwc_probe;
#ifdef VIDIOCPWCGREALSIZE
	struct pwc_imagesize pwc_imagesize;
#endif
#endif

	conf->cur_frame = -1;

	if( ( conf->fd = open( conf->device, O_RDWR ) ) < 0 )
	{
		spook_log( SL_ERR, "v4l: unable to open %s: %s",
				conf->device, strerror( errno ) );
		return -1;
	}
	if( ioctl( conf->fd, VIDIOCGCAP, &vc ) < 0 )
	{
		spook_log( SL_ERR, "v4l: error determining device capabilities" );
		return -1;
	}

	spook_log( SL_INFO, "v4l: using capture device \"%s\"", vc.name );

	if( conf->width > vc.maxwidth )
		spook_log( SL_ERR,
			"v4l: device supports a maximum frame width of %d; %d is too large",
			vc.maxwidth, conf->width );
	if( conf->height > vc.maxheight )
		spook_log( SL_ERR,
			"v4l: device supports a maximum frame height of %d; %d is too large",
			vc.maxheight, conf->height );
	if( conf->width > vc.maxwidth || conf->height > vc.maxheight )
		return -1;

#ifdef HAVE_PWC_IOCTL_H
	/* This is apparently how we're supposed to probe for PWC devices... */
	if( sscanf( vc.name, "Philips %d webcam", &i ) == 1 ||
		( ioctl( conf->fd, VIDIOCPWCPROBE, &pwc_probe ) == 0 &&
			! strcmp( vc.name, pwc_probe.name ) ) )
	{
		spook_log( SL_INFO, "v4l: activating support for Philips webcams" );
		pwc = 1;
	} else
#endif
		pwc = 0;

	if( conf->inputport < 0 || conf->inputport >= vc.channels )
	{
		spook_log( SL_ERR,
			"v4l: input port is invalid!  Valid input ports:" );

		for( i = 0; i < vc.channels; ++i )
		{
			chan.channel = i;
			if( ioctl( conf->fd, VIDIOCGCHAN, &chan ) < 0 )
			{
				spook_log( SL_ERR,
					"v4l: error getting info on input port" );
				return -1;
			}
			spook_log( SL_INFO, "v4l: input port %d: %s",
					chan.channel, chan.name );
		}
		return -1;
	}

	chan.channel = conf->inputport;
	switch( conf->inputtype )
	{
	case INPUTTYPE_NTSC:
		chan.norm = VIDEO_MODE_NTSC;
		break;
	case INPUTTYPE_PAL:
		chan.norm = VIDEO_MODE_PAL;
		break;
	default:
		chan.norm = VIDEO_MODE_AUTO;
		break;
	}
	if( ioctl( conf->fd, VIDIOCSCHAN, &chan ) < 0 )
	{
		spook_log( SL_ERR, "v4l: error selecting input port" );
		return -1;
	}

	if( ioctl( conf->fd, VIDIOCGPICT, &pict ) < 0 )
	{
		spook_log( SL_ERR, "v4l: error querying palette parameters" );
		return -1;
	}

	pict.palette = VIDEO_PALETTE_YUV420P;
	pict.depth = 24;

	if( ioctl( conf->fd, VIDIOCSPICT, &pict ) < 0 )
	{
		spook_log( SL_ERR, "v4l: error setting palette parameters" );
		return -1;
	}

	win.x = win.y = 0;
	win.width = conf->width;
	win.height = conf->height;
	win.chromakey = 0;
	win.flags = 0;
	win.clips = NULL;
	win.clipcount = 0;
	if( conf->inputtype == INPUTTYPE_WEBCAM && conf->fps > 0 )
	{
#ifdef HAVE_PWC_IOCTL_H
		if( pwc ) win.flags |= conf->fps << PWC_FPS_SHIFT;
		else
#endif
		{
			spook_log( SL_ERR, "v4l: don't know how to set frame rate; try using \"FrameRate auto\"" );
			return -1;
		}
	}
	
	if( ioctl( conf->fd, VIDIOCSWIN, &win ) < 0 )
	{
		if( conf->inputtype == INPUTTYPE_WEBCAM && conf->fps > 0 )
			spook_log( SL_ERR,
				"v4l: unable to set requested frame size/frame rate" );
		else
			spook_log( SL_ERR,
				"v4l: unable to set requested frame size" );
		return -1;
	}

	memset( &win, 0, sizeof( win ) );

	if( ioctl( conf->fd, VIDIOCGWIN, &win ) < 0 )
	{
		spook_log( SL_ERR, "v4l: error verifying image parameters" );
		return -1;
	}

#ifdef HAVE_PWC_IOCTL_H
	if( pwc )
	{
		int fps;

		fps = ( win.flags & PWC_FPS_FRMASK ) >> PWC_FPS_SHIFT;
		if( fps != conf->fps )
		{
			if( fps == 0 )
			{
				spook_log( SL_ERR,
					"v4l: kernel webcam drivers too old to set frame rate" );
				return -1;
			}
			spook_log( SL_INFO,
				"v4l: camera selected frame rate %d", fps );
			conf->fincr = 1;
			conf->fbase = fps;
		}
	}
#endif

#ifdef VIDIOCPWCGREALSIZE
	if( pwc && ioctl( conf->fd, VIDIOCPWCGREALSIZE, &pwc_imagesize ) == 0 )
	{
		real_width = pwc_imagesize.width;
		real_height = pwc_imagesize.height;
	} else
#endif
	{
		real_width = win.width;
		real_height = win.height;
	}

	if( real_width != conf->width || real_height != conf->height )
	{
		spook_log( SL_INFO, "v4l: hardware resized frames to %dx%d",
				real_width, real_height );
		conf->width = real_width;
		conf->height = real_height;

		win.x = win.y = 0;
		win.width = conf->width;
		win.height = conf->height;
		win.chromakey = 0;
		win.flags = 0;
		win.clips = NULL;
		win.clipcount = 0;
		
		if( ioctl( conf->fd, VIDIOCSWIN, &win ) < 0 )
		{
			spook_log( SL_ERR,
				"v4l: error setting image parameters" );
			return -1;
		}
	}

	memset( &conf->vm, 0, sizeof( conf->vm ) );

#ifdef HAVE_PWC_IOCTL_H
	if( pwc &&
		ioctl( conf->fd, VIDIOCPWCSAWB, &conf->pwc_whitebalance ) < 0 )
	{
		spook_log( SL_ERR,
			"v4l: error setting white balance on Philips webcam" );
		return -1;
	}
#endif

	if( ioctl( conf->fd, VIDIOCGMBUF, &conf->vm ) < 0 )
	{
		spook_log( SL_ERR, "v4l: error configuring driver for DMA" );
		return -1;
	}

	if( ! ( conf->mmap_buf = (unsigned char *)mmap( NULL, conf->vm.size, PROT_READ, MAP_SHARED, conf->fd, 0 ) ) )
	{
		spook_log( SL_ERR, "v4l: error mapping driver memory" );
		return -1;
	}

	return 0;
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct v4l_input *conf = (struct v4l_input *)s->private;

	*fincr = conf->fincr;
	*fbase = conf->fbase;
}

static void set_running( struct stream *s, int running )
{
	struct v4l_input *conf = (struct v4l_input *)s->private;

	conf->running = running;
}

#if 0
void v4l_close( int fd )
{
	munmap( m, vm.size );
	close( fd );
	xvid_encore( xvid_handle, XVID_ENC_DESTROY, NULL, NULL );
}
#endif

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct v4l_input *conf;

	conf = (struct v4l_input *)malloc( sizeof( struct v4l_input ) );
	conf->output = NULL;
	conf->device[0] = 0;
	conf->width = 0;
	conf->height = 0;
	conf->inputport = -1;
	conf->inputtype = 0;
#ifdef HAVE_PWC_IOCTL_H
	conf->pwc_whitebalance.mode = PWC_WB_AUTO;
#endif
	conf->fps = -1;
	conf->fd = -1;
	conf->running = 0;

	return conf;
}

static int end_block( void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;
	int i;

	if( ! conf->output )
	{
		spook_log( SL_ERR, "v4l: missing output stream name" );
		return -1;
	}
	if( ! conf->device[0] )
	{
		spook_log( SL_ERR, "v4l: missing V4L device name" );
		return -1;
	}
	switch( conf->inputtype )
	{
	case 0:
		spook_log( SL_ERR, "v4l: input type not specified" );
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
		conf->fincr = 1;
		conf->fbase = 25;
		if( conf->width == 0 )
		{
			conf->width = 320;
			conf->height = 240;
		}
		break;
	case INPUTTYPE_WEBCAM:
		if( conf->fps < 0 )
		{
			spook_log( SL_ERR,
				"v4l: framerate not specified for webcam" );
			return -1;
		} else if( conf->fps > 0 )
		{
			conf->fincr = 1;
			conf->fbase = conf->fps;
		} else spook_log( SL_INFO, "v4l: must figure out framerate, this will take some time..." );
		if( conf->inputport < 0 ) conf->inputport = 0;
		if( conf->width == 0 )
		{
			conf->width = 352;
			conf->height = 288;
		}
		break;
	}
	if( v4l_setup( conf ) < 0 ) return -1;
	conf->ex = new_exchanger( 8, get_back_frame, conf );
	for( i = 0; i < 8; ++i ) exchange_frame( conf->ex, new_frame() );
	pthread_create( &conf->thread, NULL, capture_loop, conf );

	return 0;
}

static int set_device( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	strcpy( conf->device, tokens[1].v.str );
	return 0;
}

static int set_output( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	conf->output = new_stream( tokens[1].v.str, FORMAT_RAW_UYVY, conf );
	if( ! conf->output )
	{
		spook_log( SL_ERR, "v4l: unable to create stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}
	conf->output->get_framerate = get_framerate;
	conf->output->set_running = set_running;
	return 0;
}

static int set_framesize( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	conf->width = tokens[1].v.num;
	conf->height = tokens[2].v.num;

	return 0;
}

static int set_inputport( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	conf->inputport = tokens[1].v.num;

	return 0;
}

static int set_inputtype( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	if( ! strcasecmp( tokens[1].v.str, "webcam" ) )
		conf->inputtype = INPUTTYPE_WEBCAM;
	else if( ! strcasecmp( tokens[1].v.str, "ntsc" ) )
		conf->inputtype = INPUTTYPE_NTSC;
	else if( ! strcasecmp( tokens[1].v.str, "pal" ) )
		conf->inputtype = INPUTTYPE_PAL;
	else
	{
		spook_log( SL_ERR,
			"v4l: video mode \"%s\" is unsupported; try NTSC, PAL or WEBCAM",
			tokens[1].v.str );
		return -1;
	}

	return 0;
}

static int set_framerate_num( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	if( conf->fps >= 0 )
	{
		spook_log( SL_ERR, "v4l: frame rate has already been set!" );
		return -1;
	}
	conf->fps = tokens[1].v.num;

	return 0;
}

static int set_framerate_str( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	if( strcasecmp( tokens[1].v.str, "auto" ) )
	{
		spook_log( SL_ERR,
			"v4l: frame rate should be a number or \"auto\"" );
		return -1;
	}

	conf->fps = 0;

	return 0;
}

#ifdef HAVE_PWC_IOCTL_H
static int set_pwc_whitebalance_str( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	if( ! strcasecmp( tokens[1].v.str, "auto" ) )
		conf->pwc_whitebalance.mode = PWC_WB_AUTO;
	else if( ! strcasecmp( tokens[1].v.str, "indoor" ) )
		conf->pwc_whitebalance.mode = PWC_WB_INDOOR;
	else if( ! strcasecmp( tokens[1].v.str, "outdoor" ) )
		conf->pwc_whitebalance.mode = PWC_WB_OUTDOOR;
	else if( ! strcasecmp( tokens[1].v.str, "fl" ) )
		conf->pwc_whitebalance.mode = PWC_WB_FL;
	else
	{
		spook_log( SL_ERR,
			"v4l: PWC-WhiteBalance setting \"%s\" is invalid",
			tokens[1].v.str );
		return -1;
	}

	return 0;
}

static int set_pwc_whitebalance_num( int num_tokens, struct token *tokens, void *d )
{
	struct v4l_input *conf = (struct v4l_input *)d;

	if( tokens[1].v.num < 0 || tokens[1].v.num > 65535 ||
		tokens[2].v.num < 0 || tokens[2].v.num > 65535 )
	{
		spook_log( SL_ERR,
			"v4l: PWC-WhiteBalance settings must be in the range 0-65535" );
		return -1;
	}
	conf->pwc_whitebalance.mode = PWC_WB_MANUAL;
	conf->pwc_whitebalance.manual_red = tokens[1].v.num;
	conf->pwc_whitebalance.manual_blue = tokens[2].v.num;

	return 0;
}
#endif

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "device", set_device, 1, 1, { TOKEN_STR } },
	{ "output", set_output, 1, 1, { TOKEN_STR } },
	{ "framesize", set_framesize, 2, 2, { TOKEN_NUM, TOKEN_NUM } },
	{ "inputport", set_inputport, 1, 1, { TOKEN_NUM } },
	{ "inputtype", set_inputtype, 1, 1, { TOKEN_STR } },
	{ "framerate", set_framerate_num, 1, 1, { TOKEN_NUM } },
	{ "framerate", set_framerate_str, 1, 1, { TOKEN_STR } },
#ifdef HAVE_PWC_IOCTL_H
	{ "pwc-whitebalance", set_pwc_whitebalance_str, 1, 1, { TOKEN_STR } },
	{ "pwc-whitebalance", set_pwc_whitebalance_num, 2, 2, { TOKEN_NUM, TOKEN_NUM } },
#endif

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

void v4l_init(void)
{
	register_config_context( "input", "v4l", start_block, end_block,
					config_statements );
}
