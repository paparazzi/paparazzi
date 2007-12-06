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
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <conversions.h>

struct converter {
	struct stream_destination *input;
	struct stream *output;
};

static struct stream *stream_list = NULL;

static void convert_uyvy_to_rgb24( struct frame *uyvy, void *d )
{
	struct stream *s = (struct stream *)d;
	struct frame *rgb;

	rgb = new_frame();
	rgb->format = FORMAT_RAW_RGB24;
	rgb->width = uyvy->width;
	rgb->height = uyvy->height;
	rgb->length = uyvy->height * uyvy->width * 3;
	rgb->key = uyvy->key;
	uyvy2rgb( uyvy->d, rgb->d, uyvy->length / 2 );
	unref_frame( uyvy );
	deliver_frame_to_stream( rgb, s );
}

static void get_framerate( struct stream *s, int *fincr, int *fbase )
{
	struct stream_destination *dest =
		(struct stream_destination *)s->private;

	dest->stream->get_framerate( dest->stream, fincr, fbase );
}

static void set_running( struct stream *s, int running )
{
	set_waiting( (struct stream_destination *)s->private, running );
}

void deliver_frame_to_stream( struct frame *f, void *d )
{
	struct stream *s = (struct stream *)d;
	struct stream_destination *dest;

	for( dest = s->dest_list; dest; dest = dest->next )
	{
		if( ! dest->waiting ) continue;
		ref_frame( f );
		dest->process_frame( f, dest->d );
	}

	unref_frame( f );
}

int format_match( int format, int *formats, int format_count )
{
	int i;

	if( format_count == 0 ) return 1;

	for( i = 0; i < format_count; ++i )
		if( format == formats[i] ) return 1;
	return 0;
}

static struct stream_destination *new_dest( struct stream *s,
			frame_deliver_func process_frame, void *d )
{
	struct stream_destination *dest;

	dest = (struct stream_destination *)
			malloc( sizeof( struct stream_destination ) );
	dest->next = s->dest_list;
	dest->prev = NULL;
	if( dest->next ) dest->next->prev = dest;
	s->dest_list = dest;
	dest->stream = s;
	dest->waiting = 0;
	dest->process_frame = process_frame;
	dest->d = d;

	return dest;
}

struct stream *new_convert_stream( struct stream *base, int format,
					frame_deliver_func convert )
{
	struct stream *s;

	s = (struct stream *)malloc( sizeof( struct stream ) );
	s->next = base->next;
	s->prev = base;
	base->next = s;
	if( s->next ) s->next->prev = s;
	strcpy( s->name, base->name );
	s->format = format;
	s->dest_list = NULL;
	s->get_framerate = get_framerate;
	s->set_running = set_running;
	s->private = new_dest( base, convert, s );
	return s;
}

struct stream_destination *connect_to_stream( char *name,
		frame_deliver_func process_frame, void *d,
		int *formats, int format_count )
{
	struct stream *s;
	int found_one = 0;

	for( s = stream_list; s; s = s->next )
		if( ! strcmp( s->name, name ) &&
			    format_match( s->format, formats, format_count ) )
			return new_dest( s, process_frame, d );

	for( s = stream_list; s; s = s->next )
		if( ! strcmp( s->name, name ) )
		{
			found_one = 1;
			switch( s->format )
			{
			case FORMAT_RAW_UYVY:
				if( format_match( FORMAT_RAW_RGB24,
						formats, format_count ) )
				{
					s = new_convert_stream( s,
							FORMAT_RAW_RGB24,
							convert_uyvy_to_rgb24 );
					return new_dest( s, process_frame, d );
				}
				break;
			}
		}

	if( found_one )
		spook_log( SL_ERR, "unable to convert stream %s", name );
	return NULL;
}

void del_stream( struct stream *s )
{
	if( s->next ) s->next->prev = s->prev;
	if( s->prev ) s->prev->next = s->next;
	else stream_list = s->next;
	free( s );
}

struct stream *new_stream( char *name, int format, void *d )
{
	struct stream *s;

	s = (struct stream *)malloc( sizeof( struct stream ) );
	s->next = stream_list;
	s->prev = NULL;
	if( s->next ) s->next->prev = s;
	stream_list = s;
	strcpy( s->name, name );
	s->format = format;
	s->dest_list = NULL;
	s->get_framerate = NULL;
	s->set_running = NULL;
	s->private = d;
	return s;
}

void set_waiting( struct stream_destination *dest, int waiting )
{
	struct stream *s = dest->stream;

	/* We call set_running every time a destination starts listening,
	 * or when the last destination stops listening.  It is good to know
	 * when new listeners come so maybe the source can send a keyframe. */

	if( dest->waiting )
	{
		if( waiting ) return; /* no change in status */
		dest->waiting = 0;

		/* see if anybody else is listening */
		for( dest = s->dest_list; dest; dest = dest->next )
			waiting |= dest->waiting;
		if( waiting ) return; /* others are still listening */
	} else
	{
		if( ! waiting ) return; /* no change in status */
		dest->waiting = 1;
	}

	s->set_running( s, waiting );
}
