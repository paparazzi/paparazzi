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

struct stream_destination {
	struct stream_destination *next;
	struct stream_destination *prev;
	struct stream *stream;
	int waiting;
	frame_deliver_func process_frame;
	void *d;
};

struct stream {
	struct stream *next;
	struct stream *prev;
	char name[256];
	int format;
	struct stream_destination *dest_list;
	void (*get_framerate)( struct stream *s, int *fincr, int *fbase );
	void (*set_running)( struct stream *s, int running );
	void *private;
};

struct stream *new_stream( char *name, int format, void *d );
void deliver_frame_to_stream( struct frame *f, void *d );
struct stream_destination *connect_to_stream( char *name,
		frame_deliver_func process_frame, void *d,
		int *formats, int format_count );
void del_stream( struct stream *s );
void set_waiting( struct stream_destination *dest, int waiting );
