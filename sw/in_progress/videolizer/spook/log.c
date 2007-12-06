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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <pthread.h>

#define LOG_BUFFER_SIZE		4096

static int spookpid, minlevel;
static const char *month[] = {	"Jan", "Feb", "Mar", "Apr", "May", "Jun",
				"Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
static char *rb_data;
static int rb_in, rb_out, rb_size;
static pthread_mutex_t log_mutex;

int spook_log_init( int min )
{
	pthread_mutex_init( &log_mutex, NULL );
	spookpid = getpid();
	minlevel = min;
	rb_in = 0;
	rb_out = 0;
	rb_size = LOG_BUFFER_SIZE;
	rb_data = (char *)malloc( rb_size );
	return 0;
}

void spook_log( int level, char *fmt, ... )
{
	va_list ap;
	char newfmt[512], line[1024];
	struct tm tm;
	time_t t;
	int i, len, overwrite = 0;

	va_start( ap, fmt );
	if( level < minlevel )
	{
		va_end( ap );
		return;
	}
	time( &t );
	localtime_r( &t, &tm );
	sprintf( newfmt, "%s %2d %02d:%02d:%02d spook[%d]: %s\n",
			month[tm.tm_mon], tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec,
			spookpid, fmt );
	len = vsnprintf( line, sizeof( line ), newfmt, ap );
	va_end( ap );
	fputs( line, stderr );
	pthread_mutex_lock( &log_mutex );
	for( i = 0; i < len; ++i )
	{
		rb_data[rb_in++] = line[i];
		if( rb_in == rb_size ) rb_in = 0;
		if( rb_in == rb_out ) overwrite = 1;
	}
	if( overwrite )
	{
		rb_out = rb_in;
		while( rb_data[rb_out] != '\n' )
			if( ++rb_out == rb_size ) rb_out = 0;
		if( ++rb_out == rb_size ) rb_out = 0;
	}
	pthread_mutex_unlock( &log_mutex );
}

void send_log_buffer( int fd )
{
	pthread_mutex_lock( &log_mutex );
	if( rb_in < rb_out )
	{
		write( fd, rb_data + rb_out, rb_size - rb_out );
		write( fd, rb_data, rb_in );
	} else if( rb_in > rb_out )
		write( fd, rb_data + rb_out, rb_in - rb_out );
	pthread_mutex_unlock( &log_mutex );
}
