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
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>

#include <event.h>
#include <log.h>
#include <frame.h>

// #define METER_DEBUG

static struct frame_slot *frame_heap_take = NULL, *frame_heap_put = NULL;
static pthread_mutex_t frame_heap_mutex;
static int max_frame_size = 0;

void init_frame_heap( int size, int count )
{
	struct frame_slot *f = NULL, *prev = NULL;

	max_frame_size = size;
	pthread_mutex_init( &frame_heap_mutex, NULL );
	while( count-- > 0 )
	{
		f = (struct frame_slot *)
				malloc( sizeof( struct frame_slot ) );
		f->f = (struct frame *)malloc( sizeof( struct frame ) + size );
		f->f->size = size;
		pthread_mutex_init( &f->f->mutex, NULL );
		f->prev = prev;
		if( prev ) prev->next = f;
		else frame_heap_take = f;
		prev = f;
	}
	frame_heap_take->prev = f;
	f->next = frame_heap_take;
	frame_heap_put = frame_heap_take;
}

int get_max_frame_size(void)
{
	return max_frame_size;
}

struct frame *new_frame(void)
{
	struct frame *f;

	pthread_mutex_lock( &frame_heap_mutex );

	if( frame_heap_take->f )
	{
		f = frame_heap_take->f;
		frame_heap_take->f = NULL;
		frame_heap_take = frame_heap_take->next;
		f->ref_count = 1;
		f->destructor = NULL;
		f->destructor_data = NULL;
		f->d = (unsigned char *)f + sizeof( struct frame );
		f->format = FORMAT_EMPTY;
		f->width = 0;
		f->height = 0;
		f->length = 0;
		f->key = 0;
	} else
	{
		spook_log( SL_WARN, "Ack!  Out of frame buffers!" );
		f = NULL;
	}

	pthread_mutex_unlock( &frame_heap_mutex );

	return f;
}

static int clone_destructor( struct frame *f, void *d )
{
	unref_frame( (struct frame *)d );
	return 0;
}

struct frame *clone_frame( struct frame *orig )
{
	struct frame *f;

	if( ! ( f = new_frame() ) ) return NULL;
	f->destructor = clone_destructor;
	f->destructor_data = orig;
	f->format = orig->format;
	f->width = orig->width;
	f->height = orig->height;
	f->length = orig->length;
	f->key = orig->key;
	f->step = orig->step;
	f->d = orig->d;
	ref_frame( orig );
	return f;
}

void ref_frame( struct frame *f )
{
	pthread_mutex_lock( &f->mutex );
	++f->ref_count;
	pthread_mutex_unlock( &f->mutex );
}

void unref_frame( struct frame *f )
{
	int r;

	pthread_mutex_lock( &f->mutex );
	r = --f->ref_count;
	pthread_mutex_unlock( &f->mutex );
	if( r > 0 ) return;

	if( f->destructor )
	{
		f->ref_count = 1;
		if( f->destructor( f, f->destructor_data ) ) return;
	}

	pthread_mutex_lock( &frame_heap_mutex );
	if( frame_heap_put->f )
	{
		spook_log( SL_WARN, "Ack!  There is a frame at frame_heap_put!" );
		return;
	}
	frame_heap_put->f = f;
	frame_heap_put = frame_heap_put->next;
	pthread_mutex_unlock( &frame_heap_mutex );
}

static void exchanger_read( struct event_info *ei, void *d )
{
	struct frame_exchanger *ex = (struct frame_exchanger *)d;
	unsigned char c;
	int ret;
	struct frame *f;

	for(;;)
	{
		ret = read( ex->master_fd, &c, 1 );
		if( ret <= 0 )
		{
			if( ret < 0 && errno == EAGAIN ) return;
			spook_log( SL_ERR, "We lost an exchanger fd!" );
			exit( 1 );
		}
		pthread_mutex_lock( &ex->mutex );
		f = ex->master_read->f;
		ex->master_read->f = NULL;
		ex->master_read = ex->master_read->next;
		pthread_mutex_unlock( &ex->mutex );
		ex->f( f, ex->d );
	}
}

struct frame_exchanger *new_exchanger( int slots,
					frame_deliver_func func, void *d )
{
	struct frame_slot *f = NULL, *prev = NULL;
	struct frame_exchanger *ex;
	int fds[2];

	ex = (struct frame_exchanger *)
			malloc( sizeof( struct frame_exchanger ) );

	while( slots-- > 0 )
	{
		f = (struct frame_slot *)
				malloc( sizeof( struct frame_slot ) );
		f->f = NULL;
		f->pending = 0;
		f->prev = prev;
		if( prev ) prev->next = f;
		else ex->slave_cur = f;
		prev = f;
	}
	ex->slave_cur->prev = f;
	f->next = ex->slave_cur;
	ex->master_read = ex->master_write = ex->slave_cur;

	pipe( fds );
	ex->master_fd = fds[0];
	ex->slave_fd = fds[1];

	fcntl( ex->master_fd, F_SETFL, O_NONBLOCK );
	ex->master_event = add_fd_event( ex->master_fd, 0, 0,
						exchanger_read, ex );

	pthread_mutex_init( &ex->mutex, NULL );
	pthread_cond_init( &ex->slave_wait, NULL );

	ex->f = func;
	ex->d = d;

	return ex;
}

int exchange_frame( struct frame_exchanger *ex, struct frame *frame )
{
	pthread_mutex_lock( &ex->mutex );
	if( ex->master_write->f )
	{
		spook_log( SL_WARN, "Exchanger is full, dropping frame!" );
		pthread_mutex_unlock( &ex->mutex );
		return -1;
	}
	ex->master_write->f = frame;
	ex->master_write->pending = 1;
	ex->master_write = ex->master_write->next;
	pthread_cond_signal( &ex->slave_wait );
	pthread_mutex_unlock( &ex->mutex );

	return 0;
}

struct frame *get_next_frame( struct frame_exchanger *ex, int wait )
{
	struct frame *f = NULL;

	pthread_mutex_lock( &ex->mutex );
	if( ex->slave_cur->pending ) f = ex->slave_cur->f;
	if( ! f && wait )
	{
		pthread_cond_wait( &ex->slave_wait, &ex->mutex );
		if( ex->slave_cur->pending ) f = ex->slave_cur->f;
		if( ! f ) spook_log( SL_WARN, "Slave signalled but no frame??" );
	}
	pthread_mutex_unlock( &ex->mutex );
	return f;
}

void deliver_frame( struct frame_exchanger *ex, struct frame *f )
{
	unsigned char c = 0;

	ex->slave_cur->f = f;
	ex->slave_cur->pending = 0;
	if( write( ex->slave_fd, &c, 1 ) <= 0 ) exit( 0 );
	ex->slave_cur = ex->slave_cur->next;
}

void meter_init( struct meter *m, int fbase, int downstream )
{
	m->started = 0;
	m->downstream = downstream;
	m->rate = fbase;
	m->ticks = 0;
	m->slip = 0;
	m->avg_slip = 0;
	m->precomp = 0;
	m->correction_rate = 0;
	m->corrected = 0;
}

int meter_count( struct meter *m, int ticks, int *rate )
{
	int msec;
	time_ref now;
	double expected;

	if( ! m->started )
	{
		time_now( &m->last_check );
		m->started = 1;
		m->ticks = ticks;
		return 0;
	}
	time_now( &now );
	msec = time_diff( &m->last_check, &now );
	if( msec < 30000 )
	{
		m->ticks += ticks;
		return 0;
	}
	expected = m->rate * (double)msec / 1000.0;
	if( ! m->downstream ) expected -= m->corrected;
#ifdef METER_DEBUG
	printf( "meter: expected %f in %d msec, got %d\n",
			expected, msec, m->ticks );
#endif
	m->slip += expected - (double)m->ticks - (double)m->precomp;
	m->slip += m->corrected;
	m->avg_slip = 4.0 * m->avg_slip / 5.0 +
		( expected - (double)m->ticks ) * 1000.0 / (double)msec / 5.0;
	/* Pre-compensate by 1/2 the average slip */
	m->precomp = 30 * m->avg_slip / 2;
#ifdef METER_DEBUG
	printf( "meter: total slip = %f avg slip = %.3f/sec precomp = %d\n",
			m->slip, m->avg_slip, m->precomp );
#endif
	m->slip += m->precomp;
	m->correction_rate = m->ticks / m->slip;
	if( rate ) *rate = m->correction_rate;
	m->ticks = ticks;
	m->last_check = now;
	m->corrected = 0;
	return rate != NULL;
}

int meter_get_adjustment( struct meter *m )
{
	int msec;

	if( m->slip == 0 ) return 0;
	msec = time_ago( &m->last_check );
	return m->slip * msec / 30000 + m->corrected;
}

void meter_report_correction( struct meter *m, int ticks )
{
	m->corrected -= ticks;
#ifdef METER_DEBUG
//	printf( "meter: total slip = %f correction = %d\n",
//			m->slip, m->corrected );
#endif
}
