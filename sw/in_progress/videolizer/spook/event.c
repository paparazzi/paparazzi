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

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include "event.h"

static struct event *time_event_list = NULL;
static struct event *fd_event_list = NULL;
static struct event *always_event_list = NULL;
static int end_loop = 0;

int time_diff( time_ref *tr_start, time_ref *tr_end )
{
	return ( ( tr_end->tv_sec - tr_start->tv_sec ) * 1000000
		+ tr_end->tv_usec - tr_start->tv_usec + 500 ) / 1000;
}

int time_ago( time_ref *tr )
{
	struct timeval now;

	gettimeofday( &now, NULL );
	return time_diff( tr, &now );
}

void time_now( time_ref *tr )
{
	gettimeofday( (struct timeval *)tr, NULL );
}

void time_add( time_ref *tr, int msec )
{
	tr->tv_sec += msec / 1000;
	tr->tv_usec += ( msec % 1000 ) * 1000;
	if( tr->tv_usec >= 1000000 )
	{
		tr->tv_usec -= 1000000;
		++tr->tv_sec;
	}
}

void time_future( time_ref *tr, int msec )
{
	gettimeofday( tr, NULL );
	time_add( tr, msec );
}

static inline void remove_from_list( struct soft_event **se )
{
	if( (*se)->next != *se )
	{
		(*se)->prev->next = (*se)->next;
		(*se)->next->prev = (*se)->prev;
		*se = (*se)->next;
	} else *se = NULL;
}

static inline void add_to_list( struct soft_event *se, struct soft_event **list )
{
	if( *list )
	{
		se->next = *list;
		se->prev = (*list)->prev;
		se->next->prev = se;
		se->prev->next = se;
	} else
	{
		se->next = se;
		se->prev = se;
		*list = se;
	}
}

struct soft_queue *new_soft_queue( int length )
{
	struct soft_queue *sq;
	struct soft_event *se;
	int i;

	sq = (struct soft_queue *)malloc( sizeof( struct soft_queue ) );
	sq->empty_se_list = NULL;
	sq->se_list = NULL;
	sq->ev_list = NULL;
	sq->fd_event = NULL;
	pthread_mutex_init( &sq->mutex, NULL );
	pthread_cond_init( &sq->wait, NULL );
	for( i = 0; i < length; ++i )
	{
		se = (struct soft_event *)malloc( sizeof( struct soft_event ) );
		add_to_list( se, &sq->empty_se_list );
	}
	return sq;
}

static struct event *new_event( callback f, void *d )
{
	struct event *e;

	e = (struct event *)malloc( sizeof( struct event ) );
	e->next = NULL;
	e->prev = NULL;
	e->type = 0;
	e->flags = 0;
	e->func = f;
	e->data = d;
	return e;
}

static void strip_events( struct event **list )
{
	struct event *e, *n;

	for( e = *list; e; e = n )
	{
		n = e->next;
		if( e->flags & EVENT_F_REMOVE )
		{
			if( e->next ) e->next->prev = e->prev;
			if( e->prev ) e->prev->next = e->next;
			else *list = e->next;
		}
	}
}

struct event *add_timer_event( int msec, unsigned int flags, callback f, void *d )
{
	struct event *e;

	e = new_event( f, d );
	e->type = EVENT_TIME;
	e->flags = flags;
	e->ev.time.ival = msec;
	e->next = time_event_list;
	if( e->next ) e->next->prev = e;
	time_event_list = e;
	time_now( &e->ev.time.fire );
	resched_event( e, NULL );
	return e;
}

struct event *add_alarm_event( time_ref *t, unsigned int flags, callback f, void *d )
{
	struct event *e;

	e = new_event( f, d );
	e->type = EVENT_TIME;
	e->flags = flags | EVENT_F_ONESHOT;
	e->next = time_event_list;
	if( e->next ) e->next->prev = e;
	time_event_list = e;
	resched_event( e, t );
	return e;
}

void resched_event( struct event *e, time_ref *tr )
{
	if( tr )
		e->ev.time.fire = *tr;
	else if( e->flags & EVENT_F_ENABLED )
		time_add( &e->ev.time.fire, e->ev.time.ival );
	else
		time_future( &e->ev.time.fire, e->ev.time.ival );

	e->flags &= ~EVENT_F_REMOVE;
	e->flags |= EVENT_F_ENABLED;
}

struct event *add_fd_event( int fd, int write, unsigned int flags, callback f, void *d )
{
	struct event *e;

	e = new_event( f, d );
	e->type = EVENT_FD;
	e->flags = flags | EVENT_F_ENABLED;
	e->ev.fd.fd = fd;
	e->ev.fd.write = write;
	e->next = fd_event_list;
	if( e->next ) e->next->prev = e;
	fd_event_list = e;
	return e;
}

struct event *add_always_event( unsigned int flags, callback f, void *d )
{
	struct event *e;

	e = new_event( f, d );
	e->type = EVENT_ALWAYS;
	e->flags = flags | EVENT_F_ENABLED;
	e->next = always_event_list;
	if( e->next ) e->next->prev = e;
	always_event_list = e;
	return e;
}

static void *locked_get_next_event( struct soft_queue *sq )
{
	struct soft_event *se;
	void *d;

	if( ! sq->se_list ) return NULL;

	se = sq->se_list;
	remove_from_list( &sq->se_list );
	d = se->event_data;
	add_to_list( se, &sq->empty_se_list );

	return d;
}

void *get_next_event( struct soft_queue *sq )
{
	void *d;

	pthread_mutex_lock( &sq->mutex );
	if( ! sq->se_list ) pthread_cond_wait( &sq->wait, &sq->mutex );
	d = locked_get_next_event( sq );
	pthread_mutex_unlock( &sq->mutex );
	return d;
}

int soft_queue_add( struct soft_queue *sq, void *d )
{
	struct soft_event *se;
	unsigned char c = 0;

	pthread_mutex_lock( &sq->mutex );

	if( ! ( se = sq->empty_se_list ) )
	{
		pthread_mutex_unlock( &sq->mutex );
		return -1;
	}
	remove_from_list( &sq->empty_se_list );
	add_to_list( se, &sq->se_list );
	se->event_data = d;

	if( sq->fd_event ) write( sq->fds[1], &c, 1 );
	else pthread_cond_signal( &sq->wait );

	pthread_mutex_unlock( &sq->mutex );

	return 0;
}

static void sq_run( struct event_info *ei, void *d )
{
	struct soft_queue *sq = (struct soft_queue *)d;
	struct event_info i;
	struct event *e;
	unsigned char c;

	pthread_mutex_lock( &sq->mutex );

	if( read( sq->fds[0], &c, 1 ) < 1 )
	{
		remove_event( sq->fd_event );
		sq->fd_event = NULL;
		close( sq->fds[0] );
		close( sq->fds[1] );
		pthread_mutex_unlock( &sq->mutex );
		return;
	}
	i.type = EVENT_SOFT_QUEUE;
	i.data = locked_get_next_event( sq );
	for( e = sq->ev_list; e; e = e->next )
	{
		if( end_loop ) break;
		if( e->ev.sq == sq )
		{
			i.e = e;
			if( e->flags & EVENT_F_ONESHOT )
				e->flags |= EVENT_F_REMOVE;
			pthread_mutex_unlock( &sq->mutex );
			(*e->func)( &i, e->data );
			pthread_mutex_lock( &sq->mutex );
		}
	}
	strip_events( &sq->ev_list );

	pthread_mutex_unlock( &sq->mutex );
}

struct event *add_softqueue_event( struct soft_queue *sq, unsigned int flags,
		callback f, void *d )
{
	struct event *e;

	pthread_mutex_lock( &sq->mutex );

	if( ! sq->fd_event )
	{
		if( pipe( sq->fds ) < 0 )
		{
			pthread_mutex_unlock( &sq->mutex );
			return NULL;
		}
		sq->fd_event = add_fd_event( sq->fds[0], 0, 0, sq_run, sq );
	}
	e = new_event( f, d );
	e->type = EVENT_SOFT_QUEUE;
	e->flags = flags | EVENT_F_ENABLED;
	e->ev.sq = sq;
	e->next = sq->ev_list;
	if( e->next ) e->next->prev = e;
	sq->ev_list = e;

	pthread_mutex_unlock( &sq->mutex );

	return e;
}

void remove_event( struct event *e )
{
	e->flags |= EVENT_F_REMOVE;
	e->flags &= ~( EVENT_F_RUNNING | EVENT_F_ENABLED );
}

void set_event_interval( struct event *e, int msec )
{
	e->ev.time.ival = msec;
	if( e->flags & EVENT_F_ENABLED ) resched_event( e, NULL );
}

void set_event_enabled( struct event *e, int enabled )
{
	e->flags &= ~EVENT_F_ENABLED;
	if( enabled ) e->flags |= EVENT_F_ENABLED;
}

int get_event_enabled( struct event *e )
{
	return e->flags & EVENT_F_ENABLED ? 1 : 0;
}

void exit_event_loop(void)
{
	end_loop = 1;
}

void event_loop( int single )
{
	struct timeval t, *st;
	struct event *e;
	struct event_info ei;
	int diff, nexttime = 0, highfd, ret;
	fd_set rfds, wfds;

	end_loop = 0;

	do {
		st = NULL;
		/* check how long the timeout should be */
		for( e = time_event_list; e; e = e->next )
			if( e->flags & EVENT_F_ENABLED )
			{
				diff = -time_ago( &e->ev.time.fire );
				if( diff < 5 ) diff = 0;
				if( ! st || diff < nexttime ) nexttime = diff;
				st = &t;
				e->flags |= EVENT_F_RUNNING;
			} else e->flags &= ~EVENT_F_RUNNING;
		for( e = always_event_list; e; e = e->next )
			if( e->flags & EVENT_F_ENABLED )
			{
				st = &t;
				nexttime = 0;
				e->flags |= EVENT_F_RUNNING;
			} else e->flags &= ~EVENT_F_RUNNING;
		if( st )
		{
			st->tv_sec = nexttime / 1000;
			st->tv_usec = ( nexttime % 1000 ) * 1000;
		}
		FD_ZERO( &rfds );
		FD_ZERO( &wfds );
		highfd = -1;
		/* This is all so ugly...  It should use poll() eventually. */
		for( e = fd_event_list; e; e = e->next )
		{
			if( e->flags & EVENT_F_ENABLED )
			{
				FD_SET( e->ev.fd.fd,
					e->ev.fd.write ? &wfds : &rfds );
				if( e->ev.fd.fd > highfd )
					highfd = e->ev.fd.fd;
				e->flags |= EVENT_F_RUNNING;
			} else e->flags &= ~EVENT_F_RUNNING;
		}
		ret = select( highfd + 1, &rfds, &wfds, NULL, st );
		for( e = time_event_list; e; e = e->next )
		{
			if( ! ( e->flags & EVENT_F_RUNNING ) ) continue;
			if( end_loop ) break;
			diff = -time_ago( &e->ev.time.fire );
			if( diff < 5 )
			{
				if( ! ( e->flags & EVENT_F_ONESHOT ) )
					resched_event( e, NULL );
				else e->flags |= EVENT_F_REMOVE;
				ei.e = e;
				ei.type = EVENT_TIME;
				ei.data = NULL;
				(*e->func)( &ei, e->data );
			}
		}
		for( e = always_event_list; e; e = e->next )
		{
			if( ! ( e->flags & EVENT_F_RUNNING ) ) continue;
			if( end_loop ) break;
			if( e->flags & EVENT_F_ONESHOT )
				e->flags |= EVENT_F_REMOVE;
			ei.e = e;
			ei.type = EVENT_ALWAYS;
			ei.data = NULL;
			(*e->func)( &ei, e->data );
		}
		if( ret > 0 ) for( e = fd_event_list; e; e = e->next )
		{
			if( ! ( e->flags & EVENT_F_RUNNING ) ) continue;
			if( end_loop ) break;
			if( FD_ISSET( e->ev.fd.fd,
					e->ev.fd.write ? &wfds : &rfds ) )
			{
				if( e->flags & EVENT_F_ONESHOT )
					e->flags |= EVENT_F_REMOVE;
				ei.e = e;
				ei.type = EVENT_FD;
				ei.data = NULL;
				(*e->func)( &ei, e->data );
			}
		}
		strip_events( &time_event_list );
		strip_events( &fd_event_list );
		strip_events( &always_event_list );
	} while( ! end_loop && ! single );
}
