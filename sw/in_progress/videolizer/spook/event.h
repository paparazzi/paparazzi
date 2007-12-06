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
#include <sys/time.h>

#ifndef _EVENT_H
#define _EVENT_H

#define EVENT_TIME		1
#define EVENT_FD		2
#define EVENT_ALWAYS		3
#define EVENT_SOFT_QUEUE	4

#define EVENT_F_ENABLED		1
#define EVENT_F_REMOVE		2
#define EVENT_F_ONESHOT		4
#define EVENT_F_RUNNING		8

struct event_info;

typedef struct timeval time_ref;
typedef void (*callback)( struct event_info *e, void *d );

struct time_event {
	time_ref fire;
	int ival;
};

struct fd_event {
	int fd;
	int write; // 0 = read, 1 = write
};

struct soft_event {
	struct soft_event *next;
	struct soft_event *prev;
	void *event_data;
};

struct soft_queue {
	pthread_mutex_t mutex;
	pthread_cond_t wait;
	struct soft_event *empty_se_list;
	struct soft_event *se_list;
	struct event *ev_list;
	int fds[2];
	struct event *fd_event;
};

struct event {
	struct event *prev;
	struct event *next;
	callback func;
	void *data;
	int type;
	int flags;
	union {
		struct time_event time;
		struct fd_event fd;
		struct soft_queue *sq;
	} ev;
};

struct event_info {
	struct event *e;
	int type;
	void *data;
};

int time_diff( time_ref *tr_start, time_ref *tr_end );
int time_ago( time_ref *tr );

void time_now( time_ref *tr );
void time_add( time_ref *tr, int msec );
void time_future( time_ref *tr, int msec );

struct soft_queue *new_soft_queue( int length );
void *get_next_event( struct soft_queue *sq );
int soft_queue_add( struct soft_queue *sq, void *d );
struct event *add_timer_event( int msec, unsigned int flags, callback f, void *d );
struct event *add_alarm_event( time_ref *t, unsigned int flags, callback f, void *d );
void resched_event( struct event *e, time_ref *t );
struct event *add_fd_event( int fd, int write, unsigned int flags, callback f, void *d );
struct event *add_always_event( unsigned int flags, callback f, void *d );
struct event *add_softqueue_event( struct soft_queue *sq, unsigned int flags,
		callback f, void *d );
void remove_event( struct event *e );
void set_event_interval( struct event *e, int msec );
void set_event_enabled( struct event *e, int enabled );
int get_event_enabled( struct event *e );
void exit_event_loop(void);
void event_loop( int single );

#endif /* EVENT_H */
