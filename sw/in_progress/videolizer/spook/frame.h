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

#define FORMAT_EMPTY		0
#define FORMAT_RAW_RGB24	1
#define FORMAT_RAW_UYVY		2
#define FORMAT_RAW_BGR24	3
#define FORMAT_PCM		64
#define FORMAT_MPEG4		100
#define FORMAT_JPEG		101
#define FORMAT_MPV		102
#define FORMAT_H263		103
#define FORMAT_MPA		200
#define FORMAT_ALAW		201

struct frame;

typedef int (*frame_destructor)( struct frame *f, void *d );

struct frame
{
	int ref_count;
	int size;
	pthread_mutex_t mutex; /* only used to lock the ref_count */
	frame_destructor destructor;
	void *destructor_data;
	int format;
	int width;
	int height;
	int length;
	int key;
	int step;
	unsigned char *d;
};

void init_frame_heap( int size, int count );
int get_max_frame_size(void);
struct frame *new_frame(void);
struct frame *clone_frame( struct frame *orig );
void ref_frame( struct frame *f );
void unref_frame( struct frame *f );


struct frame_slot {
	struct frame_slot *next;
	struct frame_slot *prev;
	int pending;
	struct frame *f;
};

typedef void (*frame_deliver_func)( struct frame *f, void *d );

struct frame_exchanger {
	int master_fd;
	int slave_fd;
	pthread_mutex_t mutex;
	pthread_cond_t slave_wait;
	struct event *master_event;
	frame_deliver_func f;
	void *d;
	struct frame_slot *slave_cur;
	struct frame_slot *master_read;
	struct frame_slot *master_write;
};

struct meter {
	int started;
	int downstream;
	time_ref last_check;
	int rate;
	int ticks;
	double slip;
	double avg_slip;
	int precomp;
	int correction_rate;
	int corrected;
};

struct frame_exchanger *new_exchanger( int slots,
					frame_deliver_func func, void *d );

/* master functions */
int exchange_frame( struct frame_exchanger *ex, struct frame *frame );

/* slave functions */
struct frame *get_next_frame( struct frame_exchanger *ex, int wait );
void deliver_frame( struct frame_exchanger *ex, struct frame *f );

/* sample rate metering functions */
void meter_init( struct meter *m, int fbase, int downstream );
int meter_count( struct meter *m, int ticks, int *rate );
int meter_get_adjustment( struct meter *m );
void meter_report_correction( struct meter *m, int ticks );

/* audio.c */
struct audio_ring;
struct audio_ring *new_audio_ring( int sample_size, int sample_rate,
		int framelen, struct soft_queue *queue );
void audio_ring_input( struct audio_ring *ring, unsigned char *d, int len );
