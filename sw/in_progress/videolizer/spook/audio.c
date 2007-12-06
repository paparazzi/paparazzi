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

struct audio_ring {
	struct meter meter;
	struct soft_queue *queue;
	int framelen;
	int sample_size;
	int sample_rate;
	int adjust_rate;
	int adjust_count;
	int i_r;
	int i_w;
	int buflen;
	unsigned char *buf;
};

struct audio_ring *new_audio_ring( int sample_size, int sample_rate,
		int framelen, struct soft_queue *queue )
{
	struct audio_ring *ring;

	if( ! ( ring = malloc( sizeof( struct audio_ring ) ) ) )
		return NULL;
	meter_init( &ring->meter, sample_rate, 1 );
	ring->queue = queue;
	ring->framelen = framelen;
	ring->sample_size = sample_size;
	ring->sample_rate = sample_rate;
	ring->adjust_rate = 0;
	ring->adjust_count = 0;
	ring->i_r = ring->i_w = 0;
	ring->buflen = sample_size * sample_rate;
	ring->buflen -= ring->buflen % framelen;
	if( ! ( ring->buf = malloc( ring->buflen ) ) )
	{
		free( ring );
		return NULL;
	}
	return ring;
}

void audio_ring_input( struct audio_ring *ring, unsigned char *d, int len )
{
	int i, correction = 0;
	struct frame *f;
	int avail;

	if( meter_count( &ring->meter, len / ring->sample_size,
				&ring->adjust_rate ) )
		ring->adjust_count = 1;

	if( ring->i_w < ring->i_r )
		avail = ring->buflen - ring->i_r + ring->i_w;
	else
		avail = ring->i_w - ring->i_r;

	for( i = 0; i < len; i += ring->sample_size )
	{
		if( ring->adjust_rate < 0 && --ring->adjust_count == 0 )
		{
			ring->adjust_count = -ring->adjust_rate;
			--correction;
			continue;
		} else if( ring->adjust_rate > 0 && --ring->adjust_count == 0 )
		{
			ring->adjust_count = ring->adjust_rate;
			++correction;
			memcpy( ring->buf + ring->i_w, d + i,
					ring->sample_size );
			ring->i_w += ring->sample_size;
			if( ring->i_w == ring->buflen ) ring->i_w = 0;
			avail += ring->sample_size;
		}
		memcpy( ring->buf + ring->i_w, d + i, ring->sample_size );
		ring->i_w += ring->sample_size;
		if( ring->i_w == ring->buflen ) ring->i_w = 0;
		avail += ring->sample_size;
		if( avail >= ring->framelen )
		{
			if( ( f = new_frame() ) )
			{
				f->format = FORMAT_PCM;
				f->width = 0;
				f->height = 0;
				f->key = 1;
				f->step = ring->sample_size;
				f->length = ring->framelen;
				f->d = ring->buf + ring->i_r;
				if( soft_queue_add( ring->queue, f ) < 0 )
					unref_frame( f );
			} else spook_log( SL_WARN, "audio: dropping frame" );
			ring->i_r += ring->framelen;
			if( ring->i_r == ring->buflen ) ring->i_r = 0;
			avail -= ring->framelen;
		}
	}
	meter_report_correction( &ring->meter, correction );
}
