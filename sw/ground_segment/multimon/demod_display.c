/*
 *      demod_display.c -- signal display
 *
 *      Copyright (C) 1996
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* ---------------------------------------------------------------------- */

#include "multimon.h"
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <signal.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

/* ---------------------------------------------------------------------- */

#define SAMPLING_RATE 22050

/* ---------------------------------------------------------------------- */

static void scope_init(struct demod_state *s)
{
	memset(&s->l1.scope, 0, sizeof(s->l1.scope));
	s->l1.scope.dispnum = xdisp_start();
	if (s->l1.scope.dispnum == -1)
		return;
}

/* ---------------------------------------------------------------------- */

#define MEMSIZE sizeof(s->l1.scope.data)/sizeof(s->l1.scope.data[0])

static void scope_demod(struct demod_state *s, float *buffer, int length)
{
	float *src, *dst;
	int i;

	if (s->l1.scope.dispnum == -1)
		return;
	if (length >= MEMSIZE) {
		src = buffer+length-MEMSIZE;
		dst = s->l1.scope.data;
		i = MEMSIZE;
	} else {
		i = MEMSIZE-length;
		memmove(s->l1.scope.data, s->l1.scope.data+i,
			i*sizeof(s->l1.scope.data[0]));
		src = buffer;
		dst = s->l1.scope.data+i;
		i = length;
	}
	s->l1.scope.datalen += i;
	memcpy(dst, src, i*sizeof(s->l1.scope.data[0]));
	if (s->l1.scope.datalen < MEMSIZE)
		return;
	if (xdisp_update(s->l1.scope.dispnum, s->l1.scope.data))
		s->l1.scope.datalen = 0;
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_scope = {
	"SCOPE", SAMPLING_RATE, 0, scope_init, scope_demod
};

/* ---------------------------------------------------------------------- */
