/*
 *      mkcostab.c -- cosine table generator
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
/* ---------------------------------------------------------------------- */

#define COSTABSIZE 0x400

/* ---------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
	int i;
	FILE *fi, *ff;
	float f;

	if (!(fi = fopen("costabi.c", "w")))
		exit(1);
	if (!(ff = fopen("costabf.c", "w")))
		exit(1);
	fprintf(fi, "/*\n * This file is machine generated, DO NOT EDIT!\n */\n\n"
		"int costabi[%i] = {", COSTABSIZE);
	fprintf(ff, "/*\n * This file is machine generated, DO NOT EDIT!\n */\n\n"
		"float costabf[%i] = {", COSTABSIZE);
	for (i = 0; i < COSTABSIZE; i++) {
		if ((i & 3) == 0)
			fprintf(ff, "\n\t");
		if ((i & 7) == 0)
			fprintf(fi, "\n\t");
		f = cos(M_PI*2.0*i/COSTABSIZE);
		fprintf(ff, "%12.9f", f);
		fprintf(fi, "%6i", (int)(32767.0*f));
		if (i < COSTABSIZE-1) {
			fprintf(ff, ", ");
			fprintf(fi, ", ");
		}
	}
	fprintf(ff, "\n};\n");
	fprintf(fi, "\n};\n");
	exit(0);
}
