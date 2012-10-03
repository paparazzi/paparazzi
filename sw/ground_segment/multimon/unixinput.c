/*
 *      unixinput.c -- input sound samples
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
#include "pprz.h"
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>
#include <stdlib.h>

#include <sys/soundcard.h>
#include <sys/ioctl.h>

/* ---------------------------------------------------------------------- */

static const char *allowed_types[] = {
  "raw", "aiff", "au", "hcom", "sf", "voc", "cdr", "dat",
  "smp", "wav", "maud", "vwe", NULL
};

/* ---------------------------------------------------------------------- */

static const struct demod_param *dem[] = { ALL_DEMOD };

#define NUMDEMOD (sizeof(dem)/sizeof(dem[0]))

static struct demod_state dem_st[NUMDEMOD];
static unsigned int dem_mask[(NUMDEMOD+31)/32];

#define MASK_SET(n) dem_mask[(n)>>5] |= 1<<((n)&0x1f)
#define MASK_RESET(n) dem_mask[(n)>>5] &= ~(1<<((n)&0x1f))
#define MASK_ISSET(n) (dem_mask[(n)>>5] & 1<<((n)&0x1f))

/* ---------------------------------------------------------------------- */

static int verbose_level = 0;

/* ---------------------------------------------------------------------- */

void verbprintf(int verb_level, const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  if (verb_level <= verbose_level)
    vfprintf(stdout, fmt, args);
  va_end(args);
}

/* ---------------------------------------------------------------------- */

static void process_buffer(float *buf, unsigned int len)
{
  int i;

  for (i = 0; i <  NUMDEMOD; i++)
    if (MASK_ISSET(i) && dem[i]->demod)
      dem[i]->demod(dem_st+i, buf, len);
}

/* ---------------------------------------------------------------------- */

static void input_sound(unsigned int sample_rate, unsigned int overlap,
			const char *ifname)
{
  int sndparam;
  int fd;
  union {
    short s[8192];
    unsigned char b[8192];
  } b;
  float fbuf[16384];
  unsigned int fbuf_cnt = 0;
  int i;
  short *sp;
  unsigned char *bp;
  int fmt = 0;
  int stereo = 0;

  if ((fd = open(ifname ? ifname : "/dev/dsp", O_RDONLY)) < 0) {
    perror("open");
    exit (10);
  }
  sndparam = AFMT_S16_LE; /* we want 16 bits/sample signed */
  /* little endian; works only on little endian systems! */
  if (ioctl(fd, SNDCTL_DSP_SETFMT, &sndparam) == -1) {
    perror("ioctl: SNDCTL_DSP_SETFMT");
    exit (10);
  }
  if (sndparam != AFMT_S16_LE) {
    fmt = 1;
    sndparam = AFMT_U8;
    if (ioctl(fd, SNDCTL_DSP_SETFMT, &sndparam) == -1) {
      perror("ioctl: SNDCTL_DSP_SETFMT");
      exit (10);
    }
    if (sndparam != AFMT_U8) {
      perror("ioctl: SNDCTL_DSP_SETFMT");
      exit (10);
    }
  }
  sndparam = 0;   /* we want only 1 channel */
  if (ioctl(fd, SNDCTL_DSP_STEREO, &sndparam) == -1) {
    perror("ioctl: SNDCTL_DSP_STEREO");
    exit (10);
  }
  if (sndparam == 1) {
    fprintf(stderr, "soundif: Warning, cannot set the channel "
	    "number to 1, will use stereo\n");
    stereo=1;
  } else
    if (sndparam != 0) {
      fprintf(stderr, "soundif: Error, cannot set the channel "
	      "number to 1\n");
      exit (10);
    }
  sndparam = sample_rate;
  if (ioctl(fd, SNDCTL_DSP_SPEED, &sndparam) == -1) {
    perror("ioctl: SNDCTL_DSP_SPEED");
    exit (10);
  }
  if ((10*abs(sndparam-sample_rate)) > sample_rate) {
    perror("ioctl: SNDCTL_DSP_SPEED");
    exit (10);
  }
  if (sndparam != sample_rate) {
    fprintf(stderr, "Warning: Sampling rate is %u, "
	    "requested %u\n", sndparam, sample_rate);
  }
#if 0
  sndparam = 4;
  if (ioctl(fd, SOUND_PCM_SUBDIVIDE, &sndparam) == -1) {
    perror("ioctl: SOUND_PCM_SUBDIVIDE");
  }
  if (sndparam != 4) {
    perror("ioctl: SOUND_PCM_SUBDIVIDE");
  }
#endif
  for (;;) {
    if (fmt) {
      i = read(fd, bp = b.b, sizeof(b.b));
      if (i < 0 && errno != EAGAIN) {
	perror("read");
	exit(4);
      }
      if (!i)
	break;
      if (i > 0) {
	for (; i >= sizeof(b.b[0]); i -= sizeof(b.b[0]), sp++)
	  fbuf[fbuf_cnt++] = ((int)(*bp)-0x80) * (1.0/128.0);
	if (i)
	  fprintf(stderr, "warning: noninteger number of samples read\n");
	if (fbuf_cnt > overlap) {
	  process_buffer(fbuf, fbuf_cnt-overlap);
	  memmove(fbuf, fbuf+fbuf_cnt-overlap, overlap*sizeof(fbuf[0]));
	  fbuf_cnt = overlap;
	}
      }
    } else {
      i = read(fd, sp = b.s, sizeof(b.s));
      if (i < 0 && errno != EAGAIN) {
	perror("read");
	exit(4);
      }
      if (!i)
	break;
      if (i > 0) {
	if (stereo) {
	  for (; i >= sizeof(b.s[0]); i -= (sizeof(b.s[0])*2), sp+=2)
	    fbuf[fbuf_cnt++] = (*sp) * (1.0/32768.0);
	} else {
	  for (; i >= sizeof(b.s[0]); i -= sizeof(b.s[0]), sp++)
	    fbuf[fbuf_cnt++] = (*sp) * (1.0/32768.0);
	}
	if (i)
	  fprintf(stderr, "warning: noninteger number of samples read\n");
	if (fbuf_cnt > overlap) {
	  process_buffer(fbuf, fbuf_cnt-overlap);
	  memmove(fbuf, fbuf+fbuf_cnt-overlap, overlap*sizeof(fbuf[0]));
	  fbuf_cnt = overlap;
	}
      }
    }
  }
  close(fd);
}

/* ---------------------------------------------------------------------- */

static void input_file(unsigned int sample_rate, unsigned int overlap,
		       const char *fname, const char *type)
{
  struct stat statbuf;
  int pipedes[2];
  int pid = 0, soxstat;
  int fd;
  int i;
  short buffer[8192];
  float fbuf[16384];
  unsigned int fbuf_cnt = 0;
  short *sp;

  /*
   * if the input type is not raw, sox is started to convert the
   * samples to the requested format
   */
  if (!type || !strcmp(type, "raw")) {
    if ((fd = open(fname, O_RDONLY)) < 0) {
      perror("open");
      exit(10);
    }
  } else {
    if (stat(fname, &statbuf)) {
      perror("stat");
      exit(10);
    }
    if (pipe(pipedes)) {
      perror("pipe");
      exit(10);
    }
    if (!(pid = fork())) {
      char srate[8];
      /*
       * child starts here... first set up filedescriptors,
       * then start sox...
       */
      sprintf(srate, "%d", sample_rate);
      close(pipedes[0]); /* close reading pipe end */
      close(1); /* close standard output */
      if (dup2(pipedes[1], 1) < 0)
	perror("dup2");
      close(pipedes[1]); /* close writing pipe end */
      execlp("sox", "sox",
	     "-t", type, fname,
	     "-t", "raw", "-s", "-w", "-r", srate, "-",
	     NULL);
      perror("execlp");
      exit(10);
    }
    if (pid < 0) {
      perror("fork");
      exit(10);
    }
    close(pipedes[1]); /* close writing pipe end */
    fd = pipedes[0];
  }
  /*
   * demodulate
   */
  for (;;) {
    i = read(fd, sp = buffer, sizeof(buffer));
    if (i < 0 && errno != EAGAIN) {
      perror("read");
      exit(4);
    }
    if (!i)
      break;
    if (i > 0) {
      for (; i >= sizeof(buffer[0]); i -= sizeof(buffer[0]), sp++)
	fbuf[fbuf_cnt++] = (*sp) * (1.0/32768.0);
      if (i)
	fprintf(stderr, "warning: noninteger number of samples read\n");
      if (fbuf_cnt > overlap) {
	process_buffer(fbuf, fbuf_cnt-overlap);
	memmove(fbuf, fbuf+fbuf_cnt-overlap, overlap*sizeof(fbuf[0]));
	fbuf_cnt = overlap;
      }
    }
  }
  close(fd);
  waitpid(pid, &soxstat, 0);
}

/* ---------------------------------------------------------------------- */

static const char usage_str[] = "multimod\n"
  "Demodulates many different radio transmission formats\n"
  "(C) 1996 by Thomas Sailer HB9JNX/AE4WA\n"
  "  -t <type>  : input file type (any other type than raw requires sox)\n"
  "  -a <demod> : add demodulator\n"
  "  -p <fifo> : output\n"
  "  -s <demod> : subtract demodulator\n";

int main(int argc, char *argv[])
{
  int c;
  int errflg = 0;
  int i;
  char **itype;
  int mask_first = 1;
  int sample_rate = -1;
  unsigned int overlap = 0;
  char *input_type = "hw";

  fprintf(stdout, "multimod  (C) 1996/1997 by Tom Sailer HB9JNX/AE4WA\n"
	  "available demodulators:");
  for (i = 0; i < NUMDEMOD; i++)
    fprintf(stdout, " %s", dem[i]->name);
  fprintf(stdout, "\n");
  while ((c = getopt(argc, argv, "t:a:p:s:v:")) != EOF) {
    switch (c) {
    case '?':
      errflg++;
      break;

    case 'v':
      verbose_level = strtoul(optarg, 0, 0);
      break;

    case 't':
      for (itype = (char **)allowed_types; *itype; itype++)
	if (!strcmp(*itype, optarg)) {
	  input_type = *itype;
	  goto intypefound;
	}
      fprintf(stderr, "invalid input type \"%s\"\n"
	      "allowed types: ", optarg);
      for (itype = (char **)allowed_types; *itype; itype++)
	fprintf(stderr, "%s ", *itype);
      fprintf(stderr, "\n");
      errflg++;
    intypefound:
      break;

    case 'a':
      if (mask_first)
	memset(dem_mask, 0, sizeof(dem_mask));
      mask_first = 0;
      for (i = 0; i < NUMDEMOD; i++)
	if (!strcasecmp(optarg, dem[i]->name)) {
	  MASK_SET(i);
	  break;
	}
      if (i >= NUMDEMOD) {
	fprintf(stderr, "invalid mode \"%s\"\n", optarg);
	errflg++;
      }
      break;

    case 'p':
      printf("pipe=%s\n", optarg);
      strcpy(multimon_pipe_name, optarg);
      break;

    case 's':
      if (mask_first)
	memset(dem_mask, 0xff, sizeof(dem_mask));
      mask_first = 0;
      for (i = 0; i < NUMDEMOD; i++)
	if (!strcasecmp(optarg, dem[i]->name)) {
	  MASK_RESET(i);
	  break;
	}
      if (i >= NUMDEMOD) {
	fprintf(stderr, "invalid mode \"%s\"\n", optarg);
	errflg++;
      }
      break;

    }
  }
  if (errflg) {
    (void)fprintf(stderr, usage_str);
    exit(2);
  }
  if (mask_first)
    memset(dem_mask, 0xff, sizeof(dem_mask));

  fprintf(stdout, "Enabled demodulators:");
  for (i = 0; i < NUMDEMOD; i++)
    if (MASK_ISSET(i)) {
      fprintf(stdout, " %s", dem[i]->name);
      memset(dem_st+i, 0, sizeof(dem_st[i]));
      dem_st[i].dem_par = dem[i];
      if (dem[i]->init)
	dem[i]->init(dem_st+i);
      if (sample_rate == -1)
	sample_rate = dem[i]->samplerate;
      else if (sample_rate != dem[i]->samplerate) {
	fprintf(stdout, "\n");
	fprintf(stderr, "Error: Current sampling rate %d, "
		" demodulator \"%s\" requires %d\n",
		sample_rate, dem[i]->name, dem[i]->samplerate);
	exit(3);
      }
      if (dem[i]->overlap > overlap)
	overlap = dem[i]->overlap;
    }
  fprintf(stdout, "\n");

  if (!strcmp(input_type, "hw")) {
    if ((argc - optind) >= 1)
      input_sound(sample_rate, overlap, argv[optind]);
    else
      input_sound(sample_rate, overlap, NULL);
    exit(0);
  }
  if ((argc - optind) < 1) {
    (void)fprintf(stderr, "no source files specified\n");
    exit(4);
  }
  for (i = optind; i < argc; i++)
    input_file(sample_rate, overlap, argv[i], input_type);
  exit(0);
}

/* ---------------------------------------------------------------------- */
