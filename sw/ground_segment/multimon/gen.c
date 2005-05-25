/*
 *      gen.c -- generate different test signals
 *
 *      Copyright (C) 1997
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

#include "gen.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>

#ifdef SUN_AUDIO
#include <sys/audioio.h>
#include <stropts.h>
#include <sys/conf.h>
#else /* SUN_AUDIO */
#include <sys/soundcard.h>
#include <sys/ioctl.h>
#endif /* SUN_AUDIO */

/* ---------------------------------------------------------------------- */

#ifdef __sun__
#include <stdarg.h>

int snprintf(char *buf, size_t sz, const char *fmt, ...)
{
	int i;

	va_list arg;
	va_start(arg, fmt);
	i = vsprintf(buf, fmt, arg);
	va_end(arg);
	return i;
}

#endif /* __sun__ */

/* ---------------------------------------------------------------------- */

static const char *allowed_types[] = {
	"raw", "aiff", "au", "hcom", "sf", "voc", "cdr", "dat", 
	"smp", "wav", "maud", "vwe", NULL
};

/* ---------------------------------------------------------------------- */

typedef void (*t_init_procs)(struct gen_params *, struct gen_state *);
typedef int (*t_gen_procs)(signed short *, int, struct gen_params *, struct gen_state *);

static const t_init_procs init_procs[] = {
	gen_init_dtmf, gen_init_sine, gen_init_zvei, gen_init_hdlc
};

static const t_gen_procs gen_procs[] = {
	gen_dtmf, gen_sine, gen_zvei, gen_hdlc
};

/* ---------------------------------------------------------------------- */

#define MAX_GEN 16
static struct gen_params params[MAX_GEN];
static struct gen_state state[MAX_GEN];
static int num_gen = 0;

/* ---------------------------------------------------------------------- */

static int process_buffer(short *buf, int len)
{
	int i;
	int totnum = 0, num;

	memset(buf, 0, len*sizeof(buf[0]));
	for (i = 0; i < num_gen; i++) {
		if (params[i].type >= sizeof(gen_procs)/sizeof(gen_procs[0]))
			break;
		if (!gen_procs[params[i].type])
			break;
		num = gen_procs[params[i].type](buf, len, params+i, state+i);
		if (num > totnum)
			totnum = num;
	}
	return totnum;
}

/* ---------------------------------------------------------------------- */
#ifdef SUN_AUDIO

static void output_sound(unsigned int sample_rate, const char *ifname)
{
        audio_info_t audioinfo;
        audio_info_t audioinfo2;
	audio_device_t audiodev;
	int fd;
	short buffer[8192];
	short *sp;
	int i, num, num2;

	if ((fd = open(ifname ? ifname : "/dev/audio", O_WRONLY)) < 0) {
		perror("open");
		exit (10);
	}
        if (ioctl(fd, AUDIO_GETDEV, &audiodev) == -1) {
		perror("ioctl: AUDIO_GETDEV");
		exit (10);
        }
        AUDIO_INITINFO(&audioinfo);
        audioinfo.record.sample_rate = sample_rate;
        audioinfo.record.channels = 1;
        audioinfo.record.precision = 16;
        audioinfo.record.encoding = AUDIO_ENCODING_LINEAR;
        /*audioinfo.record.gain = 0x20;
	  audioinfo.record.port = AUDIO_LINE_IN;
	  audioinfo.monitor_gain = 0;*/
        if (ioctl(fd, AUDIO_SETINFO, &audioinfo) == -1) {
		perror("ioctl: AUDIO_SETINFO");
		exit (10);
        }     
        if (ioctl(fd, I_FLUSH, FLUSHW) == -1) {
		perror("ioctl: I_FLUSH");
		exit (10);
        }
        if (ioctl(fd, AUDIO_GETINFO, &audioinfo2) == -1) {
		perror("ioctl: AUDIO_GETINFO");
		exit (10);
        }
        fprintf(stdout, "Audio device: name %s, ver %s, config %s, "
		"sampling rate %d\n", audiodev.name, audiodev.version,
		audiodev.config, audioinfo.record.sample_rate);
	do {
		num2 = num = process_buffer(sp = buffer, sizeof(buffer)/sizeof(buffer[0]));
		while (num > 0) {
			i = write(fd, sp, num*sizeof(sp[0]));
			if (i < 0 && errno != EAGAIN) {
				perror("write");
				exit(4);
			}
			if (i > 0) {
				if (i % sizeof(sp[0]))
					fprintf(stderr, "gen: warning: write wrote noninteger number of samples\n");
				num -= i / sizeof(sp[0]);
			}
		}
	} while (num2 > 0);
	close(fd);
}

#else /* SUN_AUDIO */
/* ---------------------------------------------------------------------- */

static void output_sound(unsigned int sample_rate, const char *ifname)
{
        int sndparam;
	int fd;
	union {
		short s[8192];
		unsigned char b[8192];
	} b;
	int i;
	short *sp;
	unsigned char *bp;
	int fmt = 0, num, num2;

	if ((fd = open(ifname ? ifname : "/dev/dsp", O_WRONLY)) < 0) {
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
        if (sndparam != 0) {
                fprintf(stderr, "gen: Error, cannot set the channel "
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
	do {
		num2 = num = process_buffer(b.s, sizeof(b.s)/sizeof(b.s[0]));
		if (fmt) {
			for (bp = b.b, sp = b.s, i = sizeof(b.s)/sizeof(b.s[0]); i > 0; i--, bp++, sp++)
				*bp = 0x80 + (*sp >> 8);
			bp = b.b;
			while (num > 0) {
				i = write(fd, bp, num*sizeof(bp[0]));
				if (i < 0 && errno != EAGAIN) {
					perror("write");
					exit(4);
				}
				if (i > 0) {
					if (i % sizeof(bp[0]))
						fprintf(stderr, "gen: warning: write wrote noninteger number of samples\n");
					num -= i / sizeof(bp[0]);
				}
			}
		} else {
			sp = b.s;
			while (num > 0) {
				i = write(fd, sp, num*sizeof(sp[0]));
				if (i < 0 && errno != EAGAIN) {
					perror("write");
					exit(4);
				}
				if (i > 0) {
					if (i % sizeof(sp[0]))
						fprintf(stderr, "gen: warning: write wrote noninteger number of samples\n");
					num -= i / sizeof(sp[0]);
				}
			}
		}
	} while (num2 > 0);
	close(fd);
}
#endif /* SUN_AUDIO */

/* ---------------------------------------------------------------------- */

static void output_file(unsigned int sample_rate, const char *fname, const char *type)
{
	struct stat statbuf;
	int pipedes[2];
	int pid = 0, soxstat;
	int fd;
	int i, num, num2;
	short buffer[8192];	
	short *sp;

	/*
	 * if the input type is not raw, sox is started to convert the
	 * samples to the requested format
	 */
	if (!type || !strcmp(type, "raw")) {
		if ((fd = open(fname, O_WRONLY|O_CREAT|O_EXCL, 0777)) < 0) {
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
			snprintf(srate, sizeof(srate), "%d", sample_rate);
			close(pipedes[1]); /* close writing pipe end */
			close(0); /* close standard input */
			if (dup2(pipedes[0], 0) < 0) 
				perror("dup2");
			close(pipedes[0]); /* close reading pipe end */
			execlp("sox", "sox", 
			       "-t", "raw",  "-s", "-w", "-r", srate, "-",
			       "-t", type, fname,
			       NULL);
			perror("execlp");
			exit(10);
		}
		if (pid < 0) {
			perror("fork");
			exit(10);
		}
		close(pipedes[0]); /* close reading pipe end */
		fd = pipedes[1];
	}
	/*
	 * modulate
	 */
	do {
		num2 = num = process_buffer(sp = buffer, sizeof(buffer)/sizeof(buffer[0]));
		while (num > 0) {
			i = write(fd, sp, num*sizeof(sp[0]));
			if (i < 0 && errno != EAGAIN) {
				perror("write");
				exit(4);
			}
			if (i > 0) {
				if (i % sizeof(sp[0]))
					fprintf(stderr, "gen: warning: write wrote noninteger number of samples\n");
				num -= i / sizeof(sp[0]);
			}
		}
	} while (num2 > 0);
	close(fd);
	waitpid(pid, &soxstat, 0);
}

/* ---------------------------------------------------------------------- */

static const char usage_str[] = "gen\n"
"Generates test signals\n"
"(C) 1997 by Thomas Sailer HB9JNX/AE4WA\n"
"  -t <type>  : input file type (any other type than raw requires sox)\n"
"  -a <ampl>  : amplitude\n"
"  -d <str>   : encode DTMF string\n"
"  -z <str>   : encode ZVEI string\n"
"  -s <freq>  : encode sine\n"
"  -p <text>  : encode hdlc packet\n";

void main(int argc, char *argv[])
{
	int c;
	int errflg = 0;
	char **otype;
	char *output_type = "hw";
	char *cp;

	fprintf(stdout, "gen - (C) 1997 by Tom Sailer HB9JNX/AE4WA\n");	
	while ((c = getopt(argc, argv, "t:a:d:s:z:p:")) != EOF) {
		switch (c) {
		case '?':
			errflg++;
			break;
			
		case 't':
			for (otype = (char **)allowed_types; *otype; otype++) 
				if (!strcmp(*otype, optarg)) {
					output_type = *otype;
					goto outtypefound;
				}
			fprintf(stderr, "invalid output type \"%s\"\n"
				"allowed types: ", optarg);
			for (otype = (char **)allowed_types; *otype; otype++) 
				fprintf(stderr, "%s ", *otype);
			fprintf(stderr, "\n");
			errflg++;
		outtypefound:
			break;

		case 'a':
			if (num_gen <= 0) {
				fprintf(stderr, "gen: no generator selected\n");
				errflg++;
			}
 			if (!(cp = strstr(optarg, "dB")))
				cp = strstr(optarg, "db");
			if (cp) {
				*cp = '\0';
				params[num_gen-1].ampl = 16384.0 * pow(10.0, strtod(optarg, NULL) / 20.0);
			} else {
				params[num_gen-1].ampl = 16384.0 * strtod(optarg, NULL);
			}
			break;

		case 'd':
			num_gen++;
			if (num_gen > MAX_GEN) {
				fprintf(stderr, "too many generators\n");
				errflg++;
				break;
			}
			params[num_gen-1].type = gentype_dtmf;
			params[num_gen-1].ampl = 16384;
			params[num_gen-1].p.dtmf.duration = MS(100);
			params[num_gen-1].p.dtmf.pause = MS(100);
			strncpy(params[num_gen-1].p.dtmf.str, optarg, sizeof(params[num_gen-1].p.dtmf.str));
			break;

		case 's':
			num_gen++;
			if (num_gen > MAX_GEN) {
				fprintf(stderr, "too many generators\n");
				errflg++;
				break;
			}
			params[num_gen-1].type = gentype_sine;
			params[num_gen-1].ampl = 16384;
			params[num_gen-1].p.sine.duration = MS(1000);
			params[num_gen-1].p.sine.freq = strtoul(optarg, NULL, 0);
			break;

		case 'z':
			num_gen++;
			if (num_gen > MAX_GEN) {
				fprintf(stderr, "too many generators\n");
				errflg++;
				break;
			}
			params[num_gen-1].type = gentype_zvei;
			params[num_gen-1].ampl = 16384;
			params[num_gen-1].p.zvei.duration = MS(50);
			params[num_gen-1].p.zvei.pause = MS(50);
			strncpy(params[num_gen-1].p.zvei.str, optarg, sizeof(params[num_gen-1].p.dtmf.str));
			break;

		case 'p':
			num_gen++;
			if (num_gen > MAX_GEN) {
				fprintf(stderr, "too many generators\n");
				errflg++;
				break;
			}
			params[num_gen-1].type = gentype_hdlc;
			params[num_gen-1].ampl = 16384;
			params[num_gen-1].p.hdlc.modulation = 0;
			params[num_gen-1].p.hdlc.txdelay = 100;
			params[num_gen-1].p.hdlc.pkt[0] = ('H') << 1;
			params[num_gen-1].p.hdlc.pkt[1] = ('B') << 1;
			params[num_gen-1].p.hdlc.pkt[2] = ('9') << 1;
			params[num_gen-1].p.hdlc.pkt[3] = ('J') << 1;
			params[num_gen-1].p.hdlc.pkt[4] = ('N') << 1;
			params[num_gen-1].p.hdlc.pkt[5] = ('X') << 1;
			params[num_gen-1].p.hdlc.pkt[6] = (0x00) << 1;
			params[num_gen-1].p.hdlc.pkt[7] = ('A') << 1;
			params[num_gen-1].p.hdlc.pkt[8] = ('E') << 1;
			params[num_gen-1].p.hdlc.pkt[9] = ('4') << 1;
			params[num_gen-1].p.hdlc.pkt[10] = ('W') << 1;
			params[num_gen-1].p.hdlc.pkt[11] = ('A') << 1;
			params[num_gen-1].p.hdlc.pkt[12] = (' ') << 1;
			params[num_gen-1].p.hdlc.pkt[13] = ((0x00) << 1) | 1;
			params[num_gen-1].p.hdlc.pkt[14] = 0x03;
			params[num_gen-1].p.hdlc.pkt[15] = 0xf0;
			strncpy(params[num_gen-1].p.hdlc.pkt+16, optarg, 
				sizeof(params[num_gen-1].p.hdlc.pkt)-16);
			params[num_gen-1].p.hdlc.pktlen = 16 + 
				strlen(params[num_gen-1].p.hdlc.pkt+16);
		}
	}
		
	if (errflg || num_gen <= 0) {
		(void)fprintf(stderr, usage_str);
		exit(2);
	}

	memset(state, 0, sizeof(state));
	for (c = 0; c < num_gen; c++) {
		if (params[c].type >= sizeof(init_procs)/sizeof(init_procs[0]))
			break;
		if (!init_procs[params[c].type])
			break;
		init_procs[params[c].type](params+c, state+c);
	}

	if (!strcmp(output_type, "hw")) {
		if ((argc - optind) >= 1)
			output_sound(SAMPLE_RATE, argv[optind]);
		else 
			output_sound(SAMPLE_RATE, NULL);
		exit(0);
	}
	if ((argc - optind) < 1) {
		(void)fprintf(stderr, "no destination file specified\n");
		exit(4);
	}
	output_file(SAMPLE_RATE, argv[optind], output_type);
	exit(0);
}



