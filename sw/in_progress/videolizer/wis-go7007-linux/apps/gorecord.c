/*
 * Copyright (C) 2005-2006 Micronas USA Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and the associated README documentation file (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <linux/videodev.h>
#include <linux/soundcard.h>
#include <errno.h>

#include "../include/go7007.h"

/* Note: little-endian */
#define PUT_16(p,v) ((p)[0]=(v)&0xff,(p)[1]=((v)>>8)&0xff)
#define PUT_32(p,v) ((p)[0]=(v)&0xff,(p)[1]=((v)>>8)&0xff,(p)[2]=((v)>>16)&0xff,(p)[3]=((v)>>24)&0xff)
#define FOURCC(c) (((c)[3]<<24)|((c)[2]<<16)|((c)[1]<<8)|(c)[0])

#define MAX_FILE_SIZE	(1000*1024*1024) /* Keep files just under 1GB */

#define MAX_BUFFERS	32
#define AUDIO_BUF_LEN	(256*1024)

/* Defined in tv-freq.c */
int chan_to_freq(char *name);

char *vdevice = NULL, *adevice = NULL, *avibase = NULL;
char avifile[PATH_MAX];
int vidfd = -1, audfd = -1, avifd = -1, idxfd = -1;
int buf_count = 0;
int avi_frame_count = 0;
int total_frames = 0;
int duration = -1;
unsigned int sequence;
int frame_limit = -1;
int max_frame_length = 0;
unsigned int vbytes = 0, abytes = 0;
unsigned char audio_buffer[AUDIO_BUF_LEN];
int audio_len = 0;
int nowrite = 0;
int probe = 0;
int verbose = 0;
int bitrate = 1500000;
int input = 0;
int width = 0, height = 0;
v4l2_std_id std = 0;
int fpsdrop = 1;
struct v4l2_fract frameperiod;
int audmode = V4L2_TUNER_MODE_STEREO;
int tv_freq = -1;
enum {
	FMT_MJPEG,
	FMT_MPEG1,
	FMT_MPEG2,
	FMT_MPEG4,
} format = FMT_MPEG4;
int dvd = 0;
int control_value_count = 0;
struct v4l2_control control_values[32];
int interrupted = 0;
unsigned char *buffers[MAX_BUFFERS];

void usage(char *progname)
{
	printf(
"`gorecord` captures video and audio from a WIS GO7007 video encoder and\n"
"writes it to an AVI file.  Capturing continues until the specified frame\n"
"limit has been reached or the maximum AVI file size is exceeded.\n\n"
"Usage: gorecord [OPTION]... -frames <n> [<AVI file name>]\n\n"
"Examples:\n"
"  gorecord -duration 60 -format mjpeg capture.avi  # Captures MJPEG video\n"
"                                                   # and PCM audio to\n"
"                                                   # the file capture.avi\n"
"  gorecord -frames 3000 -bitrate 800 -nowrite      # Captures MPEG4 video\n"
"                                                   # and PCM audio at\n"
"                                                   # 800 kbps but does not\n"
"                                                   # write it to disk\n\n"
"Control options:\n"
"  -verbose                      Verbosely describe all operations\n"
"  -duration <n>                 Stop capturing after <n> seconds\n"
"  -frames <n>                   Stop capturing after <n> video frames\n"
"  -noaudio                      Do not capture audio; only video\n"
"  -nowrite                      Do not write captured video/audio to a file\n"
"  -vdevice <V4L2 device path>   Explicitly specify the V4L2 device to use\n"
"  -adevice <OSS device path>    Explicitly specify the OSS device to use\n"
"Input video options:\n"
"  -input <n>                    Capture from input port <n> (default: 0)\n"
"  -mode <mode>                  Select mode for input video (default: ntsc)\n"
"                                  Baseband modes: ntsc, pal, secam\n"
"                                  TV modes: pal-bg, pal-i, pal-dk, secam-l,\n"
"                                            ntsc, ntsc-j\n"
"  -tvchan <band>:<chan>         Tune to TV channel <chan> in band <band>\n"
"                                  Bands: ntsc-bcast, ntsc-cable, ntsc-hrc,\n"
"                                         ntsc-bcast-jp, ntsc-cable-jp,\n"
"                                         europe, france, russia\n"
"  -tvaudio <mode>               Select mode for TV audio (default: stereo)\n"
"                                  Modes: mono, stereo, lang1, lang2\n"
"Sensor controls:  (defaults listed in probe information)\n"
"  -brightness <n>               Specify video brightness adjustment\n"
"  -contrast <n>                 Specify video contrast adjustment\n"
"  -saturation <n>               Specify video saturation adjustment\n"
"  -hue <n>                      Specify video hue adjustment\n"
"Output stream options:\n"
"  -width <n>                    Specify encoded image width (default: 640)\n"
"  -height <n>                   Specify encoded image height (default: 480)\n"
"  -fpsdrop <n>                  Specify encoded frame rate downscale ratio\n"
"  -bitrate <n>                  Specify a video bitrate (default: 1500 kbps)\n"
"  -format <format>              Encode video in <format> (default: mpeg4)\n"
"                                  Formats: mpeg1, mpeg2, mpeg2-dvd,\n"
"                                           mpeg4, mjpeg\n");
	exit(1);
}

void add_control_value(__u32 id, __s32 value)
{
	control_values[control_value_count].id = id;
	control_values[control_value_count].value = value;
	++control_value_count;
}

void find_devices(int noaudio)
{
	struct stat si;
	struct dirent *ent;
	int i, minor = -1;
	DIR *dir;
	FILE *file;
	char line[128], sympath[PATH_MAX], canonpath[PATH_MAX], gopath[PATH_MAX];
	static char vdev[PATH_MAX], adev[PATH_MAX];

	/* Make sure sysfs is mounted and the driver is loaded */
	if (stat("/sys/bus/usb/drivers", &si) < 0) {
		fprintf(stderr, "Unable to read /sys/bus/usb/drivers: %s\n",
				strerror(errno));
		fprintf(stderr, "Is sysfs mounted on /sys?\n");
		exit(1);
	}
	if (stat("/sys/bus/usb/drivers/go7007", &si) < 0) {
		fprintf(stderr, "Unable to read /sys/bus/usb/drivers/go7007: "
				"%s\n", strerror(errno));
		fprintf(stderr, "Is the go7007-usb kernel module loaded?\n");
		exit(1);
	}

	/* Find a Video4Linux device associated with the go7007 driver */
	for (i = 0; i < 20; ++i) {
		sprintf(sympath,
			"/sys/class/video4linux/video%d/device/driver", i);
		if (realpath(sympath, canonpath) == NULL)
			continue;
		if (!strcmp(strrchr(canonpath, '/') + 1, "go7007"))
			break;
	}
	sprintf(sympath, "/sys/class/video4linux/video%d/device", i);
	if (i == 20 || realpath(sympath, gopath) == NULL) {
		fprintf(stderr,
			"Driver loaded but no GO7007 devices found.\n");
		fprintf(stderr, "Is the device connected properly?\n");
		exit(1);
	}
	sprintf(vdev, "/dev/video%d", i);
	vdevice = vdev;
	fprintf(stderr, "%s is a GO7007 device at USB address %s\n",
			vdev, strrchr(gopath, '/') + 1);

	if (noaudio)
		return;

	/* Find the ALSA device associated with this USB address */
	fprintf(stderr, "Attempting to determine audio device...");
	for (i = 0; i < 20; ++i) {
		sprintf(sympath, "/sys/class/sound/pcmC%dD0c/device", i);
		if (realpath(sympath, canonpath) == NULL)
			continue;
		if (!strcmp(gopath, canonpath))
			break;
	}
	if (i == 20) {
		fprintf(stderr,
			"\nUnable to find associated ALSA device node\n");
		exit(1);
	}

	/* Find the OSS emulation minor number for this ALSA device */
	file = fopen("/proc/asound/oss/devices", "r");
	if (file == NULL) {
		fprintf(stderr,
			"\nUnable to open /proc/asound/oss/devices: %s\n",
			strerror(errno));
		fprintf(stderr, "Is the snd_pcm_oss module loaded?\n");
		exit(1);
	}
	while ((fgets(line, sizeof(line), file)) != NULL) {
		unsigned int n;
		int m;
		char *c;

		if ((c = strrchr(line, ':')) == NULL ||
				strcmp(c, ": digital audio\n") ||
				sscanf(line, "%d: [%u-%*u]:", &m, &n) != 2)
			continue;
		if (n == i) {
			minor = m;
			break;
		}
	}
	fclose(file);
	if (minor < 0) {
		fprintf(stderr, "\nUnable to find emulated OSS device node\n");
		exit(1);
	}
	dir = opendir("/sys/class/sound");
	if (dir == NULL) {
		fprintf(stderr, "\nUnable to read /sys/class/sound: %s\n",
				strerror(errno));
		exit(1);
	}
	while ((ent = readdir(dir)) != NULL) {
		int m = -1;

		if (strncmp(ent->d_name, "dsp", 3))
			continue;
		sprintf(adev, "/sys/class/sound/%s/dev", ent->d_name);
		file = fopen(adev, "r");
		if (file == NULL)
			continue;
		if ((fgets(line, sizeof(line), file)) != NULL)
			sscanf(line, "%*d:%d\n", &m);
		fclose(file);
		if (m == minor)
			break;
	}
	if (ent == NULL) {
		fprintf(stderr, "\nUnable to find emulated OSS device.\n");
		exit(1);
	}
	sprintf(adev, "/dev/%s", ent->d_name);
	closedir(dir);
	adevice = adev;
	fprintf(stderr, "using audio device %s\n", adev);
}

void parse_opts(int argc, char **argv)
{
	int i, noaudio = 0;

	for (i = 1; i < argc && argv[i][0] == '-'; ++i) {
		if (!strcmp(argv[i], "-help"))
			usage(argv[0]);
		else if (!strcmp(argv[i], "-nosound")) {
			noaudio = 1;
		} else if (!strcmp(argv[i], "-noaudio")) {
			noaudio = 1;
		} else if (!strcmp(argv[i], "-verbose")) {
			verbose = 1;
		} else if (!strcmp(argv[i], "-nowrite")) {
			nowrite = 1;
		} else if (i + 1 >= argc) {
			usage(argv[0]);
	/* options that take an argument go below here */
		} else if (!strcmp(argv[i], "-vdevice")) {
			vdevice = argv[++i];
		} else if (!strcmp(argv[i], "-adevice")) {
			adevice = argv[++i];
		} else if (!strcmp(argv[i], "-frames")) {
			frame_limit = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-duration")) {
			duration = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-input")) {
			input = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-width")) {
			width = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-height")) {
			height = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-fpsdrop")) {
			fpsdrop = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-bitrate")) {
			bitrate = atoi(argv[++i]);
			if (bitrate < 64000)
				bitrate *= 1000;
			if (bitrate < 64000 || bitrate > 10000000) {
				fprintf(stderr, "Bitrate must be between "
						"64 kbps and 10000 kbps\n");
				exit(1);
			}
		} else if (!strcmp(argv[i], "-format")) {
			++i;
			dvd = 0;
			if (!strcasecmp(argv[i], "mjpeg"))
				format = FMT_MJPEG;
			else if (!strcasecmp(argv[i], "mpeg1"))
				format = FMT_MPEG1;
			else if (!strcasecmp(argv[i], "mpeg2"))
				format = FMT_MPEG2;
			else if (!strcasecmp(argv[i], "mpeg4"))
				format = FMT_MPEG4;
			else if (!strcasecmp(argv[i], "mpeg2-dvd")) {
				format = FMT_MPEG2;
				dvd = 1;
			} else {
				fprintf(stderr, "Only formats \"mjpeg\", "
						"\"mpeg1\", \"mpeg2\", "
						"\"mpeg2-dvd\", and "
						"\"mpeg4\" are supported.\n");
				exit(1);
			}
		} else if (!strcmp(argv[i], "-mode")) {
			++i;
			if (!strcasecmp(argv[i], "ntsc"))
				std = V4L2_STD_NTSC_M;
			else if (!strcasecmp(argv[i], "pal"))
				std = V4L2_STD_PAL;
			else if (!strcasecmp(argv[i], "secam"))
				std = V4L2_STD_SECAM;
			else if (!strcasecmp(argv[i], "pal-bg"))
				std = V4L2_STD_PAL_BG;
			else if (!strcasecmp(argv[i], "pal-i"))
				std = V4L2_STD_PAL_I;
			else if (!strcasecmp(argv[i], "pal-dk"))
				std = V4L2_STD_PAL_DK;
			else if (!strcasecmp(argv[i], "secam-l"))
				std = V4L2_STD_SECAM_L;
			else if (!strcasecmp(argv[i], "ntsc-j"))
				std = V4L2_STD_NTSC_M_JP;
			else {
				fprintf(stderr, "Valid modes: ntsc, pal, "
						"secam, pal-bg, pal-i, pal-dk, "
						"secam-l, ntsc-j\n");
				exit(1);
			}
		} else if (!strcmp(argv[i], "-tvchan")) {
			tv_freq = chan_to_freq(argv[++i]);
			if (tv_freq < 0)
				exit(1);
		} else if (!strcmp(argv[i], "-tvaudio")) {
			++i;
			if (!strcasecmp(argv[i], "mono"))
				audmode = V4L2_TUNER_MODE_MONO;
			else if (!strcasecmp(argv[i], "stereo"))
				audmode = V4L2_TUNER_MODE_STEREO;
			else if (!strcasecmp(argv[i], "lang1"))
				audmode = V4L2_TUNER_MODE_LANG1;
			else if (!strcasecmp(argv[i], "lang2"))
				audmode = V4L2_TUNER_MODE_LANG2;
			else {
				fprintf(stderr, "Valid TV audio modes: mono, "
						"stereo, lang1, lang2\n");
				exit(1);
			}
		} else if (!strcmp(argv[i], "-brightness")) {
			add_control_value(V4L2_CID_BRIGHTNESS, atoi(argv[++i]));
		} else if (!strcmp(argv[i], "-contrast")) {
			add_control_value(V4L2_CID_CONTRAST, atoi(argv[++i]));
		} else if (!strcmp(argv[i], "-saturation")) {
			add_control_value(V4L2_CID_SATURATION, atoi(argv[++i]));
		} else if (!strcmp(argv[i], "-hue")) {
			add_control_value(V4L2_CID_HUE, atoi(argv[++i]));
		} else {
			usage(argv[0]);
		}
	}
	if (nowrite) {
		if (i != argc)
			usage(argv[0]);
	} else {
		if (i == argc) {
			nowrite = 1;
			verbose = 1;
			probe = 1;
			avibase = NULL;
		} else if (i + 1 != argc)
			usage(argv[0]);
		else
			avibase = argv[i];
	}
	if (width == 0)
		width = 640;
	if (height == 0)
		height = 480;
	if (width < 144 || width > 720) {
		fprintf(stderr, "Width must be between 144 and 720 pixels.\n");
		exit(1);
	}
	if (height < 96 || height > 576) {
		fprintf(stderr, "Height must be between 96 and 576 pixels.\n");
		exit(1);
	}
	if ((width & 0x0f) != 0 || (height & 0x0f) != 0) {
		fprintf(stderr, "Height and width must be divisible by 16.\n");
		exit(1);
	}
	if (fpsdrop < 1) {
		fprintf(stderr, "Frame rate scaler must be positive.\n");
		exit(1);
	}
	if (noaudio)
		adevice = NULL;
	if (vdevice != NULL) {
		if (adevice == NULL && !noaudio) {
			fprintf(stderr, "If -vdevice is used, please also "
				"specify an audio device with -adevice or "
				"use -noaudio.\n");
			exit(1);
		}
	} else {
		if (adevice != NULL) {
			fprintf(stderr, "If -adevice is used, please also "
				"specify a video device with -vdevice.\n");
			exit(1);
		}
		find_devices(noaudio);
	}
}

void open_avifile(void)
{
	/* First open the temporary file we'll store the index in */
	idxfd = open(avifile, O_RDWR | O_CREAT | O_TRUNC, 0666);
	if (idxfd < 0) {
		fprintf(stderr, "Unable to open %s: %s\n", avifile,
				strerror(errno));
		exit(1);
	}
	unlink(avifile);
	/* Then open the real AVI destination file */
	avifd = open(avifile, O_RDWR | O_CREAT | O_TRUNC, 0666);
	if (avifd < 0) {
		fprintf(stderr, "Unable to open %s: %s\n", avifile,
				strerror(errno));
		exit(1);
	}
	lseek(avifd, 1024 + 12, SEEK_SET);
}

void open_devices(void)
{
	vidfd = open(vdevice, O_RDWR);
	if (vidfd < 0) {
		fprintf(stderr, "Unable to open %s: %s\n", vdevice,
				strerror(errno));
		exit(1);
	}
	if (adevice == NULL)
		return;
	audfd = open(adevice, O_RDONLY);
	if (audfd < 0) {
		fprintf(stderr, "Unable to open %s: %s\n", adevice,
				strerror(errno));
		exit(1);
	}
	fcntl(audfd, F_SETFL, O_NONBLOCK);
}

void alsa_init(void)
{
	int arg;

	arg = AFMT_S16_LE;
	if (ioctl(audfd, SNDCTL_DSP_SETFMT, &arg) < 0) {
		perror("SNDCTL_DSP_SETFMT");
		exit(1);
	}
	arg = 48000;
	if (ioctl(audfd, SNDCTL_DSP_SPEED, &arg) < 0) {
		perror("SNDCTL_DSP_SPEED");
		exit(1);
	}
	arg = 1;
	if (ioctl(audfd, SNDCTL_DSP_STEREO, &arg) < 0) {
		perror("SNDCTL_DSP_STEREO");
		exit(1);
	}
}

void v4l2_init(void)
{
	struct v4l2_capability cap;
	struct v4l2_fmtdesc fmtdesc;
	struct v4l2_streamparm parm;
	struct v4l2_queryctrl ctrl;
	struct v4l2_format fmt;
	struct go7007_comp_params comp;
	struct go7007_mpeg_params mpeg;
	struct v4l2_requestbuffers req;
	struct v4l2_buffer buf;
	struct v4l2_input inp;
	struct v4l2_standard s;
	struct v4l2_tuner tuner;
	struct v4l2_frequency freq;
	__u32 i, j;

	/* First query the capabilities of the video capture device */
	if (ioctl(vidfd, VIDIOC_QUERYCAP, &cap) < 0) {
		perror("VIDIOC_QUERYCAP");
		exit(1);
	}
	if (verbose) {
		printf("\nDriver:   %s\n", cap.driver);
		printf("Card:     %s\n", cap.card);
		printf("Version:  %u.%u.%u\n\n", (cap.version >> 16) & 0xff,
				(cap.version >> 8) & 0xff, cap.version & 0xff);

		printf("Formats supported:");
		for (i = 0; ; ++i) {
			memset(&fmtdesc, 0, sizeof(fmtdesc));
			fmtdesc.index = i;
			fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (ioctl(vidfd, VIDIOC_ENUM_FMT, &fmtdesc) < 0)
				break;
			if (i > 0)
				printf(",");
			printf(" %s", fmtdesc.description);
		}
		printf("\n\n");

		printf("Input ports available:\n");
		for (i = 0; ; ++i) {
			memset(&inp, 0, sizeof(inp));
			inp.index = i;
			if (ioctl(vidfd, VIDIOC_ENUMINPUT, &inp) < 0)
				break;
			if (ioctl(vidfd, VIDIOC_S_INPUT, &i) < 0) {
				fprintf(stderr, "Unable to set input "
						"port %d\n", input);
				exit(1);
			}
			printf("   input %2d: %s (", i, inp.name);
			for (j = 0; ; ++j) {
				memset(&s, 0, sizeof(s));
				s.index = j;
				if (ioctl(vidfd, VIDIOC_ENUMSTD,
							&s) < 0)
					break;
				if (j > 0)
					printf(", ");
				printf("%s", s.name);
			}
			printf(")\n");
		}
		printf("\n");

		printf("Sensor controls available:\n");
	}

	/* Add default values for each sensor control, which will be
	 * overridden if the user has specified their own values */
	for (i = V4L2_CID_BASE; i < V4L2_CID_LASTP1; ++i) {
		memset(&ctrl, 0, sizeof(ctrl));
		ctrl.id = i;
		if (ioctl(vidfd, VIDIOC_QUERYCTRL, &ctrl) < 0 ||
				ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			continue;
		add_control_value(ctrl.id, ctrl.default_value);
		if (verbose) {
			switch (ctrl.type) {
			case V4L2_CTRL_TYPE_INTEGER:
				printf("   %-20s Range: (%5d,%5d)    "
					"Default: %d\n", ctrl.name,
					ctrl.minimum, ctrl.maximum,
					ctrl.default_value);
				break;
			default:
				printf("   %-20s\n", ctrl.name);
				break;
			}
		}
	}
	if (verbose)
		printf("\n");

	if (probe)
		return;

	/* Print some info about the input port */
	memset(&inp, 0, sizeof(inp));
	inp.index = input;
	if (ioctl(vidfd, VIDIOC_ENUMINPUT, &inp) < 0) {
		fprintf(stderr,
			"Input port %d does not exist on this hardware\n",
			input);
		exit(1);
	}
	fprintf(stderr, "Using input port %s\n", inp.name);

	if (inp.type == V4L2_INPUT_TYPE_TUNER &&
			(std == V4L2_STD_PAL || std == V4L2_STD_SECAM))
		fprintf(stderr, "Warning: PAL/SECAM TV tuning may require "
				"a specific TV band,\n         such as \"-mode "
				"pal-bg\" or \"-mode secam-l\"\n");

	/* Set the video input port */
	if (ioctl(vidfd, VIDIOC_S_INPUT, &input) < 0) {
		fprintf(stderr, "Unable to set input port %d\n", input);
		exit(1);
	}

	/* If the user did not specify a video norm, see if one is needed */
	if (std == 0) {
		memset(&s, 0, sizeof(s));
		s.index = 0;
		if (ioctl(vidfd, VIDIOC_ENUMSTD, &s) < 0) {
			fprintf(stderr, "Unable to query video standards\n");
			exit(1);
		}
		if (s.id != 0) {
			fprintf(stderr, "Warning: no video standard specified; "
					"using %s\n", s.name);
			std = s.id;
		}
	}

	if (std != 0) {
		/* Set the video norm (NTSC/PAL) if necessary */
		/* (This should be done before setting the format.) */
		if (ioctl(vidfd, VIDIOC_S_STD, &std) < 0) {
			if (inp.type == V4L2_INPUT_TYPE_TUNER)
				fprintf(stderr, "This tuner does not appear to "
						"support the selected TV "
						"mode/band.\n");
			else
				perror("unable to set video norm");
			exit(1);
		}
	}

	if (inp.type == V4L2_INPUT_TYPE_TUNER) {
		/* Set the TV audio subchannel */
		memset(&tuner, 0, sizeof(tuner));
		tuner.index = inp.tuner;
		tuner.audmode = audmode;
		if (ioctl(vidfd, VIDIOC_S_TUNER, &tuner) < 0) {
			perror("unable to set TV audio mode");
			exit(1);
		}
	}

	/* Set the tuner frequency, if necessary */
	if (tv_freq >= 0) {
		if (inp.type != V4L2_INPUT_TYPE_TUNER) {
			fprintf(stderr,
				"Input port %d cannot be tuned\n", input);
			exit(1);
		}
		memset(&freq, 0, sizeof(freq));
		freq.tuner = inp.tuner;
		freq.frequency = tv_freq;
		if (ioctl(vidfd, VIDIOC_S_FREQUENCY, &freq) < 0) {
			perror("unable to set TV tuner frequency");
			exit(1);
		}
	}

	/* Set sensor controls to user-specified values or defaults */
	while (control_value_count-- > 0)
		ioctl(vidfd, VIDIOC_S_CTRL,
				&control_values[control_value_count]);

	/* Set the format */
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	switch (format) {
	case FMT_MJPEG:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
		break;
	case FMT_MPEG1:
	case FMT_MPEG2:
	case FMT_MPEG4:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MPEG;
		break;
	}
	if (ioctl(vidfd, VIDIOC_S_FMT, &fmt) < 0) {
		fprintf(stderr, "This encoding format is unsupported!\n");
		exit(1);
	}
	/* Read back the adjusted width and height */
	width = fmt.fmt.pix.width;
	height = fmt.fmt.pix.height;

	/* Reset the frame rate to the same as the input */
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(vidfd, VIDIOC_S_PARM, &parm) < 0) {
		fprintf(stderr, "Unable to reset the frame rate\n");
		exit(1);
	}

	/* Set the frame period, if necessary */
	if (fpsdrop != 1) {
		/* Query the (input) frame rate */
		memset(&parm, 0, sizeof(parm));
		parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (ioctl(vidfd, VIDIOC_G_PARM, &parm) < 0) {
			fprintf(stderr, "Unable to query the new frame rate\n");
			exit(1);
		}

		/* Multiple the frame period by the drop rate and set it */
		frameperiod = parm.parm.capture.timeperframe;
		frameperiod.numerator *= fpsdrop;
		memset(&parm, 0, sizeof(parm));
		parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		parm.parm.capture.timeperframe = frameperiod;
		if (ioctl(vidfd, VIDIOC_S_PARM, &parm) < 0) {
			fprintf(stderr, "Unable to set a new frame rate\n");
			exit(1);
		}

		if (format == FMT_MPEG1 || format == FMT_MPEG2)
			fprintf(stderr, "Warning: non-standard frame rates "
					"with MPEG1 and MPEG2 may not "
					"be compatible\n         with some "
					"AVI players!\n");
	}

	/* Get the frame period */
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(vidfd, VIDIOC_G_PARM, &parm) < 0) {
		fprintf(stderr, "Unable to query the frame rate\n");
		exit(1);
	}
	frameperiod = parm.parm.capture.timeperframe;

	fprintf(stderr, "Capturing video at %dx%d, %.2f FPS\n", width, height,
			(double)frameperiod.denominator /
				(double)frameperiod.numerator);

	/* Set the compression parameters */
	if (format != FMT_MJPEG) {
		memset(&comp, 0, sizeof(comp));
		comp.gop_size = 100;
		comp.max_b_frames = 0; /* B frames are not yet supported */
		comp.aspect_ratio = GO7007_ASPECT_RATIO_1_1;
		comp.flags |= GO7007_COMP_CLOSED_GOP;
		if (ioctl(vidfd, GO7007IOC_S_COMP_PARAMS, &comp) < 0) {
			fprintf(stderr, "Unable to set compression params\n");
			exit(1);
		}

		memset(&mpeg, 0, sizeof(mpeg));
		switch (format) {
		case FMT_MPEG1:
			mpeg.mpeg_video_standard = GO7007_MPEG_VIDEO_MPEG1;
			break;
		case FMT_MPEG2:
			mpeg.mpeg_video_standard = GO7007_MPEG_VIDEO_MPEG2;
			break;
		case FMT_MPEG4:
			mpeg.mpeg_video_standard = GO7007_MPEG_VIDEO_MPEG4;
			break;
		default:
			break;
		}
		if (dvd)
			mpeg.flags |= GO7007_MPEG_FORCE_DVD_MODE;
		if (ioctl(vidfd, GO7007IOC_S_MPEG_PARAMS, &mpeg) < 0) {
			fprintf(stderr, "Unable to set MPEG params\n");
			exit(1);
		}
	}

	/* Set the bitrate */
	if (ioctl(vidfd, GO7007IOC_S_BITRATE, &bitrate) < 0) {
		perror("unable to set bitrate");
		exit(1);
	}

	/* Request that buffers be allocated for memory mapping */
	memset(&req, 0, sizeof(req));
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	req.count = MAX_BUFFERS;
	if (ioctl(vidfd, VIDIOC_REQBUFS, &req) < 0) {
		perror("VIDIOC_REQBUFS");
		exit(1);
	}
	if (verbose)
		printf("Received %d buffers\n", req.count);
	buf_count = req.count;

	/* Map each of the buffers into this process's memory */
	for (i = 0; i < buf_count; ++i) {
		memset(&buf, 0, sizeof(buf));
		buf.index = i;
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (ioctl(vidfd, VIDIOC_QUERYBUF, &buf) < 0) {
			perror("VIDIOC_QUERYBUF");
			exit(1);
		}
		buffers[buf.index] = (unsigned char *)mmap(NULL, buf.length,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				vidfd, buf.m.offset);
	}

	/* Queue all of the buffers for frame capture */
	for (i = 0; i < buf_count; ++i) {
		memset(&buf, 0, sizeof(buf));
		buf.index = i;
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if (ioctl(vidfd, VIDIOC_QBUF, &buf) < 0) {
			perror("VIDIOC_QBUF");
			exit(1);
		}
	}
}

void v4l2_start(void)
{
	int arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	sequence = 0;
	if (ioctl(vidfd, VIDIOC_STREAMON, &arg) < 0) {
		perror("VIDIOC_STREAMON");
		exit(1);
	}
}

void v4l2_stop(void)
{
	int arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (ioctl(vidfd, VIDIOC_STREAMOFF, &arg) < 0) {
		perror("VIDIOC_STREAMOFF");
		exit(1);
	}
}

void write_frame(unsigned char *data, int length, unsigned int fourcc, int key)
{
	unsigned char hdr[16];
	unsigned int offset;

	if (avifd < 0)
		return;

	/* Write the AVI index record */
	PUT_32(hdr, fourcc);
	PUT_32(hdr + 4, key ? 0x10 : 0);
	offset = lseek(avifd, 0, SEEK_CUR);
	PUT_32(hdr + 8, offset - 1024 - 8);
	PUT_32(hdr + 12, length);
	write(idxfd, hdr, 16);

	/* Write the frame data to the AVI file */
	PUT_32(hdr + 4, length);
	write(avifd, hdr, 8);
	write(avifd, data, (length + 1) & ~0x1); /* word-align with junk */
}

int alsa_read(void)
{
	int ret;

	ret = read(audfd, audio_buffer + audio_len,
			sizeof(audio_buffer) - audio_len);
	if (ret < 0) {
		if (errno == EAGAIN)
			return 0;
		perror("Unexpected error reading from audio device");
		return ret;
	}
	if (interrupted) return 0;
	audio_len += ret;
	abytes += ret;
	return ret;
}

int v4l2_frame_capture(struct timeval *capture_time)
{
	struct v4l2_buffer buf;
	int length;
	int key;

	/* Retrieve the filled buffer from the kernel */
	memset(&buf, 0, sizeof(buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	if (ioctl(vidfd, VIDIOC_DQBUF, &buf) < 0) {
		if (interrupted) return 0;
		perror("VIDIOC_DQBUF");
		return -1;
	}

	/* Get the capture timestamp and length of the frame data */
	*capture_time = buf.timestamp;
	key = buf.flags & V4L2_BUF_FLAG_KEYFRAME;
	length = buf.bytesused;
	if (sequence != 0)
		while (++sequence != buf.sequence)
			fprintf(stderr, "Dropped a frame!\n");

	/* First write the accumulated audio */
	if (audfd >= 0) {
		alsa_read();
		if (interrupted) return 0;
		write_frame(audio_buffer, audio_len, FOURCC("01wb"), 1);
		audio_len = 0;
	}

	/* Then write the video frame */
	write_frame(buffers[buf.index], length, FOURCC("00dc"), key);

	/* Update global variables */
	if (length > max_frame_length)
		max_frame_length = length;
	vbytes += length;

	/* Send the frame back to the kernel to be filled again */
	if (ioctl(vidfd, VIDIOC_QBUF, &buf) < 0) {
		perror("VIDIOC_QBUF");
		exit(1);
	}
	return length;
}

int add_video_stream_header(unsigned char *hdr)
{
	unsigned int video_fourcc;

	switch (format) {
	case FMT_MJPEG:
		video_fourcc = FOURCC("mjpg");
		break;
	case FMT_MPEG1:
		video_fourcc = FOURCC("mpg1");
		break;
	case FMT_MPEG2:
		video_fourcc = FOURCC("mpg2");
		break;
	case FMT_MPEG4:
		video_fourcc = FOURCC("DX50");
		break;
	default:
		video_fourcc = 0;
		break;
	}

	PUT_32(hdr, FOURCC("LIST"));
	PUT_32(hdr + 4, 12 - 8 + 64 + 48);
	PUT_32(hdr + 8, FOURCC("strl"));
	PUT_32(hdr + 12, FOURCC("strh"));
	PUT_32(hdr + 12 + 4, 64 - 8);
	PUT_32(hdr + 12 + 8, FOURCC("vids"));
	PUT_32(hdr + 12 + 12, video_fourcc);
	PUT_32(hdr + 12 + 28, frameperiod.numerator);
	PUT_32(hdr + 12 + 32, frameperiod.denominator);
	PUT_32(hdr + 12 + 40, avi_frame_count);
	PUT_32(hdr + 12 + 44, max_frame_length);
	PUT_16(hdr + 12 + 60, width);
	PUT_16(hdr + 12 + 62, height);
	PUT_32(hdr + 12 + 64, FOURCC("strf"));
	PUT_32(hdr + 12 + 64 + 4, 48 - 8);
	PUT_32(hdr + 12 + 64 + 8, 48 - 8);
	PUT_32(hdr + 12 + 64 + 12, width);
	PUT_32(hdr + 12 + 64 + 16, height);
	PUT_32(hdr + 12 + 64 + 20, 1);
	PUT_32(hdr + 12 + 64 + 22, 24);
	PUT_32(hdr + 12 + 64 + 24, video_fourcc);
	PUT_32(hdr + 12 + 64 + 28, width * height * 3);
	return 12 + 64 + 48;
}

int add_audio_stream_header(unsigned char *hdr)
{
	PUT_32(hdr, FOURCC("LIST"));
	PUT_32(hdr + 4, 12 - 8 + 64 + 26);
	PUT_32(hdr + 8, FOURCC("strl"));

	PUT_32(hdr + 12, FOURCC("strh"));
	PUT_32(hdr + 12 + 4, 64 - 8);
	PUT_32(hdr + 12 + 8, FOURCC("auds"));
	PUT_32(hdr + 12 + 12, 1);
	PUT_32(hdr + 12 + 28, 4);
	PUT_32(hdr + 12 + 32, 48000 << 2);
	PUT_32(hdr + 12 + 40, abytes >> 2);
	PUT_32(hdr + 12 + 44, 48000 << 1);
	PUT_32(hdr + 12 + 52, 4);

	PUT_32(hdr + 12 + 64, FOURCC("strf"));
	PUT_32(hdr + 12 + 64 + 4, 26 - 8);
	PUT_16(hdr + 12 + 64 + 8, 1);
	PUT_16(hdr + 12 + 64 + 10, 2);
	PUT_32(hdr + 12 + 64 + 12, 48000);
	PUT_32(hdr + 12 + 64 + 16, 48000 << 2);
	PUT_16(hdr + 12 + 64 + 20, 4);
	PUT_16(hdr + 12 + 64 + 22, 16);
	return 12 + 64 + 26;
}

void avi_finish(void)
{
	int i, movielen, filelen, off;
	unsigned char hdr[1024 + 12];

	movielen = lseek(avifd, 0, SEEK_CUR);

	PUT_32(hdr, FOURCC("idx1"));
	PUT_32(hdr + 4, lseek(idxfd, 0, SEEK_CUR));
	write(avifd, hdr, 8);
	lseek(idxfd, 0, SEEK_SET);
	while ((i = read(idxfd, hdr, sizeof(hdr))) > 0)
		if (write(avifd, hdr, i) < 0) {
			perror("Unable to write index data to AVI file");
			exit(1);
		}
	close(idxfd);

	filelen = lseek(avifd, 0, SEEK_CUR);

	memset(hdr, 0, sizeof(hdr));
	PUT_32(hdr, FOURCC("RIFF"));
	PUT_32(hdr + 4, lseek(avifd, 0, SEEK_CUR) - 8);
	PUT_32(hdr + 8, FOURCC("AVI "));
	PUT_32(hdr + 12, FOURCC("LIST"));
	PUT_32(hdr + 12 + 8, FOURCC("hdrl"));
	PUT_32(hdr + 12 + 12, FOURCC("avih"));
	PUT_32(hdr + 12 + 12 + 4, 64 - 8);
	/* bizarre math to do microsecond arithmetic in 32-bit ints */
	/* =>   1000000 * frameperiod.numerator / frameperiod.denominator */
	PUT_32(hdr + 12 + 12 + 8, (frameperiod.numerator * 15625 /
						frameperiod.denominator) * 64 +
					((frameperiod.numerator * 15625) %
					 	frameperiod.denominator) * 64 /
					frameperiod.denominator);
	PUT_32(hdr + 12 + 12 + 20, 2320);
	PUT_32(hdr + 12 + 12 + 24, avi_frame_count);
	PUT_32(hdr + 12 + 12 + 32, audfd < 0 ? 1 : 2);
	PUT_32(hdr + 12 + 12 + 36, 128*1024);
	PUT_32(hdr + 12 + 12 + 40, width);
	PUT_32(hdr + 12 + 12 + 44, height);
	off = 64;
	off += add_video_stream_header(hdr + 12 + 12 + off);
	if (audfd >= 0)
		off += add_audio_stream_header(hdr + 12 + 12 + off);
	PUT_32(hdr + 12 + 4, 12 - 8 + off);

	PUT_32(hdr + 12 + 12 + off, FOURCC("JUNK"));
	PUT_32(hdr + 12 + 12 + off + 4, 1024 - 12 - 12 - off - 8);
	PUT_32(hdr + 1024, FOURCC("LIST"));
	PUT_32(hdr + 1024 + 4, movielen - 1024 - 8);
	PUT_32(hdr + 1024 + 8, FOURCC("movi"));
	lseek(avifd, 0, SEEK_SET);
	write(avifd, hdr, 1024 + 12);
	close(avifd);

	fprintf(stderr,
		"\n    Video data written to file: %d bytes of ", vbytes);
	switch (format) {
	case FMT_MJPEG:
		fprintf(stderr, "Motion-JPEG\n");
		break;
	case FMT_MPEG1:
		fprintf(stderr, "MPEG1\n");
		break;
	case FMT_MPEG2:
		fprintf(stderr, "MPEG2\n");
		break;
	case FMT_MPEG4:
		fprintf(stderr, "MPEG4\n");
		break;
	}
	if (abytes > 0)
		fprintf(stderr,
			"    Audio data written to file: %d bytes of %s\n",
			abytes, "uncompressed PCM");
	fprintf(stderr, "    AVI file format overhead  : %d bytes\n",
			filelen - vbytes - abytes);
	fprintf(stderr, "    Total file size           : %d bytes\n\n",
			filelen);
}

void interrupt_handler(int signal)
{
	interrupted = 1;
}

int main(int argc, char **argv)
{
	struct sigaction sa;
	struct timeval cur, start, mmark;
	int csec, vframe_len;
	unsigned int filesize;
	int file_count = 1;
	int mcount = 0, mrate = 0, mbytes = 0;

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = interrupt_handler;
	sa.sa_flags = SA_RESTART;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);

	memset(audio_buffer, 0, sizeof(audio_buffer));
	parse_opts(argc, argv);
	if (!nowrite) {
		if (strstr(avibase, "%d"))
			sprintf(avifile, avibase, file_count);
		else
			strcpy(avifile, avibase);
		open_avifile();
	}
	open_devices();
	v4l2_init();
	if (audfd >= 0)
		alsa_init();
	if (probe) {
		printf("For usage help, run `%s -help`\n", argv[0]);
		return 0;
	}
	v4l2_start();
	for (;;) {
		/* Retrieve a new video frame and audio frame */
		vframe_len = v4l2_frame_capture(&cur);
		if (interrupted) break;
		if (vframe_len < 0)
			break;

		/* Display file length, timestamp, and frame count */
		if (avifd < 0)
			filesize = vbytes + abytes;
		else
			filesize = lseek(avifd, 0, SEEK_CUR);
		++avi_frame_count;
		if (total_frames++ == 0)
			mmark = start = cur;
		csec = 100 * (cur.tv_sec - start.tv_sec) +
			(cur.tv_usec - start.tv_usec) / 10000;
		fprintf(stderr,
			"\r %02d:%02d.%02d  Frames: %5d  "
			"AVI size: %9d", csec / 6000, csec / 100 % 60,
			csec % 100, total_frames, filesize);

		/* Calculate and display video bitrate */
		if (mcount++ == 50) {
			csec = 100 * (cur.tv_sec - mmark.tv_sec) +
				(cur.tv_usec - mmark.tv_usec) / 10000;
			mrate = (4 * mbytes) / (5 * csec);
			mbytes = 0;
			mcount = 1;
			mmark = cur;
		}
		mbytes += vframe_len;
		if (mrate > 0)
			fprintf(stderr, "  Video bitrate: %5d kbps", mrate);

		/* If we've reached the maximum AVI length, make a new file
		 * if we were given a filename with %d, or otherwise stop
		 * recording */
		if (avifd >= 0 && filesize >= MAX_FILE_SIZE) {
			if(strstr(avibase, "%d")) {
				avi_finish();
				++file_count;
				sprintf(avifile, avibase, file_count);
				avi_frame_count = 0;
				max_frame_length = 0;
				abytes = 0;
				vbytes = 0;
				open_avifile();
			} else {
				fprintf(stderr,
					"\nAVI file size limit reached\n");
				break;
			}
		}

		/* If we've reached the user-specified maximum number of
		 * frames to record, stop */
		if (frame_limit >= 0 && total_frames == frame_limit)
			break;
		/* If we've reached the user-specified maximum duration, stop */
		if (duration >= 0 && cur.tv_sec >= start.tv_sec + duration &&
				cur.tv_usec >= start.tv_usec)
			break;
	}
	fprintf(stderr, "\n");
	v4l2_stop();
	if (avifd >= 0)
		avi_finish();
	return 0;
};
