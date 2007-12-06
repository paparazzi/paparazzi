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
#include <errno.h>

#include <curses.h>

#include "../include/go7007.h"

#define MAX_BUFFERS	32

char *vdevice = NULL;
int vidfd = -1;
int buf_count = 0;
unsigned int sequence;
int input = 0;
int width = 0, height = 0;
v4l2_std_id std = 0;
int interrupted = 0;
void *buffers[MAX_BUFFERS];
struct go7007_md_params regparams[4];
struct go7007_md_region regions[4];

void usage(char *progname)
{
	printf(
"`modet` detects motion in video captured from a WIS GO7007 video encoder and\n"
"displays it with ncurses to the controlling terminal.  Up to three separate\n"
"regions of interest (1-3) may be specified as lists of rectangles.  The\n"
"default region of interest (0) contains all areas of the image not specified\n"
"in other regions.  Rectangles must be aligned on 16-pixel boundaries and\n"
"regions 1-3 must not overlap.\n\n"
"Usage: modet [GLOBAL OPTION]... [-region <n> [REGION OPTION]... RECT...]\n\n"
"Example:\n"
"  modet -input 2 -region 1 0x0-720x240 -region 2 0x240-720x480\n\n"
"Global options:\n"
"  -vdevice <V4L2 device path>   Explicitly specify the V4L2 device to use\n"
"  -input <n>                    Capture from input port <n> (default: 0)\n"
"  -mode <mode>                  Select mode for input video (default: ntsc)\n"
"                                  Modes: ntsc, pal, secam\n"
"  -width <n>                    Specify captured image width\n"
"  -height <n>                   Specify captured image height\n"
"  -defaultpixelthresh           Specify default pixel change threshold\n"
"  -defaultmotionthresh          Specify default motion vector threshold\n"
"  -defaulttrigger               Specify default region trigger level\n\n"
"Region options:\n"
"  -pixelthresh                  Specify regional pixel change threshold\n"
"  -motionthresh                 Specify regional motion vector threshold\n"
"  -trigger                      Specify trigger level\n");
	exit(1);
}

void find_device(void)
{
	struct stat si;
	int i;
	char sympath[PATH_MAX], canonpath[PATH_MAX], gopath[PATH_MAX];
	static char vdev[PATH_MAX];

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
}

void parse_rectangle(char *arg, int region)
{
	struct v4l2_clip *clip;
	char *end;
	int x;

	clip = malloc(sizeof(*clip));

	clip->c.left = strtol(arg, &end, 10);
	if (*end != 'x' && *end != 'X') {
		fprintf(stderr, "Error: \"%s\" is not a valid rectangle "
				"definition\n", arg);
		exit(1);
	}
	if (clip->c.left & 0xF) {
		fprintf(stderr, "Error: %d is not a multiple of 16\n",
				clip->c.left);
		exit(1);
	}

	clip->c.top = strtol(end + 1, &end, 10);
	if (*end != '-') {
		fprintf(stderr, "Error: \"%s\" is not a valid rectangle "
				"definition\n", arg);
		exit(1);
	}
	if (clip->c.top & 0xF) {
		fprintf(stderr, "Error: %d is not a multiple of 16\n",
				clip->c.top);
		exit(1);
	}

	x = strtol(end + 1, &end, 10);
	if (*end != 'x' && *end != 'X') {
		fprintf(stderr, "Error: \"%s\" is not a valid rectangle "
				"definition\n", arg);
		exit(1);
	}
	if (x & 0xF) {
		fprintf(stderr, "Error: %d is not a multiple of 16\n", x);
		exit(1);
	}
	if (x <= clip->c.left) {
		fprintf(stderr, "Error: right edge %d must be greater than "
				"left edge %d\n", x, clip->c.left);
		exit(1);
	}
	clip->c.width = x - clip->c.left;

	x = strtol(end + 1, &end, 10);
	if (*end) {
		fprintf(stderr, "Error: \"%s\" is not a valid rectangle "
				"definition\n", arg);
		exit(1);
	}
	if (x & 0xF) {
		fprintf(stderr, "Error: %d is not a multiple of 16\n", x);
		exit(1);
	}
	if (x <= clip->c.top) {
		fprintf(stderr, "Error: bottom edge %d must be greater than "
				"top edge %d\n", x, clip->c.top);
		exit(1);
	}
	clip->c.height = x - clip->c.top;

	clip->next = regions[region].clips;
	regions[region].clips = clip;
}

void parse_opts(int argc, char **argv)
{
	int i, arg, region = 0;

	for (i = 1; i < argc; ++i) {
		if (argv[i][0] != '-') {
			if (region == 0) {
				fprintf(stderr, "Error: You must specify a "
						"region number (1-3) before "
						"listing rectangles.\n");
				exit(1);
			}
			parse_rectangle(argv[i], region);
	/* options that take no arguments go below here */
		} else if (!strcmp(argv[i], "-help")) {
			usage(argv[0]);
		} else if (i + 1 >= argc) {
			usage(argv[0]);
	/* options that take one argument go below here */
		} else if (!strcmp(argv[i], "-region")) {
			region = atoi(argv[++i]);
			if (region < 1 || region > 3) {
				fprintf(stderr, "Error: only regions 1-3 may "
					"be configured independently\n");
				exit(1);
			}
		} else if (!strcmp(argv[i], "-pixelthresh")) {
			arg = atoi(argv[++i]);
			if (arg < 0 || arg > 65535) {
				fprintf(stderr, "Error: Pixel threshold must "
						"be between 0 and 65535\n");
				exit(1);
			}
			regparams[region].pixel_threshold = arg;
		} else if (!strcmp(argv[i], "-motionthresh")) {
			arg = atoi(argv[++i]);
			if (arg < 0 || arg > 65535) {
				fprintf(stderr, "Error: Motion threshold must "
						"be between 0 and 65535\n");
				exit(1);
			}
			regparams[region].motion_threshold = arg;
		} else if (!strcmp(argv[i], "-trigger")) {
			arg = atoi(argv[++i]);
			if (arg < 0 || arg > 65535) {
				fprintf(stderr, "Error: Trigger level must "
						"be between 0 and 65535\n");
				exit(1);
			}
			regparams[region].trigger = arg;
		} else if (region != 0) {
			usage(argv[0]);
	/* options that must be specified before any -region flags go here */
		} else if (!strcmp(argv[i], "-defaultpixelthresh")) {
			arg = atoi(argv[++i]);
			if (arg < 0 || arg > 65535) {
				fprintf(stderr, "Error: Pixel threshold must "
						"be between 0 and 65535\n");
				exit(1);
			}
			for (region = 0; region < 4; ++region)
				regparams[region].pixel_threshold = arg;
			region = 0;
		} else if (!strcmp(argv[i], "-defaultmotionthresh")) {
			arg = atoi(argv[++i]);
			if (arg < 0 || arg > 65535) {
				fprintf(stderr, "Error: Motion threshold must "
						"be between 0 and 65535\n");
				exit(1);
			}
			for (region = 0; region < 4; ++region)
				regparams[region].motion_threshold = arg;
			region = 0;
		} else if (!strcmp(argv[i], "-defaulttrigger")) {
			arg = atoi(argv[++i]);
			if (arg < 0 || arg > 65535) {
				fprintf(stderr, "Error: Trigger level must "
						"be between 0 and 65535\n");
				exit(1);
			}
			for (region = 0; region < 4; ++region)
				regparams[region].trigger = arg;
			region = 0;
		} else if (!strcmp(argv[i], "-vdevice")) {
			vdevice = argv[++i];
		} else if (!strcmp(argv[i], "-input")) {
			input = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-width")) {
			width = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-height")) {
			height = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "-mode")) {
			++i;
			if (!strcasecmp(argv[i], "ntsc"))
				std = V4L2_STD_NTSC_M;
			else if (!strcasecmp(argv[i], "pal"))
				std = V4L2_STD_PAL;
			else if (!strcasecmp(argv[i], "secam"))
				std = V4L2_STD_SECAM;
			else if (!strcasecmp(argv[i], "ntsc-j"))
				std = V4L2_STD_NTSC_M_JP;
			else {
				fprintf(stderr, "Valid modes: ntsc, pal, "
						"secam, ntsc-j\n");
				exit(1);
			}
		} else {
			usage(argv[0]);
		}
	}
	if (i != argc)
		usage(argv[0]);
	find_device();
}

void open_device(void)
{
	vidfd = open(vdevice, O_RDWR);
	if (vidfd < 0) {
		fprintf(stderr, "Unable to open %s: %s\n", vdevice,
				strerror(errno));
		exit(1);
	}
}

void v4l2_init(void)
{
	struct v4l2_capability cap;
	struct v4l2_streamparm parm;
	struct v4l2_format fmt;
	struct go7007_mpeg_params mpeg;
	struct v4l2_requestbuffers req;
	struct v4l2_buffer buf;
	struct v4l2_input inp;
	struct v4l2_standard s;
	int i;

	/* First query the capabilities of the video capture device */
	if (ioctl(vidfd, VIDIOC_QUERYCAP, &cap) < 0) {
		perror("VIDIOC_QUERYCAP");
		exit(1);
	}

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

	if (width == 0 || height == 0) {
		/* Find the maximum frame size */
		memset(&fmt, 0, sizeof(fmt));
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (ioctl(vidfd, VIDIOC_G_FMT, &fmt) < 0) {
			fprintf(stderr, "Unable to query the frame size!\n");
			exit(1);
		}
		if (width == 0)
			width = fmt.fmt.pix.width;
		if (height == 0)
			height = fmt.fmt.pix.height;
	}

	/* Set the format to MPEG */
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MPEG;
	if (ioctl(vidfd, VIDIOC_S_FMT, &fmt) < 0) {
		fprintf(stderr, "This encoding format is unsupported!\n");
		exit(1);
	}
	width = fmt.fmt.pix.width;
	height = fmt.fmt.pix.height;

	/* Set the MPEG standard to MPEG4 */
	memset(&mpeg, 0, sizeof(mpeg));
	mpeg.mpeg_video_standard = GO7007_MPEG_VIDEO_MPEG4;
	if (ioctl(vidfd, GO7007IOC_S_MPEG_PARAMS, &mpeg) < 0) {
		fprintf(stderr, "Unable to set MPEG params\n");
		exit(1);
	}

	/* Get the frame period (just for display) */
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(vidfd, VIDIOC_G_PARM, &parm) < 0) {
		fprintf(stderr, "Unable to query the frame rate\n");
		exit(1);
	}
	fprintf(stderr, "Capturing video at %dx%d, %.2f FPS\n", width, height,
			(double)parm.parm.capture.timeperframe.denominator /
			(double)parm.parm.capture.timeperframe.numerator);

	/* Request that buffers be allocated for memory mapping */
	memset(&req, 0, sizeof(req));
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	req.count = MAX_BUFFERS;
	if (ioctl(vidfd, VIDIOC_REQBUFS, &req) < 0) {
		perror("VIDIOC_REQBUFS");
		exit(1);
	}
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

void validate_regions(void)
{
	struct v4l2_clip *clip;
	int i;

	for (i = 0; i < 4; ++i)
		for (clip = regions[i].clips; clip; clip = clip->next)
			if (clip->c.left + clip->c.width > width ||
					clip->c.top + clip->c.height > height) {
				fprintf(stderr, "Error: rectangle %dx%d-%dx%d "
						"does not fit in frame "
						"resolution %dx%d\n",
						clip->c.left, clip->c.top,
						clip->c.left + clip->c.width,
						clip->c.top + clip->c.height,
						width, height);
				exit(1);
			}
}

void v4l2_set_regions(void)
{
	int i;

	for (i = 0; i < 4; ++i) {
		if (i > 0 && !regions[i].clips)
			continue;
		if (ioctl(vidfd, GO7007IOC_S_MD_PARAMS, &regparams[i]) < 0) {
			fprintf(stderr, "Unable to set motion detection "
					"parameters for region %d\n", i);
			exit(1);
		}
		if (i == 0)
			continue;
		if (ioctl(vidfd, GO7007IOC_S_MD_REGION, &regions[i]) < 0) {
			fprintf(stderr, "Unable to set motion detection "
					"area for region %d\n", i);
			exit(1);
		}
	}
}

void interrupt_handler(int signal)
{
	interrupted = 1;
}

void display_bitmap(unsigned char *bitmap, int w, int h)
{
	int x, y, b, stride = (w + 7) >> 3;

	for (y = 0; y < h; ++y)
		for (x = 0; x < w; ++x) {
			b = bitmap[stride * y + (x >> 3)] & (1 << (x & 0x7));
			mvaddch(y, x, b ? '@' : '.');
		}
}

int main(int argc, char **argv)
{
	struct sigaction sa;
	struct v4l2_buffer buf;
	unsigned char *bitmap, empty_map[216] = { 0 };
	int i;

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = interrupt_handler;
	sa.sa_flags = SA_RESTART;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);

	for (i = 0; i < 4; ++i) {
		memset(&regparams[i], 0, sizeof(regparams[i]));
		memset(&regions[i], 0, sizeof(regions[i]));
		regions[i].region = i;
		regparams[i].region = i;
		regparams[i].trigger = 20;
		regparams[i].pixel_threshold = 100;
		regparams[i].motion_threshold = 8000;
	}
	parse_opts(argc, argv);
	open_device();
	v4l2_init();
	validate_regions();
	v4l2_set_regions();
	initscr();
	if (LINES < height >> 4 || COLS < 80) {
		fprintf(stderr, "Your terminal must be at least 80x%d "
				"characters for this to run.\n", height >> 4);
		endwin();
		return 1;
	}
	cbreak();
	noecho();
	nonl();
	mvprintw(1, 55, "Region 0:");
	mvprintw(2, 55, "Region 1:");
	mvprintw(3, 55, "Region 2:");
	mvprintw(4, 55, "Region 3:");
	v4l2_start();
	for (;;) {
		/* Retrieve the filled buffer from the kernel */
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if (ioctl(vidfd, VIDIOC_DQBUF, &buf) < 0) {
			if (interrupted)
				break;
			perror("VIDIOC_DQBUF");
			exit(1);
		}
		if (interrupted)
			break;

		if (buf.reserved)
			bitmap = buffers[buf.index] + buf.bytesused;
		else
			bitmap = empty_map;

		display_bitmap(bitmap, width >> 4, height >> 4);
		mvprintw(1, 65, buf.reserved & 0x1 ? "active" : "      ");
		mvprintw(2, 65, buf.reserved & 0x2 ? "active" : "      ");
		mvprintw(3, 65, buf.reserved & 0x4 ? "active" : "      ");
		mvprintw(4, 65, buf.reserved & 0x8 ? "active" : "      ");
		move(0, 0);
		refresh();

		/* Send the frame back to the kernel to be filled again */
		if (ioctl(vidfd, VIDIOC_QBUF, &buf) < 0) {
			perror("VIDIOC_QBUF");
			exit(1);
		}
	}
	endwin();
	v4l2_stop();
	return 0;
};
