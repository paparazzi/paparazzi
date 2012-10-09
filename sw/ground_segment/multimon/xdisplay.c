/*
 *      xdisplay.c -- actually displaying things
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
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

/* ---------------------------------------------------------------------- */

#define WIDTH 512
#define HEIGHT 256

union comdata {
	short s[WIDTH];
	unsigned char b[0];
};

#define SAMPLING_RATE 22050

/* ---------------------------------------------------------------------- */

/*
 * child data structures
 */

static Display *display = NULL;
static Window window;
static GC gc;
static Pixmap pixmap;
static unsigned long col_zeroline;
static unsigned long col_background;
static unsigned long col_trace;

static int cmdpipe[2];
static int datapipe[2];

/*
 * parent data
 */

struct xdisp {
	int used;
	pid_t pid;
	int cmdfd;
	int datafd;
};

#define NUMCLI 16

static struct xdisp cli[NUMCLI] = { { 0, }, };

/* ---------------------------------------------------------------------- */

static int x_error_handler(Display *disp, XErrorEvent *evt)
{
    char err_buf[256], mesg[256], number[256];
    char *mtype = "XlibMessage";

    XGetErrorText(disp, evt->error_code, err_buf, sizeof(err_buf));
    (void)fprintf(stderr, "X Error: %s\n", err_buf);
    XGetErrorDatabaseText(disp, mtype, "MajorCode",
                          "Request Major code %d", mesg, sizeof(mesg));
    (void)fprintf(stderr, mesg, evt->request_code);
    (void)sprintf(number, "%d", evt->request_code);
    XGetErrorDatabaseText(disp, "XRequest", number, "", err_buf,
                          sizeof(err_buf));
    (void)fprintf(stderr, " (%s)\n", err_buf);

    abort();
}

/* ---------------------------------------------------------------------- */

static Bool predicate(Display *display, XEvent *event, char *arg)
{
	return True;
}

/* ---------------------------------------------------------------------- */

static char *x_getkey(void)
{
	XWindowAttributes winattrs;
	XEvent evt;
	static char kbuf[32];
	int i;

	if (!display)
		return NULL;
	while (XCheckIfEvent(display, &evt, predicate, NULL)) {
		switch (evt.type) {
		case KeyPress:
			i = XLookupString((XKeyEvent *)&evt, kbuf,
					  sizeof(kbuf)-1, NULL, NULL);
			if (i) {
				kbuf[i] = 0;
				return kbuf;
			}
			continue;
		case DestroyNotify:
			XCloseDisplay(display);
			exit(0);
		case Expose:
			XGetWindowAttributes(display, window, &winattrs);
			XCopyArea(display, window, pixmap, gc, 0, 0,
				  winattrs.width, winattrs.height, 0, 0);
			continue;
		default:
			continue;
		}
	}
	return NULL;
}

/* ---------------------------------------------------------------------- */

static void process_keystrokes(void)
{
	char *cp;

	while ((cp = x_getkey())) {
		printf("X: Keys pressed: %s\n", cp);
	}
}

/* ---------------------------------------------------------------------- */

static int do_x_select(int fd, int wr)
{
	int *xconn, xconnnum;
	int max = fd, i;
	fd_set rmask, wmask;

	if (!XInternalConnectionNumbers(display, &xconn, &xconnnum)) {
		perror("XInternalConnectionNumbers");
		exit(1);
	}
	FD_ZERO(&rmask);
	FD_ZERO(&wmask);
	if (wr)
		FD_SET(fd, &wmask);
	else
		FD_SET(fd, &rmask);
	for (i = 0; i < xconnnum; i++) {
		FD_SET(xconn[i], &rmask);
		if (xconn[i] > max)
			max = xconn[i];
	}
	i = select(max+1, &rmask, &wmask, NULL, NULL);
	if (i < 0) {
		perror("select");
		exit(1);
	}
	for (i = 0; i < xconnnum; i++)
		if (FD_ISSET(xconn[i], &rmask))
			XProcessInternalConnection(display, xconn[i]);
	XFree(xconn);
	process_keystrokes();
	if (wr)
		return FD_ISSET(fd, &wmask);
	else
		return FD_ISSET(fd, &rmask);
}

/* ---------------------------------------------------------------------- */

static void child_win_init(void)
{
	XSetWindowAttributes attr;
	XGCValues gcv;
	XColor color, dummy;
	XSizeHints sizehints;

	/*
	 * start graphical output
	 */
	if (!(display = XOpenDisplay(NULL))) {
		fprintf(stderr, "X: Unable to open X display\n");
		exit(1);
	}
	XSetErrorHandler(x_error_handler);
        XAllocNamedColor(display, DefaultColormap(display, 0), "red",
                         &color, &dummy);
        col_zeroline = color.pixel;
        col_background = WhitePixel(display, 0);
        col_trace = BlackPixel(display, 0);
        attr.background_pixel = col_background;
	window = XCreateWindow(display, XRootWindow(display, 0),
			       200, 200, WIDTH, HEIGHT, 5,
			       DefaultDepth(display, 0),
			       InputOutput, DefaultVisual(display, 0),
			       CWBackPixel, &attr);
        if (!(pixmap = XCreatePixmap(display, window, WIDTH, HEIGHT,
                                     DefaultDepth(display, 0)))) {
                fprintf(stderr, "X: unable to open offscreen pixmap\n");
                exit(1);
        }
        XSelectInput(display, window, KeyPressMask | StructureNotifyMask
                     | ExposureMask) ;
	gcv.line_width = 1;
	gcv.line_style = LineSolid;
	gc = XCreateGC(display, pixmap, GCForeground | GCLineWidth, &gcv);
	/*
	 * Do not allow the window to be resized
	 */
	memset(&sizehints, 0, sizeof(sizehints));
	sizehints.min_width = sizehints.max_width = WIDTH;
	sizehints.min_height = sizehints.max_height = HEIGHT;
	sizehints.flags = PMinSize | PMaxSize;
	XSetWMNormalHints(display, window, &sizehints);
	XMapWindow(display, window);
	XSynchronize(display, 1);
}

/* ---------------------------------------------------------------------- */

#define YCOORD(x) (((x)>>8)+(HEIGHT/2))

static void child_process(void)
{
	union comdata d;
	unsigned char *bp;
	int i, j;

	/*
	 * main loop
	 */
	for (;;) {
		/*
		 * send synchronisation mark
		 */
		while (!do_x_select(cmdpipe[1], 1));
		i = write(cmdpipe[1], "r", 1);
		if (i < 1) {
			perror("write");
			exit(1);
		}
		/*
		 * read data
		 */
		for (j = sizeof(d), bp = d.b; j > 0; ) {
			while (!do_x_select(datapipe[0], 0));
			i = read(datapipe[0], bp, j);
			if (i < 1) {
				perror("read");
				exit(1);
			}
			j -= i;
			bp += i;
		}
		/*
		 * clear pixmap
		 */
		XSetState(display, gc, col_background, col_background,
			  GXcopy, AllPlanes);
                XFillRectangle(display, pixmap, gc, 0, 0,
                               WIDTH, HEIGHT);
		/*
		 * draw zero line
		 */
		XSetForeground(display, gc, col_zeroline);
		XDrawLine(display, pixmap, gc, 0, YCOORD(0), WIDTH,
			  YCOORD(0));
		/*
		 * draw input
		 */
		XSetForeground(display, gc, col_trace);
		for (i = 1; i < WIDTH; i++)
                XDrawLine(display, pixmap, gc, i-1, YCOORD(d.s[i-1]),
                          i, YCOORD(d.s[i]));
		XCopyArea(display, pixmap, window, gc, 0, 0,
			  WIDTH, HEIGHT, 0, 0);
		/* XSync(display, 0); */
	}
        XDestroyWindow(display, window);
        XCloseDisplay(display);
}

/* ---------------------------------------------------------------------- */

static void sigchld_handler(int sig)
{
	pid_t pid;
	int st;
	unsigned int cnum;

	while ((pid = wait4(0, &st, WNOHANG, NULL)) != (pid_t)-1) {
		for (cnum = 0; cnum < NUMCLI; cnum++)
			if (cli[cnum].used && cli[cnum].pid == pid) {
				cli[cnum].used = 0;
				close(cli[cnum].cmdfd);
				close(cli[cnum].datafd);
				cli[cnum].pid = (pid_t)-1;
				fprintf(stderr, "child process %i died, "
					"status %i\n", (int)pid, st);
			}
	}
}

/* ---------------------------------------------------------------------- */

void xdisp_terminate(int cnum)
{
	if (cnum  < 0 || cnum >= NUMCLI)
		return;
	kill(cli[cnum].pid, SIGTERM);
}

/* ---------------------------------------------------------------------- */

int xdisp_start(void)
{
	unsigned int cnum;

	/*
	 * find free client struct
	 */
	for (cnum = 0; (cnum < NUMCLI) && cli[cnum].used; cnum++);
	if (cnum >= NUMCLI)
		return -1;
	signal(SIGCHLD, sigchld_handler);
	/*
	 * start "IPC" mechanism (using the pipes)
	 */
	if (pipe(cmdpipe))
		return -1;
	if (pipe(datapipe)) {
		close(cmdpipe[0]);
		close(cmdpipe[1]);
	}
	if ((cli[cnum].pid = fork())) {
		if (cli[cnum].pid == (pid_t)-1) {
			/* error */
			close(cmdpipe[0]);
			close(cmdpipe[1]);
			close(datapipe[0]);
			close(datapipe[1]);
			return -1;
		}
		/* parent */
		cli[cnum].cmdfd = cmdpipe[0];
		close(cmdpipe[1]);
		close(datapipe[0]);
		cli[cnum].datafd = datapipe[1];
		cli[cnum].used = 1;
		fcntl(cmdpipe[0], F_SETFL,
		      (fcntl(cmdpipe[0], F_GETFL, 0) | O_NDELAY));
		return cnum;
	}
	/*
	 * child; the X process
	 */
	close(cmdpipe[0]);
	close(datapipe[1]);
	close(0);  /* close stdin */
	child_win_init();
	child_process();
	exit(0);
}

/* ---------------------------------------------------------------------- */

int xdisp_update(int cnum, float *f)
{
	unsigned char *bp;
	short *sp;
	int i, j;
	char c;
	union comdata d;

	if (cnum < 0 || cnum >= NUMCLI)
		return 0;
	i = read(cli[cnum].cmdfd, &c, 1);
	if (i < 0 && errno != EAGAIN) {
		perror("read");
		xdisp_terminate(cnum);
		return 0;
	}
	if (i < 1)
		return 0;
	if (c != 'r')
		return 0;
	for (sp = d.s, i = 0; i < WIDTH; i++, sp++, f++) {
		if (*f >= 1)
			*sp = 32767;
		else if (*f <= -1)
			*sp = -32767;
		else
			*sp = 32767.0 * (*f);
	}
	bp = d.b;
	j = sizeof(d);
	while (j > 0) {
		i = write(cli[cnum].datafd, bp, j);
		if (i < 0 && errno != EAGAIN) {
			perror("write");
			xdisp_terminate(cnum);
			return 0;
		}
		if (i > 0) {
			bp += i;
			j -= i;
		}
	}
	return 1;
}

/* ---------------------------------------------------------------------- */
