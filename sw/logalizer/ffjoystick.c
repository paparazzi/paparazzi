/*
 * based on ffmvforce.c
 *
 */

/*
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * You can contact the author by email at this address:
 * Johann Deneux <deneux@ifrance.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <linux/input.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define BIT(x) (1<<(x))
#define STR_LEN	64
#define	PPRZ_MAX 9600
#define	PPRZ_MIN (-PPRZ_MAX)
#define max(a,b)	((a)>(b)?(a):(b))

/* File descriptor of the force feedback /dev entry */
static int ff_fd;
static struct ff_effect effect;

static void welcome()
{
	const char* txt[] = {
"ffjoystick: generate force feedback reaction from Paparazzi autopilot",
"Set COMMANDS message repeat rate low in telemetry and"
"use aircraft with rudder on channel 1 and elevator on 2",
"USE WITH CARE !!! HOLD STRONGLY YOUR WHEEL OR JOYSTICK TO PREVENT DAMAGES",
"To run this program, run it with at least one argument.",
"",
NULL };

	const char** p = txt;

	while (*p) {
		printf("%s\n", *p);
		p++;
	}
}

static void generate_force(double nx, double ny)
{
	static int first = 1;
	double angle;

	angle = atan2(nx, -ny);
//printf("n: %4.2f %4.2f angle: %4.2f\n", nx, ny, angle);
	effect.type = FF_CONSTANT;
        effect.u.constant.level = 0x7fff * max(fabs(nx), fabs(ny));
        effect.direction = 0x8000 * (angle + M_PI)/M_PI;
//printf("level: %04x direction: %04x\n", (unsigned int)effect.u.constant.level, (unsigned int)effect.direction);
        effect.u.constant.envelope.attack_length = 0;
        effect.u.constant.envelope.attack_level = 0;
        effect.u.constant.envelope.fade_length = 0;
        effect.u.constant.envelope.fade_level = 0;
        effect.trigger.button = 0;
        effect.trigger.interval = 0;
        effect.replay.length = 0xffff;
        effect.replay.delay = 0;

	if (first) {
		effect.id = -1;
	}

        if (ioctl(ff_fd, EVIOCSFF, &effect) < 0) {
/* If updates are sent to frequently, they can be refused */
        }

	/* If first time, start to play the effect */
	if (first) {
		struct input_event play;
		play.type = EV_FF;
		play.code = effect.id;
		play.value = 1;

		if (write(ff_fd, (const void*) &play, sizeof(play)) == -1) {
			perror("Play effect");
			exit(1);
		}
	}

	first = 0;
}

void on_commands(IvyClientPtr app, void *user_data, int argc, char *argv[]){

  char* start = argv[0];
  char* stop;
  int x, y;

  strtol(start, &stop, 10);
  start=stop+1;
  x = -strtol(start, &stop, 10);
  start=stop+1;
  y = strtol(start, &stop, 10);

  if (x>PPRZ_MAX) x=PPRZ_MAX;
  if (x<PPRZ_MIN) x=PPRZ_MIN;
  if (y>PPRZ_MAX) y=PPRZ_MAX;
  if (y<PPRZ_MIN) y=PPRZ_MIN;

  generate_force((double)x/PPRZ_MAX, (double)y/PPRZ_MAX);
}

int main(int argc, char** argv)
{
	char dev_name[STR_LEN];
	int i;
	GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

	welcome();
	if (argc <= 1) return 0;

	/* Parse parameters */
	strcpy(dev_name, "/dev/input/event0");
	for (i=1; i<argc; ++i) {
		if (strcmp(argv[i], "--help") == 0) {
			printf("Usage: %s /dev/input/eventXX\n", argv[0]);
			exit(1);
		}
		else {
			strncpy(dev_name, argv[i], STR_LEN);
		}
	}

	/* Open force feedback device */
	ff_fd = open(dev_name, O_RDWR);
	if (ff_fd == -1) {
                perror("Open device file");
		exit(1);
	}

    IvyInit ("IvyExample", "IvyExample READY", NULL, NULL, NULL, NULL);
    IvyBindMsg(on_commands, NULL, "^\\S* COMMANDS (\\S*)");
    IvyStart("127.255.255.255");

    g_main_loop_run(ml);

  return 0;
}

